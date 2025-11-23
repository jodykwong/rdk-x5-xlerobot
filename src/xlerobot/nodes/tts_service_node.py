#!/usr/bin/env python3.10
# -*- coding: utf-8 -*-
"""
TTS服务节点 - ROS2节点实现

负责接收LLM响应，合成语音，并播放音频。
支持队列管理，避免音频播放重叠，实现完整的错误处理机制。

作者: Claude Code
故事ID: Epic 1 ASR→LLM→TTS串联修复
"""

import os
import sys
import time
import asyncio
import logging
import traceback
import queue  # 标准库队列，线程安全，不需要事件循环
from typing import Optional, Dict, Any, List
from dataclasses import dataclass
from pathlib import Path

# ROS2相关导入
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Header, String
from audio_msg.msg import LLMResponse, TTSStatus

# 添加项目路径到Python路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# 导入现有TTS模块
try:
    from modules.tts.simple_tts_service import SimpleTTSService
    from modules.tts.aliyun_tts_system import AliyunTTSSystem
except ImportError as e:
    print(f"❌ 导入TTS模块失败: {e}")
    print("请确保PYTHONPATH设置正确")
    sys.exit(1)


# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


@dataclass
class TTSNodeConfig:
    """TTS节点配置"""
    voice: str = "jiajia"  # 默认粤语音色（与阿里云控制台一致）
    output_dir: str = "/tmp/xlerobot_tts"
    max_queue_length: int = 5
    audio_device: str = "default"
    sample_rate: int = 16000
    format: str = "wav"


class AudioPlayTask:
    """音频播放任务"""
    def __init__(self, audio_file: str, session_id: str, text: str):
        self.audio_file = audio_file
        self.session_id = session_id
        self.text = text
        self.created_at = time.time()
        self.status = "pending"  # pending, playing, completed, failed


class TTSServiceNode(Node):
    """TTS服务节点"""

    def __init__(self):
        super().__init__('tts_service_node')

        # 配置
        self.config = TTSNodeConfig(
            voice=os.getenv('TTS_VOICE', 'xiaoyun'),
            output_dir=os.getenv('TTS_OUTPUT_DIR', '/tmp/xlerobot_tts')
        )

        # 状态管理
        self.current_state = 0  # 0=idle, 1=synthesizing, 2=playing, 3=error
        self.is_playing = False

        # 性能统计
        self.total_syntheses = 0
        self.total_playbacks = 0
        self.synthesis_times = []
        self.playback_times = []

        # 播放队列 - 使用标准库queue.Queue，避免asyncio事件循环问题
        self.play_queue: queue.Queue = queue.Queue(maxsize=self.config.max_queue_length)
        self.current_task: Optional[AudioPlayTask] = None

        # 创建输出目录
        Path(self.config.output_dir).mkdir(parents=True, exist_ok=True)

        # 初始化TTS服务
        try:
            self.tts_service = SimpleTTSService()
            # 可选：初始化阿里云TTS系统
            try:
                self.aliyun_tts = AliyunTTSSystem()
                self.get_logger().info("✅ 阿里云TTS系统初始化成功")
            except Exception as e:
                self.get_logger().warning(f"⚠️ 阿里云TTS初始化失败，使用备用TTS: {e}")
                self.aliyun_tts = None

            self.get_logger().info("✅ TTS服务初始化成功")
        except Exception as e:
            self.get_logger().error(f"❌ TTS服务初始化失败: {e}")
            self.current_state = 3  # error
            return

        # 创建订阅者 - 订阅TTS请求
        qos = QoSProfile(depth=10)
        self.tts_request_subscription = self.create_subscription(
            LLMResponse,
            '/xlerobot/llm/response',
            self.tts_request_callback,
            qos
        )

        # 添加ASR播放请求订阅者（用于处理ASR系统的播放请求）
        self.trigger_play_subscription = self.create_subscription(
            String,
            '/xlerobot/tts/trigger_play',
            self.trigger_play_callback,
            qos
        )

        # 创建状态发布者
        self.status_publisher = self.create_publisher(
            TTSStatus,
            '/xlerobot/tts/status',
            qos
        )

        # 创建状态定时器
        self.status_timer = self.create_timer(
            1.0,  # 每秒发布一次状态
            self.publish_status
        )

        # 播放工作线程将在第一个异步调用时启动
        self.playback_task = None
        self._playback_worker_started = False

        self.get_logger().info("🚀 TTS服务节点启动完成")

    def __del__(self):
        """析构函数 - 清理线程池执行器"""
        try:
            if hasattr(self, '_tts_executor'):
                self._tts_executor.shutdown(wait=False)
                self.get_logger().info("🧹 TTS线程池执行器已清理")
        except Exception as e:
            pass  # 忽略清理时的错误

    def tts_request_callback(self, msg: LLMResponse):
        """处理TTS请求"""
        self.get_logger().info(f"🤖 收到TTS请求: {msg.text[:50]}... (会话: {msg.session_id})")

        # 检查LLM响应状态
        if msg.status_code != 0:  # 非成功状态
            self.get_logger().warning(f"⚠️ LLM响应状态异常: {msg.status_code} - {msg.error_message}")
            return

        if not msg.text.strip():
            self.get_logger().warning("⚠️ LLM响应文本为空，跳过处理")
            return

        # 异步处理TTS请求 - 使用线程池执行器避免事件循环问题
        import concurrent.futures
        try:
            # 创建线程池执行器
            if not hasattr(self, '_tts_executor'):
                self._tts_executor = concurrent.futures.ThreadPoolExecutor(max_workers=2, thread_name_prefix="tts_worker")

            # 在线程中运行异步任务
            self._tts_executor.submit(self._run_async_task, self.process_tts_request(msg))
        except Exception as e:
            self.get_logger().error(f"❌ TTS任务启动失败: {e}")

    def trigger_play_callback(self, msg: String):
        """处理来自ASR的播放请求"""
        try:
            play_text = msg.data.strip()
            if not play_text:
                self.get_logger().warning("⚠️ 收到空的播放请求，忽略")
                return

            self.get_logger().info(f"🔊 收到ASR播放请求: {play_text}")

            # 创建优先播放任务 - 先生成TTS音频文件
            try:
                # 使用备用TTS服务快速生成音频
                if hasattr(self, 'simple_tts') and self.simple_tts:
                    audio_file = self.simple_tts.synthesize_speech(play_text, "jiajia")
                    if audio_file:
                        priority_play_task = AudioPlayTask(
                            audio_file=audio_file,
                            session_id="asr_trigger",
                            text=play_text
                        )
                    else:
                        self.get_logger().error("❌ ASR播放音频生成失败")
                        return
                else:
                    self.get_logger().error("❌ 没有可用的TTS服务")
                    return

            except Exception as e:
                self.get_logger().error(f"❌ ASR播放音频生成异常: {e}")
                return

            # 直接添加到播放队列（高优先级）
            try:
                self.play_queue.put_nowait(priority_play_task)
                self._start_playback_worker_if_needed()
                self.get_logger().info(f"✅ ASR播放请求已加入队列")
            except queue.Full:
                self.get_logger().warning("⚠️ 播放队列已满，跳过ASR播放请求")

        except Exception as e:
            self.get_logger().error(f"❌ 处理ASR播放请求失败: {e}")

    def _start_playback_worker_if_needed(self):
        """启动播放工作线程（如果尚未启动）"""
        if not self._playback_worker_started:
            self._playback_worker_started = True
            # 使用线程池执行器启动播放工作线程
            if hasattr(self, '_tts_executor'):
                self._tts_executor.submit(self._run_async_task, self.playback_worker())
            else:
                self.get_logger().warning("⚠️ TTS执行器未初始化，无法启动播放工作线程")

    def _run_async_task(self, coro):
        """在线程中运行异步任务的辅助方法"""
        import asyncio
        import threading

        def run_in_thread():
            try:
                # 创建新的事件循环
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)

                # 运行异步任务
                return loop.run_until_complete(coro)
            except Exception as e:
                self.get_logger().error(f"❌ 异步任务执行失败: {e}")
                return None
            finally:
                try:
                    loop.close()
                except:
                    pass

        # 在当前线程中运行（因为我们已经在线程池中）
        return run_in_thread()

    async def process_tts_request(self, llm_msg: LLMResponse):
        """异步处理TTS请求"""
        start_time = time.time()

        try:
            # 更新状态
            self.current_state = 1  # synthesizing
            self.total_syntheses += 1

            # 生成音频文件名
            timestamp = int(time.time() * 1000)
            audio_file = os.path.join(
                self.config.output_dir,
                f"tts_{timestamp}_{hash(llm_msg.text) % 10000}.wav"
            )

            # 合成语音
            await self.synthesize_speech(
                text=llm_msg.text,
                voice=self.config.voice,
                output_file=audio_file
            )

            # 计算合成时间
            synthesis_time = time.time() - start_time
            self.synthesis_times.append(synthesis_time)

            # 添加到播放队列
            play_task = AudioPlayTask(
                audio_file=audio_file,
                session_id=llm_msg.session_id,
                text=llm_msg.text
            )

            try:
                self.play_queue.put_nowait(play_task)
                self.get_logger().info(f"✅ TTS合成完成，加入播放队列: {os.path.basename(audio_file)}")
                self.current_state = 0  # idle

                # 启动播放工作线程（如果尚未启动）
                self._start_playback_worker_if_needed()
            except queue.Full:
                self.get_logger().warning("⚠️ 播放队列已满，丢弃音频")
                # 清理生成的音频文件
                if os.path.exists(audio_file):
                    os.remove(audio_file)

        except Exception as e:
            self.get_logger().error(f"❌ TTS处理失败: {e}")
            self.get_logger().error(f"详细错误: {traceback.format_exc()}")
            self.current_state = 3  # error

    async def synthesize_speech(self, text: str, voice: str, output_file: str):
        """语音合成"""
        try:
            # 优先使用阿里云TTS
            if self.aliyun_tts:
                await self.aliyun_tts.synthesize_async(text, output_file)
            else:
                # 使用备用TTS服务
                audio_path = await self.tts_service.synthesize_speech(text, voice)
                # 如果返回的是相对路径，转换为绝对路径
                if not os.path.isabs(audio_path):
                    audio_path = os.path.join(os.getcwd(), audio_path)
                # 移动文件到目标位置
                if audio_path != output_file:
                    os.rename(audio_path, output_file)

        except Exception as e:
            self.get_logger().error(f"❌ 语音合成失败: {e}")
            # 降级处理：生成静音音频文件
            await self.generate_silent_audio(output_file)

    async def generate_silent_audio(self, output_file: str):
        """生成静音音频文件作为降级方案"""
        try:
            # 使用ffmpeg生成静音音频（1秒）
            duration = 1.0
            cmd = [
                'ffmpeg', '-y',  # -y覆盖输出文件
                '-f', 'lavfi',
                '-i', f'anullsrc=channel_layout=mono:sample_rate={self.config.sample_rate}',
                '-t', str(duration),
                output_file
            ]

            process = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )

            stdout, stderr = await process.communicate()

            if process.returncode != 0:
                self.get_logger().error(f"❌ 静音音频生成失败: {stderr.decode()}")
            else:
                self.get_logger().info("✅ 生成静音音频文件")

        except Exception as e:
            self.get_logger().error(f"❌ 静音音频生成异常: {e}")

    async def playback_worker(self):
        """播放工作线程"""
        while True:
            try:
                # 等待播放任务 - 使用同步queue.Queue的get方法
                try:
                    play_task = self.play_queue.get(timeout=1.0)
                except queue.Empty:
                    # 队列为空，继续等待
                    continue
                self.current_task = play_task

                # 更新状态
                self.current_state = 2  # playing
                self.is_playing = True
                play_task.status = "playing"

                # 播放音频
                await self.play_audio(play_task.audio_file)

                # 更新任务状态
                play_task.status = "completed"
                self.current_state = 0  # idle
                self.is_playing = False

                # 计算播放时间
                playback_time = time.time() - play_task.created_at
                self.playback_times.append(playback_time)
                self.total_playbacks += 1

                self.get_logger().info(f"✅ 音频播放完成: {os.path.basename(play_task.audio_file)}")

                # 清理音频文件
                try:
                    if os.path.exists(play_task.audio_file):
                        os.remove(play_task.audio_file)
                except Exception as e:
                    self.get_logger().warning(f"⚠️ 清理音频文件失败: {e}")

            except Exception as e:
                self.get_logger().error(f"❌ 播放工作线程异常: {e}")
                if self.current_task:
                    self.current_task.status = "failed"
                self.current_state = 3  # error
                self.is_playing = False

            finally:
                self.current_task = None
                self.play_queue.task_done()

    async def play_audio(self, audio_file: str):
        """播放音频文件"""
        try:
            if not os.path.exists(audio_file):
                raise FileNotFoundError(f"音频文件不存在: {audio_file}")

            # 使用aplay播放音频
            cmd = [
                'aplay',
                '-D', self.config.audio_device,
                '-q',  # 安静模式
                audio_file
            ]

            process = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )

            stdout, stderr = await process.communicate()

            if process.returncode != 0:
                self.get_logger().error(f"❌ 音频播放失败: {stderr.decode()}")
                raise RuntimeError(f"aplay失败，返回码: {process.returncode}")

        except Exception as e:
            self.get_logger().error(f"❌ 播放音频异常: {e}")
            raise

    def publish_status(self):
        """发布节点状态"""
        msg = TTSStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.node_name = self.get_name()
        msg.state = self.current_state
        msg.queue_length = self.play_queue.qsize()
        msg.avg_synthesis_time = sum(self.synthesis_times) / len(self.synthesis_times) if self.synthesis_times else 0.0
        msg.avg_playback_time = sum(self.playback_times) / len(self.playback_times) if self.playback_times else 0.0
        msg.total_syntheses = self.total_syntheses
        msg.total_playbacks = self.total_playbacks
        msg.last_error = ""

        self.status_publisher.publish(msg)


def main(args=None):
    """主函数"""
    try:
        # 检查音频设备
        try:
            import subprocess
            result = subprocess.run(['aplay', '-l'], capture_output=True, text=True)
            if result.returncode != 0:
                print("❌ 错误: 无法访问音频设备，请检查ALSA配置")
                sys.exit(1)
        except FileNotFoundError:
            print("❌ 错误: aplay命令不可用，请安装alsa-utils")
            sys.exit(1)

        # 初始化ROS2
        rclpy.init(args=args)

        # 创建TTS服务节点
        node = TTSServiceNode()

        # 运行节点
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("🛑 收到中断信号，正在关闭节点...")
        finally:
            # 清理资源
            if hasattr(node, 'playback_task'):
                node.playback_task.cancel()

            # 清理播放队列
            while not node.play_queue.empty():
                try:
                    task = node.play_queue.get_nowait()
                    if task.audio_file and os.path.exists(task.audio_file):
                        os.remove(task.audio_file)
                except asyncio.QueueEmpty:
                    break

            node.destroy_node()
            rclpy.shutdown()

    except Exception as e:
        print(f"❌ 节点启动失败: {e}")
        print(f"详细错误: {traceback.format_exc()}")
        sys.exit(1)


if __name__ == '__main__':
    main()