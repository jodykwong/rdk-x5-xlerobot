#!/usr/bin/env python3.10
# -*- coding: utf-8 -*-
"""
ASR桥接节点 - 将ASR系统集成到ROS2架构中

此节点作为现有ASRSystem和ROS2架构之间的桥梁，实现：
1. 包装现有ASRSystem类
2. 在独立线程运行ASR监听循环
3. 捕获ASR识别结果并转换为ROS2消息
4. 发布到/asr/result和/asr/status话题

作者: Claude Code
修复日期: 2025-11-15
目的: Epic 1 ASR→LLM→TTS串联修复
"""

import os
import sys
import time
import asyncio
import logging
import threading
from typing import Optional
from pathlib import Path

# ROS2相关导入
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Header, String
from audio_msg.msg import ASRResult, ASRStatus

# 添加项目路径到Python路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# 导入现有ASR系统
try:
    from modules.asr.asr_system import ASRSystem
except ImportError as e:
    print(f"❌ 导入ASRSystem失败: {e}")
    print("请确保PYTHONPATH设置正确")
    sys.exit(1)

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class ASRBridgeNode(Node):
    """ASR系统到ROS2的桥接节点"""

    def __init__(self):
        super().__init__('asr_bridge_node')

        # 设置QoS配置
        qos_profile = QoSProfile(depth=10)

        # 创建发布者 - 使用标准架构话题名称
        self.result_publisher = self.create_publisher(
            ASRResult, '/xlerobot/asr/result', qos_profile
        )
        # 兼容性String发布者（用于std_msgs通信）
        self.voice_command_string_publisher = self.create_publisher(
            String, '/xlerobot/asr/result_string', qos_profile
        )
        self.status_publisher = self.create_publisher(
            ASRStatus, '/xlerobot/asr/status', qos_profile
        )

        # 添加TTS播放请求发布者（用于转发ASR播放请求）
        self.tts_trigger_publisher = self.create_publisher(
            String, '/xlerobot/tts/trigger_play', qos_profile
        )

        # 初始化状态变量
        self.asr_system = None
        self.asr_thread = None
        self.is_initialized = False
        self.is_processing = False
        self.start_time = time.time()

        # 统计信息
        self.total_requests = 0
        self.successful_requests = 0
        self.failed_requests = 0

        # 启动定时器初始化ASR（延迟0.1秒，避免阻塞节点启动）
        self.timer = self.create_timer(0.1, self.initialize_asr_system)
        self.timer_count = 0

        self.get_logger().info('🔄 ASR桥接节点已创建，等待ASR系统初始化...')

        # 定期发布状态信息
        self.create_timer(5.0, self.publish_status)

    def initialize_asr_system(self):
        """延迟初始化ASRSystem（避免阻塞节点启动）"""
        # 只在第一次调用时执行
        self.timer_count += 1
        if self.timer_count > 1:
            # 第一次调用后销毁定时器
            self.timer.destroy()
            return
        try:
            self.get_logger().info('🚀 开始初始化ASR系统...')

            # 创建ASRSystem实例
            self.asr_system = ASRSystem()

            # 注入结果回调函数
            self.asr_system.result_callback = self.on_asr_result

            # 初始化ASR系统
            if self.asr_system.initialize():
                self.is_initialized = True
                self.get_logger().info('✅ ASR系统初始化成功')

                # 在独立线程启动ASR监听循环
                self.asr_thread = threading.Thread(
                    target=self._run_asr_loop,
                    daemon=True
                )
                self.asr_thread.start()

                self.get_logger().info('🎤 ASR监听线程已启动')

                # 发布初始化成功状态
                self.publish_status()

            else:
                self.get_logger().error('❌ ASR系统初始化失败')
                self.publish_status()

        except Exception as e:
            self.get_logger().error(f'❌ ASR系统初始化异常: {e}')
            self.publish_status()

    def _run_asr_loop(self):
        """在独立线程运行ASR监听循环"""
        try:
            self.get_logger().info('🎧 开始ASR监听循环...')

            # 直接运行ASR系统（不使用额外事件循环）
            success = self.asr_system.start()

            if success:
                self.get_logger().info('✅ ASR系统启动成功')

                # 等待ASR系统运行（阻塞直到停止）
                while self.asr_system.is_running:
                    time.sleep(0.1)
            else:
                self.get_logger().error('❌ ASR系统启动失败')

        except Exception as e:
            self.get_logger().error(f'❌ ASR监听循环异常: {e}')
        finally:
            self.get_logger().info('🛑 ASR监听循环已结束')

    def on_asr_result(self, result):
        """
        ASR结果回调 - 转换并发布ROS2消息

        Args:
            result: ASR识别结果对象，应包含text、confidence、success、error等属性
        """
        try:
            # 更新统计信息
            self.total_requests += 1
            if result.success:
                self.successful_requests += 1
            else:
                self.failed_requests += 1

            # 创建ROS2消息
            msg = ASRResult()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "asr_bridge"

            # 填充识别结果
            if hasattr(result, 'text'):
                msg.text = result.text or ""
            else:
                msg.text = str(result) if result else ""

            if hasattr(result, 'confidence'):
                msg.confidence = float(result.confidence)
            else:
                msg.confidence = 1.0  # 默认置信度

            # 设置时间信息
            current_time = int(time.time() * 1000)  # 毫秒
            msg.begin_time = current_time - 100  # 假设识别耗时100ms
            msg.end_time = current_time

            # 设置状态码和消息
            if result.success:
                msg.status_code = 0  # 成功
                msg.message = "识别成功"
            else:
                msg.status_code = -1  # 失败
                msg.message = result.error if hasattr(result, 'error') else "识别失败"

            # 发布消息
            self.result_publisher.publish(msg)

            # 同时发布String格式消息（兼容性）
            if result.success and msg.text.strip():
                string_msg = String()
                string_msg.data = msg.text
                self.voice_command_string_publisher.publish(string_msg)

            # 更新处理状态
            self.is_processing = False

            # 记录日志
            self.get_logger().info(f'📢 发布ASR结果: "{msg.text}" (置信度: {msg.confidence:.2f})')

            # 发布状态更新
            self.publish_status()

        except Exception as e:
            self.get_logger().error(f'❌ 处理ASR结果回调失败: {e}')
            self.failed_requests += 1

    def publish_status(self):
        """发布ASR状态信息"""
        try:
            msg = ASRStatus()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "asr_bridge"

            # 填充状态信息
            msg.is_initialized = self.is_initialized
            msg.is_processing = self.is_processing
            msg.language = "zh-CN"  # 默认中文
            msg.sample_rate = 16000  # 默认采样率
            msg.format = "pcm"

            # 填充统计信息
            msg.total_requests = self.total_requests
            msg.successful_requests = self.successful_requests
            msg.failed_requests = self.failed_requests

            # 计算运行时间
            msg.uptime = time.time() - self.start_time

            # 音频缓冲区大小（估算）
            msg.audio_buffer_size = 4096

            # 活跃目标数
            msg.active_goals = 1 if self.is_processing else 0

            # 兼容性字段 - 用于协调器状态监控
            if self.is_processing:
                msg.state = 1  # processing
            elif self.failed_requests > 0:
                msg.state = 2  # error
            else:
                msg.state = 0  # idle

            # 计算平均响应时间（实际测量）
            if self.successful_requests > 0:
                total_response_time = sum(self.response_times) if hasattr(self, 'response_times') else 0.0
                msg.avg_response_time = total_response_time / self.successful_requests
            else:
                msg.avg_response_time = 0.0

            # 最后错误信息（暂时为空，可以根据需要添加错误跟踪）
            msg.last_error = ""

            # 发布状态消息
            self.status_publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f'❌ 发布ASR状态失败: {e}')

    def on_shutdown(self):
        """节点关闭时的清理工作"""
        try:
            self.get_logger().info('🛑 正在关闭ASR桥接节点...')

            if self.asr_system:
                # 停止ASR系统
                if hasattr(self.asr_system, 'stop'):
                    self.asr_system.stop()

            # 等待线程结束
            if self.asr_thread and self.asr_thread.is_alive():
                self.asr_thread.join(timeout=2.0)

            self.get_logger().info('✅ ASR桥接节点已关闭')

        except Exception as e:
            self.get_logger().error(f'❌ 关闭ASR桥接节点时出错: {e}')


def main(args=None):
    """主函数"""
    try:
        # 初始化ROS2
        rclpy.init(args=args)

        # 创建ASR桥接节点
        node = ASRBridgeNode()

        # 设置信号处理
        import signal
        def signal_handler(sig, frame):
            node.on_shutdown()
            rclpy.shutdown()

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        # 运行节点
        rclpy.spin(node)

    except Exception as e:
        logger.error(f'❌ ASR桥接节点运行失败: {e}')
        import traceback
        traceback.print_exc()
    finally:
        # 清理资源
        try:
            if 'node' in locals():
                node.on_shutdown()
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()