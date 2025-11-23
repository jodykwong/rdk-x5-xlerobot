#!/usr/bin/env python3
"""
ASR服务ROS2节点

专门为XleRobot语音识别服务设计的ROS2节点：
- 实时语音识别服务
- 阿里云ASR集成
- 连续语音识别动作接口
- 识别结果发布
- ASR状态监控

作者: Dev Agent
日期: 2025-11-08
Epic: 1 - ASR语音识别模块
Story: 1.1 - 粤语语音识别基础功能
Phase: 4 - ROS2节点集成
Task: 4.2 - ASR Service Node实现
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.action import ActionServer, GoalResponse, CancelResponse, ActionResult
import std_msgs.msg
from audio_msg.msg import AudioData, ASRResult, ASRStatus
from audio_msg.srv import ASRConfigure
from audio_msg.action import ContinuousRecognition

import numpy as np
import threading
import time
import json
from typing import Optional, Dict, Any, List
from dataclasses import dataclass
from queue import Queue, Empty
import logging

# 导入ASR模块
from modules.asr.aliyun_asr_service import AliyunASRService, ASRConfig, ASRLanguage, ASRFormat
from modules.asr.audio_processor_asr import ASRAudioProcessor
from modules.asr.asr_retry_manager import ASRRetryManager, RetryStrategy, FallbackAction

logger = logging.getLogger(__name__)


@dataclass
class ASRNodeConfig:
    """ASR节点配置"""
    app_key: str = ""
    app_secret: str = ""
    language: str = "cantonese"
    format: str = "pcm"
    sample_rate: int = 16000
    enable_continuous: bool = True
    max_audio_buffer: int = 100
    result_timeout: float = 5.0
    enable_retry: bool = True
    max_retries: int = 3
    publish_intermediate: bool = True


class ASRServiceNode(Node):
    """
    ASR服务ROS2节点

    提供语音识别服务和连续识别动作接口
    """

    def __init__(self, node_name: str = "asr_service_node"):
        """
        初始化ASR服务节点

        Args:
            node_name: 节点名称
        """
        super().__init__(node_name)

        # 节点配置
        self.config = ASRNodeConfig()
        self.is_initialized = False
        self.is_processing = False

        # ASR组件
        self.asr_service: Optional[AliyunASRService] = None
        self.audio_processor: Optional[ASRAudioProcessor] = None
        self.retry_manager: Optional[ASRRetryManager] = None

        # 音频缓冲区
        self.audio_buffer = Queue(maxsize=self.config.max_audio_buffer)
        self.result_buffer = Queue(maxsize=10)

        # 连续识别状态
        self.recognition_goals = {}  # goal_id -> goal_info
        self.active_goal_id = None

        # 统计信息
        self.stats = {
            "total_requests": 0,
            "successful_requests": 0,
            "failed_requests": 0,
            "total_recognition_time": 0.0,
            "start_time": time.time()
        }

        # QoS配置
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        # 初始化组件
        self._initialize_components()

        # 创建发布者
        self._create_publishers()

        # 创建订阅者
        self._create_subscribers()

        # 创建服务
        self._create_services()

        # 创建动作服务器
        self._create_action_servers()

        self.is_initialized = True
        self.get_logger().info("✅ ASRServiceNode 初始化完成")
        self.get_logger().info(f"  语言: {self.config.language}")
        self.get_logger().info(f"  采样率: {self.config.sample_rate}Hz")
        self.get_logger().info(f"  连续识别: {self.config.enable_continuous}")

    def _initialize_components(self) -> None:
        """初始化ASR组件"""
        try:
            # 从环境变量获取配置
            import os
            app_key = os.getenv("ALIYUN_NLS_APP_KEY", "")
            app_secret = os.getenv("ALIYUN_NLS_APP_SECRET", "")

            if not app_key or not app_secret:
                self.get_logger().warning("阿里云ASR配置缺失，使用模拟模式")
                app_key = "mock_key"
                app_secret = "mock_secret"

            self.config.app_key = app_key
            self.config.app_secret = app_secret

            # 创建ASR配置
            asr_config = ASRConfig(
                app_key=app_key,
                app_secret=app_secret,
                language=ASRLanguage.CANTONESE if self.config.language == "cantonese" else ASRLanguage.MANDARIN,
                format=ASRFormat.PCM if self.config.format == "pcm" else ASRFormat.WAV,
                sample_rate=self.config.sample_rate
            )

            # 初始化ASR服务
            self.asr_service = AliyunASRService(asr_config)

            # 初始化音频处理器
            self.audio_processor = ASRAudioProcessor()

            # 初始化重试管理器
            if self.config.enable_retry:
                self.retry_manager = ASRRetryManager(
                    max_retries=self.config.max_retries,
                    strategy=RetryStrategy.EXPONENTIAL_BACKOFF
                )

            self.get_logger().info("ASR组件初始化完成")

        except Exception as e:
            self.get_logger().error(f"ASR组件初始化失败: {e}")
            raise

    def _create_publishers(self) -> None:
        """创建发布者"""
        # ASR结果发布者
        self.result_publisher = self.create_publisher(
            ASRResult,
            '/asr/result',
            self.qos_profile
        )
        self.get_logger().info("创建 /asr/result 发布者")

        # ASR状态发布者
        self.status_publisher = self.create_publisher(
            ASRStatus,
            '/asr/status',
            QoSProfile(depth=1)
        )
        self.get_logger().info("创建 /asr/status 发布者")

        # 中间结果发布者
        if self.config.publish_intermediate:
            self.intermediate_publisher = self.create_publisher(
                ASRResult,
                '/asr/intermediate_result',
                self.qos_profile
            )
            self.get_logger().info("创建 /asr/intermediate_result 发布者")

    def _create_subscribers(self) -> None:
        """创建订阅者"""
        # 处理后音频数据订阅者
        self.audio_subscriber = self.create_subscription(
            AudioData,
            '/audio/processed',
            self._audio_callback,
            self.qos_profile
        )
        self.get_logger().info("创建 /audio/processed 订阅者")

        # 原始音频数据订阅者（备用）
        self.raw_audio_subscriber = self.create_subscription(
            AudioData,
            '/audio/raw',
            self._raw_audio_callback,
            self.qos_profile
        )
        self.get_logger().info("创建 /audio/raw 订阅者")

        # 唤醒词检测订阅者
        self.wake_word_subscriber = self.create_subscription(
            std_msgs.msg.Bool,
            '/audio/wake_word',
            self._wake_word_callback,
            QoSProfile(depth=5)
        )
        self.get_logger().info("创建 /audio/wake_word 订阅者")

    def _create_services(self) -> None:
        """创建服务"""
        # ASR配置服务
        self.configure_service = self.create_service(
            ASRConfigure,
            '/asr/configure',
            self._configure_callback
        )
        self.get_logger().info("创建 /asr/configure 服务")

    def _create_action_servers(self) -> None:
        """创建动作服务器"""
        # 连续语音识别动作
        self.continuous_recognition_server = ActionServer(
            self,
            ContinuousRecognition,
            '/asr/continuous_recognition',
            self._continuous_recognition_execute,
            goal_callback=self._continuous_recognition_goal_callback,
            cancel_callback=self._continuous_recognition_cancel_callback,
            handle_accepted_goal=self._handle_continuous_goal
        )
        self.get_logger().info("创建 /asr/continuous_recognition 动作服务器")

    def _audio_callback(self, msg: AudioData) -> None:
        """处理音频数据回调"""
        try:
            if not self.is_initialized:
                return

            # 转换音频数据
            audio_data = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32767.0

            # 添加到缓冲区
            if not self.audio_buffer.full():
                self.audio_buffer.put((audio_data, msg.sample_rate))
            else:
                # 缓冲区满时丢弃最旧的数据
                try:
                    self.audio_buffer.get_nowait()
                    self.audio_buffer.put((audio_data, msg.sample_rate))
                except Empty:
                    pass

        except Exception as e:
            self.get_logger().error(f"音频数据处理失败: {e}")

    def _raw_audio_callback(self, msg: AudioData) -> None:
        """原始音频数据回调（备用）"""
        try:
            # 只有在没有处理音频时才使用原始音频
            if not self.audio_buffer.empty():
                return

            audio_data = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32767.0

            if not self.audio_buffer.full():
                self.audio_buffer.put((audio_data, msg.sample_rate))

        except Exception as e:
            self.get_logger().error(f"原始音频数据处理失败: {e}")

    def _wake_word_callback(self, msg: std_msgs.msg.Bool) -> None:
        """唤醒词检测回调"""
        try:
            if msg.data and self.active_goal_id:
                # 唤醒词被检测到，处理当前音频
                self.get_logger().info("🎯 检测到唤醒词，开始语音识别")

        except Exception as e:
            self.get_logger().error(f"唤醒词回调处理失败: {e}")

    def _configure_callback(self, request: ASRConfigure.Request, response: ASRConfigure.Response) -> ASRConfigure.Response:
        """ASR配置服务回调"""
        try:
            self.get_logger().info("收到ASR配置请求")

            # 更新配置
            if request.language:
                self.config.language = request.language
            if request.sample_rate > 0:
                self.config.sample_rate = request.sample_rate
            if request.format:
                self.config.format = request.format

            response.success = True
            response.message = "ASR配置更新成功"
            self.get_logger().info(f"✅ ASR配置已更新: {self.config.language}")

        except Exception as e:
            response.success = False
            response.message = f"ASR配置更新失败: {str(e)}"
            self.get_logger().error(f"ASR配置更新失败: {e}")

        return response

    def _continuous_recognition_goal_callback(self, goal_request) -> GoalResponse:
        """连续识别目标回调"""
        self.get_logger().info("收到连续识别目标请求")

        # 检查是否有其他活跃目标
        if self.active_goal_id:
            self.get_logger().warning("已有活跃的连续识别目标")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _continuous_recognition_cancel_callback(self, goal_handle) -> CancelResponse:
        """连续识别取消回调"""
        self.get_logger().info("收到连续识别取消请求")

        if goal_handle.goal_id == self.active_goal_id:
            return CancelResponse.ACCEPT
        else:
            return CancelResponse.REJECT

    def _handle_continuous_goal(self, goal_handle) -> None:
        """处理连续识别目标"""
        goal_id = goal_handle.goal_id
        goal = goal_handle.request

        self.get_logger().info(f"开始处理连续识别目标: {goal_id}")

        # 存储目标信息
        self.recognition_goals[goal_id] = {
            "handle": goal_handle,
            "start_time": time.time(),
            "timeout": goal.timeout if goal.timeout > 0 else 30.0
        }
        self.active_goal_id = goal_id

        # 启动识别线程
        recognition_thread = threading.Thread(
            target=self._continuous_recognition_thread,
            args=(goal_id,),
            daemon=True,
            name=f"ContinuousRecognition-{goal_id}"
        )
        recognition_thread.start()

    def _continuous_recognition_execute(self, goal_handle) -> ActionResult:
        """连续识别动作执行"""
        # 这个方法主要为了满足接口要求，实际处理在_handle_continuous_goal中
        return ContinuousRecognition.Result()

    def _continuous_recognition_thread(self, goal_id: str) -> None:
        """连续识别线程"""
        try:
            goal_info = self.recognition_goals.get(goal_id)
            if not goal_info:
                return

            goal_handle = goal_info["handle"]
            start_time = goal_info["start_time"]
            timeout = goal_info["timeout"]

            self.get_logger().info(f"开始连续识别 (目标: {goal_id})")

            # 启动ASR会话
            if not self._start_asr_session():
                goal_handle.abort()
                return

            # 识别循环
            while rclpy.ok() and goal_id in self.recognition_goals:
                try:
                    # 检查超时
                    if time.time() - start_time > timeout:
                        self.get_logger().info(f"连续识别超时 (目标: {goal_id})")
                        break

                    # 获取音频数据
                    try:
                        audio_data, sample_rate = self.audio_buffer.get(timeout=1.0)
                    except Empty:
                        continue

                    # 执行语音识别
                    result = self._recognize_audio(audio_data, sample_rate)

                    if result:
                        # 发布识别结果
                        self._publish_asr_result(result)

                        # 发送中间结果反馈
                        feedback = ContinuousRecognition.Feedback()
                        feedback.intermediate_text = result.text
                        feedback.confidence = result.confidence
                        goal_handle.publish_feedback(feedback)

                        self.get_logger().info(f"识别结果: {result.text} (置信度: {result.confidence:.2f})")

                except Exception as e:
                    self.get_logger().error(f"连续识别处理异常: {e}")
                    time.sleep(0.1)

            # 停止ASR会话
            self._stop_asr_session()

            # 完成目标
            if goal_id in self.recognition_goals:
                result = ContinuousRecognition.Result()
                result.success = True
                result.message = "连续识别完成"
                goal_handle.succeed(result)

            self.get_logger().info(f"连续识别完成 (目标: {goal_id})")

        except Exception as e:
            self.get_logger().error(f"连续识别线程异常: {e}")
            if goal_id in self.recognition_goals:
                goal_handle = self.recognition_goals[goal_id]["handle"]
                result = ContinuousRecognition.Result()
                result.success = False
                result.message = f"识别失败: {str(e)}"
                goal_handle.abort(result)

        finally:
            # 清理目标信息
            if goal_id == self.active_goal_id:
                self.active_goal_id = None
            if goal_id in self.recognition_goals:
                del self.recognition_goals[goal_id]

    def _start_asr_session(self) -> bool:
        """启动ASR会话"""
        try:
            if self.retry_manager:
                result = self.retry_manager.execute_with_retry(
                    self.asr_service.start_recognition_session,
                    fallback_action=FallbackAction.RETURN_DEFAULT,
                    fallback_result=None
                )
                return result.success and result.data is not None
            else:
                session_id = self.asr_service.start_recognition_session()
                return session_id is not None

        except Exception as e:
            self.get_logger().error(f"ASR会话启动失败: {e}")
            return False

    def _stop_asr_session(self) -> None:
        """停止ASR会话"""
        try:
            if self.asr_service:
                self.asr_service.stop_recognition_session()

        except Exception as e:
            self.get_logger().error(f"ASR会话停止失败: {e}")

    def _recognize_audio(self, audio_data: np.ndarray, sample_rate: int) -> Optional[Any]:
        """识别音频数据"""
        try:
            if self.retry_manager:
                result = self.retry_manager.execute_with_retry(
                    self.asr_service.recognize_audio,
                    audio_data,
                    sample_rate,
                    fallback_action=FallbackAction.RETURN_DEFAULT,
                    fallback_result=None
                )
                return result.data if result.success else None
            else:
                return self.asr_service.recognize_audio(audio_data, sample_rate)

        except Exception as e:
            self.get_logger().error(f"语音识别失败: {e}")
            return None

    def _publish_asr_result(self, result) -> None:
        """发布ASR结果"""
        try:
            # 创建ASRResult消息
            msg = ASRResult()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.text = result.text
            msg.confidence = result.confidence
            msg.begin_time = result.begin_time
            msg.end_time = result.end_time
            msg.status_code = result.status_code
            msg.message = result.message

            # 发布结果
            self.result_publisher.publish(msg)

            # 更新统计信息
            self.stats["total_requests"] += 1
            if result.status_code == 200:
                self.stats["successful_requests"] += 1
            else:
                self.stats["failed_requests"] += 1

        except Exception as e:
            self.get_logger().error(f"ASR结果发布失败: {e}")

    def start_status_timer(self) -> None:
        """启动状态发布定时器"""
        self.status_timer = self.create_timer(
            2.0,  # 每2秒发布一次状态
            self._publish_status
        )
        self.get_logger().info("状态发布定时器已启动")

    def _publish_status(self) -> None:
        """发布ASR状态"""
        try:
            msg = ASRStatus()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.is_initialized = self.is_initialized
            msg.is_processing = self.is_processing
            msg.language = self.config.language
            msg.sample_rate = self.config.sample_rate
            msg.format = self.config.format
            msg.total_requests = self.stats["total_requests"]
            msg.successful_requests = self.stats["successful_requests"]
            msg.failed_requests = self.stats["failed_requests"]
            msg.uptime = time.time() - self.stats["start_time"]
            msg.audio_buffer_size = self.audio_buffer.qsize()
            msg.active_goals = len(self.recognition_goals)

            self.status_publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"ASR状态发布失败: {e}")

    def get_statistics(self) -> Dict[str, Any]:
        """获取统计信息"""
        success_rate = (self.stats["successful_requests"] / self.stats["total_requests"]
                       if self.stats["total_requests"] > 0 else 0.0)

        return {
            **self.stats,
            "success_rate": success_rate,
            "uptime": time.time() - self.stats["start_time"],
            "is_initialized": self.is_initialized,
            "is_processing": self.is_processing,
            "active_goals": len(self.recognition_goals),
            "config": {
                "language": self.config.language,
                "sample_rate": self.config.sample_rate,
                "format": self.config.format,
                "enable_continuous": self.config.enable_continuous
            }
        }


def main(args=None):
    """主函数"""
    try:
        # 初始化ROS2
        rclpy.init(args=args)

        # 创建ASR服务节点
        asr_node = ASRServiceNode()

        # 启动状态定时器
        asr_node.start_status_timer()

        # 运行节点
        try:
            rclpy.spin(asr_node)
        except KeyboardInterrupt:
            asr_node.get_logger().info("收到中断信号")
        finally:
            # 清理资源
            asr_node.destroy_node()
            rclpy.shutdown()

    except Exception as e:
        logging.error(f"ASR服务节点运行失败: {e}")
        raise


if __name__ == "__main__":
    main()