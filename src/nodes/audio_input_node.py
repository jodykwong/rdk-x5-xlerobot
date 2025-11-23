#!/usr/bin/env python3
"""
音频输入ROS2节点

专门为XleRobot音频采集系统设计的ROS2节点：
- 实时音频数据采集
- 音频设备管理和配置
- 音频预处理和发布
- 唤醒词检测集成
- 音频状态监控

作者: Dev Agent
日期: 2025-11-08
Epic: 1 - ASR语音识别模块
Story: 1.1 - 粤语语音识别基础功能
Phase: 4 - ROS2节点集成
Task: 4.1 - Audio Input Node实现
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import std_msgs.msg
from audio_msg.msg import AudioData, AudioConfig, AudioStatus
from audio_msg.srv import SetAudioDevice, AudioConfigure

import numpy as np
import threading
import time
import json
from typing import Optional, Dict, Any
from dataclasses import dataclass
import logging

# 导入音频处理模块
from modules.asr.enhanced_audio_input import EnhancedAudioInput
from modules.asr.audio_preprocessor import AudioPreprocessor
from modules.asr.wake_word_detector import WakeWordDetector

logger = logging.getLogger(__name__)


@dataclass
class AudioNodeConfig:
    """音频节点配置"""
    device_id: Optional[str] = None
    sample_rate: int = 16000
    channels: int = 1
    chunk_size: int = 1024
    format: str = "S16_LE"
    enable_preprocessing: bool = True
    enable_wake_word: bool = True
    wake_word_threshold: float = 0.85
    publish_raw: bool = True
    publish_processed: bool = True
    buffer_size: int = 10


class AudioInputNode(Node):
    """
    音频输入ROS2节点

    提供实时音频采集、预处理和唤醒词检测功能
    """

    def __init__(self, node_name: str = "audio_input_node"):
        """
        初始化音频输入节点

        Args:
            node_name: 节点名称
        """
        super().__init__(node_name)

        # 节点配置
        self.config = AudioNodeConfig()
        self.is_running = False
        self.recording_thread = None

        # 音频组件
        self.audio_input: Optional[EnhancedAudioInput] = None
        self.preprocessor: Optional[AudioPreprocessor] = None
        self.wake_word_detector: Optional[WakeWordDetector] = None

        # 统计信息
        self.stats = {
            "total_frames": 0,
            "processed_frames": 0,
            "wake_word_detections": 0,
            "errors": 0,
            "start_time": time.time()
        }

        # QoS配置
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5
        )

        # 初始化组件
        self._initialize_components()

        # 创建发布者
        self._create_publishers()

        # 创建订阅者
        self._create_subscribers()

        # 创建服务
        self._create_services()

        # 启动音频采集
        self._start_audio_capture()

        self.get_logger().info("✅ AudioInputNode 初始化完成")
        self.get_logger().info(f"  采样率: {self.config.sample_rate}Hz")
        self.get_logger().info(f"  通道数: {self.config.channels}")
        self.get_logger().info(f"  块大小: {self.config.chunk_size}")

    def _initialize_components(self) -> None:
        """初始化音频组件"""
        try:
            # 初始化音频输入
            self.audio_input = EnhancedAudioInput()

            # 初始化预处理器
            if self.config.enable_preprocessing:
                self.preprocessor = AudioPreprocessor()

            # 初始化唤醒词检测器
            if self.config.enable_wake_word:
                self.wake_word_detector = WakeWordDetector()

            self.get_logger().info("音频组件初始化完成")

        except Exception as e:
            self.get_logger().error(f"音频组件初始化失败: {e}")
            raise

    def _create_publishers(self) -> None:
        """创建发布者"""
        # 原始音频数据发布者
        if self.config.publish_raw:
            self.raw_audio_publisher = self.create_publisher(
                AudioData,
                '/audio/raw',
                self.qos_profile
            )
            self.get_logger().info("创建 /audio/raw 发布者")

        # 处理后音频数据发布者
        if self.config.publish_processed:
            self.processed_audio_publisher = self.create_publisher(
                AudioData,
                '/audio/processed',
                self.qos_profile
            )
            self.get_logger().info("创建 /audio/processed 发布者")

        # 唤醒词检测结果发布者
        if self.config.enable_wake_word:
            self.wake_word_publisher = self.create_publisher(
                std_msgs.msg.Bool,
                '/audio/wake_word',
                self.qos_profile
            )
            self.get_logger().info("创建 /audio/wake_word 发布者")

        # 音频状态发布者
        self.status_publisher = self.create_publisher(
            AudioStatus,
            '/audio/status',
            QoSProfile(depth=1)
        )
        self.get_logger().info("创建 /audio/status 发布者")

    def _create_subscribers(self) -> None:
        """创建订阅者"""
        # 配置更新订阅者
        self.config_subscriber = self.create_subscription(
            AudioConfig,
            '/audio/config_update',
            self._config_callback,
            self.qos_profile
        )
        self.get_logger().info("创建 /audio/config_update 订阅者")

    def _create_services(self) -> None:
        """创建服务"""
        # 设置音频设备服务
        self.device_service = self.create_service(
            SetAudioDevice,
            '/audio/set_device',
            self._set_device_callback
        )
        self.get_logger().info("创建 /audio/set_device 服务")

        # 音频配置服务
        self.configure_service = self.create_service(
            AudioConfigure,
            '/audio/configure',
            self._configure_callback
        )
        self.get_logger().info("创建 /audio/configure 服务")

    def _start_audio_capture(self) -> None:
        """启动音频采集"""
        try:
            self.is_running = True
            self.recording_thread = threading.Thread(
                target=self._audio_recording_loop,
                daemon=True,
                name="AudioRecording"
            )
            self.recording_thread.start()

            self.get_logger().info("✅ 音频采集已启动")

        except Exception as e:
            self.get_logger().error(f"音频采集启动失败: {e}")
            self.is_running = False
            raise

    def _audio_recording_loop(self) -> None:
        """音频录制循环"""
        while self.is_running and rclpy.ok():
            try:
                # 采集音频数据
                audio_data = self.audio_input.read_audio_chunk()

                if audio_data is not None:
                    self._process_audio_data(audio_data)

                # 短暂休眠避免CPU占用过高
                time.sleep(0.001)

            except Exception as e:
                self.get_logger().error(f"音频录制循环异常: {e}")
                self.stats["errors"] += 1
                time.sleep(0.1)

    def _process_audio_data(self, audio_data: np.ndarray) -> None:
        """处理音频数据"""
        self.stats["total_frames"] += 1

        try:
            # 发布原始音频数据
            if self.config.publish_raw and hasattr(self, 'raw_audio_publisher'):
                self._publish_audio_data(audio_data, self.raw_audio_publisher, False)

            # 音频预处理
            processed_audio = audio_data
            if self.preprocessor:
                processed_audio = self.preprocessor.process_audio(audio_data)
                self.stats["processed_frames"] += 1

            # 发布处理后音频数据
            if self.config.publish_processed and hasattr(self, 'processed_audio_publisher'):
                self._publish_audio_data(processed_audio, self.processed_audio_publisher, True)

            # 唤醒词检测
            if self.wake_word_detector and processed_audio is not None:
                wake_detected = self.wake_word_detector.detect_wake_word(processed_audio)
                if wake_detected:
                    self.stats["wake_word_detections"] += 1
                    self._publish_wake_word_detection(True)

        except Exception as e:
            self.get_logger().error(f"音频数据处理失败: {e}")
            self.stats["errors"] += 1

    def _publish_audio_data(self, audio_data: np.ndarray, publisher, is_processed: bool) -> None:
        """发布音频数据"""
        try:
            # 创建AudioData消息
            msg = AudioData()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.sample_rate = self.config.sample_rate
            msg.channels = self.config.channels
            msg.format = self.config.format
            msg.is_processed = is_processed

            # 转换音频数据为字节
            if audio_data.dtype == np.float32:
                audio_bytes = (audio_data * 32767).astype(np.int16).tobytes()
            else:
                audio_bytes = audio_data.tobytes()

            msg.data = audio_bytes
            msg.frame_size = len(audio_data)

            # 发布消息
            publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"音频数据发布失败: {e}")

    def _publish_wake_word_detection(self, detected: bool) -> None:
        """发布唤醒词检测结果"""
        try:
            if hasattr(self, 'wake_word_publisher'):
                msg = std_msgs.msg.Bool()
                msg.data = detected
                self.wake_word_publisher.publish(msg)

                self.get_logger().info(f"🎯 唤醒词检测: {'触发' if detected else '无触发'}")

        except Exception as e:
            self.get_logger().error(f"唤醒词检测结果发布失败: {e}")

    def _publish_status(self) -> None:
        """发布音频状态"""
        try:
            msg = AudioStatus()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.is_recording = self.is_running
            msg.device_id = self.config.device_id or "default"
            msg.sample_rate = self.config.sample_rate
            msg.channels = self.config.channels
            msg.total_frames = self.stats["total_frames"]
            msg.processed_frames = self.stats["processed_frames"]
            msg.wake_word_detections = self.stats["wake_word_detections"]
            msg.errors = self.stats["errors"]
            msg.uptime = time.time() - self.stats["start_time"]

            self.status_publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"音频状态发布失败: {e}")

    def _config_callback(self, msg: AudioConfig) -> None:
        """配置更新回调"""
        try:
            self.get_logger().info("收到音频配置更新")

            # 更新配置
            if msg.sample_rate > 0:
                self.config.sample_rate = msg.sample_rate
            if msg.channels > 0:
                self.config.channels = msg.channels
            if msg.chunk_size > 0:
                self.config.chunk_size = msg.chunk_size

            self.get_logger().info(f"配置已更新: {self.config.sample_rate}Hz, {self.config.channels}ch")

        except Exception as e:
            self.get_logger().error(f"配置更新处理失败: {e}")

    def _set_device_callback(self, request: SetAudioDevice.Request, response: SetAudioDevice.Response) -> SetAudioDevice.Response:
        """设置音频设备服务回调"""
        try:
            self.get_logger().info(f"收到设备设置请求: {request.device_id}")

            # 停止当前录制
            was_running = self.is_running
            if self.is_running:
                self.is_running = False
                if self.recording_thread:
                    self.recording_thread.join(timeout=2.0)

            # 设置新设备
            self.config.device_id = request.device_id

            # 重新初始化音频输入
            if self.audio_input:
                self.audio_input.close()
            self.audio_input = EnhancedAudioInput(device_id=request.device_id)

            # 恢复录制
            if was_running:
                self._start_audio_capture()

            response.success = True
            response.message = f"设备已设置为: {request.device_id}"
            self.get_logger().info(f"✅ 设备设置成功: {request.device_id}")

        except Exception as e:
            response.success = False
            response.message = f"设备设置失败: {str(e)}"
            self.get_logger().error(f"设备设置失败: {e}")

        return response

    def _configure_callback(self, request: AudioConfigure.Request, response: AudioConfigure.Response) -> AudioConfigure.Response:
        """音频配置服务回调"""
        try:
            self.get_logger().info("收到音频配置请求")

            # 更新配置参数
            if request.sample_rate > 0:
                self.config.sample_rate = request.sample_rate
            if request.channels > 0:
                self.config.channels = request.channels
            if request.chunk_size > 0:
                self.config.chunk_size = request.chunk_size

            # 更新启用标志
            if request.enable_preprocessing is not None:
                self.config.enable_preprocessing = request.enable_preprocessing
            if request.enable_wake_word is not None:
                self.config.enable_wake_word = request.enable_wake_word

            response.success = True
            response.message = "配置更新成功"
            self.get_logger().info(f"✅ 音频配置已更新")

        except Exception as e:
            response.success = False
            response.message = f"配置更新失败: {str(e)}"
            self.get_logger().error(f"配置更新失败: {e}")

        return response

    def start_status_timer(self) -> None:
        """启动状态发布定时器"""
        self.status_timer = self.create_timer(
            1.0,  # 每秒发布一次状态
            self._publish_status
        )
        self.get_logger().info("状态发布定时器已启动")

    def stop_recording(self) -> None:
        """停止音频录制"""
        self.get_logger().info("停止音频录制")
        self.is_running = False

        if self.recording_thread and self.recording_thread.is_alive():
            self.recording_thread.join(timeout=3.0)

        if self.audio_input:
            self.audio_input.close()

        self.get_logger().info("✅ 音频录制已停止")

    def get_statistics(self) -> Dict[str, Any]:
        """获取统计信息"""
        return {
            **self.stats,
            "uptime": time.time() - self.stats["start_time"],
            "is_running": self.is_running,
            "config": {
                "sample_rate": self.config.sample_rate,
                "channels": self.config.channels,
                "chunk_size": self.config.chunk_size,
                "enable_preprocessing": self.config.enable_preprocessing,
                "enable_wake_word": self.config.enable_wake_word
            }
        }


def main(args=None):
    """主函数"""
    try:
        # 初始化ROS2
        rclpy.init(args=args)

        # 创建音频输入节点
        audio_node = AudioInputNode()

        # 启动状态定时器
        audio_node.start_status_timer()

        # 运行节点
        try:
            rclpy.spin(audio_node)
        except KeyboardInterrupt:
            audio_node.get_logger().info("收到中断信号")
        finally:
            # 清理资源
            audio_node.stop_recording()
            audio_node.destroy_node()
            rclpy.shutdown()

    except Exception as e:
        logging.error(f"音频输入节点运行失败: {e}")
        raise


if __name__ == "__main__":
    main()