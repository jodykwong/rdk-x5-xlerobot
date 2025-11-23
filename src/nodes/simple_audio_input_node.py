#!/usr/bin/env python3
"""
简单音频输入节点 - Pure Online Architecture
==============================================

简化的ROS2音频输入节点，专为纯在线语音服务设计。
提供基础的音频录制和话题发布功能。

功能：
- ALSA音频录制
- /audio/raw话题发布
- 基础设备配置
- 简单的服务接口

作者: Developer Agent
版本: 1.0 (纯在线架构)
日期: 2025-11-09
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import Header
from audio_msg.msg import AudioFrame
from std_srvs.srv import SetBool
import numpy as np
import threading
import time
import logging

# 导入音频录制器
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'modules', 'asr'))
from simple_audio_recorder import SimpleAudioRecorder

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class SimpleAudioInputNode(Node):
    """
    简单音频输入节点

    专为纯在线服务设计，提供基础音频录制功能。
    严格遵循简化原则，避免复杂配置。
    """

    def __init__(self):
        """初始化音频输入节点"""
        super().__init__('simple_audio_input_node')

        # 音频录制器
        self.recorder = SimpleAudioRecorder()
        self.is_recording = False
        self.recording_thread = None

        # QoS配置
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5
        )

        # 发布者
        self.audio_publisher = self.create_publisher(
            AudioFrame,
            '/audio/raw',
            qos_profile
        )

        # 服务
        self.start_recording_service = self.create_service(
            SetBool,
            '/audio/start_recording',
            self.start_recording_callback
        )

        self.stop_recording_service = self.create_service(
            SetBool,
            '/audio/stop_recording',
            self.stop_recording_callback
        )

        # 配置参数
        self.default_duration = 3.0
        self.auto_publish = True

        # 统计信息
        self.frames_published = 0
        self.recording_count = 0

        logger.info("SimpleAudioInputNode初始化完成")
        logger.info(f"话题: /audio/raw, 服务: /audio/start_recording, /audio/stop_recording")

    def start_recording_callback(self, request, response):
        """
        开始录制服务回调

        Args:
            request: SetBool请求数据
            response: SetBool响应数据

        Returns:
            SetBool: 服务响应
        """
        try:
            logger.info("收到录制请求")

            # 开始录制
            success = self.start_recording(self.default_duration)

            response.success = success
            if success:
                response.message = f"开始录制，时长: {self.default_duration}秒"
                logger.info("录制启动成功")
            else:
                response.message = "录制启动失败，可能已经在录制中"
                logger.warning("录制启动失败")

        except Exception as e:
            logger.error(f"录制服务回调异常: {e}")
            response.success = False
            response.message = f"服务异常: {str(e)}"

        return response

    def stop_recording_callback(self, request, response):
        """
        停止录制服务回调

        Args:
            request: SetBool请求数据
            response: SetBool响应数据

        Returns:
            SetBool: 服务响应
        """
        try:
            logger.info("收到停止录制请求")

            # 停止录制
            audio_data = self.stop_recording()

            if len(audio_data) > 0:
                response.success = True
                response.message = f"录制停止，数据长度: {len(audio_data)}样本"
                self.recording_count += 1
                logger.info(f"录制停止成功，数据长度: {len(audio_data)}样本")
            else:
                response.success = False
                response.message = "没有录制到音频数据"
                logger.warning("录制停止但无数据")

        except Exception as e:
            logger.error(f"停止录制服务回调异常: {e}")
            response.success = False
            response.message = f"服务异常: {str(e)}"

        return response

    def start_recording(self, duration: float = 3.0) -> bool:
        """
        开始录制音频

        Args:
            duration: 录制时长（秒）

        Returns:
            bool: 录制启动成功状态
        """
        if self.is_recording:
            logger.warning("已在录制中")
            return False

        try:
            def recording_callback():
                """录制完成回调"""
                audio_data = self.recorder.stop_recording()
                if self.auto_publish and len(audio_data) > 0:
                    self.publish_audio_frame(audio_data)

            success = self.recorder.start_recording(
                duration=duration,
                callback=recording_callback
            )

            if success:
                self.is_recording = True
                logger.info(f"开始录制音频，时长: {duration}秒")

            return success

        except Exception as e:
            logger.error(f"开始录制失败: {e}")
            return False

    def stop_recording(self) -> np.ndarray:
        """
        停止录制并返回音频数据

        Returns:
            np.ndarray: 音频数据
        """
        if not self.is_recording:
            logger.warning("当前没有在录制")
            return np.array([], dtype=np.int16)

        try:
            self.is_recording = False
            audio_data = self.recorder.stop_recording()

            logger.info(f"录制停止，数据长度: {len(audio_data)}样本")
            return audio_data

        except Exception as e:
            logger.error(f"停止录制失败: {e}")
            return np.array([], dtype=np.int16)

    def publish_audio_frame(self, audio_data: np.ndarray) -> bool:
        """
        发布音频帧

        Args:
            audio_data: 音频数据

        Returns:
            bool: 发布成功状态
        """
        try:
            if len(audio_data) == 0:
                logger.warning("音频数据为空，跳过发布")
                return False

            # 创建音频帧消息
            frame = AudioFrame()
            frame.data = audio_data.tobytes()
            frame.frame_type = "audio"
            frame.smart_audio = False

            # 发布消息
            self.audio_publisher.publish(frame)
            self.frames_published += 1

            logger.debug(f"发布音频帧: {len(audio_data)}样本, 帧号: {self.frames_published}")
            return True

        except Exception as e:
            logger.error(f"发布音频帧失败: {e}")
            return False

    def record_and_publish(self, duration: float = 3.0) -> bool:
        """
        录制并发布音频帧

        Args:
            duration: 录制时长

        Returns:
            bool: 操作成功状态
        """
        try:
            logger.info(f"开始录制并发布音频，时长: {duration}秒")

            # 录制音频
            audio_data = self.recorder.stop_recording()
            if self.is_recording:
                audio_data = self.stop_recording()

            # 重新开始录制
            success = self.start_recording(duration)
            if not success:
                return False

            # 等待录制完成
            time.sleep(duration + 0.1)

            # 获取音频数据
            audio_data = self.stop_recording()

            # 发布音频
            if len(audio_data) > 0:
                return self.publish_audio_frame(audio_data)
            else:
                logger.warning("录制完成但没有数据")
                return False

        except Exception as e:
            logger.error(f"录制并发布音频失败: {e}")
            return False

    def get_node_info(self) -> dict:
        """
        获取节点信息

        Returns:
            dict: 节点信息
        """
        return {
            "node_name": self.get_name(),
            "namespace": self.get_namespace(),
            "topic": "/audio/raw",
            "services": [
                "/audio/start_recording",
                "/audio/stop_recording"
            ],
            "audio_config": self.recorder.get_audio_config(),
            "is_recording": self.is_recording,
            "frames_published": self.frames_published,
            "recording_count": self.recording_count,
            "architecture": "pure_online"
        }

    def test_node(self) -> bool:
        """
        测试节点功能

        Returns:
            bool: 测试通过状态
        """
        logger.info("开始音频输入节点测试...")

        try:
            # 测试音频录制器
            recorder_test = self.recorder.test_recording()
            if not recorder_test:
                logger.error("音频录制器测试失败")
                return False

            # 测试录制功能
            success = self.start_recording(duration=0.5)
            if not success:
                logger.error("启动录制测试失败")
                return False

            # 等待录制完成
            time.sleep(0.6)

            # 停止录制
            audio_data = self.stop_recording()
            if len(audio_data) == 0:
                logger.error("录制测试失败：没有数据")
                return False

            # 测试发布功能
            publish_success = self.publish_audio_frame(audio_data)
            if not publish_success:
                logger.error("音频发布测试失败")
                return False

            logger.info("音频输入节点测试通过")
            return True

        except Exception as e:
            logger.error(f"音频输入节点测试异常: {e}")
            return False

    def __del__(self):
        """析构函数"""
        if self.is_recording:
            self.stop_recording()


def main(args=None):
    """主函数"""
    try:
        # 初始化ROS2
        rclpy.init(args=args)

        # 创建节点
        node = SimpleAudioInputNode()

        # 显示节点信息
        info = node.get_node_info()
        logger.info("=== Simple Audio Input Node 启动 ===")
        for key, value in info.items():
            if key != "audio_config":
                logger.info(f"{key}: {value}")

        # 显示音频配置
        audio_config = info["audio_config"]
        logger.info("音频配置:")
        for key, value in audio_config.items():
            logger.info(f"  {key}: {value}")

        # 运行测试
        logger.info("\n运行节点测试...")
        test_result = node.test_node()
        logger.info(f"测试结果: {'通过' if test_result else '失败'}")

        if test_result:
            logger.info("\n节点开始运行，按Ctrl+C停止...")
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                logger.info("收到停止信号")
        else:
            logger.error("节点测试失败，退出")

        # 清理
        node.destroy_node()
        rclpy.shutdown()
        logger.info("节点已停止")

    except Exception as e:
        logger.error(f"节点运行异常: {e}")


if __name__ == '__main__':
    main()