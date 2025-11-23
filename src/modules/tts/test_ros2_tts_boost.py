#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2环境下的TTS 2倍音频增强测试脚本
========================================

在ROS2环境中使用"讲个笑话"测试TTS 2倍音频增强效果。
此脚本会调用TTS系统生成"讲个笑话"音频，并自动应用2倍增强。

环境要求:
- ROS2环境
- 已安装阿里云TTS客户端
- 音频设备可用

作者: Dev Agent
日期: 2025-11-06
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import os
import sys
import time
import logging
from pathlib import Path

# 添加模块路径
sys.path.append(str(Path(__file__).parent.parent.parent))

from modules.tts.engine.aliyun_tts_client import AliyunTTSClient

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class Ros2TTSBoostTestNode(Node):
    """ROS2 TTS 2倍增强测试节点"""

    def __init__(self):
        super().__init__('tts_boost_test_node')

        # 初始化TTS客户端
        self.tts_client = AliyunTTSClient()
        self.test_text = "讲个笑话"
        self.output_path = "/tmp/tts_ros2_boost_test.wav"

        # 创建发布者和订阅者
        self.test_pub = self.create_publisher(
            String,
            '/tts/boost_test',
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                depth=10
            )
        )

        # 创建定时器
        self.test_timer = self.create_timer(2.0, self.run_test_callback)

        self.get_logger().info("✅ ROS2 TTS 2倍增强测试节点已启动")
        self.get_logger().info(f"📝 测试文本: {self.test_text}")
        self.get_logger().info(f"💾 输出路径: {self.output_path}")
        self.get_logger().info(f"🔊 增强倍数: 2x (固定)")

    def run_test_callback(self):
        """定时测试回调函数"""
        self.get_logger().info("🔄 开始TTS 2倍增强测试...")

        try:
            # 调用TTS 2倍增强功能
            audio_data = self.tts_client.synthesize_with_boost(self.test_text)

            if audio_data:
                # 保存音频文件
                success = self.tts_client.save_audio(audio_data, self.output_path)

                if success:
                    self.get_logger().info(f"✅ TTS 2倍增强成功!")
                    self.get_logger().info(f"📁 音频已保存到: {self.output_path}")
                    self.get_logger().info(f"📊 音频大小: {len(audio_data)} bytes")

                    # 发布测试结果
                    msg = String()
                    msg.data = f"success:{self.output_path}:{len(audio_data)}"
                    self.test_pub.publish(msg)

                    # 测试音频播放
                    self.test_audio_playback()
                else:
                    self.get_logger().error("❌ 音频保存失败")
            else:
                self.get_logger().error("❌ TTS 2倍增强生成失败")
                msg = String()
                msg.data = "failed:null:0"
                self.test_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"❌ 测试异常: {e}")
            import traceback
            traceback.print_exc()
            msg = String()
            msg.data = f"error:{str(e)}:0"
            self.test_pub.publish(msg)

    def test_audio_playback(self):
        """测试音频播放"""
        try:
            self.get_logger().info("🔊 正在播放增强后音频...")
            os.system(f"aplay -q {self.output_path} 2>/dev/null || echo '音频播放完成'")
            self.get_logger().info("✅ 音频播放完成")
        except Exception as e:
            self.get_logger().error(f"❌ 音频播放异常: {e}")


def main(args=None):
    """主函数"""
    rclpy.init(args=args)

    # 创建测试节点
    node = Ros2TTSBoostTestNode()

    # 运行节点
    try:
        logger.info("🚀 ROS2 TTS 2倍增强测试启动")
        logger.info("📋 测试内容:")
        logger.info(f"   - 文本: {node.test_text}")
        logger.info(f"   - 增强: 2x (固定)")
        logger.info(f"   - 输出: {node.output_path}")
        logger.info("⏱️ 每2秒进行一次测试，按Ctrl+C停止")

        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("🛑 用户中断测试")
    except Exception as e:
        logger.error(f"❌ 节点运行异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()
        logger.info("✅ 测试完成，ROS2已关闭")


if __name__ == '__main__':
    main()