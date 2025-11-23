#!/usr/bin/env python3.10
"""
XLeRobot语音链路测试脚本
测试ASR→LLM→TTS完整功能链路
"""
import rclpy
from rclpy.node import Node
from audio_msg.msg import ASRResult, LLMResponse
from std_msgs.msg import String
import time

class VoiceChainTester(Node):
    def __init__(self):
        super().__init__('voice_chain_tester')

        # 创建发布者
        self.asr_publisher = self.create_publisher(
            ASRResult, '/xlerobot/asr/result', 10)

        # 创建订阅者
        self.llm_subscription = self.create_subscription(
            LLMResponse, '/xlerobot/llm/response', self.llm_callback, 10)

        self.llm_string_sub = self.create_subscription(
            String, '/xlerobot/llm/response_string', self.llm_string_callback, 10)

        # 测试状态
        self.test_sent = False
        self.llm_received = False
        self.test_text = "测试语音识别功能"

        self.get_logger().info("🧪 XLeRobot语音链路测试节点启动")
        self.get_logger().info(f"📝 测试文本: {self.test_text}")

        # 延迟发送测试消息
        self.timer = self.create_timer(3.0, self.send_test_message)

    def send_test_message(self):
        """发送测试ASR结果"""
        if not self.test_sent:
            # 创建模拟ASR结果
            asr_result = ASRResult()
            asr_result.header.stamp = self.get_clock().now().to_msg()
            asr_result.header.frame_id = "voice_test"
            asr_result.text = self.test_text
            asr_result.confidence = 0.95
            asr_result.begin_time = 0
            asr_result.end_time = 1000
            asr_result.status_code = 200
            asr_result.message = "测试成功"

            self.asr_publisher.publish(asr_result)
            self.test_sent = True

            self.get_logger().info("📤 已发送测试ASR结果")
            self.get_logger().info("⏳ 等待LLM响应...")

            # 设置超时定时器
            self.timeout_timer = self.create_timer(15.0, self.test_timeout)

    def llm_callback(self, msg):
        """LLM响应回调"""
        self.get_logger().info(f"📥 收到LLM响应 (类型: {type(msg).__name__})")
        self.get_logger().info(f"📄 响应内容: {msg}")
        self.llm_received = True

    def llm_string_callback(self, msg):
        """LLM字符串响应回调"""
        self.get_logger().info(f"📥 收到LLM字符串响应: {msg.data}")
        if msg.data:
            self.get_logger().info("✅ ASR→LLM链路测试成功！")
            self.llm_received = True

    def test_timeout(self):
        """测试超时"""
        if not self.llm_received:
            self.get_logger().warning("⚠️ 15秒内未收到LLM响应")
            self.get_logger().info("🔍 可能原因:")
            self.get_logger().info("   1. LLM节点未正确启动")
            self.get_logger().info("   2. 环境变量配置问题")
            self.get_logger().info("   3. 网络连接问题")

        self.get_logger().info("🏁 测试完成")
        rclpy.shutdown()

def main():
    rclpy.init()

    try:
        tester = VoiceChainTester()

        # 测试最大运行时间20秒
        timeout_timer = tester.create_timer(20.0, lambda: rclpy.shutdown())

        rclpy.spin(tester)

    except KeyboardInterrupt:
        print("测试被用户中断")
    except Exception as e:
        print(f"测试异常: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()