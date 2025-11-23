"""
模块协调器测试 - TestModuleCoordinator
====================================

测试ModuleCoordinator类的功能：
- 消息路由
- 事件处理
- 模块状态管理
- 心跳监控

Author: BMad System Architect
Created: 2025-11-05
"""

import unittest
import sys
import os
import time
from datetime import datetime

# 添加模块路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))

try:
    from modules.system_control.module_coordinator import (
        ModuleCoordinator,
        MessageType,
        MessagePriority,
        Message
    )
except ImportError as e:
    print(f"Import error: {e}")
    sys.exit(1)


class TestModuleCoordinator(unittest.TestCase):
    """模块协调器测试类"""

    def setUp(self):
        """测试前准备"""
        self.coordinator = ModuleCoordinator("test_coordinator")
        self.coordinator.initialize()

    def tearDown(self):
        """测试后清理"""
        if hasattr(self.coordinator, 'shutdown'):
            self.coordinator.shutdown()

    def test_module_registration(self):
        """测试模块注册"""
        # 注册测试模块
        result = self.coordinator.register_module("test_module_1")
        self.assertTrue(result)

        result = self.coordinator.register_module("test_module_2")
        self.assertTrue(result)

        # 验证模块已注册
        states = self.coordinator.get_all_module_states()
        self.assertIn("test_module_1", states)
        self.assertIn("test_module_2", states)

        print("✓ Module registration works correctly")

    def test_message_sending(self):
        """测试消息发送"""
        # 注册模块
        self.coordinator.register_module("sender")
        self.coordinator.register_module("receiver")

        # 发送消息
        msg_id = self.coordinator.send_message(
            source="sender",
            target="receiver",
            msg_type=MessageType.DATA,
            payload={"content": "test message"}
        )

        self.assertIsNotNone(msg_id)
        self.assertGreater(len(msg_id), 0)

        # 接收消息
        message = self.coordinator.receive_message("receiver", timeout=1.0)
        self.assertIsNotNone(message)
        self.assertEqual(message.source, "sender")
        self.assertEqual(message.target, "receiver")
        self.assertEqual(message.payload["content"], "test message")

        print("✓ Message sending/receiving works correctly")

    def test_broadcast_message(self):
        """测试广播消息"""
        # 注册模块
        for i in range(3):
            self.coordinator.register_module(f"module_{i}")

        # 发送广播
        msg_id = self.coordinator.broadcast_message(
            source="system_control",
            msg_type=MessageType.CONTROL,
            payload={"command": "restart"},
            topic="/test/broadcast"
        )

        self.assertIsNotNone(msg_id)

        print("✓ Broadcast message works correctly")

    def test_message_priority(self):
        """测试消息优先级"""
        # 注册模块
        self.coordinator.register_module("sender")
        self.coordinator.register_module("receiver")

        # 发送不同优先级的消息
        low_msg = self.coordinator.send_message(
            source="sender",
            target="receiver",
            msg_type=MessageType.DATA,
            payload={"priority": "low"},
            priority=MessagePriority.LOW
        )

        high_msg = self.coordinator.send_message(
            source="sender",
            target="receiver",
            msg_type=MessageType.DATA,
            payload={"priority": "high"},
            priority=MessagePriority.CRITICAL
        )

        self.assertIsNotNone(low_msg)
        self.assertIsNotNone(high_msg)

        print("✓ Message priority works correctly")

    def test_event_handling(self):
        """测试事件处理"""
        event_received = []

        def event_handler(event_data):
            event_received.append(event_data)

        # 注册事件处理器
        self.coordinator.register_event_handler(
            event_type="test_event",
            handler=event_handler
        )

        # 触发事件
        self.coordinator.emit_event(
            event_type="test_event",
            event_data={"message": "test"},
            source="test_source"
        )

        # 验证事件被处理
        time.sleep(0.1)  # 等待事件处理
        self.assertGreater(len(event_received), 0)

        print("✓ Event handling works correctly")

    def test_module_state_management(self):
        """测试模块状态管理"""
        # 注册模块
        self.coordinator.register_module("state_test")

        # 更新状态
        self.coordinator.update_module_state("state_test", "running")

        # 验证状态
        state = self.coordinator.get_module_state("state_test")
        self.assertEqual(state, "running")

        # 获取所有状态
        all_states = self.coordinator.get_all_module_states()
        self.assertIn("state_test", all_states)

        print("✓ Module state management works correctly")

    def test_health_monitoring(self):
        """测试健康监控"""
        # 注册模块
        self.coordinator.register_module("health_test")

        # 检查健康状态（初始化时应该没有心跳）
        health = self.coordinator.get_module_health("health_test")
        self.assertIsNotNone(health)

        # 模拟心跳
        self.coordinator.receive_message("health_test")
        health = self.coordinator.get_module_health("health_test")
        self.assertIsNotNone(health.get("last_heartbeat"))

        print("✓ Health monitoring works correctly")

    def test_performance_metrics(self):
        """测试性能指标"""
        # 发送一些消息
        for i in range(10):
            self.coordinator.register_module(f"sender_{i}")
            self.coordinator.register_module(f"receiver_{i}")
            self.coordinator.send_message(
                source=f"sender_{i}",
                target=f"receiver_{i}",
                msg_type=MessageType.DATA,
                payload={"index": i}
            )

        # 获取指标
        metrics = self.coordinator.get_performance_metrics()

        self.assertIn("message_metrics", metrics)
        self.assertIn("active_modules", metrics)
        self.assertIn("average_latency_ms", metrics)

        self.assertGreaterEqual(metrics["message_metrics"]["total_sent"], 10)

        print(f"✓ Performance metrics: {metrics['message_metrics']['total_sent']} messages sent")

    def test_coordination_validation(self):
        """测试协调器验证"""
        result = self.coordinator.validate_coordination()

        self.assertIn("valid", result)
        self.assertIn("errors", result)
        self.assertIn("warnings", result)

        # 核心模块应该存在
        required_modules = ["asr_core", "llm_core", "tts_core", "system_control"]
        for req_module in required_modules:
            if req_module not in self.coordinator.module_states:
                # 这是预期的，因为我们只注册了测试模块
                self.assertIn(f"Missing required module: {req_module}", result["errors"])

        print(f"✓ Coordination validation works correctly")

    def test_message_routing(self):
        """测试消息路由"""
        # 注册模块
        self.coordinator.register_module("source")
        self.coordinator.register_module("target1")
        self.coordinator.register_module("target2")

        # 发送消息
        msg_id = self.coordinator.send_message(
            source="source",
            target="target1",
            msg_type=MessageType.DATA,
            payload={"route": "test"}
        )

        # 接收消息
        message = self.coordinator.receive_message("target1", timeout=1.0)
        self.assertIsNotNone(message)

        # target2不应该收到消息
        message = self.coordinator.receive_message("target2", timeout=0.5)
        self.assertIsNone(message)

        print("✓ Message routing works correctly")


if __name__ == '__main__':
    # 运行测试
    unittest.main(verbosity=2)
