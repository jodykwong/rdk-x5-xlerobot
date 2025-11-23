"""
系统架构测试 - TestSystemArchitecture
===================================

测试SystemArchitecture类的功能：
- 核心模块注册
- 依赖关系管理
- 消息流配置
- 架构验证

Author: BMad System Architect
Created: 2025-11-05
"""

import unittest
import sys
import os
from datetime import datetime

# 添加模块路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))

try:
    from modules.system_control.architecture import (
        SystemArchitecture,
        ModuleType,
        ModuleState,
        ModuleInfo,
        MessageFlow
    )
except ImportError as e:
    print(f"Import error: {e}")
    sys.exit(1)


class TestSystemArchitecture(unittest.TestCase):
    """系统架构测试类"""

    def setUp(self):
        """测试前准备"""
        self.arch = SystemArchitecture("test_arch")
        self.arch.initialize()

    def tearDown(self):
        """测试后清理"""
        import asyncio
        if hasattr(self.arch, 'shutdown'):
            asyncio.run(self.arch.shutdown())

    def test_core_modules_registration(self):
        """测试核心模块注册"""
        # 验证5个核心模块都已注册
        modules = self.arch.list_modules()

        module_types = [m.module_type for m in modules]
        self.assertIn(ModuleType.ASR, module_types)
        self.assertIn(ModuleType.LLM, module_types)
        self.assertIn(ModuleType.TTS, module_types)
        self.assertIn(ModuleType.SYSTEM_CONTROL, module_types)
        self.assertIn(ModuleType.SMART_HOME, module_types)

        print(f"✓ Registered {len(modules)} core modules")

    def test_module_registration(self):
        """测试模块注册和注销"""
        # 创建测试模块
        test_module = ModuleInfo(
            module_id="test_module",
            module_type=ModuleType.SYSTEM_CONTROL,
            name="Test Module",
            version="1.0.0"
        )

        # 注册模块
        result = self.arch.register_module(test_module)
        self.assertTrue(result)

        # 验证模块存在
        module_info = self.arch.get_module_info("test_module")
        self.assertIsNotNone(module_info)
        self.assertEqual(module_info.module_id, "test_module")

        # 注销模块
        result = self.arch.unregister_module("test_module")
        self.assertTrue(result)

        # 验证模块已删除
        module_info = self.arch.get_module_info("test_module")
        self.assertIsNone(module_info)

        print("✓ Module registration/unregistration works correctly")

    def test_module_lifecycle(self):
        """测试模块生命周期"""
        # 创建测试模块
        test_module = ModuleInfo(
            module_id="lifecycle_test",
            module_type=ModuleType.SYSTEM_CONTROL,
            name="Lifecycle Test",
            version="1.0.0"
        )

        self.arch.register_module(test_module)

        # 启动模块
        result = self.arch.start_module("lifecycle_test")
        self.assertTrue(result)

        # 检查状态
        state = self.arch.get_module_state("lifecycle_test")
        self.assertEqual(state, ModuleState.READY)

        # 停止模块
        result = self.arch.stop_module("lifecycle_test")
        self.assertTrue(result)

        # 检查状态
        state = self.arch.get_module_state("lifecycle_test")
        self.assertEqual(state, ModuleState.STOPPED)

        print("✓ Module lifecycle management works correctly")

    def test_dependency_management(self):
        """测试依赖关系管理"""
        # 验证依赖关系图
        dep_graph = self.arch.get_dependency_graph()

        # LLM模块应该依赖ASR和系统控制
        self.assertIn("asr_core", dep_graph.get("llm_core", set()))
        self.assertIn("system_control", dep_graph.get("llm_core", set()))

        # TTS模块应该依赖LLM和系统控制
        self.assertIn("llm_core", dep_graph.get("tts_core", set()))
        self.assertIn("system_control", dep_graph.get("tts_core", set()))

        print("✓ Dependency management works correctly")

    def test_message_flows(self):
        """测试消息流配置"""
        flows = self.arch.get_message_flows()

        # 验证消息流
        flow_topics = [f.topic for f in flows]

        self.assertIn("/xlerobot/asr/text", flow_topics)
        self.assertIn("/xlerobot/llm/reply", flow_topics)
        self.assertIn("/xlerobot/control/command", flow_topics)

        print(f"✓ Configured {len(flows)} message flows")

    def test_system_health(self):
        """测试系统健康状态"""
        health = self.arch.get_system_health()

        self.assertIn("total_modules", health)
        self.assertIn("health_percentage", health)
        self.assertIn("availability", health)
        self.assertIn("timestamp", health)

        self.assertGreaterEqual(health["health_percentage"], 0)
        self.assertLessEqual(health["health_percentage"], 100)

        print(f"✓ System health: {health['health_percentage']:.1f}% modules healthy")

    def test_architecture_validation(self):
        """测试架构验证"""
        result = self.arch.validate_architecture()

        self.assertIn("valid", result)
        self.assertIn("errors", result)
        self.assertIn("warnings", result)

        # 应该没有循环依赖错误
        circular_deps = [e for e in result["errors"] if "circular" in e.lower()]
        self.assertEqual(len(circular_deps), 0, "No circular dependencies allowed")

        print(f"✓ Architecture validation passed: {result['valid']}")

    def test_performance_metrics(self):
        """测试性能指标"""
        # 更新性能指标
        metrics = {
            "cpu_usage": 45.0,
            "memory_usage": 60.0,
            "latency_ms": 8.5
        }
        self.arch.update_performance_metrics("asr_core", metrics)

        # 验证指标已更新
        module_info = self.arch.get_module_info("asr_core")
        self.assertIn("cpu_usage", module_info.performance_metrics)

        print("✓ Performance metrics tracking works correctly")

    def test_module_dependent_check(self):
        """测试模块依赖检查"""
        # LLM应该依赖ASR
        self.assertTrue(
            self.arch.is_module_dependent_on("llm_core", "asr_core")
        )

        # ASR不应该依赖LLM
        self.assertFalse(
            self.arch.is_module_dependent_on("asr_core", "llm_core")
        )

        print("✓ Module dependency check works correctly")


class TestModuleInfo(unittest.TestCase):
    """模块信息测试类"""

    def test_module_info_creation(self):
        """测试模块信息创建"""
        module = ModuleInfo(
            module_id="test_001",
            module_type=ModuleType.ASR,
            name="Test ASR",
            version="1.0.0",
            dependencies=["system_control"]
        )

        self.assertEqual(module.module_id, "test_001")
        self.assertEqual(module.module_type, ModuleType.ASR)
        self.assertEqual(module.name, "Test ASR")
        self.assertEqual(module.version, "1.0.0")
        self.assertIn("system_control", module.dependencies)

        print("✓ ModuleInfo creation works correctly")


class TestMessageFlow(unittest.TestCase):
    """消息流测试类"""

    def test_message_flow_creation(self):
        """测试消息流创建"""
        flow = MessageFlow(
            source_module="asr_core",
            target_module="llm_core",
            message_type="voice_text",
            topic="/xlerobot/asr/text"
        )

        self.assertEqual(flow.source_module, "asr_core")
        self.assertEqual(flow.target_module, "llm_core")
        self.assertEqual(flow.message_type, "voice_text")
        self.assertEqual(flow.topic, "/xlerobot/asr/text")

        print("✓ MessageFlow creation works correctly")


if __name__ == '__main__':
    # 运行测试
    unittest.main(verbosity=2)
