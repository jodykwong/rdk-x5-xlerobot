"""
可扩展性管理器测试 - TestScalabilityManager
==========================================

测试ScalabilityManager类的功能：
- 水平扩展
- 插件管理
- 负载均衡
- 模块分析

Author: BMad System Architect
Created: 2025-11-05
"""

import unittest
import sys
import os
import time

# 添加模块路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))

try:
    from modules.system_control.scalability_manager import (
        ScalabilityManager,
        ScalingStrategy,
        ModuleLoad,
        ScalingMetrics,
        ModuleLoadData,
        PluginInfo
    )
except ImportError as e:
    print(f"Import error: {e}")
    sys.exit(1)


class TestScalabilityManager(unittest.TestCase):
    """可扩展性管理器测试类"""

    def setUp(self):
        """测试前准备"""
        self.scalability = ScalabilityManager()
        self.scalability.initialize()

    def tearDown(self):
        """测试后清理"""
        if hasattr(self.scalability, 'shutdown'):
            self.scalability.shutdown()

    def test_horizontal_scaling(self):
        """测试水平扩展"""
        # 扩展ASR模块
        result = self.scalability.scale_horizontal(
            module_type="asr",
            target_instances=3,
            strategy="add"
        )

        self.assertTrue(result)

        # 验证实例已注册
        instances = self.scalability.load_balancer.module_instances.get("asr", [])
        self.assertGreaterEqual(len(instances), 3)

        print(f"✓ Horizontal scaling: {len(instances)} instances")

    def test_plugin_management(self):
        """测试插件管理"""
        # 创建测试插件信息
        plugin_info = PluginInfo(
            plugin_id="test_plugin",
            name="Test Plugin",
            version="1.0.0",
            description="Test plugin for unit testing"
        )

        # 注册插件
        self.scalability.register_plugin(plugin_info)

        # 获取插件状态
        status = self.scalability.get_plugin_status()

        self.assertIn("total_plugins", status)
        self.assertIn("loaded_plugins", status)
        self.assertGreaterEqual(status["total_plugins"], 1)

        print(f"✓ Plugin management: {status['total_plugins']} plugins registered")

    def test_module_load_registration(self):
        """测试模块负载注册"""
        # 注册负载数据
        load_data = ModuleLoadData(
            module_id="test_module",
            cpu_usage=45.0,
            memory_usage=60.0,
            request_rate=100.0,
            avg_latency=8.5,
            error_count=0
        )

        self.scalability.register_module_load("test_module", load_data)

        # 验证负载数据已存储
        self.assertIn("test_module", self.scalability.module_loads)

        print("✓ Module load registration works correctly")

    def test_modularity_check(self):
        """测试模块化检查"""
        # 创建测试文件列表
        test_files = [
            __file__,  # 当前文件
            os.path.join(os.path.dirname(__file__), 'test_architecture.py')
        ]

        # 只检查存在的文件
        existing_files = [f for f in test_files if os.path.exists(f)]

        if existing_files:
            result = self.scalability.check_modularity(existing_files)

            self.assertIn("valid", result)
            self.assertIn("metrics", result)

            print(f"✓ Modularity check: {result['metrics']}")

    def test_scalability_validation(self):
        """测试可扩展性验证"""
        result = self.scalability.validate_scalability()

        self.assertIn("valid", result)
        self.assertIn("errors", result)
        self.assertIn("warnings", result)

        # 水平扩展配置应该有效
        config = self.scalability.scaling_config["horizontal_scaling"]
        self.assertLess(config["min_instances"], config["max_instances"])

        print(f"✓ Scalability validation: {result['valid']}")


class TestScalingMetrics(unittest.TestCase):
    """扩展指标测试类"""

    def test_scaling_metrics_creation(self):
        """测试扩展指标创建"""
        from datetime import datetime

        metrics = ScalingMetrics(
            timestamp=datetime.now(),
            active_modules=5,
            module_loads={"module1": ModuleLoad.LOW},
            resource_utilization={"cpu": 50.0},
            throughput=100.0,
            latency=8.5,
            error_rate=0.1
        )

        self.assertEqual(metrics.active_modules, 5)
        self.assertIn("module1", metrics.module_loads)

        print("✓ ScalingMetrics creation works correctly")


class TestModuleLoadData(unittest.TestCase):
    """模块负载数据测试类"""

    def test_module_load_data_creation(self):
        """测试模块负载数据创建"""
        load_data = ModuleLoadData(
            module_id="test",
            cpu_usage=50.0,
            memory_usage=60.0,
            request_rate=100.0,
            avg_latency=10.0,
            error_count=0
        )

        self.assertEqual(load_data.module_id, "test")
        self.assertEqual(load_data.cpu_usage, 50.0)
        self.assertEqual(load_data.memory_usage, 60.0)

        print("✓ ModuleLoadData creation works correctly")


class TestPluginInfo(unittest.TestCase):
    """插件信息测试类"""

    def test_plugin_info_creation(self):
        """测试插件信息创建"""
        plugin_info = PluginInfo(
            plugin_id="my_plugin",
            name="My Plugin",
            version="2.0.0",
            description="A sample plugin"
        )

        self.assertEqual(plugin_info.plugin_id, "my_plugin")
        self.assertEqual(plugin_info.name, "My Plugin")
        self.assertEqual(plugin_info.version, "2.0.0")

        print("✓ PluginInfo creation works correctly")


if __name__ == '__main__':
    # 运行测试
    unittest.main(verbosity=2)
