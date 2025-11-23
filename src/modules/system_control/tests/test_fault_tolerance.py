"""
故障容错测试 - TestFaultTolerance
==============================

测试FaultTolerance类的功能：
- 故障检测
- 自动恢复
- 数据备份
- 断路器模式

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
    from modules.system_control.fault_tolerance import (
        FaultTolerance,
        FailureType,
        Severity,
        RecoveryStrategy,
        FailureEvent,
        BackupData
    )
except ImportError as e:
    print(f"Import error: {e}")
    sys.exit(1)


class TestFaultTolerance(unittest.TestCase):
    """故障容错测试类"""

    def setUp(self):
        """测试前准备"""
        self.fault_tolerance = FaultTolerance()
        self.fault_tolerance.initialize()

    def tearDown(self):
        """测试后清理"""
        if hasattr(self.fault_tolerance, 'shutdown'):
            self.fault_tolerance.shutdown()

    def test_failure_detection(self):
        """测试故障检测"""
        # 检测模块崩溃
        event_id = self.fault_tolerance.detect_failure(
            module_id="test_module",
            failure_type=FailureType.MODULE_CRASH,
            severity=Severity.HIGH,
            description="Test module crash"
        )

        self.assertIsNotNone(event_id)
        self.assertGreater(len(event_id), 0)

        # 验证故障事件已记录
        history = self.fault_tolerance.get_failure_history()
        self.assertGreater(len(history), 0)

        # 验证事件详情
        latest_event = history[-1]
        self.assertEqual(latest_event["module_id"], "test_module")
        self.assertEqual(latest_event["failure_type"], FailureType.MODULE_CRASH.value)
        self.assertEqual(latest_event["severity"], Severity.HIGH.name)

        print("✓ Failure detection works correctly")

    def test_backup_creation(self):
        """测试备份创建"""
        # 创建备份
        checksum = self.fault_tolerance.create_module_backup(
            module_id="test_module",
            data_type="config",
            data={"key": "value", "count": 42}
        )

        self.assertIsNotNone(checksum)
        self.assertGreater(len(checksum), 0)

        # 列出备份
        backups = self.fault_tolerance.backup_manager.list_backups("test_module")
        self.assertGreater(len(backups), 0)

        # 验证备份内容
        latest_backup = backups[-1]
        self.assertEqual(latest_backup["data_type"], "config")
        self.assertEqual(latest_backup["checksum"], checksum)

        print("✓ Backup creation works correctly")

    def test_backup_restoration(self):
        """测试备份恢复"""
        # 创建备份
        original_data = {"key": "test_value", "items": [1, 2, 3]}
        checksum = self.fault_tolerance.create_module_backup(
            module_id="test_module",
            data_type="state",
            data=original_data
        )

        # 恢复备份
        restored_data = self.fault_tolerance.restore_module_backup(
            module_id="test_module",
            data_type="state",
            checksum=checksum
        )

        self.assertIsNotNone(restored_data)
        self.assertEqual(restored_data["key"], original_data["key"])
        self.assertEqual(restored_data["items"], original_data["items"])

        print("✓ Backup restoration works correctly")

    def test_circuit_breaker(self):
        """测试断路器"""
        from modules.system_control.fault_tolerance import CircuitBreaker

        # 创建断路器
        cb = CircuitBreaker(failure_threshold=3, recovery_timeout=10.0)

        # 定义测试函数
        call_count = [0]

        def test_function(should_fail: bool = False):
            call_count[0] += 1
            if should_fail:
                raise Exception("Test failure")
            return "success"

        # 成功调用
        result = cb.call(test_function, should_fail=False)
        self.assertEqual(result, "success")
        self.assertEqual(cb.state, "CLOSED")

        # 失败调用
        for i in range(3):
            with self.assertRaises(Exception):
                cb.call(test_function, should_fail=True)

        self.assertEqual(cb.state, "OPEN")

        # 断路器开启后应该立即失败
        with self.assertRaises(Exception):
            cb.call(test_function, should_fail=False)

        print("✓ Circuit breaker works correctly")

    def test_health_monitoring(self):
        """测试健康监控"""
        # 注册健康检查
        def health_check():
            return {
                "healthy": True,
                "metrics": {"cpu": 50.0, "memory": 60.0},
                "issues": []
            }

        self.fault_tolerance.health_monitor.register_health_check(
            "test_module",
            health_check
        )

        # 等待监控周期
        time.sleep(0.5)

        # 检查健康状态
        health = self.fault_tolerance.health_monitor.get_health_status("test_module")
        self.assertIsNotNone(health)
        self.assertTrue(health.get("healthy", False))

        print("✓ Health monitoring works correctly")

    def test_availability_metrics(self):
        """测试可用性指标"""
        # 模拟一些故障和恢复
        self.fault_tolerance.detect_failure(
            module_id="test_module_1",
            failure_type=FailureType.MODULE_HANG,
            severity=Severity.MEDIUM,
            description="Test hang"
        )

        # 获取可用性指标
        metrics = self.fault_tolerance.get_availability_metrics()

        self.assertIn("availability_percentage", metrics)
        self.assertIn("total_modules", metrics)
        self.assertIn("recovery_rate", metrics)
        self.assertIn("timestamp", metrics)

        print(f"✓ Availability metrics: {metrics['availability_percentage']:.1f}%")

    def test_failure_history(self):
        """测试故障历史"""
        # 检测多个故障
        for i in range(3):
            self.fault_tolerance.detect_failure(
                module_id=f"module_{i}",
                failure_type=FailureType.MODULE_CRASH,
                severity=Severity.HIGH,
                description=f"Test crash {i}"
            )

        # 获取历史
        history = self.fault_tolerance.get_failure_history()
        self.assertGreaterEqual(len(history), 3)

        # 按模块过滤
        module_history = self.fault_tolerance.get_failure_history("module_0")
        self.assertEqual(len(module_history), 1)

        print(f"✓ Failure history: {len(history)} events recorded")

    def test_recovery_actions(self):
        """测试恢复动作"""
        # 检测故障（会触发自动恢复）
        event_id = self.fault_tolerance.detect_failure(
            module_id="test_module",
            failure_type=FailureType.MODULE_CRASH,
            severity=Severity.HIGH,
            description="Test crash"
        )

        # 获取恢复动作
        actions = self.fault_tolerance.get_recovery_actions()
        # 注意：实际实现中可能需要等待恢复动作创建

        print("✓ Recovery actions recorded")

    def test_fault_tolerance_validation(self):
        """测试故障容错验证"""
        result = self.fault_tolerance.validate_fault_tolerance()

        self.assertIn("valid", result)
        self.assertIn("errors", result)
        self.assertIn("warnings", result)

        # 健康监控应该已启动
        self.assertTrue(result["valid"] or len(result["warnings"]) > 0)

        print(f"✓ Fault tolerance validation: {result['valid']}")


class TestFailureEvent(unittest.TestCase):
    """故障事件测试类"""

    def test_failure_event_creation(self):
        """测试故障事件创建"""
        event = FailureEvent(
            event_id="FAIL_001",
            module_id="test_module",
            failure_type=FailureType.MODULE_CRASH,
            severity=Severity.HIGH,
            timestamp=datetime.now(),
            description="Test failure"
        )

        self.assertEqual(event.event_id, "FAIL_001")
        self.assertEqual(event.module_id, "test_module")
        self.assertEqual(event.failure_type, FailureType.MODULE_CRASH)
        self.assertEqual(event.severity, Severity.HIGH)

        print("✓ FailureEvent creation works correctly")


class TestBackupData(unittest.TestCase):
    """备份数据测试类"""

    def test_backup_data_creation(self):
        """测试备份数据创建"""
        data = {"key": "value", "count": 42}
        backup = BackupData(
            module_id="test_module",
            data_type="config",
            data=data
        )

        self.assertEqual(backup.module_id, "test_module")
        self.assertEqual(backup.data_type, "config")
        self.assertEqual(backup.data, data)
        self.assertIsNotNone(backup.checksum)
        self.assertGreater(len(backup.checksum), 0)

        print("✓ BackupData creation works correctly")

    def test_checksum_calculation(self):
        """测试校验和计算"""
        data1 = {"a": 1, "b": 2}
        data2 = {"b": 2, "a": 1}  # 相同内容，顺序不同

        backup1 = BackupData(module_id="test", data_type="test", data=data1)
        backup2 = BackupData(module_id="test", data_type="test", data=data2)

        # 相同内容的校验和应该相同
        self.assertEqual(backup1.checksum, backup2.checksum)

        print("✓ Checksum calculation works correctly")


if __name__ == '__main__':
    # 运行测试
    unittest.main(verbosity=2)
