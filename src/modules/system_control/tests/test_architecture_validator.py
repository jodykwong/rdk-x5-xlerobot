"""
架构验证器测试 - TestArchitectureValidator
=========================================

测试ArchitectureValidator类的功能：
- 架构验证
- 代码质量检查
- 依赖分析
- 架构原则检查

Author: BMad System Architect
Created: 2025-11-05
"""

import unittest
import sys
import os

# 添加模块路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))

try:
    from modules.system_control.architecture_validator import (
        ArchitectureValidator,
        ValidationLevel,
        ValidationResult
    )
except ImportError as e:
    print(f"Import error: {e}")
    sys.exit(1)


class TestArchitectureValidator(unittest.TestCase):
    """架构验证器测试类"""

    def setUp(self):
        """测试前准备"""
        self.validator = ArchitectureValidator()

    def test_architecture_validation_basic(self):
        """测试基础架构验证"""
        # 使用当前测试文件进行验证
        test_files = [__file__]

        result = self.validator.validate_architecture(
            test_files,
            level=ValidationLevel.BASIC
        )

        self.assertIn("timestamp", result)
        self.assertIn("total_checks", result)
        self.assertIn("passed", result)
        self.assertIn("warnings", result)
        self.assertIn("failed", result)
        self.assertIn("metrics", result)

        print(f"✓ Basic validation: {result['passed']} passed, {result['warnings']} warnings")

    def test_architecture_validation_standard(self):
        """测试标准架构验证"""
        test_files = [__file__]

        result = self.validator.validate_architecture(
            test_files,
            level=ValidationLevel.STANDARD
        )

        # 标准验证应该包含代码质量检查
        quality_checks = [
            c for c in result["checks"]
            if c["id"].startswith("QUALITY_")
        ]

        print(f"✓ Standard validation: {len(quality_checks)} quality checks")

    def test_architecture_validation_strict(self):
        """测试严格架构验证"""
        test_files = [__file__]

        result = self.validator.validate_architecture(
            test_files,
            level=ValidationLevel.STRICT
        )

        # 严格验证应该包含架构原则检查
        principle_checks = [
            c for c in result["checks"]
            if c["id"].startswith("PRINCIPLE_")
        ]

        print(f"✓ Strict validation: {len(principle_checks)} principle checks")

    def test_validation_metrics(self):
        """测试验证指标"""
        test_files = [__file__]

        result = self.validator.validate_architecture(
            test_files,
            level=ValidationLevel.BASIC
        )

        metrics = result["metrics"]

        self.assertIn("modularity_score", metrics)
        self.assertIn("coupling_score", metrics)
        self.assertIn("cohesion_score", metrics)
        self.assertIn("testability_score", metrics)
        self.assertIn("maintainability_score", metrics)
        self.assertIn("overall_score", metrics)

        # 得分应该在0-100之间
        for score_name, score_value in metrics.items():
            self.assertGreaterEqual(score_value, 0.0)
            self.assertLessEqual(score_value, 100.0)

        print(f"✓ Validation metrics: overall={metrics['overall_score']:.1f}")

    def test_validation_history(self):
        """测试验证历史"""
        # 进行多次验证
        for _ in range(3):
            self.validator.validate_architecture(
                [__file__],
                level=ValidationLevel.BASIC
            )

        # 获取历史
        history = self.validator.get_validation_history()

        self.assertGreaterEqual(len(history), 3)

        for entry in history:
            self.assertIn("timestamp", entry)
            self.assertIn("check_count", entry)
            self.assertIn("passed", entry)

        print(f"✓ Validation history: {len(history)} entries")

    def test_current_metrics(self):
        """测试当前指标"""
        # 执行验证
        self.validator.validate_architecture(
            [__file__],
            level=ValidationLevel.BASIC
        )

        # 获取当前指标
        metrics = self.validator.get_current_metrics()

        self.assertIsNotNone(metrics)
        self.assertGreaterEqual(metrics.overall_score, 0.0)
        self.assertLessEqual(metrics.overall_score, 100.0)

        print("✓ Current metrics available")


if __name__ == '__main__':
    # 运行测试
    unittest.main(verbosity=2)
