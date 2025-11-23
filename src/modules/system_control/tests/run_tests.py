"""
系统控制模块测试运行脚本
=======================

运行系统控制模块的所有测试并生成报告。

Author: BMad System Architect
Created: 2025-11-05
"""

import unittest
import sys
import os
from io import StringIO

# 添加项目根目录到路径
project_root = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..')
sys.path.insert(0, project_root)

# 导入所有测试模块
from test_architecture import TestSystemArchitecture
from test_module_coordinator import TestModuleCoordinator
from test_fault_tolerance import TestFaultTolerance
from test_scalability_manager import TestScalabilityManager
from test_architecture_validator import TestArchitectureValidator
from test_system_control_integration import TestSystemControlIntegration


def run_tests():
    """运行所有测试"""
    # 创建测试套件
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    # 添加测试类
    test_classes = [
        TestSystemArchitecture,
        TestModuleCoordinator,
        TestFaultTolerance,
        TestScalabilityManager,
        TestArchitectureValidator,
        TestSystemControlIntegration
    ]

    for test_class in test_classes:
        tests = loader.loadTestsFromTestCase(test_class)
        suite.addTests(tests)

    # 运行测试
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    # 输出摘要
    print("\n" + "=" * 70)
    print("测试摘要")
    print("=" * 70)
    print(f"总测试数: {result.testsRun}")
    print(f"成功: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"失败: {len(result.failures)}")
    print(f"错误: {len(result.errors)}")

    if result.wasSuccessful():
        print("\n✓ 所有测试通过！")
        return True
    else:
        print("\n✗ 部分测试失败")
        return False


if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1)
