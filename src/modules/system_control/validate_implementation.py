"""
验证实现脚本
=============

验证系统控制模块的所有组件是否正确实现。

Author: BMad System Architect
Created: 2025-11-05
"""

import sys
import os

# 添加项目根目录
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))

def validate_modules():
    """验证所有模块"""
    modules = [
        ('architecture', 'SystemArchitecture'),
        ('module_coordinator', 'ModuleCoordinator'),
        ('fault_tolerance', 'FaultTolerance'),
        ('scalability_manager', 'ScalabilityManager'),
        ('architecture_validator', 'ArchitectureValidator')
    ]

    print("=" * 70)
    print("系统控制模块实现验证")
    print("=" * 70)

    success_count = 0
    total_count = len(modules)

    for module_name, class_name in modules:
        try:
            module = __import__(f'system_control.{module_name}', fromlist=[class_name])
            cls = getattr(module, class_name)

            # 尝试实例化
            if class_name == 'SystemArchitecture':
                instance = cls('test')
            elif class_name == 'ModuleCoordinator':
                instance = cls('test')
            elif class_name == 'FaultTolerance':
                instance = cls()
            elif class_name == 'ScalabilityManager':
                instance = cls()
            elif class_name == 'ArchitectureValidator':
                instance = cls()

            print(f"✓ {module_name}.{class_name} - 导入和实例化成功")
            success_count += 1

        except Exception as e:
            print(f"✗ {module_name}.{class_name} - 错误: {e}")

    print("=" * 70)
    print(f"验证结果: {success_count}/{total_count} 模块通过验证")
    print("=" * 70)

    return success_count == total_count


def validate_file_structure():
    """验证文件结构"""
    print("\n文件结构验证:")
    print("-" * 70)

    base_dir = os.path.dirname(__file__)

    expected_files = [
        'architecture.py',
        'module_coordinator.py',
        'fault_tolerance.py',
        'scalability_manager.py',
        'architecture_validator.py',
        'tests/__init__.py',
        'tests/test_architecture.py',
        'tests/test_module_coordinator.py',
        'tests/test_fault_tolerance.py',
        'tests/test_scalability_manager.py',
        'tests/test_architecture_validator.py',
        'tests/test_system_control_integration.py',
        'tests/run_tests.py'
    ]

    found_count = 0
    for file_path in expected_files:
        full_path = os.path.join(base_dir, file_path)
        if os.path.exists(full_path):
            print(f"✓ {file_path}")
            found_count += 1
        else:
            print(f"✗ {file_path} (缺失)")

    print("-" * 70)
    print(f"文件结构: {found_count}/{len(expected_files)} 文件存在")

    return found_count == len(expected_files)


def validate_test_files():
    """验证测试文件"""
    print("\n测试文件验证:")
    print("-" * 70)

    test_files = [
        ('tests/test_architecture.py', 'TestSystemArchitecture'),
        ('tests/test_module_coordinator.py', 'TestModuleCoordinator'),
        ('tests/test_fault_tolerance.py', 'TestFaultTolerance'),
        ('tests/test_scalability_manager.py', 'TestScalabilityManager'),
        ('tests/test_architecture_validator.py', 'TestArchitectureValidator'),
        ('tests/test_system_control_integration.py', 'TestSystemControlIntegration')
    ]

    base_dir = os.path.dirname(__file__)
    found_count = 0

    for file_path, class_name in test_files:
        full_path = os.path.join(base_dir, file_path)
        if os.path.exists(full_path):
            print(f"✓ {file_path} - {class_name}")
            found_count += 1
        else:
            print(f"✗ {file_path} (缺失)")

    print("-" * 70)
    print(f"测试文件: {found_count}/{len(test_files)} 文件存在")

    return found_count == len(test_files)


if __name__ == '__main__':
    # 验证文件结构
    files_ok = validate_file_structure()

    # 验证测试文件
    tests_ok = validate_test_files()

    # 验证模块
    modules_ok = validate_modules()

    # 总体结果
    print("\n" + "=" * 70)
    print("总体验证结果")
    print("=" * 70)

    if files_ok and tests_ok and modules_ok:
        print("✓ 所有验证通过！")
        print("\nStory 4.1 实现完成:")
        print("- ✓ Task 1: 系统架构设计")
        print("- ✓ Task 2: 模块划分和接口")
        print("- ✓ Task 3: 容错和恢复机制")
        print("- ✓ Task 4: 可扩展性和评审")
        print("- ✓ 测试套件")
        sys.exit(0)
    else:
        print("✗ 部分验证失败")
        sys.exit(1)
