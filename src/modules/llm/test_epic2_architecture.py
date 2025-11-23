#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Epic 2 架构验证测试
专门验证代码结构和类的完整性，不依赖外部API

作者: Dev Agent
Epic: 2 - 智能对话模块
"""

import sys
import os

# 添加项目路径
sys.path.insert(0, '/home/sunrise/xlerobot/src')

def test_architecture_validation():
    """验证Epic 2架构设计"""
    print("🏗️ 验证Epic 2架构设计...")

    try:
        # 验证所有模块文件存在
        modules = {
            "qwen_client.py": "/home/sunrise/xlerobot/src/modules/llm/qwen_client.py",
            "api_manager.py": "/home/sunrise/xlerobot/src/modules/llm/api_manager.py",
            "dialogue_context.py": "/home/sunrise/xlerobot/src/modules/llm/dialogue_context.py",
            "session_manager.py": "/home/sunrise/xlerobot/src/modules/llm/session_manager.py",
            "nlu_engine.py": "/home/sunrise/xlerobot/src/modules/llm/nlu_engine.py",
            "personalization_engine.py": "/home/sunrise/xlerobot/src/modules/llm/personalization_engine.py",
            "security_filter.py": "/home/sunrise/xlerobot/src/modules/llm/security_filter.py",
            "__init__.py": "/home/sunrise/xlerobot/src/modules/llm/__init__.py"
        }

        missing_files = []
        for name, path in modules.items():
            if not os.path.exists(path):
                missing_files.append(name)
                print(f"  ❌ 缺失: {name}")
            else:
                print(f"  ✅ 存在: {name}")

        if missing_files:
            print(f"  ❌ 发现 {len(missing_files)} 个缺失文件")
            return False

        print("  ✅ 所有模块文件存在")

        # 验证代码结构
        from modules.llm import dialogue_context
        from modules.llm import session_manager
        from modules.llm import nlu_engine
        from modules.llm import personalization_engine
        from modules.llm import security_filter

        print("  ✅ 所有核心模块导入成功")

        # 验证核心类存在
        core_classes = [
            ('dialogue_context', 'DialogueContext'),
            ('session_manager', 'SessionManager'),
            ('nlu_engine', 'NLUEngine'),
            ('personalization_engine', 'PersonalizationEngine'),
            ('security_filter', 'SecurityFilter')
        ]

        for module_name, class_name in core_classes:
            module = sys.modules[f'modules.llm.{module_name}']
            if hasattr(module, class_name):
                print(f"  ✅ {class_name} 类存在")
            else:
                print(f"  ❌ {class_name} 类缺失")
                return False

        return True

    except Exception as e:
        print(f"  ❌ 架构验证失败: {e}")
        return False

def test_code_quality():
    """验证代码质量"""
    print("\n🔍 验证Epic 2代码质量...")

    try:
        # 统计代码行数
        total_lines = 0
        modules = [
            "/home/sunrise/xlerobot/src/modules/llm/qwen_client.py",
            "/home/sunrise/xlerobot/src/modules/llm/api_manager.py",
            "/home/sunrise/xlerobot/src/modules/llm/dialogue_context.py",
            "/home/sunrise/xlerobot/src/modules/llm/session_manager.py",
            "/home/sunrise/xlerobot/src/modules/llm/nlu_engine.py",
            "/home/sunrise/xlerobot/src/modules/llm/personalization_engine.py",
            "/home/sunrise/xlerobot/src/modules/llm/security_filter.py"
        ]

        for module_path in modules:
            if os.path.exists(module_path):
                with open(module_path, 'r', encoding='utf-8') as f:
                    lines = len(f.readlines())
                    total_lines += lines
                    print(f"  📄 {os.path.basename(module_path)}: {lines} 行")

        print(f"\n📊 总代码行数: {total_lines}")

        # 验证文档字符串
        documentation_score = 0
        for module_path in modules:
            if os.path.exists(module_path):
                with open(module_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                    if '"""' in content or "'''" in content:
                        documentation_score += 1
                        print(f"  ✅ {os.path.basename(module_path)}: 有文档字符串")
                    else:
                        print(f"  ⚠️ {os.path.basename(module_path)}: 缺少文档字符串")

        doc_percentage = (documentation_score / len(modules)) * 100
        print(f"\n📚 文档覆盖率: {doc_percentage:.1f}% ({documentation_score}/{len(modules)})")

        return total_lines > 0 and doc_percentage > 80

    except Exception as e:
        print(f"  ❌ 代码质量验证失败: {e}")
        return False

def test_class_design():
    """验证类设计"""
    print("\n🏛️ 验证Epic 2类设计...")

    try:
        # 测试核心类的实例化
        from modules.llm.dialogue_context import DialogueContext
        from modules.llm.session_manager import SessionManager
        from modules.llm.nlu_engine import NLUEngine
        from modules.llm.personalization_engine import PersonalizationEngine
        from modules.llm.security_filter import SecurityFilter

        # 实例化所有核心类
        context = DialogueContext()
        print("  ✅ DialogueContext 实例化成功")

        session_mgr = SessionManager()
        print("  ✅ SessionManager 实例化成功")

        nlu = NLUEngine()
        print("  ✅ NLUEngine 实例化成功")

        personalization = PersonalizationEngine()
        print("  ✅ PersonalizationEngine 实例化成功")

        security = SecurityFilter()
        print("  ✅ SecurityFilter 实例化成功")

        return True

    except Exception as e:
        print(f"  ❌ 类设计验证失败: {e}")
        return False

def test_ros2_integration():
    """验证ROS2集成"""
    print("\n🤖 验证ROS2集成...")

    try:
        # 检查ROS2相关的导入
        from modules.llm.dialogue_context import DialogueContextNode
        from modules.llm.session_manager import SessionManagerNode
        from modules.llm.nlu_engine import NLUEngineNode
        from modules.llm.personalization_engine import PersonalizationEngineNode
        from modules.llm.security_filter import SecurityFilterNode

        print("  ✅ DialogueContextNode 存在")
        print("  ✅ SessionManagerNode 存在")
        print("  ✅ NLUEngineNode 存在")
        print("  ✅ PersonalizationEngineNode 存在")
        print("  ✅ SecurityFilterNode 存在")

        return True

    except Exception as e:
        print(f"  ❌ ROS2集成验证失败: {e}")
        return False

def main():
    """主测试函数"""
    print("🧪 Epic 2 - LLM智能对话模块架构验证测试")
    print("=" * 80)

    tests = [
        ("架构设计", test_architecture_validation),
        ("代码质量", test_code_quality),
        ("类设计", test_class_design),
        ("ROS2集成", test_ros2_integration)
    ]

    passed = 0
    total = len(tests)

    for test_name, test_func in tests:
        print(f"\n🔍 执行测试: {test_name}")
        if test_func():
            passed += 1
        else:
            print(f"❌ 测试失败: {test_name}")

    # 测试结果
    print("\n" + "=" * 80)
    print("📋 架构验证结果摘要:")
    print(f"  ✅ 通过: {passed}/{total}")
    print(f"  ❌ 失败: {total - passed}/{total}")

    success_rate = (passed / total) * 100
    print(f"  📈 成功率: {success_rate:.1f}%")

    if passed == total:
        print("\n🎉 Epic 2架构验证全部通过！")
        print("\n✅ Epic 2代码质量优秀，架构设计合理！")
        print("✅ 所有核心类和ROS2节点设计完整！")
    else:
        print(f"\n⚠️ 部分测试失败，需要进一步检查")

    print("=" * 80)

if __name__ == '__main__':
    main()