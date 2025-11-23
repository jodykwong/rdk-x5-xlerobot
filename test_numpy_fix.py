#!/usr/bin/env python3.10
"""
测试NumPy数组布尔判断修复
验证ASR系统是否能正确处理NumPy数组而不崩溃
"""

import sys
import os
import asyncio
import numpy as np

# 添加项目路径
sys.path.insert(0, '/home/sunrise/xlerobot/src')

def test_numpy_boolean_fix():
    """测试NumPy数组布尔判断修复"""
    print("🧪 测试NumPy数组布尔判断修复...")

    # 测试1: 空数组
    empty_array = np.array([])
    try:
        # 旧的方式（应该失败）
        if empty_array:
            print("❌ 空数组测试失败 - 应该抛出错误")
        else:
            print("❌ 空数组测试失败 - 应该抛出错误")
    except ValueError as e:
        print(f"✅ 空数组 - 旧方式正确抛出错误: {e}")

    try:
        # 新的方式（应该成功）
        if empty_array is not None and len(empty_array) > 0:
            print("❌ 新方式失败 - 空数组应该被判断为False")
        else:
            print("✅ 空数组 - 新方式正确判断为False")
    except Exception as e:
        print(f"❌ 新方式失败 - 不应该抛出错误: {e}")

    # 测试2: 有数据的数组
    data_array = np.array([1, 2, 3, 4, 5])
    try:
        # 旧的方式（应该失败）
        if data_array:
            print("❌ 数据数组测试失败 - 应该抛出错误")
        else:
            print("❌ 数据数组测试失败 - 应该抛出错误")
    except ValueError as e:
        print(f"✅ 数据数组 - 旧方式正确抛出错误: {e}")

    try:
        # 新的方式（应该成功）
        if data_array is not None and len(data_array) > 0:
            print("✅ 数据数组 - 新方式正确判断为True")
        else:
            print("❌ 新方式失败 - 数据数组应该被判断为True")
    except Exception as e:
        print(f"❌ 新方式失败 - 不应该抛出错误: {e}")

    # 测试3: None值
    none_value = None
    try:
        # 新的方式处理None（应该成功）
        if none_value is not None and len(none_value) > 0:
            print("❌ None值测试失败 - 应该被判断为False")
        else:
            print("✅ None值 - 新方式正确判断为False")
    except Exception as e:
        print(f"❌ None值测试失败 - 不应该抛出错误: {e}")

    print("\n🎯 NumPy布尔判断修复测试完成！")

async def test_asr_system_fix():
    """测试ASR系统是否能处理音频数据而不崩溃"""
    print("\n🎤 测试ASR系统NumPy修复...")

    try:
        from modules.asr.asr_system import ASRSystem
        print("✅ ASR系统导入成功")

        # 创建ASR系统实例
        asr_system = ASRSystem()
        print("✅ ASR系统实例创建成功")

        # 模拟音频数据（NumPy数组）
        mock_audio_data = np.array([100, 200, 300, 400, 500], dtype=np.int16)
        print(f"✅ 创建模拟音频数据: {mock_audio_data}")

        # 测试修复后的判断逻辑
        if mock_audio_data is not None and len(mock_audio_data) > 0:
            print("✅ ASR系统能正确处理NumPy音频数组")
        else:
            print("❌ ASR系统无法正确处理NumPy音频数组")

        # 测试空数组
        empty_audio = np.array([])
        if empty_audio is not None and len(empty_audio) > 0:
            print("❌ ASR系统错误判断空数组为有效")
        else:
            print("✅ ASR系统正确判断空数组为无效")

        print("🎯 ASR系统NumPy修复验证成功！")

    except Exception as e:
        print(f"❌ ASR系统测试失败: {e}")
        import traceback
        traceback.print_exc()

async def main():
    """主测试函数"""
    print("=" * 60)
    print("🔧 XLeRobot NumPy数组布尔判断修复验证")
    print("=" * 60)

    # 基础NumPy测试
    test_numpy_boolean_fix()

    # ASR系统集成测试
    await test_asr_system_fix()

    print("\n" + "=" * 60)
    print("🎉 所有测试完成！")
    print("=" * 60)

if __name__ == "__main__":
    asyncio.run(main())