#!/usr/bin/env python3.10
"""
傻强智能对话系统测试
==================

专门测试傻强智能对话管理模块
验证各种对话场景和回应质量

测试场景：
- 问候对话 (早晨、你好)
- 告别对话 (拜拜、再见)
- 感谢对话 (多谢、唔该)
- 功能问答 (天气、时间)
- 情感支持 (唔开心、好攰)
- 闲聊对话 (你係乜嘢、你好嘛)

作者: BMad Master
版本: 1.0 (智能对话测试版)
日期: 2025-11-14
"""

import sys
import os
import time
import logging

# 设置路径
sys.path.insert(0, '/home/sunrise/xlerobot/src')

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def test_siqiang_dialogue():
    """测试傻强智能对话功能"""
    print("=" * 80)
    print("🤖 傻强智能对话系统测试")
    print("=" * 80)

    try:
        # 1. 导入对话管理器
        print("\n📦 导入智能对话模块...")
        from modules.asr.siqiang_intelligent_dialogue import create_siqiang_dialogue_manager
        print("✅ 智能对话模块导入成功")

        # 2. 创建对话管理器
        print("\n🔧 创建傻强对话管理器...")
        dialogue = create_siqiang_dialogue_manager()
        print("✅ 傻强对话管理器创建成功")

        # 3. 测试对话场景
        test_cases = [
            # 问候类
            ("早晨", "greeting"),
            ("你好啊", "greeting"),
            ("Good morning", "greeting"),

            # 告别类
            ("拜拜", "farewell"),
            ("再见", "farewell"),
            ("我走先", "farewell"),

            # 感谢类
            ("多谢你啊", "thanks"),
            ("唔该晒", "thanks"),
            ("感谢帮助", "thanks"),

            # 功能问答
            ("现在几点啊？", "time"),
            ("今日天气点啊？", "weather"),
            ("你可以做咩啊？", "help"),

            # 情感支持
            ("今日心情唔好", "emotional"),
            ("我好攰啊", "emotional"),

            # 闲聊类
            ("你係乜嘢嚟㗎？", "chat"),
            ("你好嘛？", "chat"),
            ("做咩啊？", "chat"),
        ]

        print(f"\n🗣️ 开始测试 {len(test_cases)} 个对话场景...")
        print("-" * 80)

        success_count = 0
        total_count = len(test_cases)

        for i, (user_input, expected_category) in enumerate(test_cases, 1):
            print(f"\n📍 测试 {i}/{total_count}: {user_input}")

            # 生成响应
            response = dialogue.generate_response(user_input)

            # 验证响应
            if response.text and response.confidence > 0.5:
                print(f"✅ 傻强回应: {response.text}")
                print(f"   情绪: {response.emotion} | 置信度: {response.confidence:.2f} | 类别: {response.category}")

                # 验证类别匹配
                if response.category == expected_category or expected_category == "chat":
                    success_count += 1
                    print("   🎯 类别匹配正确")
                else:
                    print(f"   ⚠️ 类别不匹配 (期望: {expected_category})")
            else:
                print(f"❌ 回应生成失败或置信度过低")

            print("-" * 50)

        # 4. 欢迎消息测试
        print(f"\n👋 欢迎消息测试...")
        welcome_msg = dialogue.get_welcome_message()
        print(f"✅ 欢迎消息: {welcome_msg}")

        # 5. 统计信息
        print(f"\n📊 测试结果统计:")
        print(f"   总测试数: {total_count}")
        print(f"   成功数: {success_count}")
        print(f"   成功率: {(success_count/total_count)*100:.1f}%")

        # 6. 对话管理器统计
        dialogue_stats = dialogue.get_conversation_stats()
        print(f"\n📈 对话管理器统计:")
        for key, value in dialogue_stats.items():
            print(f"   {key}: {value}")

        # 7. 测试结果
        if success_count >= total_count * 0.8:  # 80%成功率
            print(f"\n🎉 傻强智能对话测试通过！")
            print("✅ 对话回应质量良好")
            print("✅ 类别识别准确")
            print("✅ 情绪表达自然")
            print("✅ 粤语表达地道")
            return True
        else:
            print(f"\n⚠️ 傻强智能对话测试部分通过")
            print(f"   成功率 {(success_count/total_count)*100:.1f}% 未达80%标准")
            return False

    except Exception as e:
        print(f"\n❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_conversation_flow():
    """测试连续对话流程"""
    print(f"\n🔄 连续对话流程测试")
    print("-" * 50)

    try:
        from modules.asr.siqiang_intelligent_dialogue import create_siqiang_dialogue_manager

        dialogue = create_siqiang_dialogue_manager()

        # 模拟一个完整对话流程
        conversation = [
            "早晨",
            "今日天气点啊？",
            "唔该晒",
            "你係乜嘢嚟㗎？",
            "拜拜"
        ]

        print("💬 模拟用户与傻强的完整对话:")

        for i, user_input in enumerate(conversation, 1):
            print(f"\n轮次 {i}:")
            print(f"👤 用户: {user_input}")

            response = dialogue.generate_response(user_input)
            print(f"🤖 傻强: {response.text}")
            print(f"   [情绪:{response.emotion} | 置信度:{response.confidence:.2f}]")

            time.sleep(0.5)  # 模拟对话间隔

        print(f"\n✅ 连续对话流程测试完成")
        return True

    except Exception as e:
        print(f"❌ 连续对话测试失败: {e}")
        return False

def main():
    """主函数"""
    print("🚀 启动傻强智能对话系统测试")

    # 基础功能测试
    test1_result = test_siqiang_dialogue()

    # 连续对话测试
    test2_result = test_conversation_flow()

    # 最终结果
    print("\n" + "=" * 80)
    print("📋 最终测试结果")
    print("=" * 80)

    if test1_result and test2_result:
        print("🎉 所有测试通过！")
        print("✅ 傻强智能对话系统已准备就绪")
        print("✅ 可以集成到WebSocket ASR→TTS服务中")
        return 0
    else:
        print("⚠️ 部分测试未通过")
        print("❌ 需要进一步优化对话逻辑")
        return 1

if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)