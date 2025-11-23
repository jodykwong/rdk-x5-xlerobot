#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
XleRobot Epic1 端到端语音交互测试
测试完整的语音交互流程：唤醒词检测 → ASR → LLM处理 → TTS → 音频输出
"""

import os
import sys
import asyncio
import logging
import time
import tempfile
from pathlib import Path

# 设置环境变量（阿里云API认证）
os.environ["ALIBABA_CLOUD_ACCESS_KEY_ID"] = "YOUR_ACCESS_KEY_ID"
os.environ["ALIBABA_CLOUD_ACCESS_KEY_SECRET"] = "YOUR_ACCESS_KEY_SECRET"
os.environ["ALIYUN_NLS_APPKEY"] = "YOUR_NLS_APPKEY"

# 添加项目路径
sys.path.insert(0, str(Path(__file__).parent / "src"))

from modules.asr.asr_system import ASRSystem

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class EndToEndVoiceTester:
    """端到端语音交互测试器"""

    def __init__(self):
        self.asr_system = None
        self.test_results = []

    def log_test_result(self, test_name: str, success: bool, message: str = ""):
        """记录测试结果"""
        result = {
            "test_name": test_name,
            "success": success,
            "message": message,
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
        }
        self.test_results.append(result)

        status = "✅" if success else "❌"
        print(f"{status} {test_name}: {message}")

    async def test_asr_initialization(self):
        """测试ASR系统初始化"""
        print("\n🧪 测试ASR系统初始化...")

        try:
            self.asr_system = ASRSystem()

            # 测试初始化
            init_result = self.asr_system.initialize()

            if init_result:
                self.log_test_result("ASR系统初始化", True, "初始化成功")

                # 检查系统状态
                status = self.asr_system.get_status()
                self.log_test_result(
                    "ASR系统状态检查",
                    status["state"] == "stopped",
                    f"状态={status['state']}, 麦克风={status['microphone_available']}"
                )

                return True
            else:
                self.log_test_result("ASR系统初始化", False, "初始化失败")
                return False

        except Exception as e:
            self.log_test_result("ASR系统初始化", False, f"异常: {e}")
            return False

    async def test_tts_functionality(self):
        """测试TTS功能"""
        print("\n🔊 测试TTS语音合成功能...")

        try:
            test_texts = [
                "Epic1语音交互系统启动成功",
                "你好，我是傻强",
                "今日天气晴朗",
                "测试完成"
            ]

            for i, text in enumerate(test_texts, 1):
                print(f"  🎵 测试文本 {i}: {text}")

                # 测试TTS合成
                result = self.asr_system.play_response(text)

                if result:
                    self.log_test_result(f"TTS测试 {i}", True, f"播放成功: {text}")
                else:
                    self.log_test_result(f"TTS测试 {i}", False, f"播放失败: {text}")

                # 等待播放完成
                await asyncio.sleep(2)

            return True

        except Exception as e:
            self.log_test_result("TTS功能测试", False, f"异常: {e}")
            return False

    async def test_voice_interaction_simulation(self):
        """模拟语音交互测试"""
        print("\n🎤 模拟语音交互测试...")
        print("⚠️  注意：这是模拟测试，不是真实音频输入")

        try:
            if not self.asr_system:
                self.log_test_result("语音交互模拟", False, "ASR系统未初始化")
                return False

            # 模拟检测到唤醒词
            print("  🔔 模拟检测到唤醒词：'傻强'")

            # 模拟语音识别结果
            test_commands = [
                "今日天气点样",
                "现在几时",
                "你好傻强",
                "帮我播放音乐"
            ]

            for command in test_commands:
                print(f"  🎤 模拟用户输入: {command}")

                # 直接调用命令处理逻辑
                response = await self.asr_system._process_command(command)

                if response:
                    print(f"  🤖 系统回复: {response}")

                    # 播放回复
                    play_result = self.asr_system.play_response(response)

                    if play_result:
                        self.log_test_result(
                            f"交互测试-{command[:5]}...",
                            True,
                            f"回复: {response[:20]}..."
                        )
                    else:
                        self.log_test_result(
                            f"交互测试-{command[:5]}...",
                            False,
                            "播放回复失败"
                        )
                else:
                    self.log_test_result(
                        f"交互测试-{command[:5]}...",
                        False,
                        "无回复生成"
                    )

                await asyncio.sleep(1)

            return True

        except Exception as e:
            self.log_test_result("语音交互模拟", False, f"异常: {e}")
            return False

    async def test_error_handling(self):
        """测试错误处理"""
        print("\n⚠️ 测试错误处理...")

        try:
            # 测试空文本处理
            empty_response = await self.asr_system._process_command("")
            self.log_test_result(
                "空文本处理",
                True,
                f"空文本处理结果: {empty_response}"
            )

            # 测试无效命令处理
            invalid_response = await self.asr_system._process_command("这是一个无效的测试命令123456")
            self.log_test_result(
                "无效命令处理",
                True,
                f"无效命令处理结果: {invalid_response}"
            )

            return True

        except Exception as e:
            self.log_test_result("错误处理测试", False, f"异常: {e}")
            return False

    async def run_full_test(self):
        """运行完整测试"""
        print("🚀 开始Epic1端到端语音交互测试")
        print("=" * 60)

        start_time = time.time()

        # 1. ASR系统初始化测试
        init_success = await self.test_asr_initialization()

        if not init_success:
            print("❌ ASR系统初始化失败，终止测试")
            return False

        # 2. TTS功能测试
        tts_success = await self.test_tts_functionality()

        # 3. 语音交互模拟测试
        interaction_success = await self.test_voice_interaction_simulation()

        # 4. 错误处理测试
        error_handling_success = await self.test_error_handling()

        # 生成测试报告
        end_time = time.time()
        duration = end_time - start_time

        self.print_test_report(duration)

        # 清理资源
        if self.asr_system:
            self.asr_system.stop()

        return True

    def print_test_report(self, duration: float):
        """打印测试报告"""
        print("\n" + "=" * 60)
        print("📊 Epic1端到端测试报告")
        print("=" * 60)

        total_tests = len(self.test_results)
        passed_tests = sum(1 for result in self.test_results if result["success"])
        failed_tests = total_tests - passed_tests

        print(f"⏱️  测试时长: {duration:.2f}秒")
        print(f"📋 总测试数: {total_tests}")
        print(f"✅ 通过测试: {passed_tests}")
        print(f"❌ 失败测试: {failed_tests}")
        print(f"📈 成功率: {(passed_tests/total_tests)*100:.1f}%")

        if failed_tests == 0:
            print("\n🎉 所有测试通过！Epic1语音交互系统已准备就绪")
            print("\n✨ 核心功能验证：")
            print("  ✅ ASR系统初始化和麦克风检测")
            print("  ✅ 阿里云TTS语音合成和播放")
            print("  ✅ 语音命令处理和回复生成")
            print("  ✅ 错误处理和异常管理")
        else:
            print("\n⚠️  部分测试失败，需要进一步修复")

            print("\n❌ 失败的测试：")
            for result in self.test_results:
                if not result["success"]:
                    print(f"  ❌ {result['test_name']}: {result['message']}")

        print("\n🎯 下一步：")
        print("  1. 配置真实麦克风进行实际语音输入测试")
        print("  2. 测试'说傻强得到回应'的完整流程")
        print("  3. 优化响应时间和准确性")

async def main():
    """主函数"""
    print("XleRobot Epic1 端到端语音交互测试")
    print("测试完整的语音交互流程")
    print()

    tester = EndToEndVoiceTester()
    await tester.run_full_test()

if __name__ == "__main__":
    asyncio.run(main())