#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
XleRobot Epic1 实时语音交互测试
在真实环境中测试"说傻强得到回应"的完整流程
"""

import os
import sys
import asyncio
import logging
import time
import signal
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

class RealVoiceInteractionTester:
    """实时语音交互测试器"""

    def __init__(self):
        self.asr_system = None
        self.running = False
        self.interaction_count = 0
        self.successful_interactions = 0
        self.start_time = None

    def signal_handler(self, signum, frame):
        """信号处理器"""
        print(f"\n🛑 收到停止信号 {signum}，正在关闭...")
        self.running = False
        if self.asr_system:
            self.asr_system.stop()
        self.print_summary()
        sys.exit(0)

    async def initialize_system(self):
        """初始化语音系统"""
        print("🚀 初始化Epic1实时语音交互系统...")
        print("=" * 60)

        try:
            # 创建ASR系统
            self.asr_system = ASRSystem()

            # 初始化系统
            if not self.asr_system.initialize():
                print("❌ ASR系统初始化失败")
                return False

            # 检查麦克风状态
            status = self.asr_system.get_status()
            if not status["microphone_available"]:
                print("⚠️ 警告：麦克风不可用，但将继续测试")
            else:
                print("✅ 麦克风检测成功")

            # 播放启动提示
            print("🔊 播放启动提示...")
            self.asr_system.play_response("傻强系统已启动，请说出唤醒词")

            # 等待播放完成
            await asyncio.sleep(2)

            return True

        except Exception as e:
            print(f"❌ 初始化失败: {e}")
            return False

    async def start_real_interaction_loop(self):
        """启动真实语音交互循环"""
        print("\n🎯 启动实时语音交互...")
        print("💡 使用说明：")
        print("  1. 请清晰地说出'傻强'来唤醒系统")
        print("  2. 唤醒后说出您的命令，如'今日天气'")
        print("  3. 系统会自动识别并回复")
        print("  4. 按 Ctrl+C 停止测试")
        print("\n🎤 开始监听...")
        print("-" * 60)

        self.running = True
        self.start_time = time.time()

        # 注册信号处理器
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        # 启动ASR系统
        if not self.asr_system.start():
            print("❌ ASR系统启动失败")
            return

        # 主交互循环
        last_status_time = time.time()

        try:
            while self.running:
                await asyncio.sleep(1)

                # 每30秒显示一次状态
                current_time = time.time()
                if current_time - last_status_time >= 30:
                    self.print_status()
                    last_status_time = current_time

        except KeyboardInterrupt:
            print("\n🛑 用户停止测试")
        finally:
            self.running = False
            if self.asr_system:
                self.asr_system.stop()

    def print_status(self):
        """打印系统状态"""
        if self.asr_system:
            status = self.asr_system.get_status()
            runtime = time.time() - self.start_time if self.start_time else 0

            print(f"\n📊 系统状态 (运行时间: {runtime:.1f}秒)")
            print(f"  状态: {status['state']}")
            print(f"  总监听次数: {status['stats']['total_listens']}")
            print(f"  唤醒检测次数: {status['stats']['wake_detections']}")
            print(f"  成功识别次数: {status['stats']['successful_recognitions']}")
            print(f"  成功率: {self.calculate_success_rate():.1f}%")

    def calculate_success_rate(self):
        """计算交互成功率"""
        if self.interaction_count == 0:
            return 0.0
        return (self.successful_interactions / self.interaction_count) * 100

    def print_summary(self):
        """打印测试总结"""
        print("\n" + "=" * 60)
        print("📊 Epic1实时语音交互测试总结")
        print("=" * 60)

        if self.start_time:
            runtime = time.time() - self.start_time
            print(f"⏱️  运行时间: {runtime:.1f}秒")

        print(f"📋 总交互次数: {self.interaction_count}")
        print(f"✅ 成功交互次数: {self.successful_interactions}")
        print(f"📈 交互成功率: {self.calculate_success_rate():.1f}%")

        if self.asr_system:
            status = self.asr_system.get_status()
            print(f"\n🎤 系统统计：")
            print(f"  总监听次数: {status['stats']['total_listens']}")
            print(f"  唤醒检测次数: {status['stats']['wake_detections']}")
            print(f"  成功识别次数: {status['stats']['successful_recognitions']}")

        if self.successful_interactions > 0:
            print(f"\n🎉 测试成功！Epic1可以响应'说傻强得到回应'")
        else:
            print(f"\n⚠️  建议检查麦克风配置和音频环境")

    async def run_test(self):
        """运行测试"""
        print("XleRobot Epic1 实时语音交互测试")
        print("测试'说傻强得到回应'的真实场景")
        print()

        # 初始化系统
        if not await self.initialize_system():
            print("❌ 系统初始化失败，无法继续测试")
            return False

        # 启动交互循环
        await self.start_real_interaction_loop()

        return True

async def main():
    """主函数"""
    print("🎙️  准备开始Epic1实时语音交互测试...")
    print("请确保：")
    print("  ✅ 麦克风已连接并工作正常")
    print("  ✅ 音响已连接并可以播放声音")
    print("  ✅ 网络连接正常（阿里云API）")
    print("  ✅ 环境安静，避免干扰")
    print()

    answer = input("是否准备好了？(y/n): ")
    if answer.lower() not in ['y', 'yes', '是']:
        print("测试已取消")
        return

    tester = RealVoiceInteractionTester()
    await tester.run_test()

if __name__ == "__main__":
    asyncio.run(main())