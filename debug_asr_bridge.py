#!/usr/bin/env python3.10
"""
调试ASR桥接节点的asyncio错误
"""

import os
import sys
import time
import asyncio
import logging
import threading
import traceback
from typing import Optional

# 添加项目路径到Python路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

# 配置详细日志
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

try:
    # 导入ASR系统
    from modules.asr.asr_system import ASRSystem
    print("✅ ASR系统导入成功")
except ImportError as e:
    print(f"❌ 导入ASRSystem失败: {e}")
    sys.exit(1)

class DebugASRBridge:
    """调试版本的ASR桥接节点"""

    def __init__(self):
        self.asr_system = None
        self.asr_thread = None
        self.is_initialized = False
        self.start_time = time.time()

    def test_asr_directly(self):
        """直接测试ASR系统"""
        try:
            print("🔍 测试1: 直接测试ASR系统")

            # 创建ASRSystem实例
            self.asr_system = ASRSystem()
            print("✅ ASR系统实例创建成功")

            # 初始化ASR系统
            if self.asr_system.initialize():
                self.is_initialized = True
                print("✅ ASR系统初始化成功")

                # 启动ASR系统
                print("🚀 启动ASR系统...")
                success = self.asr_system.start()

                if success:
                    print("✅ ASR系统启动成功")
                    print(f"✅ 运行状态: {self.asr_system.is_running}")

                    # 等待5秒，观察系统运行
                    print("⏳ 等待5秒观察系统运行...")
                    for i in range(50):
                        if not self.asr_system.is_running:
                            print(f"❌ ASR系统在第{i*0.1:.1f}秒停止运行")
                            break
                        time.sleep(0.1)

                    print(f"✅ 最终状态: {self.asr_system.is_running}")

                    # 停止系统
                    self.asr_system.stop()
                    print("✅ ASR系统已停止")

                else:
                    print("❌ ASR系统启动失败")

            else:
                print("❌ ASR系统初始化失败")

        except Exception as e:
            print(f"❌ 直接测试ASR系统异常: {e}")
            traceback.print_exc()

    def test_asr_in_thread(self):
        """在线程中测试ASR系统（模拟桥接节点）"""
        try:
            print("\n🔍 测试2: 在独立线程中测试ASR系统")

            # 重新创建ASRSystem实例
            self.asr_system = ASRSystem()

            # 注入结果回调函数
            self.asr_system.result_callback = self.on_asr_result

            # 初始化ASR系统
            if self.asr_system.initialize():
                self.is_initialized = True
                print("✅ ASR系统初始化成功")

                # 在独立线程启动ASR监听循环（完全模拟桥接节点）
                print("🧵 启动ASR监听线程...")
                self.asr_thread = threading.Thread(
                    target=self._run_asr_loop,
                    daemon=True
                )
                self.asr_thread.start()
                print("✅ ASR监听线程已启动")

                # 等待5秒，观察线程运行
                print("⏳ 等待5秒观察线程运行...")
                time.sleep(5)

                # 检查线程状态
                if self.asr_thread.is_alive():
                    print("✅ ASR监听线程仍在运行")
                else:
                    print("❌ ASR监听线程已停止")

                # 停止ASR系统
                if self.asr_system:
                    self.asr_system.stop()
                    print("✅ ASR系统已停止")

            else:
                print("❌ ASR系统初始化失败")

        except Exception as e:
            print(f"❌ 线程测试ASR系统异常: {e}")
            traceback.print_exc()

    def _run_asr_loop(self):
        """在独立线程运行ASR监听循环（完全复制桥接节点代码）"""
        try:
            print("🎧 开始ASR监听循环...")

            # 直接运行ASR系统（不使用额外事件循环）
            success = self.asr_system.start()

            if success:
                print("✅ ASR系统启动成功")

                # 等待ASR系统运行（阻塞直到停止）
                while self.asr_system.is_running:
                    time.sleep(0.1)
            else:
                print("❌ ASR系统启动失败")

        except Exception as e:
            print(f"❌ ASR监听循环异常: {e}")
            traceback.print_exc()
        finally:
            print("🛑 ASR监听循环已结束")

    def on_asr_result(self, result):
        """ASR结果回调"""
        try:
            print(f"📢 收到ASR结果: {result}")
        except Exception as e:
            print(f"❌ 处理ASR结果回调失败: {e}")

def main():
    """主函数"""
    print("=" * 60)
    print("XLeRobot ASR桥接节点调试工具")
    print("=" * 60)

    debug_bridge = DebugASRBridge()

    # 测试1: 直接测试ASR系统
    debug_bridge.test_asr_directly()

    # 测试2: 在线程中测试ASR系统（模拟桥接节点）
    debug_bridge.test_asr_in_thread()

    print("\n" + "=" * 60)
    print("调试测试完成")
    print("=" * 60)

if __name__ == '__main__':
    main()