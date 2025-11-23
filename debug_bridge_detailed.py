#!/usr/bin/env python3.10
"""
详细的桥接节点调试版本 - 捕获完整的异常信息
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

# 模拟ROS2环境，但不使用ROS2
class MockNode:
    """模拟ROS2节点，排除ROS2本身的问题"""

    def __init__(self):
        self.asr_system = None
        self.asr_thread = None
        self.is_initialized = False
        self.start_time = time.time()

    def info(self, msg):
        print(f"[INFO] [mock_node]: {msg}")

    def error(self, msg):
        print(f"[ERROR] [mock_node]: {msg}")

    def warning(self, msg):
        print(f"[WARNING] [mock_node]: {msg}")

def test_asr_with_detailed_error_capture():
    """测试ASR系统并捕获所有错误详情"""

    print("=" * 60)
    print("详细ASR错误调试 - 排除ROS2影响")
    print("=" * 60)

    node = MockNode()

    try:
        # 模拟桥接节点的完整初始化流程
        node.info('🔄 ASR桥接节点已创建，等待ASR系统初始化...')

        # 导入ASR系统
        from modules.asr.asr_system import ASRSystem
        node.info('🚀 开始初始化ASR系统...')

        # 创建ASRSystem实例
        node.asr_system = ASRSystem()

        # 注入结果回调函数
        node.asr_system.result_callback = lambda result: node.info(f'📢 收到ASR结果: {result}')

        # 初始化ASR系统
        if node.asr_system.initialize():
            node.is_initialized = True
            node.info('✅ ASR系统初始化成功')

            # 在独立线程启动ASR监听循环
            node.asr_thread = threading.Thread(
                target=run_asr_loop_with_detailed_errors,
                args=(node,),
                daemon=True
            )
            node.asr_thread.start()
            node.info('🎤 ASR监听线程已启动')

            # 等待5秒，观察运行状态
            node.info('⏳ 等待5秒观察系统运行...')
            time.sleep(5)

            # 检查线程状态
            if node.asr_thread.is_alive():
                node.info('✅ ASR监听线程仍在运行')
            else:
                node.warning('⚠️ ASR监听线程已停止')

            # 停止ASR系统
            if node.asr_system:
                node.asr_system.stop()
                node.info('✅ ASR系统已停止')

        else:
            node.error('❌ ASR系统初始化失败')

    except Exception as e:
        node.error(f'❌ ASR系统初始化异常: {e}')
        traceback.print_exc()

def run_asr_loop_with_detailed_errors(node):
    """在独立线程运行ASR监听循环，捕获详细错误信息"""

    print(f"🎧 开始ASR监听循环 (线程ID: {threading.get_ident()})...")

    try:
        # 检查当前线程是否有事件循环
        try:
            current_loop = asyncio.get_running_loop()
            print(f"⚠️ 当前线程已有事件循环: {current_loop}")
        except RuntimeError:
            print("✅ 当前线程没有事件循环，可以创建新的")

        # 尝试不同的启动方法
        print("\n🔍 方法1: 直接调用 start() 方法")
        success = node.asr_system.start()
        print(f"方法1结果: {success}")

        if success:
            print("✅ ASR系统启动成功")

            # 等待系统运行
            print("⏳ 等待ASR系统运行...")
            count = 0
            while node.asr_system.is_running and count < 50:  # 最多5秒
                time.sleep(0.1)
                count += 1

                if count % 10 == 0:  # 每秒打印一次状态
                    print(f"  运行状态: {node.asr_system.is_running} (已运行{count*0.1:.1f}秒)")

            print(f"✅ 最终状态: {node.asr_system.is_running}")
        else:
            print("❌ ASR系统启动失败")

    except Exception as e:
        print(f"❌ ASR监听循环异常: {e}")
        print(f"异常类型: {type(e)}")
        print(f"异常模块: {e.__class__.__module__}")

        # 打印完整的堆栈跟踪
        print("\n📋 完整异常堆栈:")
        traceback.print_exc()

        # 尝试获取更多异常信息
        if hasattr(e, '__cause__') and e.__cause__:
            print(f"\n🔗 原因异常: {e.__cause__}")
        if hasattr(e, '__context__') and e.__context__:
            print(f"\n🔗 上下文异常: {e.__context__}")

    finally:
        print("🛑 ASR监听循环已结束")

def test_alternative_startup_methods():
    """测试不同的ASR启动方法"""

    print("\n" + "=" * 60)
    print("测试替代启动方法")
    print("=" * 60)

    try:
        from modules.asr.asr_system import ASRSystem

        print("🔍 测试方法2: 直接调用 _start_listening_thread")
        asr_system = ASRSystem()

        if asr_system.initialize():
            print("✅ 初始化成功，尝试直接启动监听线程...")

            # 直接调用监听线程方法，绕过start()方法
            asr_system._start_listening_thread()

            # 等待3秒
            time.sleep(3)

            # 检查状态
            print(f"运行状态: {asr_system.is_running}")

            # 停止系统
            asr_system.stop()
            print("✅ 系统已停止")
        else:
            print("❌ 初始化失败")

    except Exception as e:
        print(f"❌ 替代方法异常: {e}")
        traceback.print_exc()

def main():
    """主函数"""
    print("XLeRobot ASR详细错误调试工具")
    print("目标: 精确定位asyncio错误的来源")

    # 测试1: 标准方法，但排除ROS2影响
    test_asr_with_detailed_error_capture()

    # 测试2: 尝试替代启动方法
    test_alternative_startup_methods()

    print("\n" + "=" * 60)
    print("调试完成")
    print("=" * 60)

if __name__ == '__main__':
    main()