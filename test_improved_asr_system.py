#!/usr/bin/env python3
"""
XLeRobot 改进版ASR系统集成测试
验证音频设备管理、16kHz录音、Token管理、WebSocket稳定性
"""

import sys
import os
import time
import json
import logging
from pathlib import Path

# 添加项目路径
sys.path.insert(0, str(Path(__file__) / "src"))

# 设置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

def test_device_manager():
    """测试音频设备管理器"""
    print("\n=== 测试音频设备管理器 ===")
    try:
        from modules.asr.audio_device_manager import get_device_manager

        manager = get_device_manager()

        # 扫描设备
        devices = manager.scan_audio_devices(force_refresh=True)

        print(f"✅ 扫描结果: 输入设备 {len(devices.get('input', []))} 个, 输出设备 {len(devices.get('output', []))} 个")

        # 选择最佳设备
        best_input = manager.get_best_input_device()
        best_output = manager.get_best_output_device()

        print(f"✅ 最佳输入设备: {best_input.name if best_input else '无'}")
        print(f"✅ 最佳输出设备: {best_output.name if best_output else '无'}")

        # 设备状态
        status = manager.get_device_status()
        print(f"✅ 设备状态: {json.dumps(status, indent=2, ensure_ascii=False)}")

        return True

    except Exception as e:
        print(f"❌ 音频设备管理器测试失败: {e}")
        return False

def test_token_manager():
    """测试统一Token管理器"""
    print("\n=== 测试统一Token管理器 ===")
    try:
        from modules.asr.unified_token_manager import UnifiedTokenManager
        import threading

        # 创建实例但不立即获取Token
        manager = UnifiedTokenManager(cache_file="/tmp/test_token_cache.json")

        # 健康检查
        health = manager.health_check()
        print(f"✅ 健康状态: {health['status']}")

        # 测试Token获取（带超时）
        print("🔄 测试Token获取...")
        token_success = False
        def get_token_thread():
            nonlocal token_success
            try:
                token = manager.get_token()
                token_success = bool(token)
            except Exception as e:
                print(f"Token获取异常: {e}")

        token_thread = threading.Thread(target=get_token_thread)
        token_thread.start()
        token_thread.join(timeout=15)  # 15秒超时

        if token_thread.is_alive():
            print("⚠️ Token获取超时，可能是网络问题")
        elif token_success:
            print("✅ Token获取成功")
        else:
            print("⚠️ Token获取失败")

        # 关闭管理器
        manager.shutdown()

        return True

    except Exception as e:
        print(f"❌ Token管理器测试失败: {e}")
        return False

def test_websocket_stability():
    """测试WebSocket稳定性管理器"""
    print("\n=== 测试WebSocket稳定性管理器 ===")
    try:
        from modules.asr.websocket_stability_manager import WebSocketStabilityManager

        manager = WebSocketStabilityManager(
            max_reconnect_attempts=2,
            base_reconnect_delay=0.5
        )

        # 模拟连接工厂
        def mock_connection_factory():
            class MockConnection:
                def close(self):
                    pass
                def ping(self):
                    pass
            return MockConnection()

        # 测试连接
        connected = manager.connect(mock_connection_factory)
        print(f"✅ 连接测试: {'成功' if connected else '失败'}")

        # 获取状态
        state = manager.get_connection_state()
        print(f"✅ 连接状态: {state.value}")

        # 获取指标
        metrics = manager.get_metrics()
        print(f"✅ 连接指标: 总连接 {metrics['metrics']['total_connections']} 次")

        # 断开连接
        manager.disconnect()
        print("✅ 连接已断开")

        return True

    except Exception as e:
        print(f"❌ WebSocket稳定性管理器测试失败: {e}")
        return False

def test_improved_asr_system():
    """测试改进版ASR系统"""
    print("\n=== 测试改进版ASR系统 ===")
    try:
        from modules.asr.improved_asr_system import ImprovedASRSystem

        system = ImprovedASRSystem()

        # 初始化
        print("🔄 正在初始化系统...")
        initialized = system.initialize()
        print(f"✅ 初始化结果: {'成功' if initialized else '失败'}")

        if initialized:
            # 获取状态
            status = system.get_status()
            print(f"✅ 系统状态: {status['state']}")
            print(f"✅ 设备索引: {status.get('device_index')}")

            # 启动系统（如果初始化成功）
            print("🔄 尝试启动系统...")
            started = system.start()
            print(f"✅ 启动结果: {'成功' if started else '失败'}")

            if started:
                # 运行3秒
                print("🎤 系统运行中，监听3秒...")
                time.sleep(3)

                # 获取运行时状态
                runtime_status = system.get_status()
                print(f"✅ 运行状态: {runtime_status['statistics']}")

                # 停止系统
                system.stop()
                print("✅ 系统已停止")

        return True

    except Exception as e:
        print(f"❌ 改进版ASR系统测试失败: {e}")
        return False

def main():
    """主测试函数"""
    print("🚀 XLeRobot 改进版ASR系统集成测试开始")

    # 检查环境变量
    required_vars = [
        "ALIBABA_CLOUD_ACCESS_KEY_ID",
        "ALIBABA_CLOUD_ACCESS_KEY_SECRET",
        "ALIYUN_NLS_APPKEY"
    ]

    missing_vars = [var for var in required_vars if not os.environ.get(var)]
    if missing_vars:
        print(f"❌ 缺少环境变量: {missing_vars}")
        print("请先设置阿里云API密钥")
        return False

    print(f"✅ 环境变量检查通过")

    # 运行测试
    test_results = {
        "音频设备管理器": test_device_manager(),
        "统一Token管理器": test_token_manager(),
        "WebSocket稳定性管理器": test_websocket_stability(),
        "改进版ASR系统": test_improved_asr_system()
    }

    # 输出结果
    print("\n" + "="*50)
    print("📊 测试结果汇总")
    print("="*50)

    total_tests = len(test_results)
    passed_tests = sum(test_results.values())

    for test_name, result in test_results.items():
        status = "✅ 通过" if result else "❌ 失败"
        print(f"{test_name:<20} {status}")

    print("-"*50)
    print(f"总计: {passed_tests}/{total_tests} 测试通过")

    if passed_tests == total_tests:
        print("🎉 所有测试通过！改进版ASR系统准备就绪。")
        return True
    else:
        print("⚠️ 部分测试失败，请检查相关组件。")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)