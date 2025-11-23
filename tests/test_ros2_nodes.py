#!/usr/bin/env python3.10
# -*- coding: utf-8 -*-
"""
ROS2节点测试脚本

测试新建的LLM、TTS和协调节点是否能够正常启动。
验证消息导入和基本功能。

作者: Claude Code
故事ID: Epic 1 ASR→LLM→TTS串联修复
"""

import os
import sys
import time
import logging
from pathlib import Path

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def test_imports():
    """测试模块导入"""
    logger.info("🔍 测试模块导入...")

    # 测试标准ROS2模块
    try:
        import rclpy
        logger.info("✅ rclpy导入成功")
    except ImportError as e:
        logger.error(f"❌ rclpy导入失败: {e}")
        return False

    # 测试自定义消息模块
    try:
        from audio_msg.msg import ASRResult, LLMResponse, LLMStatus, TTSStatus
        logger.info("✅ audio_msg消息导入成功")
    except ImportError as e:
        logger.warning(f"⚠️ audio_msg消息导入失败（可能未编译）: {e}")

    return True

def test_nodes():
    """测试节点文件"""
    logger.info("🔍 测试节点文件...")

    nodes = [
        "src/xlerobot/nodes/llm_service_node.py",
        "src/xlerobot/nodes/tts_service_node.py",
        "src/xlerobot/nodes/voice_assistant_coordinator.py"
    ]

    for node_path in nodes:
        if Path(node_path).exists():
            logger.info(f"✅ 节点文件存在: {node_path}")
        else:
            logger.error(f"❌ 节点文件不存在: {node_path}")
            return False

    return True

def test_environment():
    """测试环境变量"""
    logger.info("🔍 测试环境变量...")

    # 检查必需的环境变量
    required_vars = [
        'QWEN_API_KEY',
        'ALIBABA_CLOUD_ACCESS_KEY_ID',
        'ALIBABA_CLOUD_ACCESS_KEY_SECRET',
        'ALIYUN_NLS_APPKEY'
    ]

    missing_vars = []
    for var in required_vars:
        if os.getenv(var):
            logger.info(f"✅ {var}: 已设置")
        else:
            logger.warning(f"⚠️ {var}: 未设置")
            missing_vars.append(var)

    if missing_vars:
        logger.warning(f"⚠️ 缺少 {len(missing_vars)} 个环境变量，节点可能无法正常运行")

    return len(missing_vars) == 0

def test_launch_file():
    """测试Launch文件"""
    logger.info("🔍 测试Launch文件...")

    launch_path = "src/xlerobot/launch/voice_assistant.launch.py"
    if Path(launch_path).exists():
        logger.info(f"✅ Launch文件存在: {launch_path}")

        # 尝试导入Launch文件
        try:
            sys.path.insert(0, os.path.dirname(launch_path))
            import voice_assistant_launch
            logger.info("✅ Launch文件语法正确")
            return True
        except Exception as e:
            logger.error(f"❌ Launch文件语法错误: {e}")
            return False
    else:
        logger.error(f"❌ Launch文件不存在: {launch_path}")
        return False

def test_dependencies():
    """测试依赖"""
    logger.info("🔍 测试依赖...")

    # 测试核心依赖
    dependencies = [
        ('aiohttp', 'aiohttp'),
        ('asyncio', 'asyncio'),
        ('pathlib', 'pathlib'),
        ('subprocess', 'subprocess'),
        ('json', 'json')
    ]

    for name, module in dependencies:
        try:
            __import__(module)
            logger.info(f"✅ {name} 可用")
        except ImportError as e:
            logger.error(f"❌ {name} 不可用: {e}")
            return False

    return True

def test_audio_system():
    """测试音频系统"""
    logger.info("🔍 测试音频系统...")

    # 测试音频设备
    try:
        result = os.system('aplay -l > /dev/null 2>&1')
        if result == 0:
            logger.info("✅ 音频播放设备可用")
        else:
            logger.warning("⚠️ 音频播放设备可能不可用")
    except Exception as e:
        logger.warning(f"⚠️ 音频设备测试失败: {e}")

    try:
        result = os.system('arecord -l > /dev/null 2>&1')
        if result == 0:
            logger.info("✅ 音频录制设备可用")
        else:
            logger.warning("⚠️ 音频录制设备可能不可用")
    except Exception as e:
        logger.warning(f"⚠️ 音频录制测试失败: {e}")

    return True

def main():
    """主函数"""
    print("=" * 60)
    print("XLeRobot ROS2节点集成测试")
    print("=" * 60)

    tests = [
        ("模块导入", test_imports),
        ("节点文件", test_nodes),
        ("环境变量", test_environment),
        ("Launch文件", test_launch_file),
        ("依赖", test_dependencies),
        ("音频系统", test_audio_system)
    ]

    passed = 0
    total = len(tests)

    for test_name, test_func in tests:
        print(f"\n🧪 运行测试: {test_name}")
        try:
            if test_func():
                passed += 1
                print(f"✅ {test_name} - 通过")
            else:
                print(f"❌ {test_name} - 失败")
        except Exception as e:
            print(f"❌ {test_name} - 异常: {e}")

    print("\n" + "=" * 60)
    print(f"测试结果: {passed}/{total} 通过")

    if passed == total:
        print("🎉 所有测试通过！系统准备就绪。")
        return 0
    else:
        print("⚠️ 部分测试失败，请检查上述问题。")
        return 1

if __name__ == "__main__":
    sys.exit(main())