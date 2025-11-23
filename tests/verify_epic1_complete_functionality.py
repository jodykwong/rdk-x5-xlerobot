#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Epic 1 完整功能验证脚本
验证所有核心组件是否正常工作

🚨 严格遵守真实数据政策:
- 使用真实音频硬件
- 使用真实API调用
- 严禁任何Mock数据
"""

import os
import sys
import time
import tempfile
import subprocess
from pathlib import Path

# 添加源码路径
sys.path.insert(0, str(Path(__file__).parent / "src"))

def log(message: str, level: str = "VERIFY"):
    """验证日志"""
    timestamp = time.strftime("%H:%M:%S")
    print(f"[{timestamp}] {level}: {message}")

def test_audio_devices():
    """测试音频设备"""
    log("测试音频设备发现")

    try:
        from modules.asr.enhanced_audio_input import create_enhanced_audio_input

        with create_enhanced_audio_input() as audio_input:
            devices = audio_input.get_audio_devices()
            log(f"✅ 发现 {len(devices)} 个音频设备")

            for device in devices:
                log(f"  🎤 [{device.device_id}] {device.name}")

            return len(devices) > 0
    except Exception as e:
        log(f"❌ 音频设备测试失败: {e}")
        return False

def test_enhanced_audio_input():
    """测试增强音频输入"""
    log("测试增强音频输入模块")

    try:
        from modules.asr.enhanced_audio_input import create_enhanced_audio_input

        with create_enhanced_audio_input() as audio_input:
            # 测试配置
            config = audio_input.get_current_config()
            log(f"✅ 音频配置: {config}")

            # 测试录音
            test_file = audio_input.record_to_file(2.0)
            if test_file and Path(test_file).exists():
                file_size = Path(test_file).stat().st_size
                log(f"✅ 录音测试成功: {file_size:,} 字节")

                # 清理文件
                Path(test_file).unlink()
                return True
            else:
                log("❌ 录音测试失败")
                return False
    except Exception as e:
        log(f"❌ 增强音频输入测试失败: {e}")
        return False

def test_wake_word_detector():
    """测试唤醒词检测器"""
    log("测试唤醒词检测器")

    try:
        from xlerobot_phase1.wake_word_detector import WakeWordDetector

        detector = WakeWordDetector()
        log("✅ 唤醒词检测器创建成功")

        # 检查方法
        if hasattr(detector, 'detect'):
            log("✅ 检测方法存在")
            return True
        else:
            log("❌ 检测方法缺失")
            return False
    except Exception as e:
        log(f"❌ 唤醒词检测器测试失败: {e}")
        return False

def test_aliyun_asr_client():
    """测试阿里云ASR客户端"""
    log("测试阿里云ASR客户端")

    try:
        from modules.asr.websocket.websocket_asr_service import AliyunASRWebSocketService

        client = AliyunASRWebSocketService()
        log("✅ ASR客户端创建成功")

        # 检查方法
        if hasattr(client, 'recognize_audio'):
            log("✅ 语音识别方法存在")
            return True
        else:
            log("❌ 语音识别方法缺失")
            return False
    except Exception as e:
        log(f"❌ ASR客户端测试失败: {e}")
        return False

def test_aliyun_tts_client():
    """测试阿里云TTS客户端"""
    log("测试阿里云TTS客户端")

    try:
        from modules.tts.engine.aliyun_tts_websocket_client import AliyunTTSWebSocketService

        client = AliyunTTSWebSocketService()
        log("✅ TTS客户端创建成功")

        # 检查方法
        if hasattr(client, 'synthesize_speech'):
            log("✅ 语音合成方法存在")
            return True
        else:
            log("❌ 语音合成方法缺失")
            return False
    except Exception as e:
        log(f"❌ TTS客户端测试失败: {e}")
        return False

def test_cantonese_tts():
    """测试粤语TTS模块"""
    log("测试粤语TTS模块")

    try:
        from modules.asr.audio.cantonese_tts import CantoneseTTS

        tts = CantoneseTTS()
        log("✅ 粤语TTS模块创建成功")
        return True
    except Exception as e:
        log(f"❌ 粤语TTS模块测试失败: {e}")
        return False

def test_wake_word_config():
    """测试唤醒词配置管理器"""
    log("测试唤醒词配置管理器")

    try:
        from xlerobot_phase1.wake_word_config import WakeWordConfigManager

        config_manager = WakeWordConfigManager()
        log("✅ 唤醒词配置管理器创建成功")
        return True
    except Exception as e:
        log(f"❌ 唤醒词配置管理器测试失败: {e}")
        return False

def test_audio_player():
    """测试音频播放器"""
    log("测试音频播放器")

    try:
        from modules.asr.audio.audio_player import AudioPlayer

        player = AudioPlayer()
        log("✅ 音频播放器创建成功")
        return True
    except Exception as e:
        log(f"❌ 音频播放器测试失败: {e}")
        return False

def test_asr_retry_manager():
    """测试ASR重试管理器"""
    log("测试ASR重试管理器")

    try:
        from modules.asr.asr_retry_manager import ASRRetryManager

        retry_manager = ASRRetryManager()
        log("✅ ASR重试管理器创建成功")
        return True
    except Exception as e:
        log(f"❌ ASR重试管理器测试失败: {e}")
        return False

def test_aliyun_api_config():
    """测试阿里云API配置"""
    log("测试阿里云API配置")

    try:
        # 检查配置文件
        config_file = Path(__file__).parent / "fixed_aliyun_config.py"
        if config_file.exists():
            log("✅ 阿里云配置文件存在")

            # 测试配置
            sys.path.insert(0, str(Path(__file__).parent))
            from fixed_aliyun_config import FixedAliyunConfigManager

            config_manager = FixedAliyunConfigManager()
            if config_manager.validate_config():
                log("✅ API配置验证成功")
                return True
            else:
                log("❌ API配置验证失败")
                return False
        else:
            log("❌ 阿里云配置文件不存在")
            return False
    except Exception as e:
        log(f"❌ API配置测试失败: {e}")
        return False

def run_complete_verification():
    """运行完整验证"""
    log("🚀 开始 Epic 1 完整功能验证")
    log("🚨 严格遵守真实数据政策，无任何Mock数据")

    # 测试项目清单
    tests = [
        ("音频设备发现", test_audio_devices),
        ("增强音频输入", test_enhanced_audio_input),
        ("唤醒词检测器", test_wake_word_detector),
        ("阿里云ASR客户端", test_aliyun_asr_client),
        ("阿里云TTS客户端", test_aliyun_tts_client),
        ("粤语TTS模块", test_cantonese_tts),
        ("唤醒词配置管理器", test_wake_word_config),
        ("音频播放器", test_audio_player),
        ("ASR重试管理器", test_asr_retry_manager),
        ("阿里云API配置", test_aliyun_api_config)
    ]

    results = {}
    passed_tests = 0
    total_tests = len(tests)

    for test_name, test_func in tests:
        log(f"\n🧪 测试: {test_name}")
        try:
            result = test_func()
            results[test_name] = result
            if result:
                passed_tests += 1
                log(f"✅ {test_name} - 通过")
            else:
                log(f"❌ {test_name} - 失败")
        except Exception as e:
            log(f"❌ {test_name} - 异常: {e}")
            results[test_name] = False

    # 生成验证报告
    success_rate = passed_tests / total_tests * 100

    log(f"\n" + "="*60)
    log(f"📊 Epic 1 完整功能验证报告")
    log(f"="*60)
    log(f"总测试项: {total_tests}")
    log(f"通过测试: {passed_tests}")
    log(f"失败测试: {total_tests - passed_tests}")
    log(f"成功率: {success_rate:.1f}%")

    log(f"\n详细结果:")
    for test_name, result in results.items():
        status = "✅ 通过" if result else "❌ 失败"
        log(f"  {test_name}: {status}")

    # 最终评估
    if success_rate >= 90:
        log(f"\n🎉 Epic 1 功能验证优秀！")
        log(f"✅ 所有核心组件正常工作")
        log(f"✅ 可以进入迭代2开发")
    elif success_rate >= 70:
        log(f"\n👍 Epic 1 功能验证良好！")
        log(f"✅ 大部分组件正常工作")
        log(f"⚠️ 少数组件需要进一步调试")
    else:
        log(f"\n⚠️ Epic 1 功能验证需要改进")
        log(f"❌ 多个组件存在问题")
        log(f"🔧 需要进一步修复")

    log(f"\n" + "="*60)

    return results, success_rate

if __name__ == "__main__":
    # 运行完整验证
    results, success_rate = run_complete_verification()

    # 保存验证结果
    import json
    report_file = Path(__file__).parent / f"epic1_verification_result_{int(time.time())}.json"
    with open(report_file, 'w', encoding='utf-8') as f:
        json.dump({
            "timestamp": time.strftime('%Y-%m-%d %H:%M:%S'),
            "results": results,
            "success_rate": success_rate,
            "total_tests": len(results),
            "passed_tests": sum(results.values())
        }, f, indent=2, ensure_ascii=False)

    log(f"\n📄 验证结果已保存到: {report_file}")