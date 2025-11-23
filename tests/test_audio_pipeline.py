#!/usr/bin/env python3.10
# -*- coding: utf-8 -*-
"""
音频管道测试
测试录制、处理和播放的基本功能

作者: BMad代理团队
"""

import os
import subprocess
import time
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def test_audio_pipeline():
    """测试音频管道"""
    print("🎵 音频管道功能测试")
    print("=" * 40)

    test_file = "test_audio_pipeline.wav"
    test_text = "你好，这是XLeRobot音频测试"

    try:
        # 测试1: 音频录制
        print("1. 测试音频录制 (2秒)...")
        record_cmd = [
            "arecord",
            "-D", "hw:1",  # ES8326设备
            "-d", "2",     # 2秒
            "-f", "cd",    # CD质量
            "-r", "16000", # 16kHz采样率
            "-c", "1",     # 单声道
            test_file
        ]

        result = subprocess.run(record_cmd, capture_output=True, text=True)
        if result.returncode == 0:
            print("✅ 音频录制成功")
            file_size = os.path.getsize(test_file)
            print(f"   文件大小: {file_size} bytes")
        else:
            print(f"❌ 音频录制失败: {result.stderr}")
            return False

        # 测试2: 音频播放
        print("2. 测试音频播放...")
        play_cmd = ["aplay", "-D", "hw:1", "-q", test_file]
        result = subprocess.run(play_cmd, capture_output=True, text=True)

        if result.returncode == 0:
            print("✅ 音频播放成功")
        else:
            print(f"❌ 音频播放失败: {result.stderr}")
            return False

        # 测试3: 音频格式验证
        print("3. 验证音频格式...")
        format_cmd = ["file", test_file]
        result = subprocess.run(format_cmd, capture_output=True, text=True)

        if result.returncode == 0:
            print(f"✅ 音频格式: {result.stdout.strip()}")
        else:
            print(f"⚠️ 无法验证格式: {result.stderr}")

        return True

    except Exception as e:
        print(f"❌ 音频管道测试异常: {e}")
        return False

    finally:
        # 清理测试文件
        if os.path.exists(test_file):
            os.remove(test_file)
            print("🗑️ 清理测试文件")

def test_asr_basic():
    """测试ASR基础功能"""
    print("\n🎤 ASR基础功能测试")
    print("=" * 40)

    try:
        # 测试ASR模块导入
        sys.path.insert(0, '/home/sunrise/xlerobot/src')
        from modules.asr.websocket_asr_service import WebSocketASRService

        print("✅ ASR服务导入成功")

        # 测试服务创建（不启动连接）
        try:
            asr_service = WebSocketASRService(enable_optimization=False)
            print("✅ ASR服务创建成功")

            # 检查健康状态
            health = asr_service.health_check()
            print(f"✅ 健康检查: {health}")

            return True

        except Exception as e:
            print(f"⚠️ ASR服务创建问题（可能缺少API密钥）: {e}")
            return False

    except ImportError as e:
        print(f"❌ ASR模块导入失败: {e}")
        return False

def main():
    """主函数"""
    success_count = 0
    total_tests = 2

    # 音频管道测试
    if test_audio_pipeline():
        success_count += 1

    # ASR基础测试
    if test_asr_basic():
        success_count += 1

    # 结果汇总
    print(f"\n📊 测试结果: {success_count}/{total_tests} 通过")
    success_rate = (success_count / total_tests) * 100
    print(f"成功率: {success_rate:.1f}%")

    if success_rate == 100:
        print("🎉 音频系统完全正常！")
    elif success_rate >= 50:
        print("✅ 音频系统基本正常")
    else:
        print("❌ 音频系统存在问题")

if __name__ == "__main__":
    import sys
    main()