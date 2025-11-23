#!/usr/bin/env python3
"""
简单音频节点测试
====================

测试简化的音频录制和基本功能，不依赖复杂的ROS2消息类型。

作者: Developer Agent
版本: 1.0 (纯在线架构)
日期: 2025-11-09
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'modules', 'asr'))

from simple_audio_recorder import SimpleAudioRecorder
from audio_converter import AudioConverter
import numpy as np
import time
import logging

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def test_audio_recording():
    """测试音频录制功能"""
    logger.info("=== 音频录制测试 ===")

    recorder = SimpleAudioRecorder()

    # 显示配置
    config = recorder.get_audio_config()
    print("音频配置:")
    for key, value in config.items():
        print(f"  {key}: {value}")

    # 测试录制
    print("\n录制测试音频 (1秒)...")
    success = recorder.start_recording(duration=1.0)

    if success:
        time.sleep(1.1)
        audio_data = recorder.stop_recording()

        if len(audio_data) > 0:
            print(f"录制成功: {len(audio_data)}样本")
            return audio_data
        else:
            print("录制失败: 没有数据")
            return None
    else:
        print("录制启动失败")
        return None


def test_audio_conversion(audio_data):
    """测试音频格式转换"""
    logger.info("=== 音频转换测试 ===")

    if audio_data is None or len(audio_data) == 0:
        print("跳过转换测试: 无音频数据")
        return False

    converter = AudioConverter()

    # 显示配置
    print(f"转换器配置: {converter.sample_rate}Hz, {converter.channels}通道, {converter.sample_width * 8}bit")

    try:
        # PCM → WAV
        print("PCM → WAV转换...")
        wav_data = converter.pcm_to_wav(audio_data)
        print(f"WAV数据: {len(wav_data)}字节")

        # WAV → Base64
        print("WAV → Base64转换...")
        base64_data = converter.wav_to_base64(wav_data)
        print(f"Base64数据: {len(base64_data)}字符")

        # Base64 → WAV
        print("Base64 → WAV转换...")
        wav_data_2 = converter.base64_to_wav(base64_data)
        print(f"还原WAV数据: {len(wav_data_2)}字节")

        # 验证一致性
        if wav_data == wav_data_2:
            print("✅ 转换一致性验证通过")
            return True
        else:
            print("❌ 转换一致性验证失败")
            return False

    except Exception as e:
        print(f"转换测试失败: {e}")
        return False


def test_complete_workflow():
    """测试完整工作流程"""
    logger.info("=== 完整工作流程测试 ===")

    try:
        # 步骤1: 录制音频
        print("步骤1: 录制音频")
        audio_data = test_audio_recording()

        if audio_data is None:
            print("❌ 录制失败，无法继续测试")
            return False

        # 步骤2: 格式转换
        print("\n步骤2: 音频格式转换")
        conversion_success = test_audio_conversion(audio_data)

        if not conversion_success:
            print("❌ 转换失败")
            return False

        print("\n✅ 完整工作流程测试通过")
        return True

    except Exception as e:
        print(f"❌ 工作流程测试异常: {e}")
        return False


def main():
    """主函数"""
    print("=== 简单音频节点功能测试 ===")
    print("纯在线架构 - Phase 3 功能验证")
    print(f"测试时间: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print()

    # 运行测试
    success = test_complete_workflow()

    print(f"\n=== 测试结果 ===")
    print(f"总体结果: {'通过' if success else '失败'}")

    if success:
        print("✅ Phase 3 核心功能验证完成")
        print("✅ 音频录制和转换功能正常")
        print("✅ 纯在线架构实施就绪")
    else:
        print("❌ 测试失败，需要检查问题")

    print("\n测试完成")


if __name__ == "__main__":
    main()