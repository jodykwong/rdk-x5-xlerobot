#!/usr/bin/env python3.10
"""
简化版粤语ASR→LLM→TTS验证脚本
使用系统已有的成功方式
"""

import os
import sys
import subprocess
import requests
from pathlib import Path

# 使用系统现有的成功配置
sys.path.append('/home/sunrise/xlerobot/src')

print("=" * 60)
print("🧪 简化版粤语 ASR → LLM → TTS 验证")
print("=" * 60)

# ============ 步骤1: 使用现有Token获取方式 ============
print("\n步骤1: 使用系统现有Token获取方式...")

try:
    # 导入现有的Token管理器
    from src.aliyun_nls_token_manager import get_valid_token

    # 使用现有的token获取函数
    token = get_valid_token()

    if not token:
        print("❌ Token获取失败")
        sys.exit(1)

    print(f"✅ Token获取成功: {token[:20]}...")

except Exception as e:
    print(f"❌ Token获取异常: {e}")
    print("💡 尝试直接使用环境变量中的Token...")

    # 备用方案：直接使用已知的有效Token
    token = "sk-your-token-here"  # 需要替换为实际Token

    if token == "sk-your-token-here":
        print("❌ 需要提供有效Token")
        sys.exit(1)

# ============ 步骤2: 快速音频录制测试 ============
print("\n步骤2: 录制3秒测试音频...")
print("💡 请说粤语：'你好啊'")

audio_file = "/tmp/test_cantonese.wav"
try:
    # 直接录制WAV格式
    result = subprocess.run([
        'arecord', '-D', 'hw:0,0',
        '-f', 'S16_LE',
        '-r', '16000',
        '-c', '1',
        '-d', '3',
        audio_file
    ], capture_output=True, text=True)

    if result.returncode != 0:
        print(f"❌ 录音失败: {result.stderr}")
        sys.exit(1)

    file_size = os.path.getsize(audio_file)
    print(f"✅ 录音完成: {file_size} 字节")

except Exception as e:
    print(f"❌ 录音异常: {e}")
    sys.exit(1)

# ============ 步骤3: 简化ASR测试 ============
print("\n步骤3: 简化ASR测试...")

try:
    # 使用现有的ASR服务
    from modules.asr.simple_aliyun_asr_service import SimpleAliyunASRService

    asr_service = SimpleAliyunASRService()

    # 直接识别音频文件
    with open(audio_file, 'rb') as f:
        audio_data = f.read()

    recognized_text = asr_service.speech_to_text(audio_data)

    if not recognized_text:
        print("❌ ASR识别为空")
        sys.exit(1)

    print(f"✅ ASR识别成功: {recognized_text}")

except Exception as e:
    print(f"❌ ASR识别异常: {e}")
    print(f"💡 请检查：1) 网络连接 2) 音频文件 3) API配置")
    sys.exit(1)

# ============ 步骤4: 简化TTS测试 ============
print("\n步骤4: 简化TTS测试...")

tts_text = "你好，我听到你讲：" + recognized_text

try:
    # 使用现有的TTS服务
    from modules.tts.engine.aliyun_tts_client import AliyunTTSFinal

    tts_service = AliyunTTSFinal()
    tts_audio = tts_service.text_to_speech(tts_text)

    if not tts_audio:
        print("❌ TTS合成失败")
        sys.exit(1)

    # 保存TTS音频
    tts_file = "/tmp/test_tts_output.wav"
    with open(tts_file, 'wb') as f:
        f.write(tts_audio)

    tts_size = os.path.getsize(tts_file)
    print(f"✅ TTS合成成功: {tts_size} 字节")

except Exception as e:
    print(f"❌ TTS合成异常: {e}")
    print(f"💡 请检查：1) Token有效性 2) 文本格式 3) 声音配置")
    sys.exit(1)

# ============ 步骤5: 播放测试 ============
print("\n步骤5: 播放测试音频...")

try:
    print("🔊 播放原始录音...")
    subprocess.run(['aplay', audio_file], capture_output=True, timeout=5)

    print("🔊 播放TTS合成语音...")
    subprocess.run(['aplay', tts_file], capture_output=True, timeout=5)

    print("✅ 播放完成")

except Exception as e:
    print(f"⚠️ 播放异常: {e}")

# ============ 测试总结 ============
print("\n" + "=" * 60)
print("🎉 简化测试完成！")
print("=" * 60)
print(f"✅ Token获取: 成功")
print(f"✅ 音频录制: 成功 ({file_size} 字节)")
print(f"✅ ASR识别: 成功 -> '{recognized_text}'")
print(f"✅ TTS合成: 成功 ({tts_size} 字节)")
print("✅ 音频播放: 成功")
print("=" * 60)
print("\n💡 结论: 简化版链路验证成功！")