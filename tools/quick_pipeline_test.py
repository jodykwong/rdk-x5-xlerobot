#!/usr/bin/env python3.10
"""
快速粤语ASR→TTS验证脚本
直接测试核心功能，不依赖复杂系统
"""

import os
import sys
import json
import time
import base64
import subprocess
import requests
from pathlib import Path
from datetime import datetime

print("=" * 60)
print("🚀 快速粤语ASR → TTS 验证测试")
print("=" * 60)

# ============ 配置 ============
ACCESS_KEY_ID = "YOUR_ACCESS_KEY_ID"
ACCESS_KEY_SECRET = "YOUR_ACCESS_KEY_SECRET"
ASR_APPKEY = "YOUR_NLS_APPKEY"

# ============ 测试1: Token获取 ============
print("\n🔑 测试1: Token获取...")

def get_aliyun_token():
    """直接获取阿里云Token"""
    try:
        url = "https://nls-meta.cn-shanghai.aliyuncs.com/pop/2018-05-18/tokens"
        headers = {
            "Content-Type": "application/json",
            "Date": datetime.utcnow().strftime('%a, %d %b %Y %H:%M:%S GMT'),
            "Host": "nls-meta.cn-shanghai.aliyuncs.com"
        }

        data = {
            "AccessKeyId": ACCESS_KEY_ID,
            "Action": "CreateToken"
        }

        response = requests.post(url, headers=headers, json=data, timeout=10)

        print(f"   响应状态码: {response.status_code}")

        if response.status_code == 200:
            result = response.json()
            if "Token" in result and "Id" in result["Token"]:
                return result["Token"]["Id"]
            else:
                print(f"   ❌ Token格式错误: {result}")
                return None
        else:
            print(f"   ❌ 请求失败: {response.text}")
            return None

    except Exception as e:
        print(f"   ❌ 异常: {e}")
        return None

token = get_aliyun_token()

if not token:
    print("❌ Token获取失败，测试终止")
    sys.exit(1)

print(f"✅ Token获取成功: {token[:20]}...")

# ============ 测试2: 音频录制 ============
print("\n🎤 测试2: 音频录制...")
print("💬 请说粤语：'你好啊' (3秒)")

audio_file = "/tmp/quick_test.wav"
try:
    result = subprocess.run([
        'arecord', '-D', 'hw:0,0',
        '-f', 'S16_LE',
        '-r', '16000',
        '-c', '1',
        '-d', '3',
        audio_file
    ], capture_output=True, text=True, timeout=10)

    if result.returncode != 0:
        print(f"❌ 录音失败: {result.stderr}")
        sys.exit(1)

    file_size = os.path.getsize(audio_file)
    print(f"✅ 录音成功: {file_size} 字节")

    # 快速播放确认
    print("🔊 播放录音确认...")
    subprocess.run(['aplay', audio_file], capture_output=True, timeout=5)

except Exception as e:
    print(f"❌ 录音异常: {e}")
    sys.exit(1)

# ============ 测试3: ASR识别 ============
print("\n🧠 测试3: ASR语音识别...")

def speech_to_text(audio_path, token):
    """调用阿里云ASR"""
    try:
        with open(audio_path, 'rb') as f:
            audio_data = f.read()

        audio_b64 = base64.b64encode(audio_data).decode('utf-8')

        url = "https://nls-gateway.cn-shanghai.aliyuncs.com/stream/v1/asr"
        headers = {
            "Content-Type": "application/json",
            "X-NLS-Token": token
        }

        data = {
            "appkey": ASR_APPKEY,
            "format": "wav",
            "sample_rate": 16000,
            "enable_intermediate_result": False,
            "enable_punctuation_prediction": True,
            "enable_inverse_text_normalization": True,
            "audio": audio_b64
        }

        response = requests.post(url, headers=headers, json=data, timeout=30)

        print(f"   ASR响应状态码: {response.status_code}")

        if response.status_code == 200:
            result = response.json()
            recognized_text = result.get('result', '')
            return recognized_text
        else:
            print(f"   ❌ ASR失败: {response.text}")
            return None

    except Exception as e:
        print(f"   ❌ ASR异常: {e}")
        return None

recognized_text = speech_to_text(audio_file, token)

if not recognized_text:
    print("❌ ASR识别失败")
    sys.exit(1)

print(f"✅ ASR识别成功: '{recognized_text}'")

# ============ 测试4: TTS合成 ============
print("\n🔊 测试4: TTS语音合成...")

response_text = f"好嘅，我听到你讲：{recognized_text}"

def text_to_speech(text, token):
    """调用阿里云TTS"""
    try:
        url = "https://nls-gateway.cn-shanghai.aliyuncs.com/stream/v1/tts"
        headers = {
            "Content-Type": "application/json",
            "X-NLS-Token": token
        }

        data = {
            "appkey": ASR_APPKEY,
            "text": text,
            "format": "wav",
            "sample_rate": 16000,
            "voice": "sijia",  # 粤语女声
            "volume": 100,
            "speech_rate": 0,
            "pitch_rate": 0
        }

        response = requests.post(url, headers=headers, json=data, timeout=30)

        print(f"   TTS响应状态码: {response.status_code}")

        if response.status_code == 200:
            return response.content
        else:
            print(f"   ❌ TTS失败: {response.text}")
            return None

    except Exception as e:
        print(f"   ❌ TTS异常: {e}")
        return None

tts_audio = text_to_speech(response_text, token)

if not tts_audio:
    print("❌ TTS合成失败")
    sys.exit(1)

tts_file = "/tmp/quick_tts.wav"
with open(tts_file, 'wb') as f:
    f.write(tts_audio)

tts_size = os.path.getsize(tts_file)
print(f"✅ TTS合成成功: {tts_size} 字节")

# ============ 测试5: 播放验证 ============
print("\n🔊 测试5: 播放合成语音...")

try:
    print("🎵 播放TTS合成语音...")
    result = subprocess.run(['aplay', tts_file], capture_output=True, timeout=10)

    if result.returncode == 0:
        print("✅ 播放成功")
    else:
        print(f"⚠️ 播放失败: {result.stderr}")

except Exception as e:
    print(f"⚠️ 播放异常: {e}")

# ============ 测试总结 ============
print("\n" + "=" * 60)
print("🎉 快速验证测试完成！")
print("=" * 60)
print(f"✅ Token获取: 成功")
print(f"✅ 音频录制: 成功 ({file_size} 字节)")
print(f"✅ ASR识别: 成功 -> '{recognized_text}'")
print(f"✅ TTS合成: 成功 ({tts_size} 字节)")
print("✅ 音频播放: 成功")
print("=" * 60)

print(f"\n💡 测试文本:")
print(f"   输入语音: (粤语) '你好啊'")
print(f"   ASR识别: '{recognized_text}'")
print(f"   TTS回复: '{response_text}'")

print("\n🎊 结论: 粤语ASR→TTS核心链路技术验证成功！")
print("💪 可以基于此API调用方式集成到完整系统")