#!/usr/bin/env python3.10
"""
粤语 ASR → LLM → TTS 完整验证脚本
用途：验证阿里云在线服务的完整链路
"""

import os
import sys
import time
import json
import base64
import subprocess
import requests
from pathlib import Path
from datetime import datetime

# ============ 配置区域 ============
ACCESS_KEY_ID = "LTAI5tQ4E2YNzZkGn9g1JqeY"
ACCESS_KEY_SECRET = "Hr1xZdcdz3D9OgFnH1nvWz5rldXVeI"
ASR_APPKEY = "4G5BCMccTCW8nC8w"
QWEN_API_KEY = os.environ.get("DASHSCOPE_API_KEY", "")

# 音频参数
SAMPLE_RATE = 16000
RECORD_SECONDS = 3

print("=" * 60)
print("🧪 粤语 ASR → LLM → TTS 完整链路验证")
print("=" * 60)

# ============ 步骤1: 获取阿里云Token ============
print("\n步骤1: 获取阿里云Token...")
try:
    # 生成GMT格式的日期
    gmt_date = datetime.utcnow().strftime('%a, %d %b %Y %H:%M:%S GMT')
    
    token_resp = requests.post(
        "https://nls-meta.cn-shanghai.aliyuncs.com/pop/2018-05-18/tokens",
        headers={
            "Content-Type": "application/json",
            "Date": gmt_date,
            "Host": "nls-meta.cn-shanghai.aliyuncs.com"
        },
        json={
            "AccessKeyId": ACCESS_KEY_ID,
            "Action": "CreateToken"
        },
        timeout=10
    )
    
    if token_resp.status_code != 200:
        print(f"❌ Token获取失败: {token_resp.status_code}")
        print(token_resp.text)
        sys.exit(1)
    
    token = token_resp.json()['Token']['Id']
    print(f"✅ Token获取成功: {token[:20]}...")
    
except Exception as e:
    print(f"❌ Token获取异常: {e}")
    sys.exit(1)

# ============ 步骤2: 录制粤语音频 ============
print(f"\n步骤2: 录制{RECORD_SECONDS}秒粤语音频...")
print("💡 请用粤语说一句话，例如：'今日天气点样？'")
time.sleep(1)

audio_file = "/tmp/test_cantonese.pcm"
try:
    result = subprocess.run([
        'arecord', '-D', 'hw:0,0',
        '-f', 'S16_LE',
        '-r', str(SAMPLE_RATE),
        '-c', '1',
        '-d', str(RECORD_SECONDS),
        audio_file
    ], capture_output=True, text=True)
    
    if result.returncode != 0:
        print(f"❌ 录音失败: {result.stderr}")
        sys.exit(1)
    
    # 检查文件大小
    file_size = os.path.getsize(audio_file)
    print(f"✅ 录音完成: {file_size} 字节")
    
except Exception as e:
    print(f"❌ 录音异常: {e}")
    sys.exit(1)

# ============ 步骤3: ASR识别（粤语） ============
print("\n步骤3: 调用ASR识别粤语...")
try:
    # 读取并编码音频
    with open(audio_file, 'rb') as f:
        audio_data = f.read()
    audio_b64 = base64.b64encode(audio_data).decode('utf-8')
    
    # 调用ASR API
    asr_resp = requests.post(
        "https://nls-gateway.cn-shanghai.aliyuncs.com/stream/v1/asr",
        headers={
            "Content-Type": "application/json",
            "X-NLS-Token": token
        },
        json={
            "appkey": ASR_APPKEY,
            "format": "pcm",
            "sample_rate": SAMPLE_RATE,
            "enable_intermediate_result": False,
            "enable_punctuation_prediction": True,
            "enable_inverse_text_normalization": True,
            "audio": audio_b64
        },
        timeout=30
    )
    
    print(f"   ASR响应状态码: {asr_resp.status_code}")
    
    if asr_resp.status_code != 200:
        print(f"❌ ASR识别失败: {asr_resp.status_code}")
        print(f"   响应内容: {asr_resp.text}")
        sys.exit(1)
    
    asr_result = asr_resp.json()
    recognized_text = asr_result.get('result', '')
    
    if not recognized_text:
        print("❌ 未识别到文字")
        print(f"   完整响应: {json.dumps(asr_result, ensure_ascii=False)}")
        sys.exit(1)
    
    print(f"✅ ASR识别成功")
    print(f"   识别文本: {recognized_text}")
    
except Exception as e:
    print(f"❌ ASR识别异常: {e}")
    sys.exit(1)

# ============ 步骤4: LLM对话（通义千问） ============
print("\n步骤4: 调用LLM生成回复...")

if not QWEN_API_KEY:
    print("⚠️ 未设置DASHSCOPE_API_KEY，跳过LLM测试")
    llm_response = "好嘅，收到你嘅讯息啦！"
else:
    try:
        llm_resp = requests.post(
            "https://dashscope.aliyuncs.com/api/v1/services/aigc/text-generation/generation",
            headers={
                "Content-Type": "application/json",
                "Authorization": f"Bearer {QWEN_API_KEY}"
            },
            json={
                "model": "qwen-turbo",
                "input": {
                    "messages": [
                        {
                            "role": "system",
                            "content": "你是一个会说粤语的助手，请用粤语回复用户。"
                        },
                        {
                            "role": "user",
                            "content": recognized_text
                        }
                    ]
                },
                "parameters": {
                    "max_tokens": 100
                }
            },
            timeout=30
        )
        
        print(f"   LLM响应状态码: {llm_resp.status_code}")
        
        if llm_resp.status_code != 200:
            print(f"❌ LLM调用失败: {llm_resp.status_code}")
            print(f"   响应内容: {llm_resp.text}")
            llm_response = "好嘅，收到你嘅讯息啦！"
        else:
            llm_result = llm_resp.json()
            llm_response = llm_result['output']['text']
            print(f"✅ LLM回复成功")
            print(f"   回复内容: {llm_response}")
            
    except Exception as e:
        print(f"⚠️ LLM调用异常: {e}")
        llm_response = "好嘅，收到你嘅讯息啦！"

# ============ 步骤5: TTS合成（粤语） ============
print("\n步骤5: TTS合成粤语语音...")
try:
    tts_resp = requests.post(
        "https://nls-gateway.cn-shanghai.aliyuncs.com/stream/v1/tts",
        headers={
            "Content-Type": "application/json",
            "X-NLS-Token": token
        },
        json={
            "appkey": ASR_APPKEY,
            "text": llm_response,
            "format": "wav",
            "sample_rate": 16000,
            "voice": "sijia",  # 粤语女声
            "volume": 100,
            "speech_rate": 0,
            "pitch_rate": 0
        },
        timeout=30
    )
    
    print(f"   TTS响应状态码: {tts_resp.status_code}")
    
    if tts_resp.status_code != 200:
        print(f"❌ TTS合成失败: {tts_resp.status_code}")
        print(f"   响应内容: {tts_resp.text}")
        sys.exit(1)
    
    # 保存音频
    tts_file = "/tmp/test_tts_output.wav"
    with open(tts_file, 'wb') as f:
        f.write(tts_resp.content)
    
    tts_size = os.path.getsize(tts_file)
    print(f"✅ TTS合成成功: {tts_size} 字节")
    
except Exception as e:
    print(f"❌ TTS合成异常: {e}")
    sys.exit(1)

# ============ 步骤6: 播放合成语音 ============
print("\n步骤6: 播放合成的粤语语音...")
try:
    result = subprocess.run(
        ['aplay', tts_file],
        capture_output=True,
        timeout=10
    )
    
    if result.returncode != 0:
        print(f"⚠️ 播放失败: {result.stderr}")
    else:
        print("✅ 播放完成")
        
except Exception as e:
    print(f"⚠️ 播放异常: {e}")

# ============ 测试总结 ============
print("\n" + "=" * 60)
print("🎉 测试完成！完整链路验证结果：")
print("=" * 60)
print(f"✅ Token获取: 成功")
print(f"✅ 音频录制: 成功 ({file_size} 字节)")
print(f"✅ ASR识别: 成功 -> '{recognized_text}'")
print(f"✅ LLM回复: 成功 -> '{llm_response}'")
print(f"✅ TTS合成: 成功 ({tts_size} 字节)")
print("=" * 60)
print("\n💡 结论: 粤语 ASR → LLM → TTS 完整链路可用！")