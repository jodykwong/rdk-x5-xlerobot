#!/usr/bin/env python3
"""
验证阿里云TTS粤语发音人支持
"""

import requests
import sys
from fixed_aliyun_config import FixedAliyunConfigManager

def test_cantonese_voices():
    """测试不同的粤语发音人"""
    print("🔊 验证阿里云TTS粤语发音人支持")

    # 获取配置
    config_manager = FixedAliyunConfigManager()
    config = config_manager.get_config()
    token = config_manager.get_valid_token()

    # 粤语测试文本
    cantonese_text = "你好，我係粤语语音合成测试"

    # 根据阿里云文档，支持的粤语发音人
    cantonese_voices = {
        "xiaoyun": "小云 (普通话/通用)",
        "siyue": "思月 (粤语女声)",
        "xiaomei": "小美 (普通话女声)",
        "xiaogang": "小刚 (普通话男声)",
        "xiaowang": "小王 (普通话男声)"
    }

    print(f"📝 测试文本: {cantonese_text}")
    print("="*60)

    for voice_name, description in cantonese_voices.items():
        print(f"\n🔊 测试发音人: {voice_name} - {description}")

        try:
            # HTTP请求
            url = "https://nls-gateway.cn-shanghai.aliyuncs.com/stream/v1/tts"
            headers = {
                'Content-Type': 'application/json',
                'Authorization': f'Bearer {token}'
            }

            payload = {
                'appkey': config.app_key,
                'token': token,
                'text': cantonese_text,
                'voice': voice_name,
                'format': 'wav',
                'sample_rate': 16000,
                'volume': 100,
                'speech_rate': 0,
                'pitch_rate': 0
            }

            response = requests.post(url, json=payload, headers=headers, timeout=10)

            if response.status_code == 200:
                if response.content.startswith(b'RIFF'):
                    # 保存音频文件
                    filename = f"voice_test_{voice_name}.wav"
                    with open(filename, 'wb') as f:
                        f.write(response.content)

                    print(f"   ✅ 合成成功: {len(response.content)} bytes")
                    print(f"   💾 保存为: {filename}")

                    # 播放音频
                    import subprocess
                    try:
                        subprocess.run(['aplay', filename], check=True, capture_output=True)
                        print(f"   🔊 播放成功")

                        # 检查是否真的是粤语发音
                        if voice_name == "siyue":
                            print("   🎯 这是专业的粤语发音人")
                        else:
                            print("   ⚠️ 这可能是普通话发音人")
                    except subprocess.CalledProcessError:
                        print("   ⚠️ 播放失败，但文件保存成功")
                else:
                    print(f"   ❌ 响应不是音频格式")
            else:
                print(f"   ❌ HTTP错误: {response.status_code}")
                print(f"   📄 错误信息: {response.text[:100]}")

        except Exception as e:
            print(f"   ❌ 异常: {e}")

    print("\n" + "="*60)
    print("💡 发音人说明:")
    print("- siyue: 思月 - 专业的粤语女声发音人")
    print("- xiaoyun: 小云 - 通用发音人，支持多种语言")
    print("- 其他发音人可能主要为普通话")

if __name__ == "__main__":
    test_cantonese_voices()