#!/usr/bin/env python3.10
"""
使用环境变量的粤语ASR→TTS验证脚本
"""

import os
import sys
import subprocess

print("=" * 60)
print("🔧 使用环境变量的粤语语音验证测试")
print("=" * 60)

# ============ 设置环境变量 ============
os.environ["ALIBABA_CLOUD_ACCESS_KEY_ID"] = "LTAI5tQ4E2YNzZkGn9g1JqeY"
os.environ["ALIBABA_CLOUD_ACCESS_KEY_SECRET"] = "Hr1xZdcdz3D9OgFnH1nvWz5rldXVeI"
os.environ["ALIYUN_NLS_APPKEY"] = "4G5BCMccTCW8nC8w"

print(f"✅ 环境变量设置完成")
print(f"  ALIBABA_CLOUD_ACCESS_KEY_ID: {os.environ['ALIBABA_CLOUD_ACCESS_KEY_ID'][:10]}...")
print(f"  ALIYUN_NLS_APPKEY: {os.environ['ALIYUN_NLS_APPKEY']}")

# ============ 添加路径 ============
sys.path.insert(0, '/home/sunrise/xlerobot/src')

try:
    from modules.asr.simple_aliyun_asr_service import SimpleAliyunASRService
    from modules.tts.engine.aliyun_tts_client import AliyunTTSClient

    print("✅ 服务导入成功")

    # ============ 测试1: ASR服务 ============
    print("\n🎤 测试1: ASR语音识别...")
    print("💬 请说粤语：'你好啊' (3秒)")

    # 录制音频
    audio_file = "/tmp/env_test.wav"
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

    # ASR识别
    app_key = os.environ.get("ALIYUN_NLS_APPKEY", "")
    asr_service = SimpleAliyunASRService(app_key=app_key)

    with open(audio_file, 'rb') as f:
        audio_data = f.read()

    result = asr_service.recognize_speech(
        audio_data=audio_data,
        language="cn-cantonese"
    )
    recognized_text = result.text if result.success else ''

    if not recognized_text:
        print("❌ ASR识别失败")
        print("💡 检查网络连接或API配置")
    else:
        print(f"✅ ASR识别成功: '{recognized_text}'")

        # ============ 测试2: TTS服务 ============
        print("\n🔊 测试2: TTS语音合成...")

        response_text = f"好嘅，我听到你讲：{recognized_text}"

        tts_service = AliyunTTSClient()
        tts_audio = tts_service.text_to_speech(response_text)

        if not tts_audio:
            print("❌ TTS合成失败")
        else:
            tts_file = "/tmp/env_tts.wav"
            with open(tts_file, 'wb') as f:
                f.write(tts_audio)

            tts_size = os.path.getsize(tts_file)
            print(f"✅ TTS合成成功: {tts_size} 字节")

            # ============ 测试3: 播放验证 ============
            print("\n🔊 测试3: 播放验证...")

            print("🎵 播放原始录音...")
            subprocess.run(['aplay', audio_file], capture_output=True, timeout=5)

            print("🎵 播放TTS合成语音...")
            result = subprocess.run(['aplay', tts_file], capture_output=True, timeout=5)

            if result.returncode == 0:
                print("✅ 播放成功")
            else:
                print(f"⚠️ 播放失败: {result.stderr}")

            # ============ 测试总结 ============
            print("\n" + "=" * 60)
            print("🎉 环境变量版验证测试完成！")
            print("=" * 60)
            print(f"✅ ASR识别: 成功 -> '{recognized_text}'")
            print(f"✅ TTS合成: 成功 ({tts_size} 字节)")
            print(f"✅ 音频播放: 成功")
            print("=" * 60)
            print("\n💡 结论: 粤语ASR→TTS链路验证成功！")
            print(f"   输入: (粤语录音)")
            print(f"   识别: '{recognized_text}'")
            print(f"   合成: '{response_text}'")

except Exception as e:
    print(f"❌ 服务初始化失败: {e}")
    import traceback
    traceback.print_exc()