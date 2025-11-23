#!/usr/bin/env python3.10
"""
TTS功能测试脚本
单独测试TTS语音合成功能
"""

import os
import sys

print("=" * 60)
print("🔊 TTS语音合成功能测试")
print("=" * 60)

# ============ 检查环境变量 ============
required_env_vars = [
    "ALIBABA_CLOUD_ACCESS_KEY_ID",
    "ALIBABA_CLOUD_ACCESS_KEY_SECRET",
    "ALIYUN_NLS_APPKEY"
]

missing_vars = [var for var in required_env_vars if not os.getenv(var)]
if missing_vars:
    print("❌ 缺少必需的环境变量:")
    for var in missing_vars:
        print(f"  - {var}")
    print("\n请先设置这些环境变量，例如:")
    print("  export ALIBABA_CLOUD_ACCESS_KEY_ID='your_key_id'")
    print("  export ALIBABA_CLOUD_ACCESS_KEY_SECRET='your_key_secret'")
    print("  export ALIYUN_NLS_APPKEY='your_app_key'")
    sys.exit(1)

print(f"✅ 环境变量检查通过")

# ============ 添加路径 ============
sys.path.insert(0, '/home/sunrise/xlerobot/src')

try:
    from modules.tts.engine.aliyun_tts_client import AliyunTTSClient

    print("✅ TTS服务导入成功")

    # ============ 测试TTS服务 ============
    print("\n🔊 测试TTS语音合成...")

    test_texts = [
        "你好，我係语音助手",
        "今日天气几好",
        "多谢你嘅使用",
        "粤语语音测试成功"
    ]

    tts_service = AliyunTTSClient()

    for i, text in enumerate(test_texts, 1):
        print(f"\n测试{i}: 合成文本 '{text}'...")

        try:
            tts_audio = tts_service.synthesize(text, voice="sijia")

            if not tts_audio:
                print(f"❌ TTS合成失败")
            else:
                tts_file = f"/tmp/tts_test_{i}.wav"
                with open(tts_file, 'wb') as f:
                    f.write(tts_audio)

                tts_size = os.path.getsize(tts_file)
                print(f"✅ TTS合成成功: {tts_size} 字节 -> {tts_file}")

                # 播放测试
                import subprocess
                print("🔊 播放测试...")
                result = subprocess.run(['aplay', tts_file], capture_output=True, timeout=5)

                if result.returncode == 0:
                    print("✅ 播放成功")
                else:
                    print(f"⚠️ 播放失败: {result.stderr}")

        except Exception as e:
            print(f"❌ TTS测试异常: {e}")

    # ============ 测试总结 ============
    print("\n" + "=" * 60)
    print("🎉 TTS功能测试完成！")
    print("=" * 60)

    # 检查生成的文件
    import glob
    tts_files = glob.glob("/tmp/tts_test_*.wav")
    if tts_files:
        print(f"✅ 生成了 {len(tts_files)} 个TTS音频文件:")
        for f in tts_files:
            size = os.path.getsize(f)
            print(f"   📄 {f} ({size} 字节)")

    print("=" * 60)
    print("💡 结论: TTS语音合成功能验证完成！")

except Exception as e:
    print(f"❌ TTS服务初始化失败: {e}")
    import traceback
    traceback.print_exc()