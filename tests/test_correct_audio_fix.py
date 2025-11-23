#!/usr/bin/env python3
"""
测试正确的音频修复 - 使用启动脚本中的ASR系统
"""
import sys
import os
import time
import logging

# 设置环境变量
os.environ["ALIBABA_CLOUD_ACCESS_KEY_ID"] = "YOUR_ACCESS_KEY_ID"
os.environ["ALIBABA_CLOUD_ACCESS_KEY_SECRET"] = "YOUR_ACCESS_KEY_SECRET"
os.environ["ALIYUN_NLS_APPKEY"] = "YOUR_NLS_APPKEY"
os.environ["PYTHONPATH"] = "/home/sunrise/xlerobot/src"

# 设置路径
sys.path.insert(0, '/home/sunrise/xlerobot/src')

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def test_correct_asr_system():
    """测试修复后的ASR系统"""
    print("🎯 测试正确的ASR系统音频修复")
    print("=" * 50)

    try:
        # 导入正确的ASR系统
        from modules.asr.asr_system import ASRSystem

        print("✅ ASR系统模块导入成功")

        # 创建ASR系统实例
        asr_system = ASRSystem()
        print("✅ ASR系统实例创建成功")

        # 初始化系统
        print("\n🔧 初始化ASR系统...")
        init_success = asr_system.initialize()

        if init_success:
            print("✅ ASR系统初始化成功")
        else:
            print("❌ ASR系统初始化失败")
            return False

        # 获取系统状态
        print("\n📊 系统状态:")
        status = asr_system.get_status()
        print(f"   麦克风: {'✅' if status.get('microphone', False) else '❌'}")
        print(f"   ASR服务: {'✅' if status.get('asr_service', False) else '❌'}")
        print(f"   TTS服务: {'✅' if status.get('tts_service', False) else '❌'}")

        # 测试录音功能（不启动监听，只测试硬件）
        print("\n🎤 测试麦克风设备...")
        try:
            import speech_recognition as sr

            # 使用和ASR系统相同的设备设置
            microphone = sr.Microphone(device_index=2)  # sysdefault设备
            recognizer = sr.Recognizer()

            print(f"   麦克风设备: {microphone}")

            # 测试噪音调整
            with microphone as source:
                print("   正在调整环境噪音...")
                recognizer.adjust_for_ambient_noise(source, duration=1)
                print("   ✅ 环境噪音调整完成")

                # 检查音频流是否正常
                print("   测试音频流...")
                # 短暂录音测试
                audio_data = recognizer.listen(source, timeout=2, phrase_time_limit=1)
                print(f"   ✅ 音频数据采集成功 (长度: {len(audio_data.get_raw_data())} bytes)")

                # 检查音频幅度
                audio_array = audio_data.get_array()
                max_amplitude = max(abs(x) for x in audio_array)
                print(f"   音频幅度: {max_amplitude}")

                if max_amplitude > 1000:
                    print("   ✅ 音频信号强度良好")
                elif max_amplitude > 100:
                    print("   ⚠️ 音频信号较弱")
                else:
                    print("   ❌ 音频信号太弱")

        except Exception as e:
            print(f"   ❌ 麦克风测试失败: {e}")
            return False

        print("\n🎉 正确的ASR系统音频修复验证完成")
        return True

    except Exception as e:
        print(f"❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_correct_asr_system()
    if success:
        print("\n✅ 音频修复验证成功，可以启动完整的语音服务")
        print("   启动命令: ./start_voice_assistant.sh")
    else:
        print("\n❌ 音频修复验证失败，需要进一步调试")