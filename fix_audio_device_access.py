#!/usr/bin/env python3.10
"""
音频设备访问问题修复脚本
=======================

解决ThreadSafeAudioRecorder的音频设备访问问题：
1. 动态检测可用的音频输入设备
2. 选择最佳设备配置
3. 测试设备可访问性

作者: Claude Code Agent
日期: 2025-11-18
"""

import pyaudio
import sys
import time
import numpy as np

def list_audio_devices():
    """列出所有可用的音频设备"""
    print("🔍 检测可用音频设备...")

    p = pyaudio.PyAudio()

    print("\n📋 音频输入设备列表:")
    print("=" * 60)

    input_devices = []
    for i in range(p.get_device_count()):
        info = p.get_device_info_by_index(i)
        if info['maxInputChannels'] > 0:
            input_devices.append({
                'index': i,
                'name': info['name'],
                'channels': info['maxInputChannels'],
                'sample_rate': int(info['defaultSampleRate'])
            })
            print(f"设备 {i}: {info['name']}")
            print(f"  输入通道: {info['maxInputChannels']}")
            print(f"  默认采样率: {info['defaultSampleRate']}Hz")
            print(f"  API: {info.get('hostApiLongName', 'Unknown')}")
            print("-" * 40)

    p.terminate()
    return input_devices

def test_device_access(device_index, sample_rate=16000, channels=1, duration=2):
    """测试特定设备的访问能力"""
    print(f"\n🎵 测试设备 {device_index} (采样率: {sample_rate}Hz, 通道: {channels})")

    p = pyaudio.PyAudio()

    try:
        stream = p.open(
            format=pyaudio.paInt16,
            channels=channels,
            rate=sample_rate,
            input=True,
            input_device_index=device_index,
            frames_per_buffer=1024
        )

        print(f"✅ 设备 {device_index} 打开成功")

        # 尝试录制音频
        print(f"🎤 录制 {duration} 秒音频...")
        frames = []

        for i in range(0, int(sample_rate / 1024 * duration)):
            data = stream.read(1024, exception_on_overflow=False)
            frames.append(data)

        stream.stop_stream()
        stream.close()

        print(f"✅ 音频录制成功，共 {len(frames)} 帧")

        # 分析音频数据
        audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)
        max_amplitude = np.max(np.abs(audio_data))
        rms = np.sqrt(np.mean(audio_data.astype(float) ** 2))

        print(f"📊 音频分析:")
        print(f"  数据长度: {len(audio_data)} samples")
        print(f"  最大幅度: {max_amplitude}")
        print(f"  RMS值: {rms:.2f}")
        print(f"  时长: {len(audio_data)/sample_rate:.2f}秒")

        if rms > 100:
            print("✅ 音频质量良好")
            success = True
        else:
            print("⚠️ 音频信号较弱，可能是静音")
            success = True  # 设备可访问，只是没有声音

    except Exception as e:
        print(f"❌ 设备访问失败: {e}")
        success = False

    finally:
        p.terminate()

    return success

def find_best_device():
    """找到最佳音频输入设备"""
    devices = list_audio_devices()

    if not devices:
        print("❌ 未找到可用的音频输入设备")
        return None

    print(f"\n🔬 测试 {len(devices)} 个设备的访问能力...")

    best_device = None
    best_score = -1

    for device in devices:
        success = test_device_access(
            device['index'],
            sample_rate=16000,  # ASR要求的采样率
            channels=1,          # ASR要求的通道数
            duration=1
        )

        if success:
            # 评分系统
            score = 0
            if device['sample_rate'] == 16000:
                score += 50  # 采样率匹配
            if 'USB' in device['name']:
                score += 30  # USB设备通常质量更好
            if device['channels'] >= 2:
                score += 20  # 支持多通道

            print(f"📊 设备 {device['index']} 评分: {score}")

            if score > best_score:
                best_score = score
                best_device = device

    if best_device:
        print(f"\n🎯 推荐使用设备 {best_device['index']}: {best_device['name']}")
        print(f"   评分: {best_score}")
        print(f"   支持采样率: {best_device['sample_rate']}Hz")
        print(f"   支持通道: {best_device['channels']}")

        return best_device
    else:
        print("❌ 所有设备都无法访问")
        return None

def create_fix_script(best_device):
    """创建修复脚本"""
    if not best_device:
        print("❌ 无法创建修复脚本：没有可用设备")
        return

    fix_code = f'''#!/usr/bin/env python3.10
"""
自动生成的音频设备修复配置
========================

最佳设备配置:
- 设备索引: {best_device['index']}
- 设备名称: {best_device['name']}
- 采样率: 16000Hz
- 通道: 1
"""

# 修复ThreadSafeAudioRecorder的设备索引
AUDIO_DEVICE_INDEX = {best_device['index']}
AUDIO_DEVICE_NAME = "{best_device['name']}"

def get_optimal_audio_config():
    """获取优化的音频配置"""
    return {{
        'device_index': AUDIO_DEVICE_INDEX,
        'device_name': AUDIO_DEVICE_NAME,
        'sample_rate': 16000,
        'channels': 1,
        'format': 'int16',
        'chunk_size': 1024
    }}

print("✅ 音频设备配置已优化")
print(f"📋 使用设备: {{AUDIO_DEVICE_NAME}} (索引: {{AUDIO_DEVICE_INDEX}})")
'''

    fix_file_path = "/home/sunrise/xlerobot/audio_device_config.py"
    with open(fix_file_path, 'w', encoding='utf-8') as f:
        f.write(fix_code)

    print(f"✅ 修复配置已保存: {fix_file_path}")
    return fix_file_path

def main():
    """主函数"""
    print("🚀 开始音频设备问题诊断和修复...")

    # 1. 列出设备
    devices = list_audio_devices()

    if not devices:
        print("❌ 未找到音频输入设备，请检查硬件连接")
        return False

    # 2. 测试设备访问
    best_device = find_best_device()

    if best_device:
        # 3. 创建修复配置
        fix_file = create_fix_script(best_device)

        print(f"\n🎉 音频设备问题诊断完成!")
        print(f"📋 建议使用设备: {best_device['name']} (索引: {best_device['index']})")
        print(f"📄 修复配置: {fix_file}")
        print(f"\n📌 下一步操作:")
        print(f"1. 修改ThreadSafeAudioRecorder使用设备索引 {best_device['index']}")
        print(f"2. 重新运行唤醒词检测测试")

        return True
    else:
        print("\n❌ 音频设备访问存在问题，无法找到可用设备")
        print("📌 可能的解决方案:")
        print("1. 检查音频设备权限")
        print("2. 重启音频服务")
        print("3. 检查设备连接")

        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)