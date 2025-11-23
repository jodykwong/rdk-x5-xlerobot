#!/usr/bin/env python3
"""
简化真实音频测试 - 严禁Mock数据
==============================

⚠️ 严禁Mock数据声明：
- 使用真实USB麦克风音频输入
- 禁止任何模拟或硬编码音频数据
- 确保所有音频数据来自真实麦克风
- 使用真实扬声器输出

严格使用真实音频设备进行基础功能验证。
"""

import sys
import os
import time
import numpy as np
import pyaudio
import wave
import logging

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


def test_real_audio_recording():
    """测试真实音频录制"""
    logger.info("🎤 测试真实音频录制")
    logger.info("⚠️ 严禁Mock数据 - 使用真实USB麦克风")

    try:
        # 创建PyAudio实例
        audio = pyaudio.PyAudio()

        # 使用USB音频设备
        input_device = 0  # USB Audio Device
        logger.info(f"使用USB音频设备: {input_device}")

        # 音频参数
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 16000
        CHUNK = 1024

        logger.info(f"音频参数: {RATE}Hz, {CHANNELS}通道, 16-bit")

        # 打开音频流
        stream = audio.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=RATE,
            input=True,
            input_device_index=input_device,
            frames_per_buffer=CHUNK
        )

        logger.info("✅ 音频流已打开")

        # 录制音频
        logger.info("🎤 开始录制3秒音频...")
        logger.info("请对着USB麦克风说话，例如：'测试语音'")

        frames = []
        for i in range(0, int(RATE / CHUNK * 3)):
            data = stream.read(CHUNK, exception_on_overflow=False)
            frames.append(data)
            if i % 50 == 0:  # 每半秒显示进度
                progress = (i + 1) / (int(RATE / CHUNK * 3)) * 100
                print(f"\r录制进度: {progress:.1f}%", end="", flush=True)

        print()  # 换行
        logger.info("✅ 录制完成")

        # 停止音频流
        stream.stop_stream()
        stream.close()
        audio.terminate()

        # 转换为numpy数组
        audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)

        logger.info(f"✅ 音频数据: {len(audio_data)}样本")
        logger.info(f"✅ 录制时长: {len(audio_data)/RATE:.2f}秒")

        return audio_data

    except Exception as e:
        logger.error(f"❌ 录音失败: {e}")
        return None


def test_real_audio_playback(audio_data):
    """测试真实音频播放"""
    if audio_data is None:
        logger.error("❌ 无音频数据可播放")
        return False

    logger.info("🔊 测试真实音频播放")
    logger.info("⚠️ 严禁Mock数据 - 使用真实扬声器")

    try:
        # 创建PyAudio实例
        audio = pyaudio.PyAudio()

        # 使用USB音频设备
        output_device = 0  # USB Audio Device
        logger.info(f"使用USB音频设备: {output_device}")

        # 音频参数
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 16000

        # 打开音频流
        stream = audio.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=RATE,
            output=True,
            output_device_index=output_device
        )

        logger.info("✅ 播放流已打开")

        # 播放音频
        logger.info("🔊 播放录制的音频...")
        stream.write(audio_data.tobytes())

        # 停止音频流
        stream.stop_stream()
        stream.close()
        audio.terminate()

        logger.info("✅ 播放完成")
        return True

    except Exception as e:
        logger.error(f"❌ 播放失败: {e}")
        return False


def test_audio_processing(audio_data):
    """测试音频处理"""
    if audio_data is None:
        logger.error("❌ 无音频数据可处理")
        return False

    logger.info("🔄 测试音频处理")
    logger.info("⚠️ 严禁Mock数据 - 处理真实录制音频")

    try:
        # 基础音频分析
        sample_rate = 16000

        # 计算音频统计
        mean_value = np.mean(np.abs(audio_data))
        max_value = np.max(np.abs(audio_data))

        logger.info(f"✅ 平均音量: {mean_value:.2f}")
        logger.info(f"✅ 最大音量: {max_value:.2f}")

        # 检查是否有有效音频
        if mean_value < 100:
            logger.warning("⚠️ 音频音量较低，可能未检测到声音")
        else:
            logger.info("✅ 检测到有效音频信号")

        # 简单的语音活动检测
        is_speech = mean_value > 500
        logger.info(f"✅ 语音活动检测: {'检测到语音' if is_speech else '未检测到语音'}")

        return True

    except Exception as e:
        logger.error(f"❌ 音频处理失败: {e}")
        return False


def main():
    """主函数"""
    print("=" * 60)
    print("🚫 严禁Mock数据声明")
    print("本测试严格使用真实USB音频设备")
    print("禁止任何模拟、Mock或硬编码数据")
    print("=" * 60)
    print()

    # 测试1: 录制真实音频
    logger.info("🧪 测试1: 真实音频录制")
    audio_data = test_real_audio_recording()

    if audio_data is None:
        logger.error("❌ 测试1失败，无法继续")
        return False

    # 测试2: 音频处理
    logger.info("🧪 测试2: 音频处理分析")
    process_success = test_audio_processing(audio_data)

    # 测试3: 音频播放
    logger.info("🧪 测试3: 真实音频播放")
    playback_success = test_real_audio_playback(audio_data)

    # 生成测试报告
    print("\n" + "=" * 50)
    print("📊 真实音频测试结果:")
    print(f"✅ 音频录制: {'成功' if audio_data is not None else '失败'}")
    print(f"✅ 音频处理: {'成功' if process_success else '失败'}")
    print(f"✅ 音频播放: {'成功' if playback_success else '失败'}")
    print(f"✅ 数据来源: 真实USB麦克风")
    print(f"✅ 输出设备: 真实USB扬声器")
    print(f"✅ Mock数据: 严格禁止")

    # 判断测试结果
    success = (audio_data is not None and process_success and playback_success)

    if success:
        print("\n🎉 真实音频测试成功！")
        print("✅ 所有功能使用真实USB音频设备")
        print("✅ 无任何Mock或模拟数据")
        print("✅ Story 1.1基础音频功能验证通过")
    else:
        print("\n❌ 真实音频测试失败")
        print("请检查USB音频设备连接")

    return success


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)