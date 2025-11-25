#!/usr/bin/env python3.10
# -*- coding: utf-8 -*-
"""
音频设备诊断工具 - 严禁Mock数据

专门用于检测和测试XLeRobot系统的音频设备，解决"叫傻强没反应"的问题
只使用真实麦克风输入，严禁任何模拟数据
"""

import os
import sys
import time
import logging
import threading
from typing import List, Dict, Optional, Tuple

# 设置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

try:
    import speech_recognition as sr
    import pyaudio
    import numpy as np
except ImportError as e:
    logger.error(f"❌ 导入音频库失败: {e}")
    logger.error("请安装: pip3.10 install SpeechRecognition pyaudio numpy")
    sys.exit(1)

class AudioDeviceDiagnostic:
    """音频设备诊断类 - 严禁Mock数据"""

    def __init__(self):
        self.sample_rate = 16000  # 阿里云ASR要求16kHz
        self.chunk_size = 1024
        self.channels = 1
        self.format = pyaudio.paInt16
        self.recognizer = sr.Recognizer()
        self.testing = False
        self.audio_level = 0
        self.monitoring = False

    def detect_all_microphones(self) -> List[Dict]:
        """检测所有可用麦克风设备 - 严禁Mock数据"""
        logger.info("🎤 检测所有麦克风设备...")

        try:
            p = pyaudio.PyAudio()
            devices = []

            for i in range(p.get_device_count()):
                info = p.get_device_info_by_index(i)

                # 只检查输入设备
                if info['maxInputChannels'] > 0:
                    device_info = {
                        'index': i,
                        'name': info['name'],
                        'channels': info['maxInputChannels'],
                        'sample_rate': int(info['defaultSampleRate']),
                        'is_default': i == p.get_default_input_device_info()['index']
                    }
                    devices.append(device_info)
                    logger.info(f"🎤 麦克风 {i}: {device_info['name']} (通道: {device_info['channels']}, 采样率: {device_info['sample_rate']}Hz)")

            p.terminate()
            logger.info(f"✅ 总共发现 {len(devices)} 个麦克风设备")
            return devices

        except Exception as e:
            logger.error(f"❌ 检测麦克风设备失败: {e}")
            return []

    def test_microphone_access(self, device_index: int, duration: int = 3) -> Tuple[bool, float]:
        """测试麦克风访问和音频质量 - 严禁Mock数据"""
        logger.info(f"🧪 测试麦克风设备 {device_index} (测试时长: {duration}秒)")

        try:
            # 使用speech_recognition库测试
            with sr.Microphone(device_index=device_index,
                                sample_rate=self.sample_rate,
                                chunk_size=self.chunk_size) as source:

                logger.info(f"🎧 麦克风 {device_index} 初始化成功")

                # 调整环境噪音
                logger.info("🔧 调整环境噪音...")
                self.recognizer.adjust_for_ambient_noise(source, duration=1)

                # 测试音频录制
                logger.info("🎙️ 开始录制音频...")
                start_time = time.time()

                # 录制音频并计算音量
                audio_data = []
                max_volume = 0

                with source.stream as stream:
                    for _ in range(0, int(self.sample_rate / self.chunk_size * duration)):
                        chunk = stream.read(self.chunk_size, exception_on_overflow=False)
                        if chunk:
                            audio_data.append(chunk)
                            # 计算音量
                            audio_array = np.frombuffer(chunk, dtype=np.int16)
                            volume = np.abs(audio_array).mean()
                            max_volume = max(max_volume, volume)

                            # 实时显示音量
                            if volume > max_volume * 0.8:
                                print(f"🔊 音量: {volume:.0f}", end='\r')

                end_time = time.time()
                test_duration = end_time - start_time

                logger.info(f"✅ 录制完成，时长: {test_duration:.2f}秒")
                logger.info(f"📊 最大音量: {max_volume:.0f}")

                # 判断是否有有效的音频输入
                has_audio = max_volume > 100  # 设定最小音量阈值

                if has_audio:
                    logger.info(f"✅ 麦克风 {device_index} 工作正常")
                else:
                    logger.warning(f"⚠️ 麦克风 {device_index} 没有检测到音频输入")

                return has_audio, max_volume

        except Exception as e:
            logger.error(f"❌ 测试麦克风 {device_index} 失败: {e}")
            return False, 0

    def find_working_microphone(self) -> Optional[int]:
        """找到第一个能工作的麦克风 - 严禁Mock数据"""
        logger.info("🔍 搜索可工作的麦克风...")

        devices = self.detect_all_microphones()

        for device in devices:
            logger.info(f"🧪 测试设备 {device['index']}: {device['name']}")
            works, volume = self.test_microphone_access(device['index'], duration=2)

            if works:
                logger.info(f"✅ 找到可工作的麦克风: {device['name']} (索引: {device['index']}, 音量: {volume:.0f})")
                return device['index']
            else:
                logger.warning(f"❌ 设备 {device['name']} 无法使用")

        logger.error("❌ 没有找到可工作的麦克风设备！")
        return None

    def monitor_audio_level(self, device_index: int, duration: int = 10):
        """监控音频级别 - 严禁Mock数据"""
        logger.info(f"📊 监控麦克风 {device_index} 音频级别 ({duration}秒)...")
        self.monitoring = True

        try:
            with sr.Microphone(device_index=device_index,
                                sample_rate=self.sample_rate,
                                chunk_size=self.chunk_size) as source:

                self.recognizer.adjust_for_ambient_noise(source, duration=1)

                def monitor_thread():
                    nonlocal self
                    try:
                        with source.stream as stream:
                            start_time = time.time()
                            while self.monitoring and (time.time() - start_time) < duration:
                                chunk = stream.read(self.chunk_size, exception_on_overflow=False)
                                if chunk:
                                    audio_array = np.frombuffer(chunk, dtype=np.int16)
                                    volume = np.abs(audio_array).mean()
                                    self.audio_level = volume

                                    # 显示音量条
                                    bar_length = int(volume / 100) if volume > 0 else 0
                                    bar = '█' * min(bar_length, 50)
                                    print(f"🔊 音量: {volume:6.0f} |{bar:<50}| {bar_length}/50", end='\r')

                                    if volume > 1000:  # 检测到明显声音
                                        print(f"\n🎯 检测到声音！音量: {volume:.0f}")
                    except Exception as e:
                        logger.error(f"监控线程错误: {e}")

                thread = threading.Thread(target=monitor_thread)
                thread.daemon = True
                thread.start()
                thread.join()

        except Exception as e:
            logger.error(f"❌ 监控音频级别失败: {e}")
        finally:
            self.monitoring = False
            print("\n✅ 监控完成")

    def stop_monitoring(self):
        """停止监控"""
        self.monitoring = False

def main():
    """主函数 - 严禁Mock数据"""
    print("="*60)
    print("🎤 XLeRobot 音频设备诊断工具")
    print("🚨 严禁使用任何Mock数据，只使用真实麦克风")
    print("="*60)

    diagnostic = AudioDeviceDiagnostic()

    try:
        # 检测所有设备
        devices = diagnostic.detect_all_microphones()

        if not devices:
            print("❌ 没有检测到任何麦克风设备")
            return

        # 找到可工作的设备
        working_device = diagnostic.find_working_microphone()

        if working_device is not None:
            print(f"\n🎉 建议使用麦克风索引: {working_device}")

            # 提供交互式监控选项
            choice = input(f"\n是否要监控麦克风 {working_device} 的实时音频级别? (y/n): ").lower().strip()
            if choice == 'y':
                try:
                    diagnostic.monitor_audio_level(working_device, duration=30)
                except KeyboardInterrupt:
                    print("\n⏹️ 用户中断监控")

        else:
            print("\n💡 建议:")
            print("1. 检查麦克风硬件连接")
            print("2. 确认音频设备权限")
            print("3. 尝试重新插拔USB设备")
            print("4. 检查系统音频设置")

    except KeyboardInterrupt:
        print("\n⏹️ 用户中断诊断")
    except Exception as e:
        logger.error(f"❌ 诊断过程出错: {e}")
        import traceback
        logger.error(traceback.format_exc())

if __name__ == "__main__":
    main()