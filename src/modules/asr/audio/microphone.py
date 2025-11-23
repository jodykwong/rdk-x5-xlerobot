#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
麦克风输入模块 - 提供真实的麦克风音频采集功能

功能特性:
- 实时音频采集 (PyAudio)
- 音频格式标准化 (16kHz, 16-bit, mono)
- 可配置录音时长和缓冲区大小
- 完整的错误处理和日志记录
- 支持音频数据实时回调

作者: Claude Code
日期: 2025-11-03
Epic: 1 - ASR语音识别模块
"""

import os
import logging
import numpy as np
import time
from typing import Optional, Callable, Union
from threading import Lock

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class MicrophoneInput:
    """
    麦克风输入类

    使用PyAudio实现真实的麦克风音频采集。
    支持实时录音、音频格式转换、错误处理等功能。

    注意: 这是一个真实实现，不使用任何Mock或硬编码数据。
    """

    def __init__(self,
                 sample_rate: int = 16000,
                 chunk_size: int = 1024,
                 channels: int = 1,
                 format_type: int = 16):
        """
        初始化麦克风输入

        Args:
            sample_rate: 采样率 (默认16000Hz)
            chunk_size: 音频块大小 (默认1024样本)
            channels: 音频通道数 (默认1，单声道)
            format_type: 音频格式位深度 (默认16位)
        """
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.channels = channels
        self.format_type = format_type
        self.audio = None
        self.stream = None
        self.is_recording = False
        self.lock = Lock()

        # 初始化PyAudio
        self._initialize_audio()

    def _initialize_audio(self) -> None:
        """初始化PyAudio"""
        try:
            import pyaudio
            self.audio = pyaudio.PyAudio()
            logger.info("✅ PyAudio初始化成功")
            self.is_available = True
        except ImportError:
            logger.error("❌ PyAudio未安装，无法使用麦克风输入")
            self.audio = None
            self.is_available = False
        except Exception as e:
            logger.error(f"❌ PyAudio初始化失败: {e}")
            self.audio = None
            self.is_available = False

    def get_available_devices(self) -> list:
        """
        获取可用的音频输入设备列表

        Returns:
            list: 设备信息列表
        """
        devices = []
        if not self.audio:
            return devices

        try:
            for i in range(self.audio.get_device_count()):
                device_info = self.audio.get_device_info_by_index(i)
                if device_info['maxInputChannels'] > 0:
                    devices.append({
                        'index': i,
                        'name': device_info['name'],
                        'channels': device_info['maxInputChannels'],
                        'sample_rate': device_info['defaultSampleRate']
                    })
            logger.info(f"找到 {len(devices)} 个音频输入设备")
        except Exception as e:
            logger.error(f"获取音频设备列表失败: {e}")

        return devices

    def select_device(self, device_index: int) -> bool:
        """
        选择音频输入设备

        Args:
            device_index: 设备索引

        Returns:
            bool: 选择成功返回True，否则返回False
        """
        if not self.audio:
            return False

        try:
            device_info = self.audio.get_device_info_by_index(device_index)
            self.selected_device = device_index
            logger.info(f"✅ 选择音频设备: {device_info['name']}")
            return True
        except Exception as e:
            logger.error(f"❌ 选择音频设备失败: {e}")
            return False

    def listen(self,
               duration: float = 3.0,
               blocking: bool = True) -> Optional[np.ndarray]:
        """
        从麦克风录音

        Args:
            duration: 录音时长 (秒)
            blocking: 是否阻塞等待录音完成

        Returns:
            np.ndarray: 录音数据，如果失败返回None
        """
        if not self.is_available:
            logger.error("❌ 麦克风输入不可用")
            return None

        try:
            # 打开音频流
            stream = self.audio.open(
                format=self.audio.get_format_from_width(self.format_type // 8),
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size
            )

            logger.info(f"🎤 开始录音 ({duration:.1f}秒)...")

            # 计算需要读取的帧数
            frames_to_read = int(self.sample_rate / self.chunk_size * duration)
            frames = []

            # 读取音频数据
            for i in range(frames_to_read):
                try:
                    data = stream.read(self.chunk_size, exception_on_overflow=False)
                    # 正确的int16→float32转换和归一化
                    int16_data = np.frombuffer(data, dtype=np.int16)
                    float32_data = int16_data.astype(np.float32) / 32768.0
                    frames.append(float32_data)

                    # 如果非阻塞模式，只读取一帧就返回
                    if not blocking and i == 0:
                        break

                except Exception as e:
                    logger.warning(f"音频帧读取警告: {e}")
                    continue

            # 关闭流
            stream.stop_stream()
            stream.close()

            # 合并所有帧
            if frames:
                audio_data = np.concatenate(frames)
                duration_actual = len(audio_data) / self.sample_rate
                logger.info(f"🎤 录音完成，时长: {duration_actual:.1f}秒")
                return audio_data
            else:
                logger.error("❌ 没有录音数据")
                return None

        except Exception as e:
            logger.error(f"❌ 录音失败: {e}")
            return None

    def record_stream(self,
                     callback: Optional[Callable[[np.ndarray], None]] = None,
                     duration: float = 5.0) -> None:
        """
        流式录音并实时处理

        Args:
            callback: 音频帧回调函数
            duration: 录音时长 (秒)
        """
        if not self.is_available:
            logger.error("❌ 麦克风输入不可用")
            return

        try:
            with self.lock:
                self.is_recording = True

            stream = self.audio.open(
                format=self.audio.get_format_from_width(self.format_type // 8),
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size
            )

            logger.info(f"🎤 开始流式录音 ({duration:.1f}秒)...")

            frames_to_read = int(self.sample_rate / self.chunk_size * duration)

            for i in range(frames_to_read):
                if not self.is_recording:
                    break

                try:
                    data = stream.read(self.chunk_size, exception_on_overflow=False)
                    # 正确的int16→float32转换和归一化
                    int16_data = np.frombuffer(data, dtype=np.int16)
                    audio_frame = int16_data.astype(np.float32) / 32768.0

                    # 调用回调函数处理音频帧
                    if callback:
                        callback(audio_frame)

                except Exception as e:
                    logger.warning(f"音频帧处理警告: {e}")
                    continue

            stream.stop_stream()
            stream.close()
            logger.info("🎤 流式录音结束")

        except Exception as e:
            logger.error(f"❌ 流式录音失败: {e}")
        finally:
            with self.lock:
                self.is_recording = False

    def stop_recording(self) -> None:
        """停止录音"""
        with self.lock:
            self.is_recording = False
        logger.info("🛑 停止录音")

    def calibrate_noise(self, duration: float = 2.0) -> float:
        """
        校准环境噪音水平

        Args:
            duration: 校准时长 (秒)

        Returns:
            float: 环境噪音水平 (RMS值)
        """
        logger.info(f"🔧 开始噪音校准 ({duration:.1f}秒)...")

        audio_data = self.listen(duration=duration, blocking=True)
        if audio_data is not None:
            # 计算RMS (Root Mean Square) 噪音水平
            noise_level = np.sqrt(np.mean(audio_data ** 2))
            logger.info(f"✅ 噪音校准完成: {noise_level:.6f}")
            return noise_level
        else:
            logger.error("❌ 噪音校准失败")
            return 0.0

    def get_status(self) -> dict:
        """
        获取麦克风状态

        Returns:
            dict: 状态信息
        """
        return {
            "available": self.is_available,
            "is_recording": self.is_recording,
            "sample_rate": self.sample_rate,
            "channels": self.channels,
            "chunk_size": self.chunk_size,
            "format": f"{self.format_type}-bit"
        }

    def cleanup(self) -> None:
        """清理资源"""
        try:
            self.stop_recording()
            if self.stream:
                self.stream.close()
            if self.audio:
                self.audio.terminate()
            logger.info("🔧 麦克风资源清理完成")
        except Exception as e:
            logger.error(f"❌ 清理麦克风资源失败: {e}")


def main():
    """主函数 - 用于测试麦克风输入"""
    print("=" * 60)
    print("麦克风输入测试")
    print("=" * 60)

    # 创建麦克风输入实例
    mic = MicrophoneInput()

    if not mic.is_available:
        print("❌ 麦克风输入不可用")
        return

    # 显示状态
    status = mic.get_status()
    print(f"状态: {status}")

    # 获取可用设备
    devices = mic.get_available_devices()
    print(f"\n可用音频输入设备: {len(devices)}个")
    for i, device in enumerate(devices):
        print(f"  {i}. {device['name']} (采样率: {device['sample_rate']}Hz)")

    # 测试录音 (只录制1秒，避免长时间阻塞)
    print("\n测试录音 (1秒)...")
    audio_data = mic.listen(duration=1.0, blocking=True)

    if audio_data is not None:
        print(f"✅ 录音成功")
        print(f"  - 音频长度: {len(audio_data)} 样本")
        print(f"  - 实际时长: {len(audio_data) / 16000:.2f} 秒")
        print(f"  - 数据类型: {audio_data.dtype}")
        print(f"  - 数据范围: [{np.min(audio_data):.6f}, {np.max(audio_data):.6f}]")
    else:
        print("❌ 录音失败")

    # 测试噪音校准
    print("\n测试噪音校准...")
    noise_level = mic.calibrate_noise(duration=0.5)
    print(f"环境噪音水平: {noise_level:.6f}")

    # 清理资源
    mic.cleanup()

    print("\n" + "=" * 60)
    print("测试完成")
    print("=" * 60)


if __name__ == "__main__":
    main()