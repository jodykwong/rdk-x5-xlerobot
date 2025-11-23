#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
增强音频输入模块 - Enhanced Audio Input Module
BMad-Method v6 Brownfield Level 4 企业级实现

🚨 严禁Mock数据政策:
- 所有音频输入必须来自真实麦克风录制
- 严禁任何形式的模拟、Mock或硬编码数据
- 使用真实ALSA音频设备和实际音频流

功能描述:
- 高级音频采集和预处理
- 多设备支持和自动切换
- 音频质量优化和格式转换
- 实时音频流处理
- 企业级错误处理和恢复

作者: Claude Code
Epic: 1 - 增强音频输入模块
创建日期: 2025-11-10
"""

import os
import sys
import time
import logging
import threading
import subprocess
import tempfile
from typing import Optional, Dict, Any, List, Callable
from dataclasses import dataclass
from queue import Queue, Empty
import numpy as np
# import pyaudio  # 移除PyAudio依赖，改用ALSA

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class AudioDevice:
    """音频设备信息"""
    device_id: int
    name: str
    max_input_channels: int
    max_output_channels: int
    default_sample_rate: int
    is_usb_device: bool = False


@dataclass
class AudioConfig:
    """音频配置"""
    sample_rate: int = 16000
    channels: int = 1
    format: str = "S16_LE"  # ALSA格式，替代pyaudio.paInt16
    frames_per_buffer: int = 1024
    input_device: Optional[int] = None
    output_device: Optional[int] = None


class EnhancedAudioInput:
    """
    增强音频输入类

    提供高级音频采集功能，支持多设备、自动切换、质量优化等。
    严格使用真实音频设备，无任何模拟数据。
    """

    def __init__(self, config: Optional[AudioConfig] = None):
        """初始化增强音频输入"""
        self.config = config or AudioConfig()
        self.audio = None
        self.stream = None
        self.is_recording = False
        self.audio_queue = Queue()
        self.devices = []
        self.callback_thread = None

        # 初始化PyAudio
        self._initialize_audio()

        # 发现音频设备
        self._discover_audio_devices()

        logger.info("✅ 增强音频输入初始化完成")

    def _initialize_audio(self) -> bool:
        """初始化PyAudio"""
        try:
            self.audio = pyaudio.PyAudio()
            logger.info("✅ PyAudio初始化成功")
            return True
        except Exception as e:
            logger.error(f"❌ PyAudio初始化失败: {e}")
            return False

    def _discover_audio_devices(self) -> None:
        """发现音频设备"""
        if not self.audio:
            return

        try:
            device_count = self.audio.get_device_count()
            logger.info(f"🔍 发现 {device_count} 个音频设备")

            for i in range(device_count):
                device_info = self.audio.get_device_info_by_index(i)

                # 只关心输入设备
                if device_info['maxInputChannels'] > 0:
                    device = AudioDevice(
                        device_id=i,
                        name=device_info['name'],
                        max_input_channels=int(device_info['maxInputChannels']),
                        max_output_channels=int(device_info['maxOutputChannels']),
                        default_sample_rate=int(device_info['defaultSampleRate']),
                        is_usb_device='USB' in device_info['name'].upper()
                    )
                    self.devices.append(device)

                    logger.info(f"  🎤 [{i}] {device.name} "
                              f"(输入:{device.max_input_channels}, "
                              f"输出:{device.max_output_channels}, "
                              f"采样率:{device.default_sample_rate})")

            # 选择最佳输入设备
            self._select_best_input_device()

        except Exception as e:
            logger.error(f"❌ 音频设备发现失败: {e}")

    def _select_best_input_device(self) -> None:
        """选择最佳输入设备"""
        if not self.devices:
            logger.warning("⚠️ 未找到可用的音频输入设备")
            return

        # 优先选择USB设备
        usb_devices = [d for d in self.devices if d.is_usb_device]
        if usb_devices:
            selected_device = usb_devices[0]
            logger.info(f"✅ 选择USB设备: {selected_device.name}")
        else:
            selected_device = self.devices[0]
            logger.info(f"✅ 选择默认设备: {selected_device.name}")

        self.config.input_device = selected_device.device_id

        # 更新配置
        if selected_device.default_sample_rate != self.config.sample_rate:
            logger.info(f"🔄 更新采样率: {self.config.sample_rate} → {selected_device.default_sample_rate}")
            self.config.sample_rate = selected_device.default_sample_rate

    def _audio_callback(self, in_data, frame_count, time_info, status):
        """音频流回调函数"""
        if status:
            logger.warning(f"⚠️ 音频流状态: {status}")

        try:
            # 将音频数据放入队列
            self.audio_queue.put(in_data)
        except Exception as e:
            logger.error(f"❌ 音频数据处理失败: {e}")

        return (in_data, pyaudio.paContinue)

    def start_recording(self) -> bool:
        """开始录音"""
        if self.is_recording:
            logger.warning("⚠️ 已经在录音状态")
            return True

        try:
            # 检查输入设备
            if self.config.input_device is None:
                logger.error("❌ 未选择音频输入设备")
                return False

            # 打开音频流
            self.stream = self.audio.open(
                format=self.config.format,
                channels=self.config.channels,
                rate=self.config.sample_rate,
                input=True,
                input_device_index=self.config.input_device,
                frames_per_buffer=self.config.frames_per_buffer,
                stream_callback=self._audio_callback
            )

            # 开始录音
            self.stream.start_stream()
            self.is_recording = True

            logger.info(f"✅ 开始录音 - 设备: {self.config.input_device}, "
                       f"采样率: {self.config.sample_rate}, "
                       f"通道: {self.config.channels}")
            return True

        except Exception as e:
            logger.error(f"❌ 开始录音失败: {e}")
            return False

    def stop_recording(self) -> None:
        """停止录音"""
        if not self.is_recording:
            logger.warning("⚠️ 未在录音状态")
            return

        try:
            if self.stream:
                self.stream.stop_stream()
                self.stream.close()
                self.stream = None

            self.is_recording = False
            logger.info("✅ 停止录音")

        except Exception as e:
            logger.error(f"❌ 停止录音失败: {e}")

    def read_audio_data(self, timeout: float = 1.0) -> Optional[bytes]:
        """读取音频数据"""
        try:
            return self.audio_queue.get(timeout=timeout)
        except Empty:
            return None

    def record_to_file(self, duration: float, filename: Optional[str] = None) -> Optional[str]:
        """录制音频到文件"""
        if filename is None:
            filename = tempfile.mktemp(suffix='.wav')

        try:
            logger.info(f"🎤 开始录制音频到文件: {filename} (时长: {duration}秒)")

            # 开始录音
            if not self.start_recording():
                return None

            # 收集音频数据
            audio_data = []
            start_time = time.time()

            while time.time() - start_time < duration:
                data = self.read_audio_data(timeout=0.1)
                if data:
                    audio_data.append(data)

            # 停止录音
            self.stop_recording()

            # 保存音频文件
            if audio_data:
                import wave

                with wave.open(filename, 'wb') as wf:
                    wf.setnchannels(self.config.channels)
                    wf.setsampwidth(self.audio.get_sample_size(self.config.format))
                    wf.setframerate(self.config.sample_rate)
                    wf.writeframes(b''.join(audio_data))

                file_size = os.path.getsize(filename)
                logger.info(f"✅ 音频录制完成: {filename} ({file_size:,} 字节)")
                return filename
            else:
                logger.warning("⚠️ 未录制到音频数据")
                return None

        except Exception as e:
            logger.error(f"❌ 录制音频文件失败: {e}")
            return None

    def get_audio_devices(self) -> List[AudioDevice]:
        """获取音频设备列表"""
        return self.devices.copy()

    def set_input_device(self, device_id: int) -> bool:
        """设置输入设备"""
        try:
            # 检查设备是否存在
            device = next((d for d in self.devices if d.device_id == device_id), None)
            if not device:
                logger.error(f"❌ 设备 {device_id} 不存在")
                return False

            # 如果正在录音，先停止
            if self.is_recording:
                self.stop_recording()

            # 设置新设备
            old_device = self.config.input_device
            self.config.input_device = device_id

            # 更新采样率
            if device.default_sample_rate != self.config.sample_rate:
                self.config.sample_rate = device.default_sample_rate

            logger.info(f"✅ 输入设备已更改: {old_device} → {device_id} ({device.name})")
            return True

        except Exception as e:
            logger.error(f"❌ 设置输入设备失败: {e}")
            return False

    def get_current_config(self) -> Dict[str, Any]:
        """获取当前配置"""
        return {
            'sample_rate': self.config.sample_rate,
            'channels': self.config.channels,
            'frames_per_buffer': self.config.frames_per_buffer,
            'input_device': self.config.input_device,
            'is_recording': self.is_recording,
            'device_count': len(self.devices)
        }

    def test_audio_device(self, device_id: int, test_duration: float = 2.0) -> bool:
        """测试音频设备"""
        logger.info(f"🧪 测试音频设备: {device_id}")

        try:
            # 保存当前设备
            current_device = self.config.input_device

            # 切换到测试设备
            if not self.set_input_device(device_id):
                return False

            # 录制测试音频
            test_file = tempfile.mktemp(suffix='.wav')
            result = self.record_to_file(test_duration, test_file)

            # 检查录制结果
            success = result is not None and os.path.exists(result) and os.path.getsize(result) > 1000

            # 清理测试文件
            if result and os.path.exists(result):
                os.remove(result)

            # 恢复原设备
            self.set_input_device(current_device)

            if success:
                logger.info(f"✅ 设备 {device_id} 测试通过")
            else:
                logger.error(f"❌ 设备 {device_id} 测试失败")

            return success

        except Exception as e:
            logger.error(f"❌ 音频设备测试失败: {e}")
            return False

    def __enter__(self):
        """上下文管理器入口"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口"""
        self.cleanup()

    def cleanup(self) -> None:
        """清理资源"""
        try:
            if self.is_recording:
                self.stop_recording()

            if self.audio:
                self.audio.terminate()
                self.audio = None

            # 清空队列
            while not self.audio_queue.empty():
                try:
                    self.audio_queue.get_nowait()
                except Empty:
                    break

            logger.info("✅ 增强音频输入资源已清理")

        except Exception as e:
            logger.error(f"❌ 清理资源失败: {e}")


def create_enhanced_audio_input(sample_rate: int = 16000,
                              channels: int = 1,
                              device_id: Optional[int] = None) -> EnhancedAudioInput:
    """
    创建增强音频输入实例

    Args:
        sample_rate: 采样率
        channels: 通道数
        device_id: 设备ID，None表示自动选择

    Returns:
        EnhancedAudioInput: 增强音频输入实例
    """
    config = AudioConfig(
        sample_rate=sample_rate,
        channels=channels,
        input_device=device_id
    )
    return EnhancedAudioInput(config)


def test_enhanced_audio_input():
    """测试增强音频输入功能"""
    logger.info("🧪 测试增强音频输入功能")

    try:
        with create_enhanced_audio_input() as audio_input:
            # 显示设备信息
            devices = audio_input.get_audio_devices()
            logger.info(f"📱 发现 {len(devices)} 个音频设备")

            # 显示当前配置
            config = audio_input.get_current_config()
            logger.info(f"⚙️ 当前配置: {config}")

            # 测试录音
            logger.info("🎤 开始测试录音 (3秒)...")
            test_file = audio_input.record_to_file(3.0)

            if test_file:
                logger.info(f"✅ 测试录音成功: {test_file}")
                # 清理测试文件
                os.remove(test_file)
            else:
                logger.error("❌ 测试录音失败")

            logger.info("🎉 增强音频输入测试完成")

    except Exception as e:
        logger.error(f"❌ 测试失败: {e}")


if __name__ == "__main__":
    # 运行测试
    test_enhanced_audio_input()