#!/usr/bin/env python3.10
# -*- coding: utf-8 -*-

"""
ALSA音频输入模块 - ALSA Audio Input Module
BMad-Method v6 Brownfield Level 4 企业级实现

🚨 严禁Mock数据政策:
- 所有音频输入必须来自真实麦克风录制
- 严禁任何形式的模拟、Mock或硬编码数据
- 使用真实ALSA音频设备和实际音频流

功能描述:
- 高级音频采集和预处理
- 多设备支持和自动切换
- PyAudio + ALSA 双引擎支持
- 音频质量优化和格式转换
- 实时音频流处理
- 企业级错误处理和恢复

作者: Claude Code
Epic: 1 - ALSA音频输入兼容性修复
创建日期: 2025-11-19
"""

import os
import sys
import time
import logging
import threading
import subprocess
import tempfile
import wave
import struct
from typing import Optional, Dict, Any, List, Callable, Union
from dataclasses import dataclass
from queue import Queue, Empty
import numpy as np

# 尝试导入PyAudio，失败时使用ALSA
try:
    import pyaudio
    HAS_PYAUDIO = True
except ImportError:
    HAS_PYAUDIO = False
    logging.warning("⚠️ PyAudio未安装，将使用ALSA备选方案")

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
    device_type: str = "unknown"  # "pyaudio" or "alsa"


@dataclass
class AudioConfig:
    """音频配置"""
    sample_rate: int = 16000
    channels: int = 1
    format: str = "S16_LE"  # ALSA格式
    frames_per_buffer: int = 1024
    input_device: Optional[str] = None  # 可以是设备ID或ALSA设备名
    output_device: Optional[str] = None


class ALSAAudioInput:
    """ALSA音频输入引擎"""

    def __init__(self):
        self.recording_process = None
        self.audio_file = None
        self.is_recording = False

    def discover_devices(self) -> List[AudioDevice]:
        """发现ALSA音频设备"""
        devices = []

        try:
            # 使用arecord -l获取设备列表
            result = subprocess.run(['arecord', '-l'],
                                  capture_output=True, text=True, timeout=5)

            if result.returncode == 0:
                lines = result.stdout.split('\n')
                current_device = None

                for line in lines:
                    line = line.strip()
                    if line.startswith('card'):
                        # 解析卡号和设备号
                        parts = line.split(':')
                        if len(parts) >= 2:
                            card_num = int(parts[0].split()[-1])
                            device_info = parts[1].strip()
                            if device_info.startswith('device'):
                                device_num = int(device_info.split()[1][:-1])
                                device_name = ' '.join(device_info.split()[2:])

                                # 构建设备ID
                                device_id = f"hw:{card_num},{device_num}"

                                device = AudioDevice(
                                    device_id=len(devices),  # 使用序列号作为ID
                                    name=f"{device_name} ({device_id})",
                                    max_input_channels=2,  # 假设支持2通道
                                    max_output_channels=0,
                                    default_sample_rate=44100,
                                    is_usb_device='USB' in device_name.upper(),
                                    device_type="alsa"
                                )
                                devices.append(device)
                                logger.info(f"  🎤 ALSA设备: {device.name}")

            # 如果没有找到设备，添加默认设备
            if not devices:
                default_device = AudioDevice(
                    device_id=0,
                    name="Default ALSA Device (default)",
                    max_input_channels=1,
                    max_output_channels=0,
                    default_sample_rate=16000,
                    is_usb_device=False,
                    device_type="alsa"
                )
                devices.append(default_device)
                logger.info("  🎤 添加默认ALSA设备")

        except Exception as e:
            logger.error(f"❌ ALSA设备发现失败: {e}")
            # 添加默认设备作为备选
            default_device = AudioDevice(
                device_id=0,
                name="Fallback ALSA Device",
                max_input_channels=1,
                max_output_channels=0,
                default_sample_rate=16000,
                is_usb_device=False,
                device_type="alsa"
            )
            devices.append(default_device)

        return devices

    def start_recording(self, config: AudioConfig) -> bool:
        """开始录音"""
        if self.is_recording:
            logger.warning("⚠️ 已经在录音状态")
            return True

        try:
            # 创建临时音频文件
            self.audio_file = tempfile.mktemp(suffix='.wav')

            # 选择ALSA设备
            device_name = "default"
            if config.input_device:
                # 如果是数字，使用hw格式
                try:
                    device_id = int(config.input_device)
                    device_name = f"hw:{device_id}"
                except (ValueError, TypeError):
                    # 如果是字符串，直接使用
                    device_name = config.input_device

            # 构建arecord命令
            cmd = [
                'arecord',
                '-D', device_name,
                '-f', config.format,
                '-r', str(config.sample_rate),
                '-c', str(config.channels),
                '-t', 'wav',
                self.audio_file
            ]

            # 启动录音进程
            self.recording_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                stdin=subprocess.PIPE
            )

            # 检查进程是否正常启动
            time.sleep(0.1)
            if self.recording_process.poll() is not None:
                stdout, stderr = self.recording_process.communicate()
                logger.error(f"❌ ALSA录音进程启动失败: {stderr.decode()}")
                return False

            self.is_recording = True
            logger.info(f"✅ ALSA录音开始 - 设备: {device_name}, "
                       f"采样率: {config.sample_rate}, "
                       f"通道: {config.channels}")
            return True

        except Exception as e:
            logger.error(f"❌ ALSA录音开始失败: {e}")
            self.cleanup()
            return False

    def stop_recording(self) -> Optional[str]:
        """停止录音并返回音频文件路径"""
        if not self.is_recording or not self.recording_process:
            logger.warning("⚠️ 未在录音状态")
            return None

        try:
            # 终止录音进程
            self.recording_process.terminate()
            try:
                self.recording_process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.recording_process.kill()
                self.recording_process.wait()

            self.is_recording = False

            # 检查音频文件是否生成
            if self.audio_file and os.path.exists(self.audio_file):
                file_size = os.path.getsize(self.audio_file)
                logger.info(f"✅ ALSA录音停止: {self.audio_file} ({file_size:,} 字节)")
                return self.audio_file
            else:
                logger.warning("⚠️ ALSA录音文件未生成")
                return None

        except Exception as e:
            logger.error(f"❌ ALSA录音停止失败: {e}")
            return None
        finally:
            self.recording_process = None

    def cleanup(self):
        """清理资源"""
        try:
            if self.is_recording and self.recording_process:
                self.recording_process.terminate()
                try:
                    self.recording_process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.recording_process.kill()
                    self.recording_process.wait()

            if self.audio_file and os.path.exists(self.audio_file):
                os.remove(self.audio_file)
                self.audio_file = None

            self.is_recording = False
            self.recording_process = None

        except Exception as e:
            logger.error(f"❌ ALSA资源清理失败: {e}")


class HybridAudioInput:
    """
    混合音频输入类 - PyAudio + ALSA

    自动检测并使用最佳可用的音频输入方案
    """

    def __init__(self, config: Optional[AudioConfig] = None):
        """初始化混合音频输入"""
        self.config = config or AudioConfig()
        self.pyaudio_engine = None
        self.alsa_engine = ALSAAudioInput()
        self.current_engine = None
        self.devices = []
        self.audio_queue = Queue()
        self.is_recording = False
        self.recording_thread = None

        # 选择音频引擎
        self._select_audio_engine()

        # 发现音频设备
        self._discover_audio_devices()

        logger.info("✅ 混合音频输入初始化完成")
        logger.info(f"  - 当前引擎: {self.current_engine}")
        logger.info(f"  - 设备数量: {len(self.devices)}")

    def _select_audio_engine(self) -> None:
        """选择最佳音频引擎"""
        # 首先尝试PyAudio
        if HAS_PYAUDIO:
            try:
                self.pyaudio_engine = pyaudio.PyAudio()
                device_count = self.pyaudio_engine.get_device_count()
                if device_count > 0:
                    self.current_engine = "pyaudio"
                    logger.info("✅ 选择PyAudio引擎")
                    return
            except Exception as e:
                logger.warning(f"⚠️ PyAudio引擎测试失败: {e}")
                if self.pyaudio_engine:
                    self.pyaudio_engine.terminate()
                    self.pyaudio_engine = None

        # 使用ALSA作为备选
        self.current_engine = "alsa"
        logger.info("✅ 选择ALSA引擎 (备选方案)")

    def _discover_audio_devices(self) -> None:
        """发现音频设备"""
        self.devices = []

        if self.current_engine == "pyaudio":
            self._discover_pyaudio_devices()
        else:
            self._discover_alsa_devices()

    def _discover_pyaudio_devices(self) -> None:
        """发现PyAudio设备"""
        try:
            device_count = self.pyaudio_engine.get_device_count()
            logger.info(f"🔍 PyAudio发现 {device_count} 个音频设备")

            for i in range(device_count):
                device_info = self.pyaudio_engine.get_device_info_by_index(i)

                if device_info['maxInputChannels'] > 0:
                    device = AudioDevice(
                        device_id=i,
                        name=device_info['name'],
                        max_input_channels=int(device_info['maxInputChannels']),
                        max_output_channels=int(device_info['maxOutputChannels']),
                        default_sample_rate=int(device_info['defaultSampleRate']),
                        is_usb_device='USB' in device_info['name'].upper(),
                        device_type="pyaudio"
                    )
                    self.devices.append(device)
                    logger.info(f"  🎤 [{i}] {device.name}")

            # 选择最佳输入设备
            self._select_best_input_device()

        except Exception as e:
            logger.error(f"❌ PyAudio设备发现失败: {e}")

    def _discover_alsa_devices(self) -> None:
        """发现ALSA设备"""
        logger.info("🔍 发现ALSA音频设备")
        self.devices = self.alsa_engine.discover_devices()

        # 选择最佳输入设备
        self._select_best_input_device()

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

        # 更新采样率
        if selected_device.default_sample_rate != self.config.sample_rate:
            logger.info(f"🔄 更新采样率: {self.config.sample_rate} → {selected_device.default_sample_rate}")
            self.config.sample_rate = selected_device.default_sample_rate

    def start_recording(self) -> bool:
        """开始录音"""
        if self.is_recording:
            logger.warning("⚠️ 已经在录音状态")
            return True

        if self.current_engine == "pyaudio":
            return self._start_pyaudio_recording()
        else:
            return self._start_alsa_recording()

    def _start_pyaudio_recording(self) -> bool:
        """开始PyAudio录音"""
        try:
            # 检查输入设备
            if self.config.input_device is None:
                logger.error("❌ 未选择音频输入设备")
                return False

            # 打开音频流
            self.stream = self.pyaudio_engine.open(
                format=pyaudio.paInt16,
                channels=self.config.channels,
                rate=self.config.sample_rate,
                input=True,
                input_device_index=self.config.input_device,
                frames_per_buffer=self.config.frames_per_buffer,
                stream_callback=self._pyaudio_callback
            )

            # 开始录音
            self.stream.start_stream()
            self.is_recording = True

            logger.info(f"✅ PyAudio录音开始 - 设备: {self.config.input_device}")
            return True

        except Exception as e:
            logger.error(f"❌ PyAudio录音开始失败: {e}")
            return False

    def _start_alsa_recording(self) -> bool:
        """开始ALSA录音"""
        try:
            # 启动ALSA录音
            success = self.alsa_engine.start_recording(self.config)
            if success:
                self.is_recording = True
                logger.info("✅ ALSA录音开始")
            return success

        except Exception as e:
            logger.error(f"❌ ALSA录音开始失败: {e}")
            return False

    def _pyaudio_callback(self, in_data, frame_count, time_info, status):
        """PyAudio音频流回调函数"""
        if status:
            logger.warning(f"⚠️ 音频流状态: {status}")

        try:
            self.audio_queue.put(in_data)
        except Exception as e:
            logger.error(f"❌ 音频数据处理失败: {e}")

        return (in_data, pyaudio.paContinue)

    def stop_recording(self) -> Optional[str]:
        """停止录音"""
        if not self.is_recording:
            logger.warning("⚠️ 未在录音状态")
            return None

        try:
            if self.current_engine == "pyaudio":
                return self._stop_pyaudio_recording()
            else:
                return self._stop_alsa_recording()

        except Exception as e:
            logger.error(f"❌ 录音停止失败: {e}")
            return None
        finally:
            self.is_recording = False

    def _stop_pyaudio_recording(self) -> Optional[str]:
        """停止PyAudio录音"""
        try:
            if self.stream:
                self.stream.stop_stream()
                self.stream.close()
                self.stream = None

            # 收集音频数据
            audio_data = []
            while not self.audio_queue.empty():
                try:
                    data = self.audio_queue.get_nowait()
                    audio_data.append(data)
                except Empty:
                    break

            # 保存音频文件
            if audio_data:
                audio_file = tempfile.mktemp(suffix='.wav')
                with wave.open(audio_file, 'wb') as wf:
                    wf.setnchannels(self.config.channels)
                    wf.setsampwidth(self.pyaudio_engine.get_sample_size(pyaudio.paInt16))
                    wf.setframerate(self.config.sample_rate)
                    wf.writeframes(b''.join(audio_data))

                file_size = os.path.getsize(audio_file)
                logger.info(f"✅ PyAudio录音停止: {audio_file} ({file_size:,} 字节)")
                return audio_file
            else:
                logger.warning("⚠️ PyAudio录音数据为空")
                return None

        except Exception as e:
            logger.error(f"❌ PyAudio录音停止失败: {e}")
            return None

    def _stop_alsa_recording(self) -> Optional[str]:
        """停止ALSA录音"""
        audio_file = self.alsa_engine.stop_recording()
        if audio_file:
            logger.info("✅ ALSA录音停止")
        return audio_file

    def get_audio_devices(self) -> List[AudioDevice]:
        """获取音频设备列表"""
        return self.devices.copy()

    def get_current_config(self) -> Dict[str, Any]:
        """获取当前配置"""
        return {
            'sample_rate': self.config.sample_rate,
            'channels': self.config.channels,
            'frames_per_buffer': self.config.frames_per_buffer,
            'input_device': self.config.input_device,
            'is_recording': self.is_recording,
            'device_count': len(self.devices),
            'current_engine': self.current_engine,
            'has_pyaudio': HAS_PYAUDIO
        }

    def test_audio_device(self, device_id: Union[int, str], test_duration: float = 2.0) -> bool:
        """测试音频设备"""
        logger.info(f"🧪 测试音频设备: {device_id}")

        try:
            # 保存当前设备
            current_device = self.config.input_device

            # 切换到测试设备
            self.config.input_device = device_id

            # 录制测试音频
            success = self.start_recording()
            if not success:
                self.config.input_device = current_device
                return False

            time.sleep(test_duration)

            # 停止录音
            audio_file = self.stop_recording()

            # 检查录制结果
            test_success = (audio_file is not None and
                          os.path.exists(audio_file) and
                          os.path.getsize(audio_file) > 1000)

            # 清理测试文件
            if audio_file and os.path.exists(audio_file):
                os.remove(audio_file)

            # 恢复原设备
            self.config.input_device = current_device

            if test_success:
                logger.info(f"✅ 设备 {device_id} 测试通过")
            else:
                logger.error(f"❌ 设备 {device_id} 测试失败")

            return test_success

        except Exception as e:
            logger.error(f"❌ 音频设备测试失败: {e}")
            self.config.input_device = current_device
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

            if self.pyaudio_engine:
                self.pyaudio_engine.terminate()
                self.pyaudio_engine = None

            if self.alsa_engine:
                self.alsa_engine.cleanup()

            # 清空队列
            while not self.audio_queue.empty():
                try:
                    self.audio_queue.get_nowait()
                except Empty:
                    break

            logger.info("✅ 混合音频输入资源已清理")

        except Exception as e:
            logger.error(f"❌ 清理资源失败: {e}")


def create_hybrid_audio_input(sample_rate: int = 16000,
                             channels: int = 1,
                             device_id: Optional[Union[int, str]] = None) -> HybridAudioInput:
    """
    创建混合音频输入实例

    Args:
        sample_rate: 采样率
        channels: 通道数
        device_id: 设备ID，None表示自动选择

    Returns:
        HybridAudioInput: 混合音频输入实例
    """
    config = AudioConfig(
        sample_rate=sample_rate,
        channels=channels,
        input_device=device_id
    )
    return HybridAudioInput(config)


def test_hybrid_audio_input():
    """测试混合音频输入功能"""
    logger.info("🧪 测试混合音频输入功能")

    try:
        with create_hybrid_audio_input() as audio_input:
            # 显示设备信息
            devices = audio_input.get_audio_devices()
            logger.info(f"📱 发现 {len(devices)} 个音频设备")

            # 显示当前配置
            config = audio_input.get_current_config()
            logger.info(f"⚙️ 当前配置: {config}")

            # 测试录音
            logger.info("🎤 开始测试录音 (3秒)...")
            success = audio_input.start_recording()

            if success:
                time.sleep(3.0)
                audio_file = audio_input.stop_recording()

                if audio_file:
                    file_size = os.path.getsize(audio_file)
                    logger.info(f"✅ 测试录音成功: {audio_file} ({file_size:,} 字节)")
                    # 清理测试文件
                    os.remove(audio_file)
                else:
                    logger.error("❌ 测试录音失败 - 无音频文件")
            else:
                logger.error("❌ 测试录音失败 - 录音启动失败")

            logger.info("🎉 混合音频输入测试完成")

    except Exception as e:
        logger.error(f"❌ 测试失败: {e}")


if __name__ == "__main__":
    # 运行测试
    test_hybrid_audio_input()