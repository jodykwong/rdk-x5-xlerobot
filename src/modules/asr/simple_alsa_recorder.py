#!/usr/bin/env python3.10
"""
Simple ALSA Recorder - 基于ALSA命令行的简单录音器
==================================================

使用ALSA命令行工具进行录音，避免PyAudio的复杂性问题。

作者: Claude Code Agent
版本: 1.0
日期: 2025-11-18
"""

import subprocess
import numpy as np
import threading
import time
import tempfile
import os
import logging
from typing import Optional, Callable
from enum import Enum

# 配置日志
logger = logging.getLogger(__name__)

class RecordingState(Enum):
    """录音状态枚举"""
    IDLE = "idle"
    STARTING = "starting"
    RECORDING = "recording"
    STOPPING = "stopping"

class SimpleALSARecorder:
    """
    基于ALSA命令行的简单录音器

    使用arecord命令进行录音，稳定可靠。
    """

    def __init__(self):
        """初始化ALSA录音器"""
        # 音频参数
        self.target_rate = 16000  # ASR要求的16kHz
        self.channels = 1  # 单声道
        self.format = "cd"  # CD质量 (16-bit, 44.1kHz)

        # 状态管理
        self._state = RecordingState.IDLE
        self._lock = threading.Lock()

        # 录音相关
        self._temp_file = None
        self._recording_thread = None
        self._callback = None
        self._completion_event = threading.Event()

        # 统计信息
        self._stats = {
            'total_attempts': 0,
            'successful_recordings': 0,
            'failed_recordings': 0,
            'average_duration': 0.0,
            'total_duration': 0.0
        }

        logger.info("SimpleALSARecorder初始化完成")
        logger.info(f"目标参数: {self.target_rate}Hz, {self.channels}通道, 16-bit")

    def start_recording(self, duration: float = 3.0,
                       callback: Optional[Callable] = None) -> bool:
        """
        开始录音

        Args:
            duration: 录制时长（秒）
            callback: 完成回调函数

        Returns:
            bool: 是否成功启动
        """
        with self._lock:
            if self._state != RecordingState.IDLE:
                logger.debug("录音正在进行中")
                return False

            self._state = RecordingState.STARTING
            self._callback = callback
            self._completion_event.clear()

            # 创建临时文件
            try:
                fd, self._temp_file = tempfile.mkstemp(suffix='.wav')
                os.close(fd)
                logger.debug(f"创建临时文件: {self._temp_file}")
                # 确保文件存在
                open(self._temp_file, 'wb').close()
            except Exception as e:
                logger.error(f"创建临时文件失败: {e}")
                self._state = RecordingState.IDLE
                return False

            # 更新统计
            self._stats['total_attempts'] += 1

            # 启动录音线程
            self._recording_thread = threading.Thread(
                target=self._record_worker,
                args=(duration,),
                daemon=True
            )
            self._recording_thread.start()

            self._state = RecordingState.RECORDING
            logger.info(f"✅ 开始录音，时长: {duration}秒")
            return True

    def stop_recording(self) -> np.ndarray:
        """
        停止录音

        Returns:
            np.ndarray: 录音数据
        """
        with self._lock:
            # 检查是否有录音在进行或已完成
            if self._state not in [RecordingState.RECORDING, RecordingState.IDLE]:
                logger.debug(f"没有录音在进行，当前状态: {self._state}")
                return np.array([], dtype=np.int16)

            # 如果已经完成，直接读取数据
            if self._state == RecordingState.IDLE and self._recording_thread and not self._recording_thread.is_alive():
                logger.info("录音已完成，读取数据...")
                audio_data = self._load_recording()
                self._cleanup()
                return audio_data

            self._state = RecordingState.STOPPING
            logger.info("停止录音...")

            # 等待录音线程完成
            if self._recording_thread and self._recording_thread.is_alive():
                self._recording_thread.join(timeout=5.0)
                if self._recording_thread.is_alive():
                    logger.warning("录音线程未完成")

            # 读取录音数据
            audio_data = self._load_recording()

            # 清理资源
            self._cleanup()

            # 触发回调
            if self._callback and len(audio_data) > 0:
                try:
                    self._callback(audio_data)
                except Exception as e:
                    logger.error(f"回调函数执行失败: {e}")

            self._completion_event.set()
            return audio_data

    def _record_worker(self, duration: float):
        """录音工作线程"""
        try:
            start_time = time.time()

            # 构建arecord命令
            cmd = [
                'arecord',
                '-D', 'plughw:0,0',  # USB设备
                '-d', str(int(duration)),  # 转换为整数
                '-f', self.format,
                '-c', str(self.channels),
                '-r', str(44100),  # 先用44.1kHz录制
                self._temp_file
            ]

            logger.debug(f"执行命令: {' '.join(cmd)}")

            # 执行录音
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=duration + 5  # 额外5秒超时
            )

            if result.returncode == 0:
                actual_duration = time.time() - start_time
                logger.info(f"录音完成，实际时长: {actual_duration:.2f}秒")

                # 更新统计
                self._stats['successful_recordings'] += 1
                self._stats['total_duration'] += actual_duration
                if self._stats['successful_recordings'] > 0:
                    self._stats['average_duration'] = (
                        self._stats['total_duration'] / self._stats['successful_recordings']
                    )
            else:
                logger.error(f"录音失败: {result.stderr}")
                self._stats['failed_recordings'] += 1

        except subprocess.TimeoutExpired:
            logger.error("录音超时")
            self._stats['failed_recordings'] += 1
        except Exception as e:
            logger.error(f"录音线程异常: {e}")
            self._stats['failed_recordings'] += 1
        finally:
            with self._lock:
                if self._state == RecordingState.RECORDING:
                    self._state = RecordingState.IDLE

    def _load_recording(self) -> np.ndarray:
        """加载录音文件"""
        if not self._temp_file or not os.path.exists(self._temp_file):
            logger.warning("录音文件不存在")
            return np.array([], dtype=np.int16)

        try:
            # 检查文件大小
            file_size = os.path.getsize(self._temp_file)
            logger.debug(f"录音文件大小: {file_size} bytes")

            if file_size < 100:  # 文件太小，可能是空的
                logger.warning("录音文件为空")
                return np.array([], dtype=np.int16)

            # 使用 scipy 读取 WAV 文件
            import scipy.io.wavfile as wavfile

            sample_rate, data = wavfile.read(self._temp_file)

            logger.debug(f"原始音频: 采样率={sample_rate}, 数据长度={len(data)}, 数据类型={data.dtype}")

            # 转换为单声道
            if len(data.shape) > 1:
                data = data[:, 0]  # 取左声道

            # 重采样到16kHz
            if sample_rate != self.target_rate:
                data = self._resample_audio(data, sample_rate, self.target_rate)

            # 确保是int16格式
            if data.dtype != np.int16:
                if np.issubdtype(data.dtype, np.floating):
                    data = (data * 32767).astype(np.int16)
                else:
                    data = data.astype(np.int16)

            logger.info(f"✅ 加载音频数据: {len(data)} samples, 采样率={self.target_rate}Hz")
            return data

        except ImportError:
            logger.error("需要安装 scipy: pip install scipy")
            return np.array([], dtype=np.int16)
        except Exception as e:
            logger.error(f"加载音频文件失败: {e}")
            # 尝试删除可能损坏的文件
            if self._temp_file and os.path.exists(self._temp_file):
                try:
                    os.unlink(self._temp_file)
                    logger.debug("删除损坏的录音文件")
                except:
                    pass
            return np.array([], dtype=np.int16)

    def _resample_audio(self, audio_data: np.ndarray, from_rate: int, to_rate: int) -> np.ndarray:
        """重采样音频数据"""
        if from_rate == to_rate:
            return audio_data

        # 简单线性插值重采样
        ratio = to_rate / from_rate
        new_length = int(len(audio_data) * ratio)

        original_indices = np.arange(len(audio_data))
        new_indices = np.arange(new_length) / ratio

        resampled = np.interp(new_indices, original_indices, audio_data.astype(float))

        return resampled.astype(np.int16)

    def _cleanup(self):
        """清理资源"""
        # 删除临时文件
        if self._temp_file and os.path.exists(self._temp_file):
            try:
                os.unlink(self._temp_file)
                logger.debug(f"删除临时文件: {self._temp_file}")
            except Exception as e:
                logger.error(f"删除临时文件失败: {e}")
            finally:
                self._temp_file = None

        self._state = RecordingState.IDLE

    def get_state(self) -> RecordingState:
        """获取当前状态"""
        with self._lock:
            return self._state

    def get_stats(self) -> dict:
        """获取统计信息"""
        with self._lock:
            stats = self._stats.copy()
            stats['current_state'] = self._state.value
            return stats

    def get_completion_event(self) -> threading.Event:
        """获取完成事件"""
        return self._completion_event

    def force_stop(self):
        """强制停止录音"""
        with self._lock:
            if self._state == RecordingState.RECORDING:
                logger.warning("强制停止录音")
                self._state = RecordingState.STOPPING
                if self._recording_thread and self._recording_thread.is_alive():
                    self._recording_thread.join(timeout=1.0)
                self._cleanup()

    def get_audio_config(self) -> dict:
        """获取音频配置"""
        return {
            'sample_rate': self.target_rate,
            'channels': self.channels,
            'format': 'int16',
            'device': 'plughw:0,0'
        }

    def test_recording(self) -> bool:
        """测试录音功能"""
        logger.info("开始ALSA录音测试...")

        success = self.start_recording(duration=2.0)

        if success:
            time.sleep(2.5)
            audio_data = self.stop_recording()

            if len(audio_data) > 0:
                logger.info(f"✅ ALSA录音测试成功: {len(audio_data)} samples")
                return True
            else:
                logger.warning("⚠️ 录音完成但无数据")
                return False
        else:
            logger.error("❌ ALSA录音测试启动失败")
            return False

def create_alsa_recorder() -> SimpleALSARecorder:
    """创建ALSA录音器实例"""
    return SimpleALSARecorder()

# 向后兼容说明 - 建议直接使用SimpleALSARecorder避免循环引用
# ThreadSafeAudioRecorder = SimpleALSARecorder  # 已移除，避免循环引用