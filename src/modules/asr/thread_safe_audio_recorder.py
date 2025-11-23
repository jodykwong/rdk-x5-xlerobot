#!/usr/bin/env python3
"""
Thread-Safe Audio Recorder - 并发安全的音频录制器
========================================================

专为高并发语音服务设计的线程安全音频录制器。
解决SimpleAudioRecorder的并发问题，支持稳定的音频捕获。

功能：
- 线程安全的状态管理
- 原子性录音启动和停止
- 智能并发冲突处理
- 详细的诊断和监控
- 完善的错误恢复机制

作者: Developer Agent
版本: 2.0 (线程安全版本)
日期: 2025-11-18
"""

import numpy as np
import threading
import time
import asyncio
from typing import Optional, Callable
from enum import Enum
import logging
import subprocess
import os

# 配置日志
logger = logging.getLogger(__name__)

class RecordingState(Enum):
    """录音状态枚举"""
    IDLE = "idle"
    STARTING = "starting"
    RECORDING = "recording"
    STOPPING = "stopping"

class ThreadSafeAudioRecorder:
    """
    线程安全的音频录制器

    解决并发录音冲突问题，提供稳定的音频录制功能。
    支持高并发的ASR语音服务场景。
    """

    def __init__(self):
        """初始化线程安全音频录制器"""
        # 音频参数（阿里云API兼容，ALSA友好）
        self.FORMAT = "S16_LE"  # 16-bit little endian
        self.CHANNELS = 1  # 单声道
        self.RATE = 16000  # 16kHz
        self.CHUNK = 1024  # 缓冲区大小

        # ALSA设备参数
        self.device = "default"  # 默认ALSA设备
        self.temp_file = f"/tmp/thread_safe_recording_{os.getpid()}.wav"

        # 线程安全状态管理
        self._recording_lock = threading.RLock()
        self._state = RecordingState.IDLE
        self._state_change_count = 0

        # 录音相关
        self._audio_data = []
        self._recording_thread = None
        self._callback = None
        self._completion_event = threading.Event()

        # 状态监控
        self._start_time = None
        self._requested_duration = 0.0
        self._actual_duration = 0.0

        # ALSA录音进程
        self._recording_process = None
        self._temp_audio_file = None

        # 统计信息
        self._stats = {
            'total_attempts': 0,
            'successful_recordings': 0,
            'failed_recordings': 0,
            'concurrent_conflicts': 0,
            'average_duration': 0.0,
            'state_changes': 0
        }

        logger.info("ThreadSafeAudioRecorder初始化完成")
        logger.info(f"音频参数: {self.RATE}Hz, {self.CHANNELS}通道, 16-bit")

    def start_recording(self, duration: float = 3.0,
                       callback: Optional[Callable] = None) -> bool:
        """
        开始录制音频 - 线程安全版本

        Args:
            duration: 录制时长（秒）
            callback: 录制完成回调函数

        Returns:
            bool: 录制启动成功状态
        """
        with self._recording_lock:
            # 更新统计
            self._stats['total_attempts'] += 1

            # 原子性状态检查和更新
            if self._state != RecordingState.IDLE:
                state_str = self._state.value
                logger.debug(f"录音忙碌，当前状态: {state_str}")
                self._stats['concurrent_conflicts'] += 1
                return False

            # 状态转换：IDLE -> STARTING
            self._set_state(RecordingState.STARTING)

            # 设置录音参数
            self._requested_duration = duration
            self._callback = callback
            self._completion_event.clear()
            self._audio_data = []

            try:
                # 启动录音线程
                self._recording_thread = threading.Thread(
                    target=self._record_worker_safe,
                    args=(duration,),
                    daemon=True
                )
                self._recording_thread.start()

                # 等待确认录音线程真正开始
                if self._wait_for_state_change(timeout=1.0, target_states=[RecordingState.RECORDING]):
                    # 状态转换：STARTING -> RECORDING
                    self._start_time = time.time()
                    logger.info(f"✅ 线程安全启动录音，时长: {duration}秒")
                    return True
                else:
                    # 超时，重置状态
                    self._set_state(RecordingState.IDLE)
                    self._stats['failed_recordings'] += 1
                    logger.warning("⚠️ 录音启动超时")
                    return False

            except Exception as e:
                # 发生异常，重置状态
                self._set_state(RecordingState.IDLE)
                self._stats['failed_recordings'] += 1
                logger.error(f"❌ 录音启动失败: {e}")
                return False

    def stop_recording(self) -> np.ndarray:
        """
        停止录制音频 - 线程安全版本

        Returns:
            np.ndarray: 录制的音频数据
        """
        with self._recording_lock:
            if self._state == RecordingState.IDLE:
                logger.debug("当前无录音进行")
                return np.array([], dtype=np.int16)

            if self._state != RecordingState.RECORDING:
                logger.debug(f"当前状态不适合停止: {self._state.value}")
                return np.array([], dtype=np.int16)

            # 状态转换：RECORDING -> STOPPING
            self._set_state(RecordingState.STOPPING)
            self._actual_duration = time.time() - self._start_time if self._start_time else 0

            # 等待录音线程自然完成
            if self._recording_thread and self._recording_thread.is_alive():
                logger.debug("等待录音线程完成...")
                self._recording_thread.join(timeout=2.0)
                if self._recording_thread.is_alive():
                    logger.warning("⚠️ 录音线程未在预期时间内完成")

            # 收集结果
            if self._audio_data:
                audio_array = np.concatenate(self._audio_data)
                logger.info(f"✅ 录音完成，数据长度: {len(audio_array)} samples, 实际时长: {self._actual_duration:.2f}秒")

                # 更新统计
                self._stats['successful_recordings'] += 1
                if self._stats['successful_recordings'] > 0:
                    total_duration = self._stats.get('total_duration', 0.0) + self._actual_duration
                    self._stats['total_duration'] = total_duration
                    self._stats['average_duration'] = total_duration / self._stats['successful_recordings']

            else:
                audio_array = np.array([], dtype=np.int16)
                logger.warning("⚠️ 无录音数据")
                self._stats['failed_recordings'] += 1

            # 重置状态和清理资源
            self._set_state(RecordingState.IDLE)
            self._cleanup_resources()
            self._completion_event.set()

            # 调用回调函数
            if self._callback:
                try:
                    self._callback(audio_array)
                except Exception as e:
                    logger.error(f"❌ 回调函数执行失败: {e}")

            return audio_array

    def get_state(self) -> RecordingState:
        """获取当前录音状态"""
        with self._recording_lock:
            return self._state

    def get_completion_event(self) -> threading.Event:
        """获取完成事件"""
        return self._completion_event

    def get_stats(self) -> dict:
        """获取录音统计信息"""
        with self._recording_lock:
            stats = self._stats.copy()
            stats['current_state'] = self._state.value
            stats['state_change_count'] = self._state_change_count
            return stats

    def force_stop(self) -> np.ndarray:
        """强制停止录音（紧急情况使用）"""
        logger.warning("⚠️ 强制停止录音")
        return self.stop_recording()

    def _set_state(self, new_state: RecordingState):
        """线程安全的状态设置"""
        old_state = self._state
        self._state = new_state
        self._state_change_count += 1

        if old_state != new_state:
            logger.debug(f"状态变化: {old_state.value} -> {new_state.value}")

    def _wait_for_state_change(self, timeout: float, target_states: list) -> bool:
        """等待状态变化"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self._state in target_states:
                return True
            time.sleep(0.01)  # 10ms检查间隔
        return False

    def _record_worker_safe(self, duration: float):
        """
        线程安全的ALSA录音工作线程
        """
        try:
            logger.debug(f"ALSA录音线程开始，时长: {duration}秒")

            # 状态转换：STARTING -> RECORDING
            with self._recording_lock:
                if self._state == RecordingState.STOPPING:
                    logger.debug("录音线程检测到停止信号，提前退出")
                    return
                self._set_state(RecordingState.RECORDING)

            # 创建临时音频文件
            self._temp_audio_file = f"/tmp/thread_safe_recording_{os.getpid()}_{int(time.time())}.wav"

            try:
                # 构建arecord命令
                arecord_cmd = [
                    "arecord",
                    "-D", self.device,  # ALSA设备
                    "-f", self.FORMAT,   # 格式 S16_LE
                    "-c", str(self.CHANNELS),  # 声道数
                    "-r", str(self.RATE),      # 采样率
                    "-d", str(int(duration)),  # 时长
                    self._temp_audio_file
                ]

                logger.info(f"启动ALSA录音: {' '.join(arecord_cmd)}")

                # 启动arecord进程
                self._recording_process = subprocess.Popen(
                    arecord_cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )

                # 等待录音完成
                return_code = self._recording_process.wait()

                if return_code == 0:
                    logger.info(f"ALSA录音完成，文件: {self._temp_audio_file}")

                    # 读取录音文件并转换为numpy数组
                    if os.path.exists(self._temp_audio_file):
                        try:
                            # 使用numpy读取WAV文件
                            import wave
                            with wave.open(self._temp_audio_file, 'rb') as wav_file:
                                frames = wav_file.readframes(-1)
                                audio_data = np.frombuffer(frames, dtype=np.int16)

                                # 重采样到目标采样率（如果需要）
                                if wav_file.getframerate() != self.RATE:
                                    logger.debug(f"重采样从 {wav_file.getframerate()}Hz 到 {self.RATE}Hz")
                                    # 简单线性插值重采样
                                    audio_data = self._resample_audio(
                                        audio_data,
                                        wav_file.getframerate(),
                                        self.RATE
                                    )

                                with self._recording_lock:
                                    self._audio_data = [audio_data]

                                logger.debug(f"音频数据加载成功: {len(audio_data)} samples")

                        except Exception as e:
                            logger.error(f"读取WAV文件失败: {e}")
                            with self._recording_lock:
                                self._audio_data = []
                    else:
                        logger.error("录音文件不存在")
                        with self._recording_lock:
                            self._audio_data = []
                else:
                    stderr_output = self._recording_process.stderr.read() if self._recording_process.stderr else ""
                    logger.error(f"ALSA录音失败 (返回码: {return_code}): {stderr_output}")
                    with self._recording_lock:
                        self._audio_data = []

            except Exception as e:
                logger.error(f"❌ ALSA录音操作失败: {e}")
                with self._recording_lock:
                    self._audio_data = []
            finally:
                self._cleanup_resources()
                logger.debug("ALSA录音线程结束")

        except Exception as e:
            logger.error(f"❌ ALSA录音线程异常: {e}")
        finally:
            # 确保状态重置
            with self._recording_lock:
                if self._state == RecordingState.RECORDING:
                    self._set_state(RecordingState.IDLE)

    def _cleanup_resources(self):
        """清理ALSA音频资源"""
        # 清理录音进程
        try:
            if self._recording_process:
                if self._recording_process.poll() is None:  # 进程仍在运行
                    self._recording_process.terminate()
                    try:
                        self._recording_process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        self._recording_process.kill()
                        self._recording_process.wait()
                self._recording_process = None
                logger.debug("ALSA录音进程已清理")
        except Exception as e:
            logger.error(f"❌ 清理ALSA录音进程失败: {e}")

        # 清理临时音频文件
        try:
            if self._temp_audio_file and os.path.exists(self._temp_audio_file):
                os.remove(self._temp_audio_file)
                logger.debug(f"临时音频文件已清理: {self._temp_audio_file}")
                self._temp_audio_file = None
        except Exception as e:
            logger.error(f"❌ 清理临时音频文件失败: {e}")

    def _resample_audio(self, audio_data: np.ndarray, original_rate: int, target_rate: int) -> np.ndarray:
        """
        简单的音频重采样

        Args:
            audio_data: 原始音频数据
            original_rate: 原始采样率
            target_rate: 目标采样率

        Returns:
            重采样后的音频数据
        """
        if original_rate == target_rate:
            return audio_data

        # 计算重采样比例
        ratio = target_rate / original_rate
        new_length = int(len(audio_data) * ratio)

        # 使用线性插值进行重采样
        indices = np.linspace(0, len(audio_data) - 1, new_length)
        return np.interp(indices, np.arange(len(audio_data)), audio_data).astype(np.int16)

    def get_audio_config(self) -> dict:
        """获取音频配置"""
        return {
            'sample_rate': self.RATE,
            'channels': self.CHANNELS,
            'format': 'int16',
            'chunk_size': self.CHUNK,
            'bytes_per_sample': 2,
            'architecture': 'thread_safe',
            'thread_safe': True
        }

    def test_recording(self) -> bool:
        """
        测试录音功能

        Returns:
            bool: 测试是否成功
        """
        try:
            logger.info("开始录音测试...")
            success = self.start_recording(duration=1.0)

            if success:
                # 等待录音完成
                time.sleep(1.5)

                # 获取音频数据
                audio_data = self.stop_recording()

                if len(audio_data) > 0:
                    logger.info(f"✅ 录音测试成功，数据长度: {len(audio_data)}")
                    return True
                else:
                    logger.warning("⚠️ 录音测试完成但无数据")
                    return False
            else:
                logger.error("❌ 录音测试启动失败")
                return False

        except Exception as e:
            logger.error(f"❌ 录音测试异常: {e}")
            return False

    def __del__(self):
        """析构函数"""
        try:
            if self._state == RecordingState.RECORDING:
                logger.info("析构时停止录音...")
                self.force_stop()
        except:
            pass

def create_thread_safe_recorder() -> ThreadSafeAudioRecorder:
    """
    创建线程安全录音器实例

    Returns:
        ThreadSafeAudioRecorder: 录音器实例
    """
    return ThreadSafeAudioRecorder()

# 向后兼容的别名
SimpleAudioRecorder = ThreadSafeAudioRecorder
# 向后兼容 - 建议使用SimpleALSARecorder
# 为了避免循环引用，这里不再导入，而是提供使用说明
