#!/usr/bin/env python3.10
"""
Thread-Safe Audio Recorder - 修复版本
====================================

修复了设备访问和采样率兼容性问题的线程安全音频录制器。

修复内容：
1. 动态设备检测，不再硬编码设备索引
2. 使用设备原生采样率，避免采样率冲突
3. 增加超时时间，改善稳定性
4. 优化状态管理，减少竞争条件

作者: Claude Code Agent
版本: 2.1 (修复版本)
日期: 2025-11-18
"""

import pyaudio
import numpy as np
import threading
import time
import asyncio
from typing import Optional, Callable
from enum import Enum
import logging

# 配置日志
logger = logging.getLogger(__name__)

class RecordingState(Enum):
    """录音状态枚举"""
    IDLE = "idle"
    STARTING = "starting"
    RECORDING = "recording"
    STOPPING = "stopping"

class ThreadSafeAudioRecorderFixed:
    """
    线程安全的音频录制器 - 修复版本

    修复了设备访问、采样率兼容性和超时机制问题。
    """

    def __init__(self):
        """初始化线程安全音频录制器"""
        # 音频参数
        self.FORMAT = pyaudio.paInt16  # 16-bit
        self.CHANNELS = 1  # 单声道
        self.RATE = 16000  # 16kHz ASR要求
        self.CHUNK = 1024  # 缓冲区大小

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

        # PyAudio实例
        self._audio = None
        self._stream = None
        self._selected_device_index = None
        self._device_sample_rate = 44100

        # 统计信息
        self._stats = {
            'total_attempts': 0,
            'successful_recordings': 0,
            'failed_recordings': 0,
            'concurrent_conflicts': 0,
            'average_duration': 0.0,
            'state_changes': 0,
            'device_detection_time': 0.0
        }

        logger.info("ThreadSafeAudioRecorderFixed初始化完成")
        logger.info(f"目标音频参数: {self.RATE}Hz, {self.CHANNELS}通道, 16-bit")

    def _detect_best_audio_device(self) -> bool:
        """动态检测最佳音频设备"""
        detection_start = time.time()

        try:
            p = pyaudio.PyAudio()
            device_count = p.get_device_count()

            logger.debug(f"检测到 {device_count} 个音频设备")

            best_device_index = None
            best_device_score = -1
            device_sample_rate = 44100  # 默认采样率

            # 遍历所有设备，寻找最佳输入设备
            for i in range(device_count):
                try:
                    device_info = p.get_device_info_by_index(i)
                    if device_info['maxInputChannels'] > 0:
                        device_native_rate = int(device_info['defaultSampleRate'])
                        device_name = device_info['name']

                        logger.debug(f"设备 {i}: {device_name}")
                        logger.debug(f"  原生采样率: {device_native_rate}Hz")
                        logger.debug(f"  输入通道: {device_info['maxInputChannels']}")

                        # 评分系统
                        score = 0
                        if 'USB' in device_name.upper():
                            score += 50  # USB设备优先
                        if device_native_rate == 16000:
                            score += 30  # 理想采样率
                        elif device_native_rate in [44100, 48000]:
                            score += 20  # 常见采样率
                        if device_info['maxInputChannels'] >= 2:
                            score += 10  # 多通道设备

                        logger.debug(f"  设备评分: {score}")

                        if score > best_device_score:
                            best_device_score = score
                            best_device_index = i
                            device_sample_rate = device_native_rate

                            if score >= 60:  # 找到足够好的设备
                                logger.info(f"选择音频设备: {device_name} (索引: {i}, 评分: {score})")
                                break

                except Exception as e:
                    logger.warning(f"设备 {i} 检测失败: {e}")
                    continue

            # 如果没有找到合适的设备，使用第一个可用设备
            if best_device_index is None:
                logger.warning("未找到合适的音频设备，尝试使用默认设备")
                for i in range(device_count):
                    try:
                        device_info = p.get_device_info_by_index(i)
                        if device_info['maxInputChannels'] > 0:
                            best_device_index = i
                            device_sample_rate = int(device_info['defaultSampleRate'])
                            logger.info(f"使用默认设备: {device_info['name']} (索引: {i})")
                            break
                    except Exception as e:
                        continue

            p.terminate()

            if best_device_index is not None:
                self._selected_device_index = best_device_index
                self._device_sample_rate = device_sample_rate
                detection_time = time.time() - detection_start
                self._stats['device_detection_time'] = detection_time

                logger.info(f"设备检测完成，耗时: {detection_time:.3f}秒")
                logger.info(f"选择设备索引: {best_device_index}, 设备采样率: {device_sample_rate}Hz")
                return True
            else:
                logger.error("未找到任何可用的音频输入设备")
                return False

        except Exception as e:
            logger.error(f"音频设备检测失败: {e}")
            return False

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
                if self._wait_for_state_change(timeout=3.0, target_states=[RecordingState.RECORDING]):
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
                self._recording_thread.join(timeout=3.0)
                if self._recording_thread.is_alive():
                    logger.warning("⚠️ 录音线程未在预期时间内完成")

            # 收集结果
            if self._audio_data:
                audio_array = np.concatenate(self._audio_data)
                logger.info(f"✅ 录音完成，数据长度: {len(audio_array)} samples, 实际时长: {self._actual_duration:.2f}秒")

                # 重采样到16kHz（如果需要）
                if self._device_sample_rate != self.RATE:
                    audio_array = self._resample_audio(audio_array, self._device_sample_rate, self.RATE)
                    logger.debug(f"音频重采样: {self._device_sample_rate}Hz -> {self.RATE}Hz")

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

    def _resample_audio(self, audio_data: np.ndarray, from_rate: int, to_rate: int) -> np.ndarray:
        """简单的线性重采样"""
        if from_rate == to_rate:
            return audio_data

        # 计算重采样比例
        ratio = to_rate / from_rate
        new_length = int(len(audio_data) * ratio)

        # 线性插值重采样
        original_indices = np.arange(len(audio_data))
        new_indices = np.arange(new_length) / ratio

        # 使用numpy的线性插值
        resampled = np.interp(new_indices, original_indices, audio_data.astype(float))

        return resampled.astype(np.int16)

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
            if self._selected_device_index is not None:
                stats['selected_device_index'] = self._selected_device_index
                stats['device_sample_rate'] = self._device_sample_rate
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
        线程安全的录音工作线程
        """
        try:
            logger.debug(f"录音线程开始，时长: {duration}秒")

            # 状态转换：STARTING -> RECORDING
            with self._recording_lock:
                if self._state == RecordingState.STOPPING:
                    logger.debug("录音线程检测到停止信号，提前退出")
                    return
                self._set_state(RecordingState.RECORDING)

            # 检测音频设备（首次使用时）
            if self._selected_device_index is None:
                if not self._detect_best_audio_device():
                    logger.error("❌ 音频设备检测失败")
                    return

            # 创建PyAudio实例和音频流
            self._audio = pyaudio.PyAudio()

            try:
                # 使用检测到的设备打开音频流
                self._stream = self._audio.open(
                    format=self.FORMAT,
                    channels=self.CHANNELS,
                    rate=self._device_sample_rate,
                    input=True,
                    input_device_index=self._selected_device_index,
                    frames_per_buffer=self.CHUNK
                )

                logger.info("音频流已打开")

                # 录制音频
                frames_to_record = int(duration * self._device_sample_rate)
                frames_recorded = 0

                while (frames_recorded < frames_to_record and
                       self._state == RecordingState.RECORDING):
                    try:
                        # 读取音频数据
                        data = self._stream.read(self.CHUNK, exception_on_overflow=False)
                        if data:
                            audio_chunk = np.frombuffer(data, dtype=np.int16)

                            # 添加音频数据
                            with self._recording_lock:
                                self._audio_data.append(audio_chunk)

                            frames_recorded += len(audio_chunk)

                            # 调试信息（前几个块）
                            if len(self._audio_data) <= 5:
                                max_amplitude = np.max(np.abs(audio_chunk))
                                rms = np.sqrt(np.mean(audio_chunk.astype(float) ** 2))
                                logger.debug(f"[录音调试] 块{len(self._audio_data)}: 最大幅度={max_amplitude}, RMS={rms:.1f}")

                    except Exception as e:
                        if self._state == RecordingState.RECORDING:
                            logger.error(f"❌ 读取音频数据失败: {e}")
                        break

                logger.debug(f"录制完成，录制了 {len(self._audio_data)} 个音频块")

            except Exception as e:
                logger.error(f"❌ 音频流操作失败: {e}")
            finally:
                self._cleanup_resources()
                logger.debug("录音线程结束")

        except Exception as e:
            logger.error(f"❌ 录音线程异常: {e}")
        finally:
            # 确保状态重置
            with self._recording_lock:
                if self._state == RecordingState.RECORDING:
                    self._set_state(RecordingState.IDLE)

    def _cleanup_resources(self):
        """清理音频资源"""
        try:
            if self._stream:
                self._stream.stop_stream()
                self._stream.close()
                self._stream = None
                logger.debug("音频流已清理")
        except Exception as e:
            logger.error(f"❌ 清理音频流失败: {e}")

        try:
            if self._audio:
                self._audio.terminate()
                self._audio = None
                logger.debug("PyAudio实例已清理")
        except Exception as e:
            logger.error(f"❌ 清理PyAudio实例失败: {e}")

    def get_audio_config(self) -> dict:
        """获取音频配置"""
        return {
            'target_sample_rate': self.RATE,
            'channels': self.CHANNELS,
            'format': 'int16',
            'chunk_size': self.CHUNK,
            'bytes_per_sample': 2,
            'architecture': 'thread_safe_fixed',
            'thread_safe': True,
            'selected_device_index': self._selected_device_index,
            'device_sample_rate': self._device_sample_rate,
            'resampling_enabled': self._device_sample_rate != self.RATE
        }

    def test_recording(self) -> bool:
        """
        测试录音功能

        Returns:
            bool: 测试是否成功
        """
        try:
            logger.info("开始录音测试...")
            success = self.start_recording(duration=2.0)

            if success:
                # 等待录音完成
                time.sleep(2.5)

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

def create_thread_safe_recorder_fixed() -> ThreadSafeAudioRecorderFixed:
    """
    创建修复版本的线程安全录音器实例

    Returns:
        ThreadSafeAudioRecorderFixed: 录音器实例
    """
    return ThreadSafeAudioRecorderFixed()

# 向后兼容的别名
ThreadSafeAudioRecorder = ThreadSafeAudioRecorderFixed