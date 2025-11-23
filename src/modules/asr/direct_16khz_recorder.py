#!/usr/bin/env python3
"""
XLeRobot 16kHz直接录音模块
避免重采样，直接使用16kHz输入，减少延迟和处理复杂度
"""

import logging
import numpy as np
import threading
import time
import queue
from typing import Optional, Callable, Any
from dataclasses import dataclass
import speech_recognition as sr

# 导入设备管理器
from .audio_device_manager import get_device_manager, setup_16khz_recording

logger = logging.getLogger(__name__)

@dataclass
class AudioChunk:
    """音频数据块"""
    data: bytes
    sample_rate: int = 16000
    channels: int = 1
    sample_width: int = 2  # 16-bit
    duration: float = 0.0
    timestamp: float = 0.0

    def __post_init__(self):
        if self.timestamp == 0.0:
            self.timestamp = time.time()
        if self.duration == 0.0:
            # 计算音频持续时间：bytes / (sample_rate * channels * sample_width)
            self.duration = len(self.data) / (self.sample_rate * self.channels * self.sample_width)

class Direct16kHzRecorder:
    """
    16kHz直接录音器
    - 避免重采样，直接使用16kHz设备
    - 减少延迟和处理复杂度
    - 支持连续录音和静音检测
    """

    def __init__(self, device_index: Optional[int] = None, auto_device_selection: bool = True):
        """
        初始化录音器

        Args:
            device_index: 指定设备索引，None表示自动选择
            auto_device_selection: 是否自动选择最佳设备
        """
        self.device_manager = get_device_manager()
        self.device_index = device_index
        self.auto_device_selection = auto_device_selection
        self.is_recording = False
        self.recording_thread = None
        self.audio_queue = queue.Queue()
        self.stop_event = threading.Event()
        self.audio_callback = None

        # 16kHz配置
        self.target_sample_rate = 16000
        self.target_channels = 1
        self.target_sample_width = 2  # 16-bit

        # speech_recognition配置
        self.recognizer = sr.Recognizer()
        self.microphone = None

        # 静音检测配置
        self.silence_threshold = 500  # 静音阈值
        self.silence_duration = 1.0   # 静音持续时间（秒）
        self.min_audio_duration = 0.5  # 最小音频持续时间（秒）

        logger.info(f"16kHz直接录音器初始化完成，目标设备: {device_index or '自动选择'}")

    def setup_device(self) -> bool:
        """设置音频设备"""
        try:
            if self.device_index is None and self.auto_device_selection:
                # 自动选择最佳16kHz设备
                setup_result = setup_16khz_recording()
                if setup_result["success"]:
                    self.device_index = setup_result["device_index"]
                    logger.info(f"自动选择设备: {setup_result['device_name']} (索引: {self.device_index})")
                else:
                    logger.error(f"自动设备选择失败: {setup_result['error']}")
                    return False
            elif self.device_index is not None:
                # 验证指定设备
                setup_result = setup_16khz_recording(self.device_index)
                if not setup_result["success"]:
                    logger.error(f"指定设备 {self.device_index} 设置失败: {setup_result['error']}")
                    return False

            # 创建Microphone对象
            try:
                self.microphone = sr.Microphone(device_index=self.device_index)

                # 配置recognizer参数
                self.recognizer.dynamic_energy_threshold = False
                self.recognizer.energy_threshold = 300
                self.recognizer.pause_threshold = 0.8
                self.recognizer.operation_timeout = None
                self.recognizer.phrase_threshold = 0.3
                self.recognizer.non_speaking_duration = 0.5

                logger.info(f"麦克风设备设置成功: 索引 {self.device_index}")
                return True

            except Exception as e:
                logger.error(f"麦克风创建失败: {e}")
                return False

        except Exception as e:
            logger.error(f"设备设置失败: {e}")
            return False

    def start_recording(self, audio_callback: Optional[Callable[[AudioChunk], None]] = None) -> bool:
        """
        开始录音

        Args:
            audio_callback: 音频数据回调函数

        Returns:
            bool: 录音启动成功状态
        """
        if self.is_recording:
            logger.warning("录音已在进行中")
            return True

        # 设置设备
        if not self.setup_device():
            logger.error("设备设置失败，无法开始录音")
            return False

        self.audio_callback = audio_callback
        self.stop_event.clear()
        self.is_recording = True

        # 启动录音线程
        self.recording_thread = threading.Thread(target=self._recording_loop, daemon=True)
        self.recording_thread.start()

        logger.info("16kHz直接录音已开始")
        return True

    def stop_recording(self) -> None:
        """停止录音"""
        if not self.is_recording:
            return

        logger.info("正在停止录音...")
        self.stop_event.set()
        self.is_recording = False

        if self.recording_thread and self.recording_thread.is_alive():
            self.recording_thread.join(timeout=2.0)

        # 解锁设备
        if self.device_index is not None:
            self.device_manager.unlock_device(self.device_index)

        logger.info("录音已停止")

    def _recording_loop(self) -> None:
        """录音主循环"""
        try:
            with self.microphone as source:
                logger.debug("进入录音上下文")

                # 一次性调整环境噪音（避免每次都调整）
                logger.debug("调整环境噪音...")
                self.recognizer.adjust_for_ambient_noise(source, duration=0.5)

                while not self.stop_event.is_set() and self.is_recording:
                    try:
                        # 录制音频块
                        logger.debug("等待音频输入...")
                        audio = self.recognizer.listen(
                            source,
                            timeout=None,  # 无限等待
                            phrase_time_limit=None  # 无时间限制
                        )

                        if audio and not self.stop_event.is_set():
                            # 直接使用原始音频数据（已经是16kHz）
                            raw_data = audio.get_raw_data()

                            if raw_data:
                                # 创建音频块
                                chunk = AudioChunk(
                                    data=raw_data,
                                    sample_rate=self.target_sample_rate,
                                    channels=self.target_channels,
                                    sample_width=self.target_sample_width,
                                    timestamp=time.time()
                                )

                                # 静音检测
                                if not self._is_silence(chunk):
                                    # 处理音频块
                                    self._process_audio_chunk(chunk)
                                else:
                                    logger.debug("检测到静音，跳过")

                    except sr.WaitTimeoutError:
                        # 超时是正常的，继续监听
                        logger.debug("录音超时，继续监听...")
                        continue
                    except Exception as e:
                        if not self.stop_event.is_set():
                            logger.error(f"录音循环异常: {e}")
                        break

        except Exception as e:
            logger.error(f"录音线程异常: {e}")
        finally:
            self.is_recording = False
            logger.debug("录音循环结束")

    def _is_silence(self, chunk: AudioChunk) -> bool:
        """
        检测音频块是否为静音

        Args:
            chunk: 音频块

        Returns:
            bool: 是否为静音
        """
        try:
            # 转换为numpy数组进行分析
            audio_array = np.frombuffer(chunk.data, dtype=np.int16)

            # 计算RMS能量
            rms = np.sqrt(np.mean(audio_array.astype(np.float32) ** 2))

            # 转换为分贝
            if rms > 0:
                db = 20 * np.log10(rms)
                is_silent = db < self.silence_threshold
            else:
                is_silent = True

            logger.debug(f"音频能量: {rms:.2f} ({db:.1f}dB), 静音: {is_silent}")

            return is_silent

        except Exception as e:
            logger.error(f"静音检测失败: {e}")
            return False

    def _process_audio_chunk(self, chunk: AudioChunk) -> None:
        """
        处理音频块

        Args:
            chunk: 音频块
        """
        try:
            # 检查最小持续时间
            if chunk.duration < self.min_audio_duration:
                logger.debug(f"音频块过短 ({chunk.duration:.2f}s)，跳过")
                return

            # 添加到队列
            self.audio_queue.put(chunk)

            # 调用回调函数
            if self.audio_callback:
                self.audio_callback(chunk)

            logger.debug(f"处理音频块: {len(chunk.data)} bytes, {chunk.duration:.2f}s")

        except Exception as e:
            logger.error(f"音频块处理失败: {e}")

    def get_audio_chunk(self, timeout: Optional[float] = None) -> Optional[AudioChunk]:
        """
        获取音频块

        Args:
            timeout: 超时时间（秒）

        Returns:
            AudioChunk: 音频块，超时返回None
        """
        try:
            if timeout:
                chunk = self.audio_queue.get(timeout=timeout)
            else:
                chunk = self.audio_queue.get()
            return chunk
        except queue.Empty:
            return None

    def clear_queue(self) -> int:
        """清空音频队列"""
        count = 0
        try:
            while not self.audio_queue.empty():
                self.audio_queue.get_nowait()
                count += 1
        except queue.Empty:
            pass

        logger.debug(f"清空音频队列，移除 {count} 个块")
        return count

    def get_queue_size(self) -> int:
        """获取队列大小"""
        return self.audio_queue.qsize()

    def set_silence_threshold(self, threshold_db: float) -> None:
        """设置静音阈值"""
        self.silence_threshold = threshold_db
        logger.info(f"静音阈值设置为: {threshold_db}dB")

    def set_silence_duration(self, duration_seconds: float) -> None:
        """设置静音持续时间"""
        self.silence_duration = duration_seconds
        logger.info(f"静音持续时间设置为: {duration_seconds}s")

    def get_status(self) -> Dict[str, Any]:
        """获取录音器状态"""
        return {
            "is_recording": self.is_recording,
            "device_index": self.device_index,
            "target_sample_rate": self.target_sample_rate,
            "target_channels": self.target_channels,
            "queue_size": self.get_queue_size(),
            "silence_threshold": self.silence_threshold,
            "silence_duration": self.silence_duration,
            "thread_alive": self.recording_thread.is_alive() if self.recording_thread else False
        }

def create_direct_16khz_recorder(device_index: Optional[int] = None,
                                  auto_device_selection: bool = True) -> Direct16kHzRecorder:
    """
    创建16kHz直接录音器

    Args:
        device_index: 指定设备索引
        auto_device_selection: 是否自动选择设备

    Returns:
        Direct16kHzRecorder: 录音器实例
    """
    return Direct16kHzRecorder(device_index=device_index, auto_device_selection=auto_device_selection)

if __name__ == "__main__":
    # 测试代码
    import logging
    import time

    logging.basicConfig(level=logging.INFO)

    print("=== 16kHz直接录音器测试 ===")

    # 创建录音器
    recorder = create_direct_16khz_recorder(auto_device_selection=True)

    # 音频回调函数
    def audio_callback(chunk: AudioChunk):
        print(f"收到音频: {len(chunk.data)} bytes, {chunk.duration:.2f}s, 能量: {np.mean(np.frombuffer(chunk.data, dtype=np.int16)):.2f}")

    # 开始录音
    print("\n1. 开始录音...")
    if recorder.start_recording(audio_callback=audio_callback):
        print("录音启动成功")

        # 录音5秒
        print("录音中，请说话...")
        time.sleep(5)

        # 显示状态
        status = recorder.get_status()
        print(f"\n录音状态: {status}")

        # 从队列获取音频块
        print("\n2. 获取录音数据...")
        chunk_count = 0
        while recorder.get_queue_size() > 0:
            chunk = recorder.get_audio_chunk(timeout=0.1)
            if chunk:
                chunk_count += 1
                print(f"音频块 {chunk_count}: {len(chunk.data)} bytes")

        print(f"总共获取 {chunk_count} 个音频块")

        # 停止录音
        print("\n3. 停止录音...")
        recorder.stop_recording()
        print("录音已停止")

    else:
        print("录音启动失败")

    print("\n测试完成")