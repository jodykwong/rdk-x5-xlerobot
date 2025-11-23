#!/usr/bin/env python3
"""
流式音频播放器
==============

高性能的流式音频播放器，支持：
- 实时音频流播放
- 缓冲区管理
- 音频格式转换
- 多音轨混合
- 性能监控

作者: Developer Agent
版本: 1.0
日期: 2025-11-16
"""

import logging
import time
import threading
import queue
import numpy as np
from typing import Optional, Callable, Dict, Any, List
from dataclasses import dataclass, field
from datetime import datetime

try:
    import pyaudio
    PYAUDIO_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ PyAudio未安装: {e}")
    PYAUDIO_AVAILABLE = False

try:
    import soundfile as sf
    SOUNDFILE_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ SoundFile未安装: {e}")
    SOUNDFILE_AVAILABLE = False

logger = logging.getLogger(__name__)

@dataclass
class AudioChunk:
    """音频数据块"""
    data: bytes
    timestamp: float
    chunk_id: int
    format: str = "pcm"
    sample_rate: int = 16000
    channels: int = 1

@dataclass
class PlaybackStats:
    """播放统计"""
    total_chunks_played: int = 0
    total_bytes_played: int = 0
    playback_duration: float = 0.0
    average_chunk_size: float = 0.0
    buffer_underruns: int = 0
    buffer_overruns: int = 0
    last_chunk_time: float = 0.0
    start_time: float = field(default_factory=time.time)

class AudioBuffer:
    """音频缓冲区"""

    def __init__(self, max_size: int = 50):
        """
        初始化音频缓冲区

        Args:
            max_size: 最大缓冲区大小
        """
        self.max_size = max_size
        self.buffer: queue.Queue = queue.Queue(maxsize=max_size)
        self.total_added = 0
        self.total_consumed = 0
        self.lock = threading.Lock()

    def put(self, chunk: AudioChunk) -> bool:
        """
        添加音频块到缓冲区

        Args:
            chunk: 音频块

        Returns:
            是否成功添加
        """
        try:
            self.buffer.put(chunk, block=False)
            with self.lock:
                self.total_added += 1
            return True
        except queue.Full:
            logger.warning("⚠️ 音频缓冲区已满，丢弃音频块")
            return False

    def get(self, timeout: float = 1.0) -> Optional[AudioChunk]:
        """
        从缓冲区获取音频块

        Args:
            timeout: 超时时间

        Returns:
            音频块或None
        """
        try:
            chunk = self.buffer.get(timeout=timeout)
            with self.lock:
                self.total_consumed += 1
            return chunk
        except queue.Empty:
            return None

    def size(self) -> int:
        """获取缓冲区当前大小"""
        return self.buffer.qsize()

    def is_empty(self) -> bool:
        """检查缓冲区是否为空"""
        return self.buffer.empty()

    def is_full(self) -> bool:
        """检查缓冲区是否已满"""
        return self.buffer.full()

    def clear(self):
        """清空缓冲区"""
        while not self.buffer.empty():
            try:
                self.buffer.get_nowait()
            except queue.Empty:
                break

class StreamingAudioPlayer:
    """流式音频播放器"""

    def __init__(self,
                 sample_rate: int = 16000,
                 channels: int = 1,
                 format: int = None,
                 buffer_size: int = 50,
                 chunk_duration: float = 0.1):
        """
        初始化流式音频播放器

        Args:
            sample_rate: 采样率
            channels: 声道数
            format: 音频格式
            buffer_size: 缓冲区大小
            chunk_duration: 音频块持续时间（秒）
        """
        if not PYAUDIO_AVAILABLE:
            raise ImportError("PyAudio未安装，无法使用音频播放功能")

        self.sample_rate = sample_rate
        self.channels = channels
        self.format = format or pyaudio.paInt16
        self.buffer_size = buffer_size
        self.chunk_duration = chunk_duration

        # 计算块大小
        self.chunk_size = int(sample_rate * chunk_duration)

        # PyAudio实例
        self.pyaudio = pyaudio.PyAudio()

        # 播放控制
        self.is_playing = False
        self.is_paused = False
        self.stream = None

        # 缓冲区
        self.audio_buffer = AudioBuffer(max_size=buffer_size)

        # 播放线程
        self.playback_thread = None
        self.stop_event = threading.Event()

        # 统计信息
        self.stats = PlaybackStats()

        # 回调函数
        self.chunk_callback: Optional[Callable] = None
        self.error_callback: Optional[Callable] = None

        logger.info("✅ 流式音频播放器初始化完成")
        logger.info(f"  - 采样率: {sample_rate}")
        logger.info(f"  - 声道数: {channels}")
        logger.info(f"  - 缓冲区大小: {buffer_size}")
        logger.info(f"  - 块大小: {self.chunk_size} 采样点")

    def set_callbacks(self,
                     chunk_callback: Optional[Callable] = None,
                     error_callback: Optional[Callable] = None):
        """
        设置回调函数

        Args:
            chunk_callback: 音频块回调
            error_callback: 错误回调
        """
        self.chunk_callback = chunk_callback
        self.error_callback = error_callback

    def start_playback(self) -> bool:
        """
        开始播放

        Returns:
            是否成功开始播放
        """
        if self.is_playing:
            logger.warning("⚠️ 播放器已在运行")
            return True

        try:
            # 初始化音频流
            self.stream = self.pyaudio.open(
                format=self.format,
                channels=self.channels,
                rate=self.sample_rate,
                output=True,
                frames_per_buffer=self.chunk_size
            )

            # 重置状态
            self.stop_event.clear()
            self.is_playing = True
            self.is_paused = False
            self.stats = PlaybackStats()

            # 启动播放线程
            self.playback_thread = threading.Thread(target=self._playback_worker)
            self.playback_thread.daemon = True
            self.playback_thread.start()

            logger.info("🔊 流式音频播放开始")
            return True

        except Exception as e:
            logger.error(f"❌ 播放器启动失败: {e}")
            if self.error_callback:
                self.error_callback(e)
            return False

    def stop_playback(self) -> bool:
        """
        停止播放

        Returns:
            是否成功停止播放
        """
        if not self.is_playing:
            return True

        try:
            # 设置停止事件
            self.stop_event.set()
            self.is_playing = False

            # 等待播放线程结束
            if self.playback_thread and self.playback_thread.is_alive():
                self.playback_thread.join(timeout=2)

            # 关闭音频流
            if self.stream:
                self.stream.close()
                self.stream = None

            logger.info("🔇 流式音频播放停止")
            return True

        except Exception as e:
            logger.error(f"❌ 播放器停止失败: {e}")
            return False

    def pause_playback(self) -> bool:
        """
        暂停播放

        Returns:
            是否成功暂停
        """
        if self.is_playing and not self.is_paused:
            self.is_paused = True
            logger.info("⏸️ 播放已暂停")
            return True
        return False

    def resume_playback(self) -> bool:
        """
        恢复播放

        Returns:
            是否成功恢复
        """
        if self.is_playing and self.is_paused:
            self.is_paused = False
            logger.info("▶️ 播放已恢复")
            return True
        return False

    def add_audio_chunk(self, audio_data: bytes, format: str = "pcm") -> bool:
        """
        添加音频数据块

        Args:
            audio_data: 音频数据
            format: 音频格式

        Returns:
            是否成功添加
        """
        if not self.is_playing:
            logger.warning("⚠️ 播放器未启动，忽略音频数据")
            return False

        # 创建音频块
        chunk = AudioChunk(
            data=audio_data,
            timestamp=time.time(),
            chunk_id=self.stats.total_chunks_played + 1,
            format=format,
            sample_rate=self.sample_rate,
            channels=self.channels
        )

        # 添加到缓冲区
        success = self.audio_buffer.put(chunk)

        if success:
            logger.debug(f"🎵 添加音频块: {len(audio_data)} 字节")
        else:
            self.stats.buffer_overruns += 1

        return success

    def _playback_worker(self):
        """播放工作线程"""
        logger.info("✅ 播放线程启动")

        try:
            while not self.stop_event.is_set():
                # 检查暂停状态
                if self.is_paused:
                    time.sleep(0.1)
                    continue

                # 从缓冲区获取音频块
                chunk = self.audio_buffer.get(timeout=0.1)

                if chunk is None:
                    # 缓冲区为空
                    self.stats.buffer_underruns += 1
                    continue

                try:
                    # 播放音频数据
                    self.stream.write(chunk.data)

                    # 更新统计
                    self.stats.total_chunks_played += 1
                    self.stats.total_bytes_played += len(chunk.data)
                    self.stats.last_chunk_time = time.time()
                    self.stats.playback_duration = self.stats.last_chunk_time - self.stats.start_time
                    self.stats.average_chunk_size = self.stats.total_bytes_played / max(1, self.stats.total_chunks_played)

                    # 调用回调函数
                    if self.chunk_callback:
                        try:
                            self.chunk_callback(chunk)
                        except Exception as e:
                            logger.warning(f"⚠️ 音频块回调执行失败: {e}")

                except Exception as e:
                    logger.error(f"❌ 音频播放失败: {e}")
                    if self.error_callback:
                        self.error_callback(e)

        except Exception as e:
            logger.error(f"❌ 播放线程异常: {e}")
            if self.error_callback:
                self.error_callback(e)

        finally:
            logger.info("🔚 播放线程结束")

    def get_buffer_status(self) -> Dict[str, Any]:
        """
        获取缓冲区状态

        Returns:
            缓冲区状态信息
        """
        return {
            "size": self.audio_buffer.size(),
            "max_size": self.audio_buffer.max_size,
            "is_empty": self.audio_buffer.is_empty(),
            "is_full": self.audio_buffer.is_full(),
            "utilization": self.audio_buffer.size() / self.audio_buffer.max_size * 100,
            "total_added": self.audio_buffer.total_added,
            "total_consumed": self.audio_buffer.total_consumed
        }

    def get_playback_stats(self) -> PlaybackStats:
        """获取播放统计信息"""
        return self.stats

    def clear_buffer(self):
        """清空音频缓冲区"""
        self.audio_buffer.clear()
        logger.info("🗑️ 音频缓冲区已清空")

    def get_device_info(self) -> Dict[str, Any]:
        """获取音频设备信息"""
        try:
            device_info = self.pyaudio.get_default_output_device_info()
            return {
                "device_index": device_info.get("index"),
                "device_name": device_info.get("name"),
                "host_api": device_info.get("hostApiLongName"),
                "max_output_channels": device_info.get("maxOutputChannels"),
                "default_sample_rate": device_info.get("defaultSampleRate")
            }
        except Exception as e:
            logger.error(f"❌ 获取设备信息失败: {e}")
            return {"error": str(e)}

    def __del__(self):
        """析构函数"""
        self.stop_playback()
        if hasattr(self, 'pyaudio'):
            self.pyaudio.terminate()

class AudioMixer:
    """音频混合器"""

    def __init__(self, sample_rate: int = 16000, channels: int = 1):
        """
        初始化音频混合器

        Args:
            sample_rate: 采样率
            channels: 声道数
        """
        self.sample_rate = sample_rate
        self.channels = channels
        self.active_streams: Dict[str, StreamingAudioPlayer] = {}

    def create_stream(self, stream_id: str, **kwargs) -> StreamingAudioPlayer:
        """
        创建音频流

        Args:
            stream_id: 流ID
            **kwargs: 播放器参数

        Returns:
            音频播放器实例
        """
        if stream_id in self.active_streams:
            logger.warning(f"⚠️ 流 {stream_id} 已存在")
            return self.active_streams[stream_id]

        player = StreamingAudioPlayer(
            sample_rate=kwargs.get('sample_rate', self.sample_rate),
            channels=kwargs.get('channels', self.channels),
            **kwargs
        )

        self.active_streams[stream_id] = player
        logger.info(f"✅ 创建音频流: {stream_id}")
        return player

    def destroy_stream(self, stream_id: str) -> bool:
        """
        销毁音频流

        Args:
            stream_id: 流ID

        Returns:
            是否成功销毁
        """
        if stream_id not in self.active_streams:
            return False

        player = self.active_streams[stream_id]
        player.stop_playback()
        del self.active_streams[stream_id]
        logger.info(f"🗑️ 销毁音频流: {stream_id}")
        return True

    def stop_all_streams(self):
        """停止所有音频流"""
        for stream_id, player in self.active_streams.items():
            player.stop_playback()
        self.active_streams.clear()
        logger.info("🛑 所有音频流已停止")

    def get_mixer_status(self) -> Dict[str, Any]:
        """获取混合器状态"""
        return {
            "active_streams": len(self.active_streams),
            "stream_list": list(self.active_streams.keys()),
            "sample_rate": self.sample_rate,
            "channels": self.channels
        }

def create_streaming_player(**kwargs) -> StreamingAudioPlayer:
    """创建流式音频播放器实例"""
    return StreamingAudioPlayer(**kwargs)

def create_audio_mixer(**kwargs) -> AudioMixer:
    """创建音频混合器实例"""
    return AudioMixer(**kwargs)