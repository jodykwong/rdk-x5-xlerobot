#!/usr/bin/env python3
"""
ASR音频处理模块

专门为语音识别优化的音频处理：
- 音频数据格式转换
- 音频质量优化
- 语音段检测和分割
- 批量音频处理

作者: Dev Agent
日期: 2025-11-08
Epic: 1 - ASR语音识别模块
Story: 1.1 - 粤语语音识别基础功能
Phase: 3 - 基础语音识别
Task: 3.2 - 实现音频数据格式转换和打包
"""

import numpy as np
import logging
import wave
import io
import base64
from typing import List, Optional, Tuple, Dict, Any
from dataclasses import dataclass
import soundfile as sf
import threading
from queue import Queue, Empty
import time

try:
    import librosa
except ImportError:
    librosa = None
    logging.warning("librosa not available, using fallback resampling methods")

logger = logging.getLogger(__name__)


@dataclass
class AudioSegment:
    """音频段信息"""
    data: np.ndarray
    sample_rate: int
    start_time: float
    end_time: float
    energy: float
    is_speech: bool
    confidence: float


@dataclass
class ASRAudioConfig:
    """ASR音频配置"""
    target_sample_rate: int = 16000
    target_channels: int = 1  # 单声道
    target_format: str = "pcm"  # pcm, wav
    target_bit_depth: int = 16  # 16-bit
    max_segment_duration: float = 30.0  # 最大片段时长(秒)
    min_segment_duration: float = 1.0   # 最小片段时长(秒)
    vad_threshold: float = 0.3  # VAD阈值
    enable_silence_removal: bool = True  # 移除静音
    max_silence_duration: float = 1.0  # 最大静音时长(秒)


class ASRAudioProcessor:
    """
    ASR音频处理器

    专门为语音识别优化的音频处理模块
    """

    def __init__(self, config: Optional[ASRAudioConfig] = None):
        """
        初始化ASR音频处理器

        Args:
            config: 音频配置
        """
        self.config = config or ASRAudioConfig()

        # 音频缓冲区
        self.audio_buffer = Queue(maxsize=1000)
        self.processed_segments = Queue(maxsize=100)

        # 处理状态
        self.is_processing = False
        self.total_samples_processed = 0
        self.total_duration_processed = 0.0

        # 线程安全
        self._lock = threading.Lock()

        logger.info(f"ASRAudioProcessor 初始化完成")
        logger.info(f"  目标采样率: {self.config.target_sample_rate}Hz")
        logger.info(f"  目标格式: {self.config.target_format}")
        logger.info(f"  最大片段时长: {self.config.max_segment_duration}s")

    def convert_audio_format(self,
                             audio_data,
                             source_sample_rate: int,
                             source_channels: int = 1,
                             source_bit_depth: int = 16) -> bytes:
        """
        转换音频格式到ASR要求的格式

        Args:
            audio_data: 原始音频数据 (numpy.ndarray 或 bytes)
            source_sample_rate: 原始采样率
            source_channels: 原始声道数
            source_bit_depth: 原始位深度

        Returns:
            bytes: 转换后的音频数据
        """
        try:
            with self._lock:
                # 类型检查和转换
                if isinstance(audio_data, bytes):
                    # 如果是bytes，转换为numpy数组
                    try:
                        # 假设16位PCM数据
                        audio_data = np.frombuffer(audio_data, dtype=np.int16)
                        logger.debug(f"音频数据从bytes转换为numpy数组，长度: {len(audio_data)}")
                    except Exception as e:
                        logger.error(f"bytes转numpy数组失败: {e}")
                        return b""
                elif not isinstance(audio_data, np.ndarray):
                    logger.error(f"音频数据必须是numpy.ndarray或bytes，但得到: {type(audio_data)}")
                    return b""

                # 现在audio_data保证是numpy数组，继续处理
                # 转换为单声道
                if source_channels > 1:
                    audio_data = np.mean(audio_data.reshape(-1, source_channels), axis=1)
                elif source_channels == 0:
                    audio_data = audio_data

                # 转换采样率
                if source_sample_rate != self.config.target_sample_rate:
                    if librosa is not None:
                        audio_data = librosa.resample(
                            audio_data,
                            orig_sr=source_sample_rate,
                            target_sr=self.config.target_sample_rate
                        )
                    else:
                        # 使用简单的线性插值
                        audio_data = self._resample_audio_linear(
                            audio_data,
                            source_sample_rate,
                            self.config.target_sample_rate
                        )

                # 转换位深度
                if source_bit_depth != self.config.target_bit_depth:
                    audio_data = self._convert_bit_depth(
                        audio_data,
                        source_bit_depth,
                        self.config.target_bit_depth
                    )

                # 归一化到[-1, 1]范围
                audio_data = np.clip(audio_data / 32768.0, -1.0, 1.0)

                # 转换为字节流
                if self.config.target_format.lower() == "pcm":
                    return self._convert_to_pcm(audio_data)
                elif self.config.target_format.lower() == "wav":
                    return self._convert_to_wav(audio_data)
                else:
                    raise ValueError(f"不支持的音频格式: {self.config.target_format}")

        except Exception as e:
            logger.error(f"音频格式转换失败: {e}")
            return b""

    def _resample_audio_linear(self, audio_data: np.ndarray, source_sr: int, target_sr: int) -> np.ndarray:
        """线性插值重采样"""
        try:
            # 计算重采样比例
            ratio = target_sr / source_sr
            target_length = int(len(audio_data) * ratio)

            # 线性插值
            indices = np.linspace(0, len(audio_data) - 1, target_length)
            return np.interp(indices, np.arange(len(audio_data)), audio_data)
        except Exception as e:
            logger.error(f"线性重采样失败: {e}")
            return audio_data

    def _convert_bit_depth(self, audio_data: np.ndarray, source_depth: int, target_depth: int) -> np.ndarray:
        """转换位深度"""
        try:
            if source_depth == 16 and target_depth == 16:
                return audio_data
            elif source_depth == 32 and target_depth == 16:
                return (audio_data / 65536.0).astype(np.float32)
            elif source_depth == 16 and target_depth == 32:
                return (audio_data * 65536.0).astype(np.int32)
            else:
                # 通用转换
                max_val = 2 ** (source_depth - 1)
                target_max_val = 2 ** (target_depth - 1)
                return (audio_data / max_val * target_max_val).astype(np.int16 if target_depth == 16 else np.int32)
        except Exception as e:
            logger.error(f"位深度转换失败: {e}")
            return audio_data

    def _convert_to_pcm(self, audio_data: np.ndarray) -> bytes:
        """转换为PCM格式"""
        try:
            # 转换为16位整数
            pcm_data = (audio_data * 32767).astype(np.int16)
            return pcm_data.tobytes()
        except Exception as e:
            logger.error(f"PCM转换失败: {e}")
            return b""

    def _convert_to_wav(self, audio_data: np.ndarray) -> bytes:
        """转换为WAV格式"""
        try:
            buffer = io.BytesIO()

            with wave.open(buffer, 'wb') as wav_file:
                wav_file.setnchannels(self.config.target_channels)
                wav_file.setsampwidth(self.config.target_bit_depth // 8)
                wav_file.setframerate(self.config.target_sample_rate)
                wav_file.writeframes(audio_data.astype(np.int16))

            return buffer.getvalue()
        except Exception as e:
            logger.error(f"WAV转换失败: {e}")
            return b""

    def convert_to_base64(self, audio_bytes: bytes) -> str:
        """
        转换为Base64格式

        Args:
            audio_bytes: 音频字节数据

        Returns:
            str: Base64编码的字符串
        """
        try:
            return base64.b64encode(audio_bytes).decode('utf-8')
        except Exception as e:
            logger.error(f"Base64转换失败: {e}")
            return ""

    def detect_voice_activity(self, audio_data: np.ndarray, sample_rate: int) -> np.ndarray:
        """
        语音活动检测

        Args:
            audio_data: 音频数据
            sample_rate: 采样率

        Returns:
            np.ndarray: VAD标记
        """
        try:
            # 简单的能量VAD
            frame_length = min(len(audio_data), 1024)
            hop_length = 512

            frames = []
            for i in range(0, len(audio_data) - frame_length + 1, hop_length):
                frame = audio_data[i:i + frame_length]
                energy = np.sum(frame ** 2)
                frames.append(energy)

            if not frames:
                return np.array([True])  # 空音频默认假设为语音

            # 归一化能量
            energy_array = np.array(frames)
            max_energy = np.max(energy_array)
            if max_energy > 0:
                normalized_energy = energy_array / max_energy
            else:
                normalized_energy = energy_array

            # 应用阈值
            vad_flags = normalized_energy > self.config.vad_threshold

            return vad_flags

        except Exception as e:
            logger.error(f"VAD检测失败: {e}")
            return np.array([True] * (len(audio_data) // 512))  # 默认假设为语音

    def segment_audio(self, audio_data: np.ndarray, sample_rate: int) -> List[AudioSegment]:
        """
        分割音频为语音段

        Args:
            audio_data: 音频数据
            sample_rate: 采样率

        Returns:
            List[AudioSegment]: 音频段列表
        """
        try:
            segments = []

            # 检测语音活动
            vad_flags = self.detect_voice_activity(audio_data, sample_rate)
            frame_length = min(len(audio_data), 1024)
            hop_length = 512

            # 生成音频段
            i = 0
            while i < len(audio_data):
                if i + frame_length > len(audio_data):
                    break

                frame = audio_data[i:i + frame_length]
                is_speech = i // hop_length < len(vad_flags) and vad_flags[i // hop_length]

                start_time = i / sample_rate
                end_time = (i + len(frame)) / sample_rate
                duration = end_time - start_time

                # 检查是否满足最小/最大时长要求
                if duration >= self.config.min_segment_duration and duration <= self.config.max_segment_duration:
                    energy = np.sum(frame ** 2)
                    segment = AudioSegment(
                        data=frame,
                        sample_rate=sample_rate,
                        start_time=start_time,
                        end_time=end_time,
                        energy=energy,
                        is_speech=is_speech,
                        confidence=0.8 if is_speech else 0.2
                    )
                    segments.append(segment)

                i += hop_length

            # 合并相邻的语音段
            merged_segments = self._merge_adjacent_segments(segments)

            logger.info(f"音频分割完成: {len(merged_segments)} 个段")
            return merged_segments

        except Exception as e:
            logger.error(f"音频分割失败: {e}")
            return []

    def _merge_adjacent_segments(self, segments: List[AudioSegment]) -> List[AudioSegment]:
        """合并相邻的音频段"""
        if not segments:
            return []

        merged = [segments[0]]

        for current in segments[1:]:
            last = merged[-1]

            # 检查是否相邻且都是语音
            if (current.is_speech and last.is_speech and
                current.start_time - last.end_time < 0.5):  # 间隔小于0.5秒
                # 合并段
                merged_data = np.concatenate([last.data, current.data])
                merged[-1] = AudioSegment(
                    data=merged_data,
                    sample_rate=current.sample_rate,
                    start_time=last.start_time,
                    end_time=current.end_time,
                    energy=last.energy + current.energy,
                    is_speech=True,
                    confidence=(last.confidence + current.confidence) / 2
                )
            else:
                merged.append(current)

        return merged

    def process_audio_batch(self, audio_batch: List[Tuple[np.ndarray, int]]) -> List[bytes]:
        """
        批量处理音频数据

        Args:
            audio_batch: 音频批次 [(audio_data, sample_rate), ...]

        Returns:
            List[bytes]: 处理后的音频数据
        """
        try:
            processed_audio = []

            for audio_data, sample_rate in audio_batch:
                # 转换格式
                audio_bytes = self.convert_audio_format(
                    audio_data, sample_rate
                )

                if audio_bytes:
                    processed_audio.append(audio_bytes)

            logger.info(f"批量音频处理完成: {len(processed_audio)}/{len(audio_batch)} 个音频")
            return processed_audio

        except Exception as e:
            logger.error(f"批量音频处理失败: {e}")
            return []

    def add_audio_to_buffer(self, audio_data: np.ndarray, sample_rate: int) -> bool:
        """
        添加音频到缓冲区

        Args:
            audio_data: 音频数据
            sample_rate: 采样率

        Returns:
            bool: 是否添加成功
        """
        try:
            # 转换为ASR格式
            audio_bytes = self.convert_audio_format(audio_data, sample_rate)

            if audio_bytes:
                self.audio_buffer.put(audio_bytes)
                return True

            return False

        except Exception as e:
            logger.error(f"添加音频到缓冲区失败: {e}")
            return False

    def get_processed_audio(self, timeout: float = 1.0) -> Optional[bytes]:
        """
        从处理队列获取音频

        Args:
            timeout: 超时时间(秒)

        Returns:
            bytes: 处理后的音频数据
        """
        try:
            return self.processed_segments.get(timeout=timeout)
        except Empty:
            return None

    def start_processing(self) -> None:
        """开始音频处理"""
        with self._lock:
            self.is_processing = True
        logger.info("开始ASR音频处理")

    def stop_processing(self) -> None:
        """停止音频处理"""
        with self._lock:
            self.is_processing = False
        logger.info("停止ASR音频处理")

    def get_processing_statistics(self) -> Dict[str, Any]:
        """
        获取处理统计信息

        Returns:
            Dict: 统计信息
        """
        with self._lock:
            return {
                "is_processing": self.is_processing,
                "total_samples_processed": self.total_samples_processed,
                "total_duration_processed": self.total_duration_processed,
                "buffer_size": self.audio_buffer.qsize(),
                "processed_segments_count": self.processed_segments.qsize(),
                "target_sample_rate": self.config.target_sample_rate,
                "target_format": self.config.target_format
            }

    def reset_statistics(self) -> None:
        """重置统计信息"""
        with self._lock:
            self.total_samples_processed = 0
            self.total_duration_processed = 0.0
        logger.info("ASR音频处理统计信息已重置")


# 工厂函数
def create_asr_audio_processor(sample_rate: int = 16000,
                              format: str = "pcm",
                              **kwargs) -> ASRAudioProcessor:
    """
    创建ASR音频处理器的工厂函数

    Args:
        sample_rate: 采样率
        format: 音频格式
        **kwargs: 其他配置参数

    Returns:
        ASRAudioProcessor: ASR音频处理器实例
    """
    config = ASRAudioConfig(
        target_sample_rate=sample_rate,
        target_format=format,
        **kwargs
    )

    return ASRAudioProcessor(config)


if __name__ == "__main__":
    # 测试代码
    logging.basicConfig(level=logging.INFO)

    print("🎵 ASR音频处理器测试")
    print("=" * 50)

    # 创建处理器
    processor = create_asr_audio_processor(
        sample_rate=16000,
        format="pcm"
    )

    # 创建测试音频
    duration = 5.0
    sample_rate = 16000
    t = np.linspace(0, duration, int(sample_rate * duration))

    # 生成混合信号（语音 + 静音 + 噪声）
    speech_signal = 0.5 * np.sin(2 * np.pi * 440 * t)  # 语音
    silence_segment = np.zeros(int(sample_rate * 1.0))  # 1秒静音
    noise_segment = 0.1 * np.random.randn(int(sample_rate * 2.0))  # 2秒噪声

    test_audio = np.concatenate([speech_signal, silence_segment, noise_segment])

    print(f"\n🎵 测试音频处理")
    print(f"音频长度: {len(test_audio)} 样本 ({duration:.1f}秒)")
    print(f"采样率: {sample_rate}Hz")

    # 转换音频格式
    converted_audio = processor.convert_audio_format(test_audio, sample_rate)
    print(f"转换后音频大小: {len(converted_audio)} 字节")

    # 音频分割
    segments = processor.segment_audio(test_audio, sample_rate)
    print(f"音频分割结果: {len(segments)} 个段")

    # 显示段信息
    for i, segment in enumerate(segments):
        print(f"  段 {i+1}: {segment.start_time:.2f}s - {segment.end_time:.2f}s, "
              f"语音={segment.is_speech}, 能量={segment.energy:.2f}")

    # 获取统计信息
    stats = processor.get_processing_statistics()
    print(f"\n📊 处理统计:")
    for key, value in stats.items():
        print(f"  {key}: {value}")

    print("✅ 测试完成")