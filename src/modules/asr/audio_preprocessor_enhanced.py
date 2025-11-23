#!/usr/bin/env python3
"""
音频预处理器增强版 - Enhanced Audio Preprocessor
================================================

专为粤语ASR优化的音频预处理组件。
提供噪声检测、音频增强和分段处理功能。

功能：
- 环境噪声检测和分类
- 音频质量评估
- 自适应噪声抑制
- 长句语音分段处理
- 粤语音频特性优化

约束：
- 纯在线架构，无离线存储
- 轻量级处理，低延迟
- 专为粤语语音优化

作者: Developer Agent
版本: 1.2 (Story 1.2 粤语优化)
日期: 2025-11-09
"""

import numpy as np
import logging
import time
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass, field
from enum import Enum
import base64

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class NoiseType(Enum):
    """噪声类型枚举"""
    QUIET = "quiet"           # 安静环境
    NORMAL = "normal"         # 正常环境
    NOISY = "noisy"           # 噪声环境
    VERY_NOISY = "very_noisy" # 极噪声环境
    UNKNOWN = "unknown"       # 未知环境


class AudioQuality(Enum):
    """音频质量等级"""
    EXCELLENT = "excellent"
    GOOD = "good"
    FAIR = "fair"
    POOR = "poor"


@dataclass
class AudioAnalysisResult:
    """音频分析结果"""
    noise_type: NoiseType
    quality: AudioQuality
    snr_db: float
    duration: float
    amplitude_stats: Dict[str, float]
    recommendation: str


@dataclass
class PreprocessingResult:
    """预处理结果"""
    success: bool
    processed_audio: Optional[bytes]
    analysis: AudioAnalysisResult
    processing_time: float
    segments_count: int
    error: Optional[str] = None


class AudioPreprocessorEnhanced:
    """
    音频预处理器增强版

    专为粤语ASR优化的音频预处理系统，提供：
    1. 噪声检测和分类
    2. 音频质量评估
    3. 智能分段处理
    4. 粤语特性优化
    """

    def __init__(self, sample_rate: int = 16000):
        """
        初始化音频预处理器

        Args:
            sample_rate: 采样率
        """
        self.sample_rate = sample_rate
        self.frame_size = 1024
        self.hop_size = 512

        # 粤语语音特征参数
        self.cantonese_vad_threshold = 0.02  # 语音活动检测阈值
        self.min_segment_duration = 0.5      # 最小分段时长(秒)
        self.max_segment_duration = 15.0     # 最大分段时长(秒)
        self.silence_threshold = 0.01        # 静音阈值

        # 噪声检测参数
        self.noise_thresholds = {
            NoiseType.QUIET: 25.0,      # SNR > 25dB
            NoiseType.NORMAL: 15.0,     # SNR > 15dB
            NoiseType.NOISY: 5.0,       # SNR > 5dB
        }

        logger.info(f"AudioPreprocessorEnhanced初始化完成，采样率: {sample_rate}Hz")

    def analyze_and_preprocess(self, audio_data: bytes,
                             enable_segmentation: bool = True) -> PreprocessingResult:
        """
        分析和预处理音频

        Args:
            audio_data: 音频数据
            enable_segmentation: 是否启用分段处理

        Returns:
            PreprocessingResult: 预处理结果
        """
        start_time = time.time()

        try:
            # 转换音频数据
            audio_array = self._convert_to_array(audio_data)
            if audio_array is None:
                return PreprocessingResult(
                    success=False,
                    processed_audio=None,
                    analysis=None,
                    processing_time=time.time() - start_time,
                    segments_count=0,
                    error="音频数据转换失败"
                )

            # 音频分析
            analysis = self._analyze_audio(audio_array)

            # 根据分析结果进行处理
            processed_audio = audio_data
            segments_count = 1

            if enable_segmentation and analysis.duration > self.max_segment_duration:
                # 长音频分段处理
                segments = self._segment_audio(audio_array)
                segments_count = len(segments)

                if segments_count > 1:
                    # 选择最佳分段
                    best_segment = self._select_best_segment(segments)
                    processed_audio = self._convert_to_wav(best_segment)

            elif analysis.quality in [AudioQuality.POOR, AudioQuality.FAIR]:
                # 音频增强处理
                enhanced_audio = self._enhance_audio(audio_array, analysis)
                processed_audio = self._convert_to_wav(enhanced_audio)

            processing_time = time.time() - start_time

            logger.info(f"音频预处理完成: 质量={analysis.quality.value}, "
                       f"噪声={analysis.noise_type.value}, "
                       f"分段数={segments_count}, "
                       f"处理时间={processing_time:.3f}s")

            return PreprocessingResult(
                success=True,
                processed_audio=processed_audio,
                analysis=analysis,
                processing_time=processing_time,
                segments_count=segments_count
            )

        except Exception as e:
            return PreprocessingResult(
                success=False,
                processed_audio=None,
                analysis=None,
                processing_time=time.time() - start_time,
                segments_count=0,
                error=f"预处理失败: {str(e)}"
            )

    def _convert_to_array(self, audio_data: bytes) -> Optional[np.ndarray]:
        """
        转换音频数据为numpy数组

        Args:
            audio_data: 音频数据

        Returns:
            np.ndarray: 音频数组
        """
        try:
            # 处理Base64编码
            if isinstance(audio_data, str):
                audio_bytes = base64.b64decode(audio_data)
            else:
                audio_bytes = audio_data

            # 处理WAV格式
            if audio_bytes.startswith(b'RIFF'):
                import wave
                import io
                wav_buffer = io.BytesIO(audio_bytes)
                with wave.open(wav_buffer, 'rb') as wav_file:
                    frames = wav_file.readframes(-1)
                    return np.frombuffer(frames, dtype=np.int16).astype(np.float32) / 32768.0
            else:
                # 假设16位PCM
                return np.frombuffer(audio_bytes, dtype=np.int16).astype(np.float32) / 32768.0

        except Exception as e:
            logger.error(f"音频转换失败: {e}")
            return None

    def _analyze_audio(self, audio_array: np.ndarray) -> AudioAnalysisResult:
        """
        分析音频特征

        Args:
            audio_array: 音频数组

        Returns:
            AudioAnalysisResult: 分析结果
        """
        # 基础统计
        duration = len(audio_array) / self.sample_rate
        amplitude_mean = np.mean(np.abs(audio_array))
        amplitude_std = np.std(audio_array)
        amplitude_max = np.max(np.abs(audio_array))

        amplitude_stats = {
            "mean": amplitude_mean,
            "std": amplitude_std,
            "max": amplitude_max,
            "min": np.min(np.abs(audio_array))
        }

        # 信噪比估算
        noise_floor = np.percentile(np.abs(audio_array), 10)
        signal_level = np.percentile(np.abs(audio_array), 90)

        if noise_floor > 0:
            snr_db = 20 * np.log10(signal_level / noise_floor)
        else:
            snr_db = 30.0  # 假设良好信号

        # 噪声类型分类
        if snr_db > self.noise_thresholds[NoiseType.QUIET]:
            noise_type = NoiseType.QUIET
        elif snr_db > self.noise_thresholds[NoiseType.NORMAL]:
            noise_type = NoiseType.NORMAL
        elif snr_db > self.noise_thresholds[NoiseType.NOISY]:
            noise_type = NoiseType.NOISY
        else:
            noise_type = NoiseType.VERY_NOISY

        # 音频质量评估
        if snr_db > 25 and amplitude_mean > 0.05 and amplitude_std > 0.1:
            quality = AudioQuality.EXCELLENT
        elif snr_db > 15 and amplitude_mean > 0.03:
            quality = AudioQuality.GOOD
        elif snr_db > 8 and amplitude_mean > 0.01:
            quality = AudioQuality.FAIR
        else:
            quality = AudioQuality.POOR

        # 生成处理建议
        recommendation = self._generate_recommendation(quality, noise_type, duration)

        return AudioAnalysisResult(
            noise_type=noise_type,
            quality=quality,
            snr_db=snr_db,
            duration=duration,
            amplitude_stats=amplitude_stats,
            recommendation=recommendation
        )

    def _generate_recommendation(self, quality: AudioQuality,
                               noise_type: NoiseType,
                               duration: float) -> str:
        """
        生成处理建议

        Args:
            quality: 音频质量
            noise_type: 噪声类型
            duration: 音频时长

        Returns:
            str: 处理建议
        """
        recommendations = []

        if quality == AudioQuality.POOR:
            recommendations.append("需要音频增强")
        elif quality == AudioQuality.FAIR:
            recommendations.append("建议音频增强")

        if noise_type in [NoiseType.NOISY, NoiseType.VERY_NOISY]:
            recommendations.append("启用噪声抑制")

        if duration > self.max_segment_duration:
            recommendations.append("需要分段处理")

        if duration < 0.5:
            recommendations.append("音频过短，可能影响识别")

        return "; ".join(recommendations) if recommendations else "音频质量良好"

    def _segment_audio(self, audio_array: np.ndarray) -> List[np.ndarray]:
        """
        音频分段处理

        Args:
            audio_array: 音频数组

        Returns:
            List[np.ndarray]: 分段列表
        """
        try:
            # 计算语音活动
            energy = np.array([
                np.sum(audio_array[i:i+self.frame_size] ** 2)
                for i in range(0, len(audio_array) - self.frame_size, self.hop_size)
            ])

            # 归一化能量
            energy = energy / (np.max(energy) + 1e-10)

            # 语音检测
            voice_frames = energy > self.cantonese_vad_threshold

            # 寻找语音段
            segments = []
            in_speech = False
            start_idx = 0

            for i, is_voice in enumerate(voice_frames):
                frame_start = i * self.hop_size

                if is_voice and not in_speech:
                    # 开始语音段
                    in_speech = True
                    start_idx = frame_start
                elif not is_voice and in_speech:
                    # 结束语音段
                    in_speech = False
                    segment_end = min(frame_start + self.frame_size, len(audio_array))
                    segment_duration = (segment_end - start_idx) / self.sample_rate

                    if segment_duration >= self.min_segment_duration:
                        segments.append(audio_array[start_idx:segment_end])

            # 处理最后一个语音段
            if in_speech:
                segment_end = len(audio_array)
                segment_duration = (segment_end - start_idx) / self.sample_rate
                if segment_duration >= self.min_segment_duration:
                    segments.append(audio_array[start_idx:segment_end])

            # 如果没有检测到语音段，使用原始音频
            if not segments:
                segments = [audio_array]

            return segments

        except Exception as e:
            logger.warning(f"音频分段失败: {e}")
            return [audio_array]

    def _select_best_segment(self, segments: List[np.ndarray]) -> np.ndarray:
        """
        选择最佳音频段

        Args:
            segments: 音频段列表

        Returns:
            np.ndarray: 最佳音频段
        """
        if not segments:
            return np.array([])

        # 计算每个段的质量分数
        scores = []
        for segment in segments:
            # 计算能量和方差
            energy = np.mean(segment ** 2)
            variance = np.var(segment)
            duration = len(segment) / self.sample_rate

            # 质量分数（综合能量、方差、时长）
            score = energy * variance * min(duration, 5.0) / 5.0
            scores.append(score)

        # 选择最高分数的段
        best_idx = np.argmax(scores)
        return segments[best_idx]

    def _enhance_audio(self, audio_array: np.ndarray,
                      analysis: AudioAnalysisResult) -> np.ndarray:
        """
        音频增强处理

        Args:
            audio_array: 音频数组
            analysis: 音频分析结果

        Returns:
            np.ndarray: 增强后的音频
        """
        enhanced = audio_array.copy()

        try:
            # 根据噪声类型应用不同的增强策略
            if analysis.noise_type in [NoiseType.NOISY, NoiseType.VERY_NOISY]:
                # 简单的噪声抑制
                enhanced = self._apply_noise_reduction(enhanced)

            # 音频增益调整
            if analysis.quality in [AudioQuality.POOR, AudioQuality.FAIR]:
                gain = 1.2 if analysis.quality == AudioQuality.POOR else 1.1
                enhanced = enhanced * gain

            # 限幅处理，防止失真
            max_amplitude = 0.95
            enhanced = np.clip(enhanced, -max_amplitude, max_amplitude)

        except Exception as e:
            logger.warning(f"音频增强失败: {e}")

        return enhanced

    def _apply_noise_reduction(self, audio_array: np.ndarray) -> np.ndarray:
        """
        应用噪声抑制

        Args:
            audio_array: 音频数组

        Returns:
            np.ndarray: 抑制后的音频
        """
        # 简单的谱减法噪声抑制
        try:
            # 分帧
            frames = []
            for i in range(0, len(audio_array) - self.frame_size, self.hop_size):
                frame = audio_array[i:i + self.frame_size]
                frames.append(frame)

            # 估算噪声谱（使用前几帧）
            if len(frames) > 5:
                noise_frames = frames[:3]
                noise_spectrum = np.mean([np.abs(np.fft.fft(frame)) for frame in noise_frames], axis=0)

                # 处理每帧
                enhanced_frames = []
                for frame in frames:
                    frame_spectrum = np.fft.fft(frame)
                    magnitude = np.abs(frame_spectrum)

                    # 谱减法
                    alpha = 2.0  # 过减因子
                    enhanced_magnitude = magnitude - alpha * noise_spectrum
                    enhanced_magnitude = np.maximum(enhanced_magnitude, 0.1 * magnitude)

                    # 重构信号
                    enhanced_frame = np.real(np.fft.ifft(enhanced_magnitude * np.exp(1j * np.angle(frame_spectrum))))
                    enhanced_frames.append(enhanced_frame)

                # 合并帧
                enhanced_audio = np.concatenate(enhanced_frames)
                return enhanced_audio[:len(audio_array)]

        except Exception as e:
            logger.warning(f"噪声抑制处理失败: {e}")

        return audio_array

    def _convert_to_wav(self, audio_array: np.ndarray) -> bytes:
        """
        将音频数组转换为WAV格式

        Args:
            audio_array: 音频数组

        Returns:
            bytes: WAV格式音频数据
        """
        try:
            import wave
            import io

            # 转换为16位整数
            audio_int16 = (audio_array * 32767).astype(np.int16)

            # 创建WAV文件
            wav_buffer = io.BytesIO()
            with wave.open(wav_buffer, 'wb') as wav_file:
                wav_file.setnchannels(1)
                wav_file.setsampwidth(2)
                wav_file.setframerate(self.sample_rate)
                wav_file.writeframes(audio_int16.tobytes())

            return wav_buffer.getvalue()

        except Exception as e:
            logger.error(f"WAV转换失败: {e}")
            return b""

    def get_processing_stats(self) -> Dict[str, Any]:
        """
        获取处理统计信息

        Returns:
            Dict: 统计信息
        """
        return {
            "sample_rate": self.sample_rate,
            "frame_size": self.frame_size,
            "hop_size": self.hop_size,
            "vad_threshold": self.cantonese_vad_threshold,
            "min_segment_duration": self.min_segment_duration,
            "max_segment_duration": self.max_segment_duration,
            "supported_noise_types": [t.value for t in NoiseType if t != NoiseType.UNKNOWN],
            "supported_qualities": [q.value for q in AudioQuality]
        }


# 便捷函数
def create_audio_preprocessor_enhanced(sample_rate: int = 16000) -> AudioPreprocessorEnhanced:
    """
    创建音频预处理器增强版实例

    Args:
        sample_rate: 采样率

    Returns:
        AudioPreprocessorEnhanced: 预处理器实例
    """
    return AudioPreprocessorEnhanced(sample_rate)


if __name__ == "__main__":
    # 测试代码
    print("=== Audio Preprocessor Enhanced 测试 ===")

    preprocessor = create_audio_preprocessor_enhanced()

    # 显示配置
    print("\n预处理器配置:")
    stats = preprocessor.get_processing_stats()
    for key, value in stats.items():
        print(f"  {key}: {value}")

    # 创建测试音频
    print("\n创建测试音频...")
    duration = 8.0  # 8秒，测试分段功能
    sample_rate = 16000
    frequency = 440
    t = np.linspace(0, duration, int(sample_rate * duration))
    test_audio = (np.sin(2 * np.pi * frequency * t) * 16383).astype(np.int16)

    # 转换为WAV
    import wave
    import io
    wav_buffer = io.BytesIO()
    with wave.open(wav_buffer, 'wb') as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(test_audio.tobytes())

    wav_data = wav_buffer.getvalue()

    print(f"测试音频创建完成，大小: {len(wav_data)} 字节")

    # 测试预处理
    print("\n测试音频预处理...")
    result = preprocessor.analyze_and_preprocess(wav_data)

    print(f"预处理结果: 成功={result.success}")
    if result.success:
        print(f"  噪声类型: {result.analysis.noise_type.value}")
        print(f"  音频质量: {result.analysis.quality.value}")
        print(f"  信噪比: {result.analysis.snr_db:.1f}dB")
        print(f"  时长: {result.analysis.duration:.2f}秒")
        print(f"  分段数: {result.segments_count}")
        print(f"  处理建议: {result.analysis.recommendation}")
        print(f"  处理时间: {result.processing_time:.3f}s")
    else:
        print(f"  错误: {result.error}")

    print("\n测试完成")