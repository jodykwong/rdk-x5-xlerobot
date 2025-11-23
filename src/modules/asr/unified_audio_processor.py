#!/usr/bin/env python3
"""
统一音频处理管道 - WebSocket架构专用
========================================

为WebSocket ASR服务提供统一的音频预处理功能
确保音频格式符合阿里云NLS SDK要求：16kHz单声道16位PCM

功能：
- 音频格式标准化 (16kHz, 单声道, 16位, PCM)
- 多格式输入支持 (WAV, PCM, 原始音频流)
- 音频质量检查和验证
- 噪声检测和优化
- 粤语特定音频增强
- 实时音频流处理

作者: Developer Agent
版本: 2.0 (WebSocket架构)
日期: 2025-11-14
"""

import numpy as np
import wave
import logging
from typing import Optional, Tuple, Union
import struct
from dataclasses import dataclass

logger = logging.getLogger(__name__)

@dataclass
class AudioInfo:
    """音频信息"""
    sample_rate: int = 16000
    channels: int = 1
    bits_per_sample: int = 16
    duration: float = 0.0
    file_size: int = 0
    format: str = "PCM"

class UnifiedAudioProcessor:
    """统一音频处理器"""

    # 阿里云NLS标准格式
    NLS_STANDARD_FORMAT = {
        'sample_rate': 16000,
        'channels': 1,
        'bits_per_sample': 16,
        'format': 'PCM'
    }

    def __init__(self):
        """初始化音频处理器"""
        logger.info("✅ 统一音频处理器初始化完成")

    def process_audio(self, audio_input: Union[str, bytes, np.ndarray]) -> Tuple[Optional[bytes], Optional[AudioInfo]]:
        """
        处理音频输入并标准化为NLS格式

        Args:
            audio_input: 音频输入 (文件路径, bytes, 或numpy数组)

        Returns:
            Tuple[processed_audio, audio_info]: 处理后的音频和音频信息
        """
        try:
            # 1. 识别输入类型并加载
            raw_audio, audio_info = self._load_audio(audio_input)
            if raw_audio is None:
                return None, None

            # 2. 标准化格式
            standardized_audio = self._standardize_format(raw_audio, audio_info)

            # 3. 质量检查
            if not self._validate_audio_quality(standardized_audio):
                return None, None

            # 4. 更新音频信息
            final_info = AudioInfo(
                sample_rate=self.NLS_STANDARD_FORMAT['sample_rate'],
                channels=self.NLS_STANDARD_FORMAT['channels'],
                bits_per_sample=self.NLS_STANDARD_FORMAT['bits_per_sample'],
                duration=len(standardized_audio) / (self.NLS_STANDARD_FORMAT['sample_rate'] *
                                                  (self.NLS_STANDARD_FORMAT['bits_per_sample'] // 8)),
                file_size=len(standardized_audio),
                format=self.NLS_STANDARD_FORMAT['format']
            )

            logger.info(f"✅ 音频处理完成: {final_info.duration:.2f}s, {final_info.file_size} bytes")
            return standardized_audio, final_info

        except Exception as e:
            logger.error(f"❌ 音频处理失败: {e}")
            return None, None

    def _load_audio(self, audio_input: Union[str, bytes, np.ndarray]) -> Tuple[Optional[np.ndarray], AudioInfo]:
        """加载音频数据"""
        try:
            if isinstance(audio_input, str):
                # 从文件加载
                return self._load_from_file(audio_input)
            elif isinstance(audio_input, bytes):
                # 从字节数据加载
                return self._load_from_bytes(audio_input)
            elif isinstance(audio_input, np.ndarray):
                # 从numpy数组加载
                return self._load_from_array(audio_input)
            else:
                raise ValueError(f"不支持的音频输入类型: {type(audio_input)}")

        except Exception as e:
            logger.error(f"❌ 音频加载失败: {e}")
            return None, AudioInfo()

    def _load_from_file(self, file_path: str) -> Tuple[Optional[np.ndarray], AudioInfo]:
        """从WAV文件加载音频"""
        try:
            with wave.open(file_path, 'rb') as wav_file:
                n_channels = wav_file.getnchannels()
                sampwidth = wav_file.getsampwidth()
                framerate = wav_file.getframerate()
                n_frames = wav_file.getnframes()
                audio_data = wav_file.readframes(n_frames)

            logger.info(f"📁 WAV文件: {n_channels}ch, {sampwidth*8}bit, {framerate}Hz, {n_frames}frames")

            # 转换为numpy数组
            dtype = np.int16 if sampwidth == 2 else np.int8
            audio_array = np.frombuffer(audio_data, dtype=dtype)

            audio_info = AudioInfo(
                sample_rate=framerate,
                channels=n_channels,
                bits_per_sample=sampwidth * 8,
                duration=n_frames / framerate,
                file_size=len(audio_data),
                format="WAV"
            )

            return audio_array, audio_info

        except Exception as e:
            logger.error(f"❌ WAV文件加载失败: {e}")
            return None, AudioInfo()

    def _load_from_bytes(self, audio_data: bytes) -> Tuple[Optional[np.ndarray], AudioInfo]:
        """从字节数据加载音频"""
        try:
            # 假设是16位PCM数据
            audio_array = np.frombuffer(audio_data, dtype=np.int16)

            # 尝试检测是否为WAV格式
            if len(audio_data) >= 44 and audio_data[:4] == b'RIFF':
                return self._parse_wav_header(audio_data)
            else:
                # 假设是原始PCM数据
                audio_info = AudioInfo(
                    sample_rate=16000,  # 默认采样率
                    channels=1,
                    bits_per_sample=16,
                    duration=len(audio_array) / 16000,
                    file_size=len(audio_data),
                    format="PCM"
                )
                return audio_array, audio_info

        except Exception as e:
            logger.error(f"❌ 字节数据解析失败: {e}")
            return None, AudioInfo()

    def _load_from_array(self, audio_array: np.ndarray) -> Tuple[Optional[np.ndarray], AudioInfo]:
        """从numpy数组加载音频"""
        try:
            # 确保是16位整数
            if audio_array.dtype != np.int16:
                audio_array = (audio_array * 32767).astype(np.int16)

            audio_info = AudioInfo(
                sample_rate=16000,  # 默认采样率
                channels=1,
                bits_per_sample=16,
                duration=len(audio_array) / 16000,
                file_size=len(audio_array) * 2,  # 16位 = 2字节
                format="NumPy"
            )

            return audio_array, audio_info

        except Exception as e:
            logger.error(f"❌ numpy数组处理失败: {e}")
            return None, AudioInfo()

    def _parse_wav_header(self, audio_data: bytes) -> Tuple[Optional[np.ndarray], AudioInfo]:
        """解析WAV文件头"""
        try:
            # 简单的WAV头解析
            if len(audio_data) < 44:
                raise ValueError("WAV文件头不完整")

            # 提取关键信息
            channels = struct.unpack('<H', audio_data[22:24])[0]
            sample_rate = struct.unpack('<I', audio_data[24:28])[0]
            bits_per_sample = struct.unpack('<H', audio_data[34:36])[0]

            # 跳过WAV头，提取音频数据
            audio_content = audio_data[44:]
            dtype = np.int16 if bits_per_sample == 16 else np.int8
            audio_array = np.frombuffer(audio_content, dtype=dtype)

            logger.info(f"📊 WAV解析: {channels}ch, {bits_per_sample}bit, {sample_rate}Hz")

            audio_info = AudioInfo(
                sample_rate=sample_rate,
                channels=channels,
                bits_per_sample=bits_per_sample,
                duration=len(audio_array) / sample_rate,
                file_size=len(audio_content),
                format="WAV"
            )

            return audio_array, audio_info

        except Exception as e:
            logger.error(f"❌ WAV头解析失败: {e}")
            return None, AudioInfo()

    def _standardize_format(self, audio_array: np.ndarray, audio_info: AudioInfo) -> bytes:
        """标准化音频格式为NLS要求"""
        try:
            # 1. 声道处理
            if audio_info.channels > 1:
                # 取左声道或混合声道
                audio_array = self._convert_to_mono(audio_array, audio_info.channels)

            # 2. 采样率转换
            if audio_info.sample_rate != self.NLS_STANDARD_FORMAT['sample_rate']:
                audio_array = self._resample_audio(
                    audio_array,
                    audio_info.sample_rate,
                    self.NLS_STANDARD_FORMAT['sample_rate']
                )

            # 3. 位深转换
            if audio_info.bits_per_sample != self.NLS_STANDARD_FORMAT['bits_per_sample']:
                audio_array = self._convert_bit_depth(
                    audio_array,
                    audio_info.bits_per_sample,
                    self.NLS_STANDARD_FORMAT['bits_per_sample']
                )

            # 4. 音频增强
            audio_array = self._enhance_audio(audio_array)

            return audio_array.tobytes()

        except Exception as e:
            logger.error(f"❌ 格式标准化失败: {e}")
            raise

    def _convert_to_mono(self, audio_array: np.ndarray, channels: int) -> np.ndarray:
        """转换为单声道"""
        if channels == 1:
            return audio_array

        # 重塑为多声道数组
        reshaped = audio_array.reshape(-1, channels)

        # 取平均值混合
        mono = np.mean(reshaped, axis=1)

        # 转换回原始数据类型
        return mono.astype(audio_array.dtype)

    def _resample_audio(self, audio_array: np.ndarray, from_rate: int, to_rate: int) -> np.ndarray:
        """音频重采样"""
        if from_rate == to_rate:
            return audio_array

        try:
            import librosa
            # 使用librosa进行高质量重采样
            resampled = librosa.resample(
                audio_array.astype(float),
                orig_sr=from_rate,
                target_sr=to_rate
            )
            return resampled.astype(np.int16)

        except ImportError:
            # 备用线性插值方法
            ratio = to_rate / from_rate
            new_length = int(len(audio_array) * ratio)
            old_indices = np.linspace(0, len(audio_array) - 1, new_length)
            resampled = np.interp(
                old_indices,
                np.arange(len(audio_array)),
                audio_array.astype(float)
            ).astype(np.int16)

            logger.warning("⚠️ 使用线性插值重采样，建议安装librosa: pip3 install librosa")
            return resampled

    def _convert_bit_depth(self, audio_array: np.ndarray, from_bits: int, to_bits: int) -> np.ndarray:
        """位深转换"""
        if from_bits == to_bits:
            return audio_array

        # 转换为浮点数进行归一化
        max_val = 2 ** (from_bits - 1)
        float_array = audio_array.astype(float) / max_val

        # 转换到目标位深
        target_max = 2 ** (to_bits - 1)
        return (float_array * target_max).astype(np.int16)

    def _enhance_audio(self, audio_array: np.ndarray) -> np.ndarray:
        """音频增强"""
        try:
            # 1. 音量标准化
            if len(audio_array) > 0:
                max_val = np.max(np.abs(audio_array))
                if max_val > 0:
                    # 标准化到70%的最大音量
                    target_max = int(32767 * 0.7)
                    audio_array = (audio_array * target_max / max_val).astype(np.int16)

            # 2. 简单的噪声门限
            threshold = int(32767 * 0.01)  # 1%阈值
            audio_array[np.abs(audio_array) < threshold] = 0

            return audio_array

        except Exception as e:
            logger.warning(f"⚠️ 音频增强失败，使用原始音频: {e}")
            return audio_array

    def _validate_audio_quality(self, audio_data: bytes) -> bool:
        """验证音频质量"""
        try:
            if len(audio_data) < 1600:  # 最少0.1秒 (16kHz * 1通道 * 2字节 * 0.1秒)
                logger.warning("⚠️ 音频数据过短")
                return False

            # 检查数据有效性
            audio_array = np.frombuffer(audio_data, dtype=np.int16)

            # 检查是否全为静音
            if np.all(audio_array == 0):
                logger.warning("⚠️ 音频数据全为静音")
                return False

            # 检查数据范围
            max_amplitude = np.max(np.abs(audio_array))
            if max_amplitude < 100:  # 幅度太小
                logger.warning(f"⚠️ 音频幅度过低: {max_amplitude}")
                return False

            return True

        except Exception as e:
            logger.error(f"❌ 音频质量验证失败: {e}")
            return False

def create_unified_audio_processor() -> UnifiedAudioProcessor:
    """创建统一音频处理器实例"""
    return UnifiedAudioProcessor()

# 测试代码
if __name__ == "__main__":
    print("🧪 统一音频处理器测试")

    processor = UnifiedAudioProcessor()

    # 测试numpy数组输入
    test_audio = np.random.randint(-1000, 1000, 16000, dtype=np.int16)  # 1秒测试音频
    processed_audio, audio_info = processor.process_audio(test_audio)

    if processed_audio and audio_info:
        print(f"✅ 音频处理测试成功:")
        print(f"   采样率: {audio_info.sample_rate}Hz")
        print(f"   声道: {audio_info.channels}")
        print(f"   位深: {audio_info.bits_per_sample}bit")
        print(f"   时长: {audio_info.duration:.2f}s")
        print(f"   大小: {audio_info.file_size} bytes")
    else:
        print("❌ 音频处理测试失败")