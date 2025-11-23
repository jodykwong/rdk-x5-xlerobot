#!/usr/bin/env python3
"""
Audio Converter - Pure Online Architecture
==========================================

⚠️ 严禁Mock数据声明：
- 仅处理真实音频设备录制的数据
- 禁止任何模拟或硬编码音频数据
- 确保转换的数据来源真实麦克风

音频格式转换器，专为纯在线语音服务设计。
提供PCM到WAV和Base64的简单格式转换，确保阿里云API兼容。

功能：
- PCM数据转换为WAV格式
- WAV格式转换为base64编码
- 阿里云API兼容性保证

作者: Developer Agent
版本: 1.0 (纯在线架构)
日期: 2025-11-09
"""

import numpy as np
import base64
import struct
import io
from typing import Union, Tuple
import logging

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class AudioConverter:
    """
    音频格式转换器

    专为纯在线服务设计，提供PCM↔WAV↔Base64转换功能。
    严格遵循简化原则，避免任何复杂的音频处理。
    """

    def __init__(self):
        """初始化音频转换器"""
        self.sample_rate = 16000
        self.channels = 1
        self.sample_width = 2  # 16-bit = 2 bytes

        logger.info("AudioConverter初始化完成")
        logger.info(f"音频参数: {self.sample_rate}Hz, {self.channels}通道, {self.sample_width * 8}bit")

    def pcm_to_wav(self, pcm_data: np.ndarray,
                   sample_rate: int = 16000,
                   channels: int = 1,
                   sample_width: int = 2) -> bytes:
        """
        将PCM数据转换为WAV格式

        Args:
            pcm_data: PCM音频数据 (numpy数组，int16类型)
            sample_rate: 采样率
            channels: 声道数
            sample_width: 采样宽度（字节数）

        Returns:
            bytes: WAV格式音频数据
        """
        try:
            # 验证输入数据
            if not isinstance(pcm_data, np.ndarray):
                raise ValueError("输入数据必须是numpy数组")

            if pcm_data.dtype != np.int16:
                logger.warning("输入数据不是int16类型，尝试转换")
                pcm_data = pcm_data.astype(np.int16)

            if len(pcm_data) == 0:
                raise ValueError("输入数据为空")

            # 创建WAV文件头
            wav_header = self._create_wav_header(
                len(pcm_data), sample_rate, channels, sample_width
            )

            # 转换PCM数据为字节数据
            pcm_bytes = pcm_data.astype(np.int16).tobytes()

            # 组合WAV头和PCM数据
            wav_data = wav_header + pcm_bytes

            logger.info(f"PCM到WAV转换完成: {len(pcm_data)}样本 → {len(wav_data)}字节")
            return wav_data

        except Exception as e:
            logger.error(f"PCM到WAV转换失败: {e}")
            raise

    def wav_to_base64(self, wav_data: bytes) -> str:
        """
        将WAV格式数据转换为Base64编码

        Args:
            wav_data: WAV格式音频数据

        Returns:
            str: Base64编码的音频数据
        """
        try:
            if not wav_data:
                raise ValueError("WAV数据为空")

            # Base64编码
            base64_data = base64.b64encode(wav_data).decode('utf-8')

            logger.info(f"WAV到Base64转换完成: {len(wav_data)}字节 → {len(base64_data)}字符")
            return base64_data

        except Exception as e:
            logger.error(f"WAV到Base64转换失败: {e}")
            raise

    def pcm_to_base64(self, pcm_data: np.ndarray,
                      sample_rate: int = 16000,
                      channels: int = 1,
                      sample_width: int = 2) -> str:
        """
        直接将PCM数据转换为Base64编码（WAV中间格式）

        Args:
            pcm_data: PCM音频数据
            sample_rate: 采样率
            channels: 声道数
            sample_width: 采样宽度

        Returns:
            str: Base64编码的音频数据
        """
        try:
            # PCM → WAV
            wav_data = self.pcm_to_wav(pcm_data, sample_rate, channels, sample_width)

            # WAV → Base64
            base64_data = self.wav_to_base64(wav_data)

            logger.info(f"PCM到Base64直接转换完成: {len(pcm_data)}样本 → {len(base64_data)}字符")
            return base64_data

        except Exception as e:
            logger.error(f"PCM到Base64转换失败: {e}")
            raise

    def base64_to_wav(self, base64_data: str) -> bytes:
        """
        将Base64编码转换为WAV格式数据

        Args:
            base64_data: Base64编码的音频数据

        Returns:
            bytes: WAV格式音频数据
        """
        try:
            if not base64_data:
                raise ValueError("Base64数据为空")

            # Base64解码
            wav_data = base64.b64decode(base64_data)

            logger.info(f"Base64到WAV转换完成: {len(base64_data)}字符 → {len(wav_data)}字节")
            return wav_data

        except Exception as e:
            logger.error(f"Base64到WAV转换失败: {e}")
            raise

    def get_wav_info(self, wav_data: bytes) -> dict:
        """
        获取WAV文件信息

        Args:
            wav_data: WAV格式音频数据

        Returns:
            dict: WAV文件信息
        """
        try:
            if len(wav_data) < 44:
                raise ValueError("WAV数据太短，不是有效的WAV文件")

            # 解析WAV头
            if wav_data[:4] != b'RIFF' or wav_data[8:12] != b'WAVE':
                raise ValueError("不是有效的WAV文件格式")

            # 读取格式块信息
            sample_rate = struct.unpack('<I', wav_data[24:28])[0]
            channels = struct.unpack('<H', wav_data[22:24])[0]
            byte_rate = struct.unpack('<I', wav_data[28:32])[0]
            block_align = struct.unpack('<H', wav_data[32:34])[0]
            bits_per_sample = struct.unpack('<H', wav_data[34:36])[0]

            # 计算样本数
            data_size = struct.unpack('<I', wav_data[40:44])[0]
            sample_count = data_size // (bits_per_sample // 8)

            info = {
                'sample_rate': sample_rate,
                'channels': channels,
                'bits_per_sample': bits_per_sample,
                'bytes_per_sample': bits_per_sample // 8,
                'sample_count': sample_count,
                'duration': sample_count / sample_rate,
                'byte_rate': byte_rate,
                'block_align': block_align,
                'data_size': data_size,
                'file_size': len(wav_data),
                'format': 'WAV'
            }

            logger.info(f"WAV信息解析完成: {info}")
            return info

        except Exception as e:
            logger.error(f"WAV信息解析失败: {e}")
            raise

    def _create_wav_header(self, sample_count: int,
                          sample_rate: int,
                          channels: int,
                          sample_width: int) -> bytes:
        """
        创建WAV文件头

        Args:
            sample_count: 样本数量
            sample_rate: 采样率
            channels: 声道数
            sample_width: 采样宽度（字节数）

        Returns:
            bytes: WAV文件头
        """
        # 计算参数
        byte_rate = sample_rate * channels * sample_width
        block_align = channels * sample_width
        bits_per_sample = sample_width * 8
        data_size = sample_count * sample_width
        file_size = 36 + data_size

        # WAV文件头格式
        header = struct.pack('<4sL4s', b'RIFF', file_size, b'WAVE')
        fmt_chunk = struct.pack('<4sLHHLLHH',
                               b'fmt ', 16, 1, channels, sample_rate,
                               byte_rate, block_align, bits_per_sample)
        data_chunk = struct.pack('<4sL', b'data', data_size)

        return header + fmt_chunk + data_chunk

    def validate_pcm_data(self, pcm_data: np.ndarray) -> Tuple[bool, str]:
        """
        验证PCM数据有效性

        Args:
            pcm_data: PCM音频数据

        Returns:
            Tuple[bool, str]: (是否有效, 错误信息)
        """
        try:
            if not isinstance(pcm_data, np.ndarray):
                return False, "输入数据不是numpy数组"

            if len(pcm_data) == 0:
                return False, "输入数据为空"

            if pcm_data.dtype != np.int16:
                return False, f"数据类型错误，期望int16，实际{pcm_data.dtype}"

            # 检查数据范围
            min_val = np.min(pcm_data)
            max_val = np.max(pcm_data)
            if min_val < -32768 or max_val > 32767:
                return False, f"数据值超出int16范围: [{min_val}, {max_val}]"

            # 检查是否有异常值
            if np.any(np.isnan(pcm_data)):
                return False, "数据包含NaN值"

            if np.any(np.isinf(pcm_data)):
                return False, "数据包含无穷大值"

            return True, "数据有效"

        except Exception as e:
            return False, f"验证过程中出现异常: {e}"

    def test_conversion(self) -> bool:
        """
        测试音频转换功能

        Returns:
            bool: 测试通过状态
        """
        logger.info("开始音频转换功能测试...")

        try:
            # 创建测试PCM数据
            test_samples = 8000  # 0.5秒 @ 16kHz
            frequency = 440  # A4音符
            t = np.linspace(0, 0.5, test_samples)
            test_pcm = (np.sin(2 * np.pi * frequency * t) * 16383).astype(np.int16)

            logger.info(f"创建测试PCM数据: {len(test_pcm)}样本")

            # 验证PCM数据
            is_valid, msg = self.validate_pcm_data(test_pcm)
            if not is_valid:
                logger.error(f"PCM数据验证失败: {msg}")
                return False

            # 测试PCM → WAV转换
            wav_data = self.pcm_to_wav(test_pcm)
            if len(wav_data) < 44:
                logger.error("PCM到WAV转换失败：数据太短")
                return False

            # 测试WAV信息解析
            wav_info = self.get_wav_info(wav_data)
            if wav_info['sample_rate'] != 16000 or wav_info['channels'] != 1:
                logger.error("WAV信息解析失败")
                return False

            # 测试WAV → Base64转换
            base64_data = self.wav_to_base64(wav_data)
            if not base64_data:
                logger.error("WAV到Base64转换失败")
                return False

            # 测试Base64 → WAV转换
            wav_data_2 = self.base64_to_wav(base64_data)
            if wav_data_2 != wav_data:
                logger.error("Base64到WAV转换失败：数据不一致")
                return False

            # 测试PCM → Base64直接转换
            base64_data_2 = self.pcm_to_base64(test_pcm)
            if not base64_data_2:
                logger.error("PCM到Base64直接转换失败")
                return False

            logger.info("音频转换功能测试通过")
            return True

        except Exception as e:
            logger.error(f"音频转换测试异常: {e}")
            return False


# 便捷函数
def create_audio_converter() -> AudioConverter:
    """
    创建音频转换器实例

    Returns:
        AudioConverter: 转换器实例
    """
    return AudioConverter()


if __name__ == "__main__":
    # 测试代码
    print("=== Audio Converter 测试 ===")

    converter = create_audio_converter()

    # 显示配置
    print(f"转换器配置: {converter.sample_rate}Hz, {converter.channels}通道, {converter.sample_width * 8}bit")

    # 运行测试
    print("\n运行转换测试...")
    test_result = converter.test_conversion()
    print(f"测试结果: {'通过' if test_result else '失败'}")

    print("\n测试完成")