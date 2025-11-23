#!/usr/bin/env python3
"""
简化音频预处理模块 - 纯在线服务版本

仅保留基础音频预处理功能：
- 格式转换 (PCM, WAV, base64)
- 基础归一化
- 简单VAD检测

专为纯在线服务设计，不包含复杂本地处理

作者: Dev Agent
日期: 2025-11-09
迭代: 1 - 纯在线服务
"""

import numpy as np
import logging
import base64
import wave
import io
from typing import Tuple, Optional, Union

logger = logging.getLogger(__name__)


class SimpleAudioPreprocessor:
    """
    简化音频预处理器
    
    专为纯在线服务设计，仅保留必要功能
    """
    
    def __init__(self, sample_rate: int = 16000):
        """
        初始化预处理器
        
        Args:
            sample_rate: 目标采样率，默认16kHz
        """
        self.sample_rate = sample_rate
        self.logger = logging.getLogger(__name__)
    
    def normalize_audio(self, audio_data: np.ndarray) -> np.ndarray:
        """
        音频归一化
        
        Args:
            audio_data: 原始音频数据
            
        Returns:
            归一化后的音频数据
        """
        if len(audio_data) == 0:
            return audio_data
            
        max_val = np.max(np.abs(audio_data))
        if max_val > 0:
            return audio_data / max_val
        return audio_data
    
    def simple_vad(self, audio_data: np.ndarray, threshold: float = 0.01) -> bool:
        """
        简单语音活动检测
        
        Args:
            audio_data: 音频数据
            threshold: 能量阈值
            
        Returns:
            是否包含语音
        """
        if len(audio_data) == 0:
            return False
            
        energy = np.mean(audio_data ** 2)
        return energy > threshold
    
    def pcm_to_wav_bytes(self, pcm_data: np.ndarray, sample_rate: int = 16000) -> bytes:
        """
        PCM数据转换为WAV字节流
        
        Args:
            pcm_data: PCM音频数据
            sample_rate: 采样率
            
        Returns:
            WAV格式的字节流
        """
        # 确保数据是int16格式
        if pcm_data.dtype != np.int16:
            pcm_data = (pcm_data * 32767).astype(np.int16)
        
        # 创建内存中的WAV文件
        wav_buffer = io.BytesIO()
        with wave.open(wav_buffer, 'wb') as wav_file:
            wav_file.setnchannels(1)  # 单声道
            wav_file.setsampwidth(2)  # 16-bit
            wav_file.setframerate(sample_rate)
            wav_file.writeframes(pcm_data.tobytes())
        
        return wav_buffer.getvalue()
    
    def wav_to_base64(self, wav_bytes: bytes) -> str:
        """
        WAV字节数据转换为base64字符串
        
        Args:
            wav_bytes: WAV字节数据
            
        Returns:
            base64编码的字符串
        """
        return base64.b64encode(wav_bytes).decode('utf-8')
    
    def process_for_aliyun(self, audio_data: np.ndarray) -> Optional[str]:
        """
        为阿里云API处理音频数据
        
        Args:
            audio_data: 原始音频数据
            
        Returns:
            base64编码的音频数据，失败返回None
        """
        try:
            # 1. 归一化
            normalized = self.normalize_audio(audio_data)
            
            # 2. 简单VAD检测
            if not self.simple_vad(normalized):
                self.logger.warning("音频中未检测到语音活动")
                return None
            
            # 3. 转换为WAV格式
            wav_bytes = self.pcm_to_wav_bytes(normalized, self.sample_rate)
            
            # 4. 转换为base64
            base64_audio = self.wav_to_base64(wav_bytes)
            
            return base64_audio
            
        except Exception as e:
            self.logger.error(f"音频处理失败: {e}")
            return None
    
    def get_audio_info(self, audio_data: np.ndarray) -> dict:
        """
        获取音频基本信息
        
        Args:
            audio_data: 音频数据
            
        Returns:
            音频信息字典
        """
        return {
            "duration": len(audio_data) / self.sample_rate,
            "sample_rate": self.sample_rate,
            "channels": 1,
            "dtype": str(audio_data.dtype),
            "max_amplitude": float(np.max(np.abs(audio_data))),
            "has_voice": self.simple_vad(audio_data)
        }


# 向后兼容的别名
AudioPreprocessor = SimpleAudioPreprocessor