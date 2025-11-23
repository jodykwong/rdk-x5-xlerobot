#!/usr/bin/env python3
"""
ASR模块API接口

提供Python API接口，便于模块集成

作者: Dev Agent
日期: 2025-11-02
Epic: 1 - ASR语音识别模块
Story: 1.1 - 粤语语音识别基础功能
"""

import numpy as np
import logging
from typing import Optional, Callable, Dict, Any
from .asr_core import ASREngine, ASRResult

logger = logging.getLogger(__name__)


class ASRAPI:
    """ASR API接口类"""
    
    def __init__(self, config_path: Optional[str] = None):
        """
        初始化API
        
        Args:
            config_path: 配置文件路径
        """
        self.engine = ASREngine(config_path)
        self.callback: Optional[Callable] = None
        logger.info("ASR API 初始化完成")
        
    def recognize(self, audio_data: bytes, sample_rate: int = 16000, 
                  format: str = "PCM") -> ASRResult:
        """
        同步语音识别接口
        
        Args:
            audio_data: 音频数据（字节）
            sample_rate: 采样率
            format: 音频格式
            
        Returns:
            ASRResult: 识别结果
        """
        try:
            # 将字节转换为numpy数组
            audio = self._bytes_to_numpy(audio_data, format)
            
            # 调用引擎识别
            result = self.engine.recognize(audio, sample_rate)
            
            logger.info(f"识别成功: '{result.text}'")
            return result
            
        except Exception as e:
            logger.error(f"识别失败: {e}")
            raise
            
    def start_continuous_recognition(self, callback: Optional[Callable] = None) -> None:
        """
        启动连续语音识别
        
        Args:
            callback: 结果回调函数
        """
        self.callback = callback
        logger.info("连续语音识别已启动")
        
        # TODO: 实现连续识别逻辑
        # - 音频流接收
        # - 实时处理
        # - 回调通知
        
    def stop_continuous_recognition(self) -> None:
        """停止连续语音识别"""
        self.callback = None
        logger.info("连续语音识别已停止")
        
    def set_language_mode(self, mode: str) -> None:
        """
        设置语言模式
        
        Args:
            mode: 语言模式 (cantonese, mandarin, english)
        """
        self.engine.config["language"] = mode
        logger.info(f"语言模式已设置为: {mode}")
        
    def _bytes_to_numpy(self, audio_data: bytes, format: str) -> np.ndarray:
        """将字节转换为numpy数组"""
        # TODO: 支持多种音频格式
        # - PCM
        # - WAV
        # - MP3
        
        if format.upper() == "PCM":
            # 假设16位单声道PCM
            audio_array = np.frombuffer(audio_data, dtype=np.int16)
            # 转换为float32并归一化
            audio_array = audio_array.astype(np.float32) / 32768.0
            return audio_array
        else:
            raise ValueError(f"不支持的音频格式: {format}")
            
    def get_status(self) -> Dict[str, Any]:
        """获取API状态"""
        return {
            "engine_status": self.engine.get_status(),
            "continuous_mode": self.callback is not None,
            "language": self.engine.config.get("language", "unknown")
        }
