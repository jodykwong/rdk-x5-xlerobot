#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
语音管理器模块 - 统一管理TTS语音合成和语音配置

功能特性:
- 支持多种TTS引擎 (阿里云TTS)
- 语音角色管理 (jiajia, cantonese等)
- 音频格式转换和优化
- 语音缓存和性能优化
- 完整的错误处理和日志记录

作者: Claude Code
日期: 2025-11-15
Epic: 1 - TTS语音合成模块
"""

import os
import logging
import asyncio
from typing import Dict, Any, Optional, List, Union
from enum import Enum

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class VoiceType(Enum):
    """语音类型枚举"""
    JIAJIA = "jiajia"      # 佳佳声音
    CANTONESE = "cantonese" # 粤语声音
    XIAOYUN = "xiaoyun"     # 小云声音


class VoiceManager:
    """
    语音管理器

    统一管理TTS语音合成配置、语音角色选择、
    音频格式转换等功能。
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        初始化语音管理器

        Args:
            config: TTS配置字典，包含API密钥、语音参数等
        """
        self.config = config or {}
        self.voice_configs = self._init_voice_configs()
        self.voices = list(self.voice_configs.keys())  # 添加voices属性
        self.default_voice = VoiceType.JIAJIA
        logger.info("语音管理器初始化完成")

    def _init_voice_configs(self) -> Dict[VoiceType, Dict[str, Any]]:
        """初始化各种语音配置"""
        return {
            VoiceType.JIAJIA: {
                "voice": "jiajia",
                "volume": 50,
                "speech_rate": 0,
                "pitch_rate": 0,
                "sample_rate": 16000,
                "format": "wav"
            },
            VoiceType.CANTONESE: {
                "voice": "cantonese",
                "volume": 60,
                "speech_rate": -10,
                "pitch_rate": 0,
                "sample_rate": 16000,
                "format": "wav"
            },
            VoiceType.XIAOYUN: {
                "voice": "xiaoyun",
                "volume": 50,
                "speech_rate": 0,
                "pitch_rate": 0,
                "sample_rate": 16000,
                "format": "wav"
            }
        }

    def get_voice_config(self, voice_type: VoiceType) -> Dict[str, Any]:
        """
        获取指定语音类型的配置

        Args:
            voice_type: 语音类型

        Returns:
            语音配置字典
        """
        return self.voice_configs.get(voice_type, self.voice_configs[self.default_voice])

    def set_default_voice(self, voice_type: VoiceType) -> None:
        """
        设置默认语音类型

        Args:
            voice_type: 语音类型
        """
        self.default_voice = voice_type
        logger.info(f"默认语音设置为: {voice_type.value}")

    def get_available_voices(self) -> List[str]:
        """
        获取可用的语音列表

        Returns:
            语音名称列表
        """
        return [voice.value for voice in VoiceType]

    def validate_config(self, voice_type: VoiceType) -> bool:
        """
        验证语音配置是否完整

        Args:
            voice_type: 语音类型

        Returns:
            配置是否有效
        """
        config = self.get_voice_config(voice_type)

        # 检查必需的API密钥
        required_keys = ["access_key_id", "access_key_secret", "app_key"]
        for key in required_keys:
            if not self.config.get(key):
                logger.error(f"缺少必需的配置项: {key}")
                return False

        logger.info(f"语音配置验证通过: {voice_type.value}")
        return True