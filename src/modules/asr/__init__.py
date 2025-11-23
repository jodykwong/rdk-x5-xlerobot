#!/usr/bin/env python3
"""
ASR语音识别模块

基于阿里云WebSocket API的实时粤语语音识别功能

作者: BMad Master
日期: 2025-11-13
版本: 2.0.0 - WebSocket API版本
"""

__version__ = "2.0.0"
__author__ = "BMad Master"
__email__ = "master@xlerobot.com"

from .aliyun_websocket_asr_client import AliyunWebSocketASRClient
from .config import ASRConfig

__all__ = [
    "AliyunWebSocketASRClient",
    "ASRConfig"
]
