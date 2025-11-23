"""
语音合成引擎模块
================

支持多种TTS引擎：
- AliyunTTSClient: 阿里云在线TTS（推荐）
- TTSEngine: 本地VITS引擎（可选）
"""

# 延迟导入阿里云TTS客户端
def get_aliyun_tts_client():
    """延迟导入阿里云TTS客户端"""
    from .aliyun_tts_client import AliyunTTSClient
    return AliyunTTSClient

# 延迟导入本地VITS引擎
def get_tts_engine():
    """延迟导入本地VITS引擎"""
    from .tts_engine import TTSEngine
    return TTSEngine

__all__ = ['get_aliyun_tts_client', 'get_tts_engine']
