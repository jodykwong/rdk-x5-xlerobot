"""
TTS语音合成模块
===============

提供高质量粤语语音合成功能，支持阿里云在线TTS和本地VITS引擎。

主要组件:
- AliyunTTSSystem: 阿里云TTS系统（推荐）
- AliyunTTSClient: 阿里云TTS客户端
- AliyunConfigManager: 阿里云配置管理器
- TTSEngine: 本地VITS引擎（可选）

作者: Dev Agent
版本: 2.0.0 (阿里云TTS集成)
"""

# 延迟导入，避免torch依赖问题
def get_tts_engine():
    """延迟导入TTS引擎"""
    from .engine.tts_engine import TTSEngine
    return TTSEngine

def get_text_processor():
    """延迟导入文本处理器"""
    from .text.text_processor import TextProcessor
    return TextProcessor

def get_audio_processor():
    """延迟导入音频处理器"""
    from .audio.audio_processor import AudioProcessor
    return AudioProcessor

def get_tts_config():
    """延迟导入TTS配置"""
    from .config.tts_config import TTSConfig
    return TTSConfig

# 阿里云TTS组件（主要接口）
from .aliyun_tts_system import AliyunTTSSystem, create_aliyun_tts_system, synthesize_text
from .engine.aliyun_tts_client import AliyunTTSClient
from .config.aliyun_config_manager import AliyunConfigManager

__all__ = [
    # 阿里云TTS组件
    'AliyunTTSSystem',
    'AliyunTTSClient',
    'AliyunConfigManager',
    'create_aliyun_tts_system',
    'synthesize_text',

    # 延迟导入组件
    'get_tts_engine',
    'get_text_processor',
    'get_audio_processor',
    'get_tts_config',
]

__version__ = '2.0.0'
