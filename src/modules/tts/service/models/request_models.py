"""
TTS API请求数据模型

定义TTS API接口的请求参数模型。
"""

from typing import Optional, List, Dict, Any
from pydantic import BaseModel, Field, validator
from enum import Enum


class AudioFormat(str, Enum):
    """音频格式枚举"""
    WAV = "wav"
    MP3 = "mp3"
    OGG = "ogg"
    FLAC = "flac"


class AudioQuality(str, Enum):
    """音频质量枚举"""
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    ULTRA = "ultra"


class TTSRequest(BaseModel):
    """TTS合成请求模型"""
    text: str = Field(
        ...,
        min_length=1,
        max_length=5000,
        description="要转换为语音的文本",
        example="你好，欢迎使用TTS服务"
    )
    voice_id: Optional[str] = Field(
        "default",
        description="音色ID",
        example="default"
    )
    language: Optional[str] = Field(
        "zh-CN",
        description="语言代码",
        example="zh-CN"
    )
    emotion: Optional[str] = Field(
        "neutral",
        description="情感类型",
        example="happy"
    )
    emotion_intensity: Optional[float] = Field(
        0.5,
        ge=0.0,
        le=1.0,
        description="情感强度 (0.0-1.0)",
        example=0.8
    )
    speed: Optional[float] = Field(
        1.0,
        ge=0.5,
        le=2.0,
        description="语速 (0.5-2.0)",
        example=1.2
    )
    pitch: Optional[float] = Field(
        1.0,
        ge=0.5,
        le=2.0,
        description="音调 (0.5-2.0)",
        example=1.0
    )
    volume: Optional[float] = Field(
        1.0,
        ge=0.0,
        le=2.0,
        description="音量 (0.0-2.0)",
        example=1.0
    )
    audio_format: Optional[AudioFormat] = Field(
        AudioFormat.WAV,
        description="音频格式",
        example=AudioFormat.WAV
    )
    audio_quality: Optional[AudioQuality] = Field(
        AudioQuality.MEDIUM,
        description="音频质量",
        example=AudioQuality.MEDIUM
    )
    sample_rate: Optional[int] = Field(
        22050,
        description="采样率",
        example=22050
    )
    cache_enabled: Optional[bool] = Field(
        True,
        description="是否启用缓存",
        example=True
    )

    @validator('text')
    def text_must_not_be_empty(cls, v):
        if not v or not v.strip():
            raise ValueError('文本内容不能为空')
        return v.strip()


class TTSStreamRequest(BaseModel):
    """流式TTS请求模型"""
    text: str = Field(
        ...,
        min_length=1,
        max_length=1000,
        description="要转换为语音的文本（流式）",
        example="你好"
    )
    voice_id: Optional[str] = Field(
        "default",
        description="音色ID"
    )
    language: Optional[str] = Field(
        "zh-CN",
        description="语言代码"
    )
    emotion: Optional[str] = Field(
        "neutral",
        description="情感类型"
    )
    emotion_intensity: Optional[float] = Field(
        0.5,
        ge=0.0,
        le=1.0,
        description="情感强度 (0.0-1.0)"
    )
    speed: Optional[float] = Field(
        1.0,
        ge=0.5,
        le=2.0,
        description="语速 (0.5-2.0)"
    )
    chunk_size: Optional[int] = Field(
        1024,
        ge=512,
        le=8192,
        description="音频块大小",
        example=1024
    )


class VoiceListRequest(BaseModel):
    """音色列表请求模型"""
    language: Optional[str] = Field(
        None,
        description="过滤语言",
        example="zh-CN"
    )
    gender: Optional[str] = Field(
        None,
        description="过滤性别",
        example="female"
    )
    emotion_support: Optional[bool] = Field(
        None,
        description="是否支持情感",
        example=True
    )


class HealthCheckRequest(BaseModel):
    """健康检查请求模型"""
    detailed: Optional[bool] = Field(
        False,
        description="是否返回详细信息",
        example=False
    )
