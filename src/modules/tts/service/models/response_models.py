"""
TTS API响应数据模型

定义TTS API接口的响应参数模型。
"""

from typing import Optional, List, Dict, Any
from pydantic import BaseModel, Field
from datetime import datetime
from .request_models import AudioFormat, AudioQuality


class VoiceInfo(BaseModel):
    """音色信息模型"""
    voice_id: str = Field(
        ...,
        description="音色ID",
        example="default"
    )
    name: str = Field(
        ...,
        description="音色名称",
        example="默认音色"
    )
    language: str = Field(
        ...,
        description="支持语言",
        example="zh-CN"
    )
    gender: str = Field(
        ...,
        description="性别",
        example="female"
    )
    description: Optional[str] = Field(
        None,
        description="音色描述",
        example="温暖甜美的女声"
    )
    emotion_support: bool = Field(
        ...,
        description="是否支持情感",
        example=True
    )
    sample_url: Optional[str] = Field(
        None,
        description="示例音频URL",
        example="https://example.com/sample.mp3"
    )
    quality: AudioQuality = Field(
        ...,
        description="音质等级",
        example=AudioQuality.MEDIUM
    )


class TTSResponse(BaseModel):
    """TTS合成响应模型"""
    success: bool = Field(
        ...,
        description="请求是否成功",
        example=True
    )
    audio_data: Optional[str] = Field(
        None,
        description="Base64编码的音频数据",
        example="UklGRnoGAABXQVZFZm10IBAAAAABAAEAQB8AAEAfAAABAAgAZGF0YQoGAACBhYqFbF1fdJivrJBhNjVgodDbq2EcBj+a2/LDciUFLIHO8tiJNwgZaLvt559NEAxQp+PwtmMcBjiR1/LMeSwFJHfH8N2QQAoUXrTp66hVFApGn+DyvmUdBzaJ0fPYeTcFJHfH8N2QQAoUXrTp66hVFApGn+DyvmUdBzaJ0fPYeTcF..."
    )
    audio_format: AudioFormat = Field(
        ...,
        description="音频格式",
        example=AudioFormat.WAV
    )
    audio_size: Optional[int] = Field(
        None,
        description="音频数据大小（字节）",
        example=102400
    )
    duration: Optional[float] = Field(
        None,
        description="音频时长（秒）",
        example=3.5
    )
    sample_rate: int = Field(
        ...,
        description="采样率",
        example=22050
    )
    request_id: str = Field(
        ...,
        description="请求ID",
        example="req-1234567890"
    )
    processing_time: float = Field(
        ...,
        description="处理时间（秒）",
        example=0.125
    )
    cache_hit: Optional[bool] = Field(
        False,
        description="是否命中缓存",
        example=True
    )
    message: Optional[str] = Field(
        None,
        description="响应消息",
        example="音频合成成功"
    )


class TTSStreamResponse(BaseModel):
    """流式TTS响应模型"""
    chunk_id: int = Field(
        ...,
        description="分块ID",
        example=1
    )
    audio_data: str = Field(
        ...,
        description="Base64编码的音频数据块",
        example="UklGRnoGAABXQVZFZm10IBAAAAABAAEAQB8AAEAfAAABAAgAZGF0YQoGAACBhYqFbF1fdJivrJBhNjVgodDbq2EcBj+a2/LDciUFLIHO8tiJNwgZaLvt559NEAxQp+PwtmMcBjiR1/LMeSwFJHfH8N2QQAoUXrTp66hVFApGn+DyvmUdBzaJ0fPYeTcFJHfH8N2QQAoUXrTp66hVFApGn+DyvmUdBzaJ0fPYeTcF..."
    )
    chunk_size: int = Field(
        ...,
        description="分块大小",
        example=1024
    )
    total_chunks: Optional[int] = Field(
        None,
        description="总块数",
        example=10
    )
    is_final: bool = Field(
        ...,
        description="是否为最后一个分块",
        example=False
    )
    request_id: str = Field(
        ...,
        description="请求ID",
        example="req-1234567890"
    )


class VoiceListResponse(BaseModel):
    """音色列表响应模型"""
    success: bool = Field(
        ...,
        description="请求是否成功",
        example=True
    )
    voices: List[VoiceInfo] = Field(
        ...,
        description="音色列表",
        example=[
            {
                "voice_id": "default",
                "name": "默认音色",
                "language": "zh-CN",
                "gender": "female",
                "emotion_support": True,
                "quality": "medium"
            }
        ]
    )
    total: int = Field(
        ...,
        description="音色总数",
        example=5
    )
    request_id: str = Field(
        ...,
        description="请求ID",
        example="req-1234567890"
    )


class HealthCheckResponse(BaseModel):
    """健康检查响应模型"""
    status: str = Field(
        ...,
        description="服务状态",
        example="healthy"
    )
    version: str = Field(
        ...,
        description="服务版本",
        example="1.0.0"
    )
    timestamp: datetime = Field(
        ...,
        description="检查时间",
        example="2025-11-04T10:30:00Z"
    )
    uptime: float = Field(
        ...,
        description="运行时间（秒）",
        example=3600.5
    )
    services: Optional[Dict[str, str]] = Field(
        None,
        description="依赖服务状态",
        example={
            "tts_engine": "healthy",
            "cache": "healthy"
        }
    )
    performance: Optional[Dict[str, Any]] = Field(
        None,
        description="性能指标",
        example={
            "avg_response_time": 0.125,
            "requests_per_second": 50.0,
            "cache_hit_rate": 0.85
        }
    )
    request_id: str = Field(
        ...,
        description="请求ID",
        example="req-1234567890"
    )


class ErrorResponse(BaseModel):
    """错误响应模型"""
    success: bool = Field(
        False,
        description="请求是否成功",
        example=False
    )
    error_code: str = Field(
        ...,
        description="错误代码",
        example="TTS_001"
    )
    error_message: str = Field(
        ...,
        description="错误消息",
        example="文本内容不能为空"
    )
    details: Optional[Dict[str, Any]] = Field(
        None,
        description="错误详情",
        example={
            "field": "text",
            "reason": "Text cannot be empty"
        }
    )
    request_id: Optional[str] = Field(
        None,
        description="请求ID",
        example="req-1234567890"
    )
