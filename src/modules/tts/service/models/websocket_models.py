"""
WebSocket接口数据模型

定义WebSocket流式TTS接口的数据模型。
"""

from typing import Optional, Dict, Any
from pydantic import BaseModel, Field
from enum import Enum


class WebSocketMessageType(str, Enum):
    """WebSocket消息类型枚举"""
    REQUEST = "request"
    RESPONSE = "response"
    ERROR = "error"
    HEARTBEAT = "heartbeat"


class WebSocketMessage(BaseModel):
    """WebSocket消息模型"""
    type: WebSocketMessageType = Field(
        ...,
        description="消息类型",
        example=WebSocketMessageType.REQUEST
    )
    data: Dict[str, Any] = Field(
        ...,
        description="消息数据",
        example={
            "text": "你好",
            "voice_id": "default"
        }
    )
    request_id: Optional[str] = Field(
        None,
        description="请求ID",
        example="req-1234567890"
    )


class WebSocketError(BaseModel):
    """WebSocket错误模型"""
    code: str = Field(
        ...,
        description="错误代码",
        example="WS_001"
    )
    message: str = Field(
        ...,
        description="错误消息",
        example="Invalid request format"
    )
    details: Optional[Dict[str, Any]] = Field(
        None,
        description="错误详情"
    )


class WebSocketConnectionInfo(BaseModel):
    """WebSocket连接信息模型"""
    connection_id: str = Field(
        ...,
        description="连接ID",
        example="conn-1234567890"
    )
    client_id: Optional[str] = Field(
        None,
        description="客户端ID",
        example="client-001"
    )
    connected_at: str = Field(
        ...,
        description="连接时间",
        example="2025-11-04T10:30:00Z"
    )
    status: str = Field(
        ...,
        description="连接状态",
        example="active"
    )
