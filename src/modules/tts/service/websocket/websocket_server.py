"""
TTS WebSocket服务器

基于WebSocket实现的流式TTS服务接口。
"""

import base64
import json
import uuid
import time
import asyncio
from typing import Dict, Set, Optional
from fastapi import WebSocket, WebSocketDisconnect
import logging

from ..models.request_models import TTSStreamRequest
from ..models.response_models import TTSStreamResponse
from ..models.websocket_models import WebSocketMessage, WebSocketMessageType, WebSocketError
from ..utils.performance_monitor import PerformanceMonitor

logger = logging.getLogger(__name__)


class ConnectionManager:
    """WebSocket连接管理器"""

    def __init__(self):
        self.active_connections: Dict[str, WebSocket] = {}
        self.connection_info: Dict[str, Dict] = {}

    async def connect(self, websocket: WebSocket, connection_id: str, client_id: Optional[str] = None):
        """建立连接"""
        await websocket.accept()
        self.active_connections[connection_id] = websocket
        self.connection_info[connection_id] = {
            'client_id': client_id,
            'connected_at': time.time(),
            'status': 'active'
        }
        logger.info(f"WebSocket连接建立: {connection_id}")

    def disconnect(self, connection_id: str):
        """断开连接"""
        if connection_id in self.active_connections:
            del self.active_connections[connection_id]
        if connection_id in self.connection_info:
            del self.connection_info[connection_id]
        logger.info(f"WebSocket连接断开: {connection_id}")

    async def send_message(self, connection_id: str, message: dict):
        """发送消息"""
        websocket = self.active_connections.get(connection_id)
        if websocket:
            try:
                await websocket.send_json(message)
            except Exception as e:
                logger.error(f"发送消息失败: {e}")
                self.disconnect(connection_id)

    async def broadcast(self, message: dict):
        """广播消息"""
        for connection_id in list(self.active_connections.keys()):
            await self.send_message(connection_id, message)


class TTSWebSocketServer:
    """TTS WebSocket服务器"""

    def __init__(self, tts_system, voice_manager, cache_manager):
        """
        初始化TTS WebSocket服务器

        Args:
            tts_system: TTS系统实例
            voice_manager: 音色管理器实例
            cache_manager: 缓存管理器实例
        """
        self.tts_system = tts_system
        self.voice_manager = voice_manager
        self.cache_manager = cache_manager
        self.manager = ConnectionManager()
        self.performance_monitor = PerformanceMonitor()

    async def handle_connection(self, websocket: WebSocket, connection_id: str):
        """
        处理WebSocket连接

        Args:
            websocket: WebSocket连接
            connection_id: 连接ID
        """
        await self.manager.connect(websocket, connection_id)

        try:
            while True:
                # 接收消息
                data = await websocket.receive_text()
                message_data = json.loads(data)

                # 处理消息
                await self._process_message(connection_id, message_data)

        except WebSocketDisconnect:
            self.manager.disconnect(connection_id)
            self.performance_monitor.record_connection(False)
        except Exception as e:
            logger.error(f"WebSocket错误: {e}")
            self.manager.disconnect(connection_id)
            self.performance_monitor.record_connection(False)

    async def _process_message(self, connection_id: str, message_data: dict):
        """
        处理接收到的消息

        Args:
            connection_id: 连接ID
            message_data: 消息数据
        """
        try:
            message_type = message_data.get('type')

            if message_type == WebSocketMessageType.REQUEST.value:
                await self._handle_tts_request(connection_id, message_data)
            elif message_type == WebSocketMessageType.HEARTBEAT.value:
                await self._handle_heartbeat(connection_id)
            else:
                await self._send_error(connection_id, "UNKNOWN_MESSAGE_TYPE", f"未知消息类型: {message_type}")

        except Exception as e:
            logger.error(f"处理消息失败: {e}")
            await self._send_error(connection_id, "PROCESSING_ERROR", str(e))

    async def _handle_tts_request(self, connection_id: str, message_data: dict):
        """
        处理TTS请求

        Args:
            connection_id: 连接ID
            message_data: 请求数据
        """
        start_time = time.time()

        try:
            # 解析请求数据
            data = message_data.get('data', {})
            request = TTSStreamRequest(
                text=data.get('text', ''),
                voice_id=data.get('voice_id', 'default'),
                language=data.get('language', 'zh-CN'),
                emotion=data.get('emotion', 'neutral'),
                emotion_intensity=data.get('emotion_intensity', 0.5),
                speed=data.get('speed', 1.0),
                chunk_size=data.get('chunk_size', 1024)
            )

            # 验证请求
            if not request.text.strip():
                await self._send_error(connection_id, "INVALID_REQUEST", "文本内容不能为空")
                return

            request_id = str(uuid.uuid4())

            # 检查缓存
            cache_result = self.cache_manager.get_audio(
                request.text, request.voice_id,
                language=request.language,
                emotion=request.emotion,
                emotion_intensity=request.emotion_intensity,
                speed=request.speed
            )

            if cache_result:
                audio_data, metadata = cache_result
                # 发送缓存的音频数据
                await self._send_audio_chunks(
                    connection_id, audio_data, request_id,
                    chunk_size=request.chunk_size,
                    is_final=True
                )

                # 记录性能
                processing_time = time.time() - start_time
                self.performance_monitor.record_request(processing_time, True, True)

                return

            # 流式合成
            chunk_id = 0
            async for audio_chunk in self.tts_system.synthesize_stream_async(
                text=request.text,
                voice_id=request.voice_id,
                language=request.language,
                emotion=request.emotion,
                emotion_intensity=request.emotion_intensity,
                speed=request.speed,
                chunk_size=request.chunk_size
            ):
                chunk_id += 1
                is_final = chunk_id == -1  # 最后一个块由TTS系统标记

                # 发送音频分块
                await self._send_audio_chunk(
                    connection_id, chunk_id, audio_chunk, request_id, is_final
                )

            # 记录性能
            processing_time = time.time() - start_time
            self.performance_monitor.record_request(processing_time, True, False)

        except Exception as e:
            logger.error(f"TTS请求处理失败: {e}")
            await self._send_error(connection_id, "TTS_ERROR", str(e))

    async def _send_audio_chunk(
        self,
        connection_id: str,
        chunk_id: int,
        audio_data: bytes,
        request_id: str,
        is_final: bool = False
    ):
        """
        发送音频分块

        Args:
            connection_id: 连接ID
            chunk_id: 分块ID
            audio_data: 音频数据
            request_id: 请求ID
            is_final: 是否为最后一个分块
        """
        audio_base64 = base64.b64encode(audio_data).decode('utf-8')

        response = {
            "type": "response",
            "chunk_id": chunk_id,
            "audio_data": audio_base64,
            "chunk_size": len(audio_data),
            "is_final": is_final,
            "request_id": request_id
        }

        await self.manager.send_message(connection_id, response)

    async def _send_audio_chunks(
        self,
        connection_id: str,
        audio_data: bytes,
        request_id: str,
        chunk_size: int = 1024,
        is_final: bool = True
    ):
        """
        发送音频分块（从完整音频数据分割）

        Args:
            connection_id: 连接ID
            audio_data: 音频数据
            request_id: 请求ID
            chunk_size: 分块大小
            is_final: 是否为最后一个分块
        """
        chunk_id = 0
        for i in range(0, len(audio_data), chunk_size):
            chunk = audio_data[i:i + chunk_size]
            chunk_id += 1
            await self._send_audio_chunk(
                connection_id, chunk_id, chunk, request_id,
                is_final=(i + chunk_size >= len(audio_data))
            )
            await asyncio.sleep(0.01)  # 小延迟防止过载

    async def _handle_heartbeat(self, connection_id: str):
        """
        处理心跳

        Args:
            connection_id: 连接ID
        """
        response = {
            "type": "heartbeat",
            "timestamp": time.time(),
            "status": "alive"
        }
        await self.manager.send_message(connection_id, response)

    async def _send_error(self, connection_id: str, code: str, message: str):
        """
        发送错误消息

        Args:
            connection_id: 连接ID
            code: 错误代码
            message: 错误消息
        """
        response = {
            "type": "error",
            "code": code,
            "message": message
        }
        await self.manager.send_message(connection_id, response)

    async def broadcast_message(self, message: dict):
        """广播消息"""
        await self.manager.broadcast(message)

    def get_connection_count(self) -> int:
        """获取活跃连接数"""
        return len(self.manager.active_connections)

    def get_performance_stats(self) -> dict:
        """获取性能统计"""
        return self.performance_monitor.get_stats()
