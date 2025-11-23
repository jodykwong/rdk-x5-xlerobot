"""
WebSocket接口测试

测试WebSocket流式TTS接口。
"""

import pytest
import json
from unittest.mock import Mock, AsyncMock, patch
import asyncio
import sys
import os

# 添加路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

from service.websocket.websocket_server import ConnectionManager, TTSWebSocketServer


class TestConnectionManager:
    """测试连接管理器"""

    @pytest.fixture
    def manager(self):
        """创建连接管理器"""
        return ConnectionManager()

    @pytest.mark.asyncio
    async def test_connect(self, manager):
        """测试连接建立"""
        mock_websocket = Mock()
        connection_id = "test-conn-1"

        await manager.connect(mock_websocket, connection_id)

        assert connection_id in manager.active_connections
        assert connection_id in manager.connection_info
        assert manager.connection_info[connection_id]['status'] == 'active'

    @pytest.mark.asyncio
    async def test_disconnect(self, manager):
        """测试连接断开"""
        mock_websocket = Mock()
        connection_id = "test-conn-1"

        # 先建立连接
        await manager.connect(mock_websocket, connection_id)
        assert connection_id in manager.active_connections

        # 断开连接
        manager.disconnect(connection_id)
        assert connection_id not in manager.active_connections
        assert connection_id not in manager.connection_info

    @pytest.mark.asyncio
    async def test_send_message(self, manager):
        """测试发送消息"""
        mock_websocket = Mock()
        mock_websocket.send_json = AsyncMock()
        connection_id = "test-conn-1"

        await manager.connect(mock_websocket, connection_id)

        message = {"type": "test", "data": "hello"}
        await manager.send_message(connection_id, message)

        mock_websocket.send_json.assert_called_once_with(message)

    @pytest.mark.asyncio
    async def test_broadcast(self, manager):
        """测试广播消息"""
        mock_websocket1 = Mock()
        mock_websocket1.send_json = AsyncMock()
        mock_websocket2 = Mock()
        mock_websocket2.send_json = AsyncMock()

        await manager.connect(mock_websocket1, "conn-1")
        await manager.connect(mock_websocket2, "conn-2")

        message = {"type": "broadcast", "data": "hello"}
        await manager.broadcast(message)

        mock_websocket1.send_json.assert_called_once_with(message)
        mock_websocket2.send_json.assert_called_once_with(message)


class TestTTSWebSocketServer:
    """测试TTS WebSocket服务器"""

    @pytest.fixture
    def mock_tts_system(self):
        """模拟TTS系统"""
        mock = Mock()
        mock.synthesize_stream_async = AsyncMock()
        return mock

    @pytest.fixture
    def mock_voice_manager(self):
        """模拟音色管理器"""
        return Mock()

    @pytest.fixture
    def mock_cache_manager(self):
        """模拟缓存管理器"""
        return Mock()

    @pytest.fixture
    def websocket_server(self, mock_tts_system, mock_voice_manager, mock_cache_manager):
        """创建WebSocket服务器"""
        return TTSWebSocketServer(mock_tts_system, mock_voice_manager, mock_cache_manager)

    @pytest.mark.asyncio
    async def test_process_tts_request(self, websocket_server):
        """测试处理TTS请求"""
        connection_id = "test-conn"

        message_data = {
            "type": "request",
            "data": {
                "text": "你好",
                "voice_id": "default",
                "emotion": "happy"
            }
        }

        # 模拟合成结果
        websocket_server.tts_system.synthesize_stream_async = AsyncMock()
        websocket_server.tts_system.synthesize_stream_async.return_value = [
            b"chunk1", b"chunk2", b"chunk3"
        ]

        # 模拟缓存返回None
        websocket_server.cache_manager.get_audio = Mock(return_value=None)

        # 模拟发送消息
        with patch.object(websocket_server.manager, 'send_message', new_callable=AsyncMock) as mock_send:
            await websocket_server._process_message(connection_id, message_data)

            # 验证发送了响应消息
            assert mock_send.call_count > 0

    @pytest.mark.asyncio
    async def test_process_tts_request_with_cache(self, websocket_server):
        """测试处理带缓存的TTS请求"""
        connection_id = "test-conn"

        message_data = {
            "type": "request",
            "data": {
                "text": "缓存的文本",
                "voice_id": "default"
            }
        }

        # 模拟缓存命中
        websocket_server.cache_manager.get_audio = Mock(
            return_value=(b"cached_audio", {"duration": 3.0})
        )

        # 模拟发送消息
        with patch.object(websocket_server.manager, 'send_message', new_callable=AsyncMock) as mock_send:
            await websocket_server._process_message(connection_id, message_data)

            # 验证发送了响应消息
            assert mock_send.call_count > 0

    @pytest.mark.asyncio
    async def test_process_heartbeat(self, websocket_server):
        """测试处理心跳"""
        connection_id = "test-conn"

        message_data = {
            "type": "heartbeat"
        }

        # 模拟发送心跳响应
        with patch.object(websocket_server.manager, 'send_message', new_callable=AsyncMock) as mock_send:
            await websocket_server._process_message(connection_id, message_data)

            # 验证发送了心跳响应
            mock_send.assert_called_once()
            args = mock_send.call_args[0]
            assert args[0] == connection_id
            response = args[1]
            assert response['type'] == 'heartbeat'

    @pytest.mark.asyncio
    async def test_process_error(self, websocket_server):
        """测试处理错误"""
        connection_id = "test-conn"

        message_data = {
            "type": "unknown"
        }

        # 模拟发送错误消息
        with patch.object(websocket_server.manager, 'send_message', new_callable=AsyncMock) as mock_send:
            await websocket_server._process_message(connection_id, message_data)

            # 验证发送了错误消息
            mock_send.assert_called_once()
            args = mock_send.call_args[0]
            assert args[0] == connection_id
            response = args[1]
            assert response['type'] == 'error'

    @pytest.mark.asyncio
    async def test_send_audio_chunk(self, websocket_server):
        """测试发送音频分块"""
        connection_id = "test-conn"
        chunk_id = 1
        audio_data = b"test audio chunk"
        request_id = "req-123"

        # 模拟发送消息
        with patch.object(websocket_server.manager, 'send_message', new_callable=AsyncMock) as mock_send:
            await websocket_server._send_audio_chunk(
                connection_id, chunk_id, audio_data, request_id, False
            )

            # 验证发送了正确的消息
            mock_send.assert_called_once()
            args = mock_send.call_args[0]
            assert args[0] == connection_id
            response = args[1]

            assert response['type'] == 'response'
            assert response['chunk_id'] == chunk_id
            assert 'audio_data' in response
            assert response['is_final'] is False
            assert response['request_id'] == request_id

    @pytest.mark.asyncio
    async def test_send_error(self, websocket_server):
        """测试发送错误消息"""
        connection_id = "test-conn"
        code = "TEST_ERROR"
        message = "测试错误"

        # 模拟发送错误消息
        with patch.object(websocket_server.manager, 'send_message', new_callable=AsyncMock) as mock_send:
            await websocket_server._send_error(connection_id, code, message)

            # 验证发送了正确的错误消息
            mock_send.assert_called_once()
            args = mock_send.call_args[0]
            assert args[0] == connection_id
            response = args[1]

            assert response['type'] == 'error'
            assert response['code'] == code
            assert response['message'] == message

    def test_get_connection_count(self, websocket_server):
        """测试获取连接数"""
        # 初始状态应该没有连接
        count = websocket_server.get_connection_count()
        assert count == 0

    def test_get_performance_stats(self, websocket_server):
        """测试获取性能统计"""
        stats = websocket_server.get_performance_stats()

        assert 'uptime_seconds' in stats
        assert 'total_requests' in stats
        assert 'avg_response_time' in stats
        assert 'qps' in stats

    @pytest.mark.asyncio
    async def test_handle_connection(self, websocket_server):
        """测试处理连接"""
        mock_websocket = Mock()
        mock_websocket.receive_text = AsyncMock()
        mock_websocket.receive_text.side_effect = [
            json.dumps({
                "type": "request",
                "data": {"text": "hello"}
            }),
            WebSocketDisconnect()  # 模拟断开
        ]

        connection_id = "test-conn"

        # 模拟合成
        websocket_server.tts_system.synthesize_stream_async = AsyncMock()
        websocket_server.cache_manager.get_audio = Mock(return_value=None)

        with patch.object(websocket_server.manager, 'connect', new_callable=AsyncMock):
            with patch.object(websocket_server.manager, 'disconnect'):
                # 这里应该处理连接并接收消息
                try:
                    await websocket_server.handle_connection(mock_websocket, connection_id)
                except WebSocketDisconnect:
                    pass  # 预期的断开

    @pytest.mark.asyncio
    async def test_broadcast_message(self, websocket_server):
        """测试广播消息"""
        # 模拟广播
        with patch.object(websocket_server.manager, 'broadcast', new_callable=AsyncMock) as mock_broadcast:
            message = {"type": "broadcast", "data": "test"}
            await websocket_server.broadcast_message(message)

            # 验证广播调用
            mock_broadcast.assert_called_once_with(message)


class TestWebSocketMessageFormats:
    """测试WebSocket消息格式"""

    @pytest.mark.asyncio
    async def test_valid_request_format(self, websocket_server):
        """测试有效请求格式"""
        connection_id = "test-conn"

        message_data = {
            "type": "request",
            "data": {
                "text": "测试文本",
                "voice_id": "default",
                "language": "zh-CN",
                "emotion": "happy",
                "emotion_intensity": 0.8,
                "speed": 1.0,
                "chunk_size": 1024
            }
        }

        websocket_server.cache_manager.get_audio = Mock(return_value=None)
        websocket_server.tts_system.synthesize_stream_async = AsyncMock()
        websocket_server.tts_system.synthesize_stream_async.return_value = iter([b"chunk"])

        with patch.object(websocket_server.manager, 'send_message', new_callable=AsyncMock):
            await websocket_server._process_message(connection_id, message_data)

    @pytest.mark.asyncio
    async def test_minimal_request_format(self, websocket_server):
        """测试最小请求格式"""
        connection_id = "test-conn"

        message_data = {
            "type": "request",
            "data": {
                "text": "hello"
            }
        }

        websocket_server.cache_manager.get_audio = Mock(return_value=None)
        websocket_server.tts_system.synthesize_stream_async = AsyncMock()
        websocket_server.tts_system.synthesize_stream_async.return_value = iter([b"chunk"])

        with patch.object(websocket_server.manager, 'send_message', new_callable=AsyncMock):
            await websocket_server._process_message(connection_id, message_data)

    def test_response_format(self, websocket_server):
        """测试响应格式"""
        # 验证响应格式
        import base64

        chunk_id = 1
        audio_data = b"test audio"
        request_id = "req-123"

        # 这里可以验证响应消息的格式
        # 实际测试中会通过_send_audio_chunk验证


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
