"""
API端点测试

测试RESTful API的各个端点。
"""

import pytest
import json
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch
import sys
import os

# 添加路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

from service.api.tts_api import TTSAPIServer


class TestTTSAPIEndpoints:
    """测试TTS API端点"""

    @pytest.fixture
    def mock_tts_system(self):
        """模拟TTS系统"""
        mock = Mock()
        mock.synthesize = AsyncMock(return_value=b"fake audio data")
        mock.synthesize_stream = AsyncMock(return_value=[b"chunk1", b"chunk2"])
        return mock

    @pytest.fixture
    def mock_voice_manager(self):
        """模拟音色管理器"""
        mock = Mock()
        mock.list_voices = Mock(return_value=[
            {
                "voice_id": "default",
                "name": "默认音色",
                "language": "zh-CN",
                "gender": "female",
                "emotion_support": True,
                "quality": "medium"
            },
            {
                "voice_id": "male",
                "name": "男声",
                "language": "zh-CN",
                "gender": "male",
                "emotion_support": True,
                "quality": "high"
            }
        ])
        return mock

    @pytest.fixture
    def api_server(self, mock_tts_system, mock_voice_manager):
        """创建API服务器"""
        return TTSAPIServer(mock_tts_system, mock_voice_manager)

    @pytest.fixture
    def client(self, api_server):
        """创建测试客户端"""
        return TestClient(api_server.get_app())

    def test_health_check(self, client):
        """测试健康检查"""
        response = client.get("/api/v1/tts/health")
        assert response.status_code == 200

        data = response.json()
        assert data["status"] == "healthy"
        assert "version" in data
        assert "uptime" in data
        assert "request_id" in data

    def test_health_check_detailed(self, client):
        """测试详细健康检查"""
        response = client.get("/api/v1/tts/health?detailed=true")
        assert response.status_code == 200

        data = response.json()
        assert "services" in data
        assert "performance" in data

    def test_get_voices(self, client):
        """测试获取音色列表"""
        response = client.get("/api/v1/tts/voices")
        assert response.status_code == 200

        data = response.json()
        assert data["success"] is True
        assert "voices" in data
        assert "total" in data
        assert data["total"] > 0

    def test_get_voices_with_filter(self, client, mock_voice_manager):
        """测试带过滤的音色列表"""
        response = client.get("/api/v1/tts/voices?language=zh-CN")
        assert response.status_code == 200

        data = response.json()
        assert data["success"] is True

    def test_synthesize_basic(self, client):
        """测试基本TTS合成"""
        request_data = {
            "text": "你好，欢迎使用TTS服务！",
            "voice_id": "default"
        }

        response = client.post("/api/v1/tts/synthesize", json=request_data)
        assert response.status_code == 200

        data = response.json()
        assert data["success"] is True
        assert "audio_data" in data
        assert "audio_format" in data
        assert "request_id" in data
        assert "processing_time" in data

    def test_synthesize_with_emotion(self, client):
        """测试情感TTS合成"""
        request_data = {
            "text": "今天天气真好！",
            "voice_id": "default",
            "emotion": "happy",
            "emotion_intensity": 0.8
        }

        response = client.post("/api/v1/tts/synthesize", json=request_data)
        assert response.status_code == 200

        data = response.json()
        assert data["success"] is True

    def test_synthesize_stream(self, client):
        """测试流式TTS合成"""
        request_data = {
            "text": "这是流式TTS测试",
            "voice_id": "default"
        }

        response = client.post("/api/v1/tts/synthesize-stream", json=request_data)
        assert response.status_code == 200

        data = response.json()
        assert "request_id" in data
        assert "chunks" in data

    def test_invalid_request(self, client):
        """测试无效请求"""
        # 空文本
        request_data = {
            "text": ""
        }

        response = client.post("/api/v1/tts/synthesize", json=request_data)
        assert response.status_code == 422

    def test_invalid_emotion_intensity(self, client):
        """测试无效情感强度"""
        request_data = {
            "text": "hello",
            "emotion_intensity": 2.0  # 超出范围
        }

        response = client.post("/api/v1/tts/synthesize", json=request_data)
        assert response.status_code == 422

    def test_cache_functionality(self, client, mock_tts_system):
        """测试缓存功能"""
        # 第一次请求
        request_data = {
            "text": "测试缓存",
            "voice_id": "default",
            "cache_enabled": True
        }

        response1 = client.post("/api/v1/tts/synthesize", json=request_data)
        assert response1.status_code == 200

        # 第二次相同请求（应该命中缓存）
        response2 = client.post("/api/v1/tts/synthesize", json=request_data)
        assert response2.status_code == 200

        data = response2.json()
        assert data["cache_hit"] is True

    def test_audio_format_parameter(self, client):
        """测试音频格式参数"""
        request_data = {
            "text": "hello",
            "audio_format": "mp3"
        }

        response = client.post("/api/v1/tts/synthesize", json=request_data)
        assert response.status_code == 200

        data = response.json()
        assert data["audio_format"] == "mp3"

    def test_sample_rate_parameter(self, client):
        """测试采样率参数"""
        request_data = {
            "text": "hello",
            "sample_rate": 44100
        }

        response = client.post("/api/v1/tts/synthesize", json=request_data)
        assert response.status_code == 200

        data = response.json()
        assert data["sample_rate"] == 44100

    def test_voice_parameters(self, client):
        """测试音色参数"""
        request_data = {
            "text": "hello",
            "voice_id": "custom_voice",
            "speed": 1.5,
            "pitch": 0.8,
            "volume": 1.2
        }

        response = client.post("/api/v1/tts/synthesize", json=request_data)
        assert response.status_code == 200

    def test_stream_chunk_size(self, client):
        """测试流式分块大小"""
        request_data = {
            "text": "hello",
            "chunk_size": 2048
        }

        response = client.post("/api/v1/tts/synthesize-stream", json=request_data)
        assert response.status_code == 200


class TestAPIErrorHandling:
    """测试API错误处理"""

    @pytest.fixture
    def mock_tts_system_error(self):
        """模拟TTS系统错误"""
        mock = Mock()
        mock.synthesize = AsyncMock(side_effect=Exception("TTS error"))
        return mock

    @pytest.fixture
    def api_server_error(self, mock_tts_system_error):
        """创建错误模拟的API服务器"""
        mock_voice_manager = Mock()
        return TTSAPIServer(mock_tts_system_error, mock_voice_manager)

    @pytest.fixture
    def client_error(self, api_server_error):
        """创建错误测试客户端"""
        return TestClient(api_server_error.get_app())

    def test_tts_error_handling(self, client_error):
        """测试TTS错误处理"""
        request_data = {
            "text": "hello"
        }

        response = client_error.post("/api/v1/tts/synthesize", json=request_data)
        assert response.status_code == 500

    def test_server_error_response(self, client_error):
        """测试服务器错误响应"""
        response = client_error.get("/nonexistent")
        assert response.status_code == 404


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
