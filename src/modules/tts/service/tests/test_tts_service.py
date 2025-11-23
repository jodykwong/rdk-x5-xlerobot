"""
TTS服务功能测试

测试TTS服务的各项功能。
"""

import pytest
import asyncio
from unittest.mock import Mock, AsyncMock, patch
import sys
import os

# 添加路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

from service.models.request_models import TTSRequest, AudioFormat
from service.utils.cache_manager import TTSCacheManager
from service.utils.performance_monitor import PerformanceMonitor
from service.utils.voice_loader import VoiceLoader


class TestTTSCacheManager:
    """测试TTS缓存管理器"""

    def test_generate_key(self):
        """测试缓存键生成"""
        cache = TTSCacheManager()

        key1 = cache._generate_key("hello", "voice1", language="en")
        key2 = cache._generate_key("hello", "voice1", language="en")
        key3 = cache._generate_key("hello", "voice2", language="en")

        assert key1 == key2
        assert key1 != key3

    def test_set_and_get_audio(self):
        """测试音频缓存设置和获取"""
        cache = TTSCacheManager(max_size=10, ttl=60)

        audio_data = b"test audio data"
        cache.set_audio("hello", "voice1", audio_data, language="en")

        result = cache.get_audio("hello", "voice1", language="en")

        assert result is not None
        audio, metadata = result
        assert audio == audio_data
        assert metadata['text'] == "hello"
        assert metadata['voice_id'] == "voice1"

    def test_cache_expiration(self):
        """测试缓存过期"""
        cache = TTSCacheManager(ttl=1)

        cache.set_audio("hello", "voice1", b"data")
        result = cache.get_audio("hello", "voice1")

        assert result is not None

        import time
        time.sleep(1.1)

        result = cache.get_audio("hello", "voice1")
        assert result is None

    def test_cache_eviction(self):
        """测试缓存淘汰"""
        cache = TTSCacheManager(max_size=2)

        cache.set_audio("text1", "voice1", b"data1")
        cache.set_audio("text2", "voice2", b"data2")
        cache.set_audio("text3", "voice3", b"data3")

        assert cache.get_audio("text1", "voice1") is None
        assert cache.get_audio("text2", "voice2") is not None
        assert cache.get_audio("text3", "voice3") is not None

    def test_cache_stats(self):
        """测试缓存统计"""
        cache = TTSCacheManager()

        cache.set_audio("hello", "voice1", b"data1")
        cache.set_audio("world", "voice2", b"data2")

        stats = cache.get_cache_stats()

        assert stats['audio_cache_count'] == 2
        assert stats['audio_cache_size'] > 0
        assert stats['max_size'] == 1000
        assert stats['ttl'] == 3600


class TestPerformanceMonitor:
    """测试性能监控器"""

    def test_record_request(self):
        """测试请求记录"""
        monitor = PerformanceMonitor()

        monitor.record_request(0.5, True, True)
        monitor.record_request(1.0, True, False)
        monitor.record_request(0.3, False, False)

        stats = monitor.get_stats()

        assert stats['total_requests'] == 3
        assert stats['total_successes'] == 2
        assert stats['total_errors'] == 1
        assert stats['avg_response_time'] > 0
        assert stats['cache_hits'] == 1
        assert stats['cache_misses'] == 2

    def test_response_time_percentiles(self):
        """测试响应时间百分位数"""
        monitor = PerformanceMonitor()

        for i in range(100):
            monitor.record_request(i * 0.01, True, False)

        stats = monitor.get_stats()

        assert stats['min_response_time'] >= 0
        assert stats['max_response_time'] <= 1.0
        assert stats['p95_response_time'] > stats['p50_response_time']
        assert stats['p99_response_time'] >= stats['p95_response_time']

    def test_health_status(self):
        """测试健康状态"""
        monitor = PerformanceMonitor()

        monitor.record_request(0.1, True, False)
        monitor.record_request(0.2, True, False)

        health = monitor.get_health_status()

        assert health['status'] == 'healthy'
        assert 'metrics' in health

    def test_connection_tracking(self):
        """测试连接跟踪"""
        monitor = PerformanceMonitor()

        assert monitor.active_connections == 0

        monitor.record_connection(True)
        assert monitor.active_connections == 1

        monitor.record_connection(True)
        assert monitor.active_connections == 2

        monitor.record_connection(False)
        assert monitor.active_connections == 1

        monitor.record_connection(False)
        assert monitor.active_connections == 0


class TestVoiceLoader:
    """测试音色预加载器"""

    def test_is_voice_loaded(self):
        """测试音色加载状态"""
        loader = VoiceLoader()

        assert not loader.is_voice_loaded("voice1")

    @pytest.mark.asyncio
    async def test_load_voice(self):
        """测试音色加载"""
        voice_manager = Mock()
        voice_manager.load_voice = Mock()

        loader = VoiceLoader()

        result = loader.load_voice("voice1", voice_manager)

        assert result is True
        assert loader.is_voice_loaded("voice1")

    def test_unload_voice(self):
        """测试音色卸载"""
        loader = VoiceLoader()

        # 先加载音色
        loader._loaded_voices.add("voice1")
        assert loader.is_voice_loaded("voice1")

        # 卸载音色
        result = loader.unload_voice("voice1")
        assert result is True
        assert not loader.is_voice_loaded("voice1")

    def test_get_loaded_voices(self):
        """测试获取已加载音色"""
        loader = VoiceLoader()

        loader._loaded_voices.add("voice1")
        loader._loaded_voices.add("voice2")

        voices = loader.get_loaded_voices()
        assert "voice1" in voices
        assert "voice2" in voices
        assert len(voices) == 2

    def test_shutdown(self):
        """测试关闭"""
        loader = VoiceLoader()

        # 模拟关闭
        loader.shutdown()


class TestTTSRequestModel:
    """测试TTS请求模型"""

    def test_valid_request(self):
        """测试有效请求"""
        request = TTSRequest(
            text="你好",
            voice_id="default",
            emotion="happy"
        )

        assert request.text == "你好"
        assert request.voice_id == "default"
        assert request.emotion == "happy"

    def test_invalid_empty_text(self):
        """测试空文本验证"""
        with pytest.raises(ValueError):
            TTSRequest(text="")

    def test_emotion_intensity_range(self):
        """测试情感强度范围"""
        request = TTSRequest(
            text="hello",
            emotion_intensity=0.5
        )
        assert request.emotion_intensity == 0.5

        with pytest.raises(ValueError):
            TTSRequest(text="hello", emotion_intensity=1.5)

        with pytest.raises(ValueError):
            TTSRequest(text="hello", emotion_intensity=-0.1)

    def test_speed_range(self):
        """测试语速范围"""
        request = TTSRequest(
            text="hello",
            speed=1.0
        )
        assert request.speed == 1.0

        with pytest.raises(ValueError):
            TTSRequest(text="hello", speed=3.0)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
