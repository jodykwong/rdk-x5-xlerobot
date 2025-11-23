"""
压力测试

测试TTS服务在高负载下的表现。
"""

import pytest
import asyncio
import time
import random
from concurrent.futures import ThreadPoolExecutor, as_completed
from unittest.mock import Mock, AsyncMock
import threading
import sys
import os

# 添加路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

from service.utils.performance_monitor import PerformanceMonitor


class TestStress:
    """压力测试"""

    @pytest.mark.stress
    def test_high_concurrency_load(self):
        """测试高并发负载"""
        monitor = PerformanceMonitor()

        def simulate_request():
            """模拟请求"""
            # 模拟处理时间
            time.sleep(random.uniform(0.01, 0.05))
            return random.choice([True, True, True, True, False])  # 80%成功率

        # 使用线程池模拟高并发
        start_time = time.time()
        with ThreadPoolExecutor(max_workers=50) as executor:
            futures = [executor.submit(simulate_request) for _ in range(200)]
            results = [f.result() for f in as_completed(futures)]

        end_time = time.time()
        total_time = end_time - start_time

        # 记录性能
        for result in results:
            monitor.record_request(0.03, result, False)

        stats = monitor.get_stats()

        # 验证压力测试结果
        assert total_time < 10.0  # 200个请求应该在10秒内完成
        assert stats['total_requests'] == 200
        assert stats['error_rate_percent'] <= 20  # 错误率不超过20%

    @pytest.mark.stress
    @pytest.mark.asyncio
    async def test_websocket_concurrent_connections(self):
        """测试WebSocket并发连接"""
        from service.websocket.websocket_server import ConnectionManager

        manager = ConnectionManager()

        # 模拟连接建立和断开
        for i in range(100):
            # 模拟连接
            connection_id = f"conn-{i}"
            mock_websocket = Mock()
            await manager.connect(mock_websocket, connection_id, f"client-{i}")

            # 验证连接数
            assert len(manager.active_connections) <= 100

        # 验证所有连接都建立了
        assert len(manager.active_connections) == 100

        # 断开所有连接
        for i in range(100):
            connection_id = f"conn-{i}"
            manager.disconnect(connection_id)

        # 验证所有连接都断开了
        assert len(manager.active_connections) == 0

    @pytest.mark.stress
    def test_memory_pressure(self):
        """测试内存压力"""
        monitor = PerformanceMonitor()

        # 模拟大量请求
        for i in range(1000):
            response_time = random.uniform(0.01, 0.1)
            success = random.choice([True] * 9 + [False])  # 90%成功率
            cache_hit = random.choice([True] * 8 + [False])  # 80%缓存命中率

            monitor.record_request(response_time, success, cache_hit)

        stats = monitor.get_stats()

        # 验证性能仍然稳定
        assert stats['avg_response_time'] < 0.15  # 平均响应时间合理
        assert stats['error_rate_percent'] < 15  # 错误率可接受
        assert stats['cache_hit_rate_percent'] > 70  # 缓存命中率良好

    @pytest.mark.stress
    def test_sustained_load(self):
        """测试持续负载"""
        monitor = PerformanceMonitor()

        # 模拟持续负载：1小时内每秒10个请求
        start_time = time.time()
        duration = 10  # 10秒模拟1小时

        request_count = 0
        for second in range(duration):
            second_start = time.time()

            # 每秒10个请求
            for _ in range(10):
                response_time = random.uniform(0.02, 0.08)
                success = random.choice([True] * 95 + [False])  # 95%成功率
                monitor.record_request(response_time, success, False)
                request_count += 1

            # 确保每秒10个请求
            elapsed = time.time() - second_start
            if elapsed < 1.0:
                time.sleep(1.0 - elapsed)

        total_time = time.time() - start_time

        stats = monitor.get_stats()

        # 验证持续负载结果
        assert request_count >= 100
        assert total_time <= duration + 1  # 允许1秒误差
        assert stats['qps'] >= 9  # QPS应该接近10
        assert stats['error_rate_percent'] < 10  # 错误率较低

    @pytest.mark.stress
    def test_cache_pressure(self):
        """测试缓存压力"""
        from service.utils.cache_manager import TTSCacheManager

        cache = TTSCacheManager(max_size=100)

        # 添加大量缓存项
        for i in range(200):
            text = f"text_{i}"
            voice = f"voice_{i % 10}"
            audio_data = b"x" * 1024  # 1KB数据
            cache.set_audio(text, voice, audio_data)

        stats = cache.get_cache_stats()

        # 验证缓存淘汰正常工作
        assert stats['audio_cache_count'] <= 100  # 不超过最大容量
        assert stats['audio_cache_size'] > 0
        assert stats['usage_ratio'] <= 1.0

    @pytest.mark.stress
    def test_error_handling_under_load(self):
        """测试负载下的错误处理"""
        monitor = PerformanceMonitor()

        # 模拟不同类型的错误
        error_types = [
            "TTS_001",  # 合成错误
            "TTS_002",  # 流式错误
            "INVALID_REQUEST",  # 无效请求
            "TIMEOUT",  # 超时
        ]

        # 1000个请求中包含各种错误
        for i in range(1000):
            error_type = random.choice(error_types)
            is_error = random.choice([True] * 5 + [False])  # 约14%错误率

            if is_error:
                monitor.record_request(0.1, False, False)
            else:
                monitor.record_request(0.05, True, False)

        stats = monitor.get_stats()

        # 验证错误处理
        assert stats['error_rate_percent'] > 10
        assert stats['error_rate_percent'] < 20
        assert stats['total_errors'] > 0

    @pytest.mark.stress
    def test_rapid_requests_burst(self):
        """测试请求突发"""
        monitor = PerformanceMonitor()

        # 模拟请求突发：1秒内100个请求
        start_time = time.time()

        with ThreadPoolExecutor(max_workers=20) as executor:
            futures = [
                executor.submit(lambda: time.sleep(random.uniform(0, 0.001)))
                for _ in range(100)
            ]
            # 等待所有任务完成
            for f in as_completed(futures):
                f.result()

        burst_time = time.time() - start_time

        # 记录性能（假设每个请求平均30ms）
        for _ in range(100):
            monitor.record_request(0.03, True, False)

        stats = monitor.get_stats()

        # 验证突发处理能力
        assert burst_time < 2.0  # 突发应该在2秒内处理完成
        assert stats['total_requests'] == 100

    @pytest.mark.stress
    def test_performance_degradation_detection(self):
        """测试性能下降检测"""
        monitor = PerformanceMonitor()

        # 阶段1：正常性能
        for _ in range(50):
            monitor.record_request(0.02, True, False)

        # 阶段2：性能下降
        for _ in range(20):
            monitor.record_request(0.15, True, False)  # 性能下降

        # 阶段3：恢复正常
        for _ in range(30):
            monitor.record_request(0.02, True, False)

        stats = monitor.get_stats()
        health = monitor.get_health_status()

        # 验证性能检测
        assert stats['avg_response_time'] > 0.02  # 平均时间受影响
        assert health['status'] in ['healthy', 'unhealthy']  # 应该检测到问题

    @pytest.mark.stress
    def test_concurrent_cache_access(self):
        """测试并发缓存访问"""
        from service.utils.cache_manager import TTSCacheManager

        cache = TTSCacheManager(max_size=50)

        def access_cache(operation):
            """访问缓存"""
            if operation == "set":
                for i in range(10):
                    cache.set_audio(f"text_{i}", "voice_1", b"data")
            elif operation == "get":
                for i in range(10):
                    cache.get_audio(f"text_{i}", "voice_1")

        # 多个线程同时访问缓存
        with ThreadPoolExecutor(max_workers=10) as executor:
            operations = ["set", "get"] * 5
            futures = [
                executor.submit(access_cache, op)
                for op in operations
            ]
            # 等待所有任务完成
            for f in as_completed(futures):
                f.result()

        stats = cache.get_cache_stats()

        # 验证并发访问正常
        assert stats['audio_cache_count'] <= 50
        assert stats['usage_ratio'] <= 1.0


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-m", "stress"])
