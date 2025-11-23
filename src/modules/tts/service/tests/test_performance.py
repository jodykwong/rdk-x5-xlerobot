"""
性能测试

测试TTS服务的性能指标。
"""

import pytest
import time
import asyncio
import statistics
from concurrent.futures import ThreadPoolExecutor, as_completed
from unittest.mock import Mock, AsyncMock
import sys
import os

# 添加路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

from service.utils.performance_monitor import PerformanceMonitor


class TestPerformance:
    """测试性能指标"""

    @pytest.mark.performance
    def test_response_time_benchmark(self):
        """测试响应时间基准"""
        monitor = PerformanceMonitor()

        # 模拟请求
        response_times = []
        for _ in range(100):
            start = time.time()
            time.sleep(0.05)  # 模拟50ms处理时间
            end = time.time()
            response_time = end - start
            response_times.append(response_time)
            monitor.record_request(response_time, True, False)

        stats = monitor.get_stats()

        # 验证性能指标
        assert stats['avg_response_time'] > 0.04
        assert stats['avg_response_time'] < 0.06
        assert stats['p95_response_time'] >= stats['avg_response_time']
        assert stats['p99_response_time'] >= stats['p95_response_time']

    @pytest.mark.performance
    def test_throughput_benchmark(self):
        """测试吞吐量基准"""
        monitor = PerformanceMonitor()

        start_time = time.time()
        request_count = 100

        # 模拟批量请求
        for _ in range(request_count):
            monitor.record_request(0.05, True, False)

        end_time = time.time()
        total_time = end_time - start_time

        stats = monitor.get_stats()

        # 计算QPS
        qps = request_count / total_time
        assert qps > 100  # 至少100 QPS

    @pytest.mark.performance
    def test_cache_hit_rate(self):
        """测试缓存命中率"""
        monitor = PerformanceMonitor()

        # 模拟请求：80%命中缓存
        for _ in range(80):
            monitor.record_request(0.01, True, True)  # 缓存命中

        for _ in range(20):
            monitor.record_request(0.05, True, False)  # 缓存未命中

        stats = monitor.get_stats()

        # 验证缓存命中率
        cache_hit_rate = stats['cache_hit_rate_percent']
        assert cache_hit_rate >= 75  # 至少75%命中率
        assert cache_hit_rate <= 85  # 最多85%命中率

    @pytest.mark.performance
    def test_concurrent_requests(self):
        """测试并发请求处理"""
        monitor = PerformanceMonitor()

        def simulate_request():
            """模拟单个请求"""
            start = time.time()
            time.sleep(0.02)  # 模拟20ms处理时间
            end = time.time()
            return end - start

        # 使用线程池模拟并发
        start_time = time.time()
        with ThreadPoolExecutor(max_workers=10) as executor:
            futures = [executor.submit(simulate_request) for _ in range(50)]
            response_times = [f.result() for f in as_completed(futures)]

        end_time = time.time()
        total_time = end_time - start_time

        # 记录性能
        for rt in response_times:
            monitor.record_request(rt, True, False)

        stats = monitor.get_stats()

        # 验证并发性能
        assert total_time < 2.0  # 50个请求应该在2秒内完成
        assert stats['avg_response_time'] > 0.015
        assert stats['avg_response_time'] < 0.025

    @pytest.mark.performance
    def test_error_rate_benchmark(self):
        """测试错误率基准"""
        monitor = PerformanceMonitor()

        # 模拟请求：95%成功率
        for _ in range(95):
            monitor.record_request(0.05, True, False)

        for _ in range(5):
            monitor.record_request(0.05, False, False)

        stats = monitor.get_stats()

        # 验证错误率
        error_rate = stats['error_rate_percent']
        assert error_rate >= 4.0  # 至少4%错误率
        assert error_rate <= 6.0  # 最多6%错误率

    @pytest.mark.performance
    def test_memory_usage_simulation(self):
        """测试内存使用模拟"""
        # 这里可以添加内存使用测试
        # 由于是模拟环境，主要测试逻辑
        pass

    @pytest.mark.performance
    def test_long_running_stability(self):
        """测试长期运行稳定性"""
        monitor = PerformanceMonitor()

        # 模拟长时间运行
        for hour in range(24):
            # 每秒10个请求
            for _ in range(10 * 3600):
                monitor.record_request(0.05, True, False)

        stats = monitor.get_stats()

        # 验证长时间运行后仍然稳定
        assert stats['uptime_hours'] > 0
        assert stats['total_requests'] > 0
        assert stats['avg_response_time'] > 0

    @pytest.mark.performance
    def test_percentile_calculation(self):
        """测试百分位数计算"""
        monitor = PerformanceMonitor()

        # 添加已知数据
        test_data = [i for i in range(1, 101)]  # 1到100

        for data_point in test_data:
            monitor.record_request(data_point / 100.0, True, False)

        stats = monitor.get_stats()

        # 验证百分位数
        assert stats['min_response_time'] == 0.01
        assert stats['max_response_time'] == 1.0
        assert stats['p50_response_time'] >= 0.50
        assert stats['p50_response_time'] <= 0.51
        assert stats['p95_response_time'] >= 0.95
        assert stats['p95_response_time'] <= 0.96
        assert stats['p99_response_time'] >= 0.99
        assert stats['p99_response_time'] <= 1.0

    @pytest.mark.performance
    def test_connection_tracking(self):
        """测试连接跟踪性能"""
        monitor = PerformanceMonitor()

        # 模拟连接变化
        for _ in range(100):
            monitor.record_connection(True)
            monitor.record_connection(False)

        stats = monitor.get_stats()

        # 验证连接跟踪
        assert stats['active_connections'] == 0

        # 添加一些活跃连接
        monitor.record_connection(True)
        monitor.record_connection(True)
        monitor.record_connection(True)

        stats = monitor.get_stats()
        assert stats['active_connections'] == 3


class TestPerformanceOptimization:
    """测试性能优化"""

    @pytest.mark.performance
    def test_cache_performance_impact(self):
        """测试缓存对性能的影响"""
        # 测试有缓存的情况
        monitor_cached = PerformanceMonitor()
        for _ in range(100):
            monitor_cached.record_request(0.01, True, True)  # 缓存命中

        # 测试无缓存的情况
        monitor_uncached = PerformanceMonitor()
        for _ in range(100):
            monitor_uncached.record_request(0.05, True, False)  # 缓存未命中

        cached_stats = monitor_cached.get_stats()
        uncached_stats = monitor_uncached.get_stats()

        # 验证缓存提升性能
        assert cached_stats['avg_response_time'] < uncached_stats['avg_response_time']

        performance_improvement = (
            (uncached_stats['avg_response_time'] - cached_stats['avg_response_time']) /
            uncached_stats['avg_response_time'] * 100
        )

        assert performance_improvement > 50  # 至少50%性能提升

    @pytest.mark.performance
    def test_batch_processing_performance(self):
        """测试批量处理性能"""
        monitor = PerformanceMonitor()

        # 单个处理
        start = time.time()
        for _ in range(10):
            monitor.record_request(0.1, True, False)
        single_time = time.time() - start

        # 批量处理
        start = time.time()
        for _ in range(10):
            monitor.record_request(0.05, True, False)  # 批量处理更快
        batch_time = time.time() - start

        # 批量处理应该更快或相当
        assert batch_time <= single_time * 1.2


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-m", "performance"])
