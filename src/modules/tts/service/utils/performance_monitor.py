"""
性能监控器

提供TTS服务的性能指标收集和监控功能。
"""

import time
import threading
from typing import Dict, List, Any, Optional
from collections import deque, defaultdict
from datetime import datetime, timedelta
import logging

logger = logging.getLogger(__name__)


class PerformanceMonitor:
    """性能监控器"""

    def __init__(self, window_size: int = 1000):
        """
        初始化性能监控器

        Args:
            window_size: 统计窗口大小
        """
        self.window_size = window_size
        self._lock = threading.RLock()

        # 响应时间统计（滑动窗口）
        self.response_times = deque(maxlen=window_size)

        # 请求计数
        self.request_counts = defaultdict(int)
        self.success_counts = defaultdict(int)
        self.error_counts = defaultdict(int)

        # 当前活跃连接数
        self.active_connections = 0

        # 缓存命中统计
        self.cache_hits = 0
        self.cache_misses = 0

        # 开始时间
        self.start_time = datetime.now()

    def record_request(self, response_time: float, success: bool, cache_hit: bool = False) -> None:
        """
        记录请求

        Args:
            response_time: 响应时间（秒）
            success: 是否成功
            cache_hit: 是否命中缓存
        """
        with self._lock:
            self.response_times.append(response_time)

            now = datetime.now()
            time_key = now.strftime('%Y-%m-%d %H:%M:%S')

            self.request_counts[time_key] += 1

            if success:
                self.success_counts[time_key] += 1
            else:
                self.error_counts[time_key] += 1

            if cache_hit:
                self.cache_hits += 1
            else:
                self.cache_misses += 1

    def record_connection(self, increment: bool = True) -> None:
        """
        记录连接变化

        Args:
            increment: True表示增加连接，False表示减少连接
        """
        with self._lock:
            if increment:
                self.active_connections += 1
            else:
                self.active_connections = max(0, self.active_connections - 1)

    def get_stats(self) -> Dict[str, Any]:
        """
        获取性能统计

        Returns:
            性能统计字典
        """
        with self._lock:
            now = datetime.now()
            uptime = (now - self.start_time).total_seconds()

            # 计算响应时间统计
            if self.response_times:
                times = list(self.response_times)
                avg_time = sum(times) / len(times)
                min_time = min(times)
                max_time = max(times)
                p95_time = self._percentile(times, 95)
                p99_time = self._percentile(times, 99)
            else:
                avg_time = min_time = max_time = p95_time = p99_time = 0.0

            # 计算QPS
            recent_minutes = 5
            recent_time = now - timedelta(minutes=recent_minutes)
            recent_requests = sum(
                count for time_key, count in self.request_counts.items()
                if datetime.strptime(time_key, '%Y-%m-%d %H:%M:%S') >= recent_time
            )
            qps = recent_requests / (recent_minutes * 60) if recent_minutes > 0 else 0

            # 计算错误率
            total_requests = sum(self.request_counts.values())
            total_successes = sum(self.success_counts.values())
            total_errors = sum(self.error_counts.values())

            error_rate = (total_errors / total_requests * 100) if total_requests > 0 else 0

            # 计算缓存命中率
            cache_total = self.cache_hits + self.cache_misses
            cache_hit_rate = (self.cache_hits / cache_total * 100) if cache_total > 0 else 0

            return {
                'uptime_seconds': uptime,
                'uptime_hours': uptime / 3600,
                'total_requests': total_requests,
                'total_successes': total_successes,
                'total_errors': total_errors,
                'error_rate_percent': round(error_rate, 2),
                'active_connections': self.active_connections,
                'avg_response_time': round(avg_time, 4),
                'min_response_time': round(min_time, 4),
                'max_response_time': round(max_time, 4),
                'p95_response_time': round(p95_time, 4),
                'p99_response_time': round(p99_time, 4),
                'qps': round(qps, 2),
                'cache_hits': self.cache_hits,
                'cache_misses': self.cache_misses,
                'cache_hit_rate_percent': round(cache_hit_rate, 2)
            }

    def _percentile(self, data: List[float], percentile: float) -> float:
        """
        计算百分位数

        Args:
            data: 数据列表
            percentile: 百分位数 (0-100)

        Returns:
            百分位数值
        """
        if not data:
            return 0.0

        sorted_data = sorted(data)
        index = (percentile / 100.0) * (len(sorted_data) - 1)

        if index == int(index):
            return sorted_data[int(index)]
        else:
            lower = sorted_data[int(index)]
            upper = sorted_data[int(index) + 1]
            return lower + (upper - lower) * (index - int(index))

    def reset(self) -> None:
        """重置统计"""
        with self._lock:
            self.response_times.clear()
            self.request_counts.clear()
            self.success_counts.clear()
            self.error_counts.clear()
            self.cache_hits = 0
            self.cache_misses = 0
            self.start_time = datetime.now()

    def get_health_status(self) -> Dict[str, Any]:
        """
        获取健康状态

        Returns:
            健康状态字典
        """
        stats = self.get_stats()

        # 判断健康状态
        is_healthy = True
        warnings = []

        if stats['error_rate_percent'] > 5.0:
            is_healthy = False
            warnings.append(f"错误率过高: {stats['error_rate_percent']}%")

        if stats['p95_response_time'] > 1.0:
            warnings.append(f"P95响应时间过长: {stats['p95_response_time']}s")

        if stats['cache_hit_rate_percent'] < 50.0:
            warnings.append(f"缓存命中率过低: {stats['cache_hit_rate_percent']}%")

        if stats['active_connections'] > 100:
            warnings.append(f"活跃连接数过多: {stats['active_connections']}")

        status = 'healthy' if is_healthy else 'unhealthy'

        return {
            'status': status,
            'warnings': warnings,
            'metrics': stats
        }
