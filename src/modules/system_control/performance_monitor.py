"""
性能监控器 - Story 4.2
实现性能指标采集和监控
支持<10ms延迟要求、吞吐量监控、实时性能分析
"""

import asyncio
import time
import psutil
import threading
from typing import Any, Dict, List, Optional, Callable, Union
from dataclasses import dataclass, field
from enum import Enum
from collections import deque, defaultdict
import logging

import statistics

logger = logging.getLogger(__name__)


class MetricType(Enum):
    """指标类型"""
    COUNTER = "counter"          # 计数器
    GAUGE = "gauge"              # 仪表盘
    HISTOGRAM = "histogram"      # 直方图
    TIMER = "timer"              # 计时器


class AlertLevel(Enum):
    """告警级别"""
    INFO = "info"
    WARNING = "warning"
    ERROR = "error"
    CRITICAL = "critical"


@dataclass
class Metric:
    """性能指标"""
    name: str
    type: MetricType
    value: float
    unit: str = ""
    timestamp: float = field(default_factory=time.time)
    labels: Dict[str, str] = field(default_factory=dict)
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class PerformanceThreshold:
    """性能阈值"""
    metric_name: str
    warning_threshold: Optional[float] = None
    critical_threshold: Optional[float] = None
    comparison_operator: str = ">"  # >, <, >=, <=, ==
    aggregation: str = "avg"  # avg, max, min, p95, p99


@dataclass
class Alert:
    """性能告警"""
    id: str
    metric_name: str
    level: AlertLevel
    message: str
    value: float
    threshold: float
    timestamp: float = field(default_factory=time.time)
    resolved: bool = False
    resolved_at: Optional[float] = None


class PerformanceMonitor:
    """
    性能监控器
    采集和监控性能指标，确保<10ms延迟要求
    支持实时性能分析、告警和报告
    """

    def __init__(self, buffer_size: int = 10000):
        # 指标存储
        self._metrics: Dict[str, deque] = defaultdict(lambda: deque(maxlen=buffer_size))
        self._metric_values: Dict[str, float] = {}

        # 阈值和告警
        self._thresholds: Dict[str, PerformanceThreshold] = {}
        self._alerts: Dict[str, Alert] = {}
        self._alert_handlers: List[Callable] = []

        # 系统监控
        self._system_monitor_active = False
        self._monitoring_interval = 1.0  # 秒

        # 统计信息
        self._stats = {
            'total_metrics_collected': 0,
            'total_alerts_triggered': 0,
            'monitoring_uptime': 0.0,
            'average_latency': 0.0,
            'peak_latency': 0.0,
            'messages_per_second': 0.0
        }

        # 任务存储
        self._tasks: List[asyncio.Task] = []
        self._running = False

        # 消息计数
        self._message_count = 0
        self._message_count_lock = threading.Lock()

        logger.info("性能监控器初始化完成", buffer_size=buffer_size)

    async def start(self):
        """启动性能监控器"""
        if not self._running:
            self._running = True

            # 启动后台任务
            self._tasks.append(asyncio.create_task(self._alert_monitor()))
            self._tasks.append(asyncio.create_task(self._stats_updater()))

            # 启动系统监控
            if not self._system_monitor_active:
                self._system_monitor_active = True
                self._tasks.append(asyncio.create_task(self._system_monitor()))

            logger.info("性能监控器已启动")

    async def stop(self):
        """停止性能监控器"""
        if self._running:
            self._running = False

            # 取消所有任务
            for task in self._tasks:
                task.cancel()

            # 等待任务完成
            if self._tasks:
                await asyncio.gather(*self._tasks, return_exceptions=True)

            self._system_monitor_active = False

            logger.info("性能监控器已停止")

    async def record_metric(self, metric: Union[Metric, dict, str],
                           value: Optional[float] = None,
                           metric_type: Optional[MetricType] = None,
                           unit: str = "",
                           labels: Optional[Dict[str, str]] = None,
                           increment: bool = False):
        """
        记录性能指标

        Args:
            metric: 指标对象、指标名称或指标字典
            value: 指标值
            metric_type: 指标类型
            unit: 单位
            labels: 标签
            increment: 是否增量（用于计数器）
        """
        try:
            # 创建指标对象
            if isinstance(metric, Metric):
                m = metric
            elif isinstance(metric, dict):
                m = Metric(**metric)
            elif isinstance(metric, str):
                if value is None:
                    raise ValueError(f"指标 {metric} 需要指定值")
                m = Metric(
                    name=metric,
                    type=metric_type or MetricType.GAUGE,
                    value=value,
                    unit=unit,
                    labels=labels or {}
                )
            else:
                raise ValueError(f"不支持的指标类型: {type(metric)}")

            # 处理增量
            if increment and m.type == MetricType.COUNTER:
                current_value = self._metric_values.get(m.name, 0.0)
                m.value = current_value + m.value

            # 添加到存储
            self._metrics[m.name].append(m)
            self._metric_values[m.name] = m.value

            # 更新统计
            self._stats['total_metrics_collected'] += 1

            # 检查阈值
            await self._check_threshold(m)

            logger.debug(
                "指标已记录",
                name=m.name,
                value=m.value,
                type=m.type.value,
                unit=m.unit
            )

        except Exception as e:
            logger.error("记录指标失败", error=str(e))

    async def record_latency(self, operation: str, latency_ms: float,
                           labels: Optional[Dict[str, str]] = None):
        """
        记录延迟指标（专门用于<10ms要求监控）

        Args:
            operation: 操作名称
            latency_ms: 延迟（毫秒）
            labels: 标签
        """
        metric_name = f"latency.{operation}"
        await self.record_metric(
            metric=metric_name,
            value=latency_ms,
            metric_type=MetricType.TIMER,
            unit="ms",
            labels=labels
        )

        # 更新延迟统计
        self._update_latency_stats(latency_ms)

        # 检查<10ms要求
        if latency_ms >= 10.0:
            await self._trigger_alert(
                metric_name=metric_name,
                level=AlertLevel.WARNING,
                message=f"操作 {operation} 延迟超标: {latency_ms:.2f}ms (要求 <10ms)",
                value=latency_ms,
                threshold=10.0
            )

        logger.debug(
            "延迟指标已记录",
            operation=operation,
            latency_ms=latency_ms,
            meets_requirement=latency_ms < 10.0
        )

    async def record_message(self, message_type: str = "generic",
                           size_bytes: Optional[int] = None):
        """
        记录消息指标

        Args:
            message_type: 消息类型
            size_bytes: 消息大小（字节）
        """
        with self._message_count_lock:
            self._message_count += 1

        await self.record_metric(
            metric=f"message.count.{message_type}",
            value=1,
            metric_type=MetricType.COUNTER,
            labels={"message_type": message_type},
            increment=True
        )

        if size_bytes:
            await self.record_metric(
                metric=f"message.size.{message_type}",
                value=size_bytes,
                metric_type=MetricType.GAUGE,
                unit="bytes",
                labels={"message_type": message_type}
            )

        logger.debug("消息指标已记录", message_type=message_type, size_bytes=size_bytes)

    def set_threshold(self, threshold: PerformanceThreshold):
        """
        设置性能阈值

        Args:
            threshold: 性能阈值对象
        """
        self._thresholds[threshold.metric_name] = threshold
        logger.info(
            "性能阈值已设置",
            metric=threshold.metric_name,
            warning=threshold.warning_threshold,
            critical=threshold.critical_threshold
        )

    async def _check_threshold(self, metric: Metric):
        """检查指标阈值"""
        threshold = self._thresholds.get(metric.name)
        if not threshold:
            return

        try:
            # 获取当前值
            current_value = metric.value

            # 检查警告阈值
            if threshold.warning_threshold is not None:
                if self._compare_threshold(current_value, threshold.warning_threshold, threshold.comparison_operator):
                    await self._trigger_alert(
                        metric_name=metric.name,
                        level=AlertLevel.WARNING,
                        message=f"指标 {metric.name} 达到警告阈值: {current_value:.2f}",
                        value=current_value,
                        threshold=threshold.warning_threshold
                    )

            # 检查严重阈值
            if threshold.critical_threshold is not None:
                if self._compare_threshold(current_value, threshold.critical_threshold, threshold.comparison_operator):
                    await self._trigger_alert(
                        metric_name=metric.name,
                        level=AlertLevel.CRITICAL,
                        message=f"指标 {metric.name} 达到严重阈值: {current_value:.2f}",
                        value=current_value,
                        threshold=threshold.critical_threshold
                    )

        except Exception as e:
            logger.error("检查阈值失败", metric=metric.name, error=str(e))

    def _compare_threshold(self, value: float, threshold: float, operator: str) -> bool:
        """比较阈值"""
        if operator == ">":
            return value > threshold
        elif operator == "<":
            return value < threshold
        elif operator == ">=":
            return value >= threshold
        elif operator == "<=":
            return value <= threshold
        elif operator == "==":
            return abs(value - threshold) < 0.001  # 浮点数比较
        else:
            logger.error("未知的比较运算符", operator=operator)
            return False

    async def _trigger_alert(self, metric_name: str, level: AlertLevel,
                            message: str, value: float, threshold: float):
        """触发告警"""
        try:
            alert = Alert(
                id=f"{metric_name}:{time.time()}",
                metric_name=metric_name,
                level=level,
                message=message,
                value=value,
                threshold=threshold
            )

            self._alerts[alert.id] = alert
            self._stats['total_alerts_triggered'] += 1

            # 发送告警通知
            for handler in self._alert_handlers:
                try:
                    if asyncio.iscoroutinefunction(handler):
                        await handler(alert)
                    else:
                        handler(alert)
                except Exception as e:
                    logger.error("告警处理器失败", error=str(e))

            logger.warning(
                "告警已触发",
                alert_id=alert.id,
                metric=metric_name,
                level=level.value,
                message=message
            )

        except Exception as e:
            logger.error("触发告警失败", error=str(e))

    async def _alert_monitor(self):
        """告警监控任务"""
        while self._running:
            try:
                # 检查未解决的告警
                current_time = time.time()
                for alert in list(self._alerts.values()):
                    if not alert.resolved:
                        # 检查是否需要自动解决（例如指标恢复正常）
                        threshold = self._thresholds.get(alert.metric_name)
                        if threshold:
                            current_value = self._metric_values.get(alert.metric_name, 0)
                            if not self._compare_threshold(current_value, threshold.warning_threshold, threshold.comparison_operator):
                                alert.resolved = True
                                alert.resolved_at = current_time

                                logger.info(
                                    "告警已自动解决",
                                    alert_id=alert.id,
                                    metric=alert.metric_name
                                )

                await asyncio.sleep(5.0)  # 每5秒检查一次

            except Exception as e:
                logger.error("告警监控错误", error=str(e))
                await asyncio.sleep(5)

    async def _system_monitor(self):
        """系统监控任务"""
        while self._running and self._system_monitor_active:
            try:
                # CPU使用率
                cpu_percent = psutil.cpu_percent(interval=None)
                await self.record_metric(
                    metric="system.cpu.usage",
                    value=cpu_percent,
                    metric_type=MetricType.GAUGE,
                    unit="percent"
                )

                # 内存使用率
                memory = psutil.virtual_memory()
                await self.record_metric(
                    metric="system.memory.usage",
                    value=memory.percent,
                    metric_type=MetricType.GAUGE,
                    unit="percent"
                )

                # 内存使用量
                await self.record_metric(
                    metric="system.memory.bytes",
                    value=memory.used,
                    metric_type=MetricType.GAUGE,
                    unit="bytes"
                )

                # 磁盘使用率
                disk = psutil.disk_usage('/')
                disk_percent = (disk.used / disk.total) * 100
                await self.record_metric(
                    metric="system.disk.usage",
                    value=disk_percent,
                    metric_type=MetricType.GAUGE,
                    unit="percent"
                )

                # 网络IO
                net_io = psutil.net_io_counters()
                await self.record_metric(
                    metric="system.network.bytes_sent",
                    value=net_io.bytes_sent,
                    metric_type=MetricType.COUNTER,
                    unit="bytes"
                )
                await self.record_metric(
                    metric="system.network.bytes_recv",
                    value=net_io.bytes_recv,
                    metric_type=MetricType.COUNTER,
                    unit="bytes"
                )

                await asyncio.sleep(self._monitoring_interval)

            except Exception as e:
                logger.error("系统监控错误", error=str(e))
                await asyncio.sleep(self._monitoring_interval)

    async def _stats_updater(self):
        """统计更新任务"""
        while self._running:
            try:
                self._stats['monitoring_uptime'] += 1.0

                # 计算消息每秒吞吐量
                with self._message_count_lock:
                    messages = self._message_count
                    self._message_count = 0

                self._stats['messages_per_second'] = messages / self._monitoring_interval

                await asyncio.sleep(self._monitoring_interval)

            except Exception as e:
                logger.error("统计更新错误", error=str(e))
                await asyncio.sleep(self._monitoring_interval)

    def _update_latency_stats(self, latency_ms: float):
        """更新延迟统计"""
        # 更新平均延迟
        if self._stats['average_latency'] == 0:
            self._stats['average_latency'] = latency_ms
        else:
            alpha = 0.1
            self._stats['average_latency'] = (
                (1 - alpha) * self._stats['average_latency'] +
                alpha * latency_ms
            )

        # 更新峰值延迟
        if latency_ms > self._stats['peak_latency']:
            self._stats['peak_latency'] = latency_ms

    def get_current_metrics(self, metric_name: Optional[str] = None) -> Dict[str, float]:
        """
        获取当前指标值

        Args:
            metric_name: 指标名称，None表示获取所有

        Returns:
            Dict: 指标值字典
        """
        if metric_name:
            return {metric_name: self._metric_values.get(metric_name, 0.0)}
        else:
            return self._metric_values.copy()

    def get_metric_history(self, metric_name: str,
                          limit: Optional[int] = None) -> List[Metric]:
        """
        获取指标历史

        Args:
            metric_name: 指标名称
            limit: 返回数量限制

        Returns:
            List[Metric]: 指标历史列表
        """
        if metric_name not in self._metrics:
            return []

        metrics = list(self._metrics[metric_name])
        if limit:
            metrics = metrics[-limit:]

        return metrics

    def get_metric_stats(self, metric_name: str) -> Dict[str, float]:
        """
        获取指标统计信息

        Args:
            metric_name: 指标名称

        Returns:
            Dict: 统计信息
        """
        if metric_name not in self._metrics:
            return {}

        metrics = list(self._metrics[metric_name])
        values = [m.value for m in metrics]

        if not values:
            return {}

        return {
            'count': len(values),
            'min': min(values),
            'max': max(values),
            'avg': statistics.mean(values),
            'median': statistics.median(values),
            'p95': self._percentile(values, 95),
            'p99': self._percentile(values, 99),
            'latest': values[-1] if values else 0.0
        }

    def _percentile(self, values: List[float], percentile: float) -> float:
        """计算百分位数"""
        if not values:
            return 0.0

        sorted_values = sorted(values)
        index = (percentile / 100.0) * (len(sorted_values) - 1)

        if index.is_integer():
            return sorted_values[int(index)]
        else:
            lower = sorted_values[int(index)]
            upper = sorted_values[int(index) + 1]
            return lower + (upper - lower) * (index - int(index))

    def get_alerts(self, resolved: Optional[bool] = None) -> List[Alert]:
        """
        获取告警列表

        Args:
            resolved: 解决状态过滤

        Returns:
            List[Alert]: 告警列表
        """
        alerts = list(self._alerts.values())

        if resolved is not None:
            alerts = [a for a in alerts if a.resolved == resolved]

        return sorted(alerts, key=lambda a: a.timestamp, reverse=True)

    def register_alert_handler(self, handler: Callable):
        """
        注册告警处理器

        Args:
            handler: 告警处理函数
        """
        self._alert_handlers.append(handler)
        logger.info("告警处理器已注册")

    def get_stats(self) -> Dict[str, Any]:
        """
        获取性能监控统计信息

        Returns:
            Dict: 统计信息
        """
        stats = self._stats.copy()
        stats['active_metrics'] = len(self._metric_values)
        stats['configured_thresholds'] = len(self._thresholds)
        stats['active_alerts'] = sum(1 for a in self._alerts.values() if not a.resolved)
        stats['total_alerts'] = len(self._alerts)
        return stats

    def reset_stats(self):
        """重置统计信息"""
        self._stats = {
            'total_metrics_collected': 0,
            'total_alerts_triggered': 0,
            'monitoring_uptime': 0.0,
            'average_latency': 0.0,
            'peak_latency': 0.0,
            'messages_per_second': 0.0
        }

        logger.info("性能监控统计已重置")


if __name__ == "__main__":
    async def main():
        """测试性能监控器功能"""
        monitor = PerformanceMonitor()
        await monitor.start()

        # 测试记录指标
        await monitor.record_metric(
            metric="test.counter",
            value=10,
            metric_type=MetricType.COUNTER,
            increment=True
        )

        await monitor.record_latency("message_queue", 5.5)
        await monitor.record_message("test_message", size_bytes=1024)

        # 设置阈值
        threshold = PerformanceThreshold(
            metric_name="latency.message_queue",
            warning_threshold=8.0,
            critical_threshold=12.0,
            comparison_operator=">"
        )
        monitor.set_threshold(threshold)

        # 注册告警处理器
        def alert_handler(alert: Alert):
            print(f"告警: {alert.level.value} - {alert.message}")

        monitor.register_alert_handler(alert_handler)

        # 等待一段时间
        await asyncio.sleep(3)

        # 获取统计信息
        stats = monitor.get_stats()
        print(f"监控器统计: {stats}")

        # 获取当前指标
        current_metrics = monitor.get_current_metrics()
        print(f"当前指标: {current_metrics}")

        # 获取告警
        alerts = monitor.get_alerts()
        print(f"告警列表: {alerts}")

        # 关闭
        await monitor.stop()

    asyncio.run(main())
