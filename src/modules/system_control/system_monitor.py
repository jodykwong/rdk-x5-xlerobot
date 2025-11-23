"""
系统监控模块 - Story 4.4
实现系统状态监控、资源跟踪、性能分析、告警机制
支持实时监控、告警触发、监控数据可视化
"""

import asyncio
import time
import psutil
import json
from typing import Any, Dict, List, Optional, Callable, Union
from dataclasses import dataclass, field
from enum import Enum
from collections import defaultdict, deque
import logging
import threading
from datetime import datetime, timedelta

logger = logging.getLogger(__name__)


class AlertLevel(Enum):
    """告警级别"""
    INFO = "info"
    WARNING = "warning"
    ERROR = "error"
    CRITICAL = "critical"


class MetricType(Enum):
    """指标类型"""
    CPU_USAGE = "cpu_usage"
    MEMORY_USAGE = "memory_usage"
    DISK_USAGE = "disk_usage"
    NETWORK_IO = "network_io"
    PROCESS_COUNT = "process_count"
    THREAD_COUNT = "thread_count"
    CUSTOM = "custom"


class SystemState(Enum):
    """系统状态"""
    HEALTHY = "healthy"
    DEGRADED = "degraded"
    UNHEALTHY = "unhealthy"
    CRITICAL = "critical"


@dataclass
class MetricData:
    """监控指标数据"""
    name: str
    value: float
    metric_type: MetricType
    timestamp: float = field(default_factory=time.time)
    tags: Dict[str, str] = field(default_factory=dict)
    unit: str = ""
    threshold: Optional[float] = None


@dataclass
class Alert:
    """告警信息"""
    id: str
    level: AlertLevel
    title: str
    message: str
    timestamp: float = field(default_factory=time.time)
    source: str = ""
    metric_name: Optional[str] = None
    metric_value: Optional[float] = None
    threshold: Optional[float] = None
    resolved: bool = False
    resolved_at: Optional[float] = None


@dataclass
class SystemStatus:
    """系统状态信息"""
    state: SystemState
    cpu_usage: float
    memory_usage: float
    disk_usage: float
    network_io: Dict[str, float]
    process_count: int
    thread_count: int
    timestamp: float = field(default_factory=time.time)
    details: Dict[str, Any] = field(default_factory=dict)


class SystemMonitor:
    """
    系统监控器
    提供全面的系统监控、告警、分析功能
    """

    def __init__(self,
                 collection_interval: float = 1.0,
                 history_size: int = 10000,
                 alert_check_interval: float = 0.5):
        # 监控配置
        self._collection_interval = collection_interval
        self._history_size = history_size
        self._alert_check_interval = alert_check_interval

        # 数据存储
        self._metrics: Dict[str, deque] = defaultdict(lambda: deque(maxlen=history_size))
        self._alerts: Dict[str, Alert] = {}
        self._alert_history: deque = deque(maxlen=1000)

        # 告警规则
        self._alert_rules: Dict[str, Dict[str, Any]] = {}

        # 系统信息
        self._cpu_count = psutil.cpu_count()
        self._boot_time = psutil.boot_time()
        self._disk_partitions = psutil.disk_partitions()

        # 控制标志
        self._running = False
        self._monitor_task: Optional[asyncio.Task] = None
        self._alert_task: Optional[asyncio.Task] = None
        self._lock = threading.RLock()

        # 回调函数
        self._alert_callbacks: List[Callable[[Alert], None]] = []
        self._status_callbacks: List[Callable[[SystemStatus], None]] = []

        # 统计信息
        self._stats = {
            'metrics_collected': 0,
            'alerts_triggered': 0,
            'alerts_resolved': 0,
            'monitoring_uptime': 0.0
        }

        # 设置默认告警规则
        self._setup_default_alert_rules()

        logger.info(
            "系统监控器初始化完成",
            cpu_count=self._cpu_count,
            collection_interval=collection_interval,
            history_size=history_size
        )

    def _setup_default_alert_rules(self):
        """设置默认告警规则"""
        self._alert_rules = {
            'cpu_usage': {
                'warning_threshold': 70.0,
                'critical_threshold': 90.0,
                'check_type': 'above'
            },
            'memory_usage': {
                'warning_threshold': 80.0,
                'critical_threshold': 95.0,
                'check_type': 'above'
            },
            'disk_usage': {
                'warning_threshold': 85.0,
                'critical_threshold': 95.0,
                'check_type': 'above'
            },
            'process_count': {
                'warning_threshold': 500,
                'critical_threshold': 1000,
                'check_type': 'above'
            }
        }

    async def start(self):
        """启动系统监控"""
        if not self._running:
            self._running = True
            self._start_time = time.time()

            # 启动监控任务
            self._monitor_task = asyncio.create_task(self._monitor_loop())
            self._alert_task = asyncio.create_task(self._alert_check_loop())

            logger.info("系统监控器已启动")

    async def stop(self):
        """停止系统监控"""
        if self._running:
            self._running = False

            # 取消任务
            if self._monitor_task:
                self._monitor_task.cancel()
                try:
                    await self._monitor_task
                except asyncio.CancelledError:
                    pass

            if self._alert_task:
                self._alert_task.cancel()
                try:
                    await self._alert_task
                except asyncio.CancelledError:
                    pass

            # 更新统计
            self._stats['monitoring_uptime'] = time.time() - self._start_time

            logger.info("系统监控器已停止")

    async def _monitor_loop(self):
        """监控循环"""
        while self._running:
            try:
                # 收集系统指标
                await self._collect_system_metrics()

                # 更新系统状态
                status = self.get_system_status()
                self._notify_status_callbacks(status)

                # 等待下次收集
                await asyncio.sleep(self._collection_interval)

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error("监控循环错误", error=str(e))
                await asyncio.sleep(1.0)

    async def _collect_system_metrics(self):
        """收集系统指标"""
        try:
            current_time = time.time()

            # CPU使用率
            cpu_percent = psutil.cpu_percent(interval=None)
            self._record_metric("cpu_usage", cpu_percent, MetricType.CPU_USAGE, "%")

            # CPU负载平均值
            load_avg = psutil.getloadavg()[0]  # 1分钟平均负载
            normalized_load = (load_avg / self._cpu_count) * 100
            self._record_metric("cpu_load_avg", normalized_load, MetricType.CPU_USAGE, "%")

            # 内存使用率
            memory = psutil.virtual_memory()
            self._record_metric("memory_usage", memory.percent, MetricType.MEMORY_USAGE, "%")
            self._record_metric("memory_available", memory.available / (1024**3), MetricType.MEMORY_USAGE, "GB")

            # 磁盘使用率
            disk_usage = psutil.disk_usage('/')
            disk_percent = (disk_usage.used / disk_usage.total) * 100
            self._record_metric("disk_usage", disk_percent, MetricType.DISK_USAGE, "%")
            self._record_metric("disk_free", disk_usage.free / (1024**3), MetricType.DISK_USAGE, "GB")

            # 网络IO
            net_io = psutil.net_io_counters()
            self._record_metric("network_bytes_sent", net_io.bytes_sent, MetricType.NETWORK_IO, "bytes")
            self._record_metric("network_bytes_recv", net_io.bytes_recv, MetricType.NETWORK_IO, "bytes")
            self._record_metric("network_packets_sent", net_io.packets_sent, MetricType.NETWORK_IO, "count")
            self._record_metric("network_packets_recv", net_io.packets_recv, MetricType.NETWORK_IO, "count")

            # 进程和线程数
            process_count = len(psutil.pids())
            self._record_metric("process_count", process_count, MetricType.PROCESS_COUNT, "count")

            # 获取总线程数
            total_threads = 0
            for proc in psutil.process_iter(['num_threads']):
                try:
                    total_threads += proc.info['num_threads'] or 0
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
            self._record_metric("thread_count", total_threads, MetricType.THREAD_COUNT, "count")

            # 更新统计
            self._stats['metrics_collected'] += 1

        except Exception as e:
            logger.error("收集系统指标失败", error=str(e))

    def _record_metric(self, name: str, value: float, metric_type: MetricType,
                      unit: str = "", tags: Optional[Dict[str, str]] = None):
        """记录指标"""
        metric = MetricData(
            name=name,
            value=value,
            metric_type=metric_type,
            unit=unit,
            tags=tags or {}
        )

        with self._lock:
            self._metrics[name].append(metric)

    async def _alert_check_loop(self):
        """告警检查循环"""
        while self._running:
            try:
                # 检查所有告警规则
                await self._check_alerts()

                await asyncio.sleep(self._alert_check_interval)

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error("告警检查循环错误", error=str(e))
                await asyncio.sleep(1.0)

    async def _check_alerts(self):
        """检查告警"""
        try:
            current_time = time.time()

            for metric_name, rule in self._alert_rules.items():
                if metric_name not in self._metrics:
                    continue

                # 获取最新指标值
                with self._lock:
                    if not self._metrics[metric_name]:
                        continue
                    latest_metric = self._metrics[metric_name][-1]
                    value = latest_metric.value
                    threshold = rule.get('critical_threshold')
                    check_type = rule.get('check_type', 'above')

                if threshold is None:
                    continue

                # 检查是否触发告警
                triggered = False
                alert_level = None

                if check_type == 'above':
                    if value >= rule.get('critical_threshold', 100):
                        triggered = True
                        alert_level = AlertLevel.CRITICAL
                    elif value >= rule.get('warning_threshold', 80):
                        triggered = True
                        alert_level = AlertLevel.WARNING

                # 如果触发了告警
                if triggered:
                    # 检查是否已有未解决的告警
                    existing_alert = self._find_active_alert(metric_name, alert_level)
                    if not existing_alert:
                        await self._trigger_alert(
                            metric_name=metric_name,
                            value=value,
                            threshold=threshold,
                            level=alert_level
                        )
                else:
                    # 检查是否可以解决现有告警
                    await self._resolve_alerts(metric_name)

        except Exception as e:
            logger.error("检查告警失败", error=str(e))

    def _find_active_alert(self, metric_name: str, level: AlertLevel) -> Optional[Alert]:
        """查找活跃告警"""
        with self._lock:
            for alert in self._alerts.values():
                if (not alert.resolved and
                    alert.metric_name == metric_name and
                    alert.level == level):
                    return alert
        return None

    async def _trigger_alert(self, metric_name: str, value: float,
                           threshold: float, level: AlertLevel):
        """触发告警"""
        alert_id = f"{metric_name}_{level.value}_{int(time.time())}"

        alert = Alert(
            id=alert_id,
            level=level,
            title=f"{metric_name} {level.value.upper()}",
            message=f"{metric_name} 当前值 {value:.2f} 超过阈值 {threshold:.2f}",
            metric_name=metric_name,
            metric_value=value,
            threshold=threshold
        )

        with self._lock:
            self._alerts[alert_id] = alert
            self._alert_history.append(alert)

        # 更新统计
        self._stats['alerts_triggered'] += 1

        # 通知回调
        self._notify_alert_callbacks(alert)

        logger.warning(
            "告警触发",
            alert_id=alert_id,
            metric=metric_name,
            value=value,
            threshold=threshold,
            level=level.value
        )

    async def _resolve_alerts(self, metric_name: str):
        """解决告警"""
        alerts_to_resolve = []

        with self._lock:
            for alert_id, alert in self._alerts.items():
                if not alert.resolved and alert.metric_name == metric_name:
                    alerts_to_resolve.append(alert_id)

        for alert_id in alerts_to_resolve:
            await self._resolve_alert(alert_id)

    async def _resolve_alert(self, alert_id: str):
        """解决单个告警"""
        if alert_id not in self._alerts:
            return

        alert = self._alerts[alert_id]
        alert.resolved = True
        alert.resolved_at = time.time()

        # 更新统计
        self._stats['alerts_resolved'] += 1

        logger.info(
            "告警已解决",
            alert_id=alert_id,
            metric=alert.metric_name,
            duration=alert.resolved_at - alert.timestamp
        )

    def get_system_status(self) -> SystemStatus:
        """
        获取系统状态

        Returns:
            SystemStatus: 系统状态信息
        """
        # 获取最新指标
        cpu_usage = self.get_metric_value("cpu_usage")
        memory_usage = self.get_metric_value("memory_usage")
        disk_usage = self.get_metric_value("disk_usage")
        process_count = self.get_metric_value("process_count")
        thread_count = self.get_metric_value("thread_count")

        # 获取网络IO
        net_bytes_sent = self.get_metric_value("network_bytes_sent")
        net_bytes_recv = self.get_metric_value("network_bytes_recv")

        # 确定系统状态
        state = self._determine_system_state(
            cpu_usage, memory_usage, disk_usage, process_count
        )

        return SystemStatus(
            state=state,
            cpu_usage=cpu_usage or 0.0,
            memory_usage=memory_usage or 0.0,
            disk_usage=disk_usage or 0.0,
            network_io={
                'bytes_sent': net_bytes_sent or 0.0,
                'bytes_recv': net_bytes_recv or 0.0
            },
            process_count=int(process_count or 0),
            thread_count=int(thread_count or 0),
            details={
                'cpu_count': self._cpu_count,
                'boot_time': self._boot_time,
                'disk_partitions': len(self._disk_partitions)
            }
        )

    def _determine_system_state(self, cpu: float, memory: float,
                               disk: float, processes: int) -> SystemState:
        """确定系统状态"""
        # 检查关键指标
        if cpu >= 95 or memory >= 95 or disk >= 95:
            return SystemState.CRITICAL
        elif cpu >= 85 or memory >= 90 or disk >= 90 or processes >= 800:
            return SystemState.UNHEALTHY
        elif cpu >= 70 or memory >= 80 or disk >= 85 or processes >= 500:
            return SystemState.DEGRADED
        else:
            return SystemState.HEALTHY

    def get_metric_value(self, metric_name: str,
                        aggregation: str = "latest") -> Optional[float]:
        """
        获取指标值

        Args:
            metric_name: 指标名称
            aggregation: 聚合方式 (latest, avg, min, max)

        Returns:
            Optional[float]: 指标值
        """
        with self._lock:
            if metric_name not in self._metrics or not self._metrics[metric_name]:
                return None

            metrics = list(self._metrics[metric_name])
            values = [m.value for m in metrics]

            if aggregation == "latest":
                return values[-1] if values else None
            elif aggregation == "avg":
                return sum(values) / len(values) if values else None
            elif aggregation == "min":
                return min(values) if values else None
            elif aggregation == "max":
                return max(values) if values else None
            else:
                return values[-1] if values else None

    def get_metric_history(self, metric_name: str,
                          duration: Optional[float] = None) -> List[MetricData]:
        """
        获取指标历史数据

        Args:
            metric_name: 指标名称
            duration: 时间范围（秒）

        Returns:
            List[MetricData]: 历史数据
        """
        with self._lock:
            if metric_name not in self._metrics:
                return []

            if duration is None:
                return list(self._metrics[metric_name])

            current_time = time.time()
            cutoff_time = current_time - duration

            return [m for m in self._metrics[metric_name]
                   if m.timestamp >= cutoff_time]

    def get_active_alerts(self, level: Optional[AlertLevel] = None) -> List[Alert]:
        """
        获取活跃告警

        Args:
            level: 告警级别过滤

        Returns:
            List[Alert]: 活跃告警列表
        """
        with self._lock:
            alerts = [alert for alert in self._alerts.values() if not alert.resolved]
            if level:
                alerts = [alert for alert in alerts if alert.level == level]
            return sorted(alerts, key=lambda a: a.timestamp, reverse=True)

    def get_alert_history(self, limit: int = 100) -> List[Alert]:
        """
        获取告警历史

        Args:
            limit: 返回数量限制

        Returns:
            List[Alert]: 告警历史列表
        """
        with self._lock:
            return list(self._alert_history)[-limit:]

    def add_alert_rule(self, metric_name: str, warning_threshold: float,
                      critical_threshold: float, check_type: str = "above"):
        """
        添加告警规则

        Args:
            metric_name: 指标名称
            warning_threshold: 警告阈值
            critical_threshold: 严重阈值
            check_type: 检查类型 (above, below)
        """
        self._alert_rules[metric_name] = {
            'warning_threshold': warning_threshold,
            'critical_threshold': critical_threshold,
            'check_type': check_type
        }

        logger.info(
            "告警规则已添加",
            metric=metric_name,
            warning=warning_threshold,
            critical=critical_threshold,
            check_type=check_type
        )

    def remove_alert_rule(self, metric_name: str):
        """删除告警规则"""
        if metric_name in self._alert_rules:
            del self._alert_rules[metric_name]
            logger.info("告警规则已删除", metric=metric_name)

    def register_alert_callback(self, callback: Callable[[Alert], None]):
        """注册告警回调"""
        self._alert_callbacks.append(callback)
        logger.info("告警回调已注册", callback=callback.__name__)

    def register_status_callback(self, callback: Callable[[SystemStatus], None]):
        """注册状态回调"""
        self._status_callbacks.append(callback)
        logger.info("状态回调已注册", callback=callback.__name__)

    def _notify_alert_callbacks(self, alert: Alert):
        """通知告警回调"""
        for callback in self._alert_callbacks:
            try:
                callback(alert)
            except Exception as e:
                logger.error("告警回调执行失败", callback=callback.__name__, error=str(e))

    def _notify_status_callbacks(self, status: SystemStatus):
        """通知状态回调"""
        for callback in self._status_callbacks:
            try:
                callback(status)
            except Exception as e:
                logger.error("状态回调执行失败", callback=callback.__name__, error=str(e))

    def get_stats(self) -> Dict[str, Any]:
        """
        获取监控器统计信息

        Returns:
            Dict: 统计信息
        """
        stats = self._stats.copy()
        stats['active_metrics'] = len(self._metrics)
        stats['active_alerts'] = len(self.get_active_alerts())
        stats['alert_rules_count'] = len(self._alert_rules)
        stats['system_state'] = self.get_system_status().state.value
        return stats

    async def export_metrics(self, format: str = "json") -> str:
        """
        导出监控指标

        Args:
            format: 导出格式 (json, csv)

        Returns:
            str: 导出的数据
        """
        try:
            with self._lock:
                data = {
                    'export_time': datetime.now().isoformat(),
                    'metrics': {},
                    'alerts': [self._alert_to_dict(alert) for alert in self._alert_history],
                    'stats': self._stats
                }

                for metric_name, metrics in self._metrics.items():
                    data['metrics'][metric_name] = [
                        {
                            'timestamp': m.timestamp,
                            'value': m.value,
                            'unit': m.unit,
                            'tags': m.tags
                        }
                        for m in metrics
                    ]

            if format == "json":
                return json.dumps(data, indent=2, default=str)
            else:
                # 简化的CSV导出
                lines = ["metric_name,timestamp,value,unit"]
                for metric_name, metrics in self._metrics.items():
                    for m in metrics:
                        lines.append(f"{metric_name},{m.timestamp},{m.value},{m.unit}")
                return "\n".join(lines)

        except Exception as e:
            logger.error("导出指标失败", error=str(e))
            raise

    def _alert_to_dict(self, alert: Alert) -> Dict[str, Any]:
        """告警转字典"""
        return {
            'id': alert.id,
            'level': alert.level.value,
            'title': alert.title,
            'message': alert.message,
            'timestamp': alert.timestamp,
            'source': alert.source,
            'metric_name': alert.metric_name,
            'metric_value': alert.metric_value,
            'threshold': alert.threshold,
            'resolved': alert.resolved,
            'resolved_at': alert.resolved_at
        }


if __name__ == "__main__":
    async def main():
        """测试系统监控器"""
        monitor = SystemMonitor(collection_interval=1.0)

        # 注册告警回调
        def alert_handler(alert: Alert):
            print(f"🚨 告警: {alert.title} - {alert.message}")

        monitor.register_alert_callback(alert_handler)

        # 注册状态回调
        def status_handler(status: SystemStatus):
            print(f"系统状态: {status.state.value}, CPU: {status.cpu_usage:.1f}%, "
                  f"内存: {status.memory_usage:.1f}%, 磁盘: {status.disk_usage:.1f}%")

        monitor.register_status_callback(status_handler)

        # 启动监控
        await monitor.start()

        # 运行一段时间
        await asyncio.sleep(10)

        # 获取统计信息
        stats = monitor.get_stats()
        print(f"\n监控器统计: {stats}")

        # 导出指标
        metrics_json = await monitor.export_metrics("json")
        print(f"\n指标数据: {metrics_json[:500]}...")

        # 停止监控
        await monitor.stop()

    asyncio.run(main())
