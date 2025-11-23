"""
资源监控器 - Story 4.4
实现CPU、内存、磁盘、网络等资源的细粒度监控
支持资源使用趋势分析、预测告警、资源优化建议
"""

import asyncio
import time
import psutil
import json
from typing import Any, Dict, List, Optional, Tuple
from dataclasses import dataclass, field
from enum import Enum
from collections import deque, defaultdict
import logging

logger = logging.getLogger(__name__)


class ResourceType(Enum):
    """资源类型"""
    CPU = "cpu"
    MEMORY = "memory"
    DISK = "disk"
    NETWORK = "network"
    PROCESS = "process"


@dataclass
class ResourceUsage:
    """资源使用情况"""
    resource_type: ResourceType
    usage_percent: float
    total: float
    used: float
    free: float
    timestamp: float = field(default_factory=time.time)
    details: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ProcessResource:
    """进程资源使用"""
    pid: int
    name: str
    cpu_percent: float
    memory_percent: float
    memory_mb: float
    num_threads: int
    num_fds: int
    create_time: float
    timestamp: float = field(default_factory=time.time)


@dataclass
class ResourceAlert:
    """资源告警"""
    resource_type: ResourceType
    current_usage: float
    threshold: float
    severity: str  # warning, critical
    message: str
    timestamp: float = field(default_factory=time.time)
    pid: Optional[int] = None
    process_name: Optional[str] = None


class ResourceMonitor:
    """
    资源监控器
    提供全面的系统资源监控和分析功能
    """

    def __init__(self,
                 collection_interval: float = 2.0,
                 history_size: int = 1000,
                 top_process_count: int = 10):
        # 配置
        self._collection_interval = collection_interval
        self._history_size = history_size
        self._top_process_count = top_process_count

        # 数据存储
        self._resource_history: Dict[ResourceType, deque] = {
            resource_type: deque(maxlen=history_size)
            for resource_type in ResourceType
        }
        self._process_history: deque = deque(maxlen=history_size)
        self._current_processes: Dict[int, ProcessResource] = {}
        self._resource_alerts: deque = deque(maxlen=100)

        # 告警配置
        self._alert_thresholds = {
            ResourceType.CPU: {'warning': 70.0, 'critical': 90.0},
            ResourceType.MEMORY: {'warning': 80.0, 'critical': 95.0},
            ResourceType.DISK: {'warning': 85.0, 'critical': 95.0},
            ResourceType.NETWORK: {'warning': 80.0, 'critical': 95.0},
            ResourceType.PROCESS: {'warning': 80.0, 'critical': 95.0}
        }

        # 控制标志
        self._running = False
        self._monitor_task: Optional[asyncio.Task] = None
        self._lock = asyncio.Lock()

        # 回调函数
        self._alert_callbacks: List[Callable[[ResourceAlert], None]] = []

        # 统计信息
        self._stats = {
            'collections_performed': 0,
            'alerts_generated': 0,
            'average_collection_time_ms': 0.0
        }

        logger.info("资源监控器初始化完成", collection_interval=collection_interval)

    async def start(self):
        """启动资源监控"""
        if not self._running:
            self._running = True
            self._start_time = time.time()

            self._monitor_task = asyncio.create_task(self._monitor_loop())

            logger.info("资源监控器已启动")

    async def stop(self):
        """停止资源监控"""
        if self._running:
            self._running = False

            if self._monitor_task:
                self._monitor_task.cancel()
                try:
                    await self._monitor_task
                except asyncio.CancelledError:
                    pass

            self._stats['monitoring_uptime'] = time.time() - self._start_time

            logger.info("资源监控器已停止")

    async def _monitor_loop(self):
        """监控循环"""
        while self._running:
            try:
                start_time = time.perf_counter()

                # 收集资源使用情况
                await self._collect_resource_usage()

                # 收集进程信息
                await self._collect_process_info()

                # 检查告警
                await self._check_resource_alerts()

                # 更新统计
                collection_time_ms = (time.perf_counter() - start_time) * 1000
                self._update_stats(collection_time_ms)

                await asyncio.sleep(self._collection_interval)

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error("资源监控循环错误", error=str(e))
                await asyncio.sleep(1.0)

    async def _collect_resource_usage(self):
        """收集资源使用情况"""
        try:
            current_time = time.time()

            # CPU使用率
            cpu_percent = psutil.cpu_percent(interval=None)
            cpu_usage = ResourceUsage(
                resource_type=ResourceType.CPU,
                usage_percent=cpu_percent,
                total=self._get_cpu_count(),
                used=cpu_percent,
                free=100.0 - cpu_percent,
                details={
                    'load_average': psutil.getloadavg()[0] if self._get_cpu_count() > 0 else 0,
                    'per_cpu': psutil.cpu_percent(interval=None, percpu=True)
                }
            )
            self._resource_history[ResourceType.CPU].append(cpu_usage)

            # 内存使用情况
            memory = psutil.virtual_memory()
            memory_usage = ResourceUsage(
                resource_type=ResourceType.MEMORY,
                usage_percent=memory.percent,
                total=memory.total,
                used=memory.used,
                free=memory.available,
                details={
                    'cached': memory.cached,
                    'buffers': memory.buffers,
                    'shared': getattr(memory, 'shared', 0)
                }
            )
            self._resource_history[ResourceType.MEMORY].append(memory_usage)

            # 磁盘使用情况
            disk_usage = psutil.disk_usage('/')
            disk_percent = (disk_usage.used / disk_usage.total) * 100
            disk_io = psutil.disk_io_counters()
            disk_resource_usage = ResourceUsage(
                resource_type=ResourceType.DISK,
                usage_percent=disk_percent,
                total=disk_usage.total,
                used=disk_usage.used,
                free=disk_usage.free,
                details={
                    'read_bytes': disk_io.read_bytes if disk_io else 0,
                    'write_bytes': disk_io.write_bytes if disk_io else 0,
                    'read_count': disk_io.read_count if disk_io else 0,
                    'write_count': disk_io.write_count if disk_io else 0
                }
            )
            self._resource_history[ResourceType.DISK].append(disk_resource_usage)

            # 网络使用情况
            net_io = psutil.net_io_counters()
            total_bytes = net_io.bytes_sent + net_io.bytes_recv
            max_bytes = 100 * 1024 * 1024  # 假设100Mbps为最大值
            network_percent = min((total_bytes / max_bytes) * 100, 100.0)
            network_usage = ResourceUsage(
                resource_type=ResourceType.NETWORK,
                usage_percent=network_percent,
                total=max_bytes,
                used=total_bytes,
                free=max_bytes - total_bytes,
                details={
                    'bytes_sent': net_io.bytes_sent,
                    'bytes_recv': net_io.bytes_recv,
                    'packets_sent': net_io.packets_sent,
                    'packets_recv': net_io.packets_recv,
                    'errin': net_io.errin,
                    'errout': net_io.errout
                }
            )
            self._resource_history[ResourceType.NETWORK].append(network_usage)

            logger.debug("资源使用情况已收集")

        except Exception as e:
            logger.error("收集资源使用情况失败", error=str(e))

    async def _collect_process_info(self):
        """收集进程信息"""
        try:
            processes = []
            current_time = time.time()

            # 获取所有进程
            for proc in psutil.process_iter(['pid', 'name', 'cpu_percent', 'memory_percent',
                                            'memory_info', 'num_threads', 'num_fds',
                                            'create_time']):
                try:
                    pinfo = proc.info
                    pid = pinfo['pid']

                    # 获取内存MB
                    memory_mb = pinfo['memory_info'].rss / (1024 * 1024) if pinfo['memory_info'] else 0

                    process_resource = ProcessResource(
                        pid=pid,
                        name=pinfo['name'],
                        cpu_percent=pinfo['cpu_percent'] or 0.0,
                        memory_percent=pinfo['memory_percent'] or 0.0,
                        memory_mb=memory_mb,
                        num_threads=pinfo['num_threads'] or 0,
                        num_fds=pinfo['num_fds'] or 0,
                        create_time=pinfo['create_time']
                    )

                    processes.append(process_resource)
                    self._current_processes[pid] = process_resource

                except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                    continue

            # 按CPU使用率排序，获取TOP进程
            top_processes = sorted(processes, key=lambda p: p.cpu_percent, reverse=True)[:self._top_process_count]

            # 记录当前快照
            self._process_history.append({
                'timestamp': current_time,
                'processes': top_processes,
                'total_process_count': len(processes)
            })

            logger.debug(f"进程信息已收集: {len(processes)} 个进程")

        except Exception as e:
            logger.error("收集进程信息失败", error=str(e))

    async def _check_resource_alerts(self):
        """检查资源告警"""
        try:
            current_time = time.time()

            # 检查每种资源的告警
            for resource_type, history in self._resource_history.items():
                if not history:
                    continue

                latest_usage = history[-1]
                usage_percent = latest_usage.usage_percent

                # 检查告警阈值
                thresholds = self._alert_thresholds.get(resource_type, {})
                warning_threshold = thresholds.get('warning')
                critical_threshold = thresholds.get('critical')

                # 生成告警
                if critical_threshold and usage_percent >= critical_threshold:
                    alert = ResourceAlert(
                        resource_type=resource_type,
                        current_usage=usage_percent,
                        threshold=critical_threshold,
                        severity='critical',
                        message=f"{resource_type.value} 使用率过高: {usage_percent:.1f}%"
                    )
                    self._resource_alerts.append(alert)
                    self._notify_alert_callbacks(alert)
                    self._stats['alerts_generated'] += 1

                elif warning_threshold and usage_percent >= warning_threshold:
                    alert = ResourceAlert(
                        resource_type=resource_type,
                        current_usage=usage_percent,
                        threshold=warning_threshold,
                        severity='warning',
                        message=f"{resource_type.value} 使用率偏高: {usage_percent:.1f}%"
                    )
                    self._resource_alerts.append(alert)
                    self._notify_alert_callbacks(alert)
                    self._stats['alerts_generated'] += 1

            # 检查高CPU使用的进程
            for pid, process in self._current_processes.items():
                if process.cpu_percent >= 50.0:  # CPU使用率超过50%的进程
                    alert = ResourceAlert(
                        resource_type=ResourceType.PROCESS,
                        current_usage=process.cpu_percent,
                        threshold=50.0,
                        severity='warning',
                        message=f"进程 {process.name} (PID: {pid}) CPU使用率过高: {process.cpu_percent:.1f}%",
                        pid=pid,
                        process_name=process.name
                    )
                    self._resource_alerts.append(alert)
                    self._notify_alert_callbacks(alert)
                    self._stats['alerts_generated'] += 1

        except Exception as e:
            logger.error("检查资源告警失败", error=str(e))

    def _get_cpu_count(self) -> int:
        """获取CPU核心数"""
        return psutil.cpu_count(logical=True) or 1

    def _update_stats(self, collection_time_ms: float):
        """更新统计信息"""
        self._stats['collections_performed'] += 1

        if self._stats['average_collection_time_ms'] == 0:
            self._stats['average_collection_time_ms'] = collection_time_ms
        else:
            alpha = 0.1
            self._stats['average_collection_time_ms'] = (
                (1 - alpha) * self._stats['average_collection_time_ms'] +
                alpha * collection_time_ms
            )

    def get_current_resource_usage(self, resource_type: ResourceType) -> Optional[ResourceUsage]:
        """获取当前资源使用情况"""
        history = self._resource_history.get(resource_type)
        if history and history:
            return history[-1]
        return None

    def get_resource_usage_history(self, resource_type: ResourceType,
                                  duration: Optional[float] = None) -> List[ResourceUsage]:
        """获取资源使用历史"""
        history = self._resource_history.get(resource_type, deque())
        if not history:
            return []

        if duration is None:
            return list(history)

        current_time = time.time()
        cutoff_time = current_time - duration

        return [usage for usage in history if usage.timestamp >= cutoff_time]

    def get_top_processes(self, sort_by: str = 'cpu_percent',
                         limit: int = 10) -> List[ProcessResource]:
        """获取TOP进程列表"""
        if not self._process_history:
            return []

        latest_snapshot = self._process_history[-1]
        processes = latest_snapshot.get('processes', [])

        # 排序
        if sort_by == 'cpu_percent':
            processes = sorted(processes, key=lambda p: p.cpu_percent, reverse=True)
        elif sort_by == 'memory_percent':
            processes = sorted(processes, key=lambda p: p.memory_percent, reverse=True)
        elif sort_by == 'memory_mb':
            processes = sorted(processes, key=lambda p: p.memory_mb, reverse=True)

        return processes[:limit]

    def get_resource_trends(self, resource_type: ResourceType,
                           duration: float = 300.0) -> Dict[str, float]:
        """获取资源使用趋势分析"""
        history = self.get_resource_usage_history(resource_type, duration)
        if len(history) < 2:
            return {}

        values = [usage.usage_percent for usage in history]

        # 计算趋势
        if len(values) >= 3:
            # 简单线性回归计算趋势
            n = len(values)
            x = list(range(n))
            x_mean = sum(x) / n
            y_mean = sum(values) / n

            numerator = sum((x[i] - x_mean) * (values[i] - y_mean) for i in range(n))
            denominator = sum((x[i] - x_mean) ** 2 for i in range(n))

            if denominator != 0:
                slope = numerator / denominator
            else:
                slope = 0
        else:
            slope = values[-1] - values[0]

        return {
            'current': values[-1] if values else 0.0,
            'average': sum(values) / len(values) if values else 0.0,
            'min': min(values) if values else 0.0,
            'max': max(values) if values else 0.0,
            'trend': 'increasing' if slope > 0.5 else 'decreasing' if slope < -0.5 else 'stable',
            'slope': slope
        }

    def get_resource_alerts(self, limit: int = 20) -> List[ResourceAlert]:
        """获取资源告警"""
        return list(self._resource_alerts)[-limit:]

    def get_process_count(self) -> int:
        """获取当前进程数量"""
        return len(self._current_processes)

    def find_process_by_name(self, name: str) -> List[ProcessResource]:
        """根据进程名查找进程"""
        return [proc for proc in self._current_processes.values()
                if name.lower() in proc.name.lower()]

    def get_process_by_pid(self, pid: int) -> Optional[ProcessResource]:
        """根据PID获取进程信息"""
        return self._current_processes.get(pid)

    def set_alert_threshold(self, resource_type: ResourceType,
                           warning: float, critical: float):
        """设置告警阈值"""
        self._alert_thresholds[resource_type] = {
            'warning': warning,
            'critical': critical
        }
        logger.info(
            "告警阈值已更新",
            resource=resource_type.value,
            warning=warning,
            critical=critical
        )

    def register_alert_callback(self, callback):
        """注册告警回调"""
        self._alert_callbacks.append(callback)
        logger.info("告警回调已注册", callback=callback.__name__)

    def _notify_alert_callbacks(self, alert: ResourceAlert):
        """通知告警回调"""
        for callback in self._alert_callbacks:
            try:
                callback(alert)
            except Exception as e:
                logger.error("告警回调执行失败", callback=callback.__name__, error=str(e))

    def get_stats(self) -> Dict[str, Any]:
        """获取统计信息"""
        stats = self._stats.copy()
        stats['resource_types_monitored'] = len(self._resource_history)
        stats['current_processes'] = len(self._current_processes)
        stats['active_alerts'] = len([a for a in self._resource_alerts
                                     if time.time() - a.timestamp < 300])  # 最近5分钟
        return stats

    async def export_data(self, format: str = 'json') -> str:
        """导出监控数据"""
        try:
            data = {
                'export_time': time.time(),
                'resource_usage': {},
                'process_snapshots': list(self._process_history),
                'alerts': [self._alert_to_dict(alert) for alert in self._resource_alerts],
                'stats': self._stats
            }

            # 导出资源使用情况
            for resource_type, history in self._resource_history.items():
                data['resource_usage'][resource_type.value] = [
                    {
                        'timestamp': usage.timestamp,
                        'usage_percent': usage.usage_percent,
                        'total': usage.total,
                        'used': usage.used,
                        'free': usage.free,
                        'details': usage.details
                    }
                    for usage in history
                ]

            if format == 'json':
                return json.dumps(data, indent=2, default=str)
            else:
                return str(data)

        except Exception as e:
            logger.error("导出数据失败", error=str(e))
            raise

    def _alert_to_dict(self, alert: ResourceAlert) -> Dict[str, Any]:
        """告警转字典"""
        return {
            'resource_type': alert.resource_type.value,
            'current_usage': alert.current_usage,
            'threshold': alert.threshold,
            'severity': alert.severity,
            'message': alert.message,
            'timestamp': alert.timestamp,
            'pid': alert.pid,
            'process_name': alert.process_name
        }


if __name__ == "__main__":
    async def main():
        """测试资源监控器"""
        monitor = ResourceMonitor(collection_interval=1.0)

        # 注册告警回调
        def alert_handler(alert: ResourceAlert):
            print(f"🚨 资源告警 [{alert.severity}]: {alert.message}")

        monitor.register_alert_callback(alert_handler)

        # 启动监控
        await monitor.start()

        # 运行一段时间
        await asyncio.sleep(10)

        # 获取当前资源使用情况
        cpu_usage = monitor.get_current_resource_usage(ResourceType.CPU)
        print(f"\n当前CPU使用率: {cpu_usage.usage_percent:.1f}%" if cpu_usage else "无CPU数据")

        memory_usage = monitor.get_current_resource_usage(ResourceType.MEMORY)
        print(f"当前内存使用率: {memory_usage.usage_percent:.1f}%" if memory_usage else "无内存数据")

        # 获取TOP进程
        top_processes = monitor.get_top_processes('cpu_percent', 5)
        print("\nTOP 5 CPU使用进程:")
        for proc in top_processes:
            print(f"  - {proc.name} (PID: {proc.pid}): CPU {proc.cpu_percent:.1f}%, "
                  f"内存 {proc.memory_mb:.1f}MB")

        # 获取资源趋势
        cpu_trends = monitor.get_resource_trends(ResourceType.CPU, 60.0)
        print(f"\nCPU趋势分析: {cpu_trends}")

        # 停止监控
        await monitor.stop()

    asyncio.run(main())
