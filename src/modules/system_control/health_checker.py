"""
系统健康检查器 - Story 4.4
提供快速系统健康检查、基础监控、资源预警功能
轻量级、低开销的系统状态监控
"""

import asyncio
import time
import psutil
from typing import Any, Dict, List, Optional, Callable, Union
from dataclasses import dataclass, field
from enum import Enum
import logging

logger = logging.getLogger(__name__)


class HealthStatus(Enum):
    """健康状态"""
    HEALTHY = "healthy"
    WARNING = "warning"
    CRITICAL = "critical"
    UNKNOWN = "unknown"


@dataclass
class HealthCheck:
    """健康检查项"""
    name: str
    status: HealthStatus
    value: Any
    threshold: Any
    message: str
    duration_ms: float = 0.0
    timestamp: float = field(default_factory=time.time)
    details: Dict[str, Any] = field(default_factory=dict)


@dataclass
class SystemHealth:
    """系统健康信息"""
    overall_status: HealthStatus
    score: float  # 0-100 健康分数
    checks: List[HealthCheck]
    timestamp: float = field(default_factory=time.time)
    uptime_seconds: float = 0.0


class HealthChecker:
    """
    系统健康检查器
    提供快速、低开销的系统健康检查
    适用于资源受限环境或高频健康检查场景
    """

    def __init__(self,
                 check_interval: float = 5.0,
                 cpu_threshold: float = 80.0,
                 memory_threshold: float = 85.0,
                 disk_threshold: float = 90.0,
                 load_threshold: float = 2.0):
        # 配置
        self._check_interval = check_interval
        self._cpu_threshold = cpu_threshold
        self._memory_threshold = memory_threshold
        self._disk_threshold = disk_threshold
        self._load_threshold = load_threshold

        # 系统信息
        self._cpu_count = psutil.cpu_count()
        self._boot_time = psutil.boot_time()

        # 控制标志
        self._running = False
        self._check_task: Optional[asyncio.Task] = None
        self._lock = asyncio.Lock()

        # 回调函数
        self._health_callbacks: List[Callable[[SystemHealth], None]] = []

        # 缓存的系统健康信息
        self._last_health: Optional[SystemHealth] = None

        # 统计信息
        self._stats = {
            'checks_performed': 0,
            'checks_failed': 0,
            'average_check_time_ms': 0.0
        }

        logger.info(
            "系统健康检查器初始化完成",
            cpu_count=self._cpu_count,
            check_interval=check_interval
        )

    async def start(self):
        """启动健康检查"""
        if not self._running:
            self._running = True
            self._start_time = time.time()

            # 启动健康检查任务
            self._check_task = asyncio.create_task(self._check_loop())

            logger.info("系统健康检查器已启动")

    async def stop(self):
        """停止健康检查"""
        if self._running:
            self._running = False

            if self._check_task:
                self._check_task.cancel()
                try:
                    await self._check_task
                except asyncio.CancelledError:
                    pass

            logger.info("系统健康检查器已停止")

    async def _check_loop(self):
        """健康检查循环"""
        while self._running:
            try:
                start_time = time.perf_counter()

                # 执行健康检查
                health = await self.check_system_health()

                # 更新缓存
                async with self._lock:
                    self._last_health = health

                # 通知回调
                self._notify_health_callbacks(health)

                # 更新统计
                check_time_ms = (time.perf_counter() - start_time) * 1000
                self._update_stats(check_time_ms)

                # 等待下次检查
                await asyncio.sleep(self._check_interval)

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error("健康检查循环错误", error=str(e))
                await asyncio.sleep(1.0)

    async def check_system_health(self) -> SystemHealth:
        """
        执行系统健康检查

        Returns:
            SystemHealth: 系统健康信息
        """
        checks = []
        start_time = time.perf_counter()

        try:
            # CPU使用率检查
            cpu_check = await self._check_cpu_usage()
            checks.append(cpu_check)

            # 内存使用率检查
            memory_check = await self._check_memory_usage()
            checks.append(memory_check)

            # 磁盘使用率检查
            disk_check = await self._check_disk_usage()
            checks.append(disk_check)

            # 系统负载检查
            load_check = await self._check_system_load()
            checks.append(load_check)

            # 进程数量检查
            process_check = await self._check_process_count()
            checks.append(process_check)

            # 线程数量检查
            thread_check = await self._check_thread_count()
            checks.append(thread_check)

            # 计算总体健康状态和分数
            overall_status, score = self._calculate_overall_health(checks)

            # 计算系统运行时间
            uptime_seconds = time.time() - self._boot_time

            return SystemHealth(
                overall_status=overall_status,
                score=score,
                checks=checks,
                uptime_seconds=uptime_seconds
            )

        except Exception as e:
            logger.error("系统健康检查失败", error=str(e))

            # 返回错误状态
            return SystemHealth(
                overall_status=HealthStatus.CRITICAL,
                score=0.0,
                checks=checks,
                uptime_seconds=time.time() - self._boot_time
            )

    async def _check_cpu_usage(self) -> HealthCheck:
        """检查CPU使用率"""
        start_time = time.perf_counter()

        try:
            cpu_percent = psutil.cpu_percent(interval=None)

            if cpu_percent >= 90:
                status = HealthStatus.CRITICAL
                message = f"CPU使用率过高: {cpu_percent:.1f}%"
            elif cpu_percent >= self._cpu_threshold:
                status = HealthStatus.WARNING
                message = f"CPU使用率偏高: {cpu_percent:.1f}%"
            else:
                status = HealthStatus.HEALTHY
                message = f"CPU使用率正常: {cpu_percent:.1f}%"

            duration_ms = (time.perf_counter() - start_time) * 1000

            return HealthCheck(
                name="cpu_usage",
                status=status,
                value=cpu_percent,
                threshold=self._cpu_threshold,
                message=message,
                duration_ms=duration_ms,
                details={
                    'cpu_count': self._cpu_count,
                    'load_average': psutil.getloadavg()[0] if self._cpu_count > 0 else 0
                }
            )

        except Exception as e:
            duration_ms = (time.perf_counter() - start_time) * 1000
            return HealthCheck(
                name="cpu_usage",
                status=HealthStatus.UNKNOWN,
                value=None,
                threshold=self._cpu_threshold,
                message=f"CPU检查失败: {str(e)}",
                duration_ms=duration_ms
            )

    async def _check_memory_usage(self) -> HealthCheck:
        """检查内存使用率"""
        start_time = time.perf_counter()

        try:
            memory = psutil.virtual_memory()

            if memory.percent >= 95:
                status = HealthStatus.CRITICAL
                message = f"内存使用率过高: {memory.percent:.1f}%"
            elif memory.percent >= self._memory_threshold:
                status = HealthStatus.WARNING
                message = f"内存使用率偏高: {memory.percent:.1f}%"
            else:
                status = HealthStatus.HEALTHY
                message = f"内存使用率正常: {memory.percent:.1f}%"

            duration_ms = (time.perf_counter() - start_time) * 1000

            return HealthCheck(
                name="memory_usage",
                status=status,
                value=memory.percent,
                threshold=self._memory_threshold,
                message=message,
                duration_ms=duration_ms,
                details={
                    'total_gb': memory.total / (1024**3),
                    'available_gb': memory.available / (1024**3),
                    'used_gb': memory.used / (1024**3),
                    'free_gb': memory.free / (1024**3)
                }
            )

        except Exception as e:
            duration_ms = (time.perf_counter() - start_time) * 1000
            return HealthCheck(
                name="memory_usage",
                status=HealthStatus.UNKNOWN,
                value=None,
                threshold=self._memory_threshold,
                message=f"内存检查失败: {str(e)}",
                duration_ms=duration_ms
            )

    async def _check_disk_usage(self) -> HealthCheck:
        """检查磁盘使用率"""
        start_time = time.perf_counter()

        try:
            disk_usage = psutil.disk_usage('/')
            disk_percent = (disk_usage.used / disk_usage.total) * 100

            if disk_percent >= 95:
                status = HealthStatus.CRITICAL
                message = f"磁盘使用率过高: {disk_percent:.1f}%"
            elif disk_percent >= self._disk_threshold:
                status = HealthStatus.WARNING
                message = f"磁盘使用率偏高: {disk_percent:.1f}%"
            else:
                status = HealthStatus.HEALTHY
                message = f"磁盘使用率正常: {disk_percent:.1f}%"

            duration_ms = (time.perf_counter() - start_time) * 1000

            return HealthCheck(
                name="disk_usage",
                status=status,
                value=disk_percent,
                threshold=self._disk_threshold,
                message=message,
                duration_ms=duration_ms,
                details={
                    'total_gb': disk_usage.total / (1024**3),
                    'used_gb': disk_usage.used / (1024**3),
                    'free_gb': disk_usage.free / (1024**3),
                    'percent': disk_percent
                }
            )

        except Exception as e:
            duration_ms = (time.perf_counter() - start_time) * 1000
            return HealthCheck(
                name="disk_usage",
                status=HealthStatus.UNKNOWN,
                value=None,
                threshold=self._disk_threshold,
                message=f"磁盘检查失败: {str(e)}",
                duration_ms=duration_ms
            )

    async def _check_system_load(self) -> HealthCheck:
        """检查系统负载"""
        start_time = time.perf_counter()

        try:
            load_avg = psutil.getloadavg()[0]  # 1分钟平均负载
            normalized_load = load_avg / self._cpu_count if self._cpu_count > 0 else 0

            if normalized_load >= 2.0:
                status = HealthStatus.CRITICAL
                message = f"系统负载过高: {normalized_load:.2f}"
            elif normalized_load >= self._load_threshold:
                status = HealthStatus.WARNING
                message = f"系统负载偏高: {normalized_load:.2f}"
            else:
                status = HealthStatus.HEALTHY
                message = f"系统负载正常: {normalized_load:.2f}"

            duration_ms = (time.perf_counter() - start_time) * 1000

            return HealthCheck(
                name="system_load",
                status=status,
                value=normalized_load,
                threshold=self._load_threshold,
                message=message,
                duration_ms=duration_ms,
                details={
                    'load_avg_1min': load_avg,
                    'load_avg_5min': psutil.getloadavg()[1],
                    'load_avg_15min': psutil.getloadavg()[2],
                    'cpu_count': self._cpu_count
                }
            )

        except Exception as e:
            duration_ms = (time.perf_counter() - start_time) * 1000
            return HealthCheck(
                name="system_load",
                status=HealthStatus.UNKNOWN,
                value=None,
                threshold=self._load_threshold,
                message=f"系统负载检查失败: {str(e)}",
                duration_ms=duration_ms
            )

    async def _check_process_count(self) -> HealthCheck:
        """检查进程数量"""
        start_time = time.perf_counter()

        try:
            process_count = len(psutil.pids())

            if process_count >= 1000:
                status = HealthStatus.CRITICAL
                message = f"进程数量过多: {process_count}"
            elif process_count >= 500:
                status = HealthStatus.WARNING
                message = f"进程数量偏多: {process_count}"
            else:
                status = HealthStatus.HEALTHY
                message = f"进程数量正常: {process_count}"

            duration_ms = (time.perf_counter() - start_time) * 1000

            return HealthCheck(
                name="process_count",
                status=status,
                value=process_count,
                threshold=500,
                message=message,
                duration_ms=duration_ms
            )

        except Exception as e:
            duration_ms = (time.perf_counter() - start_time) * 1000
            return HealthCheck(
                name="process_count",
                status=HealthStatus.UNKNOWN,
                value=None,
                threshold=500,
                message=f"进程数量检查失败: {str(e)}",
                duration_ms=duration_ms
            )

    async def _check_thread_count(self) -> HealthCheck:
        """检查线程数量"""
        start_time = time.perf_counter()

        try:
            # 获取总线程数
            total_threads = 0
            for proc in psutil.process_iter(['num_threads']):
                try:
                    threads = proc.info['num_threads']
                    if threads:
                        total_threads += threads
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass

            if total_threads >= 5000:
                status = HealthStatus.CRITICAL
                message = f"线程数量过多: {total_threads}"
            elif total_threads >= 3000:
                status = HealthStatus.WARNING
                message = f"线程数量偏多: {total_threads}"
            else:
                status = HealthStatus.HEALTHY
                message = f"线程数量正常: {total_threads}"

            duration_ms = (time.perf_counter() - start_time) * 1000

            return HealthCheck(
                name="thread_count",
                status=status,
                value=total_threads,
                threshold=3000,
                message=message,
                duration_ms=duration_ms
            )

        except Exception as e:
            duration_ms = (time.perf_counter() - start_time) * 1000
            return HealthCheck(
                name="thread_count",
                status=HealthStatus.UNKNOWN,
                value=None,
                threshold=3000,
                message=f"线程数量检查失败: {str(e)}",
                duration_ms=duration_ms
            )

    def _calculate_overall_health(self, checks: List[HealthCheck]) -> tuple[HealthStatus, float]:
        """计算总体健康状态和分数"""
        if not checks:
            return HealthStatus.UNKNOWN, 0.0

        # 计算分数
        total_score = 0.0
        critical_count = 0
        warning_count = 0

        for check in checks:
            if check.status == HealthStatus.CRITICAL:
                total_score += 0.0
                critical_count += 1
            elif check.status == HealthStatus.WARNING:
                total_score += 50.0
                warning_count += 1
            elif check.status == HealthStatus.HEALTHY:
                total_score += 100.0
            else:  # UNKNOWN
                total_score += 75.0

        # 计算平均分数
        avg_score = total_score / len(checks)

        # 确定总体状态
        if critical_count > 0:
            overall_status = HealthStatus.CRITICAL
        elif warning_count > 0:
            overall_status = HealthStatus.WARNING
        else:
            overall_status = HealthStatus.HEALTHY

        return overall_status, avg_score

    def _update_stats(self, check_time_ms: float):
        """更新统计信息"""
        self._stats['checks_performed'] += 1

        if self._stats['average_check_time_ms'] == 0:
            self._stats['average_check_time_ms'] = check_time_ms
        else:
            # 移动平均
            alpha = 0.1
            self._stats['average_check_time_ms'] = (
                (1 - alpha) * self._stats['average_check_time_ms'] +
                alpha * check_time_ms
            )

    async def get_current_health(self) -> Optional[SystemHealth]:
        """获取当前健康状态"""
        async with self._lock:
            return self._last_health

    def quick_health_check(self) -> SystemHealth:
        """
        快速健康检查（同步方法）
        适用于不需要持续监控的场景

        Returns:
            SystemHealth: 系统健康信息
        """
        # 同步执行健康检查
        checks = []

        # CPU
        cpu_percent = psutil.cpu_percent(interval=None)
        cpu_status = HealthStatus.WARNING if cpu_percent >= 80 else HealthStatus.HEALTHY
        checks.append(HealthCheck(
            name="cpu_usage",
            status=cpu_status,
            value=cpu_percent,
            threshold=80.0,
            message=f"CPU: {cpu_percent:.1f}%"
        ))

        # 内存
        memory = psutil.virtual_memory()
        memory_status = HealthStatus.WARNING if memory.percent >= 85 else HealthStatus.HEALTHY
        checks.append(HealthCheck(
            name="memory_usage",
            status=memory_status,
            value=memory.percent,
            threshold=85.0,
            message=f"内存: {memory.percent:.1f}%"
        ))

        # 磁盘
        disk_usage = psutil.disk_usage('/')
        disk_percent = (disk_usage.used / disk_usage.total) * 100
        disk_status = HealthStatus.WARNING if disk_percent >= 90 else HealthStatus.HEALTHY
        checks.append(HealthCheck(
            name="disk_usage",
            status=disk_status,
            value=disk_percent,
            threshold=90.0,
            message=f"磁盘: {disk_percent:.1f}%"
        ))

        # 计算总体状态
        overall_status, score = self._calculate_overall_health(checks)

        return SystemHealth(
            overall_status=overall_status,
            score=score,
            checks=checks,
            uptime_seconds=time.time() - self._boot_time
        )

    def register_health_callback(self, callback: Callable[[SystemHealth], None]):
        """注册健康状态回调"""
        self._health_callbacks.append(callback)
        logger.info("健康状态回调已注册", callback=callback.__name__)

    def _notify_health_callbacks(self, health: SystemHealth):
        """通知健康状态回调"""
        for callback in self._health_callbacks:
            try:
                callback(health)
            except Exception as e:
                logger.error("健康状态回调执行失败", callback=callback.__name__, error=str(e))

    def get_stats(self) -> Dict[str, Any]:
        """获取统计信息"""
        stats = self._stats.copy()
        stats['cpu_count'] = self._cpu_count
        stats['boot_time'] = self._boot_time
        stats['running'] = self._running
        return stats


if __name__ == "__main__":
    async def main():
        """测试健康检查器"""
        checker = HealthChecker(check_interval=2.0)

        # 注册健康状态回调
        def health_handler(health: SystemHealth):
            print(f"健康状态: {health.overall_status.value}, "
                  f"分数: {health.score:.1f}/100")
            for check in health.checks:
                print(f"  - {check.name}: {check.status.value} - {check.message}")

        checker.register_health_callback(health_handler)

        # 启动健康检查
        await checker.start()

        # 运行一段时间
        await asyncio.sleep(10)

        # 快速健康检查
        print("\n快速健康检查:")
        quick_health = checker.quick_health_check()
        print(f"总体状态: {quick_health.overall_status.value}")
        print(f"健康分数: {quick_health.score:.1f}/100")

        # 停止检查
        await checker.stop()

    asyncio.run(main())
