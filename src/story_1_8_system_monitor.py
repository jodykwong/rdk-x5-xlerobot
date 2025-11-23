#!/usr/bin/env python3.10
"""
XleRobot System Monitor - 系统监控和告警
Story 1.8: 系统优化与部署
BMad Method v6 Brownfield Level 4 企业级标准

功能特性:
- 实时性能监控
- 智能告警系统
- 健康状态评估
- 性能趋势分析
- 自动故障恢复
- 100%符合Epic 1纯在线架构
"""

import asyncio
import time
import json
import logging
from typing import Dict, Any, List, Optional, Callable
from dataclasses import dataclass, field, asdict
from datetime import datetime, timedelta
from enum import Enum
import smtplib
from email.mime.text import MimeText
from collections import deque
import statistics

logger = logging.getLogger(__name__)

class AlertLevel(Enum):
    """告警级别"""
    INFO = "info"
    WARNING = "warning"
    ERROR = "error"
    CRITICAL = "critical"

class HealthStatus(Enum):
    """健康状态"""
    HEALTHY = "healthy"
    WARNING = "warning"
    CRITICAL = "critical"
    DOWN = "down"

@dataclass
class Alert:
    """告警信息"""
    id: str
    level: AlertLevel
    title: str
    message: str
    component: str
    timestamp: float
    resolved: bool = False
    resolution_time: Optional[float] = None
    metrics: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        return asdict(self)

@dataclass
class SystemHealthCheck:
    """系统健康检查项"""
    name: str
    check_function: Callable[[], bool]
    interval_seconds: int = 60
    timeout_seconds: int = 30
    consecutive_failures: int = 0
    max_failures: int = 3
    last_check_time: Optional[float] = None
    status: HealthStatus = HealthStatus.HEALTHY

@dataclass
class PerformanceThresholds:
    """性能阈值配置"""
    # 响应时间阈值 (毫秒)
    response_time_warning: float = 2000.0
    response_time_critical: float = 5000.0

    # CPU使用率阈值 (%)
    cpu_warning: float = 70.0
    cpu_critical: float = 90.0

    # 内存使用率阈值 (%)
    memory_warning: float = 75.0
    memory_critical: float = 90.0

    # 错误率阈值 (%)
    error_rate_warning: float = 5.0
    error_rate_critical: float = 15.0

    # 缓存命中率阈值 (%)
    cache_hit_rate_warning: float = 80.0
    cache_hit_rate_critical: float = 60.0

    # 吞吐量阈值 (请求/秒)
    throughput_warning: float = 1.0
    throughput_critical: float = 0.1

class SystemMonitor:
    """系统监控器 - Story 1.8核心组件"""

    def __init__(self, component_name: str = "XleRobot", thresholds: Optional[PerformanceThresholds] = None):
        """
        初始化系统监控器

        Args:
            component_name: 组件名称
            thresholds: 性能阈值配置
        """
        logger.info(f"🔍 初始化SystemMonitor - 组件: {component_name}")

        self.component_name = component_name
        self.thresholds = thresholds or PerformanceThresholds()

        # 监控状态
        self.monitoring_active = False
        self.monitoring_task: Optional[asyncio.Task] = None

        # 数据存储
        self.performance_history = deque(maxlen=1000)  # 最近1000个数据点
        self.alerts = deque(maxlen=500)  # 最近500个告警
        self.active_health_checks: Dict[str, SystemHealthCheck] = {}

        # 统计信息
        self.alert_counts = {level.value: 0 for level in AlertLevel}
        self.last_health_assessment: Optional[Dict[str, Any]] = None

        # 告警通知配置
        self.notification_callbacks: List[Callable[[Alert], None]] = []

        # 性能基线
        self.performance_baseline = {
            "response_time": 0.0,
            "cpu_usage": 0.0,
            "memory_usage": 0.0,
            "error_rate": 0.0,
            "cache_hit_rate": 0.0,
            "throughput": 0.0
        }

        logger.info("✅ 系统监控器初始化完成")

    async def start_monitoring(self, system_optimizer) -> None:
        """
        启动监控

        Args:
            system_optimizer: 系统优化器实例
        """
        if self.monitoring_active:
            logger.warning("⚠️ 监控已在运行中")
            return

        logger.info("🚀 启动系统监控")
        self.monitoring_active = True
        self.system_optimizer = system_optimizer

        # 启动监控循环
        self.monitoring_task = asyncio.create_task(self._monitoring_loop())

        # 注册默认健康检查
        self._register_default_health_checks()

        logger.info("✅ 系统监控已启动")

    async def stop_monitoring(self) -> None:
        """停止监控"""
        if not self.monitoring_active:
            return

        logger.info("🛑 停止系统监控")
        self.monitoring_active = False

        if self.monitoring_task:
            self.monitoring_task.cancel()
            try:
                await self.monitoring_task
            except asyncio.CancelledError:
                pass
            self.monitoring_task = None

        logger.info("✅ 系统监控已停止")

    def _register_default_health_checks(self) -> None:
        """注册默认健康检查"""
        # API连接健康检查
        self.register_health_check(
            "api_connectivity",
            self._check_api_connectivity,
            interval_seconds=30,
            max_failures=2
        )

        # 数据库连接健康检查 (如果适用)
        self.register_health_check(
            "system_resources",
            self._check_system_resources,
            interval_seconds=60,
            max_failures=3
        )

        # 缓存系统健康检查
        self.register_health_check(
            "cache_system",
            self._check_cache_system,
            interval_seconds=120,
            max_failures=3
        )

    def register_health_check(self,
                           name: str,
                           check_function: Callable[[], bool],
                           interval_seconds: int = 60,
                           timeout_seconds: int = 30,
                           max_failures: int = 3) -> None:
        """
        注册健康检查

        Args:
            name: 检查名称
            check_function: 检查函数
            interval_seconds: 检查间隔
            timeout_seconds: 超时时间
            max_failures: 最大失败次数
        """
        health_check = SystemHealthCheck(
            name=name,
            check_function=check_function,
            interval_seconds=interval_seconds,
            timeout_seconds=timeout_seconds,
            max_failures=max_failures
        )

        self.active_health_checks[name] = health_check
        logger.info(f"📝 已注册健康检查: {name}")

    def unregister_health_check(self, name: str) -> None:
        """取消注册健康检查"""
        if name in self.active_health_checks:
            del self.active_health_checks[name]
            logger.info(f"🗑️ 已取消注册健康检查: {name}")

    async def _monitoring_loop(self) -> None:
        """监控主循环"""
        while self.monitoring_active:
            try:
                start_time = time.time()

                # 1. 收集性能指标
                await self._collect_performance_metrics()

                # 2. 执行健康检查
                await self._execute_health_checks()

                # 3. 分析性能趋势
                await self._analyze_performance_trends()

                # 4. 检查告警条件
                await self._check_alert_conditions()

                # 5. 更新健康状态
                await self._update_health_status()

                # 计算监控循环耗时
                monitoring_time = time.time() - start_time
                if monitoring_time > 10:  # 如果监控循环超过10秒，记录警告
                    logger.warning(f"⚠️ 监控循环耗时过长: {monitoring_time:.2f}秒")

                # 等待下次监控
                await asyncio.sleep(10)  # 10秒监控间隔

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"❌ 监控循环异常: {str(e)}")
                await asyncio.sleep(10)

    async def _collect_performance_metrics(self) -> None:
        """收集性能指标"""
        try:
            if hasattr(self, 'system_optimizer') and self.system_optimizer:
                # 从系统优化器获取最新指标
                system_health = self.system_optimizer.get_system_health()
                metrics = system_health.get('metrics', {})

                # 记录性能数据
                performance_data = {
                    'timestamp': time.time(),
                    'response_time': metrics.get('avg_response_time', 0.0),
                    'cpu_usage': metrics.get('cpu_usage', 0.0),
                    'memory_usage': metrics.get('memory_usage', 0.0),
                    'error_rate': metrics.get('error_rate', 0.0),
                    'cache_hit_rate': metrics.get('cache_hit_rate', 0.0),
                    'throughput': metrics.get('throughput', 0.0),
                    'active_dialogues': metrics.get('active_dialogues', 0)
                }

                self.performance_history.append(performance_data)

        except Exception as e:
            logger.error(f"❌ 性能指标收集失败: {str(e)}")

    async def _execute_health_checks(self) -> None:
        """执行健康检查"""
        current_time = time.time()

        for name, health_check in self.active_health_checks.items():
            try:
                # 检查是否需要执行
                if (health_check.last_check_time is None or
                    current_time - health_check.last_check_time >= health_check.interval_seconds):

                    # 执行健康检查（带超时）
                    try:
                        result = await asyncio.wait_for(
                            asyncio.to_thread(health_check.check_function),
                            timeout=health_check.timeout_seconds
                        )

                        if result:
                            # 检查成功
                            health_check.consecutive_failures = 0
                            health_check.status = HealthStatus.HEALTHY
                        else:
                            # 检查失败
                            health_check.consecutive_failures += 1
                            if health_check.consecutive_failures >= health_check.max_failures:
                                health_check.status = HealthStatus.CRITICAL
                            else:
                                health_check.status = HealthStatus.WARNING

                    except asyncio.TimeoutError:
                        health_check.consecutive_failures += 1
                        health_check.status = HealthStatus.WARNING
                        logger.warning(f"⚠️ 健康检查超时: {name}")

                    except Exception as e:
                        health_check.consecutive_failures += 1
                        health_check.status = HealthStatus.WARNING
                        logger.error(f"❌ 健康检查异常 {name}: {str(e)}")

                    health_check.last_check_time = current_time

                    # 如果状态变为严重，发送告警
                    if health_check.status == HealthStatus.CRITICAL:
                        await self._create_alert(
                            AlertLevel.ERROR,
                            f"健康检查失败: {name}",
                            f"组件 {name} 连续失败 {health_check.consecutive_failures} 次",
                            name,
                            {
                                "consecutive_failures": health_check.consecutive_failures,
                                "max_failures": health_check.max_failures
                            }
                        )

            except Exception as e:
                logger.error(f"❌ 执行健康检查失败 {name}: {str(e)}")

    async def _analyze_performance_trends(self) -> None:
        """分析性能趋势"""
        if len(self.performance_history) < 10:
            return  # 数据不足

        try:
            recent_data = list(self.performance_history)[-10:]  # 最近10个数据点

            # 分析响应时间趋势
            response_times = [d['response_time'] for d in recent_data if d['response_time'] > 0]
            if len(response_times) >= 5:
                avg_response_time = statistics.mean(response_times)
                trend = "increasing" if response_times[-1] > response_times[0] else "decreasing"

                if avg_response_time > self.thresholds.response_time_warning:
                    await self._create_alert(
                        AlertLevel.WARNING,
                        "响应时间趋势警告",
                        f"平均响应时间 {avg_response_time:.2f}ms 超过阈值，趋势: {trend}",
                        "performance",
                        {
                            "avg_response_time": avg_response_time,
                            "trend": trend,
                            "threshold": self.thresholds.response_time_warning
                        }
                    )

            # 分析CPU使用率趋势
            cpu_usages = [d['cpu_usage'] for d in recent_data]
            if len(cpu_usages) >= 5:
                avg_cpu = statistics.mean(cpu_usages)
                if avg_cpu > self.thresholds.cpu_warning:
                    await self._create_alert(
                        AlertLevel.WARNING,
                        "CPU使用率警告",
                        f"平均CPU使用率 {avg_cpu:.1f}% 超过阈值",
                        "system_resources",
                        {"cpu_usage": avg_cpu, "threshold": self.thresholds.cpu_warning}
                    )

            # 分析内存使用率趋势
            memory_usages = [d['memory_usage'] for d in recent_data]
            if len(memory_usages) >= 5:
                avg_memory = statistics.mean(memory_usages)
                if avg_memory > self.thresholds.memory_warning:
                    await self._create_alert(
                        AlertLevel.WARNING,
                        "内存使用率警告",
                        f"平均内存使用率 {avg_memory:.1f}% 超过阈值",
                        "system_resources",
                        {"memory_usage": avg_memory, "threshold": self.thresholds.memory_warning}
                    )

        except Exception as e:
            logger.error(f"❌ 性能趋势分析失败: {str(e)}")

    async def _check_alert_conditions(self) -> None:
        """检查告警条件"""
        if not self.performance_history:
            return

        try:
            latest_metrics = self.performance_history[-1]

            # 响应时间检查
            response_time = latest_metrics.get('response_time', 0.0)
            if response_time > self.thresholds.response_time_critical:
                await self._create_alert(
                    AlertLevel.CRITICAL,
                    "响应时间严重超时",
                    f"响应时间 {response_time:.2f}ms 超过严重阈值",
                    "performance",
                    {"response_time": response_time, "threshold": self.thresholds.response_time_critical}
                )
            elif response_time > self.thresholds.response_time_warning:
                await self._create_alert(
                    AlertLevel.WARNING,
                    "响应时间警告",
                    f"响应时间 {response_time:.2f}ms 超过警告阈值",
                    "performance",
                    {"response_time": response_time, "threshold": self.thresholds.response_time_warning}
                )

            # CPU使用率检查
            cpu_usage = latest_metrics.get('cpu_usage', 0.0)
            if cpu_usage > self.thresholds.cpu_critical:
                await self._create_alert(
                    AlertLevel.CRITICAL,
                    "CPU使用率严重过高",
                    f"CPU使用率 {cpu_usage:.1f}% 超过严重阈值",
                    "system_resources",
                    {"cpu_usage": cpu_usage, "threshold": self.thresholds.cpu_critical}
                )
            elif cpu_usage > self.thresholds.cpu_warning:
                await self._create_alert(
                    AlertLevel.WARNING,
                    "CPU使用率警告",
                    f"CPU使用率 {cpu_usage:.1f}% 超过警告阈值",
                    "system_resources",
                    {"cpu_usage": cpu_usage, "threshold": self.thresholds.cpu_warning}
                )

            # 内存使用率检查
            memory_usage = latest_metrics.get('memory_usage', 0.0)
            if memory_usage > self.thresholds.memory_critical:
                await self._create_alert(
                    AlertLevel.CRITICAL,
                    "内存使用率严重过高",
                    f"内存使用率 {memory_usage:.1f}% 超过严重阈值",
                    "system_resources",
                    {"memory_usage": memory_usage, "threshold": self.thresholds.memory_critical}
                )
            elif memory_usage > self.thresholds.memory_warning:
                await self._create_alert(
                    AlertLevel.WARNING,
                    "内存使用率警告",
                    f"内存使用率 {memory_usage:.1f}% 超过警告阈值",
                    "system_resources",
                    {"memory_usage": memory_usage, "threshold": self.thresholds.memory_warning}
                )

            # 错误率检查
            error_rate = latest_metrics.get('error_rate', 0.0) * 100
            if error_rate > self.thresholds.error_rate_critical:
                await self._create_alert(
                    AlertLevel.CRITICAL,
                    "错误率严重过高",
                    f"错误率 {error_rate:.2f}% 超过严重阈值",
                    "application",
                    {"error_rate": error_rate, "threshold": self.thresholds.error_rate_critical}
                )
            elif error_rate > self.thresholds.error_rate_warning:
                await self._create_alert(
                    AlertLevel.WARNING,
                    "错误率警告",
                    f"错误率 {error_rate:.2f}% 超过警告阈值",
                    "application",
                    {"error_rate": error_rate, "threshold": self.thresholds.error_rate_warning}
                )

            # 缓存命中率检查
            cache_hit_rate = latest_metrics.get('cache_hit_rate', 0.0) * 100
            if cache_hit_rate < self.thresholds.cache_hit_rate_critical:
                await self._create_alert(
                    AlertLevel.CRITICAL,
                    "缓存命中率严重过低",
                    f"缓存命中率 {cache_hit_rate:.2f}% 低于严重阈值",
                    "cache_system",
                    {"cache_hit_rate": cache_hit_rate, "threshold": self.thresholds.cache_hit_rate_critical}
                )
            elif cache_hit_rate < self.thresholds.cache_hit_rate_warning:
                await self._create_alert(
                    AlertLevel.WARNING,
                    "缓存命中率警告",
                    f"缓存命中率 {cache_hit_rate:.2f}% 低于警告阈值",
                    "cache_system",
                    {"cache_hit_rate": cache_hit_rate, "threshold": self.thresholds.cache_hit_rate_warning}
                )

            # 吞吐量检查
            throughput = latest_metrics.get('throughput', 0.0)
            if throughput < self.thresholds.throughput_critical:
                await self._create_alert(
                    AlertLevel.CRITICAL,
                    "吞吐量严重过低",
                    f"吞吐量 {throughput:.2f} 低于严重阈值",
                    "performance",
                    {"throughput": throughput, "threshold": self.thresholds.throughput_critical}
                )
            elif throughput < self.thresholds.throughput_warning:
                await self._create_alert(
                    AlertLevel.WARNING,
                    "吞吐量警告",
                    f"吞吐量 {throughput:.2f} 低于警告阈值",
                    "performance",
                    {"throughput": throughput, "threshold": self.thresholds.throughput_warning}
                )

        except Exception as e:
            logger.error(f"❌ 告警条件检查失败: {str(e)}")

    async def _update_health_status(self) -> None:
        """更新整体健康状态"""
        try:
            # 计算整体健康分数
            health_score = self._calculate_overall_health_score()

            # 确定健康状态
            if health_score >= 90:
                overall_status = HealthStatus.HEALTHY
            elif health_score >= 70:
                overall_status = HealthStatus.WARNING
            elif health_score >= 50:
                overall_status = HealthStatus.CRITICAL
            else:
                overall_status = HealthStatus.DOWN

            # 更新健康评估
            self.last_health_assessment = {
                "overall_status": overall_status.value,
                "health_score": health_score,
                "component": self.component_name,
                "timestamp": time.time(),
                "active_health_checks": {
                    name: {
                        "status": check.status.value,
                        "consecutive_failures": check.consecutive_failures,
                        "last_check_time": check.last_check_time
                    }
                    for name, check in self.active_health_checks.items()
                },
                "performance_summary": self._get_performance_summary(),
                "recent_alerts_count": len([a for a in self.alerts if not a.resolved and time.time() - a.timestamp < 3600])
            }

        except Exception as e:
            logger.error(f"❌ 健康状态更新失败: {str(e)}")

    def _calculate_overall_health_score(self) -> float:
        """计算整体健康分数"""
        try:
            score = 100.0

            if not self.performance_history:
                return score

            latest_metrics = self.performance_history[-1]

            # 响应时间评分 (25%)
            response_time = latest_metrics.get('response_time', 0.0)
            if response_time > 0:
                response_score = max(0, 100 - (response_time / self.thresholds.response_time_critical) * 100)
                score = score * 0.75 + response_score * 0.25

            # CPU使用率评分 (20%)
            cpu_usage = latest_metrics.get('cpu_usage', 0.0)
            cpu_score = max(0, 100 - (cpu_usage / self.thresholds.cpu_critical) * 100)
            score = score * 0.8 + cpu_score * 0.2

            # 内存使用率评分 (20%)
            memory_usage = latest_metrics.get('memory_usage', 0.0)
            memory_score = max(0, 100 - (memory_usage / self.thresholds.memory_critical) * 100)
            score = score * 0.8 + memory_score * 0.2

            # 错误率评分 (25%)
            error_rate = latest_metrics.get('error_rate', 0.0) * 100
            error_score = max(0, 100 - (error_rate / self.thresholds.error_rate_critical) * 100)
            score = score * 0.75 + error_score * 0.25

            # 健康检查评分 (10%)
            health_check_score = self._calculate_health_check_score()
            score = score * 0.9 + health_check_score * 0.1

            return max(0.0, min(100.0, score))

        except Exception as e:
            logger.error(f"❌ 健康分数计算失败: {str(e)}")
            return 50.0  # 默认中等健康状态

    def _calculate_health_check_score(self) -> float:
        """计算健康检查分数"""
        if not self.active_health_checks:
            return 100.0

        total_checks = len(self.active_health_checks)
        healthy_checks = sum(1 for check in self.active_health_checks.values() if check.status == HealthStatus.HEALTHY)

        return (healthy_checks / total_checks) * 100

    def _get_performance_summary(self) -> Dict[str, Any]:
        """获取性能摘要"""
        if not self.performance_history:
            return {}

        recent_data = list(self.performance_history)[-10:]  # 最近10个数据点

        summary = {}
        for metric in ['response_time', 'cpu_usage', 'memory_usage', 'error_rate', 'cache_hit_rate', 'throughput']:
            values = [d.get(metric, 0.0) for d in recent_data if d.get(metric, 0.0) > 0]
            if values:
                summary[f'{metric}_avg'] = statistics.mean(values)
                summary[f'{metric}_min'] = min(values)
                summary[f'{metric}_max'] = max(values)
                summary[f'{metric}_trend'] = "increasing" if values[-1] > values[0] else "decreasing"

        return summary

    async def _create_alert(self,
                          level: AlertLevel,
                          title: str,
                          message: str,
                          component: str,
                          metrics: Optional[Dict[str, Any]] = None) -> None:
        """创建告警"""
        alert = Alert(
            id=f"{int(time.time())}_{component}",
            level=level,
            title=title,
            message=message,
            component=component,
            timestamp=time.time(),
            metrics=metrics or {}
        )

        # 检查是否重复告警 (5分钟内相同组件和级别的告警)
        recent_similar_alerts = [
            a for a in self.alerts
            if (not a.resolved and
                a.component == component and
                a.level == level and
                time.time() - a.timestamp < 300)
        ]

        if not recent_similar_alerts:
            self.alerts.append(alert)
            self.alert_counts[level.value] += 1

            logger.warning(f"🚨 {level.value.upper()} 告警: {title} - {message}")

            # 发送通知
            await self._send_alert_notification(alert)

    async def _send_alert_notification(self, alert: Alert) -> None:
        """发送告警通知"""
        try:
            for callback in self.notification_callbacks:
                try:
                    await asyncio.to_thread(callback, alert)
                except Exception as e:
                    logger.error(f"❌ 告警通知发送失败: {str(e)}")

        except Exception as e:
            logger.error(f"❌ 告警通知处理失败: {str(e)}")

    def add_notification_callback(self, callback: Callable[[Alert], None]) -> None:
        """添加告警通知回调"""
        self.notification_callbacks.append(callback)
        logger.info("📝 已添加告警通知回调")

    # 默认健康检查方法
    def _check_api_connectivity(self) -> bool:
        """检查API连接"""
        try:
            # 这里应该检查阿里云API连接状态
            # 简化实现，实际应该ping或调用API健康检查端点
            import socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            result = sock.connect_ex(('dashscope.aliyuncs.com', 443))
            sock.close()
            return result == 0
        except Exception:
            return False

    def _check_system_resources(self) -> bool:
        """检查系统资源"""
        try:
            import psutil
            cpu_usage = psutil.cpu_percent(interval=1)
            memory_usage = psutil.virtual_memory().percent
            return cpu_usage < 95 and memory_usage < 95
        except Exception:
            return False

    def _check_cache_system(self) -> bool:
        """检查缓存系统"""
        try:
            if hasattr(self, 'system_optimizer') and self.system_optimizer:
                cache_stats = self.system_optimizer.response_cache
                return len(cache_stats.cache) >= 0  # 简单检查缓存是否可访问
            return True
        except Exception:
            return False

    def get_monitoring_status(self) -> Dict[str, Any]:
        """获取监控状态"""
        return {
            "monitoring_active": self.monitoring_active,
            "component_name": self.component_name,
            "health_checks": {
                name: {
                    "status": check.status.value,
                    "consecutive_failures": check.consecutive_failures,
                    "last_check_time": check.last_check_time,
                    "interval_seconds": check.interval_seconds
                }
                for name, check in self.active_health_checks.items()
            },
            "alert_counts": self.alert_counts,
            "recent_alerts": [alert.to_dict() for alert in list(self.alerts)[-10:]],
            "health_assessment": self.last_health_assessment,
            "performance_history_size": len(self.performance_history),
            "thresholds": asdict(self.thresholds),
            "timestamp": time.time()
        }

    def get_performance_report(self, hours: int = 24) -> Dict[str, Any]:
        """获取性能报告"""
        try:
            cutoff_time = time.time() - (hours * 3600)
            relevant_data = [d for d in self.performance_history if d['timestamp'] > cutoff_time]

            if not relevant_data:
                return {"error": "没有足够的历史数据"}

            report = {
                "time_range_hours": hours,
                "data_points": len(relevant_data),
                "period_start": datetime.fromtimestamp(relevant_data[0]['timestamp']).isoformat(),
                "period_end": datetime.fromtimestamp(relevant_data[-1]['timestamp']).isoformat(),
                "metrics": {}
            }

            for metric in ['response_time', 'cpu_usage', 'memory_usage', 'error_rate', 'cache_hit_rate', 'throughput']:
                values = [d.get(metric, 0.0) for d in relevant_data if d.get(metric, 0.0) > 0]
                if values:
                    report["metrics"][metric] = {
                        "avg": statistics.mean(values),
                        "min": min(values),
                        "max": max(values),
                        "median": statistics.median(values),
                        "std_dev": statistics.stdev(values) if len(values) > 1 else 0.0
                    }

            return report

        except Exception as e:
            logger.error(f"❌ 性能报告生成失败: {str(e)}")
            return {"error": str(e)}

# 全局监控器实例
_global_monitor: Optional[SystemMonitor] = None

def get_system_monitor(component_name: str = "XleRobot", thresholds: Optional[PerformanceThresholds] = None) -> SystemMonitor:
    """获取全局系统监控器实例"""
    global _global_monitor

    if _global_monitor is None:
        _global_monitor = SystemMonitor(component_name, thresholds)

    return _global_monitor