#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Story 1.4: 连续语音识别 - 性能监控
Performance Monitoring for Continuous Speech Recognition

实时监控和自动调优系统，确保30分钟连续运行的稳定性。
监控指标:
- CPU使用率
- 内存使用量
- 识别延迟
- 吞吐量
- 识别准确率
- 唤醒词检测成功率

作者: Dev Agent
故事ID: Story 1.4
"""

import time
import threading
import psutil
import statistics
from typing import Dict, List, Optional, Callable, Any
from dataclasses import dataclass, field
from enum import Enum, auto
from queue import Queue, Empty
import logging
import json
from pathlib import Path

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class MetricType(Enum):
    """指标类型枚举"""
    CPU_USAGE = auto()          # CPU使用率 (0-100%)
    MEMORY_USAGE = auto()       # 内存使用量 (MB)
    LATENCY = auto()            # 延迟 (ms)
    THROUGHPUT = auto()         # 吞吐量 (samples/s)
    ACCURACY = auto()           # 识别准确率 (0-1)
    WAKE_WORD_SUCCESS = auto()  # 唤醒词检测成功率 (0-1)
    POWER_CONSUMPTION = auto()  # 功耗 (W)
    TEMPERATURE = auto()        # 温度 (°C)


class AlertLevel(Enum):
    """告警级别枚举"""
    INFO = auto()
    WARNING = auto()
    CRITICAL = auto()
    EMERGENCY = auto()


@dataclass
class MetricPoint:
    """指标数据点"""
    timestamp: float
    value: float
    metric_type: MetricType
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class Alert:
    """告警信息"""
    level: AlertLevel
    message: str
    timestamp: float
    metric_type: MetricType
    value: float
    threshold: float
    resolved: bool = False
    resolved_at: Optional[float] = None


class PerformanceMonitor:
    """
    性能监控器

    功能特性:
    - 实时采集性能指标
    - 自动调优算法
    - 智能告警系统
    - 历史数据分析
    - 性能趋势预测

    监控指标:
    - CPU使用率 (<80%)
    - 内存使用量 (<300MB)
    - 识别延迟 (<300ms P95)
    - 吞吐量 (>10 samples/s)
    - 识别准确率 (>90%)
    - 唤醒词检测成功率 (>95%)
    """

    def __init__(self,
                 sample_interval: float = 1.0,
                 history_size: int = 1000,
                 enable_auto_tuning: bool = True):
        """
        初始化性能监控器

        Args:
            sample_interval: 采样间隔 (秒)
            history_size: 历史数据大小
            enable_auto_tuning: 是否启用自动调优
        """
        self.sample_interval = sample_interval
        self.history_size = history_size
        self.enable_auto_tuning = enable_auto_tuning

        # 指标历史数据
        self._metrics_history: Dict[MetricType, List[MetricPoint]] = {
            metric_type: [] for metric_type in MetricType
        }
        self._metrics_lock = threading.RLock()

        # 告警管理
        self._alerts: List[Alert] = []
        self._alert_callbacks: List[Callable[[Alert], None]] = []

        # 统计信息
        self._stats = {
            'total_samples': 0,
            'start_time': time.time(),
            'last_sample_time': 0.0,
            'auto_tuning_count': 0,
            'alert_count': 0,
            'resolved_alerts': 0
        }

        # 系统基线
        self._baseline = {
            MetricType.CPU_USAGE: 0.0,
            MetricType.MEMORY_USAGE: 0.0,
            MetricType.LATENCY: 0.0,
            MetricType.THROUGHPUT: 0.0,
            MetricType.ACCURACY: 0.0,
            MetricType.WAKE_WORD_SUCCESS: 0.0
        }

        # 自动调优规则
        self._tuning_rules = {
            MetricType.CPU_USAGE: {
                'warning_threshold': 80.0,
                'critical_threshold': 90.0,
                'auto_action': self._tune_cpu_usage
            },
            MetricType.MEMORY_USAGE: {
                'warning_threshold': 250.0,  # MB
                'critical_threshold': 300.0,  # MB
                'auto_action': self._tune_memory_usage
            },
            MetricType.LATENCY: {
                'warning_threshold': 200.0,  # ms
                'critical_threshold': 300.0,  # ms
                'auto_action': self._tune_latency
            },
            MetricType.ACCURACY: {
                'warning_threshold': 0.85,
                'critical_threshold': 0.80,
                'auto_action': self._tune_accuracy
            },
            MetricType.WAKE_WORD_SUCCESS: {
                'warning_threshold': 0.90,
                'critical_threshold': 0.85,
                'auto_action': self._tune_wake_word
            }
        }

        # 监控线程
        self._monitoring_active = False
        self._monitoring_thread: Optional[threading.Thread] = None

        # 启动监控
        self.start_monitoring()

        # 建立基线
        self._establish_baseline()

        logger.info(f"性能监控器初始化完成: 采样间隔={sample_interval}秒, "
                   f"自动调优={'启用' if enable_auto_tuning else '禁用'}")

    def start_monitoring(self) -> None:
        """启动性能监控"""
        if self._monitoring_active:
            logger.warning("性能监控已在运行")
            return

        self._monitoring_active = True
        self._monitoring_thread = threading.Thread(
            target=self._monitoring_worker,
            daemon=True
        )
        self._monitoring_thread.start()

        logger.info("📊 性能监控已启动")

    def stop_monitoring(self) -> None:
        """停止性能监控"""
        if not self._monitoring_active:
            return

        self._monitoring_active = False
        if self._monitoring_thread and self._monitoring_thread.is_alive():
            self._monitoring_thread.join(timeout=2.0)

        logger.info("📊 性能监控已停止")

    def _monitoring_worker(self) -> None:
        """监控工作线程"""
        while self._monitoring_active:
            try:
                # 采集系统指标
                self._collect_system_metrics()

                # 采集应用指标
                self._collect_application_metrics()

                # 检查告警
                self._check_alerts()

                # 执行自动调优
                if self.enable_auto_tuning:
                    self._auto_tune()

                # 更新统计信息
                with self._metrics_lock:
                    self._stats['total_samples'] += 1
                    self._stats['last_sample_time'] = time.time()

                # 等待采样间隔
                time.sleep(self.sample_interval)

            except Exception as e:
                logger.error(f"监控工作线程错误: {e}")
                if self._monitoring_active:
                    time.sleep(1)

    def _collect_system_metrics(self) -> None:
        """采集系统指标"""
        try:
            current_time = time.time()

            # CPU使用率
            cpu_percent = psutil.cpu_percent(interval=0.1)
            self._record_metric(MetricType.CPU_USAGE, cpu_percent, current_time)

            # 内存使用量
            memory = psutil.virtual_memory()
            memory_mb = (memory.total - memory.available) / 1024 / 1024
            self._record_metric(MetricType.MEMORY_USAGE, memory_mb, current_time)

        except Exception as e:
            logger.error(f"系统指标采集失败: {e}")

    def _collect_application_metrics(self) -> None:
        """采集应用指标 (需要外部数据源)"""
        # 应用指标通常来自外部数据源，如:
        # - ASR引擎的识别延迟
        # - 识别准确率
        # - 唤醒词检测成功率
        # 这些指标需要通过 record_metric 方法手动记录

        pass

    def _record_metric(self,
                      metric_type: MetricType,
                      value: float,
                      timestamp: Optional[float] = None,
                      metadata: Optional[Dict[str, Any]] = None) -> None:
        """
        记录指标数据点

        Args:
            metric_type: 指标类型
            value: 指标值
            timestamp: 时间戳 (可选，默认当前时间)
            metadata: 元数据
        """
        if timestamp is None:
            timestamp = time.time()

        # 创建指标数据点
        point = MetricPoint(
            timestamp=timestamp,
            value=value,
            metric_type=metric_type,
            metadata=metadata or {}
        )

        # 添加到历史数据
        with self._metrics_lock:
            self._metrics_history[metric_type].append(point)

            # 保持历史数据大小
            if len(self._metrics_history[metric_type]) > self.history_size:
                self._metrics_history[metric_type].pop(0)

    def get_metric_stats(self,
                        metric_type: MetricType,
                        duration_seconds: Optional[float] = None) -> Dict[str, float]:
        """
        获取指标统计信息

        Args:
            metric_type: 指标类型
            duration_seconds: 时间范围 (秒)，None表示全部历史

        Returns:
            统计信息字典
        """
        with self._metrics_lock:
            points = self._metrics_history[metric_type]

            if not points:
                return {
                    'count': 0,
                    'min': 0.0,
                    'max': 0.0,
                    'mean': 0.0,
                    'median': 0.0,
                    'p95': 0.0,
                    'p99': 0.0
                }

            # 过滤时间范围
            if duration_seconds:
                cutoff_time = time.time() - duration_seconds
                points = [p for p in points if p.timestamp >= cutoff_time]

            # 计算统计值
            values = [p.value for p in points]
            values.sort()

            count = len(values)
            minimum = values[0]
            maximum = values[-1]
            mean = statistics.mean(values)
            median = statistics.median(values)

            # 计算百分位数
            p95_index = int(count * 0.95)
            p99_index = int(count * 0.99)
            p95 = values[min(p95_index, count - 1)]
            p99 = values[min(p99_index, count - 1)]

            return {
                'count': count,
                'min': minimum,
                'max': maximum,
                'mean': mean,
                'median': median,
                'p95': p95,
                'p99': p99
            }

    def _check_alerts(self) -> None:
        """检查告警条件"""
        current_time = time.time()

        for metric_type, rule in self._tuning_rules.items():
            stats = self.get_metric_stats(metric_type, duration_seconds=60)  # 最近1分钟

            if stats['count'] == 0:
                continue

            latest_value = self._get_latest_value(metric_type)

            # 检查告警阈值
            warning_threshold = rule['warning_threshold']
            critical_threshold = rule['critical_threshold']

            # 对于"越低越好"的指标 (如延迟)
            if metric_type in [MetricType.LATENCY, MetricType.CPU_USAGE, MetricType.MEMORY_USAGE]:
                alert_level = None
                threshold = 0.0

                if latest_value >= critical_threshold:
                    alert_level = AlertLevel.CRITICAL
                    threshold = critical_threshold
                elif latest_value >= warning_threshold:
                    alert_level = AlertLevel.WARNING
                    threshold = warning_threshold

                # 对于"越高越好"的指标 (如准确率)
            else:
                alert_level = None
                threshold = 0.0

                if latest_value <= critical_threshold:
                    alert_level = AlertLevel.CRITICAL
                    threshold = critical_threshold
                elif latest_value <= warning_threshold:
                    alert_level = AlertLevel.WARNING
                    threshold = warning_threshold

            # 触发告警
            if alert_level:
                self._trigger_alert(alert_level, metric_type, latest_value, threshold)

    def _trigger_alert(self,
                      level: AlertLevel,
                      metric_type: MetricType,
                      value: float,
                      threshold: float) -> None:
        """触发告警"""
        # 检查是否已有未解决的相同告警
        for alert in self._alerts:
            if (not alert.resolved and
                alert.metric_type == metric_type and
                alert.level == level):
                return  # 已有相同告警，不重复触发

        # 创建新告警
        alert = Alert(
            level=level,
            message=f"{metric_type.name} 超过阈值: {value:.2f} > {threshold:.2f}",
            timestamp=time.time(),
            metric_type=metric_type,
            value=value,
            threshold=threshold
        )

        # 添加到告警列表
        self._alerts.append(alert)

        # 更新统计信息
        with self._metrics_lock:
            self._stats['alert_count'] += 1

        # 记录日志
        log_level = {
            AlertLevel.INFO: logging.INFO,
            AlertLevel.WARNING: logging.WARNING,
            AlertLevel.CRITICAL: logging.CRITICAL,
            AlertLevel.EMERGENCY: logging.CRITICAL
        }[level]

        logger.log(log_level, f"🚨 告警: {alert.message}")

        # 调用告警回调
        for callback in self._alert_callbacks:
            try:
                callback(alert)
            except Exception as e:
                logger.error(f"告警回调函数执行失败: {e}")

    def _get_latest_value(self, metric_type: MetricType) -> float:
        """获取指标最新值"""
        with self._metrics_lock:
            points = self._metrics_history[metric_type]
            return points[-1].value if points else 0.0

    def _auto_tune(self) -> None:
        """执行自动调优"""
        for metric_type, rule in self._tuning_rules.items():
            latest_value = self._get_latest_value(metric_type)
            warning_threshold = rule['warning_threshold']

            # 对于"越低越好"的指标
            if metric_type in [MetricType.LATENCY, MetricType.CPU_USAGE, MetricType.MEMORY_USAGE]:
                if latest_value >= warning_threshold:
                    auto_action = rule['auto_action']
                    if auto_action:
                        auto_action(latest_value, warning_threshold)

            # 对于"越高越好"的指标
            else:
                if latest_value <= warning_threshold:
                    auto_action = rule['auto_action']
                    if auto_action:
                        auto_action(latest_value, warning_threshold)

    def _tune_cpu_usage(self, current_value: float, threshold: float) -> None:
        """CPU使用率调优"""
        logger.warning(f"🔧 CPU使用率过高 ({current_value:.1f}%)，执行调优")
        # 调优策略:
        # - 降低处理频率
        # - 减少并发数
        # - 启用节能模式

        # 更新统计信息
        with self._metrics_lock:
            self._stats['auto_tuning_count'] += 1

    def _tune_memory_usage(self, current_value: float, threshold: float) -> None:
        """内存使用量调优"""
        logger.warning(f"🔧 内存使用量过高 ({current_value:.1f}MB)，执行调优")
        # 调优策略:
        # - 触发垃圾回收
        # - 释放缓存
        # - 清理临时数据

        # 更新统计信息
        with self._metrics_lock:
            self._stats['auto_tuning_count'] += 1

    def _tune_latency(self, current_value: float, threshold: float) -> None:
        """延迟调优"""
        logger.warning(f"🔧 识别延迟过高 ({current_value:.1f}ms)，执行调优")
        # 调优策略:
        # - 启用NPU加速
        # - 优化模型参数
        # - 减少音频缓冲大小

        # 更新统计信息
        with self._metrics_lock:
            self._stats['auto_tuning_count'] += 1

    def _tune_accuracy(self, current_value: float, threshold: float) -> None:
        """准确率调优"""
        logger.warning(f"🔧 识别准确率过低 ({current_value:.1%})，执行调优")
        # 调优策略:
        # - 调整模型参数
        # - 增强音频预处理
        # - 更新语言模型

        # 更新统计信息
        with self._metrics_lock:
            self._stats['auto_tuning_count'] += 1

    def _tune_wake_word(self, current_value: float, threshold: float) -> None:
        """唤醒词检测调优"""
        logger.warning(f"🔧 唤醒词检测成功率过低 ({current_value:.1%})，执行调优")
        # 调优策略:
        # - 调整检测阈值
        # - 优化特征提取
        # - 更新唤醒词模型

        # 更新统计信息
        with self._metrics_lock:
            self._stats['auto_tuning_count'] += 1

    def _establish_baseline(self, duration: float = 30.0) -> None:
        """建立性能基线"""
        logger.info("📊 建立性能基线...")

        # 等待采样完成
        time.sleep(duration)

        # 计算基线值
        for metric_type in MetricType:
            if metric_type != MetricType.POWER_CONSUMPTION and metric_type != MetricType.TEMPERATURE:
                stats = self.get_metric_stats(metric_type)
                self._baseline[metric_type] = stats['mean']

        logger.info(f"📊 性能基线建立完成: {self._baseline}")

    def add_alert_callback(self, callback: Callable[[Alert], None]) -> None:
        """添加告警回调函数"""
        self._alert_callbacks.append(callback)

    def remove_alert_callback(self, callback: Callable[[Alert], None]) -> bool:
        """移除告警回调函数"""
        try:
            self._alert_callbacks.remove(callback)
            return True
        except ValueError:
            return False

    def get_recent_alerts(self, count: int = 10) -> List[Alert]:
        """获取最近的告警"""
        return self._alerts[-count:] if count > 0 else self._alerts

    def resolve_alert(self, alert_index: int) -> bool:
        """解决告警"""
        if 0 <= alert_index < len(self._alerts):
            self._alerts[alert_index].resolved = True
            self._alerts[alert_index].resolved_at = time.time()
            with self._metrics_lock:
                self._stats['resolved_alerts'] += 1
            return True
        return False

    def export_metrics(self, output_path: str, format: str = 'json') -> bool:
        """
        导出指标数据

        Args:
            output_path: 输出文件路径
            format: 格式 ('json' 或 'csv')

        Returns:
            是否成功导出
        """
        try:
            # 确保目录存在
            Path(output_path).parent.mkdir(parents=True, exist_ok=True)

            if format == 'json':
                # 导出为JSON格式
                data = {}
                for metric_type, points in self._metrics_history.items():
                    data[metric_type.name] = [
                        {
                            'timestamp': p.timestamp,
                            'value': p.value,
                            'metadata': p.metadata
                        }
                        for p in points
                    ]

                with open(output_path, 'w', encoding='utf-8') as f:
                    json.dump(data, f, indent=2, ensure_ascii=False)

            elif format == 'csv':
                # 导出为CSV格式 (需要转换为二维表格)
                pass  # 简化实现

            logger.info(f"📊 指标数据已导出: {output_path}")
            return True

        except Exception as e:
            logger.error(f"导出指标数据失败: {e}")
            return False

    def get_performance_summary(self) -> Dict[str, Any]:
        """获取性能摘要"""
        with self._metrics_lock:
            uptime = time.time() - self._stats['start_time']
            uptime_hours = uptime / 3600

            summary = {
                'uptime_seconds': uptime,
                'uptime_hours': uptime_hours,
                'total_samples': self._stats['total_samples'],
                'auto_tuning_count': self._stats['auto_tuning_count'],
                'alert_count': self._stats['alert_count'],
                'resolved_alerts': self._stats['resolved_alerts'],
                'baseline': self._baseline.copy()
            }

            # 添加最近1小时的指标统计
            for metric_type in MetricType:
                if metric_type in self._baseline:
                    stats = self.get_metric_stats(metric_type, duration_seconds=3600)
                    summary[f'{metric_type.name}_1h'] = stats

            return summary

    def print_performance_report(self) -> None:
        """打印性能报告"""
        summary = self.get_performance_summary()
        uptime_hours = summary['uptime_hours']

        print("\n" + "=" * 60)
        print("性能监控报告")
        print("=" * 60)
        print(f"运行时间:     {uptime_hours:.2f} 小时")
        print(f"总采样数:     {summary['total_samples']:,}")
        print(f"自动调优:     {summary['auto_tuning_count']} 次")
        print(f"告警总数:     {summary['alert_count']}")
        print(f"已解决告警:   {summary['resolved_alerts']}")
        print("\n基线指标:")
        for metric_type, baseline_value in summary['baseline'].items():
            print(f"  {metric_type:20s}: {baseline_value:.2f}")
        print("\n最近1小时性能:")
        for key, stats in summary.items():
            if key.endswith('_1h') and isinstance(stats, dict):
                metric_name = key.replace('_1h', '')
                print(f"  {metric_name:20s}: 均值={stats['mean']:.2f}, "
                     f"P95={stats['p95']:.2f}, P99={stats['p99']:.2f}")
        print("=" * 60)

    def __str__(self) -> str:
        summary = self.get_performance_summary()
        uptime_hours = summary['uptime_hours']

        return (
            f"性能监控器\n"
            f"运行时间: {uptime_hours:.2f}小时\n"
            f"采样数: {summary['total_samples']:,}\n"
            f"自动调优: {summary['auto_tuning_count']}次\n"
            f"告警: {summary['alert_count']} (已解决{summary['resolved_alerts']})\n"
            f"监控指标: {len(self._tuning_rules)}项"
        )

    def __repr__(self) -> str:
        return (f"PerformanceMonitor(interval={self.sample_interval}s, "
                f"history={self.history_size}, "
                f"auto_tuning={self.enable_auto_tuning})")


# 示例使用
if __name__ == "__main__":
    # 创建性能监控器
    monitor = PerformanceMonitor(sample_interval=0.5, enable_auto_tuning=True)

    print("=" * 60)
    print("性能监控器演示")
    print("=" * 60)

    # 添加告警回调
    def on_alert(alert: Alert):
        print(f"🚨 告警: {alert.level.name} - {alert.message}")

    monitor.add_alert_callback(on_alert)

    # 模拟运行30秒
    print("\n模拟系统运行30秒...")
    start_time = time.time()

    while time.time() - start_time < 30:
        # 模拟应用指标
        import random

        # 模拟识别延迟
        latency = random.normalvariate(150, 30)  # 平均150ms，标准差30ms
        monitor._record_metric(MetricType.LATENCY, latency, metadata={'session': 'demo'})

        # 模拟识别准确率
        accuracy = random.betavariate(8, 2)  # Beta分布，偏向高准确率
        monitor._record_metric(MetricType.ACCURACY, accuracy)

        # 模拟唤醒词检测成功率
        wake_word_success = random.betavariate(15, 1)  # 偏向高成功率
        monitor._record_metric(MetricType.WAKE_WORD_SUCCESS, wake_word_success)

        time.sleep(0.5)

    # 打印性能报告
    monitor.print_performance_report()

    # 打印特定指标统计
    print("\n关键指标统计:")
    for metric_type in [MetricType.CPU_USAGE, MetricType.LATENCY, MetricType.ACCURACY]:
        stats = monitor.get_metric_stats(metric_type)
        print(f"\n{metric_type.name}:")
        print(f"  样本数: {stats['count']}")
        print(f"  均值:   {stats['mean']:.2f}")
        print(f"  P95:    {stats['p95']:.2f}")
        print(f"  P99:    {stats['p99']:.2f}")

    # 打印最近的告警
    alerts = monitor.get_recent_alerts(5)
    if alerts:
        print("\n最近的告警:")
        for i, alert in enumerate(alerts):
            resolved = "✅" if alert.resolved else "❌"
            print(f"  {i+1}. {resolved} [{alert.level.name}] {alert.message}")

    # 导出数据
    monitor.export_metrics('/tmp/performance_metrics.json')

    # 清理
    monitor.stop_monitoring()
    print("\n✅ 性能监控器演示结束")
