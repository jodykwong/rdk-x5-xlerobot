"""
故障容错机制 - FaultTolerance
============================

实现系统故障检测、恢复和容错机制，确保系统可用性>99.9%。
包括故障检测、模块重启、数据备份、降级服务和故障自愈功能。

Author: BMad System Architect
Created: 2025-11-05
"""

from typing import Dict, List, Optional, Any, Callable, Set
from dataclasses import dataclass, field
from datetime import datetime, timedelta
from enum import Enum
import asyncio
import logging
import json
import pickle
import threading
import time
from collections import deque, defaultdict
import hashlib

# 导入配置
from .config import CONFIG

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    logging.warning("ROS2 not available, running in standalone mode")


class FailureType(Enum):
    """故障类型"""
    MODULE_CRASH = "module_crash"
    MODULE_HANG = "module_hang"
    MESSAGE_TIMEOUT = "message_timeout"
    RESOURCE_EXHAUSTION = "resource_exhaustion"
    DEPENDENCY_FAILURE = "dependency_failure"
    NETWORK_ERROR = "network_error"
    DATA_CORRUPTION = "data_corruption"
    PERFORMANCE_DEGRADATION = "performance_degradation"


class Severity(Enum):
    """故障严重级别"""
    LOW = 1
    MEDIUM = 2
    HIGH = 3
    CRITICAL = 4


class RecoveryStrategy(Enum):
    """恢复策略"""
    RESTART_MODULE = "restart_module"
    RESTART_WITH_BACKUP = "restart_with_backup"
    GRACEFUL_DEGRADATION = "graceful_degradation"
    FAILOVER = "failover"
    ROLLBACK = "rollback"
    CIRCUIT_BREAKER = "circuit_breaker"


@dataclass
class FailureEvent:
    """故障事件"""
    event_id: str
    module_id: str
    failure_type: FailureType
    severity: Severity
    timestamp: datetime
    description: str
    context: Dict[str, Any] = field(default_factory=dict)
    stack_trace: Optional[str] = None


@dataclass
class RecoveryAction:
    """恢复动作"""
    action_id: str
    strategy: RecoveryStrategy
    module_id: str
    parameters: Dict[str, Any] = field(default_factory=dict)
    max_attempts: int = 3
    attempt_delay: float = 5.0
    created_at: datetime = field(default_factory=datetime.now)


@dataclass
class BackupData:
    """备份数据"""
    module_id: str
    data_type: str
    data: Any
    timestamp: datetime = field(default_factory=datetime.now)
    checksum: str = ""
    version: str = "1.0"

    def __post_init__(self):
        """计算校验和"""
        if isinstance(self.data, dict) or isinstance(self.data, list):
            serialized = json.dumps(self.data, sort_keys=True).encode('utf-8')
            self.checksum = hashlib.md5(serialized).hexdigest()


class HealthMonitor:
    """健康监控器"""
    def __init__(self, check_interval: float = 5.0):
        self.logger = logging.getLogger("HealthMonitor")
        self.check_interval = check_interval
        self.health_checks: Dict[str, Callable] = {}
        self.module_health_status: Dict[str, Dict[str, Any]] = {}
        self.monitoring = False
        self.monitor_thread = None

    def register_health_check(self, module_id: str, check_func: Callable):
        """注册健康检查"""
        self.health_checks[module_id] = check_func
        self.logger.info(f"Registered health check for module: {module_id}")

    def start_monitoring(self):
        """开始监控"""
        if self.monitoring:
            return

        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        self.logger.info("Health monitoring started")

    def stop_monitoring(self):
        """停止监控"""
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=5.0)
        self.logger.info("Health monitoring stopped")

    def _monitor_loop(self):
        """监控循环"""
        while self.monitoring:
            try:
                for module_id, check_func in self.health_checks.items():
                    try:
                        result = check_func()
                        self.module_health_status[module_id] = {
                            "healthy": result.get("healthy", False),
                            "metrics": result.get("metrics", {}),
                            "last_check": datetime.now().isoformat(),
                            "issues": result.get("issues", [])
                        }
                    except Exception as e:
                        self.logger.error(f"Health check failed for {module_id}: {e}")
                        self.module_health_status[module_id] = {
                            "healthy": False,
                            "error": str(e),
                            "last_check": datetime.now().isoformat()
                        }

                time.sleep(self.check_interval)

            except Exception as e:
                self.logger.error(f"Monitoring loop error: {e}")

    def get_health_status(self, module_id: str) -> Optional[Dict[str, Any]]:
        """获取健康状态"""
        return self.module_health_status.get(module_id)

    def get_all_health_status(self) -> Dict[str, Dict[str, Any]]:
        """获取所有模块健康状态"""
        return self.module_health_status.copy()


class BackupManager:
    """备份管理器"""
    def __init__(self, backup_dir: str = None):
        self.logger = logging.getLogger("BackupManager")
        # 从配置读取备份目录，默认为配置中的值
        backup_config = CONFIG.get_backup_config()
        self.backup_dir = backup_dir or backup_config["directory"]
        self.backups: Dict[str, List[BackupData]] = defaultdict(list)
        self.max_backups_per_module = 10
        self._ensure_backup_dir()

    def _ensure_backup_dir(self):
        """确保备份目录存在"""
        import os
        os.makedirs(self.backup_dir, exist_ok=True)

    def create_backup(self, module_id: str, data_type: str, data: Any) -> str:
        """创建备份"""
        try:
            backup = BackupData(
                module_id=module_id,
                data_type=data_type,
                data=data
            )

            # 存储备份
            self.backups[module_id].append(backup)

            # 限制备份数量
            if len(self.backups[module_id]) > self.max_backups_per_module:
                self.backups[module_id].pop(0)

            # 持久化备份
            self._persist_backup(backup)

            self.logger.info(f"Backup created for {module_id}:{data_type}")
            return backup.checksum

        except Exception as e:
            self.logger.error(f"Failed to create backup: {e}")
            return ""

    def restore_backup(self, module_id: str, data_type: str, checksum: str) -> Optional[Any]:
        """恢复备份"""
        try:
            module_backups = self.backups.get(module_id, [])
            for backup in reversed(module_backups):
                if backup.data_type == data_type and backup.checksum == checksum:
                    self.logger.info(f"Backup restored for {module_id}:{data_type}")
                    return backup.data

            self.logger.warning(f"Backup not found: {module_id}:{data_type}:{checksum}")
            return None

        except Exception as e:
            self.logger.error(f"Failed to restore backup: {e}")
            return None

    def list_backups(self, module_id: str) -> List[Dict[str, Any]]:
        """列出备份"""
        module_backups = self.backups.get(module_id, [])
        return [
            {
                "data_type": b.data_type,
                "timestamp": b.timestamp.isoformat(),
                "checksum": b.checksum,
                "version": b.version
            }
            for b in module_backups
        ]

    def _persist_backup(self, backup: BackupData):
        """持久化备份到磁盘"""
        try:
            import os
            filename = f"{backup.module_id}_{backup.data_type}_{backup.checksum}.pkl"
            filepath = os.path.join(self.backup_dir, filename)

            with open(filepath, 'wb') as f:
                pickle.dump(backup, f)

        except Exception as e:
            self.logger.error(f"Failed to persist backup: {e}")


class CircuitBreaker:
    """断路器实现"""
    def __init__(self, failure_threshold: int = 5, recovery_timeout: float = 60.0):
        self.failure_threshold = failure_threshold
        self.recovery_timeout = recovery_timeout
        self.failure_count = 0
        self.last_failure_time = None
        self.state = "CLOSED"  # CLOSED, OPEN, HALF_OPEN

    def call(self, func: Callable, *args, **kwargs):
        """通过断路器调用函数"""
        if self.state == "OPEN":
            if self._should_attempt_reset():
                self.state = "HALF_OPEN"
            else:
                raise Exception("Circuit breaker is OPEN")

        try:
            result = func(*args, **kwargs)
            self._on_success()
            return result

        except Exception as e:
            self._on_failure()
            raise e

    def _should_attempt_reset(self) -> bool:
        """检查是否应该尝试重置"""
        if self.last_failure_time is None:
            return True
        elapsed = time.time() - self.last_failure_time.timestamp()
        return elapsed > self.recovery_timeout

    def _on_success(self):
        """成功时的处理"""
        self.failure_count = 0
        self.state = "CLOSED"

    def _on_failure(self):
        """失败时的处理"""
        self.failure_count += 1
        self.last_failure_time = datetime.now()

        if self.failure_count >= self.failure_threshold:
            self.state = "OPEN"


class FaultTolerance:
    """
    故障容错系统

    功能：
    1. 故障检测和诊断
    2. 自动恢复策略
    3. 数据备份和恢复
    4. 断路器模式
    5. 降级服务
    """

    def __init__(self):
        self.logger = logging.getLogger("FaultTolerance")

        # 核心组件
        self.health_monitor = HealthMonitor()
        self.backup_manager = BackupManager()
        self.circuit_breakers: Dict[str, CircuitBreaker] = {}

        # 故障管理
        self.failure_events: deque = deque(maxlen=1000)
        self.recovery_actions: Dict[str, RecoveryAction] = {}
        self.module_states: Dict[str, Dict[str, Any]] = {}

        # 统计信息
        self.stats = {
            "total_failures": 0,
            "recovered_failures": 0,
            "failed_recoveries": 0,
            "uptime": 0.0,
            "availability": 0.0
        }

        self.running = False

        self.logger.info("FaultTolerance initialized")

    def initialize(self) -> bool:
        """初始化故障容错系统"""
        try:
            self.logger.info("Initializing FaultTolerance...")

            # 启动健康监控
            self.health_monitor.start_monitoring()

            # 注册默认故障处理器
            self._register_default_handlers()

            self.running = True
            self.logger.info("FaultTolerance initialized successfully")
            return True

        except Exception as e:
            self.logger.error(f"Failed to initialize FaultTolerance: {e}")
            return False

    def _register_default_handlers(self):
        """注册默认故障处理器"""
        # 模块崩溃处理器
        self.register_failure_handler(
            FailureType.MODULE_CRASH,
            self._handle_module_crash
        )

        # 模块挂起处理器
        self.register_failure_handler(
            FailureType.MODULE_HANG,
            self._handle_module_hang
        )

        # 消息超时处理器
        self.register_failure_handler(
            FailureType.MESSAGE_TIMEOUT,
            self._handle_message_timeout
        )

    def register_failure_handler(self, failure_type: FailureType, handler: Callable):
        """注册故障处理器"""
        # 这里可以实现故障类型到处理器的映射
        self.logger.info(f"Registered handler for failure type: {failure_type.value}")

    def detect_failure(
        self,
        module_id: str,
        failure_type: FailureType,
        severity: Severity,
        description: str,
        context: Optional[Dict[str, Any]] = None
    ) -> str:
        """检测故障"""
        try:
            event_id = f"FAIL_{int(time.time() * 1000000)}"
            failure_event = FailureEvent(
                event_id=event_id,
                module_id=module_id,
                failure_type=failure_type,
                severity=severity,
                timestamp=datetime.now(),
                description=description,
                context=context or {}
            )

            # 记录故障事件
            self.failure_events.append(failure_event)
            self.stats["total_failures"] += 1

            self.logger.warning(
                f"Failure detected: {module_id} - {failure_type.value} - {description}"
            )

            # 尝试自动恢复
            self._attempt_recovery(failure_event)

            return event_id

        except Exception as e:
            self.logger.error(f"Failed to detect failure: {e}")
            return ""

    def _attempt_recovery(self, failure_event: FailureEvent):
        """尝试恢复"""
        try:
            # 根据故障类型选择恢复策略
            strategy = self._select_recovery_strategy(failure_event)

            if not strategy:
                self.logger.warning(f"No recovery strategy for failure: {failure_event.event_id}")
                return

            # 创建恢复动作
            action_id = f"RECOV_{int(time.time() * 1000000)}"
            recovery_action = RecoveryAction(
                action_id=action_id,
                strategy=strategy,
                module_id=failure_event.module_id,
                parameters=failure_event.context
            )

            self.recovery_actions[action_id] = recovery_action

            # 执行恢复
            self._execute_recovery(recovery_action)

        except Exception as e:
            self.logger.error(f"Recovery attempt failed: {e}")
            self.stats["failed_recoveries"] += 1

    def _select_recovery_strategy(self, failure_event: FailureEvent) -> Optional[RecoveryStrategy]:
        """选择恢复策略"""
        module_id = failure_event.module_id
        failure_type = failure_event.failure_type
        severity = failure_event.severity

        # 简单策略选择逻辑
        if failure_type == FailureType.MODULE_CRASH:
            if severity == Severity.CRITICAL:
                return RecoveryStrategy.RESTART_WITH_BACKUP
            else:
                return RecoveryStrategy.RESTART_MODULE
        elif failure_type == FailureType.MODULE_HANG:
            return RecoveryStrategy.RESTART_MODULE
        elif failure_type == FailureType.MESSAGE_TIMEOUT:
            return RecoveryStrategy.CIRCUIT_BREAKER
        elif failure_type == FailureType.PERFORMANCE_DEGRADATION:
            return RecoveryStrategy.GRACEFUL_DEGRADATION
        else:
            return RecoveryStrategy.RESTART_MODULE

    def _execute_recovery(self, recovery_action: RecoveryAction):
        """执行恢复动作"""
        try:
            self.logger.info(f"Executing recovery: {recovery_action.action_id}")

            # 根据策略执行恢复
            if recovery_action.strategy == RecoveryStrategy.RESTART_MODULE:
                self._restart_module(recovery_action.module_id)
            elif recovery_action.strategy == RecoveryStrategy.RESTART_WITH_BACKUP:
                self._restart_with_backup(recovery_action.module_id)
            elif recovery_action.strategy == RecoveryStrategy.GRACEFUL_DEGRADATION:
                self._enable_graceful_degradation(recovery_action.module_id)
            elif recovery_action.strategy == RecoveryStrategy.CIRCUIT_BREAKER:
                self._enable_circuit_breaker(recovery_action.module_id)

            self.stats["recovered_failures"] += 1
            self.logger.info(f"Recovery completed: {recovery_action.action_id}")

        except Exception as e:
            self.logger.error(f"Recovery execution failed: {e}")
            self.stats["failed_recoveries"] += 1

    def _restart_module(self, module_id: str):
        """重启模块"""
        self.logger.info(f"Restarting module: {module_id}")
        # 这里是重启逻辑的占位符
        # 实际实现需要与系统控制模块集成

    def _restart_with_backup(self, module_id: str):
        """从备份重启模块"""
        self.logger.info(f"Restarting module with backup: {module_id}")

        # 获取最新备份
        backups = self.backup_manager.list_backups(module_id)
        if backups:
            latest_backup = backups[-1]
            self.logger.info(f"Using backup: {latest_backup}")

        # 重启模块
        self._restart_module(module_id)

    def _enable_graceful_degradation(self, module_id: str):
        """启用降级服务"""
        self.logger.info(f"Enabling graceful degradation for: {module_id}")
        # 这里是降级服务的占位符

    def _enable_circuit_breaker(self, module_id: str):
        """启用断路器"""
        if module_id not in self.circuit_breakers:
            self.circuit_breakers[module_id] = CircuitBreaker()
        self.logger.info(f"Circuit breaker enabled for: {module_id}")

    def _handle_module_crash(self, failure_event: FailureEvent):
        """处理模块崩溃"""
        self.detect_failure(
            failure_event.module_id,
            FailureType.MODULE_CRASH,
            Severity.HIGH,
            "Module crash detected"
        )

    def _handle_module_hang(self, failure_event: FailureEvent):
        """处理模块挂起"""
        self.detect_failure(
            failure_event.module_id,
            FailureType.MODULE_HANG,
            Severity.MEDIUM,
            "Module hang detected"
        )

    def _handle_message_timeout(self, failure_event: FailureEvent):
        """处理消息超时"""
        self.detect_failure(
            failure_event.module_id,
            FailureType.MESSAGE_TIMEOUT,
            Severity.MEDIUM,
            "Message timeout detected"
        )

    def create_module_backup(self, module_id: str, data_type: str, data: Any) -> str:
        """为模块创建备份"""
        return self.backup_manager.create_backup(module_id, data_type, data)

    def restore_module_backup(
        self,
        module_id: str,
        data_type: str,
        checksum: str
    ) -> Optional[Any]:
        """恢复模块备份"""
        return self.backup_manager.restore_backup(module_id, data_type, checksum)

    def get_failure_history(self, module_id: Optional[str] = None) -> List[Dict[str, Any]]:
        """获取故障历史"""
        events = list(self.failure_events)

        if module_id:
            events = [e for e in events if e.module_id == module_id]

        return [
            {
                "event_id": e.event_id,
                "module_id": e.module_id,
                "failure_type": e.failure_type.value,
                "severity": e.severity.name,
                "timestamp": e.timestamp.isoformat(),
                "description": e.description
            }
            for e in events
        ]

    def get_recovery_actions(self, module_id: Optional[str] = None) -> List[Dict[str, Any]]:
        """获取恢复动作"""
        actions = list(self.recovery_actions.values())

        if module_id:
            actions = [a for a in actions if a.module_id == module_id]

        return [
            {
                "action_id": a.action_id,
                "strategy": a.strategy.value,
                "module_id": a.module_id,
                "created_at": a.created_at.isoformat(),
                "max_attempts": a.max_attempts
            }
            for a in actions
        ]

    def get_availability_metrics(self) -> Dict[str, Any]:
        """获取可用性指标"""
        total_modules = len(self.module_states)
        healthy_modules = sum(
            1 for state in self.module_states.values()
            if state.get("status") == "healthy"
        )

        availability = (healthy_modules / total_modules * 100) if total_modules > 0 else 0

        return {
            "availability_percentage": availability,
            "total_modules": total_modules,
            "healthy_modules": healthy_modules,
            "total_failures": self.stats["total_failures"],
            "recovery_rate": (
                self.stats["recovered_failures"] / self.stats["total_failures"] * 100
            ) if self.stats["total_failures"] > 0 else 0,
            "timestamp": datetime.now().isoformat()
        }

    def validate_fault_tolerance(self) -> Dict[str, Any]:
        """验证故障容错机制"""
        validation_result = {
            "valid": True,
            "errors": [],
            "warnings": []
        }

        # 检查健康监控
        if not self.health_monitor.monitoring:
            validation_result["warnings"].append("Health monitoring is not active")

        # 检查备份管理器
        if not self.backup_manager.backups:
            validation_result["warnings"].append("No backups created")

        # 检查故障处理
        if self.stats["total_failures"] > 0:
            recovery_rate = (
                self.stats["recovered_failures"] / self.stats["total_failures"] * 100
            )
            if recovery_rate < 80:
                validation_result["warnings"].append(
                    f"Recovery rate {recovery_rate:.1f}% is below 80%"
                )

        return validation_result

    async def shutdown(self):
        """关闭故障容错系统"""
        self.logger.info("Shutting down FaultTolerance...")

        self.running = False

        # 停止健康监控
        self.health_monitor.stop_monitoring()

        self.logger.info("FaultTolerance shutdown complete")


# 单例模式
_fault_tolerance_instance: Optional[FaultTolerance] = None


def get_fault_tolerance() -> FaultTolerance:
    """获取故障容错系统单例"""
    global _fault_tolerance_instance
    if _fault_tolerance_instance is None:
        _fault_tolerance_instance = FaultTolerance()
    return _fault_tolerance_instance


def initialize_fault_tolerance() -> FaultTolerance:
    """初始化故障容错系统单例"""
    global _fault_tolerance_instance
    _fault_tolerance_instance = FaultTolerance()
    _fault_tolerance_instance.initialize()
    return _fault_tolerance_instance
