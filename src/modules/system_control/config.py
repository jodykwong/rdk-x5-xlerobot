"""
系统控制模块配置 - SystemControlConfig
========================================

所有系统控制模块的集中配置管理。
避免代码中的魔法数字，提供统一的配置接口。

Author: BMad System Architect
Created: 2025-11-05
"""

from typing import Dict, Any, Optional
from dataclasses import dataclass, field
import os
import json


@dataclass
class SystemControlConfig:
    """系统控制模块配置类"""

    # =========================================================================
    # 线程池配置 (Thread Pool Configuration)
    # =========================================================================
    DEFAULT_THREAD_POOL_SIZE: int = 4
    MAX_THREAD_POOL_SIZE: int = 16
    THREAD_POOL_KEEPALIVE_SECONDS: int = 60

    # =========================================================================
    # 消息队列配置 (Message Queue Configuration)
    # =========================================================================
    DEFAULT_QUEUE_SIZE: int = 1000
    MAX_QUEUE_SIZE: int = 10000
    QUEUE_HIGH_WATERMARK: int = 800
    QUEUE_LOW_WATERMARK: int = 200
    MESSAGE_TIMEOUT_SECONDS: float = 5.0

    # =========================================================================
    # 性能阈值配置 (Performance Thresholds)
    # =========================================================================
    LATENCY_THRESHOLD_MS: float = 10.0  # 模块通信延迟阈值
    LATENCY_P99_THRESHOLD_MS: float = 50.0  # P99延迟阈值
    THROUGHPUT_MIN_MESSAGES_PER_SECOND: int = 1000  # 最小吞吐量
    SYSTEM_AVAILABILITY_THRESHOLD: float = 99.9  # 系统可用性阈值
    MODULE_STARTUP_TIMEOUT_SECONDS: float = 30.0  # 模块启动超时

    # =========================================================================
    # 健康检查配置 (Health Check Configuration)
    # =========================================================================
    HEALTH_CHECK_INTERVAL_SECONDS: float = 5.0
    HEALTH_CHECK_TIMEOUT_SECONDS: float = 1.0
    HEALTH_CHECK_CONSECUTIVE_FAILURES: int = 3
    SYSTEM_HEALTH_THRESHOLD: float = 80.0

    # =========================================================================
    # 故障容错配置 (Fault Tolerance Configuration)
    # =========================================================================
    RETRY_MAX_ATTEMPTS: int = 3
    RETRY_BACKOFF_MULTIPLIER: float = 2.0
    RETRY_INITIAL_DELAY_SECONDS: float = 1.0
    CIRCUIT_BREAKER_FAILURE_THRESHOLD: int = 5
    CIRCUIT_BREAKER_RECOVERY_TIMEOUT_SECONDS: float = 60.0
    CIRCUIT_BREAKER_HALF_OPEN_MAX_CALLS: int = 3

    # =========================================================================
    # 备份和恢复配置 (Backup and Recovery Configuration)
    # =========================================================================
    BACKUP_ENABLED: bool = True
    BACKUP_DIRECTORY: str = os.path.expanduser("~/.xlerobot/backups")
    BACKUP_RETENTION_DAYS: int = 30
    BACKUP_COMPRESSION_ENABLED: bool = True
    BACKUP_ENCRYPTION_ENABLED: bool = False
    AUTO_BACKUP_INTERVAL_HOURS: int = 24
    MAX_BACKUP_SIZE_MB: int = 1000

    # =========================================================================
    # 可扩展性配置 (Scalability Configuration)
    # =========================================================================
    HORIZONTAL_SCALE_THRESHOLD: int = 80  # CPU使用率阈值
    MIN_SCALE_INSTANCES: int = 1
    MAX_SCALE_INSTANCES: int = 10
    SCALE_COOLDOWN_SECONDS: float = 300.0  # 5分钟
    PLUGIN_LOAD_TIMEOUT_SECONDS: float = 10.0
    MAX_PLUGINS: int = 50

    # =========================================================================
    # ROS2配置 (ROS2 Configuration)
    # =========================================================================
    ROS2_QOS_RELIABILITY: str = "RELIABLE"
    ROS2_QOS_DURABILITY: str = "VOLATILE"
    ROS2_MAX_PUBLISHERS: int = 10
    ROS2_MAX_SUBSCRIBERS: int = 10
    ROS2_CALLBACK_GROUP_TYPE: str = "ReentrantCallbackGroup"

    # =========================================================================
    # 日志配置 (Logging Configuration)
    # =========================================================================
    LOG_LEVEL: str = "INFO"
    LOG_FORMAT: str = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    LOG_FILE_ENABLED: bool = True
    LOG_FILE_PATH: str = os.path.expanduser("~/.xlerobot/logs/system_control.log")
    LOG_FILE_MAX_SIZE_MB: int = 100
    LOG_FILE_BACKUP_COUNT: int = 5
    LOG_ROTATION_ENABLED: bool = True

    # =========================================================================
    # 监控配置 (Monitoring Configuration)
    # =========================================================================
    METRICS_ENABLED: bool = True
    METRICS_COLLECTION_INTERVAL_SECONDS: float = 1.0
    METRICS_RETENTION_HOURS: int = 168  # 7天
    PERFORMANCE_MONITORING_ENABLED: bool = True
    ALERT_THRESHOLD_CPU_PERCENT: float = 80.0
    ALERT_THRESHOLD_MEMORY_PERCENT: float = 85.0

    # =========================================================================
    # 安全配置 (Security Configuration)
    # =========================================================================
    PLUGIN_SIGNATURE_VERIFICATION_ENABLED: bool = True
    PLUGIN_SANDBOXING_ENABLED: bool = True
    PLUGIN_WHITELIST_ENABLED: bool = True
    MAX_PLUGIN_EXECUTION_TIME_SECONDS: float = 30.0
    ENCRYPTION_ALGORITHM: str = "AES-256"
    SESSION_TIMEOUT_MINUTES: int = 60

    # =========================================================================
    # 开发配置 (Development Configuration)
    # =========================================================================
    DEBUG_MODE: bool = False
    MOCK_MODE: bool = False
    PROFILING_ENABLED: bool = False
    TEST_MODE: bool = False

    def __post_init__(self):
        """初始化后处理"""
        # 创建必要的目录
        self._create_directories()

        # 验证配置
        self._validate_config()

    def _create_directories(self):
        """创建必要的目录"""
        import os
        directories = [
            self.BACKUP_DIRECTORY,
            os.path.dirname(self.LOG_FILE_PATH)
        ]

        for directory in directories:
            if directory:
                os.makedirs(directory, exist_ok=True)

    def _validate_config(self):
        """验证配置参数"""
        errors = []

        # 验证数值范围
        if self.DEFAULT_THREAD_POOL_SIZE <= 0:
            errors.append("DEFAULT_THREAD_POOL_SIZE must be positive")
        if self.MAX_THREAD_POOL_SIZE < self.DEFAULT_THREAD_POOL_SIZE:
            errors.append("MAX_THREAD_POOL_SIZE must be >= DEFAULT_THREAD_POOL_SIZE")
        if self.LATENCY_THRESHOLD_MS <= 0:
            errors.append("LATENCY_THRESHOLD_MS must be positive")
        if self.DEFAULT_QUEUE_SIZE <= 0:
            errors.append("DEFAULT_QUEUE_SIZE must be positive")
        if self.RETRY_MAX_ATTEMPTS <= 0:
            errors.append("RETRY_MAX_ATTEMPTS must be positive")

        if errors:
            raise ValueError(f"Configuration validation failed: {'; '.join(errors)}")

    def get_thread_pool_config(self) -> Dict[str, Any]:
        """获取线程池配置"""
        return {
            "max_workers": self.DEFAULT_THREAD_POOL_SIZE,
            "thread_name_prefix": "SystemControl",
            "keepalive_seconds": self.THREAD_POOL_KEEPALIVE_SECONDS
        }

    def get_queue_config(self) -> Dict[str, Any]:
        """获取队列配置"""
        return {
            "max_size": self.DEFAULT_QUEUE_SIZE,
            "high_watermark": self.QUEUE_HIGH_WATERMARK,
            "low_watermark": self.QUEUE_LOW_WATERMARK,
            "timeout_seconds": self.MESSAGE_TIMEOUT_SECONDS
        }

    def get_performance_thresholds(self) -> Dict[str, Any]:
        """获取性能阈值配置"""
        return {
            "latency_ms": self.LATENCY_THRESHOLD_MS,
            "latency_p99_ms": self.LATENCY_P99_THRESHOLD_MS,
            "throughput_min_msgs_per_sec": self.THROUGHPUT_MIN_MESSAGES_PER_SECOND,
            "availability_threshold": self.SYSTEM_AVAILABILITY_THRESHOLD,
            "startup_timeout_sec": self.MODULE_STARTUP_TIMEOUT_SECONDS
        }

    def get_fault_tolerance_config(self) -> Dict[str, Any]:
        """获取故障容错配置"""
        return {
            "retry_max_attempts": self.RETRY_MAX_ATTEMPTS,
            "retry_backoff_multiplier": self.RETRY_BACKOFF_MULTIPLIER,
            "retry_initial_delay_sec": self.RETRY_INITIAL_DELAY_SECONDS,
            "circuit_breaker_failure_threshold": self.CIRCUIT_BREAKER_FAILURE_THRESHOLD,
            "circuit_breaker_recovery_timeout_sec": self.CIRCUIT_BREAKER_RECOVERY_TIMEOUT_SECONDS,
            "circuit_breaker_half_open_max_calls": self.CIRCUIT_BREAKER_HALF_OPEN_MAX_CALLS
        }

    def get_backup_config(self) -> Dict[str, Any]:
        """获取备份配置"""
        return {
            "enabled": self.BACKUP_ENABLED,
            "directory": self.BACKUP_DIRECTORY,
            "retention_days": self.BACKUP_RETENTION_DAYS,
            "compression_enabled": self.BACKUP_COMPRESSION_ENABLED,
            "encryption_enabled": self.BACKUP_ENCRYPTION_ENABLED,
            "auto_backup_interval_hours": self.AUTO_BACKUP_INTERVAL_HOURS,
            "max_backup_size_mb": self.MAX_BACKUP_SIZE_MB
        }

    def get_scalability_config(self) -> Dict[str, Any]:
        """获取可扩展性配置"""
        return {
            "horizontal_scale_threshold": self.HORIZONTAL_SCALE_THRESHOLD,
            "min_instances": self.MIN_SCALE_INSTANCES,
            "max_instances": self.MAX_SCALE_INSTANCES,
            "scale_cooldown_sec": self.SCALE_COOLDOWN_SECONDS,
            "plugin_load_timeout_sec": self.PLUGIN_LOAD_TIMEOUT_SECONDS,
            "max_plugins": self.MAX_PLUGINS
        }

    def get_security_config(self) -> Dict[str, Any]:
        """获取安全配置"""
        return {
            "plugin_signature_verification_enabled": self.PLUGIN_SIGNATURE_VERIFICATION_ENABLED,
            "plugin_sandboxing_enabled": self.PLUGIN_SANDBOXING_ENABLED,
            "plugin_whitelist_enabled": self.PLUGIN_WHITELIST_ENABLED,
            "max_plugin_execution_time_sec": self.MAX_PLUGIN_EXECUTION_TIME_SECONDS,
            "encryption_algorithm": self.ENCRYPTION_ALGORITHM,
            "session_timeout_minutes": self.SESSION_TIMEOUT_MINUTES
        }

    def update_from_file(self, config_path: str):
        """从文件更新配置"""
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                config_data = json.load(f)
                for key, value in config_data.items():
                    if hasattr(self, key):
                        setattr(self, key, value)
                        self._validate_config()
        else:
            raise FileNotFoundError(f"Configuration file not found: {config_path}")

    def save_to_file(self, config_path: str):
        """保存配置到文件"""
        config_data = {
            key: getattr(self, key)
            for key in dir(self)
            if not key.startswith('_') and not callable(getattr(self, key))
        }

        with open(config_path, 'w') as f:
            json.dump(config_data, f, indent=2)


# 全局配置实例
CONFIG = SystemControlConfig()
