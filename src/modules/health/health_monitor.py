#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
健康监控和自动恢复模块

提供系统健康监控、故障检测和自动恢复功能。
支持服务自动重启、性能监控和异常处理。

作者: Dev Agent
功能: 健康检查、自动恢复、性能监控
"""

import os
import sys
import time
import asyncio
import logging
import threading
import subprocess
import psutil
import signal
import json
from typing import Dict, Any, List, Optional, Callable
from dataclasses import dataclass, field
from datetime import datetime, timedelta
from enum import Enum
import weakref

logger = logging.getLogger(__name__)


class HealthStatus(Enum):
    """健康状态枚举"""
    HEALTHY = "healthy"
    WARNING = "warning"
    CRITICAL = "critical"
    FAILED = "failed"
    RESTARTING = "restarting"


class ServiceType(Enum):
    """服务类型枚举"""
    VOICE_SERVICE = "voice_service"
    TTS_SERVICE = "tts_service"
    LLM_SERVICE = "llm_service"
    VISION_SERVICE = "vision_service"
    MONITOR_SERVICE = "monitor_service"


@dataclass
class HealthCheck:
    """健康检查项目"""
    name: str
    check_func: Callable[[], Dict[str, Any]]
    interval: float = 30.0
    timeout: float = 10.0
    max_failures: int = 3
    critical: bool = False


@dataclass
class ServiceStatus:
    """服务状态"""
    service_name: str
    service_type: ServiceType
    pid: Optional[int] = None
    status: HealthStatus = HealthStatus.FAILED
    last_check: Optional[datetime] = None
    failure_count: int = 0
    restart_count: int = 0
    last_restart: Optional[datetime] = None
    cpu_usage: float = 0.0
    memory_usage: float = 0.0
    uptime: float = 0.0
    error_message: Optional[str] = None
    auto_restart: bool = True
    health_checks: List[HealthCheck] = field(default_factory=list)


class HealthMonitor:
    """
    健康监控和自动恢复系统

    功能特性:
    - 实时健康监控
    - 自动故障检测
    - 服务自动重启
    - 性能监控
    - 异常处理和恢复
    """

    def __init__(
        self,
        check_interval: float = 10.0,
        max_restart_attempts: int = 5,
        restart_delay: float = 5.0,
        enable_auto_recovery: bool = True
    ):
        """
        初始化健康监控器

        Args:
            check_interval: 检查间隔(秒)
            max_restart_attempts: 最大重启尝试次数
            restart_delay: 重启延迟(秒)
            enable_auto_recovery: 是否启用自动恢复
        """
        self.check_interval = check_interval
        self.max_restart_attempts = max_restart_attempts
        self.restart_delay = restart_delay
        self.enable_auto_recovery = enable_auto_recovery

        # 服务管理
        self.services: Dict[str, ServiceStatus] = {}
        self.service_processes: Dict[str, subprocess.Popen] = {}

        # 监控控制
        self.is_running = False
        self.monitor_task: Optional[asyncio.Task] = None
        self.lock = threading.RLock()

        # 统计信息
        self.stats = {
            'total_checks': 0,
            'total_restarts': 0,
            'total_failures': 0,
            'total_recoveries': 0,
            'start_time': datetime.now(),
            'last_check': None,
            'uptime': 0.0
        }

        # 恢复策略
        self.recovery_strategies = {
            ServiceType.VOICE_SERVICE: self._recover_voice_service,
            ServiceType.TTS_SERVICE: self._recover_tts_service,
            ServiceType.LLM_SERVICE: self._recover_llm_service,
            ServiceType.VISION_SERVICE: self._recover_vision_service,
            ServiceType.MONITOR_SERVICE: self._recover_monitor_service
        }

        # 预警阈值
        self.thresholds = {
            'max_cpu_usage': 80.0,
            'max_memory_usage': 85.0,
            'max_failure_count': 5,
            'max_response_time': 30.0
        }

        logger.info("✅ 健康监控器初始化完成")
        logger.info(f"   - 检查间隔: {check_interval}秒")
        logger.info(f"   - 自动恢复: {'启用' if enable_auto_recovery else '禁用'}")

    def register_service(
        self,
        service_name: str,
        service_type: ServiceType,
        pid: Optional[int] = None,
        auto_restart: bool = True,
        health_checks: Optional[List[HealthCheck]] = None
    ):
        """
        注册服务到监控系统

        Args:
            service_name: 服务名称
            service_type: 服务类型
            pid: 进程ID
            auto_restart: 是否自动重启
            health_checks: 健康检查项目
        """
        with self.lock:
            service_status = ServiceStatus(
                service_name=service_name,
                service_type=service_type,
                pid=pid,
                auto_restart=auto_restart,
                health_checks=health_checks or []
            )

            # 如果提供了PID，添加到进程跟踪
            if pid:
                self.service_processes[service_name] = weakref.ref(
                    psutil.Process(pid) if psutil.pid_exists(pid) else None
                )

            self.services[service_name] = service_status

            logger.info(f"📝 注册服务: {service_name} (PID: {pid})")

    def unregister_service(self, service_name: str):
        """取消注册服务"""
        with self.lock:
            if service_name in self.services:
                del self.services[service_name]
                if service_name in self.service_processes:
                    del self.service_processes[service_name]
                logger.info(f"🗑️ 取消注册服务: {service_name}")

    async def start_monitoring(self):
        """启动健康监控"""
        if self.is_running:
            logger.warning("⚠️ 健康监控已在运行")
            return

        self.is_running = True
        self.monitor_task = asyncio.create_task(self._monitoring_loop())

        logger.info("🚀 健康监控已启动")

    async def stop_monitoring(self):
        """停止健康监控"""
        if not self.is_running:
            return

        self.is_running = False

        if self.monitor_task:
            self.monitor_task.cancel()
            try:
                await self.monitor_task
            except asyncio.CancelledError:
                pass

        logger.info("🛑 健康监控已停止")

    async def _monitoring_loop(self):
        """监控循环"""
        while self.is_running:
            try:
                await self._perform_health_checks()
                await self._update_system_stats()
                await asyncio.sleep(self.check_interval)
            except Exception as e:
                logger.error(f"❌ 监控循环错误: {e}")
                await asyncio.sleep(1)

    async def _perform_health_checks(self):
        """执行健康检查"""
        with self.lock:
            self.stats['total_checks'] += 1
            self.stats['last_check'] = datetime.now()

            for service_name, service_status in self.services.items():
                try:
                    await self._check_service_health(service_name, service_status)
                except Exception as e:
                    logger.error(f"❌ 健康检查失败 {service_name}: {e}")
                    self._mark_service_failed(service_name, str(e))

    async def _check_service_health(self, service_name: str, service_status: ServiceStatus):
        """检查单个服务健康状态"""
        current_time = datetime.now()
        service_status.last_check = current_time

        # 1. 进程存活检查
        is_process_alive = self._check_process_alive(service_status)
        if not is_process_alive:
            await self._handle_service_failure(service_name, "进程未运行")
            return

        # 2. 资源使用检查
        cpu_usage, memory_usage = self._get_resource_usage(service_status)
        service_status.cpu_usage = cpu_usage
        service_status.memory_usage = memory_usage

        # 3. 健康检查项目
        all_checks_passed = True
        for health_check in service_status.health_checks:
            try:
                check_result = await asyncio.wait_for(
                    asyncio.to_thread(health_check.check_func),
                    timeout=health_check.timeout
                )

                if not check_result.get('healthy', False):
                    all_checks_passed = False
                    if health_check.critical:
                        service_status.failure_count += 1
                        logger.warning(f"⚠️ 关键检查失败: {health_check.name} for {service_name}")

            except asyncio.TimeoutError:
                all_checks_passed = False
                service_status.failure_count += 1
                logger.warning(f"⚠️ 健康检查超时: {health_check.name} for {service_name}")
            except Exception as e:
                all_checks_passed = False
                service_status.failure_count += 1
                logger.error(f"❌ 健康检查错误: {health_check.name} for {service_name}: {e}")

        # 4. 更新服务状态
        if all_checks_passed and service_status.failure_count == 0:
            if service_status.status != HealthStatus.HEALTHY:
                logger.info(f"✅ 服务恢复正常: {service_name}")
            service_status.status = HealthStatus.HEALTHY
            service_status.failure_count = 0
            service_status.error_message = None

        elif service_status.failure_count >= self.thresholds['max_failure_count']:
            await self._handle_service_failure(service_name, f"连续失败 {service_status.failure_count} 次")

        elif cpu_usage > self.thresholds['max_cpu_usage'] or memory_usage > self.thresholds['max_memory_usage']:
            if service_status.status != HealthStatus.WARNING:
                logger.warning(f"⚠️ 服务资源使用过高: {service_name} (CPU: {cpu_usage:.1f}%, MEM: {memory_usage:.1f}%)")
            service_status.status = HealthStatus.WARNING

        elif service_status.failure_count > 0:
            service_status.status = HealthStatus.WARNING

    def _check_process_alive(self, service_status: ServiceStatus) -> bool:
        """检查进程是否存活"""
        if not service_status.pid:
            return False

        try:
            process = psutil.Process(service_status.pid)
            return process.is_running() and process.status() != psutil.STATUS_ZOMBIE
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            return False

    def _get_resource_usage(self, service_status: ServiceStatus) -> tuple:
        """获取服务资源使用情况"""
        if not service_status.pid:
            return 0.0, 0.0

        try:
            process = psutil.Process(service_status.pid)
            cpu_usage = process.cpu_percent()
            memory_usage = process.memory_percent()
            return cpu_usage, memory_usage
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            return 0.0, 0.0

    async def _handle_service_failure(self, service_name: str, error_message: str):
        """处理服务故障"""
        with self.lock:
            service_status = self.services.get(service_name)
            if not service_status:
                return

            service_status.status = HealthStatus.CRITICAL
            service_status.error_message = error_message
            self.stats['total_failures'] += 1

            logger.error(f"🚨 服务故障: {service_name} - {error_message}")

            # 检查是否需要自动重启
            if (self.enable_auto_recovery and
                service_status.auto_restart and
                service_status.restart_count < self.max_restart_attempts):

                await self._restart_service(service_name, service_status)

    async def _restart_service(self, service_name: str, service_status: ServiceStatus):
        """重启服务"""
        logger.info(f"🔄 开始重启服务: {service_name}")

        service_status.status = HealthStatus.RESTARTING
        service_status.restart_count += 1
        service_status.last_restart = datetime.now()

        try:
            # 停止现有进程
            if service_status.pid:
                await self._stop_process(service_status.pid)

            # 等待一段时间
            await asyncio.sleep(self.restart_delay)

            # 执行恢复策略
            recovery_func = self.recovery_strategies.get(service_status.service_type)
            if recovery_func:
                new_pid = await recovery_func(service_name, service_status)
                if new_pid:
                    service_status.pid = new_pid
                    self.stats['total_restarts'] += 1
                    self.stats['total_recoveries'] += 1
                    logger.info(f"✅ 服务重启成功: {service_name} (新PID: {new_pid})")
                else:
                    logger.error(f"❌ 服务重启失败: {service_name}")

        except Exception as e:
            logger.error(f"❌ 重启过程出错 {service_name}: {e}")
            service_status.status = HealthStatus.FAILED

    async def _stop_process(self, pid: int):
        """停止进程"""
        try:
            process = psutil.Process(pid)
            process.terminate()

            # 等待进程结束
            try:
                process.wait(timeout=10)
            except psutil.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)

            logger.info(f"✅ 进程已停止: PID {pid}")

        except psutil.NoSuchProcess:
            logger.info(f"📝 进程不存在: PID {pid}")
        except Exception as e:
            logger.error(f"❌ 停止进程失败 PID {pid}: {e}")

    # 服务恢复策略
    async def _recover_voice_service(self, service_name: str, service_status: ServiceStatus) -> Optional[int]:
        """恢复语音服务"""
        try:
            script_path = "/home/sunrise/xlerobot/src/modules/health/restart_voice_service.py"
            cmd = [
                sys.executable, script_path,
                "--service-name", service_name,
                "--config", json.dumps({
                    "alibaba_key": os.getenv("ALIBABA_CLOUD_ACCESS_KEY_ID", ""),
                    "alibaba_secret": os.getenv("ALIBABA_CLOUD_ACCESS_KEY_SECRET", ""),
                    "appkey": os.getenv("ALIYUN_NLS_APPKEY", "")
                })
            ]

            proc = subprocess.Popen(cmd, cwd="/home/sunrise/xlerobot")

            # 等待服务启动
            await asyncio.sleep(3)

            if proc.poll() is None:  # 进程还在运行
                return proc.pid
            else:
                logger.error(f"语音服务启动失败，退出码: {proc.returncode}")
                return None

        except Exception as e:
            logger.error(f"语音服务恢复失败: {e}")
            return None

    async def _recover_tts_service(self, service_name: str, service_status: ServiceStatus) -> Optional[int]:
        """恢复TTS服务"""
        # TTS服务通常集成在语音服务中，这里简化处理
        return await self._recover_voice_service(service_name, service_status)

    async def _recover_llm_service(self, service_name: str, service_status: ServiceStatus) -> Optional[int]:
        """恢复LLM服务"""
        try:
            # 创建LLM服务恢复进程
            cmd = [
                sys.executable, "-c",
                """
import sys
sys.path.append('/home/sunrise/xlerobot/src')
from modules.llm.qwen_client import QwenAPIClient, QwenConfig
import time
import os

config = QwenConfig(
    model_name='qwen3-vl-plus',
    api_key=os.getenv('QWEN_API_KEY', '')
)

client = QwenAPIClient(config)
print(f"LLM服务已启动，PID: {os.getpid()}")

try:
    while True:
        time.sleep(10)
        # 定期健康检查
except KeyboardInterrupt:
    print("LLM服务停止")
"""
            ]

            proc = subprocess.Popen(cmd, cwd="/home/sunrise/xlerobot")

            await asyncio.sleep(2)

            if proc.poll() is None:
                return proc.pid
            else:
                return None

        except Exception as e:
            logger.error(f"LLM服务恢复失败: {e}")
            return None

    async def _recover_vision_service(self, service_name: str, service_status: ServiceStatus) -> Optional[int]:
        """恢复视觉服务"""
        # 视觉服务恢复策略
        logger.info(f"视觉服务恢复: {service_name}")
        return None

    async def _recover_monitor_service(self, service_name: str, service_status: ServiceStatus) -> Optional[int]:
        """恢复监控服务"""
        logger.info(f"监控服务恢复: {service_name}")
        return None

    def _mark_service_failed(self, service_name: str, error_message: str):
        """标记服务为失败状态"""
        with self.lock:
            service_status = self.services.get(service_name)
            if service_status:
                service_status.status = HealthStatus.FAILED
                service_status.error_message = error_message
                service_status.failure_count += 1

    async def _update_system_stats(self):
        """更新系统统计"""
        current_time = datetime.now()
        uptime = (current_time - self.stats['start_time']).total_seconds()
        self.stats['uptime'] = uptime

    def get_system_status(self) -> Dict[str, Any]:
        """获取系统状态"""
        with self.lock:
            return {
                'monitoring_active': self.is_running,
                'stats': self.stats.copy(),
                'services': {
                    name: {
                        'status': status.status.value,
                        'pid': status.pid,
                        'cpu_usage': status.cpu_usage,
                        'memory_usage': status.memory_usage,
                        'failure_count': status.failure_count,
                        'restart_count': status.restart_count,
                        'last_restart': status.last_restart.isoformat() if status.last_restart else None,
                        'error_message': status.error_message
                    }
                    for name, status in self.services.items()
                },
                'thresholds': self.thresholds,
                'timestamp': datetime.now().isoformat()
            }

    def get_service_status(self, service_name: str) -> Optional[Dict[str, Any]]:
        """获取特定服务状态"""
        with self.lock:
            service_status = self.services.get(service_name)
            if service_status:
                return {
                    'name': service_status.service_name,
                    'type': service_status.service_type.value,
                    'status': service_status.status.value,
                    'pid': service_status.pid,
                    'cpu_usage': service_status.cpu_usage,
                    'memory_usage': service_status.memory_usage,
                    'failure_count': service_status.failure_count,
                    'restart_count': service_status.restart_count,
                    'last_check': service_status.last_check.isoformat() if service_status.last_check else None,
                    'last_restart': service_status.last_restart.isoformat() if service_status.last_restart else None,
                    'error_message': service_status.error_message,
                    'uptime': service_status.uptime
                }
            return None

    def force_restart_service(self, service_name: str) -> bool:
        """强制重启服务"""
        with self.lock:
            service_status = self.services.get(service_name)
            if not service_status:
                logger.error(f"❌ 服务不存在: {service_name}")
                return False

            # 异步执行重启
            asyncio.create_task(self._restart_service(service_name, service_status))
            return True


# 全局健康监控器实例
global_health_monitor = HealthMonitor()