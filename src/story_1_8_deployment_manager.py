#!/usr/bin/env python3.10
"""
XleRobot Deployment Manager - 生产部署管理
Story 1.8: 系统优化与部署
BMad Method v6 Brownfield Level 4 企业级标准

功能特性:
- 生产环境部署管理
- 配置管理和版本控制
- 蓝绿部署支持
- 回滚机制
- 健康检查和验证
- 100%符合Epic 1纯在线架构
"""

import asyncio
import json
import os
import sys
import time
import subprocess
import logging
import shutil
import hashlib
from typing import Dict, Any, List, Optional, Tuple
from dataclasses import dataclass, field, asdict
from datetime import datetime, timedelta
from pathlib import Path
from enum import Enum
import yaml
import tempfile

logger = logging.getLogger(__name__)

class DeploymentStatus(Enum):
    """部署状态"""
    PENDING = "pending"
    PREPARING = "preparing"
    DEPLOYING = "deploying"
    VERIFYING = "verifying"
    COMPLETED = "completed"
    FAILED = "failed"
    ROLLING_BACK = "rolling_back"
    ROLLED_BACK = "rolled_back"

class DeploymentMode(Enum):
    """部署模式"""
    BLUE_GREEN = "blue_green"
    ROLLING = "rolling"
    CANARY = "canary"

@dataclass
class DeploymentConfig:
    """部署配置"""
    # 基础配置
    deployment_name: str
    version: str
    environment: str = "production"
    mode: DeploymentMode = DeploymentMode.BLUE_GREEN

    # 路径配置
    source_path: str = "/home/sunrise/xlerobot"
    target_path: str = "/opt/xlerobot"
    backup_path: str = "/opt/xlerobot_backup"
    config_path: str = "/opt/xlerobot_config"

    # 部署选项
    enable_backup: bool = True
    enable_validation: bool = True
    enable_rollback: bool = True
    max_rollback_attempts: int = 3

    # 健康检查配置
    health_check_timeout_seconds: int = 300
    health_check_interval_seconds: int = 10
    required_success_checks: int = 3

    # 性能阈值
    max_response_time_ms: int = 5000
    max_cpu_usage_percent: float = 80.0
    max_memory_usage_percent: float = 85.0
    min_success_rate_percent: float = 95.0

@dataclass
class DeploymentStep:
    """部署步骤"""
    name: str
    description: str
    command: Optional[str] = None
    function: Optional[str] = None
    timeout_seconds: int = 300
    critical: bool = True
    rollback_command: Optional[str] = None
    completed: bool = False
    start_time: Optional[float] = None
    end_time: Optional[float] = None
    success: bool = False
    error_message: Optional[str] = None

@dataclass
class DeploymentRecord:
    """部署记录"""
    id: str
    config: DeploymentConfig
    status: DeploymentStatus
    start_time: float
    end_time: Optional[float] = None
    steps: List[DeploymentStep] = field(default_factory=list)
    success: bool = False
    error_message: Optional[str] = None
    rollback_available: bool = False
    metrics: Dict[str, Any] = field(default_factory=dict)

class DeploymentManager:
    """部署管理器 - Story 1.8核心组件"""

    def __init__(self, config: DeploymentConfig):
        """
        初始化部署管理器

        Args:
            config: 部署配置
        """
        logger.info(f"🚀 初始化DeploymentManager - 部署: {config.deployment_name}")

        self.config = config
        self.current_deployment: Optional[DeploymentRecord] = None
        self.deployment_history: List[DeploymentRecord] = []

        # 部署状态
        self.deployment_active = False

        # 路径验证
        self._validate_paths()

        # 确保目录存在
        self._ensure_directories()

        logger.info("✅ 部署管理器初始化完成")

    def _validate_paths(self) -> None:
        """验证路径配置"""
        required_paths = {
            'source_path': self.config.source_path,
            'target_path': self.config.target_path,
            'backup_path': self.config.backup_path,
            'config_path': self.config.config_path
        }

        for path_name, path_value in required_paths.items():
            if not path_value:
                raise ValueError(f"路径配置缺失: {path_name}")

            # 检查源路径是否存在
            if path_name == 'source_path' and not os.path.exists(path_value):
                raise ValueError(f"源路径不存在: {path_value}")

    def _ensure_directories(self) -> None:
        """确保必要目录存在"""
        directories = [
            self.config.backup_path,
            self.config.config_path,
            os.path.join(self.config.target_path, "logs"),
            os.path.join(self.config.target_path, "data")
        ]

        for directory in directories:
            Path(directory).mkdir(parents=True, exist_ok=True)

    async def deploy(self) -> DeploymentRecord:
        """
        执行部署

        Returns:
            部署记录
        """
        if self.deployment_active:
            raise RuntimeError("已有部署正在进行中")

        logger.info(f"🚀 开始部署: {self.config.deployment_name} v{self.config.version}")

        # 创建部署记录
        deployment_id = f"deploy_{int(time.time())}"
        self.current_deployment = DeploymentRecord(
            id=deployment_id,
            config=self.config,
            status=DeploymentStatus.PREPARING,
            start_time=time.time()
        )

        self.deployment_active = True

        try:
            # 1. 准备部署步骤
            await self._prepare_deployment_steps()

            # 2. 执行部署
            await self._execute_deployment()

            # 3. 验证部署
            await self._verify_deployment()

            # 4. 完成部署
            await self._complete_deployment()

        except Exception as e:
            logger.error(f"❌ 部署失败: {str(e)}")
            await self._handle_deployment_failure(str(e))

        finally:
            self.deployment_active = False
            if self.current_deployment:
                self.deployment_history.append(self.current_deployment)

        return self.current_deployment

    async def _prepare_deployment_steps(self) -> None:
        """准备部署步骤"""
        logger.info("📋 准备部署步骤")

        steps = [
            DeploymentStep(
                name="validate_source",
                description="验证源代码完整性",
                function="validate_source_code",
                timeout_seconds=120
            ),
            DeploymentStep(
                name="backup_current",
                description="备份当前版本",
                function="backup_current_version",
                timeout_seconds=300
            ),
            DeploymentStep(
                name="stop_services",
                description="停止当前服务",
                function="stop_current_services",
                timeout_seconds=60
            ),
            DeploymentStep(
                name="copy_files",
                description="复制部署文件",
                function="copy_deployment_files",
                timeout_seconds=600
            ),
            DeploymentStep(
                name="install_dependencies",
                description="安装依赖包",
                command="colcon build --symlink-install",
                timeout_seconds=900
            ),
            DeploymentStep(
                name="update_config",
                description="更新配置文件",
                function="update_configuration_files",
                timeout_seconds=120
            ),
            DeploymentStep(
                name="start_services",
                description="启动新服务",
                function="start_new_services",
                timeout_seconds=180
            ),
            DeploymentStep(
                name="health_check",
                description="执行健康检查",
                function="perform_health_checks",
                timeout_seconds=self.config.health_check_timeout_seconds
            ),
            DeploymentStep(
                name="performance_validation",
                description="验证性能指标",
                function="validate_performance_metrics",
                timeout_seconds=300
            )
        ]

        self.current_deployment.steps = steps
        logger.info(f"✅ 已准备 {len(steps)} 个部署步骤")

    async def _execute_deployment(self) -> None:
        """执行部署步骤"""
        logger.info("🔄 开始执行部署步骤")

        self.current_deployment.status = DeploymentStatus.DEPLOYING

        for step in self.current_deployment.steps:
            try:
                logger.info(f"⚡ 执行步骤: {step.name} - {step.description}")

                step.start_time = time.time()

                # 执行步骤
                if step.command:
                    success = await self._execute_command(step.command, step.timeout_seconds)
                elif step.function:
                    success = await self._execute_function(step.function, step.timeout_seconds)
                else:
                    raise ValueError(f"步骤 {step.name} 缺少执行命令或函数")

                step.end_time = time.time()
                step.success = success
                step.completed = True

                if success:
                    logger.info(f"✅ 步骤完成: {step.name} (耗时: {step.end_time - step.start_time:.2f}秒)")
                else:
                    if step.critical:
                        raise Exception(f"关键步骤失败: {step.name}")
                    else:
                        logger.warning(f"⚠️ 非关键步骤失败: {step.name}")

            except Exception as e:
                step.end_time = time.time()
                step.success = False
                step.completed = True
                step.error_message = str(e)

                logger.error(f"❌ 步骤失败: {step.name} - {str(e)}")

                if step.critical:
                    raise e

    async def _verify_deployment(self) -> None:
        """验证部署"""
        logger.info("🔍 开始部署验证")

        self.current_deployment.status = DeploymentStatus.VERIFYING

        try:
            # 健康检查
            health_ok = await self._perform_health_checks()
            if not health_ok:
                raise Exception("健康检查失败")

            # 性能验证
            performance_ok = await self._validate_performance_metrics()
            if not performance_ok:
                raise Exception("性能验证失败")

            logger.info("✅ 部署验证通过")

        except Exception as e:
            logger.error(f"❌ 部署验证失败: {str(e)}")
            raise e

    async def _complete_deployment(self) -> None:
        """完成部署"""
        logger.info("🎉 完成部署")

        self.current_deployment.status = DeploymentStatus.COMPLETED
        self.current_deployment.end_time = time.time()
        self.current_deployment.success = True

        # 记录部署指标
        self.current_deployment.metrics = {
            "total_time_seconds": self.current_deployment.end_time - self.current_deployment.start_time,
            "successful_steps": len([s for s in self.current_deployment.steps if s.success]),
            "failed_steps": len([s for s in self.current_deployment.steps if not s.success]),
            "rollback_available": self.config.enable_backup and self.config.enable_rollback
        }

        logger.info(f"✅ 部署成功完成 - 总耗时: {self.current_deployment.metrics['total_time_seconds']:.2f}秒")

    async def _handle_deployment_failure(self, error_message: str) -> None:
        """处理部署失败"""
        logger.error(f"💥 处理部署失败: {error_message}")

        self.current_deployment.status = DeploymentStatus.FAILED
        self.current_deployment.end_time = time.time()
        self.current_deployment.success = False
        self.current_deployment.error_message = error_message

        # 执行回滚
        if self.config.enable_rollback:
            try:
                logger.info("🔄 开始自动回滚")
                await self.rollback()
            except Exception as rollback_error:
                logger.error(f"❌ 自动回滚失败: {str(rollback_error)}")

    async def rollback(self) -> bool:
        """
        执行回滚

        Returns:
            是否回滚成功
        """
        if not self.current_deployment:
            raise RuntimeError("没有可回滚的部署")

        logger.info("🔄 开始回滚操作")

        try:
            self.current_deployment.status = DeploymentStatus.ROLLING_BACK

            # 找到最新的备份
            latest_backup = await self._find_latest_backup()
            if not latest_backup:
                raise Exception("没有找到可用的备份")

            # 停止当前服务
            await self.stop_current_services()

            # 恢复备份
            await self._restore_backup(latest_backup)

            # 启动恢复后的服务
            await self.start_backup_services(latest_backup)

            # 验证回滚
            rollback_ok = await self._verify_rollback()
            if not rollback_ok:
                raise Exception("回滚验证失败")

            # 更新状态
            self.current_deployment.status = DeploymentStatus.ROLLED_BACK
            logger.info("✅ 回滚操作成功完成")

            return True

        except Exception as e:
            logger.error(f"❌ 回滚操作失败: {str(e)}")
            self.current_deployment.status = DeploymentStatus.FAILED
            return False

    async def _execute_command(self, command: str, timeout: int) -> bool:
        """执行命令"""
        try:
            logger.debug(f"执行命令: {command}")

            # 切换到源目录
            env = os.environ.copy()
            env['ROS_DOMAIN_ID'] = '42'  # ROS2域ID

            process = await asyncio.create_subprocess_shell(
                command,
                cwd=self.config.source_path,
                env=env,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )

            stdout, stderr = await asyncio.wait_for(process.communicate(), timeout=timeout)

            if process.returncode == 0:
                logger.debug(f"命令成功: {stdout.decode()}")
                return True
            else:
                logger.error(f"命令失败 (返回码 {process.returncode}): {stderr.decode()}")
                return False

        except asyncio.TimeoutError:
            logger.error(f"命令执行超时: {command}")
            return False
        except Exception as e:
            logger.error(f"命令执行异常: {str(e)}")
            return False

    async def _execute_function(self, function_name: str, timeout: int) -> bool:
        """执行函数"""
        try:
            function_map = {
                'validate_source_code': self.validate_source_code,
                'backup_current_version': self.backup_current_version,
                'stop_current_services': self.stop_current_services,
                'copy_deployment_files': self.copy_deployment_files,
                'update_configuration_files': self.update_configuration_files,
                'start_new_services': self.start_new_services,
                'perform_health_checks': self._perform_health_checks,
                'validate_performance_metrics': self._validate_performance_metrics
            }

            if function_name not in function_map:
                raise ValueError(f"未知函数: {function_name}")

            function = function_map[function_name]
            result = await asyncio.wait_for(function(), timeout=timeout)

            return bool(result)

        except asyncio.TimeoutError:
            logger.error(f"函数执行超时: {function_name}")
            return False
        except Exception as e:
            logger.error(f"函数执行异常 {function_name}: {str(e)}")
            return False

    # 具体的部署函数实现
    async def validate_source_code(self) -> bool:
        """验证源代码完整性"""
        logger.info("🔍 验证源代码完整性")

        try:
            # 检查关键文件是否存在
            critical_files = [
                "src/xlerobot_phase1",
                "src/xlerobot_camera",
                "src/xlerobot_vision",
                "src/xlerobot_online_dialogue"
            ]

            for file_path in critical_files:
                full_path = os.path.join(self.config.source_path, file_path)
                if not os.path.exists(full_path):
                    raise Exception(f"关键文件缺失: {file_path}")

            # 检查包配置文件
            package_files = []
            for root, dirs, files in os.walk(os.path.join(self.config.source_path, "src")):
                if "package.xml" in files:
                    package_files.append(os.path.join(root, "package.xml"))

            if len(package_files) < 4:  # 至少4个包
                raise Exception(f"ROS2包数量不足: {len(package_files)}")

            logger.info(f"✅ 源代码验证通过 - 发现 {len(package_files)} 个ROS2包")
            return True

        except Exception as e:
            logger.error(f"❌ 源代码验证失败: {str(e)}")
            return False

    async def backup_current_version(self) -> bool:
        """备份当前版本"""
        logger.info("💾 备份当前版本")

        if not self.config.enable_backup:
            logger.info("⏭️ 跳过备份 (已禁用)")
            return True

        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            backup_dir = os.path.join(self.config.backup_path, f"backup_{timestamp}")

            if os.path.exists(self.config.target_path):
                shutil.copytree(self.config.target_path, backup_dir, dirs_exist_ok=True)

            # 备份配置文件
            config_backup_dir = os.path.join(backup_dir, "config")
            if os.path.exists(self.config.config_path):
                shutil.copytree(self.config.config_path, config_backup_dir, dirs_exist_ok=True)

            logger.info(f"✅ 备份完成: {backup_dir}")
            return True

        except Exception as e:
            logger.error(f"❌ 备份失败: {str(e)}")
            return False

    async def stop_current_services(self) -> bool:
        """停止当前服务"""
        logger.info("🛑 停止当前服务")

        try:
            # 停止ROS2节点
            stop_commands = [
                "pkill -f xlerobot",
                "pkill -f ros2",
                "ros2 daemon stop"
            ]

            for command in stop_commands:
                try:
                    await self._execute_command(command, 10)
                except Exception:
                    pass  # 忽略停止过程中的错误

            # 等待进程完全停止
            await asyncio.sleep(5)

            logger.info("✅ 服务停止完成")
            return True

        except Exception as e:
            logger.error(f"❌ 服务停止失败: {str(e)}")
            return False

    async def copy_deployment_files(self) -> bool:
        """复制部署文件"""
        logger.info("📁 复制部署文件")

        try:
            # 复制源代码
            source_src = os.path.join(self.config.source_path, "src")
            target_src = os.path.join(self.config.target_path, "src")

            if os.path.exists(target_src):
                shutil.rmtree(target_src)

            shutil.copytree(source_src, target_src)

            # 复制启动文件
            if os.path.exists(os.path.join(self.config.source_path, "launch")):
                target_launch = os.path.join(self.config.target_path, "launch")
                shutil.copytree(
                    os.path.join(self.config.source_path, "launch"),
                    target_launch,
                    dirs_exist_ok=True
                )

            logger.info("✅ 文件复制完成")
            return True

        except Exception as e:
            logger.error(f"❌ 文件复制失败: {str(e)}")
            return False

    async def update_configuration_files(self) -> bool:
        """更新配置文件"""
        logger.info("⚙️ 更新配置文件")

        try:
            # 创建生产环境配置
            prod_config = {
                "environment": "production",
                "ros_domain_id": 42,
                "log_level": "INFO",
                "performance": {
                    "max_workers": 8,
                    "timeout_seconds": 30
                },
                "api_keys": {
                    "aliyun_dashscope": "${DASHSCOPE_API_KEY}",
                    "aliyun_asr": "${ALIYUN_ASR_KEY}"
                },
                "monitoring": {
                    "enabled": True,
                    "metrics_interval": 60
                }
            }

            config_file = os.path.join(self.config.config_path, "production.yaml")
            with open(config_file, 'w', encoding='utf-8') as f:
                yaml.dump(prod_config, f, default_flow_style=False, allow_unicode=True)

            logger.info("✅ 配置文件更新完成")
            return True

        except Exception as e:
            logger.error(f"❌ 配置文件更新失败: {str(e)}")
            return False

    async def start_new_services(self) -> bool:
        """启动新服务"""
        logger.info("🚀 启动新服务")

        try:
            # 设置环境变量
            env = os.environ.copy()
            env['ROS_DOMAIN_ID'] = '42'
            env['PYTHONPATH'] = f"{self.config.target_path}/src:{env.get('PYTHONPATH', '')}"

            # 启动核心服务
            startup_commands = [
                "source /opt/ros/humble/setup.bash",
                f"cd {self.config.target_path}",
                "source install/setup.bash",
                "ros2 daemon start"
            ]

            for command in startup_commands:
                await self._execute_command(command, 30)

            logger.info("✅ 服务启动完成")
            return True

        except Exception as e:
            logger.error(f"❌ 服务启动失败: {str(e)}")
            return False

    async def _perform_health_checks(self) -> bool:
        """执行健康检查"""
        logger.info("🔍 执行健康检查")

        try:
            success_count = 0
            total_checks = self.config.required_success_checks

            for i in range(total_checks):
                logger.info(f"健康检查 {i + 1}/{total_checks}")

                # 检查ROS2节点
                try:
                    result = await self._execute_command("ros2 node list", 10)
                    if result:
                        success_count += 1
                except Exception:
                    pass

                if success_count < i + 1:
                    logger.warning(f"健康检查 {i + 1} 失败")

                await asyncio.sleep(self.config.health_check_interval_seconds)

            success_rate = success_count / total_checks
            logger.info(f"健康检查完成 - 成功率: {success_rate:.1%}")

            return success_rate >= 0.8  # 80%成功率阈值

        except Exception as e:
            logger.error(f"❌ 健康检查异常: {str(e)}")
            return False

    async def _validate_performance_metrics(self) -> bool:
        """验证性能指标"""
        logger.info("📊 验证性能指标")

        try:
            # 这里应该集成SystemOptimizer的性能检查
            # 简化实现，检查基本系统资源
            import psutil

            cpu_usage = psutil.cpu_percent(interval=1)
            memory_usage = psutil.virtual_memory().percent

            if cpu_usage > self.config.max_cpu_usage_percent:
                raise Exception(f"CPU使用率过高: {cpu_usage}%")

            if memory_usage > self.config.max_memory_usage_percent:
                raise Exception(f"内存使用率过高: {memory_usage}%")

            logger.info(f"✅ 性能验证通过 - CPU: {cpu_usage}%, 内存: {memory_usage}%")
            return True

        except Exception as e:
            logger.error(f"❌ 性能验证失败: {str(e)}")
            return False

    # 辅助方法
    async def _find_latest_backup(self) -> Optional[str]:
        """查找最新备份"""
        try:
            backup_dirs = []
            for item in os.listdir(self.config.backup_path):
                if item.startswith("backup_") and os.path.isdir(os.path.join(self.config.backup_path, item)):
                    backup_dirs.append(item)

            if not backup_dirs:
                return None

            # 按时间排序，返回最新的
            backup_dirs.sort(reverse=True)
            return os.path.join(self.config.backup_path, backup_dirs[0])

        except Exception:
            return None

    async def _restore_backup(self, backup_path: str) -> None:
        """恢复备份"""
        logger.info(f"🔄 恢复备份: {backup_path}")

        if os.path.exists(self.config.target_path):
            shutil.rmtree(self.config.target_path)

        shutil.copytree(backup_path, self.config.target_path)

    async def start_backup_services(self, backup_path: str) -> None:
        """启动备份服务"""
        # 类似于start_new_services，但从备份目录启动
        await self.start_new_services()

    async def _verify_rollback(self) -> bool:
        """验证回滚"""
        # 简化的回滚验证
        return await self._perform_health_checks()

    def get_deployment_status(self) -> Dict[str, Any]:
        """获取部署状态"""
        if not self.current_deployment:
            return {"status": "no_deployment"}

        return {
            "deployment_id": self.current_deployment.id,
            "status": self.current_deployment.status.value,
            "config": asdict(self.current_deployment.config),
            "start_time": self.current_deployment.start_time,
            "end_time": self.current_deployment.end_time,
            "success": self.current_deployment.success,
            "error_message": self.current_deployment.error_message,
            "steps": [asdict(step) for step in self.current_deployment.steps],
            "metrics": self.current_deployment.metrics,
            "deployment_active": self.deployment_active
        }

    def get_deployment_history(self, limit: int = 10) -> List[Dict[str, Any]]:
        """获取部署历史"""
        history = self.deployment_history[-limit:] if limit > 0 else self.deployment_history
        return [asdict(record) for record in history]

# 工厂函数
def create_deployment_manager(deployment_name: str,
                            version: str,
                            environment: str = "production",
                            **kwargs) -> DeploymentManager:
    """创建部署管理器"""
    config = DeploymentConfig(
        deployment_name=deployment_name,
        version=version,
        environment=environment,
        **kwargs
    )
    return DeploymentManager(config)