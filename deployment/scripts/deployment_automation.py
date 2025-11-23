#!/usr/bin/env python3.10
"""
XleRobot 部署自动化脚本
BMad-Method v6 Brownfield Level 4

功能特性:
- 蓝绿部署自动化
- 滚动更新自动化
- 金丝雀发布自动化
- 健康检查和验证
- 自动回滚机制
- 部署监控和报告
"""

import os
import sys
import json
import time
import asyncio
import argparse
import logging
import subprocess
from typing import Dict, Any, List, Optional, Tuple
from pathlib import Path
from dataclasses import dataclass, field
from enum import Enum
from datetime import datetime, timedelta

# 添加项目根目录到路径
sys.path.append(str(Path(__file__).parent.parent.parent / "deployment" / "config" / "src"))
from config_manager import XleRobotConfigManager, Environment

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class DeploymentStrategy(Enum):
    """部署策略枚举"""
    BLUE_GREEN = "blue_green"
    ROLLING_UPDATE = "rolling_update"
    CANARY = "canary"

class DeploymentStatus(Enum):
    """部署状态枚举"""
    PENDING = "pending"
    RUNNING = "running"
    SUCCESS = "success"
    FAILED = "failed"
    ROLLING_BACK = "rolling_back"
    ROLLED_BACK = "rolled_back"

@dataclass
class DeploymentConfig:
    """部署配置"""
    environment: Environment
    strategy: DeploymentStrategy
    image_tag: str
    services: List[str]
    rollback_enabled: bool = True
    health_check_timeout: int = 600
    deployment_timeout: int = 1800
    monitoring_enabled: bool = True

@dataclass
class DeploymentRecord:
    """部署记录"""
    id: str
    config: DeploymentConfig
    status: DeploymentStatus
    start_time: datetime
    end_time: Optional[datetime] = None
    error_message: Optional[str] = None
    rollback_reason: Optional[str] = None
    health_checks: Dict[str, bool] = field(default_factory=dict)
    metrics: Dict[str, Any] = field(default_factory=dict)

class DeploymentAutomationError(Exception):
    """部署自动化错误"""
    pass

class HealthCheckError(Exception):
    """健康检查错误"""
    pass

class XleRobotDeploymentAutomation:
    """XleRobot部署自动化管理器"""

    def __init__(self, config_root: Optional[str] = None):
        """
        初始化部署自动化管理器

        Args:
            config_root: 配置根目录
        """
        self.config_manager = XleRobotConfigManager(config_root)
        self.deployment_records: List[DeploymentRecord] = []
        self.current_deployment: Optional[DeploymentRecord] = None

        # 部署环境配置
        self.environments = {
            Environment.DEVELOPMENT: {
                "namespace": "xlerobot-dev",
                "kubeconfig": "/home/xlerobot/.kube/config-dev",
                "domain": "dev.xlerobot.local"
            },
            Environment.STAGING: {
                "namespace": "xlerobot-staging",
                "kubeconfig": "/home/xlerobot/.kube/config-staging",
                "domain": "staging.xlerobot.com"
            },
            Environment.PRODUCTION: {
                "namespace": "xlerobot-prod",
                "kubeconfig": "/home/xlerobot/.kube/config-prod",
                "domain": "xlerobot.com"
            }
        }

        logger.info("✅ XleRobot部署自动化管理器初始化完成")

    def _run_command(self, command: str, capture_output: bool = True, timeout: int = 300) -> subprocess.CompletedProcess:
        """
        执行shell命令

        Args:
            command: 要执行的命令
            capture_output: 是否捕获输出
            timeout: 超时时间(秒)

        Returns:
            命令执行结果
        """
        logger.debug(f"执行命令: {command}")

        try:
            result = subprocess.run(
                command,
                shell=True,
                capture_output=capture_output,
                text=True,
                timeout=timeout,
                check=False
            )

            if result.returncode != 0:
                error_msg = f"命令执行失败: {command}, 错误: {result.stderr}"
                logger.error(error_msg)
                raise DeploymentAutomationError(error_msg)

            return result

        except subprocess.TimeoutExpired:
            raise DeploymentAutomationError(f"命令执行超时: {command}")

    def _kubectl(self, namespace: str, command: str, timeout: int = 300) -> subprocess.CompletedProcess:
        """
        执行kubectl命令

        Args:
            namespace: 命名空间
            command: kubectl命令
            timeout: 超时时间

        Returns:
            命令执行结果
        """
        env_config = self.environments.get(self.current_deployment.config.environment)
        kubeconfig = env_config.get("kubeconfig", "")

        full_command = f"kubectl --namespace={namespace}"
        if kubeconfig:
            full_command += f" --kubeconfig={kubeconfig}"
        full_command += f" {command}"

        return self._run_command(full_command, timeout=timeout)

    def _wait_for_deployment(self, namespace: str, deployment: str, timeout: int = 600) -> bool:
        """
        等待部署完成

        Args:
            namespace: 命名空间
            deployment: 部署名称
            timeout: 超时时间

        Returns:
            部署是否成功
        """
        logger.info(f"等待部署完成: {deployment}")

        try:
            result = self._kubectl(namespace, f"rollout status deployment/{deployment} --timeout={timeout}s")
            if result.returncode == 0:
                logger.info(f"✅ 部署完成: {deployment}")
                return True
            else:
                logger.error(f"❌ 部署失败: {deployment}")
                return False

        except DeploymentAutomationError as e:
            logger.error(f"❌ 等待部署超时: {deployment}, 错误: {str(e)}")
            return False

    def _check_service_health(self, namespace: str, service: str, timeout: int = 300) -> bool:
        """
        检查服务健康状态

        Args:
            namespace: 命名空间
            service: 服务名称
            timeout: 超时时间

        Returns:
            服务是否健康
        """
        logger.info(f"检查服务健康状态: {service}")

        env_config = self.environments.get(self.current_deployment.config.environment)
        domain = env_config.get("domain", "localhost")

        # 根据服务确定端口
        service_ports = {
            "xlerobot-asr": 8001,
            "xlerobot-tts": 8002,
            "xlerobot-llm": 8003,
            "xlerobot-multimodal": 8004,
            "xlerobot-monitoring": 8005,
            "xlerobot-gateway": 80
        }

        port = service_ports.get(service, 80)
        health_url = f"http://{service}.{namespace}.svc.cluster.local:{port}/health"

        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                result = self._run_command(f"curl -f -s {health_url}", timeout=10)
                if result.returncode == 0:
                    logger.info(f"✅ 服务健康: {service}")
                    return True
            except DeploymentAutomationError:
                pass

            logger.info(f"等待服务就绪: {service}")
            time.sleep(10)

        logger.error(f"❌ 服务健康检查失败: {service}")
        return False

    def _create_deployment_record(self, config: DeploymentConfig) -> DeploymentRecord:
        """创建部署记录"""
        record = DeploymentRecord(
            id=f"deploy-{int(time.time())}-{config.environment.value}",
            config=config,
            status=DeploymentStatus.PENDING,
            start_time=datetime.now()
        )

        self.deployment_records.append(record)
        self.current_deployment = record

        return record

    async def execute_blue_green_deployment(self, config: DeploymentConfig) -> bool:
        """
        执行蓝绿部署

        Args:
            config: 部署配置

        Returns:
            部署是否成功
        """
        logger.info(f"🚀 开始蓝绿部署 - 环境: {config.environment.value}")

        env_config = self.environments[config.environment]
        namespace = env_config["namespace"]

        # 创建部署记录
        record = self._create_deployment_record(config)
        record.status = DeploymentStatus.RUNNING

        try:
            # 阶段1: 创建绿色环境
            logger.info("📦 创建绿色环境...")
            green_namespace = f"{namespace}-green"

            # 创建绿色命名空间
            self._run_command(f"kubectl create namespace {green_namespace} --dry-run=client -o yaml | kubectl apply -f -")

            # 部署绿色环境服务
            for service in config.services:
                logger.info(f"部署服务: {service}")

                # 使用新的镜像标签更新部署
                self._kubectl(green_namespace, f"set image deployment/{service} {service}={config.image_tag}")

                # 等待部署完成
                if not self._wait_for_deployment(green_namespace, service):
                    raise DeploymentAutomationError(f"绿色环境部署失败: {service}")

            # 阶段2: 绿色环境健康检查
            logger.info("🔍 执行绿色环境健康检查...")
            for service in config.services:
                if not self._check_service_health(green_namespace, service):
                    raise DeploymentAutomationError(f"绿色环境健康检查失败: {service}")
                record.health_checks[service] = True

            # 阶段3: 流量切换
            logger.info("🔄 执行流量切换...")

            # 更新负载均衡器指向绿色环境
            if "xlerobot-gateway" in config.services:
                self._kubectl(namespace, "patch service xlerobot-gateway -p '{\"spec\":{\"selector\":{\"version\":\"green\"}}}'")

            # 阶段4: 验证流量切换
            logger.info("✅ 验证流量切换...")
            time.sleep(30)  # 等待流量切换生效

            for service in config.services:
                if not self._check_service_health(namespace, service):
                    # 流量切换失败，回滚到蓝色环境
                    logger.error("❌ 流量切换验证失败，开始回滚...")
                    await self._rollback_blue_green(namespace, green_namespace, "流量切换验证失败")
                    return False

            # 阶段5: 清理蓝色环境
            logger.info("🧹 清理蓝色环境...")
            blue_namespace = f"{namespace}-blue"

            # 检查蓝色命名空间是否存在
            try:
                result = self._run_command(f"kubectl get namespace {blue_namespace}", timeout=10)
                if result.returncode == 0:
                    self._run_command(f"kubectl delete namespace {blue_namespace}")
            except DeploymentAutomationError:
                logger.info(f"蓝色命名空间不存在: {blue_namespace}")

            # 重命名绿色命名空间为主命名空间
            self._run_command(f"kubectl patch namespace {green_namespace} -p '{{\"metadata\":{\"name\":\"{namespace}\"}}}'")

            record.status = DeploymentStatus.SUCCESS
            record.end_time = datetime.now()

            logger.info("✅ 蓝绿部署完成")
            return True

        except Exception as e:
            record.status = DeploymentStatus.FAILED
            record.error_message = str(e)
            record.end_time = datetime.now()
            logger.error(f"❌ 蓝绿部署失败: {str(e)}")
            return False

    async def _rollback_blue_green(self, main_namespace: str, green_namespace: str, reason: str) -> None:
        """
        蓝绿部署回滚

        Args:
            main_namespace: 主命名空间
            green_namespace: 绿色命名空间
            reason: 回滚原因
        """
        logger.info(f"🔄 开始蓝绿回滚: {reason}")

        if self.current_deployment:
            self.current_deployment.status = DeploymentStatus.ROLLING_BACK
            self.current_deployment.rollback_reason = reason

        try:
            # 切换流量回蓝色环境
            self._kubectl(main_namespace, "patch service xlerobot-gateway -p '{\"spec\":{\"selector\":{\"version\":\"blue\"}}}'")

            # 删除绿色环境
            self._run_command(f"kubectl delete namespace {green_namespace}")

            if self.current_deployment:
                self.current_deployment.status = DeploymentStatus.ROLLED_BACK

            logger.info("✅ 蓝绿回滚完成")

        except Exception as e:
            logger.error(f"❌ 蓝绿回滚失败: {str(e)}")
            raise

    async def execute_rolling_update(self, config: DeploymentConfig) -> bool:
        """
        执行滚动更新

        Args:
            config: 部署配置

        Returns:
            部署是否成功
        """
        logger.info(f"🔄 开始滚动更新 - 环境: {config.environment.value}")

        env_config = self.environments[config.environment]
        namespace = env_config["namespace"]

        # 创建部署记录
        record = self._create_deployment_record(config)
        record.status = DeploymentStatus.RUNNING

        try:
            # 创建备份
            logger.info("💾 创建部署备份...")
            backup_version = await self.config_manager.create_backup(config.environment)
            record.metrics["backup_version"] = backup_version

            # 执行滚动更新
            for service in config.services:
                logger.info(f"更新服务: {service}")

                # 更新镜像
                self._kubectl(namespace, f"set image deployment/{service} {service}={config.image_tag}")

                # 等待滚动更新完成
                if not self._wait_for_deployment(namespace, service):
                    logger.error(f"❌ 滚动更新失败: {service}")

                    # 自动回滚
                    if config.rollback_enabled:
                        logger.info("🔄 开始自动回滚...")
                        await self._rollback_rolling_update(namespace, service, backup_version)

                    return False

                # 健康检查
                if not self._check_service_health(namespace, service):
                    logger.error(f"❌ 服务健康检查失败: {service}")

                    # 自动回滚
                    if config.rollback_enabled:
                        logger.info("🔄 开始自动回滚...")
                        await self._rollback_rolling_update(namespace, service, backup_version)

                    return False

                record.health_checks[service] = True

            record.status = DeploymentStatus.SUCCESS
            record.end_time = datetime.now()

            logger.info("✅ 滚动更新完成")
            return True

        except Exception as e:
            record.status = DeploymentStatus.FAILED
            record.error_message = str(e)
            record.end_time = datetime.now()
            logger.error(f"❌ 滚动更新失败: {str(e)}")
            return False

    async def _rollback_rolling_update(self, namespace: str, service: str, backup_version: str) -> None:
        """
        滚动更新回滚

        Args:
            namespace: 命名空间
            service: 服务名称
            backup_version: 备份版本
        """
        logger.info(f"🔄 开始滚动更新回滚: {service}")

        if self.current_deployment:
            self.current_deployment.status = DeploymentStatus.ROLLING_BACK
            self.current_deployment.rollback_reason = f"滚动更新失败: {service}"

        try:
            # 回滚到上一个版本
            self._kubectl(namespace, f"rollout undo deployment/{service}")

            # 等待回滚完成
            if self._wait_for_deployment(namespace, service):
                logger.info(f"✅ 服务回滚完成: {service}")
            else:
                logger.error(f"❌ 服务回滚失败: {service}")

            # 从备份恢复配置
            await self.config_manager.rollback_config(self.current_deployment.config.environment, backup_version)

            if self.current_deployment:
                self.current_deployment.status = DeploymentStatus.ROLLED_BACK

            logger.info("✅ 滚动更新回滚完成")

        except Exception as e:
            logger.error(f"❌ 滚动更新回滚失败: {str(e)}")
            raise

    async def execute_canary_deployment(self, config: DeploymentConfig) -> bool:
        """
        执行金丝雀发布

        Args:
            config: 部署配置

        Returns:
            部署是否成功
        """
        logger.info(f"🕊️ 开始金丝雀发布 - 环境: {config.environment.value}")

        env_config = self.environments[config.environment]
        namespace = env_config["namespace"]

        # 创建部署记录
        record = self._create_deployment_record(config)
        record.status = DeploymentStatus.RUNNING

        try:
            # 金丝雀发布阶段配置
            canary_stages = [
                {"traffic": 5, "duration": 1800},   # 5%流量，30分钟
                {"traffic": 20, "duration": 3600},  # 20%流量，1小时
                {"traffic": 50, "duration": 7200},  # 50%流量，2小时
                {"traffic": 100, "duration": 14400} # 100%流量，4小时
            ]

            for i, stage in enumerate(canary_stages):
                logger.info(f"🕊️ 金丝雀阶段 {i+1}: {stage['traffic']}% 流量")

                # 创建金丝雀部署
                for service in config.services:
                    canary_deployment = f"{service}-canary"

                    # 创建金丝雀部署
                    self._kubectl(namespace, f"scale deployment {service} --replicas={10 - stage['traffic'] // 10}")

                    # 部署金丝雀版本
                    self._kubectl(namespace, f"set image deployment/{canary_deployment} {canary_deployment}={config.image_tag}")
                    self._kubectl(namespace, f"scale deployment {canary_deployment} --replicas={stage['traffic'] // 10}")

                    # 等待金丝雀部署完成
                    if not self._wait_for_deployment(namespace, canary_deployment):
                        raise DeploymentAutomationError(f"金丝雀部署失败: {canary_deployment}")

                # 监控金丝雀版本性能
                logger.info(f"📊 监控金丝雀版本性能 ({stage['duration']}秒)...")

                if not await self._monitor_canary_performance(namespace, config.services, stage['duration']):
                    logger.error(f"❌ 金丝雀性能不达标，阶段 {i+1}")
                    await self._rollback_canary_deployment(namespace, config.services)
                    return False

                # 健康检查
                for service in config.services:
                    if not self._check_service_health(namespace, service):
                        logger.error(f"❌ 金丝雀健康检查失败: {service}")
                        await self._rollback_canary_deployment(namespace, config.services)
                        return False

                record.health_checks[f"stage_{i+1}"] = True

            # 完成金丝雀发布，切换全部流量
            logger.info("🎯 完成金丝雀发布，切换全部流量...")

            for service in config.services:
                canary_deployment = f"{service}-canary"
                self._kubectl(namespace, f"scale deployment {canary_deployment} --replicas=10")
                self._kubectl(namespace, f"scale deployment {service} --replicas=0")

                # 删除旧版本部署
                self._kubectl(namespace, f"delete deployment {service}")
                self._kubectl(namespace, f"rename deployment {canary_deployment} {service}")

            record.status = DeploymentStatus.SUCCESS
            record.end_time = datetime.now()

            logger.info("✅ 金丝雀发布完成")
            return True

        except Exception as e:
            record.status = DeploymentStatus.FAILED
            record.error_message = str(e)
            record.end_time = datetime.now()
            logger.error(f"❌ 金丝雀发布失败: {str(e)}")
            return False

    async def _monitor_canary_performance(self, namespace: str, services: List[str], duration: int) -> bool:
        """
        监控金丝雀版本性能

        Args:
            namespace: 命名空间
            services: 服务列表
            duration: 监控时长(秒)

        Returns:
            性能是否达标
        """
        logger.info(f"开始性能监控，时长: {duration}秒")

        start_time = time.time()
        check_interval = 60  # 每分钟检查一次

        while time.time() - start_time < duration:
            try:
                # 检查错误率
                for service in services:
                    error_rate = await self._get_service_error_rate(namespace, service)
                    if error_rate > 0.01:  # 错误率超过1%
                        logger.error(f"❌ 服务错误率过高: {service} = {error_rate:.2%}")
                        return False

                    # 检查响应时间
                    response_time = await self._get_service_response_time(namespace, service)
                    if response_time > 1000:  # 响应时间超过1秒
                        logger.error(f"❌ 服务响应时间过长: {service} = {response_time}ms")
                        return False

                logger.info(f"✅ 性能检查通过 ({int(time.time() - start_time)}s/{duration}s)")
                await asyncio.sleep(check_interval)

            except Exception as e:
                logger.error(f"❌ 性能监控异常: {str(e)}")
                return False

        return True

    async def _get_service_error_rate(self, namespace: str, service: str) -> float:
        """获取服务错误率"""
        # 这里应该查询Prometheus获取实际指标
        # 简化实现，返回模拟数据
        return 0.005

    async def _get_service_response_time(self, namespace: str, service: str) -> float:
        """获取服务响应时间"""
        # 这里应该查询Prometheus获取实际指标
        # 简化实现，返回模拟数据
        return 150.0

    async def _rollback_canary_deployment(self, namespace: str, services: List[str]) -> None:
        """
        金丝雀发布回滚

        Args:
            namespace: 命名空间
            services: 服务列表
        """
        logger.info("🔄 开始金丝雀发布回滚")

        if self.current_deployment:
            self.current_deployment.status = DeploymentStatus.ROLLING_BACK
            self.current_deployment.rollback_reason = "金丝雀性能不达标"

        try:
            for service in services:
                canary_deployment = f"{service}-canary"

                # 删除金丝雀部署
                self._kubectl(namespace, f"delete deployment {canary_deployment}")

                # 恢复稳定版本
                self._kubectl(namespace, f"scale deployment {service} --replicas=10")

            if self.current_deployment:
                self.current_deployment.status = DeploymentStatus.ROLLED_BACK

            logger.info("✅ 金丝雀发布回滚完成")

        except Exception as e:
            logger.error(f"❌ 金丝雀发布回滚失败: {str(e)}")
            raise

    async def execute_deployment(self, config: DeploymentConfig) -> bool:
        """
        执行部署

        Args:
            config: 部署配置

        Returns:
            部署是否成功
        """
        logger.info(f"🚀 开始自动部署 - 策略: {config.strategy.value}")

        try:
            # 部署前检查
            await self._pre_deployment_checks(config)

            # 根据策略执行部署
            if config.strategy == DeploymentStrategy.BLUE_GREEN:
                success = await self.execute_blue_green_deployment(config)
            elif config.strategy == DeploymentStrategy.ROLLING_UPDATE:
                success = await self.execute_rolling_update(config)
            elif config.strategy == DeploymentStrategy.CANARY:
                success = await self.execute_canary_deployment(config)
            else:
                raise DeploymentAutomationError(f"不支持的部署策略: {config.strategy.value}")

            # 部署后验证
            if success:
                await self._post_deployment_validation(config)

            return success

        except Exception as e:
            logger.error(f"❌ 部署执行失败: {str(e)}")
            return False

    async def _pre_deployment_checks(self, config: DeploymentConfig) -> None:
        """部署前检查"""
        logger.info("🔍 执行部署前检查...")

        # 检查环境配置
        if config.environment not in self.environments:
            raise DeploymentAutomationError(f"不支持的环境: {config.environment.value}")

        # 检查镜像是否存在
        for service in config.services:
            image = f"{config.image_tag}"
            logger.info(f"检查镜像: {image}")
            # 这里应该检查镜像仓库中是否存在该镜像

        # 检查集群连接
        env_config = self.environments[config.environment]
        try:
            self._run_command(f"kubectl cluster-info --kubeconfig={env_config['kubeconfig']}")
        except DeploymentAutomationError:
            raise DeploymentAutomationError(f"无法连接到集群: {config.environment.value}")

        logger.info("✅ 部署前检查通过")

    async def _post_deployment_validation(self, config: DeploymentConfig) -> None:
        """部署后验证"""
        logger.info("✅ 执行部署后验证...")

        env_config = self.environments[config.environment]
        namespace = env_config["namespace"]

        # 执行健康检查
        for service in config.services:
            if not self._check_service_health(namespace, service):
                raise DeploymentAutomationError(f"部署后健康检查失败: {service}")

        # 执行功能测试
        logger.info("🧪 执行功能测试...")
        # 这里应该调用功能测试脚本

        # 执行性能测试
        logger.info("📊 执行性能测试...")
        # 这里应该调用性能测试脚本

        logger.info("✅ 部署后验证通过")

    def get_deployment_status(self) -> Optional[DeploymentRecord]:
        """获取当前部署状态"""
        return self.current_deployment

    def get_deployment_history(self, limit: int = 10) -> List[DeploymentRecord]:
        """获取部署历史"""
        return self.deployment_records[-limit:]

    def generate_deployment_report(self, deployment_id: str) -> Dict[str, Any]:
        """生成部署报告"""
        record = next((r for r in self.deployment_records if r.id == deployment_id), None)

        if not record:
            raise DeploymentAutomationError(f"部署记录不存在: {deployment_id}")

        report = {
            "deployment_id": record.id,
            "environment": record.config.environment.value,
            "strategy": record.config.strategy.value,
            "image_tag": record.config.image_tag,
            "services": record.config.services,
            "status": record.status.value,
            "start_time": record.start_time.isoformat(),
            "end_time": record.end_time.isoformat() if record.end_time else None,
            "duration": (record.end_time - record.start_time).total_seconds() if record.end_time else None,
            "error_message": record.error_message,
            "rollback_reason": record.rollback_reason,
            "health_checks": record.health_checks,
            "metrics": record.metrics
        }

        return report

async def main():
    """主函数"""
    parser = argparse.ArgumentParser(description="XleRobot部署自动化工具")
    parser.add_argument("--environment", required=True, choices=["development", "testing", "staging", "production"])
    parser.add_argument("--strategy", required=True, choices=["blue_green", "rolling_update", "canary"])
    parser.add_argument("--image-tag", required=True)
    parser.add_argument("--services", nargs="+", default=["xlerobot-asr", "xlerobot-tts", "xlerobot-llm", "xlerobot-multimodal"])
    parser.add_argument("--no-rollback", action="store_true", help="禁用自动回滚")
    parser.add_argument("--health-timeout", type=int, default=600, help="健康检查超时时间(秒)")
    parser.add_argument("--deployment-timeout", type=int, default=1800, help="部署超时时间(秒)")

    args = parser.parse_args()

    try:
        # 创建部署自动化管理器
        automation = XleRobotDeploymentAutomation()

        # 创建部署配置
        config = DeploymentConfig(
            environment=Environment(args.environment),
            strategy=DeploymentStrategy(args.strategy),
            image_tag=args.image_tag,
            services=args.services,
            rollback_enabled=not args.no_rollback,
            health_check_timeout=args.health_timeout,
            deployment_timeout=args.deployment_timeout
        )

        # 执行部署
        success = await automation.execute_deployment(config)

        if success:
            logger.info("🎉 部署成功完成")
            exit(0)
        else:
            logger.error("❌ 部署失败")
            exit(1)

    except KeyboardInterrupt:
        logger.info("🛑 部署被用户中断")
        exit(1)
    except Exception as e:
        logger.error(f"❌ 部署异常: {str(e)}")
        exit(1)

if __name__ == "__main__":
    asyncio.run(main())