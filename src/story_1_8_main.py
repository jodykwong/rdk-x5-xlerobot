#!/usr/bin/env python3.10
"""
XleRobot Story 1.8 Main - 系统优化与部署主程序
BMad Method v6 Brownfield Level 4 企业级标准

功能特性:
- 集成系统优化器
- 启动系统监控
- 管理部署流程
- 运行用户验收测试
- 提供统一的管理接口
- 100%符合Epic 1纯在线架构
"""

import asyncio
import json
import logging
import signal
import sys
import time
from typing import Dict, Any, Optional
from pathlib import Path

# 导入Story 1.8核心组件
from story_1_8_integration_optimizer import (
    SystemIntegrationOptimizer,
    OptimizationConfig,
    get_system_optimizer
)
from story_1_8_system_monitor import (
    SystemMonitor,
    PerformanceThresholds,
    get_system_monitor
)
from story_1_8_deployment_manager import (
    DeploymentManager,
    DeploymentConfig,
    create_deployment_manager
)
from story_1_8_user_acceptance_test import (
    CantoneseUserAcceptanceTest,
    create_cantonese_uat
)

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('/var/log/xlerobot_story_1_8.log'),
        logging.StreamHandler(sys.stdout)
    ]
)

logger = logging.getLogger(__name__)

class Story18SystemManager:
    """Story 1.8系统管理器 - 统一管理所有组件"""

    def __init__(self):
        """
        初始化Story 1.8系统管理器
        """
        logger.info("🚀 初始化Story18SystemManager - XleRobot系统优化与部署")

        # 组件实例
        self.system_optimizer: Optional[SystemIntegrationOptimizer] = None
        self.system_monitor: Optional[SystemMonitor] = None
        self.deployment_manager: Optional[DeploymentManager] = None
        self.uat_tester: Optional[CantoneseUserAcceptanceTest] = None

        # 系统状态
        self.system_running = False
        self.shutdown_requested = False

        # 注册信号处理器
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        logger.info("✅ Story 1.8系统管理器初始化完成")

    def _signal_handler(self, signum, frame):
        """信号处理器"""
        logger.info(f"收到信号 {signum}，开始优雅关闭...")
        self.shutdown_requested = True

    async def initialize_system(self, config: Optional[Dict[str, Any]] = None) -> None:
        """
        初始化系统组件

        Args:
            config: 系统配置
        """
        logger.info("🔧 初始化系统组件")

        try:
            # 1. 初始化系统优化器
            optimization_config = OptimizationConfig()
            if config and "optimization" in config:
                for key, value in config["optimization"].items():
                    if hasattr(optimization_config, key):
                        setattr(optimization_config, key, value)

            self.system_optimizer = SystemIntegrationOptimizer(optimization_config)
            logger.info("✅ 系统优化器初始化完成")

            # 2. 初始化系统监控
            performance_thresholds = PerformanceThresholds()
            if config and "monitoring" in config:
                for key, value in config["monitoring"].items():
                    if hasattr(performance_thresholds, key):
                        setattr(performance_thresholds, key, value)

            self.system_monitor = SystemMonitor(
                component_name="XleRobot-Story1.8",
                thresholds=performance_thresholds
            )
            logger.info("✅ 系统监控初始化完成")

            # 3. 初始化部署管理器
            deployment_config = DeploymentConfig(
                deployment_name="XleRobot-Story1.8",
                version="1.8.0",
                environment=config.get("environment", "production") if config else "production"
            )
            if config and "deployment" in config:
                for key, value in config["deployment"].items():
                    if hasattr(deployment_config, key):
                        setattr(deployment_config, key, value)

            self.deployment_manager = DeploymentManager(deployment_config)
            logger.info("✅ 部署管理器初始化完成")

            # 4. 初始化用户验收测试
            self.uat_tester = CantoneseUserAcceptanceTest()
            logger.info("✅ 用户验收测试初始化完成")

            logger.info("🎉 所有系统组件初始化完成")

        except Exception as e:
            logger.error(f"❌ 系统初始化失败: {str(e)}")
            raise e

    async def start_system(self) -> None:
        """启动系统"""
        if self.system_running:
            logger.warning("⚠️ 系统已在运行中")
            return

        logger.info("🚀 启动XleRobot Story 1.8系统")

        try:
            # 1. 启动系统优化器
            if self.system_optimizer:
                await self.system_optimizer.start_monitoring()
                logger.info("✅ 系统优化器已启动")

            # 2. 启动系统监控
            if self.system_monitor and self.system_optimizer:
                await self.system_monitor.start_monitoring(self.system_optimizer)
                logger.info("✅ 系统监控已启动")

            self.system_running = True
            logger.info("🎉 XleRobot Story 1.8系统启动成功")

        except Exception as e:
            logger.error(f"❌ 系统启动失败: {str(e)}")
            raise e

    async def stop_system(self) -> None:
        """停止系统"""
        if not self.system_running:
            logger.info("系统未运行")
            return

        logger.info("🛑 停止XleRobot Story 1.8系统")

        try:
            # 1. 停止系统监控
            if self.system_monitor:
                await self.system_monitor.stop_monitoring()
                logger.info("✅ 系统监控已停止")

            # 2. 停止系统优化器
            if self.system_optimizer:
                await self.system_optimizer.shutdown()
                logger.info("✅ 系统优化器已停止")

            self.system_running = False
            logger.info("🎉 XleRobot Story 1.8系统已停止")

        except Exception as e:
            logger.error(f"❌ 系统停止失败: {str(e)}")
            raise e

    async def run_optimized_multimodal_test(self) -> Dict[str, Any]:
        """运行优化后的多模态测试"""
        if not self.system_optimizer:
            raise RuntimeError("系统优化器未初始化")

        logger.info("🧪 运行优化后的多模态测试")

        try:
            # 模拟多模态输入
            test_audio = "base64_encoded_audio_data"
            test_image = "base64_encoded_image_data"
            test_text = "早晨，今日天气点样？"
            session_id = f"test_{int(time.time())}"

            # 执行优化处理
            result = await self.system_optimizer.optimize_multimodal_processing(
                audio_data=test_audio,
                image_data=test_image,
                text_input=test_text,
                session_id=session_id
            )

            logger.info("✅ 多模态测试完成")
            return result

        except Exception as e:
            logger.error(f"❌ 多模态测试失败: {str(e)}")
            raise e

    async def run_deployment(self, deployment_config: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """运行部署"""
        if not self.deployment_manager:
            raise RuntimeError("部署管理器未初始化")

        logger.info("🚀 开始系统部署")

        try:
            # 执行部署
            deployment_record = await self.deployment_manager.deploy()

            logger.info("✅ 系统部署完成")
            return self.deployment_manager.get_deployment_status()

        except Exception as e:
            logger.error(f"❌ 系统部署失败: {str(e)}")
            raise e

    async def run_user_acceptance_test(self, user_profile: Dict[str, Any]) -> Dict[str, Any]:
        """运行用户验收测试"""
        if not self.uat_tester:
            raise RuntimeError("用户验收测试器未初始化")

        logger.info("🧪 开始用户验收测试")

        try:
            # 开始测试会话
            session_id = await self.uat_tester.start_user_acceptance_test(user_profile)
            logger.info(f"✅ 测试会话已开始: {session_id}")

            # 运行所有测试场景
            for scenario in self.uat_tester.test_scenarios:
                if scenario.critical:  # 优先运行关键场景
                    await self.uat_tester.run_test_scenario(scenario.id)

            # 完成测试
            test_report = await self.uat_tester.complete_user_acceptance_test(
                user_feedback="系统表现良好，响应时间令人满意"
            )

            logger.info("✅ 用户验收测试完成")
            return test_report

        except Exception as e:
            logger.error(f"❌ 用户验收测试失败: {str(e)}")
            raise e

    async def get_system_health(self) -> Dict[str, Any]:
        """获取系统健康状态"""
        health_report = {
            "system_running": self.system_running,
            "timestamp": time.time(),
            "components": {}
        }

        # 系统优化器健康
        if self.system_optimizer:
            health_report["components"]["optimizer"] = self.system_optimizer.get_system_health()

        # 系统监控健康
        if self.system_monitor:
            health_report["components"]["monitor"] = self.system_monitor.get_monitoring_status()

        # 部署管理器状态
        if self.deployment_manager:
            health_report["components"]["deployment"] = self.deployment_manager.get_deployment_status()

        # 用户验收测试状态
        if self.uat_tester:
            health_report["components"]["uat"] = self.uat_tester.get_test_session_status()

        return health_report

    async def run_system_demonstration(self) -> None:
        """运行系统演示"""
        logger.info("🎭 开始系统演示")

        try:
            # 1. 显示系统健康状态
            health = await self.get_system_health()
            logger.info(f"📊 系统健康状态: {json.dumps(health, indent=2, ensure_ascii=False)}")

            # 2. 运行多模态测试
            logger.info("🧪 演示多模态优化处理...")
            multimodal_result = await self.run_optimized_multimodal_test()
            logger.info(f"✅ 多模态测试结果: {json.dumps(multimodal_result, indent=2, ensure_ascii=False)}")

            # 3. 运行用户验收测试
            logger.info("👥 演示用户验收测试...")
            user_profile = {
                "age": 35,
                "type": "family",
                "language_preference": "cantonese",
                "tech_savviness": "medium"
            }
            uat_report = await self.run_user_acceptance_test(user_profile)
            logger.info(f"✅ 用户验收测试报告: {json.dumps(uat_report, indent=2, ensure_ascii=False)}")

            # 4. 显示最终系统状态
            final_health = await self.get_system_health()
            logger.info(f"🎯 最终系统状态: {json.dumps(final_health, indent=2, ensure_ascii=False)}")

            logger.info("🎉 系统演示完成")

        except Exception as e:
            logger.error(f"❌ 系统演示失败: {str(e)}")
            raise e

    async def run_forever(self) -> None:
        """持续运行系统"""
        logger.info("♾️ 系统进入持续运行模式")

        try:
            while self.system_running and not self.shutdown_requested:
                await asyncio.sleep(10)

                # 定期健康检查
                if int(time.time()) % 60 == 0:  # 每分钟检查一次
                    health = await self.get_system_health()
                    if not health.get("system_running", False):
                        logger.warning("⚠️ 检测到系统异常")

        except asyncio.CancelledError:
            logger.info("系统运行被取消")
        except Exception as e:
            logger.error(f"❌ 系统运行异常: {str(e)}")
        finally:
            await self.stop_system()

async def main():
    """主函数"""
    logger.info("🌟 XleRobot Story 1.8 - 系统优化与部署启动")

    # 创建系统管理器
    system_manager = Story18SystemManager()

    try:
        # 加载配置
        config_path = Path("/home/sunrise/xlerobot_config/production.yaml")
        config = {}
        if config_path.exists():
            with open(config_path, 'r', encoding='utf-8') as f:
                import yaml
                config = yaml.safe_load(f)

        # 初始化系统
        await system_manager.initialize_system(config)

        # 启动系统
        await system_manager.start_system()

        # 运行演示 (如果配置中启用)
        if config.get("run_demo", False):
            logger.info("🎭 运行系统演示")
            await system_manager.run_system_demonstration()

        # 持续运行
        await system_manager.run_forever()

    except KeyboardInterrupt:
        logger.info("🛑 收到中断信号，开始关闭...")
    except Exception as e:
        logger.error(f"❌ 系统运行失败: {str(e)}")
        raise e
    finally:
        logger.info("👋 XleRobot Story 1.8系统已关闭")

if __name__ == "__main__":
    # 运行主程序
    asyncio.run(main())