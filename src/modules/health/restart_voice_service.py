#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
语音服务重启脚本

用于健康监控系统自动重启语音服务。
支持阿里云API配置和完整的服务初始化。

作者: Dev Agent
功能: 语音服务自动重启
"""

import os
import sys
import json
import argparse
import asyncio
import logging
import time
from pathlib import Path

# 添加项目路径
project_root = Path("/home/sunrise/xlerobot")
sys.path.insert(0, str(project_root / "src"))

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


async def restart_voice_service(service_name: str, config: dict):
    """重启语音服务"""
    logger.info(f"🔄 开始重启语音服务: {service_name}")

    try:
        # 设置环境变量
        os.environ["ALIBABA_CLOUD_ACCESS_KEY_ID"] = config.get("alibaba_key", "")
        os.environ["ALIBABA_CLOUD_ACCESS_KEY_SECRET"] = config.get("alibaba_secret", "")
        os.environ["ALIYUN_NLS_APPKEY"] = config.get("appkey", "")
        os.environ["PYTHONPATH"] = f"{project_root / 'src'}:{os.environ.get('PYTHONPATH', '')}"

        # 导入ASR系统
        from modules.asr.asr_system import ASRSystem

        # 创建ASR系统
        asr_system = ASRSystem()

        # 初始化系统
        if not asr_system.initialize():
            logger.error("❌ ASR系统初始化失败")
            return False

        # 启动监听
        if not asr_system.start_listening():
            logger.error("❌ ASR监听启动失败")
            asr_system.cleanup()
            return False

        logger.info(f"✅ 语音服务重启成功: {service_name}")
        logger.info(f"📊 服务PID: {os.getpid()}")
        logger.info("🎤 语音服务正在运行...")

        # 保持服务运行
        status = asr_system.get_status()
        last_status_report = time.time()

        try:
            while True:
                await asyncio.sleep(1)

                # 每30秒报告一次状态
                current_time = time.time()
                if current_time - last_status_report >= 30:
                    status = asr_system.get_status()
                    logger.info(f"📊 服务状态 - 监听: {status['stats']['total_listens']}, "
                               f"唤醒: {status['stats']['wake_detections']}, "
                               f"运行时间: {status['uptime_seconds']}秒")
                    last_status_report = current_time

        except KeyboardInterrupt:
            logger.info("🛑 收到停止信号")
            return True

        finally:
            # 清理资源
            asr_system.stop_listening()
            asr_system.cleanup()
            logger.info("✅ 语音服务已停止")

    except Exception as e:
        logger.error(f"❌ 语音服务重启失败: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description="重启语音服务")
    parser.add_argument("--service-name", required=True, help="服务名称")
    parser.add_argument("--config", required=True, help="配置JSON字符串")

    args = parser.parse_args()

    try:
        # 解析配置
        config = json.loads(args.config)
        logger.info(f"📝 重启服务: {args.service_name}")
        logger.info(f"⚙️ 配置: {config}")

        # 启动服务
        success = asyncio.run(restart_voice_service(args.service_name, config))
        exit_code = 0 if success else 1

    except Exception as e:
        logger.error(f"❌ 启动失败: {e}")
        sys.exit(1)

    sys.exit(exit_code)


if __name__ == "__main__":
    main()