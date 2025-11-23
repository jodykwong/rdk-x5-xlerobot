#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
设备发现服务

提供智能家居设备的自动发现、配对和注册功能。

作者: Dev Agent
故事ID: Story 5.1
Epic: 5 - 智能家居控制模块
"""

import asyncio
import logging
from typing import Dict, List, Optional, Any
import time

from ..core.device_interface import DeviceInfo, ProtocolType
from ..core.protocol_adapter import get_adapter_manager
from ..core.device_registry import get_device_registry


logger = logging.getLogger(__name__)


class DeviceDiscoveryService:
    """
    设备发现服务

    负责协调各协议适配器的设备发现工作，
    并将发现的设备自动注册到设备注册表中
    """

    def __init__(self):
        """初始化设备发现服务"""
        self.adapter_manager = get_adapter_manager()
        self.device_registry = get_device_registry()
        self.discovery_task = None
        self.is_running = False

        logger.info("✅ 设备发现服务初始化完成")

    async def start_discovery(self, protocols: List[ProtocolType] = None) -> bool:
        """
        开始设备发现

        Args:
            protocols: 协议列表，如果为None则发现所有协议

        Returns:
            bool: 是否成功启动
        """
        if self.is_running:
            logger.warning("⚠️ 设备发现已在运行中")
            return True

        try:
            # 初始化所有适配器
            init_results = await self.adapter_manager.initialize_all()

            # 检查是否有成功的适配器
            successful_protocols = [p for p, success in init_results.items() if success]

            if not successful_protocols:
                logger.error("❌ 没有可用的协议适配器")
                return False

            logger.info(f"✅ 启动设备发现: {len(successful_protocols)} 个协议")

            # 启动后台发现任务
            self.is_running = True
            self.discovery_task = asyncio.create_task(self._discovery_loop())

            return True

        except Exception as e:
            logger.error(f"❌ 启动设备发现失败: {e}")
            self.is_running = False
            return False

    async def stop_discovery(self) -> None:
        """停止设备发现"""
        if not self.is_running:
            logger.info("ℹ️ 设备发现未运行")
            return

        logger.info("⏹️ 停止设备发现...")

        self.is_running = False

        if self.discovery_task:
            self.discovery_task.cancel()
            try:
                await self.discovery_task
            except asyncio.CancelledError:
                pass

        logger.info("✅ 设备发现已停止")

    async def discover_once(self, protocol: ProtocolType = None) -> List[DeviceInfo]:
        """
        单次设备发现

        Args:
            protocol: 指定协议，为None则发现所有协议

        Returns:
            List[DeviceInfo]: 发现的设备列表
        """
        discovered_devices = []

        try:
            if protocol:
                # 发现指定协议设备
                adapter = self.adapter_manager.get_adapter(protocol)
                if adapter and adapter.status.value == "active":
                    logger.info(f"🔍 发现 {protocol.value} 设备...")
                    devices = await adapter.discover_devices()
                    discovered_devices.extend(devices)
                    logger.info(f"✅ {protocol.value}: 发现 {len(devices)} 个设备")
                else:
                    logger.warning(f"⚠️ 协议 {protocol.value} 适配器未就绪")
            else:
                # 发现所有协议设备
                for protocol_type in self.adapter_manager.list_adapters():
                    adapter = self.adapter_manager.get_adapter(protocol_type)
                    if adapter and adapter.status.value == "active":
                        logger.info(f"🔍 发现 {protocol_type.value} 设备...")
                        devices = await adapter.discover_devices()
                        discovered_devices.extend(devices)
                        logger.info(f"✅ {protocol_type.value}: 发现 {len(devices)} 个设备")

            # 自动注册发现的设备
            await self._register_discovered_devices(discovered_devices)

        except Exception as e:
            logger.error(f"❌ 设备发现失败: {e}")

        return discovered_devices

    async def _discovery_loop(self) -> None:
        """设备发现循环"""
        logger.info("🔄 启动设备发现循环")

        while self.is_running:
            try:
                # 定期发现设备
                await self.discover_once()

                # 等待一段时间后再次发现
                await asyncio.sleep(30)  # 30秒发现一次

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"❌ 设备发现循环异常: {e}")
                await asyncio.sleep(5)  # 出错后等待5秒再试

        logger.info("🔄 设备发现循环结束")

    async def _register_discovered_devices(self, devices: List[DeviceInfo]) -> None:
        """
        注册发现的设备

        Args:
            devices: 设备信息列表
        """
        registered_count = 0
        skipped_count = 0

        for device_info in devices:
            try:
                # 检查设备是否已存在
                if self.device_registry.device_exists(device_info.device_id):
                    skipped_count += 1
                    continue

                # 获取协议适配器
                adapter = self.adapter_manager.get_adapter(device_info.protocol)
                if not adapter:
                    logger.warning(f"⚠️ 未找到协议适配器: {device_info.protocol.value}")
                    continue

                # 创建设备实例（简化处理）
                from ..core.device_interface import DeviceInterface

                class MockDevice(DeviceInterface):
                    def __init__(self, info):
                        super().__init__(info)

                    async def connect(self):
                        return True

                    async def disconnect(self):
                        return True

                    async def send_command(self, command):
                        return True

                    async def get_status(self):
                        from ..core.device_interface import DeviceStatus
                        return DeviceStatus(self.info, self.state)

                    async def start_monitoring(self):
                        pass

                    async def stop_monitoring(self):
                        pass

                device = MockDevice(device_info)

                # 注册设备
                self.device_registry.register_device(device)

                # 关联设备和协议适配器
                self.adapter_manager.set_device_adapter(device_info.device_id, device_info.protocol)

                registered_count += 1
                logger.info(f"✅ 设备注册成功: {device_info.name}")

            except Exception as e:
                logger.error(f"❌ 注册设备失败 {device_info.name}: {e}")

        if registered_count > 0 or skipped_count > 0:
            logger.info(f"📊 设备发现统计: 新注册 {registered_count} 个，跳过 {skipped_count} 个")

    async def get_discovery_statistics(self) -> Dict[str, Any]:
        """
        获取发现统计信息

        Returns:
            Dict[str, Any]: 统计信息
        """
        stats = {
            'is_running': self.is_running,
            'adapter_statistics': self.adapter_manager.get_statistics(),
            'registry_statistics': self.device_registry.get_statistics(),
        }

        return stats


# 全局设备发现服务实例
_discovery_service = None


def get_discovery_service() -> DeviceDiscoveryService:
    """
    获取全局设备发现服务实例

    Returns:
        DeviceDiscoveryService: 设备发现服务实例
    """
    global _discovery_service
    if _discovery_service is None:
        _discovery_service = DeviceDiscoveryService()
    return _discovery_service
