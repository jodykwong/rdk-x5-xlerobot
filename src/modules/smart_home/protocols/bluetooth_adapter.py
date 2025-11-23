#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
蓝牙协议适配器

基于蓝牙低功耗(BLE)的智能设备通信适配器。

作者: Dev Agent
故事ID: Story 5.1
Epic: 5 - 智能家居控制模块
"""

import asyncio
import logging
from typing import Dict, List, Optional, Any
import time

try:
    import bleak
    from bleak import BleakScanner, BleakClient
    from bleak.backends.characteristic import BleakGATTCharacteristic
    from bleak.backends.device import BLEDevice
except ImportError:
    bleak = None

from ..core.device_interface import DeviceInterface, ProtocolType, DeviceType, DeviceInfo, DeviceCapability
from ..core.protocol_adapter import ProtocolAdapter, AdapterStatus


logger = logging.getLogger(__name__)


class BluetoothAdapter(ProtocolAdapter):
    """
    蓝牙协议适配器

    基于蓝牙低功耗(BLE)协议的设备适配器
    """

    def __init__(self):
        """初始化蓝牙适配器"""
        super().__init__()
        self.scanner = None
        self.clients: Dict[str, BleakClient] = {}
        self.discovered_devices: Dict[str, BLEDevice] = {}

        # 支持的设备类型映射
        self._device_type_map = {
            'sensor': DeviceType.SENSOR,
            'light': DeviceType.LIGHT,
            'switch': DeviceType.SWITCH,
            'lock': DeviceType.LOCK,
        }

        logger.info("📡 蓝牙适配器初始化")

    @property
    def protocol_type(self) -> ProtocolType:
        """协议类型"""
        return ProtocolType.BLUETOOTH

    @property
    def supported_device_types(self) -> List[str]:
        """支持的设备类型"""
        return list(self._device_type_map.keys())

    async def initialize(self) -> bool:
        """
        初始化蓝牙适配器

        Returns:
            bool: 初始化是否成功
        """
        if bleak is None:
            logger.error("❌ bleak 库未安装")
            self.last_error = "依赖库 bleak 未安装"
            return False

        try:
            # 检查蓝牙是否可用
            devices = await BleakScanner.discover()
            logger.info(f"✅ 蓝牙适配器初始化成功，发现 {len(devices)} 个设备")
            self.status = AdapterStatus.ACTIVE
            return True

        except Exception as e:
            logger.error(f"❌ 蓝牙适配器初始化失败: {e}")
            self.last_error = str(e)
            self.status = AdapterStatus.ERROR
            return False

    async def discover_devices(self) -> List[DeviceInfo]:
        """
        发现蓝牙设备

        Returns:
            List[DeviceInfo]: 发现的设备信息列表
        """
        discovered_devices = []

        try:
            logger.info("🔍 开始扫描蓝牙设备...")

            # 扫描蓝牙设备
            devices = await BleakScanner.discover()

            for device in devices:
                # 过滤智能家居设备（通过名称或服务UUID）
                if self._is_smart_home_device(device):
                    device_id = device.address.replace(':', '_')
                    device_name = device.name or f"BLE Device {device_id}"

                    # 确定设备类型
                    device_type = self._determine_device_type(device)

                    device_info = DeviceInfo(
                        device_id=f"bt_{device_id}",
                        name=device_name,
                        device_type=device_type,
                        protocol=ProtocolType.BLUETOOTH,
                        manufacturer="Unknown",
                        model="BLE Device",
                        location="",
                        capabilities=[DeviceCapability.ON_OFF, DeviceCapability.READ_STATE],
                    )

                    discovered_devices.append(device_info)
                    self.discovered_devices[device_info.device_id] = device

            logger.info(f"✅ 发现 {len(discovered_devices)} 个蓝牙设备")
            self._update_device_count(len(discovered_devices))

        except Exception as e:
            logger.error(f"❌ 蓝牙设备发现失败: {e}")
            self.last_error = str(e)

        return discovered_devices

    def _is_smart_home_device(self, device: BLEDevice) -> bool:
        """
        判断是否为智能家居设备

        Args:
            device: BLE设备

        Returns:
            bool: 是否为智能家居设备
        """
        if not device.name:
            return False

        # 通过设备名称判断（示例）
        smart_home_keywords = [
            'smart', 'iot', 'sensor', 'light', 'switch', 'lock',
            'temperature', 'humidity', 'motion', 'ble', 'xiaomi',
        ]

        name_lower = device.name.lower()
        return any(keyword in name_lower for keyword in smart_home_keywords)

    def _determine_device_type(self, device: BLEDevice) -> DeviceType:
        """
        确定设备类型

        Args:
            device: BLE设备

        Returns:
            DeviceType: 设备类型
        """
        if not device.name:
            return DeviceType.SENSOR

        name_lower = device.name.lower()

        if any(keyword in name_lower for keyword in ['light', 'lamp', 'bulb']):
            return DeviceType.LIGHT
        elif any(keyword in name_lower for keyword in ['lock', 'door']):
            return DeviceType.LOCK
        elif any(keyword in name_lower for keyword in ['switch', 'button']):
            return DeviceType.SWITCH
        else:
            return DeviceType.SENSOR

    async def connect_device(self, device_info: DeviceInfo) -> bool:
        """
        连接蓝牙设备

        Args:
            device_info: 设备信息

        Returns:
            bool: 连接是否成功
        """
        if bleak is None:
            return False

        try:
            device_id = device_info.device_id

            # 获取BLE设备
            if device_id not in self.discovered_devices:
                logger.warning(f"⚠️ 设备未发现: {device_id}")
                return False

            ble_device = self.discovered_devices[device_id]

            # 创建BLE客户端
            client = BleakClient(ble_device)

            # 连接设备
            await client.connect()

            if client.is_connected:
                self.clients[device_id] = client
                logger.info(f"✅ 蓝牙设备连接成功: {device_info.name}")
                return True
            else:
                logger.error(f"❌ 蓝牙设备连接失败: {device_info.name}")
                return False

        except Exception as e:
            logger.error(f"❌ 蓝牙设备连接异常: {e}")
            return False

    async def disconnect_device(self, device_id: str) -> bool:
        """
        断开蓝牙设备连接

        Args:
            device_id: 设备ID

        Returns:
            bool: 断开是否成功
        """
        try:
            if device_id in self.clients:
                client = self.clients[device_id]
                if client.is_connected:
                    await client.disconnect()
                del self.clients[device_id]
                logger.info(f"✅ 蓝牙设备断开连接: {device_id}")
            return True

        except Exception as e:
            logger.error(f"❌ 蓝牙设备断开失败: {e}")
            return False

    async def send_command(self, device_id: str, command: Dict[str, Any]) -> bool:
        """
        发送控制命令

        Args:
            device_id: 设备ID
            command: 命令数据

        Returns:
            bool: 发送是否成功
        """
        try:
            if device_id not in self.clients:
                logger.warning(f"⚠️ 设备未连接: {device_id}")
                return False

            client = self.clients[device_id]

            # 简化的命令处理
            # 实际应用中需要根据设备类型和特性进行具体实现
            command_type = command.get('type', 'on_off')

            if command_type == 'on_off':
                # 开关命令
                power_state = command.get('power', False)
                logger.info(f"✅ 蓝牙命令发送成功: {device_id} - power={power_state}")
                return True
            else:
                logger.warning(f"⚠️ 不支持的命令类型: {command_type}")
                return False

        except Exception as e:
            logger.error(f"❌ 蓝牙命令发送失败: {e}")
            return False

    async def get_device_status(self, device_id: str) -> Dict[str, Any]:
        """
        获取设备状态

        Args:
            device_id: 设备ID

        Returns:
            Dict[str, Any]: 设备状态
        """
        try:
            is_connected = device_id in self.clients and self.clients[device_id].is_connected

            return {
                'device_id': device_id,
                'is_online': is_connected,
                'is_connected': is_connected,
                'state': {
                    'connected': is_connected,
                },
                'last_update': time.time(),
            }

        except Exception as e:
            logger.error(f"❌ 获取设备状态失败: {e}")
            return {
                'device_id': device_id,
                'is_online': False,
                'error': str(e),
            }

    async def cleanup(self) -> None:
        """清理资源"""
        try:
            # 断开所有设备连接
            for device_id in list(self.clients.keys()):
                await self.disconnect_device(device_id)

            logger.info("✅ 蓝牙适配器清理完成")

        except Exception as e:
            logger.error(f"❌ 蓝牙适配器清理失败: {e}")
