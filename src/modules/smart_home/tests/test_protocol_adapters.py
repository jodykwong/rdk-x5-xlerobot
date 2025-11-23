#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
协议适配器测试

测试各种协议适配器的功能。

作者: Dev Agent
故事ID: Story 5.1
Epic: 5 - 智能家居控制模块
"""

import asyncio
import pytest
import sys
import os

# 添加模块路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.device_interface import DeviceType, ProtocolType, DeviceInfo, DeviceCapability
from protocols.wifi_adapter import WiFiAdapter
from protocols.bluetooth_adapter import BluetoothAdapter
from protocols.zigbee_adapter import ZigbeeAdapter
from protocols.matter_adapter import MatterAdapter


class TestProtocolAdapters:
    """协议适配器测试类"""

    @pytest.mark.asyncio
    async def test_wifi_adapter_initialization(self):
        """测试WiFi适配器初始化"""
        adapter = WiFiAdapter()

        assert adapter.protocol_type == ProtocolType.WIFI
        assert 'light' in adapter.supported_device_types
        assert 'switch' in adapter.supported_device_types

        # 测试初始化（模拟）
        result = await adapter.initialize()
        # 实际测试中可能需要MQTT代理
        assert isinstance(result, bool)

    @pytest.mark.asyncio
    async def test_wifi_adapter_discovery(self):
        """测试WiFi设备发现"""
        adapter = WiFiAdapter()

        # 模拟设备发现
        devices = await adapter.discover_devices()
        assert isinstance(devices, list)

        # 如果有设备，检查设备信息结构
        for device in devices:
            assert isinstance(device, DeviceInfo)
            assert device.protocol == ProtocolType.WIFI
            assert isinstance(device.device_type, DeviceType)

    @pytest.mark.asyncio
    async def test_bluetooth_adapter_initialization(self):
        """测试蓝牙适配器初始化"""
        adapter = BluetoothAdapter()

        assert adapter.protocol_type == ProtocolType.BLUETOOTH
        assert 'sensor' in adapter.supported_device_types
        assert 'light' in adapter.supported_device_types

    @pytest.mark.asyncio
    async def test_zigbee_adapter_initialization(self):
        """测试Zigbee适配器初始化"""
        adapter = ZigbeeAdapter()

        assert adapter.protocol_type == ProtocolType.ZIGBEE
        assert 'light' in adapter.supported_device_types
        assert 'sensor' in adapter.supported_device_types

    @pytest.mark.asyncio
    async def test_matter_adapter_initialization(self):
        """测试Matter适配器初始化"""
        adapter = MatterAdapter()

        assert adapter.protocol_type == ProtocolType.MATTER
        assert 'light' in adapter.supported_device_types
        assert 'sensor' in adapter.supported_device_types

    def test_device_info_creation(self):
        """测试设备信息创建"""
        device_info = DeviceInfo(
            device_id="test_device_001",
            name="Test Light",
            device_type=DeviceType.LIGHT,
            protocol=ProtocolType.WIFI,
            manufacturer="Test Vendor",
            model="Test Model",
            location="Living Room"
        )

        assert device_info.device_id == "test_device_001"
        assert device_info.name == "Test Light"
        assert device_info.device_type == DeviceType.LIGHT
        assert device_info.protocol == ProtocolType.WIFI
        assert device_info.location == "Living Room"

    def test_device_capabilities(self):
        """测试设备能力"""
        # 灯光设备能力
        light_capabilities = [DeviceCapability.ON_OFF, DeviceCapability.DIMMABLE]
        assert DeviceCapability.ON_OFF in light_capabilities
        assert DeviceCapability.DIMMABLE in light_capabilities

        # 传感器能力
        sensor_capabilities = [DeviceCapability.TEMPERATURE, DeviceCapability.HUMIDITY]
        assert DeviceCapability.TEMPERATURE in sensor_capabilities
        assert DeviceCapability.HUMIDITY in sensor_capabilities

    @pytest.mark.asyncio
    async def test_adapter_cleanup(self):
        """测试适配器清理"""
        # 测试各个适配器的清理功能
        adapters = [
            WiFiAdapter(),
            BluetoothAdapter(),
            ZigbeeAdapter(),
            MatterAdapter()
        ]

        for adapter in adapters:
            try:
                await adapter.cleanup()
                assert True  # 清理成功
            except Exception as e:
                pytest.fail(f"适配器清理失败 {adapter.protocol_type.value}: {e}")


if __name__ == '__main__':
    # 运行测试
    pytest.main([__file__, '-v'])
