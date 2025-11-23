#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智能家居控制模块

支持多种协议的智能家居设备接入、控制和管理：
- WiFi: 基于MQTT的WiFi设备
- Bluetooth: 蓝牙低功耗设备
- Zigbee: Zigbee网状网络设备
- Matter: 新一代统一协议

作者: Dev Agent
故事ID: Story 5.1
Epic: 5 - 智能家居控制模块
版本: v1.0.0
"""

__version__ = '1.0.0'
__author__ = 'Dev Agent'

# 核心组件
from .core import (
    DeviceInterface,
    DeviceInfo,
    DeviceStatus,
    DeviceCapability,
    DeviceType,
    ProtocolType,
    ProtocolAdapter,
    AdapterManager,
    ProtocolRegistry,
    DeviceRegistry,
    DeviceRegistryError,
)

# 协议适配器
from .protocols import (
    WiFiAdapter,
    BluetoothAdapter,
    ZigbeeAdapter,
    MatterAdapter,
)

# 服务
from .services import (
    DeviceDiscoveryService,
    DeviceControlService,
)

# 工具函数
from .core.protocol_adapter import get_adapter_manager, get_protocol_registry
from .core.device_registry import get_device_registry
from .services.device_discovery import get_discovery_service
from .services.device_control import get_control_service

# ROS2节点
from .iot_service_node import IoTServiceNode

# 异常
from .exceptions import (
    SmartHomeError,
    ProtocolError,
    DeviceError,
    DiscoveryError,
    ControlError,
    ConfigurationError,
)

__all__ = [
    # 核心组件
    'DeviceInterface',
    'DeviceInfo',
    'DeviceStatus',
    'DeviceCapability',
    'DeviceType',
    'ProtocolType',
    'ProtocolAdapter',
    'AdapterManager',
    'ProtocolRegistry',
    'DeviceRegistry',
    'DeviceRegistryError',

    # 协议适配器
    'WiFiAdapter',
    'BluetoothAdapter',
    'ZigbeeAdapter',
    'MatterAdapter',

    # 服务
    'DeviceDiscoveryService',
    'DeviceControlService',

    # 工具函数
    'get_adapter_manager',
    'get_protocol_registry',
    'get_device_registry',
    'get_discovery_service',
    'get_control_service',

    # ROS2节点
    'IoTServiceNode',

    # 异常
    'SmartHomeError',
    'ProtocolError',
    'DeviceError',
    'DiscoveryError',
    'ControlError',
    'ConfigurationError',
]
