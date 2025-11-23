#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智能家居控制模块 - 核心抽象层

提供统一的设备接口抽象、协议适配器模式、
设备发现和管理抽象层。

作者: Dev Agent
故事ID: Story 5.1
Epic: 5 - 智能家居控制模块
"""

from .device_interface import (
    DeviceInterface,
    DeviceInfo,
    DeviceStatus,
    DeviceCapability,
    DeviceType,
    ProtocolType,
)
from .protocol_adapter import (
    ProtocolAdapter,
    AdapterManager,
    ProtocolRegistry,
)
from .device_registry import (
    DeviceRegistry,
    DeviceRegistryError,
)

__all__ = [
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
]
