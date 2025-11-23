#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智能家居协议适配器模块

包含各种协议的适配器实现：
- WiFi: 基于MQTT的WiFi设备
- Bluetooth: 蓝牙低功耗设备
- Zigbee: Zigbee网状网络设备
- Matter: 新一代统一协议

作者: Dev Agent
故事ID: Story 5.1
Epic: 5 - 智能家居控制模块
"""

from .wifi_adapter import WiFiAdapter
from .bluetooth_adapter import BluetoothAdapter
from .zigbee_adapter import ZigbeeAdapter
from .matter_adapter import MatterAdapter

__all__ = [
    'WiFiAdapter',
    'BluetoothAdapter',
    'ZigbeeAdapter',
    'MatterAdapter',
]
