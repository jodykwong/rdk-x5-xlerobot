#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智能家居模块异常定义

定义智能家居控制模块中的所有异常类型。

作者: Dev Agent
故事ID: Story 5.1
Epic: 5 - 智能家居控制模块
"""


class SmartHomeError(Exception):
    """智能家居模块基础异常"""
    pass


class ProtocolError(SmartHomeError):
    """协议相关异常"""
    pass


class DeviceError(SmartHomeError):
    """设备相关异常"""
    pass


class DiscoveryError(SmartHomeError):
    """设备发现异常"""
    pass


class ControlError(SmartHomeError):
    """设备控制异常"""
    pass


class ConfigurationError(SmartHomeError):
    """配置异常"""
    pass
