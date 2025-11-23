#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
设备注册表

提供设备注册、查找、管理和状态维护功能。
支持按设备ID、类型、位置、协议等多种方式查找设备。

作者: Dev Agent
故事ID: Story 5.1
Epic: 5 - 智能家居控制模块
"""

import time
import logging
from typing import Dict, List, Optional, Set, Any
from collections import defaultdict
from threading import RLock

from .device_interface import DeviceInterface, DeviceInfo, DeviceStatus


logger = logging.getLogger(__name__)


class DeviceRegistryError(Exception):
    """设备注册表异常"""
    pass


class DeviceNotFoundError(DeviceRegistryError):
    """设备未找到异常"""
    pass


class DeviceAlreadyExistsError(DeviceRegistryError):
    """设备已存在异常"""
    pass


class DeviceRegistry:
    """
    设备注册表

    提供设备统一管理功能
    """

    def __init__(self):
        """初始化设备注册表"""
        self._devices: Dict[str, DeviceInterface] = {}
        self._devices_by_type: Dict[str, Set[str]] = defaultdict(set)
        self._devices_by_protocol: Dict[str, Set[str]] = defaultdict(set)
        self._devices_by_location: Dict[str, Set[str]] = defaultdict(set)
        self._lock = RLock()

        logger.info("✅ 设备注册表初始化完成")

    def register_device(self, device: DeviceInterface) -> None:
        """
        注册设备

        Args:
            device: 设备实例

        Raises:
            DeviceAlreadyExistsError: 设备已存在
        """
        with self._lock:
            device_id = device.info.device_id

            if device_id in self._devices:
                raise DeviceAlreadyExistsError(f"设备 {device_id} 已存在")

            self._devices[device_id] = device

            # 按类型索引
            device_type = device.info.device_type.value
            self._devices_by_type[device_type].add(device_id)

            # 按协议索引
            protocol = device.info.protocol.value
            self._devices_by_protocol[protocol].add(device_id)

            # 按位置索引
            if device.info.location:
                self._devices_by_location[device.info.location].add(device_id)

            logger.info(f"✅ 设备注册成功: {device.info.name} ({device_id})")

    def unregister_device(self, device_id: str) -> None:
        """
        注销设备

        Args:
            device_id: 设备ID

        Raises:
            DeviceNotFoundError: 设备未找到
        """
        with self._lock:
            if device_id not in self._devices:
                raise DeviceNotFoundError(f"设备 {device_id} 未找到")

            device = self._devices[device_id]

            # 从主索引删除
            del self._devices[device_id]

            # 从类型索引删除
            device_type = device.info.device_type.value
            self._devices_by_type[device_type].discard(device_id)
            if not self._devices_by_type[device_type]:
                del self._devices_by_type[device_type]

            # 从协议索引删除
            protocol = device.info.protocol.value
            self._devices_by_protocol[protocol].discard(device_id)
            if not self._devices_by_protocol[protocol]:
                del self._devices_by_protocol[protocol]

            # 从位置索引删除
            if device.info.location:
                self._devices_by_location[device.info.location].discard(device_id)
                if not self._devices_by_location[device.info.location]:
                    del self._devices_by_location[device.info.location]

            logger.info(f"✅ 设备注销成功: {device.info.name} ({device_id})")

    def get_device(self, device_id: str) -> DeviceInterface:
        """
        获取设备

        Args:
            device_id: 设备ID

        Returns:
            DeviceInterface: 设备实例

        Raises:
            DeviceNotFoundError: 设备未找到
        """
        with self._lock:
            if device_id not in self._devices:
                raise DeviceNotFoundError(f"设备 {device_id} 未找到")

            return self._devices[device_id]

    def device_exists(self, device_id: str) -> bool:
        """
        检查设备是否存在

        Args:
            device_id: 设备ID

        Returns:
            bool: 是否存在
        """
        with self._lock:
            return device_id in self._devices

    def list_devices(self) -> List[DeviceInterface]:
        """
        列出所有设备

        Returns:
            List[DeviceInterface]: 设备列表
        """
        with self._lock:
            return list(self._devices.values())

    def list_devices_by_type(self, device_type: str) -> List[DeviceInterface]:
        """
        按类型列出设备

        Args:
            device_type: 设备类型

        Returns:
            List[DeviceInterface]: 设备列表
        """
        with self._lock:
            device_ids = self._devices_by_type.get(device_type, set())
            return [self._devices[device_id] for device_id in device_ids]

    def list_devices_by_protocol(self, protocol: str) -> List[DeviceInterface]:
        """
        按协议列出设备

        Args:
            protocol: 协议类型

        Returns:
            List[DeviceInterface]: 设备列表
        """
        with self._lock:
            device_ids = self._devices_by_protocol.get(protocol, set())
            return [self._devices[device_id] for device_id in device_ids]

    def list_devices_by_location(self, location: str) -> List[DeviceInterface]:
        """
        按位置列出设备

        Args:
            location: 位置

        Returns:
            List[DeviceInterface]: 设备列表
        """
        with self._lock:
            device_ids = self._devices_by_location.get(location, set())
            return [self._devices[device_id] for device_id in device_ids]

    def search_devices(self, **criteria) -> List[DeviceInterface]:
        """
        搜索设备

        Args:
            **criteria: 搜索条件 (name, device_type, protocol, location, manufacturer)

        Returns:
            List[DeviceInterface]: 匹配的设备列表
        """
        with self._lock:
            results = []

            for device in self._devices.values():
                match = True

                for key, value in criteria.items():
                    if hasattr(device.info, key):
                        if str(getattr(device.info, key)).lower() != str(value).lower():
                            match = False
                            break
                    else:
                        match = False
                        break

                if match:
                    results.append(device)

            return results

    def get_online_devices(self) -> List[DeviceInterface]:
        """
        获取在线设备

        Returns:
            List[DeviceInterface]: 在线设备列表
        """
        with self._lock:
            online_devices = []
            for device in self._devices.values():
                if device.state.is_online:
                    online_devices.append(device)
            return online_devices

    def get_offline_devices(self) -> List[DeviceInterface]:
        """
        获取离线设备

        Returns:
            List[DeviceInterface]: 离线设备列表
        """
        with self._lock:
            offline_devices = []
            for device in self._devices.values():
                if not device.state.is_online:
                    offline_devices.append(device)
            return offline_devices

    def get_statistics(self) -> Dict[str, Any]:
        """
        获取设备统计信息

        Returns:
            Dict[str, Any]: 统计信息
        """
        with self._lock:
            total_count = len(self._devices)
            online_count = sum(1 for d in self._devices.values() if d.state.is_online)
            offline_count = total_count - online_count

            type_stats = {t: len(devices) for t, devices in self._devices_by_type.items()}
            protocol_stats = {p: len(devices) for p, devices in self._devices_by_protocol.items()}
            location_stats = {l: len(devices) for l, devices in self._devices_by_location.items()}

            return {
                'total_devices': total_count,
                'online_devices': online_count,
                'offline_devices': offline_count,
                'online_rate': online_count / total_count if total_count > 0 else 0,
                'type_distribution': type_stats,
                'protocol_distribution': protocol_stats,
                'location_distribution': location_stats,
            }

    def clear(self) -> None:
        """清空所有设备"""
        with self._lock:
            count = len(self._devices)
            self._devices.clear()
            self._devices_by_type.clear()
            self._devices_by_protocol.clear()
            self._devices_by_location.clear()

            logger.info(f"✅ 清空设备注册表，共移除 {count} 个设备")


# 全局设备注册表实例
_global_registry = None


def get_device_registry() -> DeviceRegistry:
    """
    获取全局设备注册表实例

    Returns:
        DeviceRegistry: 设备注册表实例
    """
    global _global_registry
    if _global_registry is None:
        _global_registry = DeviceRegistry()
    return _global_registry
