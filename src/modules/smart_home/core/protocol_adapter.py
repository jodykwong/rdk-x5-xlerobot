#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
协议适配器抽象层

实现协议适配器模式，提供统一的协议适配接口、
协议自动选择和切换、插件机制等。

作者: Dev Agent
故事ID: Story 5.1
Epic: 5 - 智能家居控制模块
"""

import logging
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Type, Any
from enum import Enum
import importlib
import inspect

from .device_interface import DeviceInterface, ProtocolType, DeviceInfo


logger = logging.getLogger(__name__)


class AdapterStatus(str, Enum):
    """适配器状态"""
    IDLE = "idle"  # 空闲
    ACTIVE = "active"  # 活跃
    ERROR = "error"  # 错误
    DISABLED = "disabled"  # 禁用


class ProtocolAdapter(ABC):
    """
    协议适配器抽象基类

    所有协议适配器必须继承此类并实现抽象方法
    """

    def __init__(self):
        """初始化协议适配器"""
        self.status: AdapterStatus = AdapterStatus.IDLE
        self.last_error: Optional[str] = None
        self._device_count = 0

    @property
    @abstractmethod
    def protocol_type(self) -> ProtocolType:
        """协议类型"""
        pass

    @property
    @abstractmethod
    def supported_device_types(self) -> List[str]:
        """支持的设备类型列表"""
        pass

    @abstractmethod
    async def initialize(self) -> bool:
        """
        初始化协议适配器

        Returns:
            bool: 初始化是否成功
        """
        pass

    @abstractmethod
    async def discover_devices(self) -> List[DeviceInfo]:
        """
        发现设备

        Returns:
            List[DeviceInfo]: 发现的设备信息列表
        """
        pass

    @abstractmethod
    async def connect_device(self, device_info: DeviceInfo) -> bool:
        """
        连接设备

        Args:
            device_info: 设备信息

        Returns:
            bool: 连接是否成功
        """
        pass

    @abstractmethod
    async def disconnect_device(self, device_id: str) -> bool:
        """
        断开设备连接

        Args:
            device_id: 设备ID

        Returns:
            bool: 断开是否成功
        """
        pass

    @abstractmethod
    async def send_command(self, device_id: str, command: Dict[str, Any]) -> bool:
        """
        发送控制命令

        Args:
            device_id: 设备ID
            command: 命令数据

        Returns:
            bool: 发送是否成功
        """
        pass

    @abstractmethod
    async def get_device_status(self, device_id: str) -> Dict[str, Any]:
        """
        获取设备状态

        Args:
            device_id: 设备ID

        Returns:
            Dict[str, Any]: 设备状态
        """
        pass

    @abstractmethod
    async def cleanup(self) -> None:
        """
        清理资源

        关闭所有连接，释放资源
        """
        pass

    def get_statistics(self) -> Dict[str, Any]:
        """
        获取协议适配器统计信息

        Returns:
            Dict[str, Any]: 统计信息
        """
        return {
            'protocol': self.protocol_type.value,
            'status': self.status.value,
            'connected_devices': self._device_count,
            'last_error': self.last_error,
        }

    def _update_device_count(self, count: int) -> None:
        """更新连接设备数量"""
        self._device_count = count


class AdapterManager:
    """
    适配器管理器

    负责管理所有协议适配器、协议选择和切换
    """

    def __init__(self):
        """初始化适配器管理器"""
        self._adapters: Dict[ProtocolType, ProtocolAdapter] = {}
        self._device_adapter_map: Dict[str, ProtocolType] = {}
        self._registry = None
        logger.info("✅ 适配器管理器初始化完成")

    def register_adapter(self, adapter: ProtocolAdapter) -> None:
        """
        注册协议适配器

        Args:
            adapter: 协议适配器实例
        """
        protocol = adapter.protocol_type

        if protocol in self._adapters:
            logger.warning(f"⚠️ 协议 {protocol.value} 适配器已存在，将覆盖")

        self._adapters[protocol] = adapter
        logger.info(f"✅ 协议适配器注册成功: {protocol.value}")

    def unregister_adapter(self, protocol: ProtocolType) -> None:
        """
        注销协议适配器

        Args:
            protocol: 协议类型
        """
        if protocol in self._adapters:
            # 清理适配器
            adapter = self._adapters[protocol]
            try:
                import asyncio
                asyncio.create_task(adapter.cleanup())
            except Exception as e:
                logger.error(f"清理适配器失败: {e}")

            # 从映射中删除
            devices_to_remove = [
                device_id for device_id, p in self._device_adapter_map.items() if p == protocol
            ]
            for device_id in devices_to_remove:
                del self._device_adapter_map[device_id]

            del self._adapters[protocol]
            logger.info(f"✅ 协议适配器注销成功: {protocol.value}")

    def get_adapter(self, protocol: ProtocolType) -> Optional[ProtocolAdapter]:
        """
        获取协议适配器

        Args:
            protocol: 协议类型

        Returns:
            Optional[ProtocolAdapter]: 适配器实例，不存在则返回None
        """
        return self._adapters.get(protocol)

    def list_adapters(self) -> List[ProtocolType]:
        """
        列出所有已注册的协议适配器

        Returns:
            List[ProtocolType]: 协议类型列表
        """
        return list(self._adapters.keys())

    def set_device_adapter(self, device_id: str, protocol: ProtocolType) -> None:
        """
        设置设备使用的协议适配器

        Args:
            device_id: 设备ID
            protocol: 协议类型
        """
        if protocol not in self._adapters:
            logger.error(f"❌ 协议 {protocol.value} 适配器未注册")
            return

        self._device_adapter_map[device_id] = protocol
        logger.debug(f"✅ 设备 {device_id} 分配到协议 {protocol.value}")

    def get_device_protocol(self, device_id: str) -> Optional[ProtocolType]:
        """
        获取设备使用的协议

        Args:
            device_id: 设备ID

        Returns:
            Optional[ProtocolType]: 协议类型
        """
        return self._device_adapter_map.get(device_id)

    async def initialize_all(self) -> Dict[str, bool]:
        """
        初始化所有适配器

        Returns:
            Dict[str, bool]: 初始化结果字典 {protocol: success}
        """
        results = {}

        for protocol, adapter in self._adapters.items():
            try:
                logger.info(f"📡 初始化协议适配器: {protocol.value}")
                success = await adapter.initialize()
                results[protocol.value] = success

                if success:
                    adapter.status = AdapterStatus.ACTIVE
                    logger.info(f"✅ 协议 {protocol.value} 适配器初始化成功")
                else:
                    adapter.status = AdapterStatus.ERROR
                    logger.error(f"❌ 协议 {protocol.value} 适配器初始化失败")

            except Exception as e:
                logger.error(f"❌ 协议 {protocol.value} 适配器初始化异常: {e}")
                adapter.last_error = str(e)
                adapter.status = AdapterStatus.ERROR
                results[protocol.value] = False

        return results

    async def cleanup_all(self) -> None:
        """清理所有适配器"""
        for adapter in self._adapters.values():
            try:
                await adapter.cleanup()
            except Exception as e:
                logger.error(f"清理适配器失败: {e}")

    def get_statistics(self) -> Dict[str, Any]:
        """
        获取适配器管理器统计信息

        Returns:
            Dict[str, Any]: 统计信息
        """
        adapter_stats = {}
        for protocol, adapter in self._adapters.items():
            adapter_stats[protocol.value] = adapter.get_statistics()

        return {
            'registered_adapters': len(self._adapters),
            'adapter_distribution': adapter_stats,
            'mapped_devices': len(self._device_adapter_map),
        }


class ProtocolRegistry:
    """
    协议注册表

    管理协议适配器的自动发现和注册
    """

    def __init__(self):
        """初始化协议注册表"""
        self._adapter_classes: Dict[ProtocolType, Type[ProtocolAdapter]] = {}
        self._package_paths = [
            'smart_home.protocols',
        ]
        logger.info("✅ 协议注册表初始化完成")

    def register_adapter_class(self, protocol: ProtocolType, adapter_class: Type[ProtocolAdapter]) -> None:
        """
        注册适配器类

        Args:
            protocol: 协议类型
            adapter_class: 适配器类
        """
        if not issubclass(adapter_class, ProtocolAdapter):
            raise ValueError(f"适配器类必须继承自 ProtocolAdapter")

        self._adapter_classes[protocol] = adapter_class
        logger.info(f"✅ 适配器类注册成功: {protocol.value}")

    def auto_discover_adapters(self) -> None:
        """自动发现协议适配器"""
        logger.info("🔍 开始自动发现协议适配器...")

        for package_path in self._package_paths:
            try:
                package = importlib.import_module(package_path)
                package_dir = package.__path__[0]

                import os
                import glob

                # 查找所有Python文件
                adapter_files = glob.glob(os.path.join(package_dir, '*_adapter.py'))

                for file_path in adapter_files:
                    module_name = os.path.basename(file_path)[:-3]  # 去掉.py后缀

                    try:
                        module = importlib.import_module(f"{package_path}.{module_name}")

                        # 查找适配器类
                        for name, obj in inspect.getmembers(module, inspect.isclass):
                            if (
                                issubclass(obj, ProtocolAdapter) and
                                obj != ProtocolAdapter and
                                hasattr(obj, 'protocol_type')
                            ):
                                protocol = obj.protocol_type
                                self.register_adapter_class(protocol, obj)
                                logger.info(f"✅ 发现适配器: {name} ({protocol.value})")

                    except Exception as e:
                        logger.warning(f"⚠️ 加载模块失败 {module_name}: {e}")

            except ImportError as e:
                logger.warning(f"⚠️ 无法导入包 {package_path}: {e}")

        logger.info(f"🔍 自动发现完成，共发现 {len(self._adapter_classes)} 个适配器")

    def create_adapter(self, protocol: ProtocolType) -> Optional[ProtocolAdapter]:
        """
        创建适配器实例

        Args:
            protocol: 协议类型

        Returns:
            Optional[ProtocolAdapter]: 适配器实例
        """
        adapter_class = self._adapter_classes.get(protocol)
        if adapter_class:
            return adapter_class()
        return None

    def list_supported_protocols(self) -> List[ProtocolType]:
        """
        列出所有支持的协议

        Returns:
            List[ProtocolType]: 协议类型列表
        """
        return list(self._adapter_classes.keys())


# 全局适配器管理器实例
_adapter_manager = None
_protocol_registry = None


def get_adapter_manager() -> AdapterManager:
    """
    获取全局适配器管理器实例

    Returns:
        AdapterManager: 适配器管理器实例
    """
    global _adapter_manager
    if _adapter_manager is None:
        _adapter_manager = AdapterManager()
    return _adapter_manager


def get_protocol_registry() -> ProtocolRegistry:
    """
    获取全局协议注册表实例

    Returns:
        ProtocolRegistry: 协议注册表实例
    """
    global _protocol_registry
    if _protocol_registry is None:
        _protocol_registry = ProtocolRegistry()
    return _protocol_registry
