#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
设备接口抽象定义

统一设备接口抽象层，定义设备基本属性、
状态、控制接口和协议类型。

作者: Dev Agent
故事ID: Story 5.1
Epic: 5 - 智能家居控制模块
"""

from enum import Enum
from typing import Dict, List, Any, Optional, Union
from dataclasses import dataclass, field
import time


class DeviceType(str, Enum):
    """设备类型枚举"""
    LIGHT = "light"  # 灯光
    SWITCH = "switch"  # 开关
    SENSOR = "sensor"  # 传感器
    THERMOSTAT = "thermostat"  # 温控器
    CURTAIN = "curtain"  # 窗帘
    OUTLET = "outlet"  # 插座
    CAMERA = "camera"  # 摄像头
    LOCK = "lock"  # 门锁
    SPEAKER = "speaker"  # 音响
    TV = "tv"  # 电视
    AIR_CONDITIONER = "air_conditioner"  # 空调
    FAN = "fan"  # 风扇
    HUMIDIFIER = "humidifier"  # 加湿器
    AIR_PURIFIER = "air_purifier"  # 空气净化器
    UNKNOWN = "unknown"  # 未知设备


class ProtocolType(str, Enum):
    """协议类型枚举"""
    WIFI = "wifi"
    BLUETOOTH = "bluetooth"
    ZIGBEE = "zigbee"
    MATTER = "matter"
    UNKNOWN = "unknown"


class DeviceCapability(str, Enum):
    """设备能力枚举"""
    # 基础能力
    ON_OFF = "on_off"  # 开关
    READ_STATE = "read_state"  # 读取状态

    # 灯光能力
    DIMMABLE = "dimmable"  # 可调亮度
    COLOR_TEMP = "color_temp"  # 色温
    RGB_COLOR = "rgb_color"  # 彩光
    SCENES = "scenes"  # 场景

    # 传感器能力
    TEMPERATURE = "temperature"  # 温度
    HUMIDITY = "humidity"  # 湿度
    MOTION = "motion"  # 运动
    LIGHT_LEVEL = "light_level"  # 光照
    BATTERY_LEVEL = "battery_level"  # 电池电量

    # 温控能力
    TARGET_TEMP = "target_temp"  # 目标温度
    HVAC_MODE = "hvac_mode"  # 模式
    FAN_SPEED = "fan_speed"  # 风速

    # 窗帘能力
    OPEN_CLOSE = "open_close"  # 开合
    POSITION = "position"  # 位置

    # 锁具能力
    LOCK_UNLOCK = "lock_unlock"  # 上锁解锁

    # 安防能力
    ARM_DISARM = "arm_disarm"  # 布撤防


@dataclass
class DeviceInfo:
    """设备信息数据模型"""
    device_id: str  # 设备唯一标识
    name: str  # 设备名称
    device_type: DeviceType  # 设备类型
    protocol: ProtocolType  # 通信协议
    manufacturer: str  # 厂商
    model: str  # 型号
    location: str = ""  # 位置：客厅、卧室等
    capabilities: List[DeviceCapability] = field(default_factory=list)  # 能力列表
    description: str = ""  # 描述
    firmware_version: str = ""  # 固件版本
    created_at: float = field(default_factory=time.time)  # 创建时间
    updated_at: float = field(default_factory=time.time)  # 更新时间


@dataclass
class DeviceState:
    """设备状态数据模型"""
    # 基础状态
    is_online: bool = False  # 在线状态
    is_connected: bool = False  # 连接状态
    last_seen: Optional[float] = None  # 最后活跃时间

    # 通用属性
    power_state: Optional[bool] = None  # 电源状态
    brightness: Optional[int] = None  # 亮度 (0-100)
    color_temp: Optional[int] = None  # 色温 (K)
    rgb_color: Optional[Dict[str, int]] = None  # RGB颜色
    target_temperature: Optional[float] = None  # 目标温度
    current_temperature: Optional[float] = None  # 当前温度
    humidity: Optional[int] = None  # 湿度 (%)
    position: Optional[int] = None  # 位置 (0-100)
    lock_state: Optional[bool] = None  # 锁定状态
    battery_level: Optional[int] = None  # 电池电量 (0-100)
    signal_strength: Optional[int] = None  # 信号强度 (0-100)
    fan_speed: Optional[int] = None  # 风速 (0-100)
    hvac_mode: Optional[str] = None  # 空调模式

    # 自定义属性
    custom_attributes: Dict[str, Any] = field(default_factory=dict)

    # 时间戳
    last_update: float = field(default_factory=time.time)

    def update_timestamp(self) -> None:
        """更新时间戳"""
        self.last_update = time.time()


@dataclass
class DeviceCommand:
    """设备控制命令"""
    command_type: str  # 命令类型
    parameters: Dict[str, Any] = field(default_factory=dict)  # 命令参数
    timestamp: float = field(default_factory=time.time)  # 时间戳
    source: str = "manual"  # 命令来源：voice, scene, manual, api
    priority: int = 0  # 优先级 (0-9)
    timeout: float = 5.0  # 超时时间 (秒)
    callback: Optional[Any] = None  # 回调函数


@dataclass
class DeviceStatus:
    """设备完整状态"""
    info: DeviceInfo  # 设备信息
    state: DeviceState  # 设备状态

    def to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        return {
            'device_id': self.info.device_id,
            'name': self.info.name,
            'device_type': self.info.device_type.value,
            'protocol': self.info.protocol.value,
            'manufacturer': self.info.manufacturer,
            'model': self.info.model,
            'location': self.info.location,
            'capabilities': [cap.value for cap in self.info.capabilities],
            'state': {
                'is_online': self.state.is_online,
                'is_connected': self.state.is_connected,
                'power_state': self.state.power_state,
                'brightness': self.state.brightness,
                'color_temp': self.state.color_temp,
                'rgb_color': self.state.rgb_color,
                'target_temperature': self.state.target_temperature,
                'current_temperature': self.state.current_temperature,
                'humidity': self.state.humidity,
                'position': self.state.position,
                'lock_state': self.state.lock_state,
                'battery_level': self.state.battery_level,
                'signal_strength': self.state.signal_strength,
                'last_update': self.state.last_update,
                'custom_attributes': self.state.custom_attributes,
            }
        }


class DeviceInterface:
    """
    设备接口抽象类

    所有设备驱动必须实现的接口
    """

    def __init__(self, device_info: DeviceInfo):
        """
        初始化设备接口

        Args:
            device_info: 设备信息
        """
        self.info = device_info
        self.state = DeviceState()
        self._callbacks: Dict[str, List] = {
            'state_changed': [],
            'connection_changed': [],
            'error': [],
        }

    async def connect(self) -> bool:
        """
        连接设备

        Returns:
            bool: 连接是否成功
        """
        raise NotImplementedError("子类必须实现此方法")

    async def disconnect(self) -> bool:
        """
        断开设备连接

        Returns:
            bool: 断开是否成功
        """
        raise NotImplementedError("子类必须实现此方法")

    async def send_command(self, command: DeviceCommand) -> bool:
        """
        发送控制命令

        Args:
            command: 控制命令

        Returns:
            bool: 命令是否发送成功
        """
        raise NotImplementedError("子类必须实现此方法")

    async def get_status(self) -> DeviceStatus:
        """
        获取设备状态

        Returns:
            DeviceStatus: 设备状态
        """
        raise NotImplementedError("子类必须实现此方法")

    async def start_monitoring(self) -> None:
        """开始状态监控"""
        raise NotImplementedError("子类必须实现此方法")

    async def stop_monitoring(self) -> None:
        """停止状态监控"""
        raise NotImplementedError("子类必须实现此方法")

    def is_connected(self) -> bool:
        """
        检查设备是否已连接

        Returns:
            bool: 连接状态
        """
        return self.state.is_connected and self.state.is_online

    def has_capability(self, capability: DeviceCapability) -> bool:
        """
        检查设备是否具有指定能力

        Args:
            capability: 设备能力

        Returns:
            bool: 是否具有该能力
        """
        return capability in self.info.capabilities

    def add_callback(self, event: str, callback) -> None:
        """
        添加事件回调

        Args:
            event: 事件类型
            callback: 回调函数
        """
        if event in self._callbacks:
            self._callbacks[event].append(callback)

    def remove_callback(self, event: str, callback) -> None:
        """
        移除事件回调

        Args:
            event: 事件类型
            callback: 回调函数
        """
        if event in self._callbacks and callback in self._callbacks[event]:
            self._callbacks[event].remove(callback)

    def _emit_event(self, event: str, *args, **kwargs) -> None:
        """
        触发事件回调

        Args:
            event: 事件类型
        """
        if event in self._callbacks:
            for callback in self._callbacks[event]:
                try:
                    if asyncio.iscoroutinefunction(callback):
                        await callback(*args, **kwargs)
                    else:
                        callback(*args, **kwargs)
                except Exception as e:
                    print(f"回调函数执行错误: {e}")

    def _update_state(self, **kwargs) -> None:
        """
        更新设备状态

        Args:
            **kwargs: 状态属性
        """
        for key, value in kwargs.items():
            if hasattr(self.state, key):
                setattr(self.state, key, value)
        self.state.update_timestamp()

        # 触发状态变化事件
        import asyncio
        asyncio.create_task(self._emit_event('state_changed', self.state))


# 导入asyncio以支持异步回调
import asyncio
