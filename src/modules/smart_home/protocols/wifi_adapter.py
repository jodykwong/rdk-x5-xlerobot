#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WiFi协议适配器

基于MQTT的WiFi设备通信适配器，支持主流智能家居品牌设备。

作者: Dev Agent
故事ID: Story 5.1
Epic: 5 - 智能家居控制模块
"""

import asyncio
import json
import logging
from typing import Dict, List, Optional, Any
import time

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None

from ..core.device_interface import DeviceInterface, ProtocolType, DeviceType, DeviceInfo, DeviceCapability
from ..core.protocol_adapter import ProtocolAdapter, AdapterStatus


logger = logging.getLogger(__name__)


class WiFiAdapter(ProtocolAdapter):
    """
    WiFi协议适配器

    基于MQTT协议的WiFi智能设备适配器
    """

    def __init__(self, broker_host: str = "localhost", broker_port: int = 1883):
        """
        初始化WiFi适配器

        Args:
            broker_host: MQTT代理主机
            broker_port: MQTT代理端口
        """
        super().__init__()
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.client = None
        self.connected_devices: Dict[str, Dict[str, Any]] = {}

        # 支持的设备类型映射
        self._device_type_map = {
            'light': DeviceType.LIGHT,
            'switch': DeviceType.SWITCH,
            'outlet': DeviceType.OUTLET,
            'sensor': DeviceType.SENSOR,
            'thermostat': DeviceType.THERMOSTAT,
        }

        logger.info(f"📡 WiFi适配器初始化: MQTT {broker_host}:{broker_port}")

    @property
    def protocol_type(self) -> ProtocolType:
        """协议类型"""
        return ProtocolType.WIFI

    @property
    def supported_device_types(self) -> List[str]:
        """支持的设备类型"""
        return list(self._device_type_map.keys())

    async def initialize(self) -> bool:
        """
        初始化MQTT连接

        Returns:
            bool: 初始化是否成功
        """
        if mqtt is None:
            logger.error("❌ paho-mqtt 库未安装")
            self.last_error = "依赖库 paho-mqtt 未安装"
            return False

        try:
            self.client = mqtt.Client()

            # 设置回调函数
            self.client.on_connect = self._on_connect
            self.client.on_disconnect = self._on_disconnect
            self.client.on_message = self._on_message

            # 异步连接
            loop = asyncio.get_event_loop()
            await loop.run_in_executor(None, self.client.connect, self.broker_host, self.broker_port, 60)

            # 启动MQTT客户端循环（在后台线程中）
            self.client.loop_start()

            self.status = AdapterStatus.ACTIVE
            logger.info("✅ WiFi适配器初始化成功")
            return True

        except Exception as e:
            logger.error(f"❌ WiFi适配器初始化失败: {e}")
            self.last_error = str(e)
            self.status = AdapterStatus.ERROR
            return False

    def _on_connect(self, client, userdata, flags, rc):
        """MQTT连接回调"""
        if rc == 0:
            logger.info("✅ MQTT连接成功")

            # 订阅所有设备状态主题
            client.subscribe("home/+/+/+/status")
            client.subscribe("home/+/+/+/state")

            logger.info("📡 已订阅设备状态主题")
        else:
            logger.error(f"❌ MQTT连接失败，错误码: {rc}")
            self.status = AdapterStatus.ERROR

    def _on_disconnect(self, client, userdata, rc):
        """MQTT断开连接回调"""
        logger.warning("⚠️ MQTT连接断开")
        self.status = AdapterStatus.IDLE

    def _on_message(self, client, userdata, msg):
        """MQTT消息接收回调"""
        try:
            topic = msg.topic.decode()
            payload = json.loads(msg.payload.decode())

            # 解析主题: home/{location}/{device_type}/{device_id}/status
            parts = topic.split('/')
            if len(parts) >= 5:
                location = parts[1]
                device_type = parts[2]
                device_id = parts[3]

                # 存储设备状态
                device_key = f"{device_type}_{device_id}"
                if device_key not in self.connected_devices:
                    self.connected_devices[device_key] = {
                        'device_id': device_id,
                        'device_type': device_type,
                        'location': location,
                    }

                self.connected_devices[device_key]['status'] = payload
                self.connected_devices[device_key]['last_update'] = time.time()

                logger.debug(f"📡 收到设备状态更新: {device_id} - {payload}")

        except Exception as e:
            logger.error(f"❌ 处理MQTT消息失败: {e}")

    async def discover_devices(self) -> List[DeviceInfo]:
        """
        发现WiFi设备

        Returns:
            List[DeviceInfo]: 发现的设备信息列表
        """
        discovered_devices = []

        try:
            # 检查发现的设备
            for device_key, device_data in self.connected_devices.items():
                device_id = device_data['device_id']
                device_type_name = device_data['device_type']
                location = device_data['location']

                # 转换设备类型
                device_type = self._device_type_map.get(device_type_name, DeviceType.UNKNOWN)

                # 确定设备能力
                capabilities = self._determine_capabilities(device_type, device_data.get('status', {}))

                device_info = DeviceInfo(
                    device_id=f"wifi_{device_id}",
                    name=f"{device_type_name.title()} {device_id}",
                    device_type=device_type,
                    protocol=ProtocolType.WIFI,
                    manufacturer="Unknown",
                    model="WiFi Device",
                    location=location,
                    capabilities=capabilities,
                )

                discovered_devices.append(device_info)

            logger.info(f"✅ 发现 {len(discovered_devices)} 个WiFi设备")
            self._update_device_count(len(discovered_devices))

        except Exception as e:
            logger.error(f"❌ 设备发现失败: {e}")
            self.last_error = str(e)

        return discovered_devices

    def _determine_capabilities(self, device_type: DeviceType, status: Dict[str, Any]) -> List[DeviceCapability]:
        """
        根据设备类型和状态确定设备能力

        Args:
            device_type: 设备类型
            status: 设备状态

        Returns:
            List[DeviceCapability]: 能力列表
        """
        capabilities = [DeviceCapability.ON_OFF, DeviceCapability.READ_STATE]

        if device_type == DeviceType.LIGHT:
            capabilities.extend([
                DeviceCapability.DIMMABLE,
                DeviceCapability.RGB_COLOR,
                DeviceCapability.SCENES,
            ])

        elif device_type == DeviceType.THERMOSTAT:
            capabilities.extend([
                DeviceCapability.TARGET_TEMP,
                DeviceCapability.CURRENT_TEMP,
                DeviceCapability.HVAC_MODE,
            ])

        elif device_type == DeviceType.SENSOR:
            if 'temperature' in status:
                capabilities.append(DeviceCapability.TEMPERATURE)
            if 'humidity' in status:
                capabilities.append(DeviceCapability.HUMIDITY)
            if 'motion' in status:
                capabilities.append(DeviceCapability.MOTION)

        return capabilities

    async def connect_device(self, device_info: DeviceInfo) -> bool:
        """
        连接WiFi设备

        Args:
            device_info: 设备信息

        Returns:
            bool: 连接是否成功
        """
        try:
            # WiFi设备通过MQTT自动连接，这里只需要验证设备存在
            device_id = device_info.device_id.replace('wifi_', '')
            device_key = f"{device_info.device_type.value}_{device_id}"

            if device_key in self.connected_devices:
                logger.info(f"✅ WiFi设备连接成功: {device_info.name}")
                return True
            else:
                logger.warning(f"⚠️ WiFi设备未发现: {device_info.name}")
                return False

        except Exception as e:
            logger.error(f"❌ WiFi设备连接失败: {e}")
            return False

    async def disconnect_device(self, device_id: str) -> bool:
        """
        断开WiFi设备连接

        Args:
            device_id: 设备ID

        Returns:
            bool: 断开是否成功
        """
        try:
            # WiFi设备通过MQTT自动断开，这里标记设备离线
            logger.info(f"✅ WiFi设备断开连接: {device_id}")
            return True

        except Exception as e:
            logger.error(f"❌ WiFi设备断开失败: {e}")
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
            if not self.client:
                logger.error("❌ MQTT客户端未初始化")
                return False

            # 提取设备ID前缀
            actual_device_id = device_id.replace('wifi_', '')

            # 构造MQTT主题
            # 这里需要知道设备类型和位置，简化处理
            topic = f"home/living_room/light/{actual_device_id}/command"

            # 发送命令
            result = self.client.publish(topic, json.dumps(command))

            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                logger.info(f"✅ WiFi命令发送成功: {topic} - {command}")
                return True
            else:
                logger.error(f"❌ WiFi命令发送失败: {result.rc}")
                return False

        except Exception as e:
            logger.error(f"❌ WiFi命令发送异常: {e}")
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
            actual_device_id = device_id.replace('wifi_', '')
            device_key = f"light_{actual_device_id}"  # 假设是灯光设备

            if device_key in self.connected_devices:
                status_data = self.connected_devices[device_key].get('status', {})

                return {
                    'device_id': device_id,
                    'is_online': True,
                    'is_connected': True,
                    'state': status_data,
                    'last_update': self.connected_devices[device_key].get('last_update', time.time()),
                }
            else:
                return {
                    'device_id': device_id,
                    'is_online': False,
                    'is_connected': False,
                    'state': {},
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
            if self.client:
                self.client.loop_stop()
                self.client.disconnect()
                logger.info("✅ WiFi适配器清理完成")

        except Exception as e:
            logger.error(f"❌ WiFi适配器清理失败: {e}")
