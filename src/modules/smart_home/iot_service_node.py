#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智能家居IoT服务节点

ROS2节点，提供智能家居设备控制服务，
集成ASR+LLM+TTS语音控制流程。

作者: Dev Agent
故事ID: Story 5.1
Epic: 5 - 智能家居控制模块
"""

import asyncio
import json
import logging
from typing import Dict, List, Optional, Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D

from .core.protocol_adapter import (
    get_adapter_manager,
    get_protocol_registry,
    ProtocolType,
    WiFiAdapter,
    BluetoothAdapter,
    ZigbeeAdapter,
    MatterAdapter,
)
from .core.device_registry import get_device_registry
from .services.device_discovery import get_discovery_service
from .services.device_control import get_control_service, DeviceControlResult


class IoTServiceNode(Node):
    """智能家居IoT服务节点"""

    def __init__(self):
        """初始化IoT服务节点"""
        super().__init__('iot_service_node')

        # 设置日志
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

        self.get_logger().info("🚀 智能家居IoT服务节点启动中...")

        # 初始化协议注册表和适配器管理器
        self.protocol_registry = get_protocol_registry()
        self.adapter_manager = get_adapter_manager()
        self.device_registry = get_device_registry()
        self.discovery_service = get_discovery_service()
        self.control_service = get_control_service()

        # 自动发现并注册协议适配器
        self._register_adapters()

        # ROS2发布者和订阅者
        self._setup_ros2_interfaces()

        # 创建定时器
        self.timer = self.create_timer(30.0, self._timer_callback)

        # 启动设备发现
        asyncio.create_task(self._start_services())

        self.get_logger().info("✅ 智能家居IoT服务节点启动完成")

    def _register_adapters(self) -> None:
        """注册协议适配器"""
        try:
            # 注册协议适配器
            self.adapter_manager.register_adapter(WiFiAdapter())
            self.adapter_manager.register_adapter(BluetoothAdapter())
            self.adapter_manager.register_adapter(ZigbeeAdapter())
            self.adapter_manager.register_adapter(MatterAdapter())

            self.logger.info("✅ 协议适配器注册完成")

        except Exception as e:
            self.logger.error(f"❌ 协议适配器注册失败: {e}")

    def _setup_ros2_interfaces(self) -> None:
        """设置ROS2接口"""
        # 订阅LLM响应话题（设备控制意图）
        self.llm_subscription = self.create_subscription(
            String,
            '/llm/response',
            self.llm_response_callback,
            10
        )

        # 订阅设备控制话题
        self.control_subscription = self.create_subscription(
            String,
            '/iot/control',
            self.device_control_callback,
            10
        )

        # 订阅设备发现话题
        self.discovery_subscription = self.create_subscription(
            String,
            '/iot/discover',
            self.device_discovery_callback,
            10
        )

        # 发布设备状态话题
        self.status_publisher = self.create_publisher(
            String,
            '/iot/device/status',
            10
        )

        # 发布场景执行结果话题
        self.scene_publisher = self.create_publisher(
            String,
            '/iot/scene/executed',
            10
        )

        # 发布控制响应话题
        self.control_response_publisher = self.create_publisher(
            String,
            '/iot/control/response',
            10
        )

        self.get_logger().info("📡 ROS2接口设置完成")

    async def _start_services(self) -> None:
        """启动服务"""
        try:
            # 启动设备发现服务
            success = await self.discovery_service.start_discovery()

            if success:
                self.get_logger().info("✅ 设备发现服务启动成功")

                # 触发一次设备发现
                devices = await self.discovery_service.discover_once()
                self.get_logger().info(f"🔍 初次发现 {len(devices)} 个设备")
            else:
                self.get_logger().error("❌ 设备发现服务启动失败")

        except Exception as e:
            self.get_logger().error(f"❌ 服务启动异常: {e}")

    def llm_response_callback(self, msg: String) -> None:
        """处理LLM响应回调（设备控制意图）"""
        try:
            self.get_logger().debug(f"📥 收到LLM响应: {msg.data[:100]}...")

            # 解析LLM响应
            data = json.loads(msg.data)

            if data.get('type') == 'device_control_intent':
                # 提取控制意图
                intent = data.get('intent', {})
                device_type = intent.get('device_type')
                action = intent.get('action')
                parameters = intent.get('parameters', {})

                self.get_logger().info(f"🎮 收到设备控制意图: {action} {device_type}")

                # 处理设备控制
                asyncio.create_task(self._handle_device_control_intent(intent))

        except json.JSONDecodeError:
            self.get_logger().warning(f"⚠️ LLM响应格式错误: {msg.data[:100]}")
        except Exception as e:
            self.get_logger().error(f"❌ 处理LLM响应失败: {e}")

    async def _handle_device_control_intent(self, intent: Dict[str, Any]) -> None:
        """处理设备控制意图"""
        try:
            device_type = intent.get('device_type')
            action = intent.get('action')
            parameters = intent.get('parameters', {})

            # 查找匹配的设备
            devices = self.device_registry.list_devices_by_type(device_type)

            if not devices:
                self.get_logger().warning(f"⚠️ 未找到 {device_type} 类型设备")
                return

            # 选择第一个设备（简化处理）
            device = devices[0]
            device_id = device.info.device_id

            # 执行控制命令
            result = await self.control_service.control_device(
                device_id=device_id,
                command_type=action,
                parameters=parameters,
                source='voice'
            )

            # 发布控制结果
            self._publish_control_response(result)

            self.get_logger().info(f"✅ 设备控制完成: {device.info.name}")

        except Exception as e:
            self.get_logger().error(f"❌ 处理设备控制意图失败: {e}")

    def device_control_callback(self, msg: String) -> None:
        """设备控制回调"""
        try:
            data = json.loads(msg.data)
            command = data.get('command')

            if command == 'discover':
                # 触发设备发现
                asyncio.create_task(self._trigger_discovery())
            elif command == 'status':
                # 获取所有设备状态
                asyncio.create_task(self._publish_all_device_status())

        except Exception as e:
            self.get_logger().error(f"❌ 设备控制回调失败: {e}")

    def device_discovery_callback(self, msg: String) -> None:
        """设备发现回调"""
        try:
            data = json.loads(msg.data)

            protocol = data.get('protocol')
            if protocol:
                protocol_type = ProtocolType(protocol.lower())
                asyncio.create_task(self._trigger_protocol_discovery(protocol_type))
            else:
                asyncio.create_task(self._trigger_discovery())

        except Exception as e:
            self.get_logger().error(f"❌ 设备发现回调失败: {e}")

    async def _trigger_discovery(self) -> None:
        """触发设备发现"""
        self.get_logger().info("🔍 触发设备发现...")
        devices = await self.discovery_service.discover_once()
        self.get_logger().info(f"✅ 发现 {len(devices)} 个设备")

    async def _trigger_protocol_discovery(self, protocol: ProtocolType) -> None:
        """触发指定协议设备发现"""
        self.get_logger().info(f"🔍 触发 {protocol.value} 协议设备发现...")
        devices = await self.discovery_service.discover_once(protocol)
        self.get_logger().info(f"✅ 发现 {len(devices)} 个 {protocol.value} 设备")

    async def _publish_all_device_status(self) -> None:
        """发布所有设备状态"""
        try:
            status_data = await self.control_service.get_all_devices_status()

            msg = String()
            msg.data = json.dumps(status_data, ensure_ascii=False)
            self.status_publisher.publish(msg)

            self.get_logger().info("📊 已发布所有设备状态")

        except Exception as e:
            self.get_logger().error(f"❌ 发布设备状态失败: {e}")

    def _publish_control_response(self, result: DeviceControlResult) -> None:
        """发布控制响应"""
        try:
            msg = String()
            msg.data = json.dumps(result.to_dict(), ensure_ascii=False)
            self.control_response_publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"❌ 发布控制响应失败: {e}")

    def _timer_callback(self) -> None:
        """定时器回调"""
        try:
            # 定期发布设备状态统计
            stats = self.control_service.get_statistics()

            self.get_logger().debug(f"📊 设备统计: {stats}")

        except Exception as e:
            self.get_logger().error(f"❌ 定时器回调失败: {e}")

    async def destroy_node(self) -> None:
        """销毁节点"""
        self.get_logger().info("🔄 正在关闭IoT服务节点...")

        try:
            # 停止设备发现服务
            await self.discovery_service.stop_discovery()

            # 清理适配器
            await self.adapter_manager.cleanup_all()

        except Exception as e:
            self.get_logger().error(f"❌ 节点关闭失败: {e}")

        await super().destroy_node()
        self.get_logger().info("✅ IoT服务节点已关闭")


def main(args=None):
    """主函数"""
    rclpy.init(args=args)

    node = IoTServiceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 关闭节点
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
