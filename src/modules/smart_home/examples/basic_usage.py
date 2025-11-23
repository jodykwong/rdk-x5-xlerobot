#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智能家居模块基本使用示例

展示如何初始化模块、发现设备、控制设备等基本功能。

作者: Dev Agent
故事ID: Story 5.1
Epic: 5 - 智能家居控制模块
"""

import asyncio
import json
import logging

# 导入智能家居模块
from smart_home import (
    get_protocol_registry,
    get_adapter_manager,
    get_device_registry,
    get_discovery_service,
    get_control_service,
    ProtocolType,
    DeviceType,
    DeviceInfo,
)


async def main():
    """主函数"""
    # 设置日志
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    logger = logging.getLogger(__name__)

    logger.info("🚀 智能家居模块基本使用示例")
    logger.info("=" * 50)

    try:
        # 1. 初始化协议注册表
        logger.info("\n1️⃣ 初始化协议注册表...")
        protocol_registry = get_protocol_registry()
        protocol_registry.auto_discover_adapters()

        supported_protocols = protocol_registry.list_supported_protocols()
        logger.info(f"✅ 支持的协议: {[p.value for p in supported_protocols]}")

        # 2. 初始化适配器管理器
        logger.info("\n2️⃣ 初始化适配器管理器...")
        adapter_manager = get_adapter_manager()

        # 手动注册适配器（示例）
        from smart_home.protocols import (
            WiFiAdapter,
            BluetoothAdapter,
            ZigbeeAdapter,
            MatterAdapter,
        )

        adapter_manager.register_adapter(WiFiAdapter())
        adapter_manager.register_adapter(BluetoothAdapter())
        adapter_manager.register_adapter(ZigbeeAdapter())
        adapter_manager.register_adapter(MatterAdapter())

        logger.info(f"✅ 已注册适配器: {[p.value for p in adapter_manager.list_adapters()]}")

        # 3. 初始化设备注册表
        logger.info("\n3️⃣ 初始化设备注册表...")
        device_registry = get_device_registry()
        logger.info("✅ 设备注册表初始化完成")

        # 4. 启动设备发现服务
        logger.info("\n4️⃣ 启动设备发现服务...")
        discovery_service = get_discovery_service()

        # 启动发现服务
        discovery_started = await discovery_service.start_discovery()

        if discovery_started:
            logger.info("✅ 设备发现服务启动成功")

            # 等待设备发现
            logger.info("🔍 正在发现设备...")
            await asyncio.sleep(2)

            # 执行设备发现
            discovered_devices = await discovery_service.discover_once()

            logger.info(f"✅ 发现 {len(discovered_devices)} 个设备")
            for device in discovered_devices:
                logger.info(f"   - {device.name} ({device.device_type.value}, {device.protocol.value})")
        else:
            logger.warning("⚠️ 设备发现服务启动失败，使用模拟设备")

            # 添加模拟设备
            mock_device = DeviceInfo(
                device_id="wifi_mock_light_001",
                name="模拟WiFi灯",
                device_type=DeviceType.LIGHT,
                protocol=ProtocolType.WIFI,
                manufacturer="模拟厂商",
                model="Mock Light",
                location="客厅",
                capabilities=[DeviceCapability.ON_OFF, DeviceCapability.DIMMABLE]
            )

            # 创建设备实例并注册
            from smart_home.core.device_interface import DeviceInterface

            class MockDevice(DeviceInterface):
                async def connect(self):
                    return True

                async def disconnect(self):
                    return True

                async def send_command(self, command):
                    logger.info(f"💡 模拟设备控制: {command.command_type} - {command.parameters}")
                    return True

                async def get_status(self):
                    return type('Status', (), {'to_dict': lambda self: {}})()

                async def start_monitoring(self):
                    pass

                async def stop_monitoring(self):
                    pass

            mock_device_instance = MockDevice(mock_device)
            device_registry.register_device(mock_device_instance)
            adapter_manager.set_device_adapter(mock_device.device_id, ProtocolType.WIFI)

        # 5. 获取设备列表
        logger.info("\n5️⃣ 设备列表:")
        devices = device_registry.list_devices()
        logger.info(f"   总计: {len(devices)} 个设备")

        for device in devices:
            logger.info(f"   - {device.info.name} ({device.info.device_id})")
            logger.info(f"     类型: {device.info.device_type.value}")
            logger.info(f"     协议: {device.info.protocol.value}")
            logger.info(f"     位置: {device.info.location}")
            logger.info(f"     能力: {[cap.value for cap in device.info.capabilities]}")

        # 6. 设备控制示例
        if devices:
            logger.info("\n6️⃣ 设备控制示例:")
            control_service = get_control_service()

            device = devices[0]
            device_id = device.info.device_id

            # 控制设备开灯
            logger.info(f"💡 发送开灯命令: {device.info.name}")
            result = await control_service.control_device(
                device_id=device_id,
                command_type='turn_on',
                parameters={'brightness': 80},
                source='example'
            )

            logger.info(f"   结果: {'✅ 成功' if result.success else '❌ 失败'}")
            logger.info(f"   消息: {result.message}")

            # 获取设备状态
            logger.info(f"📊 获取设备状态: {device.info.name}")
            status_result = await control_service.get_device_status(device_id)

            if status_result.success:
                logger.info(f"   状态数据: {json.dumps(status_result.data, indent=2, ensure_ascii=False)}")

        # 7. 批量设备控制
        logger.info("\n7️⃣ 批量设备控制示例:")
        if len(devices) > 1:
            commands = [
                {
                    'device_id': devices[0].info.device_id,
                    'command_type': 'turn_on',
                    'parameters': {},
                    'source': 'batch'
                },
                {
                    'device_id': devices[1].info.device_id,
                    'command_type': 'turn_off',
                    'parameters': {},
                    'source': 'batch'
                }
            ]

            results = await control_service.control_multiple_devices(commands, parallel=True)

            for i, result in enumerate(results):
                device_name = devices[i].info.name if i < len(devices) else 'Unknown'
                logger.info(f"   {device_name}: {'✅ 成功' if result.success else '❌ 失败'}")

        # 8. 统计信息
        logger.info("\n8️⃣ 系统统计信息:")
        stats = control_service.get_statistics()
        logger.info(f"   适配器统计: {json.dumps(stats['adapter_statistics'], indent=2, ensure_ascii=False)}")
        logger.info(f"   注册表统计: {json.dumps(stats['registry_statistics'], indent=2, ensure_ascii=False)}")

        # 9. 停止服务
        logger.info("\n9️⃣ 停止服务...")
        await discovery_service.stop_discovery()
        logger.info("✅ 服务已停止")

        logger.info("\n" + "=" * 50)
        logger.info("✅ 智能家居模块示例运行完成")

    except Exception as e:
        logger.error(f"❌ 运行异常: {e}", exc_info=True)


if __name__ == '__main__':
    # 运行示例
    asyncio.run(main())
