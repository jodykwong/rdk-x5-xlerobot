#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
设备控制服务

提供统一的设备控制接口，支持多种协议设备的控制命令。

作者: Dev Agent
故事ID: Story 5.1
Epic: 5 - 智能家居控制模块
"""

import asyncio
import logging
from typing import Dict, List, Optional, Any, Tuple
import time

from ..core.device_interface import DeviceInterface, DeviceCommand, ProtocolType
from ..core.protocol_adapter import get_adapter_manager
from ..core.device_registry import get_device_registry, DeviceNotFoundError


logger = logging.getLogger(__name__)


class DeviceControlResult:
    """设备控制结果"""

    def __init__(self, success: bool, device_id: str, message: str = "", data: Any = None):
        self.success = success
        self.device_id = device_id
        self.message = message
        self.data = data
        self.timestamp = time.time()

    def to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        return {
            'success': self.success,
            'device_id': self.device_id,
            'message': self.message,
            'data': self.data,
            'timestamp': self.timestamp,
        }


class DeviceControlService:
    """
    设备控制服务

    提供统一的设备控制接口
    """

    def __init__(self):
        """初始化设备控制服务"""
        self.adapter_manager = get_adapter_manager()
        self.device_registry = get_device_registry()

        logger.info("✅ 设备控制服务初始化完成")

    async def control_device(
        self,
        device_id: str,
        command_type: str,
        parameters: Dict[str, Any] = None,
        source: str = "manual",
        timeout: float = 5.0
    ) -> DeviceControlResult:
        """
        控制单个设备

        Args:
            device_id: 设备ID
            command_type: 命令类型
            parameters: 命令参数
            source: 命令来源
            timeout: 超时时间

        Returns:
            DeviceControlResult: 控制结果
        """
        try:
            # 获取设备
            device = self.device_registry.get_device(device_id)

            # 验证设备能力
            from ..core.device_interface import DeviceCapability
            if command_type == 'turn_on' or command_type == 'turn_off':
                if not device.has_capability(DeviceCapability.ON_OFF):
                    return DeviceControlResult(
                        False, device_id,
                        f"设备不支持开关命令",
                        {'required_capability': DeviceCapability.ON_OFF.value}
                    )

            # 构造命令
            command = DeviceCommand(
                command_type=command_type,
                parameters=parameters or {},
                source=source,
                timeout=timeout
            )

            # 发送命令
            start_time = time.time()
            success = await device.send_command(command)
            response_time = time.time() - start_time

            if success:
                logger.info(f"✅ 设备控制成功: {device_id} - {command_type} ({response_time:.2f}s)")
                return DeviceControlResult(
                    True, device_id,
                    f"控制成功",
                    {
                        'command_type': command_type,
                        'parameters': parameters,
                        'response_time': response_time
                    }
                )
            else:
                return DeviceControlResult(
                    False, device_id,
                    f"控制失败: 设备无响应",
                    {'command_type': command_type}
                )

        except DeviceNotFoundError:
            return DeviceControlResult(False, device_id, f"设备未找到")
        except Exception as e:
            logger.error(f"❌ 设备控制异常: {device_id} - {e}")
            return DeviceControlResult(False, device_id, f"控制异常: {str(e)}")

    async def control_multiple_devices(
        self,
        commands: List[Dict[str, Any]],
        parallel: bool = True,
        timeout: float = 10.0
    ) -> List[DeviceControlResult]:
        """
        批量控制多个设备

        Args:
            commands: 命令列表 [{device_id, command_type, parameters}]
            parallel: 是否并行执行
            timeout: 总超时时间

        Returns:
            List[DeviceControlResult]: 控制结果列表
        """
        logger.info(f"🎮 批量设备控制: {len(commands)} 个命令, {'并行' if parallel else '串行'}")

        start_time = time.time()

        if parallel:
            # 并行执行
            tasks = []
            for cmd in commands:
                task = asyncio.create_task(
                    self.control_device(
                        cmd['device_id'],
                        cmd.get('command_type', 'on_off'),
                        cmd.get('parameters'),
                        cmd.get('source', 'batch')
                    )
                )
                tasks.append(task)

            try:
                results = await asyncio.wait_for(
                    asyncio.gather(*tasks, return_exceptions=True),
                    timeout=timeout
                )

                # 处理异常结果
                processed_results = []
                for i, result in enumerate(results):
                    if isinstance(result, Exception):
                        processed_results.append(
                            DeviceControlResult(
                                False, commands[i].get('device_id', 'unknown'),
                                f"执行异常: {str(result)}"
                            )
                        )
                    else:
                        processed_results.append(result)

                return processed_results

            except asyncio.TimeoutError:
                logger.error(f"❌ 批量控制超时 ({timeout}s)")
                return [
                    DeviceControlResult(False, cmd.get('device_id', 'unknown'), "批量控制超时")
                    for cmd in commands
                ]

        else:
            # 串行执行
            results = []
            for cmd in commands:
                result = await self.control_device(
                    cmd['device_id'],
                    cmd.get('command_type', 'on_off'),
                    cmd.get('parameters'),
                    cmd.get('source', 'batch')
                )
                results.append(result)

            return results

    async def get_device_status(self, device_id: str) -> DeviceControlResult:
        """
        获取设备状态

        Args:
            device_id: 设备ID

        Returns:
            DeviceControlResult: 获取结果
        """
        try:
            device = self.device_registry.get_device(device_id)
            status = await device.get_status()

            logger.info(f"📊 获取设备状态成功: {device_id}")

            return DeviceControlResult(
                True, device_id, "状态获取成功", status.to_dict()
            )

        except DeviceNotFoundError:
            return DeviceControlResult(False, device_id, "设备未找到")
        except Exception as e:
            logger.error(f"❌ 获取设备状态失败: {device_id} - {e}")
            return DeviceControlResult(False, device_id, f"获取异常: {str(e)}")

    async def get_all_devices_status(self) -> Dict[str, Any]:
        """
        获取所有设备状态

        Returns:
            Dict[str, Any]: 所有设备状态
        """
        try:
            devices = self.device_registry.list_devices()
            results = {
                'total': len(devices),
                'online': 0,
                'offline': 0,
                'devices': {}
            }

            for device in devices:
                try:
                    status = await device.get_status()
                    device_id = device.info.device_id

                    results['devices'][device_id] = status.to_dict()

                    if status.state.is_online:
                        results['online'] += 1
                    else:
                        results['offline'] += 1

                except Exception as e:
                    logger.warning(f"⚠️ 获取设备状态失败 {device.info.device_id}: {e}")
                    results['devices'][device.info.device_id] = {
                        'error': str(e),
                        'is_online': False
                    }
                    results['offline'] += 1

            logger.info(f"📊 设备状态统计: 总计 {results['total']}, 在线 {results['online']}, 离线 {results['offline']}")

            return results

        except Exception as e:
            logger.error(f"❌ 获取所有设备状态失败: {e}")
            return {'error': str(e)}

    async def connect_device(self, device_id: str) -> DeviceControlResult:
        """
        连接设备

        Args:
            device_id: 设备ID

        Returns:
            DeviceControlResult: 连接结果
        """
        try:
            device = self.device_registry.get_device(device_id)

            if device.is_connected():
                return DeviceControlResult(
                    True, device_id, "设备已连接", {}
                )

            success = await device.connect()

            if success:
                logger.info(f"✅ 设备连接成功: {device_id}")
                return DeviceControlResult(
                    True, device_id, "设备连接成功", {}
                )
            else:
                return DeviceControlResult(
                    False, device_id, "设备连接失败", {}
                )

        except DeviceNotFoundError:
            return DeviceControlResult(False, device_id, "设备未找到")
        except Exception as e:
            logger.error(f"❌ 设备连接异常: {device_id} - {e}")
            return DeviceControlResult(False, device_id, f"连接异常: {str(e)}")

    async def disconnect_device(self, device_id: str) -> DeviceControlResult:
        """
        断开设备

        Args:
            device_id: 设备ID

        Returns:
            DeviceControlResult: 断开结果
        """
        try:
            device = self.device_registry.get_device(device_id)

            if not device.is_connected():
                return DeviceControlResult(
                    True, device_id, "设备已断开", {}
                )

            success = await device.disconnect()

            if success:
                logger.info(f"✅ 设备断开成功: {device_id}")
                return DeviceControlResult(
                    True, device_id, "设备断开成功", {}
                )
            else:
                return DeviceControlResult(
                    False, device_id, "设备断开失败", {}
                )

        except DeviceNotFoundError:
            return DeviceControlResult(False, device_id, "设备未找到")
        except Exception as e:
            logger.error(f"❌ 设备断开异常: {device_id} - {e}")
            return DeviceControlResult(False, device_id, f"断开异常: {str(e)}")

    def get_statistics(self) -> Dict[str, Any]:
        """
        获取控制服务统计信息

        Returns:
            Dict[str, Any]: 统计信息
        """
        return {
            'adapter_statistics': self.adapter_manager.get_statistics(),
            'registry_statistics': self.device_registry.get_statistics(),
        }


# 全局设备控制服务实例
_control_service = None


def get_control_service() -> DeviceControlService:
    """
    获取全局设备控制服务实例

    Returns:
        DeviceControlService: 设备控制服务实例
    """
    global _control_service
    if _control_service is None:
        _control_service = DeviceControlService()
    return _control_service
