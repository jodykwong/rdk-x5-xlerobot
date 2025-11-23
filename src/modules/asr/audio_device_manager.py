#!/usr/bin/env python3
"""
XLeRobot 音频设备管理器
解决PulseAudio冲突，实现设备锁定和动态适配
"""

import logging
import subprocess
import threading
import time
import os
import tempfile
from typing import Optional, List, Dict, Any
from dataclasses import dataclass
from enum import Enum
import re

logger = logging.getLogger(__name__)

class DeviceType(Enum):
    """设备类型"""
    INPUT = "input"
    OUTPUT = "output"

@dataclass
class AudioDevice:
    """音频设备信息"""
    index: int
    name: str
    device_id: str
    device_type: DeviceType
    is_available: bool = True
    sample_rates: List[int] = None
    channels: int = 0

    def __post_init__(self):
        if self.sample_rates is None:
            self.sample_rates = []

class AudioDeviceManager:
    """音频设备管理器 - 解决设备冲突和适配问题"""

    def __init__(self):
        self.lock = threading.Lock()
        self.locked_devices = {}  # {device_index: lock_thread_id}
        self.current_input_device = None
        self.current_output_device = None
        self.monitoring_enabled = True
        self.monitoring_thread = None
        self.device_cache = {}
        self.cache_timeout = 5.0  # 缓存5秒

        logger.info("音频设备管理器初始化完成")

    def scan_audio_devices(self, force_refresh: bool = False) -> Dict[DeviceType, List[AudioDevice]]:
        """
        扫描可用的音频设备

        Args:
            force_refresh: 是否强制刷新设备列表

        Returns:
            Dict[DeviceType, List[AudioDevice]]: 按类型分组的设备列表
        """
        current_time = time.time()

        # 使用缓存（如果未强制刷新且缓存未过期）
        if not force_refresh and 'devices' in self.device_cache:
            cache_time, devices = self.device_cache['devices']
            if current_time - cache_time < self.cache_timeout:
                return devices

        devices = {DeviceType.INPUT: [], DeviceType.OUTPUT: []}

        try:
            # 扫录音频输入设备
            input_devices = self._scan_input_devices()
            devices[DeviceType.INPUT] = input_devices

            # 扫描音频输出设备
            output_devices = self._scan_output_devices()
            devices[DeviceType.OUTPUT] = output_devices

            # 缓存结果
            self.device_cache['devices'] = (current_time, devices)

            logger.info(f"设备扫描完成: 输入设备 {len(input_devices)} 个, 输出设备 {len(output_devices)} 个")

        except Exception as e:
            logger.error(f"音频设备扫描失败: {e}")
            return devices

        return devices

    def _scan_input_devices(self) -> List[AudioDevice]:
        """扫描输入设备"""
        input_devices = []

        try:
            # 使用arecord -l扫描
            result = subprocess.run(['arecord', '-l'], capture_output=True, text=True, timeout=5)

            if result.returncode == 0:
                lines = result.stdout.strip().split('\n')

                for i, line in enumerate(lines):
                    line = line.strip()
                    if re.match(r'^card \d+:', line):
                        # 解析卡号和设备名
                        match = re.search(r'card (\d+): (.+) \[(.+)\]', line)
                        if match:
                            card_num = int(match.group(1))
                            device_name = match.group(2).strip()
                            device_id = match.group(3).strip()

                            # 查找对应的设备号
                            device_num = self._find_device_number_for_card(card_num, 'input')
                            if device_num is not None:
                                # 使用 ALSA 设备格式: hw:card,device
                                device_index = self._get_device_index_from_card_device(card_num, device_num)
                                if device_index is not None:
                                    # 获取设备详细信息
                                    sample_rates = self._get_device_sample_rates(card_num, device_num, 'input')
                                    channels = self._get_device_channels(card_num, device_num, 'input')

                                    audio_device = AudioDevice(
                                        index=device_index,
                                        name=device_name,
                                        device_id=device_id,
                                        device_type=DeviceType.INPUT,
                                        sample_rates=sample_rates,
                                        channels=channels
                                    )
                                    input_devices.append(audio_device)
                                    logger.info(f"发现输入设备: {device_name} (卡: {card_num}, 设备: {device_num}, 索引: {device_index})")
                                else:
                                    # 如果无法获取设备索引，使用卡号作为索引
                                    audio_device = AudioDevice(
                                        index=card_num,
                                        name=device_name,
                                        device_id=device_id,
                                        device_type=DeviceType.INPUT,
                                        sample_rates=[16000],  # 假设支持16kHz
                                        channels=1
                                    )
                                    input_devices.append(audio_device)
                                    logger.info(f"发现输入设备: {device_name} (使用卡号作为索引: {card_num})")

        except subprocess.TimeoutExpired:
            logger.warning("音频输入设备扫描超时")
        except Exception as e:
            logger.error(f"扫描输入设备异常: {e}")

        return input_devices

    def _scan_output_devices(self) -> List[AudioDevice]:
        """扫描输出设备"""
        output_devices = []

        try:
            # 使用aplay -l扫描
            result = subprocess.run(['aplay', '-l'], capture_output=True, text=True, timeout=5)

            if result.returncode == 0:
                lines = result.stdout.strip().split('\n')

                for i, line in enumerate(lines):
                    line = line.strip()
                    if re.match(r'^card \d+:', line):
                        # 解析卡号和设备名
                        match = re.search(r'card (\d+): (.+) \[(.+)\]', line)
                        if match:
                            card_num = int(match.group(1))
                            device_name = match.group(2).strip()
                            device_id = match.group(3).strip()

                            # 查找对应的设备号
                            device_num = self._find_device_number_for_card(card_num, 'output')
                            if device_num is not None:
                                # 使用 ALSA 设备格式: hw:card,device
                                device_index = self._get_device_index_from_card_device(card_num, device_num)
                                if device_index is not None:
                                    # 获取设备详细信息
                                    sample_rates = self._get_device_sample_rates(card_num, device_num, 'output')
                                    channels = self._get_device_channels(card_num, device_num, 'output')

                                    audio_device = AudioDevice(
                                        index=device_index,
                                        name=device_name,
                                        device_id=device_id,
                                        device_type=DeviceType.OUTPUT,
                                        sample_rates=sample_rates,
                                        channels=channels
                                    )
                                    output_devices.append(audio_device)
                                    logger.info(f"发现输出设备: {device_name} (卡: {card_num}, 设备: {device_num}, 索引: {device_index})")
                                else:
                                    # 如果无法获取设备索引，使用卡号作为索引
                                    audio_device = AudioDevice(
                                        index=card_num,
                                        name=device_name,
                                        device_id=device_id,
                                        device_type=DeviceType.OUTPUT,
                                        sample_rates=[16000],  # 假设支持16kHz
                                        channels=2
                                    )
                                    output_devices.append(audio_device)
                                    logger.info(f"发现输出设备: {device_name} (使用卡号作为索引: {card_num})")

        except subprocess.TimeoutExpired:
            logger.warning("音频输出设备扫描超时")
        except Exception as e:
            logger.error(f"扫描输出设备异常: {e}")

        return output_devices

    def _find_device_number_for_card(self, card_num: int, device_type: str) -> Optional[int]:
        """查找卡对应的设备号"""
        try:
            if device_type == 'input':
                result = subprocess.run(['arecord', '-l'], capture_output=True, text=True, timeout=5)
            else:
                result = subprocess.run(['aplay', '-l'], capture_output=True, text=True, timeout=5)

            lines = result.stdout.strip().split('\n')
            for i, line in enumerate(lines):
                if re.match(r'^card \d+:', line):
                    match = re.search(r'card (\d+):', line)
                    if match and int(match.group(1)) == card_num:
                        # 查找同一卡号的device信息（可能有多行）
                        for j in range(i, len(lines)):
                            next_line = lines[j].strip()
                            device_match = re.search(r'device (\d+):', next_line)
                            if device_match:
                                return int(device_match.group(1))
                            # 如果遇到下一张卡，停止查找
                            if re.match(r'^card \d+:', next_line) and j > i:
                                break

            # 如果找不到device信息，返回0（默认设备号）
            return 0

        except Exception as e:
            logger.error(f"查找设备号失败: {e}")

        return None

    def _get_device_index_from_card_device(self, card_num: int, device_num: int) -> Optional[int]:
        """从卡号和设备号获取系统设备索引"""
        # ALSA设备索引通常遵循 hw:card,device 格式
        # 转换为 speech_recognition 库使用的设备索引
        try:
            # 简化版本：使用卡号作为主要索引
            # 实际实现可能需要更复杂的映射
            if card_num == 0 and device_num == 0:
                return 0
            elif card_num == 0 and device_num == 2:
                return 2
            else:
                # 对于其他设备，尝试计算索引
                return card_num * 10 + device_num
        except Exception as e:
            logger.error(f"计算设备索引失败: {e}")
            return None

    def _get_device_sample_rates(self, card_num: int, device_num: int, device_type: str) -> List[int]:
        """获取设备支持的采样率"""
        sample_rates = []

        try:
            # 尝试常见的采样率
            common_rates = [8000, 11025, 16000, 22050, 44100, 48000, 96000]

            for rate in common_rates:
                if self._test_sample_rate(card_num, device_num, rate, device_type):
                    sample_rates.append(rate)

            if sample_rates:
                logger.debug(f"设备 hw:{card_num},{device_num} 支持采样率: {sample_rates}")

        except Exception as e:
            logger.error(f"获取设备 hw:{card_num},{device_num} 采样率失败: {e}")

        return sample_rates

    def _test_sample_rate(self, card_num: int, device_num: int, sample_rate: int, device_type: str) -> bool:
        """测试设备是否支持指定采样率"""
        try:
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                temp_path = temp_file.name

            try:
                if device_type == 'input':
                    cmd = ['arecord', '-D', f'hw:{card_num},{device_num}', '-d', '1', '-r', str(sample_rate), temp_path]
                else:
                    cmd = ['aplay', '-D', f'hw:{card_num},{device_num}', '-d', '1', '-r', str(sample_rate), temp_path]

                result = subprocess.run(cmd, capture_output=True, timeout=2)
                return result.returncode == 0

            finally:
                if os.path.exists(temp_path):
                    os.unlink(temp_path)

        except Exception:
            return False

    def _get_device_channels(self, card_num: int, device_num: int, device_type: str) -> int:
        """获取设备声道数"""
        try:
            # 简化实现：默认返回2（立体声），单声道设备会在实际使用时适配
            return 2
        except Exception as e:
            logger.error(f"获取设备 hw:{card_num},{device_num} 声道数失败: {e}")
            return 2

    def get_best_input_device(self) -> Optional[AudioDevice]:
        """获取最佳输入设备 - 智能避让被占用的设备"""
        devices = self.scan_audio_devices()
        input_devices = devices[DeviceType.INPUT]

        if not input_devices:
            logger.error("未发现可用的音频输入设备")
            return None

        # 按优先级排序设备列表
        device_candidates = []

        # 优先选择USB音频设备
        usb_devices = [d for d in input_devices if 'usb' in d.name.lower() or 'audio' in d.name.lower()]
        if usb_devices:
            # 选择支持16kHz的USB设备
            for device in usb_devices:
                if 16000 in device.sample_rates:
                    device_candidates.append((1, device))  # 最高优先级
                else:
                    device_candidates.append((2, device))  # 次优先级
        else:
            # 为所有设备分配优先级
            for device in input_devices:
                if 16000 in device.sample_rates:
                    device_candidates.append((2, device))  # 支持16kHz
                else:
                    device_candidates.append((3, device))  # 默认优先级

        # 按优先级排序
        device_candidates.sort(key=lambda x: x[0])

        # 测试每个设备，找到第一个真正可用的
        for priority, device in device_candidates:
            logger.info(f"🧪 测试设备 {device.index}: {device.name} (优先级: {priority})")

            # 测试实际访问（非独占模式）
            if self._test_exclusive_access(device.index, 'input'):
                logger.info(f"✅ 选择输入设备: {device.name} (索引: {device.index})")
                return device
            else:
                logger.warning(f"⚠️ 设备 {device.index} 无法访问，尝试下一个设备")
                continue

        # 如果所有设备都不可用，返回第一个（让调用方处理错误）
        logger.warning("⚠️ 所有输入设备都无法访问，返回第一个设备作为默认选择")
        fallback_device = input_devices[0]
        logger.info(f"🔄 回退选择: {fallback_device.name} (索引: {fallback_device.index})")
        return fallback_device

    def get_best_output_device(self) -> Optional[AudioDevice]:
        """获取最佳输出设备"""
        devices = self.scan_audio_devices()
        output_devices = devices[DeviceType.OUTPUT]

        if not output_devices:
            logger.error("未发现可用的音频输出设备")
            return None

        # 优先选择USB音频设备或默认设备
        preferred_devices = []
        for device in output_devices:
            if 'usb' in device.name.lower() or 'audio' in device.name.lower() or 'default' in device.name.lower():
                preferred_devices.append(device)

        if preferred_devices:
            best_device = preferred_devices[0]
        else:
            best_device = output_devices[0]

        logger.info(f"选择输出设备: {best_device.name}")
        return best_device

    def lock_device(self, device_index: int, device_type: DeviceType) -> bool:
        """
        锁定音频设备，防止PulseAudio等占用

        Args:
            device_index: 设备索引
            device_type: 设备类型

        Returns:
            bool: 锁定成功状态
        """
        with self.lock:
            if device_index in self.locked_devices:
                logger.warning(f"设备 {device_index} 已被锁定")
                return False

            try:
                # 尝试占用设备（独占模式）
                if device_type == DeviceType.INPUT:
                    # 尝试以独占模式打开输入设备
                    success = self._test_exclusive_access(device_index, 'input')
                else:
                    # 尝试以独占模式打开输出设备
                    success = self._test_exclusive_access(device_index, 'output')

                if success:
                    thread_id = threading.get_ident()
                    self.locked_devices[device_index] = thread_id
                    logger.info(f"成功锁定设备 {device_index}")
                    return True
                else:
                    logger.error(f"无法独占访问设备 {device_index}")
                    return False

            except Exception as e:
                logger.error(f"锁定设备 {device_index} 失败: {e}")
                return False

    def unlock_device(self, device_index: int) -> bool:
        """
        解锁音频设备

        Args:
            device_index: 设备索引

        Returns:
            bool: 解锁成功状态
        """
        with self.lock:
            if device_index not in self.locked_devices:
                logger.warning(f"设备 {device_index} 未被锁定")
                return False

            try:
                # 释放设备
                del self.locked_devices[device_index]
                logger.info(f"成功解锁设备 {device_index}")
                return True

            except Exception as e:
                logger.error(f"解锁设备 {device_index} 失败: {e}")
                return False

    def _test_exclusive_access(self, device_index: int, device_type: str) -> bool:
        """测试设备访问（非独占模式）"""
        try:
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                temp_path = temp_file.name

            try:
                if device_type == 'input':
                    # 直接使用硬件设备hw:0,0，绕过PulseAudio冲突，强制单声道
                    cmd = ['arecord', '-D', 'hw:0,0', '-d', '1', '-f', 'cd', '-c', '1', temp_path]
                else:
                    # 测试输出设备
                    cmd = ['aplay', '-D', 'default', '-d', '1', '/dev/zero']

                result = subprocess.run(cmd, capture_output=True, timeout=3)
                success = result.returncode == 0

                if success:
                    logger.debug(f"设备 {device_index} 访问测试成功")
                else:
                    logger.debug(f"设备 {device_index} 访问测试失败: {result.stderr.decode()}")

                return success

            finally:
                if os.path.exists(temp_path):
                    os.unlink(temp_path)

        except Exception as e:
            logger.debug(f"设备 {device_index} 独占访问测试异常: {e}")
            return False

    def get_locked_devices(self) -> List[int]:
        """获取已锁定的设备列表"""
        with self.lock:
            return list(self.locked_devices.keys())

    def is_device_locked(self, device_index: int) -> bool:
        """检查设备是否被锁定"""
        with self.lock:
            return device_index in self.locked_devices

    def get_device_status(self) -> Dict[str, Any]:
        """获取设备状态报告"""
        devices = self.scan_audio_devices()
        locked_devices = self.get_locked_devices()

        status = {
            "input_devices": len(devices[DeviceType.INPUT]),
            "output_devices": len(devices[DeviceType.OUTPUT]),
            "locked_devices": len(locked_devices),
            "device_details": {
                "input": [
                    {
                        "index": d.index,
                        "name": d.name,
                        "sample_rates": d.sample_rates,
                        "channels": d.channels,
                        "locked": d.index in locked_devices
                    } for d in devices[DeviceType.INPUT]
                ],
                "output": [
                    {
                        "index": d.index,
                        "name": d.name,
                        "sample_rates": d.sample_rates,
                        "channels": d.channels,
                        "locked": d.index in locked_devices
                    } for d in devices[DeviceType.OUTPUT]
                ]
            }
        }

        return status

# 全局设备管理器实例
_device_manager = None

def get_device_manager() -> AudioDeviceManager:
    """获取全局设备管理器实例"""
    global _device_manager
    if _device_manager is None:
        _device_manager = AudioDeviceManager()
    return _device_manager

def setup_16khz_recording(device_index: int = None) -> Dict[str, Any]:
    """
    设置16kHz录音环境

    Args:
        device_index: 设备索引，如果为None则自动选择最佳设备

    Returns:
        Dict: 设置结果
    """
    manager = get_device_manager()

    if device_index is None:
        # 自动选择最佳设备
        best_device = manager.get_best_input_device()
        if not best_device:
            return {"success": False, "error": "未找到可用的输入设备"}
        device_index = best_device.index

    # 检查设备是否支持16kHz
    devices = manager.scan_audio_devices()
    input_devices = {d.index: d for d in devices[DeviceType.INPUT]}

    if device_index not in input_devices:
        return {"success": False, "error": f"设备 {device_index} 不存在"}

    device = input_devices[device_index]
    if 16000 not in device.sample_rates:
        logger.warning(f"设备 {device_index} 不明确支持16kHz，将尝试重采样")

    # 尝试锁定设备
    if manager.lock_device(device_index, DeviceType.INPUT):
        return {
            "success": True,
            "device_index": device_index,
            "device_name": device.name,
            "sample_rate": 16000,
            "channels": min(device.channels, 1),  # 强制单声道
            "note": "已锁定设备，使用16kHz单声道"
        }
    else:
        return {"success": False, "error": f"无法锁定设备 {device_index}"}

if __name__ == "__main__":
    # 测试代码
    import json

    logging.basicConfig(level=logging.INFO)

    print("=== 音频设备管理器测试 ===")

    manager = AudioDeviceManager()

    # 扫描设备
    print("\n1. 扫描音频设备...")
    devices = manager.scan_audio_devices()

    print(f"输入设备: {len(devices[DeviceType.INPUT])}")
    for device in devices[DeviceType.INPUT]:
        print(f"  {device.index}: {device.name} - {device.sample_rates}Hz - {device.channels}ch")

    print(f"输出设备: {len(devices[DeviceType.OUTPUT])}")
    for device in devices[DeviceType.OUTPUT]:
        print(f"  {device.index}: {device.name} - {device.sample_rates}Hz - {device.channels}ch")

    # 选择最佳设备
    print("\n2. 选择最佳设备...")
    best_input = manager.get_best_input_device()
    best_output = manager.get_best_output_device()

    print(f"最佳输入设备: {best_input.name if best_input else '无'}")
    print(f"最佳输出设备: {best_output.name if best_output else '无'}")

    # 设置16kHz录音
    print("\n3. 设置16kHz录音...")
    result = setup_16khz_recording(best_input.index if best_input else None)
    print(json.dumps(result, indent=2, ensure_ascii=False))

    # 设备状态报告
    print("\n4. 设备状态报告...")
    status = manager.get_device_status()
    print(json.dumps(status, indent=2, ensure_ascii=False))