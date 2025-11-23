#!/usr/bin/env python3
"""
XLeRobot 改进版ASR系统
集成音频设备管理、16kHz直接录音、统一Token管理、WebSocket稳定性
"""

import asyncio
import logging
import sys
import time
import threading
import tempfile
import os
import json
from pathlib import Path
from typing import Optional, Dict, Any, List
from enum import Enum

# 导入改进的组件
from .audio_device_manager import get_device_manager, setup_16khz_recording
from .direct_16khz_recorder import create_direct_16khz_recorder, AudioChunk

# 使用现有的Token管理器（已经过验证）
try:
    from aliyun_nls_token_manager import get_token_manager
    EXISTING_TOKEN_MANAGER_AVAILABLE = True
    logger.info("✅ 现有Token管理器可用")
except ImportError as e:
    EXISTING_TOKEN_MANAGER_AVAILABLE = False
    logger.warning(f"⚠️ 现有Token管理器不可用: {e}")
    from .unified_token_manager import get_unified_token_manager, get_valid_token

from .websocket_stability_manager import WebSocketStabilityManager, ConnectionState

logger = logging.getLogger(__name__)

class ASRState(Enum):
    """ASR系统状态枚举"""
    IDLE = "idle"
    INITIALIZING = "initializing"
    READY = "ready"
    LISTENING = "listening"
    PROCESSING = "processing"
    ERROR = "error"

class ImprovedASRSystem:
    """
    改进版ASR系统

    主要改进：
    1. 音频设备管理 - 解决PulseAudio冲突
    2. 16kHz直接录音 - 避免重采样延迟
    3. 统一Token管理 - 解决Token冲突
    4. WebSocket稳定性 - 自动重连和错误恢复
    5. 完整的错误处理和监控
    """

    def __init__(self):
        # 核心组件
        self.device_manager = get_device_manager()

        # 使用现有的Token管理器
        if EXISTING_TOKEN_MANAGER_AVAILABLE:
            self.token_manager = get_token_manager()
        else:
            self.token_manager = get_unified_token_manager()

        self.stability_manager = WebSocketStabilityManager()

        # 录音组件
        self.recorder = None
        self.current_device_index = None

        # 系统状态
        self.state = ASRState.IDLE
        self.is_running = False
        self.stop_event = threading.Event()

        # 回调函数
        self.on_wake_word_detected = None
        self.on_speech_recognized = None
        self.on_error = None

        # 监听循环线程
        self.listening_thread = None

        # 统计信息
        self.stats = {
            "start_time": None,
            "total_listening_time": 0.0,
            "wake_detections": 0,
            "speech_recognitions": 0,
            "errors": 0
        }

        logger.info("改进版ASR系统初始化完成")

    def initialize(self) -> bool:
        """初始化ASR系统"""
        try:
            self.state = ASRState.INITIALIZING
            logger.info("🚀 开始初始化改进版ASR系统...")

            # 1. 验证环境配置
            if not self._verify_environment():
                logger.error("环境配置验证失败")
                return False

            # 2. 初始化Token管理
            if not self._initialize_token_manager():
                logger.error("Token管理器初始化失败")
                return False

            # 3. 初始化音频设备
            if not self._initialize_audio_device():
                logger.error("音频设备初始化失败")
                return False

            # 4. 初始化录音器
            if not self._initialize_recorder():
                logger.error("录音器初始化失败")
                return False

            # 5. 初始化稳定性管理器回调
            self._setup_stability_callbacks()

            self.state = ASRState.READY
            logger.info("✅ 改进版ASR系统初始化完成")
            return True

        except Exception as e:
            logger.error(f"❌ ASR系统初始化失败: {e}")
            self.state = ASRState.ERROR
            return False

    def _verify_environment(self) -> bool:
        """验证环境配置"""
        try:
            # 检查必需的环境变量
            required_vars = [
                "ALIBABA_CLOUD_ACCESS_KEY_ID",
                "ALIBABA_CLOUD_ACCESS_KEY_SECRET",
                "ALIYUN_NLS_APPKEY"
            ]

            missing_vars = [var for var in required_vars if not os.environ.get(var)]
            if missing_vars:
                logger.error(f"缺少环境变量: {missing_vars}")
                return False

            # 检查Python版本
            if sys.version_info < (3, 10):
                logger.error(f"Python版本过低: {sys.version}")
                return False

            logger.info("✅ 环境配置验证通过")
            return True

        except Exception as e:
            logger.error(f"环境配置验证异常: {e}")
            return False

    def _initialize_token_manager(self) -> bool:
        """初始化Token管理器"""
        try:
            # 检查Token管理器是否可用
            if not self.token_manager:
                logger.error("Token管理器未初始化")
                return False

            if EXISTING_TOKEN_MANAGER_AVAILABLE:
                # 使用现有的Token管理器，无需额外初始化
                logger.info("✅ 使用现有Token管理器，初始化成功")
                return True
            else:
                # 使用新的Token管理器，进行基础健康检查
                health = self.token_manager.health_check()
                logger.info(f"Token管理器状态: {health['status']}")

                # 只要SDK可用，就不阻塞初始化
                if health.get("sdk_ok", False):
                    logger.info("✅ Token管理器初始化成功（SDK可用）")
                    return True
                else:
                    logger.error("Token管理器SDK不可用")
                    return False

        except Exception as e:
            logger.error(f"Token管理器初始化异常: {e}")
            return False

    def _initialize_audio_device(self) -> bool:
        """初始化音频设备"""
        try:
            # 扫描设备
            devices = self.device_manager.scan_audio_devices(force_refresh=True)
            input_devices = devices.get("input", [])

            if not input_devices:
                logger.error("未发现可用的音频输入设备")
                return False

            # 选择最佳设备
            best_device = self.device_manager.get_best_input_device()
            if not best_device:
                logger.error("无法选择最佳音频设备")
                return False

            self.current_device_index = best_device.index
            logger.info(f"✅ 选择音频设备: {best_device.name} (索引: {best_device.index})")

            # 锁定设备
            if not self.device_manager.lock_device(best_device.index, "input"):
                logger.warning("设备锁定失败，但继续尝试")

            return True

        except Exception as e:
            logger.error(f"音频设备初始化异常: {e}")
            return False

    def _initialize_recorder(self) -> bool:
        """初始化录音器"""
        try:
            # 创建16kHz直接录音器
            self.recorder = create_direct_16khz_recorder(
                device_index=self.current_device_index,
                auto_device_selection=False
            )

            # 设置音频回调
            self.recorder.set_silence_threshold(500)
            self.recorder.set_silence_duration(0.5)

            logger.info("✅ 16kHz直接录音器初始化成功")
            return True

        except Exception as e:
            logger.error(f"录音器初始化异常: {e}")
            return False

    def _setup_stability_callbacks(self) -> None:
        """设置稳定性管理器回调"""
        def on_connect():
            logger.info("✅ WebSocket连接已建立")
            self.stats["errors"] = 0  # 重置错误计数

        def on_disconnect():
            logger.warning("❌ WebSocket连接已断开")

        def on_error(error):
            logger.error(f"🔥 WebSocket连接错误: {error}")
            self.stats["errors"] += 1

        self.stability_manager.set_callbacks(
            on_connect=on_connect,
            on_disconnect=on_disconnect,
            on_error=on_error
        )

    def start(self) -> bool:
        """启动ASR系统"""
        if self.is_running:
            logger.warning("ASR系统已在运行")
            return True

        try:
            logger.info("🎤 启动改进版ASR系统...")

            # 检查初始化状态
            if self.state != ASRState.READY:
                if not self.initialize():
                    return False

            # 启动录音
            def audio_callback(chunk: AudioChunk):
                """音频回调处理"""
                try:
                    # 这里可以添加唤醒词检测逻辑
                    # 现在只是记录音频块
                    logger.debug(f"收到音频块: {len(chunk.data)} bytes")

                    # 调用唤醒词检测（如果实现了）
                    if self.on_wake_word_detected:
                        # 简单的音量检测作为唤醒词检测的替代
                        if self._detect_wake_word(chunk):
                            self.on_wake_word_detected()

                except Exception as e:
                    logger.error(f"音频回调处理异常: {e}")

            if not self.recorder.start_recording(audio_callback=audio_callback):
                logger.error("录音器启动失败")
                return False

            self.is_running = True
            self.stats["start_time"] = time.time()
            self.stop_event.clear()

            # 启动监听循环
            self._start_listening_loop()

            logger.info("✅ 改进版ASR系统启动成功")
            return True

        except Exception as e:
            logger.error(f"❌ ASR系统启动失败: {e}")
            return False

    def stop(self) -> None:
        """停止ASR系统"""
        if not self.is_running:
            return

        logger.info("正在停止改进版ASR系统...")

        # 停止事件
        self.stop_event.set()
        self.is_running = False

        # 停止录音器
        if self.recorder:
            self.recorder.stop_recording()

        # 解锁设备
        if self.current_device_index is not None:
            self.device_manager.unlock_device(self.current_device_index)

        # 停止稳定性管理器
        self.stability_manager.disconnect()

        self.state = ASRState.IDLE
        logger.info("改进版ASR系统已停止")

    def _start_listening_loop(self) -> None:
        """启动监听循环"""
        def listening_loop():
            logger.info("🎯 进入监听模式，等待语音输入...")
            total_listening_start = time.time()

            while not self.stop_event.wait(1.0) and self.is_running:
                try:
                    # 从录音器获取音频数据
                    chunk = self.recorder.get_audio_chunk(timeout=0.1)
                    if chunk:
                        # 更新统计
                        self.stats["total_listening_time"] += chunk.duration

                        # 这里可以添加更复杂的处理逻辑
                        # 例如：唤醒词检测、语音识别等

                except Exception as e:
                    logger.error(f"监听循环异常: {e}")
                    self.stats["errors"] += 1

            total_listening_time = time.time() - total_listening_start
            logger.info(f"监听循环结束，总监听时间: {total_listening_time:.2f}秒")

        self.listening_thread = threading.Thread(target=listening_loop, daemon=True)
        self.listening_thread.start()

    def _detect_wake_word(self, chunk: AudioChunk) -> bool:
        """检测唤醒词（简化版本）"""
        try:
            # 转换为numpy数组
            import numpy as np
            audio_array = np.frombuffer(chunk.data, dtype=np.int16)

            # 简单的能量检测作为唤醒词检测
            energy = np.mean(audio_array.astype(np.float32) ** 2)

            # 设置阈值（需要根据实际情况调整）
            threshold = 10000  # 这个值需要根据实际环境调整

            if energy > threshold:
                logger.debug(f"检测到高能量音频: {energy:.2f}")
                self.stats["wake_detections"] += 1
                return True

            return False

        except Exception as e:
            logger.error(f"唤醒词检测异常: {e}")
            return False

    def get_status(self) -> Dict[str, Any]:
        """获取系统状态"""
        return {
            "state": self.state.value,
            "is_running": self.is_running,
            "device_index": self.current_device_index,
            "recorder_status": self.recorder.get_status() if self.recorder else None,
            "token_status": self.token_manager.get_token_info(),
            "websocket_status": self.stability_manager.get_connection_state().value,
            "statistics": self.stats.copy(),
            "device_status": self.device_manager.get_device_status()
        }

    def recognize_speech(self, audio_data: bytes) -> Optional[Dict[str, Any]]:
        """
        识别语音（使用现有的Token管理器）

        Args:
            audio_data: 音频数据

        Returns:
            Dict: 识别结果
        """
        try:
            if EXISTING_TOKEN_MANAGER_AVAILABLE:
                # 使用现有的Token管理器
                token = self.token_manager.get_token()
            else:
                # 使用新的Token管理器
                token = get_valid_token()

            if not token:
                return {"success": False, "error": "Token获取失败"}

            # 这里应该调用实际的ASR服务
            # 由于这是简化版本，返回模拟结果
            result = {
                "success": True,
                "text": "模拟识别结果",
                "confidence": 0.95,
                "timestamp": time.time()
            }

            self.stats["speech_recognitions"] += 1
            return result

        except Exception as e:
            logger.error(f"语音识别异常: {e}")
            self.stats["errors"] += 1
            return {"success": False, "error": str(e)}

if __name__ == "__main__":
    # 测试代码
    import json

    logging.basicConfig(level=logging.INFO)

    print("=== 改进版ASR系统测试 ===")

    # 创建系统
    asr_system = ImprovedASRSystem()

    # 初始化
    print("\n1. 初始化系统...")
    if asr_system.initialize():
        print("✅ 初始化成功")
    else:
        print("❌ 初始化失败")
        exit(1)

    # 获取状态
    print("\n2. 系统状态...")
    status = asr_system.get_status()
    print(json.dumps(status, indent=2, ensure_ascii=False))

    # 启动系统
    print("\n3. 启动系统...")
    if asr_system.start():
        print("✅ 系统启动成功")

        # 运行5秒
        print("🎤 正在监听音频输入...")
        time.sleep(5)

        # 获取运行时状态
        print("\n4. 运行时状态...")
        runtime_status = asr_system.get_status()
        print(json.dumps(runtime_status, indent=2, ensure_ascii=False))

        # 停止系统
        print("\n5. 停止系统...")
        asr_system.stop()
        print("✅ 系统已停止")

    else:
        print("❌ 系统启动失败")

    print("\n测试完成")