#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Epic1 ASR系统核心模块
整合在线ASR服务，支持唤醒词检测和语音识别
"""

import asyncio
import logging
import sys
import time
import inspect
# import speech_recognition as sr  # 替换为线程安全录音器
# from .thread_safe_audio_recorder import ThreadSafeAudioRecorder, RecordingState

# 使用ALSA兼容的音频输入
from .alsa_audio_input import create_hybrid_audio_input, HybridAudioInput
from .audio_recorder_manager import get_recorder_manager, RecordingState
import numpy as np
import wave
import tempfile
import os
import threading
from pathlib import Path
from typing import Optional, Dict, Any
from enum import Enum

# 添加项目路径
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))

# 使用WebSocket ASR服务（根据架构文档要求）
from modules.asr.websocket.websocket_asr_service import AliyunASRWebSocketService
from modules.asr.streaming.wake_word_detector import WakeWordDetector
# ASR桥接不需要TTS功能，暂时注释掉
# from modules.tts.engine.aliyun_tts_engine import AliyunTTSEngine

# 避免导入LLM模块以绕过ROS2依赖
# from modules.llm.qwen_multimodal_llm import QwenMultimodalLLM

logger = logging.getLogger(__name__)

class ASRState(Enum):
    """ASR系统状态枚举"""
    IDLE = "idle"                      # 空闲，等待唤醒词
    WAKE_DETECTED = "wake_detected"    # 检测到唤醒词
    LISTENING_COMMAND = "listening_command"  # 监听用户指令
    PROCESSING = "processing"          # 处理指令
    RESPONDING = "responding"          # 播放回复

# 导入阿里云NLS官方SDK
try:
    import sys
    sys.path.append('/home/sunrise/.local/lib/python3.10/site-packages')
    from nls.token import getToken
    OFFICIAL_SDK_AVAILABLE = True
    logger.info("✅ 阿里云NLS官方SDK可用")
except ImportError as e:
    OFFICIAL_SDK_AVAILABLE = False
    logger.warning(f"⚠️ 官方SDK不可用: {e}")

    # 备用：Token管理器
    try:
        from aliyun_nls_token_manager import get_token_manager
        TOKEN_MANAGER_AVAILABLE = True
    except ImportError:
        TOKEN_MANAGER_AVAILABLE = False

class ASRSystem:
    """Epic1 ASR系统集成器"""

    def __init__(self):
        self.project_root = Path("/home/sunrise/xlerobot")
        self.asr_service = None
        self.wake_detector = None
        self.tts_client = None
        self.llm_client = None
        self.is_running = False
        # 使用ALSA兼容的混合音频输入
        self.audio_input = None  # HybridAudioInput实例
        # self.recognizer = sr.Recognizer()  # 已替换为ThreadSafeAudioRecorder
        self.audio_recorder = None
        self.recording = False
        self.last_wake_time = 0
        self.wake_cooldown = 3  # 🔧 修复：增加默认冷却时间从2秒到3秒
        self.conversation_history = []  # 对话历史
        self.last_response_time = 0  # 🔧 新增：用于循环检测的最后响应时间
        self.max_history_length = 10  # 最大历史记录数

        # 🔧 回声消除：TTS播放状态标志
        self.is_playing_tts = False  # TTS播放期间禁用麦克风，防止回声循环
        self.recent_wake_times = []  # 循环检测：最近3次唤醒时间

        # 状态机管理
        self.state = ASRState.IDLE

        # 新增：结果回调函数 - 用于ROS2桥接
        self.result_callback = None  # 添加此行

        # ROS2播放请求支持
        self.ros2_tts_publisher = None
        self.use_ros2_tts = self._check_ros2_environment()

        # 初始化状态
        self.initialized = False
        self.audio_configured = False

        # 优化音频参数配置（基于ThreadSafeAudioRecorder）
        self._configure_audio_parameters()

    def _configure_audio_parameters(self):
        """配置优化音频参数（基于AudioRecorderManager）"""
        try:
            # 获取录音器管理器并配置
            recorder_manager = get_recorder_manager()
            config = recorder_manager.get_config()

            # 使用管理器的配置
            self.audio_config = config
            self.audio_configured = True

            logger.info("✅ 音频参数配置完成 - AudioRecorderManager统一管理")
            logger.debug(f"📊 音频参数: {config}")

        except Exception as e:
            logger.warning(f"⚠️ 音频参数配置失败，使用默认配置: {e}")
            self.audio_configured = False

    def _check_ros2_environment(self) -> bool:
        """检查是否在ROS2环境中运行"""
        try:
            # 检查ROS2环境变量
            if os.getenv('ROS_DISTRO'):
                try:
                    import rclpy
                    from std_msgs.msg import String
                    logger.info("✅ 检测到ROS2环境，将使用ROS2 TTS播放")
                    return True
                except ImportError:
                    logger.warning("⚠️ 检测到ROS2环境但缺少rclpy，将使用本地播放")
                    return False
            else:
                logger.info("ℹ️ 未检测到ROS2环境，将使用本地播放")
                return False
        except Exception as e:
            logger.warning(f"⚠️ ROS2环境检查失败: {e}")
            return False

    def _init_ros2_tts_publisher(self):
        """初始化ROS2 TTS播放请求发布者"""
        try:
            if not self.ros2_tts_publisher and self.use_ros2_tts:
                import rclpy
                from std_msgs.msg import String

                # 如果没有ROS2节点，创建一个临时节点
                if not hasattr(self, '_ros2_node') or self._ros2_node is None:
                    rclpy.init()
                    self._ros2_node = rclpy.create_node('asr_tts_trigger')

                self.ros2_tts_publisher = self._ros2_node.create_publisher(
                    String, '/xlerobot/tts/trigger_play', 10
                )
                logger.info("✅ ROS2 TTS播放请求发布者初始化成功")
        except Exception as e:
            logger.warning(f"⚠️ ROS2 TTS播放请求发布者初始化失败: {e}")
            self.use_ros2_tts = False

    def initialize(self) -> bool:
        """初始化ASR系统"""
        try:
            logger.info("🚀 初始化Epic1 ASR系统...")

            # 设置麦克风 - 使用智能设备选择器
            try:
                # 导入音频设备管理器
                from .audio_device_manager import get_device_manager, setup_16khz_recording

                logger.info("🔧 使用智能设备选择器初始化麦克风...")
                device_manager = get_device_manager()

                # 获取最佳输入设备
                best_device = device_manager.get_best_input_device()
                if best_device:
                    logger.info(f"🎯 选择最佳输入设备: {best_device.name} (索引: {best_device.index})")

                    # 设置16kHz录音环境
                    setup_result = setup_16khz_recording(best_device.index)
                    if setup_result["success"]:
                        logger.info(f"✅ 16kHz录音环境设置成功: {setup_result['device_name']}")
                        logger.info(f"📊 设备信息: 索引={setup_result['device_index']}, 采样率=16kHz, 声道=单声道")

                        # 初始化录音器管理器
                        self.audio_recorder = get_recorder_manager()
                        logger.info("✅ 使用AudioRecorderManager初始化智能麦克风")
                    else:
                        logger.error(f"❌ 16kHz录音环境设置失败: {setup_result.get('error', '未知错误')}")
                        raise Exception("16kHz环境设置失败")
                else:
                    logger.error("❌ 未找到可用的输入设备")
                    raise Exception("无可用输入设备")

            except Exception as e:
                logger.error(f"❌ 智能设备选择失败: {e}")
                # 回退到ALSA兼容音频输入
                logger.warning("⚠️ 回退到ALSA兼容音频输入...")
                try:
                    # 初始化混合音频输入 (PyAudio + ALSA)
                    self.audio_input = create_hybrid_audio_input(
                        sample_rate=16000,
                        channels=1
                    )
                    logger.info("✅ 混合音频输入初始化成功")
                except Exception as e2:
                    logger.error(f"❌ 混合音频输入初始化失败: {e2}")
                    self.audio_input = None

                # 同时保留AudioRecorderManager作为备选
                try:
                    self.audio_recorder = get_recorder_manager()
                    logger.info("✅ AudioRecorderManager备选初始化成功")
                except Exception as e2:
                    logger.error(f"❌ ALSA录音器初始化失败: {e2}")
                    self.audio_recorder = None
  
            # 初始化ASR服务
            app_key = os.environ.get("ALIYUN_NLS_APPKEY", "")
            token = ""

            # 优先使用官方SDK获取Token
            if OFFICIAL_SDK_AVAILABLE:
                try:
                    access_key_id = os.environ.get("ALIBABA_CLOUD_ACCESS_KEY_ID", "")
                    access_key_secret = os.environ.get("ALIBABA_CLOUD_ACCESS_KEY_SECRET", "")

                    if access_key_id and access_key_secret:
                        token = getToken(access_key_id, access_key_secret)
                        if token:
                            logger.info("✅ Token获取成功 (使用官方SDK)")
                        else:
                            logger.error("❌ 官方SDK获取Token失败")
                    else:
                        logger.error("❌ 阿里云访问密钥未设置")
                except Exception as e:
                    logger.error(f"❌ 官方SDK获取Token异常: {e}")

            # 备用：Token管理器
            elif 'TOKEN_MANAGER_AVAILABLE' in locals() and TOKEN_MANAGER_AVAILABLE:
                try:
                    token_manager = get_token_manager()
                    token = token_manager.get_token()
                    if token:
                        logger.info("✅ Token获取成功 (使用Token管理器)")
                    else:
                        logger.warning("⚠️ Token管理器获取失败")
                except Exception as e:
                    logger.warning(f"⚠️ Token管理器异常: {e}")

            # 使用WebSocket ASR服务（根据架构文档要求）
            try:
                self.asr_service = AliyunASRWebSocketService()
                logger.info("✅ WebSocket ASR服务初始化成功")
            except Exception as e:
                logger.error(f"❌ WebSocket ASR服务初始化失败: {e}")
                self.asr_service = None

            # 初始化唤醒词检测器
            self.wake_word_detector = WakeWordDetector()
            logger.info("✅ 唤醒词检测器初始化成功")

            # 初始化TTS服务
            try:
                # 动态导入TTS WebSocket引擎，严格遵循架构文档
                from modules.tts.engine.aliyun_tts_websocket_engine import AliyunTTSEngine

                tts_config = {
                    'access_key_id': os.environ.get("ALIBABA_CLOUD_ACCESS_KEY_ID", ""),
                    'access_key_secret': os.environ.get("ALIBABA_CLOUD_ACCESS_KEY_SECRET", ""),
                    'app_key': app_key,
                    'region': 'cn-shanghai',
                    'voice': 'jiajia',  # 佳佳-粤语女声
                    'format': 'wav',
                    'sample_rate': 16000,
                    'volume': 100
                }
                self.tts_client = AliyunTTSEngine(config=tts_config)
                logger.info("✅ TTS WebSocket服务初始化成功")
            except Exception as e:
                logger.warning(f"⚠️ TTS服务初始化失败: {e}")
                self.tts_client = None

            # 初始化多模态LLM (延迟导入以避免ROS2依赖)
            try:
                # 动态导入多模态LLM，避免通过LLM模块的__init__.py
                import sys
                from pathlib import Path
                llm_path = Path(__file__).parent.parent.parent / "modules" / "llm" / "qwen_multimodal_llm.py"
                if llm_path.exists():
                    sys.path.insert(0, str(llm_path.parent))
                    from qwen_multimodal_llm import QwenMultimodalLLM
                    self.llm_client = QwenMultimodalLLM()

                    if asyncio.iscoroutinefunction(self.llm_client.initialize):
                        # 异步初始化
                        loop = asyncio.get_event_loop()
                        if loop.is_running():
                            # 如果已在事件循环中，创建任务
                            asyncio.create_task(self._async_init_llm())
                        else:
                            # 如果没有事件循环，运行直到完成
                            loop.run_until_complete(self._async_init_llm())
                    else:
                        # 同步初始化
                        if self.llm_client.initialize():
                            logger.info("✅ 多模态LLM初始化成功")
                        else:
                            logger.warning("⚠️ 多模态LLM初始化失败，使用基础回复模式")
                            self.llm_client = None
                else:
                    logger.warning(f"⚠️ 多模态LLM文件不存在: {llm_path}")
                    self.llm_client = None
            except Exception as e:
                logger.warning(f"⚠️ 多模态LLM初始化异常: {e}")
                self.llm_client = None

        except Exception as e:
            logger.error(f"❌ ASR系统初始化失败: {e}")
            self.initialized = False
            return False

        logger.info("🎉 ASR系统初始化完成！")
        self.initialized = True
        return True

    async def _async_init_llm(self):
        """异步初始化LLM"""
        try:
            if await self.llm_client.initialize():
                logger.info("✅ 多模态LLM初始化成功")
            else:
                logger.warning("⚠️ 多模态LLM初始化失败，使用基础回复模式")
                self.llm_client = None
        except Exception as e:
            logger.warning(f"⚠️ 多模态LLM初始化异常: {e}")
            self.llm_client = None

    def start(self) -> bool:
        """启动ASR系统"""
        if self.is_running:
            logger.warning("⚠️ ASR系统已在运行")
            return True

        try:
            logger.info("🎤 启动语音交互服务...")
            self.is_running = True
            self._start_time = time.time()

            # 初始化线程和事件循环
            self._stop_event = threading.Event()
            self._listening_thread = None

            # 系统启动后进入静默监听模式，等待唤醒词
            logger.info("🎯 系统已启动，进入静默监听模式，等待唤醒词 '傻强'...")

            # 在独立线程中启动监听循环
            self._start_listening_thread()

            logger.info("✅ ASR系统启动成功，开始监听...")
            return True

        except Exception as e:
            logger.error(f"❌ ASR系统启动失败: {e}")
            self.is_running = False
            return False

    def _start_listening_thread(self):
        """启动监听线程"""
        def run_event_loop():
            """在独立线程中运行事件循环和监听循环"""
            try:
                # 创建新的事件循环
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)

                logger.info("🔄 监听线程事件循环已启动")

                # 运行监听循环
                loop.run_until_complete(self._listening_loop())

            except Exception as e:
                logger.error(f"❌ 监听线程错误: {e}")
            finally:
                logger.info("⏹️ 监听线程已停止")

        # 启动守护线程
        self._listening_thread = threading.Thread(
            target=run_event_loop,
            name="ASRListeningThread",
            daemon=True
        )
        self._listening_thread.start()
        logger.info("🧵 监听线程已启动")

    def start_listening(self) -> bool:
        """启动监听（兼容启动脚本调用）"""
        return self.start()

    def stop_listening(self):
        """停止监听"""
        self.stop()

    def cleanup(self):
        """清理资源"""
        self.stop()

    def get_status(self) -> Dict[str, Any]:
        """获取系统状态"""
        import time
        uptime_seconds = 0
        if hasattr(self, '_start_time'):
            uptime_seconds = int(time.time() - self._start_time)

        return {
            "state": "running" if self.is_running else "stopped",
            "asr_state": self.state.value,  # 添加ASR状态
            "uptime_seconds": uptime_seconds,
            "audio_recorder_available": self.audio_recorder is not None,
          "recorder_state": self.audio_recorder.get_state().value if self.audio_recorder else "unknown",
            "stats": {
                "total_listens": getattr(self, '_total_listens', 0),
                "wake_detections": getattr(self, '_wake_detections', 0),
                "successful_recognitions": getattr(self, '_successful_recognitions', 0)
            }
        }

    async def _listening_loop(self):
        """语音监听循环"""
        logger.info("🎯 开始监听唤醒词...")
        logger.info(f"🤖 初始状态: {self.state.value}")

        # 初始化统计计数器
        self._total_listens = 0
        self._wake_detections = 0
        self._successful_recognitions = 0

        # 启动唤醒词检测器
        if self.wake_word_detector:
            try:
                self.wake_word_detector.start_listening()
                logger.info("✅ 唤醒词检测器已启动")
            except Exception as e:
                logger.warning(f"⚠️ 唤醒词检测器启动异常: {e}")

        try:
            while self.is_running:
                # 检查停止事件
                if hasattr(self, '_stop_event') and self._stop_event.is_set():
                    logger.info("⏹️ 收到停止信号，退出监听循环")
                    break

                self._total_listens += 1

                # 每隔一段时间打印监听状态
                if self._total_listens % 20 == 1:  # 每20次监听打印一次
                    logger.info(f"🎯 监听进行中... (第{self._total_listens}次)")

                # 使用asyncio.wait_for添加超时，以便定期检查停止事件
                try:
                    # 持续监听音频（带超时）
                    audio_data = await asyncio.wait_for(
                        self._listen_for_audio(),
                        timeout=6.0  # 6秒超时，确保录音时长2秒有足够时间完成
                    )
                except asyncio.TimeoutError:
                    # 超时了，继续下一轮循环以检查停止事件
                    continue

                if audio_data is not None and audio_data.size > 0:
                    logger.debug("🎤 检测到音频输入，检查唤醒词...")

                    # 检查是否包含唤醒词 - 添加异常保护
                    try:
                        # 🔧 修复：只有真正检测到唤醒词才执行后续操作
                        wake_word_detected = self._check_wake_word(audio_data)
                        if wake_word_detected:
                            self._wake_detections += 1
                            logger.info("🔔 检测到唤醒词：傻强")

                            # 状态转换: IDLE -> WAKE_DETECTED
                            self.state = ASRState.WAKE_DETECTED

                            # 播放欢迎语（而非确认音）
                            logger.info("🔊 播放欢迎语...")
                            self.play_response("傻强系度,老细有乜可以帮到你!")

                            # 等待播放完成（优化：减少等待时间）
                            await asyncio.sleep(1)

                            # 状态转换: WAKE_DETECTED -> LISTENING_COMMAND
                            self.state = ASRState.LISTENING_COMMAND

                            # 重新监听用户指令（新音频！）
                            command_audio = await self._listen_for_command(timeout=3.0)
                            if command_audio:
                                # 状态转换: LISTENING_COMMAND -> PROCESSING
                                self.state = ASRState.PROCESSING

                                # 识别用户指令
                                text = await self._recognize_speech_from_audio(command_audio)
                                if text:
                                    self._successful_recognitions += 1
                                    logger.info(f"📝 识别结果: {text}")

                                    # 新增：触发回调 - 创建结果对象并传递给回调函数
                                    if self.result_callback:
                                        try:
                                            # 创建简单的结果对象
                                            class ASRResult:
                                                def __init__(self, text, success=True, confidence=1.0, error=None):
                                                    self.text = text
                                                    self.success = success
                                                    self.confidence = confidence
                                                    self.error = error

                                            result_obj = ASRResult(text=text, success=True, confidence=0.9)
                                            self.result_callback(result_obj)  # 触发回调
                                        except Exception as e:
                                            logger.error(f"❌ 结果回调失败: {e}")

                                # 处理回复
                                response = await self._process_command(text)
                                if response:
                                    logger.info(f"🔊 回复: {response}")

                                    # 状态转换: PROCESSING -> RESPONDING
                                    self.state = ASRState.RESPONDING

                                    self.play_response(response)
                            else:
                                logger.warning("⚠️ 语音识别失败")

                                # 新增：触发失败回调
                                if self.result_callback:
                                    try:
                                        class ASRResult:
                                            def __init__(self, text, success=True, confidence=1.0, error=None):
                                                self.text = text
                                                self.success = success
                                                self.confidence = confidence
                                                self.error = error

                                        result_obj = ASRResult(text="", success=False, confidence=0.0, error="语音识别失败")
                                        self.result_callback(result_obj)  # 触发回调
                                    except Exception as e:
                                        logger.error(f"❌ 结果回调失败: {e}")
                        else:
                            logger.warning("⚠️ 未检测到用户指令，超时返回监听模式")

                            # 状态转换: 返回 IDLE
                            self.state = ASRState.IDLE
                            logger.info("🔄 返回空闲监听模式")

                    except Exception as e:
                        logger.error(f"❌ 唤醒词检测异常: {e}")
                        # 确保状态重置
                        self.state = ASRState.IDLE

                # 短暂休息，避免CPU占用过高
                await asyncio.sleep(0.1)

        except Exception as e:
            logger.error(f"❌ 监听循环异常: {e}")
        finally:
            # 停止唤醒词检测器
            if self.wake_word_detector:
                try:
                    self.wake_word_detector.stop_listening()
                except Exception as e:
                    logger.warning(f"⚠️ 唤醒词检测器停止异常: {e}")
            logger.info("🏁 监听循环已结束")

    async def _listen_for_audio(self) -> Optional[np.ndarray]:
        """监听音频输入 - 使用线程安全录音器"""
        if not self.audio_recorder:
            logger.warning("⚠️ 录音器未初始化，模拟监听...")
            # 模拟监听 - 返回None表示没有音频
            await asyncio.sleep(0.1)
            return None

        try:
            # 等待录音器就绪，最多等待1秒（简化重试机制）
            for retry in range(10):  # 最多等待1秒
                recorder_state = self.audio_recorder.get_state()
                if recorder_state.name == 'IDLE':
                    break
                elif retry >= 9:  # 最后一次尝试失败，强制重置
                    logger.warning(f"录音器忙，强制重置状态: {recorder_state.value}")
                    try:
                        # 强制停止并重置录音器
                        self.audio_recorder.stop_recording()
                        await asyncio.sleep(0.1)
                    except Exception as e:
                        logger.debug(f"录音器重置异常: {e}")
                    return None
                else:
                    await asyncio.sleep(0.1)

            # 启动录音
            logger.info("🎤 开始音频监听...")
            success = self.audio_recorder.start_recording(duration=2.0)

            if not success:
                logger.warning("⚠️ 录音启动失败")
                return None
            else:
                logger.info("✅ 录音启动成功")

            # 等待录音完成
            try:
                await asyncio.sleep(2.5)  # 等待录音完成（duration + buffer）
                logger.debug("✅ 录音时间结束")

            except asyncio.CancelledError:
                logger.warning("⚠️ 录音被取消")
                return None

            # 获取音频数据
            logger.info("🛑 停止录音并获取数据...")
            success, audio_data = self.audio_recorder.stop_recording()

            if success and audio_data is not None and audio_data.size > 0:
                logger.info(f"🎤 成功捕获音频片段: {len(audio_data)} samples")
                return audio_data
            else:
                logger.warning(f"⏰ 录音完成但无有效音频数据 (success={success}, data={audio_data is not None})")
                return None

        except OSError as e:
            if "busy" in str(e).lower() or "16" in str(e) or "device" in str(e).lower():
                logger.error(f"❌ 音频设备被占用！错误: {e}")
                logger.error("   🔧 建议解决方案:")
                logger.error("   1. 停止其他音频程序: pkill pulseaudio")
                logger.error("   2. 检查设备权限: sudo usermod -a -G audio $USER")
                logger.error("   3. 重启音频系统: sudo systemctl restart alsa-state")
            else:
                logger.error(f"❌ 音频设备错误: {type(e).__name__}: {e}")
            return None
        except Exception as e:
            logger.error(f"❌ 音频监听失败: {type(e).__name__}: {e}")
            logger.error("   📊 系统状态检查建议:")
            logger.error("   1. 确认麦克风已连接并工作: arecord -l")
            logger.error("   2. 检查音频设备状态: cat /proc/asound/cards")
            logger.error("   3. 测试录音功能: arecord -d 3 test.wav")
            # 如果监听失败，返回None让循环继续
            return None

    async def _listen_for_command(self, timeout: float = 5.0) -> Optional[np.ndarray]:
        """
        监听用户命令（唤醒后）- 使用ThreadSafeAudioRecorder

        Args:
            timeout: 超时时间（秒）

        Returns:
            音频数据或None
        """
        if not self.audio_recorder:
            logger.error("❌ ThreadSafeAudioRecorder未初始化")
            return None

        try:
            logger.info(f"🎤 等待用户指令（超时{timeout}秒）...")

            # 使用ThreadSafeAudioRecorder录音，时长为超时时间
            duration = min(timeout, 10.0)  # 最多10秒
            success = self.audio_recorder.start_recording(duration=duration)

            if not success:
                current_state = self.audio_recorder.get_state()
                logger.warning(f"⚠️ 命令录音启动失败，当前状态: {current_state.value}")
                return None

            # 等待录音完成
            try:
                await asyncio.sleep(duration + 0.5)  # 等待录音完成（duration + buffer）
                logger.debug("✅ 命令录音时间结束")

            except asyncio.CancelledError:
                logger.warning("⚠️ 命令录音被取消")
                return None

            # 停止录音并获取音频数据
            success, audio_data = self.audio_recorder.stop_recording()

            if success and audio_data is not None and audio_data.size > 0:
                # ✅ 检查音频能量（防止静音幻觉）
                audio_energy = np.sqrt(np.mean(audio_data.astype(float) ** 2))
                ENERGY_THRESHOLD = 600  # 🔧 P3-5：统一阈值600（正常语音300-800，静音<100）

                if audio_energy < ENERGY_THRESHOLD:
                    logger.warning(f"⚠️ 用户命令音频能量过低 ({audio_energy:.1f} < {ENERGY_THRESHOLD})，返回None")
                    return None

                logger.info(f"✅ 捕获到用户指令音频，能量合格 ({audio_energy:.1f})")
                return audio_data
            else:
                logger.info("⏰ 录音完成，但无有效音频数据")
                return None

        except Exception as e:
            logger.error(f"❌ 监听用户指令失败: {type(e).__name__}: {e}")
            return None

    def _check_wake_word(self, audio_data: np.ndarray) -> bool:
        """检查音频中是否包含唤醒词 - 优先使用阿里云ASR"""
        try:
            # 🔧 回声消除P1-2：TTS播放期间禁用唤醒词检测
            if self.is_playing_tts:
                logger.debug("🔇 TTS播放中，跳过唤醒词检测（防止回声循环）")
                return False

            # 🔧 修复：增强冷却时间检查，包含动态冷却机制
            current_time = time.time()

            # 🔧 循环检测P2-4：记录唤醒时间，检测快速重复唤醒
            self.recent_wake_times.append(current_time)
            if len(self.recent_wake_times) > 3:
                self.recent_wake_times.pop(0)  # 只保留最近3次

            # 检测2秒内连续唤醒（可能的循环）
            if len(self.recent_wake_times) >= 2:
                time_gap = current_time - self.recent_wake_times[-2]
                if time_gap < 2.0:
                    logger.warning(f"⚠️ 循环警告：检测到快速重复唤醒（间隔{time_gap:.1f}秒），增加冷却时间")
                    # 强制增加冷却时间
                    self.wake_cooldown = max(self.wake_cooldown, 6)

            # 动态冷却：如果刚刚播放了TTS，增加冷却时间
            dynamic_cooldown = self.wake_cooldown
            if self.state == ASRState.RESPONDING:
                dynamic_cooldown = 6  # TTS播放后6秒冷却
            elif current_time - self.last_response_time < 5.0:
                dynamic_cooldown = 4  # 最近5秒内有响应时4秒冷却

            if current_time - self.last_wake_time < dynamic_cooldown:
                logger.debug(f"🔒 冷却中 ({current_time - self.last_wake_time:.1f}s < {dynamic_cooldown}s)，跳过唤醒词检测")
                return False

            # ✅ 音频有效性检查 - 防止发送静音/噪音给ASR
            if audio_data is None or audio_data.size < 8000:  # 至少0.5秒@16kHz
                logger.debug(f"音频长度不足 ({len(audio_data) if audio_data is not None else 0} < 8000)，跳过唤醒词检测")
                return False

            # 计算RMS能量
            audio_energy = np.sqrt(np.mean(audio_data.astype(float) ** 2))
            ENERGY_THRESHOLD = 600  # 🔧 P3-5：统一阈值600（正常语音300-800，静音<100）

            if audio_energy < ENERGY_THRESHOLD:
                logger.debug(f"音频能量过低 ({audio_energy:.1f} < {ENERGY_THRESHOLD})，跳过ASR")
                return False

            # 静音检测：检查音频是否包含有效语音信号
            # 计算零交叉率 - 静音的零交叉率很低
            zero_crossings = np.count_nonzero(np.diff(np.sign(audio_data - np.mean(audio_data))))
            zcr_rate = zero_crossings / len(audio_data) * 16000  # 转换为每秒零交叉次数

            # 静音检测：如果能量勉强够但零交率太低，可能是背景噪声
            if zcr_rate < 300:  # 🔧 修复：更严格的静音检测零交叉率阈值，从500收紧到300
                logger.warning(f"⚠️ 检测到疑似静音或噪声 (能量:{audio_energy:.1f}, 零交叉率:{zcr_rate:.0f})，跳过ASR避免幻觉")
                return False

            logger.info(f"✅ 音频检测通过 (能量:{audio_energy:.1f}, 零交叉率:{zcr_rate:.0f})，发送ASR识别")

            wake_word_detected = False

            # 方法1: 优先使用WebSocket ASR（根据架构文档要求）
            if self.asr_service:
                try:
                    # 统一处理不同类型的音频数据
                    if hasattr(audio_data, 'get_wav_data'):
                        # PyAudio AudioData对象
                        wav_data = audio_data.get_wav_data()
                    elif isinstance(audio_data, np.ndarray):
                        # numpy数组转换为WAV格式
                        import io
                        import wave
                        wav_buffer = io.BytesIO()
                        with wave.open(wav_buffer, 'wb') as wf:
                            wf.setnchannels(1)      # 单声道
                            wf.setsampwidth(2)      # 16-bit
                            wf.setframerate(16000)  # 16kHz
                            # 确保数据格式正确
                            if audio_data.dtype != np.int16:
                                audio_data_normalized = (audio_data * 32768).astype(np.int16)
                            else:
                                audio_data_normalized = audio_data
                            wf.writeframes(audio_data_normalized.tobytes())
                        wav_data = wav_buffer.getvalue()
                    else:
                        raise ValueError(f"不支持的音频数据类型: {type(audio_data)}")

                    temp_file = tempfile.NamedTemporaryFile(suffix='.wav', delete=False)
                    temp_file.write(wav_data)
                    temp_file.close()

                    # 使用WebSocket ASR服务（同步调用）
                    result = self.asr_service.recognize_file(temp_file.name)

                    # 清理临时文件
                    os.unlink(temp_file.name)

                    if result is not None and isinstance(result, str):
                        text = result.strip().lower()
                        logger.info(f"🔍 WebSocket ASR识别文本: {text}")

                        # 粤语唤醒词检测（扩展白名单，包含ASR误识别变体）
                        wake_words = [
                            # 原有词汇
                            "傻强", "傻强啊", "傻强呀", "傻強", "傻強啊", "傻強呀",
                            # ASR误识别变体（阿里云粤语ASR常见误识别）
                            "收聲", "收声", "沙强", "沙強", "小强", "小強",
                            # 其他可能变体
                            "傻彊", "傻強啊", "傻彊呀"
                        ]

                        for wake_word in wake_words:
                            if wake_word in text:
                                logger.info(f"✅ WebSocket检测到唤醒词: {wake_word} (原文: {text})")
                                wake_word_detected = True
                                self.last_wake_time = current_time
                                return True

                except Exception as e:
                    logger.error(f"❌ 阿里云ASR识别异常: {e}", exc_info=True)

            # 如果阿里云ASR没有检测到唤醒词，直接返回False
            logger.debug("❌ 阿里云ASR未检测到唤醒词")
            return False

        except Exception as e:
            logger.error(f"❌ 唤醒词检测异常: {e}")
            return False

    async def _play_wake_confirmation(self):
        """播放唤醒确认音（已废弃，现在直接播放欢迎语）"""
        # 此方法已废弃，现在在检测到唤醒词后直接播放完整的欢迎语
        logger.debug("唤醒确认音功能已整合到欢迎语播放中")
        pass

    async def _recognize_speech_from_audio(self, audio_data: np.ndarray) -> Optional[str]:
        """从音频数据进行语音识别"""
        if not self.asr_service:
            logger.error("❌ ASR服务未初始化")
            return None

        try:
            # 将AudioData转换为WAV格式的字节数据
            wav_data = audio_data.get_wav_data()

            # 调用阿里云ASR服务进行识别
            result = self._call_aliyun_asr(wav_data)

            if result and result.strip():
                logger.info(f"🎯 阿里云ASR识别结果: {result}")
                return result
            else:
                # Fallback到本地识别
                return self._fallback_local_recognition(audio_data)

        except Exception as e:
            logger.error(f"❌ 阿里云ASR识别失败: {e}")
            # 尝试本地fallback
            return self._fallback_local_recognition(audio_data)

    def _call_aliyun_asr(self, wav_data: bytes) -> Optional[str]:
        """调用阿里云ASR服务 - WebSocket版本"""
        try:
            if not self.asr_service:
                logger.error("❌ ASR服务未初始化")
                return None

            logger.info("🔍 调用阿里云ASR WebSocket服务进行识别...")

            # 保存音频数据到临时文件（WebSocket API需要文件路径）
            import tempfile
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                temp_file.write(wav_data)
                temp_file_path = temp_file.name

            try:
                # 调用WebSocket ASR服务
                result = self.asr_service.recognize_file(
                    audio_file_path=temp_file_path,
                    language="cn-cantonese",
                    format="wav"
                )

                # WebSocket API返回直接文本结果
                if result and result.strip():
                    logger.info(f"✅ 阿里云ASR WebSocket识别成功: {result}")
                    return result
                else:
                    logger.warning("⚠️ 阿里云ASR WebSocket识别返回空结果")
                    return None

            finally:
                # 清理临时文件
                import os
                try:
                    os.unlink(temp_file_path)
                except:
                    pass

        except Exception as e:
            logger.error(f"❌ 阿里云ASR WebSocket调用失败: {e}")
            return None

    def _fallback_local_recognition(self, audio_data: np.ndarray) -> Optional[str]:
        """本地语音识别作为fallback - 已禁用，使用ThreadSafeAudioRecorder"""
        # TODO: 实现基于ThreadSafeAudioRecorder的本地识别
        logger.warning("⚠️ 本地识别功能暂时禁用，使用主要ASR服务")
        return None

    async def _recognize_speech(self) -> Optional[str]:
        """语音识别（兼容旧接口）"""
        try:
            # 监听音频并进行识别
            audio_data = await self._listen_for_audio()
            if audio_data is not None and audio_data.size > 0:
                return await self._recognize_speech_from_audio(audio_data)
            return None
        except Exception as e:
            logger.error(f"语音识别失败: {e}")
            return None

    async def _process_command(self, text: str) -> Optional[str]:
        """处理语音命令 - 使用多模态LLM"""
        try:
            if not self.llm_client:
                # 如果LLM不可用，使用基础回复
                return await self._basic_command_response(text)

            # 添加对话历史
            self.conversation_history.append({
                "role": "user",
                "content": text
            })

            # 限制历史长度
            if len(self.conversation_history) > self.max_history_length:
                self.conversation_history = self.conversation_history[-self.max_history_length:]

            # 使用多模态LLM处理
            response = await self.llm_client.process_voice_command(
                text=text,
                previous_context=self.conversation_history[:-1]  # 排除当前用户消息
            )

            # 添加助手回复到历史
            if response:
                self.conversation_history.append({
                    "role": "assistant",
                    "content": response
                })

                logger.info(f"🤖 LLM处理: {text[:30]}... -> {response[:50]}...")
                return response
            else:
                return "抱歉，我现在无法处理这个问题，请稍后再试。"

        except Exception as e:
            logger.error(f"❌ LLM命令处理失败: {e}")
            return "系统遇到问题，请稍后再试。"

    async def _basic_command_response(self, text: str) -> Optional[str]:
        """基础命令回复（LLM不可用时的fallback）"""
        try:
            text_lower = text.lower()

            if "天气" in text_lower:
                if self.llm_client:
                    return await self.llm_client.get_weather_response()
                else:
                    return "今日天气晴朗，温度适宜，适合出行"
            elif "时间" in text_lower or "几时" in text_lower:
                if self.llm_client:
                    return await self.llm_client.get_time_response()
                else:
                    return await self._get_current_time()
            elif "你好" in text_lower or "哈喽" in text_lower or "hello" in text_lower:
                return "你好！我是傻强，有什么可以帮到你？"
            elif "拜拜" in text_lower or "再见" in text_lower:
                return "拜拜！有需要随时叫我傻强！"
            elif "感谢" in text_lower or "多谢" in text_lower:
                return "不客气！这是傻强应该做的。"
            else:
                return "抱歉，我没有理解您的指令，请重新说一次。"

        except Exception as e:
            logger.error(f"❌ 基础命令回复失败: {e}")
            return "抱歉，我现在无法处理这个问题。"

    async def _get_current_time(self) -> str:
        """获取当前时间"""
        try:
            import datetime
            now = datetime.datetime.now()
            return f"现在时间是{now.strftime('%H点%M分')}"
        except Exception as e:
            logger.error(f"❌ 获取时间失败: {e}")
            return "无法获取当前时间"

    def play_response(self, text: str):
        """播放语音回复 - 支持ROS2统一播放管理"""
        try:
            # 🔧 P1-1：设置TTS播放状态，禁用麦克风防止回声
            self.is_playing_tts = True
            logger.info(f"🔊 准备播放回复: {text} (麦克风已禁用)")

            # 优先使用ROS2播放请求（如果可用）
            if self.use_ros2_tts:
                if not self.ros2_tts_publisher:
                    self._init_ros2_tts_publisher()

                if self.ros2_tts_publisher:
                    try:
                        # 通过ROS2发送播放请求
                        msg = String()
                        msg.data = text
                        self.ros2_tts_publisher.publish(msg)
                        logger.info(f"✅ 已通过ROS2发送播放请求: {text}")

                        # 🔧 P1-1 & P2-3：播放完成后恢复麦克风并更新时间
                        time.sleep(0.5)  # 延迟0.5秒确保播放开始
                        self.is_playing_tts = False
                        self.last_response_time = time.time()
                        logger.debug("🎤 麦克风已恢复")
                        return True
                    except Exception as e:
                        logger.warning(f"⚠️ ROS2播放请求失败，回退到本地播放: {e}")

            # 回退到本地播放
            # 增强的TTS客户端检查
            if not self.tts_client:
                logger.warning("⚠️ TTS客户端未初始化，尝试重新初始化...")

                # 尝试重新初始化TTS
                if self._retry_init_tts():
                    logger.info("✅ TTS客户端重新初始化成功")
                else:
                    logger.error("❌ TTS客户端初始化失败，播放备用提示音")
                    self._play_fallback_sound()

                    # 🔧 恢复麦克风并更新时间
                    time.sleep(0.5)
                    self.is_playing_tts = False
                    self.last_response_time = time.time()
                    return False

            # 调用TTS服务合成语音
            try:
                audio_data = self.tts_client.synthesize(text)
                if audio_data is not None and (hasattr(audio_data, 'size') and audio_data.size > 0 or len(audio_data) > 0):
                    if hasattr(audio_data, 'size'):
                        logger.info(f"✅ 合成语音成功，长度: {audio_data.size}字节")
                    else:
                        logger.info(f"✅ 合成语音成功，长度: {len(audio_data)}字节")

                    # 播放音频（已有超时保护）
                    success = self._play_audio_data(audio_data)

                    # 🔧 P1-1 & P2-3：播放完成后恢复麦克风并更新时间
                    time.sleep(0.5)  # 延迟0.5秒
                    self.is_playing_tts = False
                    self.last_response_time = time.time()
                    logger.debug("🎤 麦克风已恢复")

                    if success:
                        logger.info("✅ 音频播放成功")
                        return True
                    else:
                        logger.warning("⚠️ 音频播放失败")
                        self._play_fallback_sound()
                        return False
                else:
                    logger.warning("⚠️ 语音合成失败，播放备用提示音")
                    self._play_fallback_sound()

                    # 🔧 恢复麦克风并更新时间
                    time.sleep(0.5)
                    self.is_playing_tts = False
                    self.last_response_time = time.time()
                    return False

            except Exception as tts_error:
                logger.error(f"❌ TTS合成过程出错: {tts_error}")
                self._play_fallback_sound()

                # 🔧 恢复麦克风并更新时间
                time.sleep(0.5)
                self.is_playing_tts = False
                self.last_response_time = time.time()
                return False

        except Exception as e:
            logger.error(f"❌ 语音播放失败: {e}")
            self._play_fallback_sound()

            # 🔧 恢复麦克风并更新时间
            time.sleep(0.5)
            self.is_playing_tts = False
            self.last_response_time = time.time()
            return False

    def _retry_init_tts(self) -> bool:
        """重新初始化TTS客户端"""
        try:
            # 查找TTS配置 - 使用WebSocket引擎
            from modules.tts.engine.aliyun_tts_websocket_engine import AliyunTTSEngine

            tts_config = {
                "voice": "jiajia",  # 佳佳-粤语女声
                "speech_rate": 0,  # 正常语速
                "volume": 100,  # 音量
                "format": "wav",  # 音频格式
                "sample_rate": 16000  # 采样率
            }

            # 尝试重新初始化
            self.tts_client = AliyunTTSEngine(config=tts_config)

            # 测试TTS是否正常工作
            test_audio = self.tts_client.synthesize("测试")
            return test_audio is not None

        except Exception as e:
            logger.error(f"❌ TTS重新初始化失败: {e}")
            self.tts_client = None
            return False

    def _play_fallback_sound(self):
        """播放备用提示音 - 静默模式避免干扰"""
        # 静默模式：不播放提示音，仅记录日志
        logger.warning("📢 TTS不可用，跳过语音播放（静默模式）")
        return False

        # 以下代码保留用于紧急情况，但默认不执行
        try:
            # 尝试播放系统提示音或预定义音频文件
            fallback_paths = [
                "src/modules/asr/audio/beep.wav",
                "src/modules/asr/audio/alert.wav",
                "/usr/share/sounds/alsa/Front_Center.wav"  # Linux系统音
            ]

            for sound_path in fallback_paths:
                if os.path.exists(sound_path):
                    logger.info(f"🔊 播放备用提示音: {sound_path}")
                    try:
                        import subprocess
                        subprocess.run(["aplay", sound_path], check=True, capture_output=True)
                        return True
                    except Exception as e:
                        logger.debug(f"播放备用提示音失败 {sound_path}: {e}")
                        continue

            # 如果没有音频文件，输出到控制台
            logger.warning("📢 无可用音频文件，仅输出到控制台: 提示音")
            print("🔔 *提示音*")

        except Exception as e:
            logger.error(f"❌ 播放备用提示音失败: {e}")
            # 最后的备用方案
            print("🔔 *系统提示音*")

    def _play_audio_data(self, audio_data: bytes) -> bool:
        """播放音频数据"""
        try:
            import pygame
            import tempfile

            # 初始化pygame音频
            pygame.mixer.init()

            # 将音频数据保存到临时文件
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                temp_file.write(audio_data)
                temp_file_path = temp_file.name

            try:
                # 播放音频文件
                pygame.mixer.music.load(temp_file_path)
                pygame.mixer.music.play()

                # 添加超时保护 - 最多等待30秒
                timeout_seconds = 30
                start_time = time.time()

                # 等待播放完成（带超时保护）
                while pygame.mixer.music.get_busy():
                    if time.time() - start_time > timeout_seconds:
                        logger.warning("⚠️ 音频播放超时，强制停止")
                        pygame.mixer.music.stop()
                        break
                    pygame.time.Clock().tick(10)

                return True

            finally:
                # 清理临时文件
                try:
                    os.unlink(temp_file_path)
                except:
                    pass
                pygame.mixer.quit()

        except ImportError:
            # 如果没有pygame，尝试使用aplay (Linux)
            try:
                import tempfile
                with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                    temp_file.write(audio_data)
                    temp_file_path = temp_file.name

                import subprocess
                result = subprocess.run(['aplay', temp_file_path],
                                      capture_output=True, text=True)

                os.unlink(temp_file_path)
                return result.returncode == 0

            except Exception as e:
                logger.error(f"❌ 使用aplay播放失败: {e}")
                return False

        except Exception as e:
            logger.error(f"❌ 音频播放失败: {e}")
            return False

    def stop(self):
        """停止ASR系统"""
        logger.info("🛑 停止ASR系统...")

        # 设置停止标志
        self.is_running = False

        # 发送停止信号给监听线程
        if hasattr(self, '_stop_event'):
            self._stop_event.set()

        # 等待监听线程结束（最多等待5秒）
        if hasattr(self, '_listening_thread') and self._listening_thread:
            logger.info("⏳ 等待监听线程停止...")
            self._listening_thread.join(timeout=5.0)
            if self._listening_thread.is_alive():
                logger.warning("⚠️ 监听线程未能在5秒内停止")
            else:
                logger.info("✅ 监听线程已停止")

        logger.info("✅ ASR系统已停止")

    def _get_access_token(self) -> str:
        """获取阿里云访问令牌"""
        try:
            import requests
            from datetime import datetime

            access_key_id = os.environ.get("ALIBABA_CLOUD_ACCESS_KEY_ID", "")
            access_key_secret = os.environ.get("ALIBABA_CLOUD_ACCESS_KEY_SECRET", "")

            if not access_key_id or not access_key_secret:
                logger.warning("⚠️ 阿里云访问密钥未设置")
                return ""

            url = "https://nls-meta.cn-shanghai.aliyuncs.com/pop/2018-05-18/tokens"
            headers = {
                "Content-Type": "application/json",
                "Date": datetime.utcnow().strftime('%a, %d %b %Y %H:%M:%S GMT'),
                "Host": "nls-meta.cn-shanghai.aliyuncs.com"
            }

            data = {
                "AccessKeyId": access_key_id,
                "Action": "CreateToken"
            }

            response = requests.post(url, headers=headers, json=data, timeout=10)

            if response.status_code == 200:
                result = response.json()
                if "Token" in result and "Id" in result["Token"]:
                    token = result["Token"]["Id"]
                    logger.info(f"✅ 获取访问令牌成功: {token[:20]}...")
                    return token
                else:
                    logger.error(f"❌ Token响应格式错误: {result}")
                    return ""
            else:
                logger.error(f"❌ 获取Token失败，状态码: {response.status_code}")
                logger.error(f"响应内容: {response.text}")
                return ""

        except Exception as e:
            logger.error(f"❌ 获取访问令牌异常: {e}")
            return ""

def main():
    """主函数用于测试"""
    logging.basicConfig(level=logging.INFO)

    asr_system = ASRSystem()

    if asr_system.initialize():
        print("✅ ASR系统初始化成功")
    else:
        print("❌ ASR系统初始化失败")
        return False

    # 模拟启动（测试用）
    print("ASR系统准备就绪")
    return True

if __name__ == "__main__":
    main()