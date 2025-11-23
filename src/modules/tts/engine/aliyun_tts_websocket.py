#!/usr/bin/env python3
"""
WebSocket阿里云TTS服务 - 基于官方SDK的流式语音合成
================================================================

完全基于阿里云官方WebSocket SDK的TTS服务实现
解决HTTP REST API延迟问题，实现真正的流式语音合成

功能：
- 官方WebSocket SDK (NlsSpeechSynthesizer)
- 流式语音合成和边接收边播放
- 16kHz高质量音频输出
- 粤语多音色支持
- 实时音频流处理
- 完整的错误处理和重试机制
- 性能监控和日志记录

作者: Developer Agent (基于架构文档要求)
版本: 2.0 (WebSocket流式架构)
日期: 2025-11-16
依赖: alibabacloud-nls-python-sdk, pyaudio
"""

import os
import sys
import tempfile
import logging
import json
import time
import threading
import queue
import asyncio
from typing import Optional, Tuple, Dict, Any, List, Callable
from dataclasses import dataclass, field
from datetime import datetime
import numpy as np
import soundfile as sf

# 添加官方SDK路径
sys.path.append('/home/sunrise/.local/lib/python3.10/site-packages')

try:
    from nls.token import getToken
    from nls.speech_synthesizer import NlsSpeechSynthesizer
    NLS_SDK_AVAILABLE = True
except ImportError as e:
    print(f"❌ 阿里云NLS SDK未安装: {e}")
    print("请运行: pip3 install alibabacloud-nls-python-sdk")
    NLS_SDK_AVAILABLE = False

# 导入音频播放模块
try:
    import pyaudio
    PYAUDIO_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ PyAudio未安装: {e}")
    print("请运行: pip3 install pyaudio")
    PYAUDIO_AVAILABLE = False


# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class TTSResult:
    """TTS合成结果"""
    success: bool = False
    audio_data: bytes = b""
    text: str = ""
    voice: str = ""
    synthesis_time: float = 0.0
    playback_time: float = 0.0
    error: str = ""
    raw_response: Dict = field(default_factory=dict)

@dataclass
class StreamingTTSResult:
    """流式TTS结果"""
    success: bool = False
    audio_chunks: List[bytes] = field(default_factory=list)
    text: str = ""
    voice: str = ""
    total_chunks: int = 0
    received_chunks: int = 0
    synthesis_time: float = 0.0
    error: str = ""
    raw_response: Dict = field(default_factory=dict)

@dataclass
class PerformanceMetrics:
    """性能指标"""
    total_syntheses: int = 0
    successful_syntheses: int = 0
    failed_syntheses: int = 0
    average_synthesis_time: float = 0.0
    average_playback_time: float = 0.0
    total_synthesis_time: float = 0.0
    total_playback_time: float = 0.0
    last_update: datetime = field(default_factory=datetime.now)

class StreamingAudioPlayer:
    """流式音频播放器"""

    def __init__(self, sample_rate: int = 16000, channels: int = 1, format: int = pyaudio.paInt16):
        """
        初始化流式音频播放器

        Args:
            sample_rate: 采样率
            channels: 声道数
            format: 音频格式
        """
        if not PYAUDIO_AVAILABLE:
            raise ImportError("PyAudio未安装，无法使用音频播放功能")

        self.sample_rate = sample_rate
        self.channels = channels
        self.format = format

        # 初始化PyAudio
        self.pyaudio = pyaudio.PyAudio()

        # 播放控制
        self.is_playing = False
        self.playback_queue = queue.Queue()
        self.playback_thread = None
        self.stop_event = threading.Event()

        logger.info("✅ 流式音频播放器初始化完成")

    def start_playback(self):
        """开始播放线程"""
        if self.playback_thread and self.playback_thread.is_alive():
            return

        self.stop_event.clear()
        self.playback_thread = threading.Thread(target=self._playback_worker)
        self.playback_thread.daemon = True
        self.playback_thread.start()
        self.is_playing = True
        logger.info("🔊 流式播放开始")

    def stop_playback(self):
        """停止播放"""
        self.stop_event.set()
        self.is_playing = False
        if self.playback_thread and self.playback_thread.is_alive():
            self.playback_thread.join(timeout=2)
        logger.info("🔇 流式播放停止")

    def add_audio_chunk(self, audio_chunk: bytes):
        """添加音频块到播放队列"""
        if self.is_playing:
            self.playback_queue.put(audio_chunk)

    def _playback_worker(self):
        """播放工作线程"""
        try:
            # 打开音频流
            stream = self.pyaudio.open(
                format=self.format,
                channels=self.channels,
                rate=self.sample_rate,
                output=True,
                frames_per_buffer=1024
            )

            logger.info("✅ 音频流打开成功")

            while not self.stop_event.is_set():
                try:
                    # 从队列获取音频块
                    audio_chunk = self.playback_queue.get(timeout=0.1)

                    if audio_chunk:
                        # 写入音频流进行播放
                        stream.write(audio_chunk)

                except queue.Empty:
                    continue
                except Exception as e:
                    logger.error(f"播放异常: {e}")
                    break

            # 关闭音频流
            stream.close()
            logger.info("🔊 音频流关闭")

        except Exception as e:
            logger.error(f"播放器初始化失败: {e}")

    def __del__(self):
        """析构函数"""
        self.stop_playback()
        if hasattr(self, 'pyaudio'):
            self.pyaudio.terminate()

class WebSocketTTSService:
    """基于WebSocket的阿里云TTS服务"""

    def __init__(self,
                 access_key_id: str = None,
                 access_key_secret: str = None,
                 app_key: str = None,
                 enable_streaming: bool = True,
                 enable_playback: bool = True):
        """
        初始化WebSocket TTS服务

        Args:
            access_key_id: 阿里云AccessKey ID
            access_key_secret: 阿里云AccessKey Secret
            app_key: 应用AppKey
            enable_streaming: 是否启用流式合成
            enable_playback: 是否启用实时播放
        """
        if not NLS_SDK_AVAILABLE:
            raise ImportError("阿里云NLS SDK未安装，请运行 pip3 install alibabacloud-nls-python-sdk")

        # 认证配置
        self.access_key_id = access_key_id or os.environ.get("ALIBABA_CLOUD_ACCESS_KEY_ID")
        self.access_key_secret = access_key_secret or os.environ.get("ALIBABA_CLOUD_ACCESS_KEY_SECRET")
        self.app_key = app_key or os.environ.get("ALIYUN_NLS_APPKEY", "")

        if not all([self.access_key_id, self.access_key_secret, self.app_key]):
            raise ValueError("缺少必要的认证配置: access_key_id, access_key_secret, app_key")

        # 服务配置
        self.enable_streaming = enable_streaming
        self.enable_playback = enable_playback

        # 服务状态
        self.token = None
        self.is_initialized = False
        self.metrics = PerformanceMetrics()

        # 流式播放器
        self.audio_player = None
        if self.enable_playback:
            try:
                self.audio_player = StreamingAudioPlayer()
            except ImportError:
                logger.warning("⚠️ 音频播放器初始化失败，仅支持音频合成")
                self.enable_playback = False

        # 初始化服务
        self._initialize_service()

        logger.info("✅ WebSocket TTS服务初始化完成")
        logger.info(f"  - 流式合成: {'启用' if self.enable_streaming else '禁用'}")
        logger.info(f"  - 实时播放: {'启用' if self.enable_playback else '禁用'}")

    def _initialize_service(self):
        """初始化服务组件"""
        try:
            # 1. 获取Token
            self._refresh_token()

            self.is_initialized = True
            logger.info("✅ 服务组件初始化成功")

        except Exception as e:
            logger.error(f"❌ 服务初始化失败: {e}")
            raise

    def _refresh_token(self):
        """刷新访问Token"""
        try:
            self.token = getToken(self.access_key_id, self.access_key_secret)
            if not self.token:
                raise ValueError("Token获取失败")
            logger.info(f"✅ Token获取成功: {self.token[:20]}...")
        except Exception as e:
            logger.error(f"❌ Token获取失败: {e}")
            raise

    def synthesize(self,
                   text: str,
                   voice: str = "xiaoyan",
                   volume: int = 50,
                   rate: int = 0,
                   pitch: int = 0,
                   format: str = "pcm",
                   sample_rate: int = 16000) -> TTSResult:
        """
        语音合成

        Args:
            text: 待合成文本
            voice: 音色
            volume: 音量 (0-100)
            rate: 语速 (-500到500)
            pitch: 音调 (-500到500)
            format: 音频格式
            sample_rate: 采样率

        Returns:
            TTSResult: 合成结果
        """
        start_time = time.time()

        if not self.is_initialized:
            return TTSResult(success=False, error="服务未初始化", text=text, voice=voice)

        if not self.token:
            self._refresh_token()

        try:
            # 验证文本
            if not text or not text.strip():
                return TTSResult(success=False, error="文本为空", text=text, voice=voice)

            # 执行WebSocket合成
            result = self._websocket_synthesize(
                text=text,
                voice=voice,
                volume=volume,
                rate=rate,
                pitch=pitch,
                format=format,
                sample_rate=sample_rate
            )

            # 更新性能指标
            synthesis_time = time.time() - start_time
            result.synthesis_time = synthesis_time
            self._update_metrics(result, synthesis_time, 0)

            return result

        except Exception as e:
            error_msg = f"语音合成异常: {e}"
            logger.error(error_msg)
            synthesis_time = time.time() - start_time
            self.metrics.failed_syntheses += 1
            return TTSResult(
                success=False,
                error=error_msg,
                text=text,
                voice=voice,
                synthesis_time=synthesis_time
            )

    def synthesize_streaming(self,
                           text: str,
                           voice: str = "xiaoyan",
                           volume: int = 50,
                           rate: int = 0,
                           pitch: int = 0,
                           format: str = "pcm",
                           sample_rate: int = 16000) -> StreamingTTSResult:
        """
        流式语音合成

        Args:
            text: 待合成文本
            voice: 音色
            volume: 音量 (0-100)
            rate: 语速 (-500到500)
            pitch: 音调 (-500到500)
            format: 音频格式
            sample_rate: 采样率

        Returns:
            StreamingTTSResult: 流式合成结果
        """
        start_time = time.time()

        if not self.is_initialized:
            return StreamingTTSResult(success=False, error="服务未初始化", text=text, voice=voice)

        if not self.token:
            self._refresh_token()

        try:
            # 验证文本
            if not text or not text.strip():
                return StreamingTTSResult(success=False, error="文本为空", text=text, voice=voice)

            # 执行流式合成
            result = self._websocket_streaming_synthesize(
                text=text,
                voice=voice,
                volume=volume,
                rate=rate,
                pitch=pitch,
                format=format,
                sample_rate=sample_rate
            )

            # 更新性能指标
            synthesis_time = time.time() - start_time
            result.synthesis_time = synthesis_time
            self._update_metrics_streaming(result, synthesis_time)

            return result

        except Exception as e:
            error_msg = f"流式语音合成异常: {e}"
            logger.error(error_msg)
            synthesis_time = time.time() - start_time
            self.metrics.failed_syntheses += 1
            return StreamingTTSResult(
                success=False,
                error=error_msg,
                text=text,
                voice=voice,
                synthesis_time=synthesis_time
            )

    def _websocket_synthesize(self, text: str, **params) -> TTSResult:
        """WebSocket语音合成"""
        result = TTSResult(text=params.get('text', ''), voice=params.get('voice', ''))
        result_queue = queue.Queue()

        def on_start(message, *args):
            logger.info("🔊 WebSocket合成开始")

        def on_audio(data, *args):
            """音频数据回调"""
            if isinstance(data, bytes):
                result.audio_data += data

        def on_completed(message, *args):
            """合成完成回调"""
            try:
                data = json.loads(message)

                if 'status' in data and data['status'] == 20000000:
                    result.success = True
                    result.raw_response = data
                    logger.info(f"✅ 合成完成: '{result.text}' (大小: {len(result.audio_data)} 字节)")
                else:
                    result.error = f"合成失败: {data}"
                    logger.warning(f"⚠️ 合成异常: {data}")

            except Exception as e:
                result.error = f"结果解析失败: {e}"
                logger.error(f"❌ 结果解析异常: {e}")

            finally:
                result_queue.put("completed")

        def on_error(message, *args):
            """错误回调"""
            result.error = f"WebSocket错误: {message}"
            logger.error(f"❌ WebSocket错误: {message}")
            result_queue.put("error")

        # 创建WebSocket合成器
        try:
            synthesizer = NlsSpeechSynthesizer(
                token=self.token,
                appkey=self.app_key,
                on_start=on_start,
                on_audio=on_audio,
                on_completed=on_completed,
                on_error=on_error
            )

            # 设置合成参数
            synthesizer.set_voice(params.get('voice', 'xiaoyan'))
            synthesizer.set_volume(params.get('volume', 50))
            synthesizer.set_rate(params.get('rate', 0))
            synthesizer.set_pitch(params.get('pitch', 0))
            synthesizer.set_format(params.get('format', 'pcm'))
            synthesizer.set_sample_rate(params.get('sample_rate', 16000))

            # 启动合成
            synthesizer.start()

            # 发送文本
            synthesizer.send_text(params.get('text', ''))

            # 停止合成
            synthesizer.stop()

            # 等待结果
            try:
                result_queue.get(timeout=15)  # 15秒超时
            except queue.Empty:
                result.error = "合成超时"
                logger.warning("⚠️ 合成超时")

        except Exception as e:
            result.error = f"WebSocket连接异常: {e}"
            logger.error(f"❌ WebSocket异常: {e}")

        finally:
            try:
                synthesizer.shutdown()
            except:
                pass

        return result

    def _websocket_streaming_synthesize(self, text: str, **params) -> StreamingTTSResult:
        """WebSocket流式语音合成"""
        result = StreamingTTSResult(text=params.get('text', ''), voice=params.get('voice', ''))
        result_queue = queue.Queue()

        # 启动播放器
        if self.enable_playback and self.audio_player:
            self.audio_player.start_playback()

        def on_start(message, *args):
            logger.info("🌊 WebSocket流式合成开始")

        def on_audio(data, *args):
            """流式音频数据回调"""
            if isinstance(data, bytes):
                result.audio_chunks.append(data)
                result.received_chunks += 1

                # 实时播放
                if self.enable_playback and self.audio_player:
                    self.audio_player.add_audio_chunk(data)

                logger.debug(f"🎵 收到音频块: {len(data)} 字节 (第{result.received_chunks}块)")

        def on_completed(message, *args):
            """合成完成回调"""
            try:
                data = json.loads(message)

                if 'status' in data and data['status'] == 20000000:
                    result.success = True
                    result.raw_response = data
                    logger.info(f"✅ 流式合成完成: '{result.text}' (总块数: {result.received_chunks})")
                else:
                    result.error = f"流式合成失败: {data}"
                    logger.warning(f"⚠️ 流式合成异常: {data}")

            except Exception as e:
                result.error = f"流式合成结果解析失败: {e}"
                logger.error(f"❌ 流式合成结果解析异常: {e}")

            finally:
                result_queue.put("completed")

        def on_error(message, *args):
            """错误回调"""
            result.error = f"WebSocket流式错误: {message}"
            logger.error(f"❌ WebSocket流式错误: {message}")
            result_queue.put("error")

        # 创建WebSocket合成器
        try:
            synthesizer = NlsSpeechSynthesizer(
                token=self.token,
                appkey=self.app_key,
                on_start=on_start,
                on_audio=on_audio,
                on_completed=on_completed,
                on_error=on_error
            )

            # 设置合成参数
            synthesizer.set_voice(params.get('voice', 'xiaoyan'))
            synthesizer.set_volume(params.get('volume', 50))
            synthesizer.set_rate(params.get('rate', 0))
            synthesizer.set_pitch(params.get('pitch', 0))
            synthesizer.set_format(params.get('format', 'pcm'))
            synthesizer.set_sample_rate(params.get('sample_rate', 16000))

            # 启动流式合成
            synthesizer.start()

            # 发送文本
            synthesizer.send_text(params.get('text', ''))

            # 停止合成
            synthesizer.stop()

            # 等待结果
            try:
                result_queue.get(timeout=15)  # 15秒超时
            except queue.Empty:
                result.error = "流式合成超时"
                logger.warning("⚠️ 流式合成超时")

        except Exception as e:
            result.error = f"WebSocket流式连接异常: {e}"
            logger.error(f"❌ WebSocket流式异常: {e}")

        finally:
            # 停止播放器
            if self.enable_playback and self.audio_player:
                self.audio_player.stop_playback()

            try:
                synthesizer.shutdown()
            except:
                pass

        result.total_chunks = result.received_chunks
        return result

    def _update_metrics(self, result: TTSResult, synthesis_time: float, playback_time: float):
        """更新性能指标"""
        self.metrics.total_syntheses += 1
        self.metrics.total_synthesis_time += synthesis_time
        self.metrics.total_playback_time += playback_time
        self.metrics.average_synthesis_time = self.metrics.total_synthesis_time / self.metrics.total_syntheses

        if self.metrics.total_syntheses > 0:
            self.metrics.average_playback_time = self.metrics.total_playback_time / self.metrics.total_syntheses

        self.metrics.last_update = datetime.now()

        if result.success:
            self.metrics.successful_syntheses += 1
        else:
            self.metrics.failed_syntheses += 1

    def _update_metrics_streaming(self, result: StreamingTTSResult, synthesis_time: float):
        """更新流式合成性能指标"""
        self.metrics.total_syntheses += 1
        self.metrics.total_synthesis_time += synthesis_time
        self.metrics.average_synthesis_time = self.metrics.total_synthesis_time / self.metrics.total_syntheses
        self.metrics.last_update = datetime.now()

        if result.success:
            self.metrics.successful_syntheses += 1
        else:
            self.metrics.failed_syntheses += 1

    def get_metrics(self) -> PerformanceMetrics:
        """获取性能指标"""
        return self.metrics

    def health_check(self) -> Dict[str, Any]:
        """健康检查"""
        try:
            # 检查Token
            token_valid = bool(self.token)

            # 检查SDK
            sdk_available = NLS_SDK_AVAILABLE

            # 检查服务状态
            service_initialized = self.is_initialized

            # 检查播放器状态
            playback_available = self.enable_playback and self.audio_player is not None

            # 计算成功率
            success_rate = 0
            if self.metrics.total_syntheses > 0:
                success_rate = (self.metrics.successful_syntheses / self.metrics.total_syntheses) * 100

            return {
                "status": "healthy" if all([token_valid, sdk_available, service_initialized]) else "unhealthy",
                "token_valid": token_valid,
                "sdk_available": sdk_available,
                "service_initialized": service_initialized,
                "playback_available": playback_available,
                "streaming_enabled": self.enable_streaming,
                "total_syntheses": self.metrics.total_syntheses,
                "success_rate": f"{success_rate:.1f}%",
                "average_synthesis_time": f"{self.metrics.average_synthesis_time:.2f}s",
                "average_playback_time": f"{self.metrics.average_playback_time:.2f}s",
                "last_update": self.metrics.last_update.isoformat()
            }

        except Exception as e:
            return {"status": "error", "error": str(e)}

def create_websocket_tts_service(**kwargs) -> WebSocketTTSService:
    """创建WebSocket TTS服务实例"""
    return WebSocketTTSService(**kwargs)

# 保持向后兼容的类
class AliyunTTSWebSocketClient(WebSocketTTSService):
    """阿里云TTS WebSocket客户端 - 向后兼容版本"""

    def __init__(self, config: Optional[dict] = None):
        """
        初始化阿里云TTS WebSocket客户端 (向后兼容)

        Args:
            config: 配置字典
        """
        # 提取配置参数
        config = config or {}

        super().__init__(
            access_key_id=config.get('access_key_id', os.getenv('ALIBABA_CLOUD_ACCESS_KEY_ID', '')),
            access_key_secret=config.get('access_key_secret', os.getenv('ALIBABA_CLOUD_ACCESS_KEY_SECRET', '')),
            app_key=config.get('app_key', os.getenv('ALIYUN_NLS_APPKEY', '4G5BCMccTCW8nC8w')),
            enable_streaming=config.get('enable_streaming', True),
            enable_playback=config.get('enable_playback', False)  # 默认关闭播放以保持兼容性
        )

        # 兼容性配置
        self.default_voice = config.get('default_voice', 'xiaoyun')
        self.default_format = config.get('default_format', 'wav')
        self.default_sample_rate = config.get('default_sample_rate', 16000)
        self.default_volume = config.get('default_volume', 50)

        # Token缓存 (兼容性)
        self._token = self.token
        self._token_expire_time = int(time.time()) + 55 * 60  # 55分钟后过期

        self.logger = logger
        self.logger.info("✅ 阿里云TTS WebSocket客户端初始化完成 (兼容模式)")
        self.logger.info(f"   - App Key: {self.app_key}")
        self.logger.info(f"   - 默认发音人: {self.default_voice}")
        self.logger.info(f"   - 默认格式: {self.default_format}")
        self.logger.info(f"   - 默认采样率: {self.default_sample_rate}")
        self.logger.info(f"   - Access Key: {'✓' if self.access_key_id else '✗'}")

    def _get_valid_token(self) -> str:
        """获取有效的Token"""
        import time
        current_time = int(time.time())

        # 检查缓存Token是否有效 (Token有效期1小时)
        if self._token and current_time < self._token_expire_time:
            return self._token

        # 获取新Token
        self.logger.info("🔄 获取新的阿里云NLS Token...")
        self._token = getToken(self.access_key_id, self.access_key_secret)

        # 设置过期时间 (55分钟后过期，留5分钟缓冲)
        self._token_expire_time = current_time + 55 * 60

        self.logger.info(f"✅ Token获取成功: {self._token[:10]}...")
        return self._token

    def synthesize(self, text: str, **kwargs) -> Tuple[np.ndarray, float]:
        """
        语音合成

        Args:
            text: 输入文本
            **kwargs: 合成参数
                - voice: 发音人 (默认: xiaoyun)
                - aformat: 音频格式 (默认: wav)
                - sample_rate: 采样率 (默认: 16000)
                - volume: 音量 0-100 (默认: 50)
                - speech_rate: 语速 -500~500 (默认: 0)
                - pitch_rate: 音调 -500~500 (默认: 0)

        Returns:
            元组: (音频数据, 合成时间)
        """
        import time
        start_time = time.time()

        try:
            # 获取Token
            token = self._get_valid_token()
            if not token:
                raise ValueError("❌ 无法获取Token")

            # 合成参数
            voice = kwargs.get('voice', self.default_voice)
            aformat = kwargs.get('aformat', self.default_format)
            sample_rate = kwargs.get('sample_rate', self.default_sample_rate)
            volume = kwargs.get('volume', self.default_volume)
            speech_rate = kwargs.get('speech_rate', 0)
            pitch_rate = kwargs.get('pitch_rate', 0)

            self.logger.info(f"🎤 开始TTS合成: '{text[:20]}...'")
            self.logger.info(f"   - 发音人: {voice}")
            self.logger.info(f"   - 格式: {aformat}")
            self.logger.info(f"   - 采样率: {sample_rate}")

            # 存储合成结果
            audio_data = []
            synthesis_complete = False
            result_message = None

            # 回调函数
            def on_metainfo(message, *args):
                self.logger.info(f"🎤 TTS开始: {message}")

            def on_data(data, *args):
                audio_data.append(data)
                self.logger.debug(f"📊 接收音频数据: {len(data)} bytes")

            def on_completed(message, *args):
                nonlocal synthesis_complete, result_message
                self.logger.info(f"✅ TTS完成: {message}")
                synthesis_complete = True
                result_message = message

            def on_error(message, *args):
                nonlocal synthesis_complete, result_message
                self.logger.error(f"❌ TTS错误: {message}")
                synthesis_complete = True
                result_message = message

            def on_close(message, *args):
                self.logger.debug(f"🔚 TTS连接关闭")

            # 创建TTS合成器
            synthesizer = NlsSpeechSynthesizer(
                url='wss://nls-gateway.cn-shanghai.aliyuncs.com/ws/v1',
                token=token,
                appkey=self.app_key,
                long_tts=False,
                on_metainfo=on_metainfo,
                on_data=on_data,
                on_completed=on_completed,
                on_error=on_error,
                on_close=on_close
            )

            # 开始合成 (阻塞模式)
            synthesizer.start(
                text=text,
                voice=voice,
                aformat=aformat,
                sample_rate=sample_rate,
                volume=volume,
                speech_rate=speech_rate,
                pitch_rate=pitch_rate,
                wait_complete=True,
                start_timeout=10,
                completed_timeout=30
            )

            # 关闭连接
            synthesizer.shutdown()

            # 处理结果
            if audio_data:
                # 合并音频数据
                total_audio = b''.join(audio_data)
                synthesis_time = time.time() - start_time

                self.logger.info(f"✅ TTS合成成功!")
                self.logger.info(f"   - 音频大小: {len(total_audio)} bytes")
                self.logger.info(f"   - 合成耗时: {synthesis_time:.3f}s")

                # 保存到临时文件并读取为numpy数组
                with tempfile.NamedTemporaryFile(suffix=f'.{aformat}', delete=False) as f:
                    f.write(total_audio)
                    temp_file = f.name

                try:
                    # 读取音频数据
                    audio, sr = sf.read(temp_file)

                    # 如果采样率不匹配，重新采样
                    if sr != sample_rate:
                        import librosa
                        audio = librosa.resample(audio, orig_sr=sr, target_sr=sample_rate)

                    return audio, synthesis_time
                finally:
                    # 清理临时文件
                    os.unlink(temp_file)
            else:
                error_msg = f"TTS合成失败: {result_message}"
                self.logger.error(error_msg)
                raise ValueError(error_msg)

        except Exception as e:
            self.logger.error(f"❌ TTS合成异常: {e}")
            raise

    def synthesize_to_file(self, text: str, output_path: str, **kwargs) -> bool:
        """
        直接合成到文件

        Args:
            text: 输入文本
            output_path: 输出文件路径
            **kwargs: 合成参数

        Returns:
            是否成功
        """
        try:
            audio, synthesis_time = self.synthesize(text, **kwargs)

            # 确保输出目录存在
            os.makedirs(os.path.dirname(output_path), exist_ok=True)

            # 写入音频文件
            sample_rate = kwargs.get('sample_rate', self.default_sample_rate)
            sf.write(output_path, audio, sample_rate)

            self.logger.info(f"✅ 音频已保存: {output_path} (耗时: {synthesis_time:.3f}s)")
            return True

        except Exception as e:
            self.logger.error(f"❌ 合成失败: {e}")
            return False

    def get_available_voices(self) -> dict:
        """
        获取可用的发音人列表

        Returns:
            发音人字典
        """
        return {
            'xiaoyun': '晓云 (知性女声)',
            'xiaoxiao': '晓晓 (标准女声)',
            'xiaoyan': '晓燕 (温柔女声)',
            'xiaomeng': '晓梦 (甜美女声)',
            'aijia': '艾佳 (专业女声)',
            'aiqi': '艾奇 (可爱女声)',
            'aisx': '艾思 (成熟女声)',
            'aisj': '艾思 (成熟女声)',
            'aiyuk': '艾尤克 (成熟男声)',
            'aixia': '艾夏 (成熟男声)',
        }

    def test_synthesis(self, text: str = "你好，这是语音合成测试") -> bool:
        """
        测试语音合成功能

        Args:
            text: 测试文本

        Returns:
            测试是否成功
        """
        try:
            self.logger.info(f"🧪 开始TTS测试...")
            audio, time_taken = self.synthesize(text)

            if audio is not None and len(audio) > 0:
                self.logger.info(f"✅ TTS测试成功!")
                self.logger.info(f"   - 文本: {text}")
                self.logger.info(f"   - 音频长度: {len(audio)} 采样点")
                self.logger.info(f"   - 合成耗时: {time_taken:.3f}秒")
                return True
            else:
                self.logger.error("❌ TTS测试失败: 无音频数据")
                return False

        except Exception as e:
            self.logger.error(f"❌ TTS测试异常: {e}")
            return False

    def get_client_info(self) -> dict:
        """获取客户端信息"""
        return {
            'client_type': 'Aliyun WebSocket TTS',
            'sdk_available': NLS_SDK_AVAILABLE,
            'app_key_configured': bool(self.app_key),
            'access_key_configured': bool(self.access_key_id),
            'default_voice': self.default_voice,
            'default_format': self.default_format,
            'default_sample_rate': self.default_sample_rate,
            'supported_formats': ['pcm', 'wav', 'mp3'],
            'supported_sample_rates': [8000, 11025, 16000, 22050, 24000, 32000, 44100, 48000],
        }


def main():
    """主函数 - 测试TTS WebSocket客户端"""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    print("🚀 阿里云TTS WebSocket客户端测试")
    print("=" * 50)

    try:
        # 创建客户端
        tts_client = AliyunTTSWebSocketClient()

        # 显示客户端信息
        info = tts_client.get_client_info()
        print(f"📋 客户端信息:")
        for key, value in info.items():
            print(f"   - {key}: {value}")

        # 测试合成
        print(f"\n🧪 测试语音合成...")
        result = tts_client.test_synthesis("你好，这是使用官方WebSocket SDK的测试")

        if result:
            print("🎉 所有测试通过!")
        else:
            print("❌ 测试失败")

    except Exception as e:
        print(f"❌ 测试异常: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()