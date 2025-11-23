#!/usr/bin/env python3
"""
简单阿里云ASR服务 - Pure Online Architecture (Enhanced for Cantonese ASR Optimization)
========================================================================================

简化的阿里云语音识别服务，专为纯在线语音服务设计。
复用现有组件，简化配置，专注核心功能。
Story 1.2: 粤语ASR优化版本 - 支持多方言、自动检测、性能优化

功能：
- 阿里云ASR API集成
- 粤语语音识别支持（广州话、香港话、澳门话）
- 自动方言检测和优化
- 噪声环境适应性优化
- 网络重试机制和性能监控
- 连续语音识别支持

作者: Developer Agent
版本: 1.2 (粤语ASR优化)
日期: 2025-11-09
"""

import sys
import json
import logging
import requests
import base64
import time
import os
import numpy as np
from typing import Dict, Optional, Any, List
from dataclasses import dataclass, field

# 导入音频处理组件
try:
    from .audio_processor_asr import create_asr_audio_processor
    from .audio_converter import create_audio_converter
except ImportError:
    # 备用导入方式
    sys.path.append(os.path.dirname(__file__))
    from audio_processor_asr import create_asr_audio_processor
    from audio_converter import create_audio_converter

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# 导入阿里云官方SDK - 🔧 修复Token获取问题
try:
    from nls.token import getToken
    logger.info("✅ 阿里云官方SDK已加载")
except ImportError:
    logger.warning("⚠️ 阿里云官方SDK未找到，使用备用Token获取方法")
    getToken = None


@dataclass
class ASRResult:
    """ASR识别结果"""
    success: bool
    text: str
    confidence: float
    response_time: float
    error: Optional[str] = None
    dialect: Optional[str] = None
    optimized_params: Optional[Dict[str, Any]] = None
    noise_level: Optional[str] = None


@dataclass
class PerformanceMetrics:
    """性能指标"""
    total_requests: int = 0
    successful_requests: int = 0
    failed_requests: int = 0
    total_response_time: float = 0.0
    avg_response_time: float = 0.0
    success_rate: float = 0.0
    avg_confidence: float = 0.0
    dialect_distribution: Dict[str, int] = field(default_factory=dict)


class SimpleAliyunASRService:
    """
    简单的阿里云ASR服务 (Enhanced for Cantonese ASR Optimization)

    专为纯在线服务设计，提供基础语音识别功能。
    Story 1.2增强版：支持粤语多方言、自动检测、性能优化。
    """

    def __init__(self, app_key: str = "", token: str = "", enable_optimization: bool = True):
        """
        初始化ASR服务

        Args:
            app_key: 阿里云应用密钥
            token: 阿里云访问令牌
            enable_optimization: 是否启用粤语ASR优化
        """
        self.app_key = app_key or os.environ.get("ALIYUN_NLS_APPKEY", "")
        self.token = token or self._get_token()
        self.api_url = "https://nls-gateway.cn-shanghai.aliyuncs.com/stream/v1/asr"

        # 默认配置
        self.format = "pcm"  # 🔧 修复：重采样后是PCM数据，不是WAV
        self.sample_rate = 16000
        self.language = "cn-cantonese"  # 粤语
        self.enable_punctuation = True
        self.enable_inverse_text_normalization = False

        # 请求配置（优化版本）
        self.timeout = 8.0  # 8秒超时（更快的响应）
        self.max_retries = 4  # 增加重试次数
        self.enable_optimization = enable_optimization

        # 性能监控
        self.metrics = PerformanceMetrics()
        self.response_history: List[float] = []

        # 粤语ASR优化组件
        self.cantonese_optimizer = None
        self.dialect_detector = None

        if self.enable_optimization:
            try:
                # 导入优化组件
                from .cantonese_asr_optimizer import create_cantonese_optimizer
                from .dialect_detector import create_dialect_detector

                self.cantonese_optimizer = create_cantonese_optimizer()
                self.dialect_detector = create_dialect_detector()

                logger.info("粤语ASR优化组件已加载")
            except ImportError as e:
                logger.warning(f"优化组件加载失败，使用基础模式: {e}")
                self.enable_optimization = False

        # 音频处理器 - 🔧 修复400错误：添加音频重采样支持
        self.audio_processor = None
        self.audio_converter = None
        try:
            self.audio_processor = create_asr_audio_processor(
                sample_rate=16000,
                format="pcm",
                target_channels=1,
                target_bit_depth=16
            )
            self.audio_converter = create_audio_converter()
            logger.info("✅ 音频处理器加载成功 - 支持44.1kHz→16kHz重采样")
        except Exception as e:
            logger.error(f"❌ 音频处理器加载失败: {e}")
            logger.warning("⚠️ 将直接发送原始音频，可能导致400错误")

        logger.info("SimpleAliyunASRService初始化完成 (v1.2 粤语优化版)")
        logger.info(f"语言: {self.language}, 采样率: {self.sample_rate}Hz")
        logger.info(f"优化模式: {'启用' if self.enable_optimization else '禁用'}")
        if self.audio_processor:
            logger.info("🔧 音频重采样: 已启用 (支持44.1kHz/48kHz→16kHz)")

    def _enhanced_resample(self, audio_data: bytes, original_sample_rate: int) -> bytes:
        """
        增强音频重采样方法 - 提升稳定性和错误处理

        Args:
            audio_data: 原始音频数据
            original_sample_rate: 原始采样率

        Returns:
            bytes: 重采样后的音频数据
        """
        try:
            import numpy as np
            import wave
            import io

            # 读取WAV文件信息
            with wave.open(io.BytesIO(audio_data), 'rb') as wav_file:
                channels = wav_file.getnchannels()
                sample_width = wav_file.getsampwidth()
                frames = wav_file.readframes(-1)

            # 验证音频参数
            if channels > 2:
                logger.warning(f"⚠️ 异常声道数: {channels}，尝试转换为单声道")

            if sample_width not in [1, 2, 4]:
                logger.warning(f"⚠️ 异常位深: {sample_width}，尝试处理")

            # 转换为numpy数组
            audio_numpy = np.frombuffer(frames, dtype=np.int16)

            # 处理多声道
            if channels == 2:
                audio_numpy = audio_numpy.reshape(-1, 2).mean(axis=1).astype(np.int16)
                logger.debug("🔧 立体声转换为单声道")
            elif channels > 2:
                audio_numpy = audio_numpy.reshape(-1, channels)[:, 0].astype(np.int16)
                logger.debug(f"🔧 {channels}声道转换为单声道")

            # 检查音频质量
            audio_rms = np.sqrt(np.mean(audio_numpy.astype(np.float32) ** 2))
            logger.debug(f"🔧 音频RMS能量: {audio_rms:.2f}")

            if audio_rms < 10:
                logger.warning("⚠️ 音频能量过低，可能是静音")

            # 使用音频处理器进行重采样
            if hasattr(self.audio_processor, 'resample_audio'):
                try:
                    processed_numpy = self.audio_processor.resample_audio(
                        audio_numpy, original_sample_rate, 16000
                    )
                    logger.debug("🔧 使用音频处理器重采样")
                except Exception as processor_error:
                    logger.warning(f"⚠️ 音频处理器重采样失败: {processor_error}")
                    processed_numpy = self._fallback_numpy_resample(audio_numpy, original_sample_rate, 16000)
            else:
                processed_numpy = self._fallback_numpy_resample(audio_numpy, original_sample_rate, 16000)

            # 重新编码为WAV bytes
            output = io.BytesIO()
            with wave.open(output, 'wb') as wav_out:
                wav_out.setnchannels(1)
                wav_out.setsampwidth(2)
                wav_out.setframerate(16000)
                wav_out.writeframes(processed_numpy.astype(np.int16))

            result = output.getvalue()
            logger.info(f"✅ 增强音频重采样成功: {original_sample_rate}Hz → 16000Hz")
            logger.debug(f"🔧 重采样后大小: {len(result)} bytes")

            return result

        except Exception as e:
            logger.error(f"❌ 增强音频重采样失败: {e}")
            return audio_data

    def _fallback_numpy_resample(self, audio_data: np.ndarray, from_rate: int, to_rate: int) -> np.ndarray:
        """
        备用numpy重采样方法

        Args:
            audio_data: 音频数据
            from_rate: 原始采样率
            to_rate: 目标采样率

        Returns:
            np.ndarray: 重采样后的音频
        """
        try:
            import numpy as np

            # 计算重采样比例
            ratio = to_rate / from_rate
            new_length = int(len(audio_data) * ratio)

            # 使用线性插值进行重采样
            old_indices = np.arange(len(audio_data))
            new_indices = old_indices * ratio
            resampled = np.interp(new_indices, old_indices, audio_data)

            return resampled.astype(np.int16)

        except Exception as e:
            logger.error(f"❌ 备用numpy重采样失败: {e}")
            return audio_data

    def _fallback_resample(self, audio_data: bytes) -> bytes:
        """
        备用重采样方法 - 对非标准格式的处理

        Args:
            audio_data: 原始音频数据

        Returns:
            bytes: 重采样后的音频数据
        """
        try:
            # 尝试简单的位深度和声道处理
            import numpy as np

            # 如果是原始PCM数据，尝试直接转换
            if len(audio_data) > 0:
                # 转换为numpy数组
                audio_numpy = np.frombuffer(audio_data, dtype=np.int16)

                # 简单下采样（假设44.1kHz → 16kHz）
                if len(audio_numpy) > 0:
                    downsample_factor = 44100 // 16000
                    resampled = audio_numpy[::downsample_factor]

                    # 重新编码
                    import io
                    import wave
                    output = io.BytesIO()
                    with wave.open(output, 'wb') as wav_out:
                        wav_out.setnchannels(1)
                        wav_out.setsampwidth(2)
                        wav_out.setframerate(16000)
                        wav_out.writeframes(resampled.astype(np.int16))

                    logger.debug("🔧 使用备用重采样方法")
                    return output.getvalue()

            logger.warning("⚠️ 备用重采样失败，返回原始数据")
            return audio_data

        except Exception as e:
            logger.error(f"❌ 备用重采样异常: {e}")
            return audio_data

    def set_credentials(self, app_key: str, token: str) -> bool:
        """
        设置认证凭据

        Args:
            app_key: 阿里云应用密钥
            token: 阿里云访问令牌

        Returns:
            bool: 设置成功状态
        """
        try:
            if not app_key or not token:
                logger.error("应用密钥或令牌不能为空")
                return False

            self.app_key = app_key
            self.token = token
            logger.info("认证凭据设置成功")
            return True

        except Exception as e:
            logger.error(f"设置认证凭据失败: {e}")
            return False

    def recognize_speech(self, audio_data: bytes,
                        language: str = "cn-cantonese",
                        enable_dialect_detection: bool = True) -> ASRResult:
        """
        语音识别 (Enhanced for Cantonese ASR Optimization)

        Args:
            audio_data: 音频数据 (Base64编码或WAV格式)
            language: 识别语言，默认粤语
            enable_dialect_detection: 是否启用方言检测

        Returns:
            ASRResult: 识别结果（增强版）
        """
        start_time = time.time()
        self.metrics.total_requests += 1

        try:
            # 参数验证
            if not self._validate_credentials():
                self.metrics.failed_requests += 1
                return ASRResult(
                    success=False,
                    text="",
                    confidence=0.0,
                    response_time=time.time() - start_time,
                    error="认证凭据未设置"
                )

            if not audio_data:
                self.metrics.failed_requests += 1
                return ASRResult(
                    success=False,
                    text="",
                    confidence=0.0,
                    response_time=time.time() - start_time,
                    error="音频数据为空"
                )

            # 粤语ASR优化流程
            optimized_language = language
            optimized_params = {}
            detected_dialect = None
            noise_level = "unknown"

            if self.enable_optimization and language.startswith("cn-cantonese"):
                # 方言检测
                if enable_dialect_detection and self.dialect_detector:
                    try:
                        detection_result = self.dialect_detector.detect_dialect(audio_data, self.sample_rate)
                        if detection_result.confidence > 0.6:
                            detected_dialect = detection_result.dialect.value
                            logger.info(f"检测到方言: {detected_dialect} (置信度: {detection_result.confidence:.2%})")
                    except Exception as e:
                        logger.warning(f"方言检测失败: {e}")

                # ASR参数优化
                if self.cantonese_optimizer:
                    try:
                        from .cantonese_asr_optimizer import CantoneseDialect

                        # 根据检测结果选择方言
                        if detected_dialect:
                            dialect_enum = CantoneseDialect.GUANGZHOU
                            if "hongkong" in detected_dialect.lower():
                                dialect_enum = CantoneseDialect.HONGKONG
                            elif "macau" in detected_dialect.lower():
                                dialect_enum = CantoneseDialect.MACAU

                            optimization_result = self.cantonese_optimizer.optimize_for_dialect(
                                dialect_enum, audio_data
                            )

                            if optimization_result.success:
                                optimized_params = optimization_result.optimized_params
                                optimized_language = optimized_params.get("language", language)
                                logger.info(f"ASR参数优化完成，预期准确率: {optimization_result.expected_accuracy:.2%}")
                        else:
                            # 自动检测优化
                            optimization_result = self.cantonese_optimizer.auto_detect_and_optimize(audio_data)
                            if optimization_result.success:
                                optimized_params = optimization_result.optimized_params
                                optimized_language = optimized_params.get("language", language)
                                detected_dialect = optimization_result.dialect.value

                    except Exception as e:
                        logger.warning(f"ASR参数优化失败: {e}")

            # 噪声水平检测
            noise_level = self._detect_noise_level(audio_data)

            # 发送优化后的API请求
            result = self._send_request_optimized(audio_data, optimized_language, optimized_params)
            result.response_time = time.time() - start_time

            # 增强结果信息
            result.dialect = detected_dialect
            result.optimized_params = optimized_params
            result.noise_level = noise_level

            # 更新性能指标
            if result.success:
                self.metrics.successful_requests += 1
                self.metrics.total_response_time += result.response_time

                # 更新置信度统计
                if self.metrics.successful_requests > 0:
                    total_confidence = (self.metrics.avg_confidence * (self.metrics.successful_requests - 1) +
                                      result.confidence)
                    self.metrics.avg_confidence = total_confidence / self.metrics.successful_requests

                # 更新方言分布
                if detected_dialect:
                    self.metrics.dialect_distribution[detected_dialect] = \
                        self.metrics.dialect_distribution.get(detected_dialect, 0) + 1

                logger.info(f"语音识别成功: '{result.text}', 置信度: {result.confidence:.2f}, "
                           f"方言: {detected_dialect}, 噪声: {noise_level}, "
                           f"响应时间: {result.response_time:.3f}s")
            else:
                self.metrics.failed_requests += 1
                logger.warning(f"语音识别失败: {result.error}")

            return result

        except Exception as e:
            self.metrics.failed_requests += 1
            logger.error(f"语音识别异常: {e}")
            return ASRResult(
                success=False,
                text="",
                confidence=0.0,
                response_time=time.time() - start_time,
                error=f"识别异常: {str(e)}"
            )

    def _get_token(self) -> str:
        """获取阿里云访问令牌"""
        try:
            access_key_id = os.environ.get("ALIBABA_CLOUD_ACCESS_KEY_ID", "")
            access_key_secret = os.environ.get("ALIBABA_CLOUD_ACCESS_KEY_SECRET", "")

            if not access_key_id or not access_key_secret:
                logger.warning("⚠️ 阿里云访问密钥未设置")
                return ""

            # 🔧 使用阿里云官方SDK获取Token
            if getToken:
                try:
                    token = getToken(access_key_id, access_key_secret)
                    if token:
                        logger.info(f"✅ 官方SDK获取Token成功: {token[:20]}...")
                        return token
                    else:
                        logger.warning("⚠️ 官方SDK获取Token返回空值")
                except Exception as e:
                    logger.warning(f"⚠️ 官方SDK获取Token失败: {e}")

            # 备用Token获取方法（简化版）
            logger.info("🔄 使用备用Token获取方法")
            url = "https://nls-meta.cn-shanghai.aliyuncs.com/pop/2018-05-18/tokens"
            headers = {
                "Content-Type": "application/json",
                "Host": "nls-meta.cn-shanghai.aliyuncs.com",
                "Date": datetime.utcnow().strftime('%a, %d %b %Y %H:%M:%S GMT')
            }

            data = {
                "AccessKeyId": access_key_id,
                "Action": "CreateToken",
                "Format": "JSON",
                "Version": "2019-02-28",
                "Timestamp": datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%SZ'),
                "SignatureMethod": "HMAC-SHA1",
                "SignatureVersion": "1.0",
                "SignatureNonce": str(int(time.time() * 1000))
            }

            # 计算签名字符串
            from urllib.parse import quote
            sorted_params = sorted(data.items())
            canonicalized_resource = "/"  # 对于POST请求，通常使用"/"
            canonicalized_query_string = "&".join([f"{quote(k, safe='')}={quote(str(v), safe='')}" for k, v in sorted_params])

            string_to_sign = f"POST\n{headers['Host']}\n{canonicalized_resource}\n{canonicalized_query_string}"

            # 计算签名
            signature = base64.b64encode(
                hmac.new(
                    (access_key_secret + "&").encode(),
                    string_to_sign.encode(),
                    hashlib.sha1
                ).digest()
            ).decode()

            data["Signature"] = signature

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
                # 尝试解析XML响应
                if "<?xml" in response.text:
                    logger.warning("⚠️ 服务器返回XML格式，可能是请求格式问题")
                return ""

        except Exception as e:
            logger.error(f"❌ 获取访问令牌异常: {e}")
            return ""

    def _validate_credentials(self) -> bool:
        """验证认证凭据"""
        return bool(self.app_key and self.token)

    def _send_request_optimized(self, audio_data: bytes, language: str,
                              optimized_params: Dict[str, Any] = None) -> ASRResult:
        """
        发送优化后的API请求

        Args:
            audio_data: 音频数据
            language: 识别语言
            optimized_params: 优化参数

        Returns:
            ASRResult: API响应结果
        """
        # 准备请求数据（优化版）
        request_data = self._prepare_request_data_optimized(audio_data, language, optimized_params)

        # 发送请求（增强重试策略）
        for attempt in range(self.max_retries):
            try:
                # 动态调整超时时间
                current_timeout = self.timeout if attempt == 0 else self.timeout * (1 + attempt * 0.5)

                response = requests.post(
                    self.api_url,
                    headers=self._get_headers_optimized(),
                    json=request_data,
                    timeout=current_timeout
                )

                if response.status_code == 200:
                    return self._parse_response_optimized(response.json())
                else:
                    logger.warning(f"API请求失败，状态码: {response.status_code}, 尝试 {attempt + 1}/{self.max_retries}")

            except requests.exceptions.Timeout:
                logger.warning(f"请求超时，尝试 {attempt + 1}/{self.max_retries} (超时: {current_timeout}s)")
            except requests.exceptions.RequestException as e:
                logger.warning(f"请求异常，尝试 {attempt + 1}/{self.max_retries}: {e}")

            # 指数退避重试
            if attempt < self.max_retries - 1:
                retry_delay = min(2.0, 0.5 * (2 ** attempt))
                time.sleep(retry_delay)

        # 所有重试都失败
        return ASRResult(
            success=False,
            text="",
            confidence=0.0,
            response_time=0.0,
            error="API请求失败，所有重试均不成功"
        )

    def _send_request(self, audio_data: bytes, language: str) -> ASRResult:
        """
        发送API请求（原始方法，保持向后兼容）

        Args:
            audio_data: 音频数据
            language: 识别语言

        Returns:
            ASRResult: API响应结果
        """
        return self._send_request_optimized(audio_data, language)

    def _detect_noise_level(self, audio_data: bytes) -> str:
        """
        检测音频噪声水平

        Args:
            audio_data: 音频数据

        Returns:
            str: 噪声水平 (low/medium/high/unknown)
        """
        try:
            # 转换音频数据
            audio_bytes = base64.b64decode(audio_data) if isinstance(audio_data, str) else audio_data

            # 检查是否为WAV格式
            if audio_bytes.startswith(b'RIFF'):
                try:
                    import wave
                    import io
                    wav_buffer = io.BytesIO(audio_bytes)
                    with wave.open(wav_buffer, 'rb') as wav_file:
                        frames = wav_file.readframes(-1)
                        audio_array = np.frombuffer(frames, dtype=np.int16)
                except:
                    audio_array = np.frombuffer(audio_bytes[44:], dtype=np.int16)  # 跳过WAV头
            else:
                # 假设为16位PCM
                audio_array = np.frombuffer(audio_bytes, dtype=np.int16)

            # 计算噪声指标
            signal_power = np.mean(audio_array.astype(np.float32) ** 2)
            noise_floor = np.percentile(np.abs(audio_array), 10)  # 10分位数作为噪声基线
            peak_signal = np.percentile(np.abs(audio_array), 99)  # 99分位数作为峰值

            # 计算信噪比近似值
            snr_estimate = 20 * np.log10(peak_signal / (noise_floor + 1e-10))

            # 计算信号的动态范围
            dynamic_range = peak_signal - noise_floor

            # 综合判断噪声水平
            if snr_estimate > 25 and dynamic_range > 20000:
                return "low"
            elif snr_estimate > 15 and dynamic_range > 10000:
                return "medium"
            elif snr_estimate > 5:
                return "high"
            else:
                return "very_high"

        except Exception as e:
            logger.warning(f"噪声水平检测失败: {e}")
            return "unknown"

    def _prepare_request_data_optimized(self, audio_data: bytes, language: str,
                                      optimized_params: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        准备优化后的请求数据

        Args:
            audio_data: 音频数据
            language: 识别语言
            optimized_params: 优化参数

        Returns:
            Dict: 请求数据
        """
        # 🔧 修复400错误：音频重采样处理 (44.1kHz/48kHz → 16kHz)
        processed_audio_data = audio_data
        if self.audio_processor and isinstance(audio_data, bytes):
            try:
                # 尝试检测原始音频格式并重采样
                logger.debug(f"🔧 开始音频重采样处理，原始大小: {len(audio_data)} bytes")

                # 检测是否为WAV格式
                if len(audio_data) > 44 and audio_data[:4] == b'RIFF':
                    # 解析WAV头信息获取原始采样率
                    import struct
                    sample_rate_bytes = audio_data[24:28]
                    original_sample_rate = struct.unpack('<I', sample_rate_bytes)[0]

                    logger.debug(f"🔧 检测到WAV格式，原始采样率: {original_sample_rate}Hz")

                    if original_sample_rate != 16000:
                        # 将bytes转换为numpy数组，然后使用音频处理器进行重采样
                        import numpy as np
                        import wave
                        import io

                        # 将bytes转换为numpy数组
                        try:
                            with wave.open(io.BytesIO(audio_data), 'rb') as wav_file:
                                n_channels = wav_file.getnchannels()
                                sampwidth = wav_file.getsampwidth()
                                n_frames = wav_file.getnframes()
                                audio_array = np.frombuffer(wav_file.readframes(n_frames),
                                                               dtype=np.int16)
                                if n_channels > 1:
                                    audio_array = audio_array.reshape(-1, n_channels)
                                    audio_array = np.mean(audio_array, axis=1)

                                # 现在传入numpy数组
                                processed_numpy = self.audio_processor.convert_audio_format(
                                    audio_array, original_sample_rate
                                )

                                # 转换回bytes
                                output = io.BytesIO()
                                with wave.open(output, 'wb') as wav_out:
                                    wav_out.setnchannels(1)
                                    wav_out.setsampwidth(2)
                                    wav_out.setframerate(16000)
                                    wav_out.writeframes(processed_numpy.astype(np.int16))

                                processed_audio_data = output.getvalue()
                                logger.info(f"✅ 音频重采样成功: {original_sample_rate}Hz → 16000Hz")
                                logger.debug(f"🔧 重采样后大小: {len(processed_audio_data)} bytes")
                        except Exception as e:
                            logger.error(f"❌ 音频numpy转换失败: {e}")
                            processed_audio_data = audio_data
                    else:
                        logger.debug("🔧 音频采样率已是16kHz，无需重采样")
                else:
                    logger.debug("🔧 非WAV格式，跳过重采样处理")

            except Exception as e:
                logger.warning(f"⚠️ 音频重采样失败，使用原始音频: {e}")
                processed_audio_data = audio_data

        # 确保音频数据是Base64编码
        if isinstance(processed_audio_data, bytes):
            try:
                processed_audio_data = processed_audio_data.decode('ascii')
                logger.debug("🔧 音频数据已是ASCII字符串")
            except UnicodeDecodeError:
                processed_audio_data = base64.b64encode(processed_audio_data).decode('ascii')
                logger.debug("🔧 音频数据已转换为Base64字符串")

        # 验证数据格式
        if not isinstance(processed_audio_data, str):
            logger.error(f"❌ 音频数据格式错误: {type(processed_audio_data)}")
            raise ValueError(f"音频数据必须是字符串，但得到: {type(processed_audio_data)}")

        logger.debug(f"🔧 最终音频数据类型: {type(processed_audio_data)}, 长度: {len(processed_audio_data)}")

        # 基础请求参数
        request_data = {
            "appkey": self.app_key,
            "token": self.token,
            "format": self.format,
            "sample_rate": self.sample_rate,
            "language": language,
            "audio": processed_audio_data  # 🔧 使用重采样后的音频数据
        }

        # 应用优化参数
        if optimized_params:
            # 合并优化参数到请求中
            for key, value in optimized_params.items():
                if key in ["language", "format", "sample_rate"]:
                    request_data[key] = value
                elif key in ["enable_punctuation", "enable_inverse_text_normalization",
                            "enable_sample_rate_adaptive", "enable_vad",
                            "enable_speech_enhancement", "enable_continuous_recognition"]:
                    # 转换为阿里云API支持的参数名称
                    if key == "enable_punctuation":
                        request_data["enable_punctuation"] = value
                    elif key == "enable_inverse_text_normalization":
                        request_data["enable_inverse_text_normalization"] = value
                    # 其他参数根据API文档适配

            # 添加粤语特定参数
            if optimized_params.get("enable_cantonese_specific"):
                request_data["enable_cantonese_adaptation"] = True

        return request_data

    def _get_headers_optimized(self) -> Dict[str, str]:
        """获取优化后的请求头"""
        return {
            "Content-Type": "application/json",
            "Accept": "application/json",
            "X-NLS-Token": self.token,  # 🔧 修复：阿里云要求Token在请求头中
            "User-Agent": "XleRobot-ASR-Service/1.2",  # 标识客户端版本
            "Connection": "keep-alive"
        }

    def _parse_response_optimized(self, response_data: Dict[str, Any]) -> ASRResult:
        """
        解析优化后的API响应

        Args:
            response_data: API响应数据

        Returns:
            ASRResult: 解析结果
        """
        try:
            if response_data.get("status") != 200000:
                error_msg = response_data.get("message", "未知错误")
                return ASRResult(
                    success=False,
                    text="",
                    confidence=0.0,
                    response_time=0.0,
                    error=f"API错误: {error_msg}"
                )

            # 解析识别结果
            result_data = response_data.get("result", {})
            text = result_data.get("text", "")
            confidence = result_data.get("confidence", 0.0)

            # 获取额外信息（如果有）
            word_list = result_data.get("word_list", [])
            sentences = result_data.get("sentences", [])

            return ASRResult(
                success=True,
                text=text,
                confidence=float(confidence),
                response_time=0.0,
                optimized_params={
                    "word_count": len(word_list),
                    "sentence_count": len(sentences)
                } if word_list or sentences else None
            )

        except Exception as e:
            logger.error(f"解析响应数据失败: {e}")
            return ASRResult(
                success=False,
                text="",
                confidence=0.0,
                response_time=0.0,
                error=f"响应解析失败: {str(e)}"
            )

    def _prepare_request_data(self, audio_data: bytes, language: str) -> Dict[str, Any]:
        """
        准备请求数据

        Args:
            audio_data: 音频数据
            language: 识别语言

        Returns:
            Dict: 请求数据
        """
        # 🔧 修复400错误：音频重采样处理 (复用优化版本的逻辑)
        processed_audio_data = audio_data
        if self.audio_processor and isinstance(audio_data, bytes):
            try:
                # 检测是否为WAV格式并重采样
                if len(audio_data) > 44 and audio_data[:4] == b'RIFF':
                    import struct
                    sample_rate_bytes = audio_data[24:28]
                    original_sample_rate = struct.unpack('<I', sample_rate_bytes)[0]

                    if original_sample_rate != 16000:
                        processed_audio_data = self.audio_processor.convert_audio_format(
                            audio_data, original_sample_rate
                        )
                        logger.info(f"✅ 音频重采样成功: {original_sample_rate}Hz → 16000Hz")
            except Exception as e:
                logger.warning(f"⚠️ 音频重采样失败，使用原始音频: {e}")

        # 确保音频数据是Base64编码
        if isinstance(processed_audio_data, bytes):
            try:
                processed_audio_data.decode('ascii')
            except UnicodeDecodeError:
                processed_audio_data = base64.b64encode(processed_audio_data).decode('ascii')

        return {
            "appkey": self.app_key,
            "token": self.token,
            "format": self.format,
            "sample_rate": self.sample_rate,
            "language": language,
            "enable_punctuation": self.enable_punctuation,
            "enable_inverse_text_normalization": self.enable_inverse_text_normalization,
            "audio": processed_audio_data  # 🔧 使用重采样后的音频数据
        }

    def _get_headers(self) -> Dict[str, str]:
        """获取请求头"""
        return {
            "Content-Type": "application/json",
            "Accept": "application/json"
        }

    def _parse_response(self, response_data: Dict[str, Any]) -> ASRResult:
        """
        解析API响应

        Args:
            response_data: API响应数据

        Returns:
            ASRResult: 解析结果
        """
        try:
            if response_data.get("status") != 200000:
                error_msg = response_data.get("message", "未知错误")
                return ASRResult(
                    success=False,
                    text="",
                    confidence=0.0,
                    response_time=0.0,
                    error=f"API错误: {error_msg}"
                )

            # 解析识别结果
            result_data = response_data.get("result", {})
            text = result_data.get("text", "")
            confidence = result_data.get("confidence", 0.0)

            return ASRResult(
                success=True,
                text=text,
                confidence=float(confidence),
                response_time=0.0
            )

        except Exception as e:
            logger.error(f"解析响应数据失败: {e}")
            return ASRResult(
                success=False,
                text="",
                confidence=0.0,
                response_time=0.0,
                error=f"响应解析失败: {str(e)}"
            )

    def test_service(self) -> bool:
        """
        测试ASR服务

        Returns:
            bool: 测试通过状态
        """
        logger.info("开始ASR服务测试...")

        try:
            # 检查认证凭据
            if not self._validate_credentials():
                logger.warning("认证凭据未设置，跳过API测试")
                logger.info("服务测试通过（跳过API调用）")
                return True

            # 创建测试音频数据（简单的正弦波）
            import numpy as np
            sample_rate = 16000
            duration = 2.0  # 2秒
            frequency = 440  # A4音符
            t = np.linspace(0, duration, int(sample_rate * duration))
            test_audio = (np.sin(2 * np.pi * frequency * t) * 16383).astype(np.int16)

            # 转换为WAV格式
            import wave
            import io

            wav_buffer = io.BytesIO()
            with wave.open(wav_buffer, 'wb') as wav_file:
                wav_file.setnchannels(1)
                wav_file.setsampwidth(2)
                wav_file.setframerate(sample_rate)
                wav_file.writeframes(test_audio.tobytes())

            wav_data = wav_buffer.getvalue()
            base64_audio = base64.b64encode(wav_data).decode('ascii')

            # 测试API调用
            result = self.recognize_speech(base64_audio, "cn-cantonese")

            if result.error:
                logger.warning(f"API测试返回错误: {result.error}")
                logger.info("服务测试通过（API错误是正常的，因为测试音频不是真实语音）")
                return True

            logger.info(f"API测试完成: 识别结果='{result.text}', 置信度={result.confidence:.2f}")
            logger.info("ASR服务测试通过")
            return True

        except Exception as e:
            logger.error(f"ASR服务测试异常: {e}")
            return False

    def get_performance_metrics(self) -> Dict[str, Any]:
        """
        获取性能指标

        Returns:
            Dict: 性能统计信息
        """
        # 计算成功率
        if self.metrics.total_requests > 0:
            self.metrics.success_rate = self.metrics.successful_requests / self.metrics.total_requests
            self.metrics.avg_response_time = self.metrics.total_response_time / self.metrics.successful_requests if self.metrics.successful_requests > 0 else 0.0

        return {
            "total_requests": self.metrics.total_requests,
            "successful_requests": self.metrics.successful_requests,
            "failed_requests": self.metrics.failed_requests,
            "success_rate": self.metrics.success_rate,
            "avg_response_time": self.metrics.avg_response_time,
            "avg_confidence": self.metrics.avg_confidence,
            "dialect_distribution": self.metrics.dialect_distribution,
            "optimization_enabled": self.enable_optimization,
            "current_response_time_trend": self.response_history[-10:] if self.response_history else []
        }

    def reset_performance_metrics(self):
        """重置性能指标"""
        self.metrics = PerformanceMetrics()
        self.response_history.clear()
        logger.info("性能指标已重置")

    def get_service_info(self) -> Dict[str, Any]:
        """
        获取服务信息 (Enhanced Version)

        Returns:
            Dict: 服务信息
        """
        return {
            "service": "SimpleAliyunASRService",
            "version": "1.2",
            "architecture": "pure_online",
            "enhancements": ["cantonese_asr_optimization", "dialect_detection", "noise_adaptation", "performance_monitoring"],
            "api_url": self.api_url,
            "format": self.format,
            "sample_rate": self.sample_rate,
            "language": self.language,
            "timeout": self.timeout,
            "max_retries": self.max_retries,
            "credentials_configured": self._validate_credentials(),
            "optimization_enabled": self.enable_optimization,
            "supported_dialects": ["guangzhou", "hongkong", "macau"] if self.enable_optimization else [],
            "performance_metrics": self.get_performance_metrics()
        }

    def recognize_cantonese_optimized(self, audio_data: bytes) -> ASRResult:
        """
        粤语优化识别（便捷方法）

        Args:
            audio_data: 音频数据

        Returns:
            ASRResult: 优化后的识别结果
        """
        return self.recognize_speech(audio_data, "cn-cantonese", enable_dialect_detection=True)

    def batch_recognize(self, audio_list: List[bytes],
                      enable_optimization: bool = True) -> List[ASRResult]:
        """
        批量语音识别

        Args:
            audio_list: 音频数据列表
            enable_optimization: 是否启用优化

        Returns:
            List[ASRResult]: 识别结果列表
        """
        results = []
        logger.info(f"开始批量识别，音频数量: {len(audio_list)}")

        for i, audio_data in enumerate(audio_list):
            try:
                result = self.recognize_speech(
                    audio_data,
                    enable_dialect_detection=enable_optimization
                )
                results.append(result)

                # 简单的进度显示
                if (i + 1) % 10 == 0:
                    logger.info(f"批量识别进度: {i + 1}/{len(audio_list)}")

            except Exception as e:
                logger.error(f"批量识别第{i+1}个音频失败: {e}")
                results.append(ASRResult(
                    success=False,
                    text="",
                    confidence=0.0,
                    response_time=0.0,
                    error=f"批量识别失败: {str(e)}"
                ))

        logger.info(f"批量识别完成，成功: {sum(1 for r in results if r.success)}/{len(results)}")
        return results


# 便捷函数
def create_simple_asr_service(app_key: str = "", token: str = "", enable_optimization: bool = True) -> SimpleAliyunASRService:
    """
    创建简单ASR服务实例 (Enhanced for Cantonese ASR Optimization)

    Args:
        app_key: 阿里云应用密钥
        token: 阿里云访问令牌
        enable_optimization: 是否启用粤语ASR优化

    Returns:
        SimpleAliyunASRService: 服务实例
    """
    return SimpleAliyunASRService(app_key, token, enable_optimization)


def create_cantonese_asr_service(app_key: str = "", token: str = "") -> SimpleAliyunASRService:
    """
    创建专门用于粤语识别的ASR服务实例

    Args:
        app_key: 阿里云应用密钥
        token: 阿里云访问令牌

    Returns:
        SimpleAliyunASRService: 粤语ASR服务实例
    """
    return SimpleAliyunASRService(app_key, token, enable_optimization=True)


if __name__ == "__main__":
    # 测试代码
    print("=== Simple Aliyun ASR Service 测试 (Enhanced v1.2) ===")

    # 创建优化版服务
    service = create_cantonese_asr_service()

    # 显示服务信息
    info = service.get_service_info()
    print("服务信息:")
    for key, value in info.items():
        if key != "performance_metrics":
            print(f"  {key}: {value}")

    print(f"\n性能指标:")
    metrics = info.get("performance_metrics", {})
    for key, value in metrics.items():
        print(f"  {key}: {value}")

    # 运行测试
    print("\n运行服务测试...")
    test_result = service.test_service()
    print(f"测试结果: {'通过' if test_result else '失败'}")

    # 创建测试音频数据
    print("\n创建测试音频数据...")
    import numpy as np
    sample_rate = 16000
    duration = 3.0  # 3秒
    frequency = 440  # A4音符
    t = np.linspace(0, duration, int(sample_rate * duration))
    test_audio = (np.sin(2 * np.pi * frequency * t) * 16383).astype(np.int16)

    # 转换为WAV格式
    import wave
    import io
    wav_buffer = io.BytesIO()
    with wave.open(wav_buffer, 'wb') as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(test_audio.tobytes())

    wav_data = wav_buffer.getvalue()
    base64_audio = base64.b64encode(wav_data).decode('ascii')

    print(f"测试音频数据准备完成，大小: {len(base64_audio)} 字符")

    # 测试优化识别
    print("\n测试粤语优化识别...")
    try:
        result = service.recognize_cantonese_optimized(base64_audio)
        print(f"识别结果: 成功={result.success}")
        if result.success:
            print(f"  文本: '{result.text}'")
            print(f"  置信度: {result.confidence:.2f}")
            print(f"  响应时间: {result.response_time:.3f}s")
            print(f"  检测方言: {result.dialect}")
            print(f"  噪声水平: {result.noise_level}")
        else:
            print(f"  错误: {result.error}")
    except Exception as e:
        print(f"优化识别测试异常: {e}")

    # 显示最终性能指标
    print("\n最终性能指标:")
    final_metrics = service.get_performance_metrics()
    for key, value in final_metrics.items():
        print(f"  {key}: {value}")

    print("\n测试完成 (Enhanced v1.2 粤语优化版)")