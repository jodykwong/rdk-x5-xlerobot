#!/usr/bin/env python3
"""
阿里云ASR服务模块

集成阿里云智能语音交互服务，支持粤语语音识别：
- 粤语Paraformer语音识别
- 实时语音转文字
- 多语言支持 (普通话/粤语)
- 网络重试和容错机制

作者: Dev Agent
日期: 2025-11-08
Epic: 1 - ASR语音识别模块
Story: 1.1 - 粤语语音识别基础功能
Phase: 3 - 基础语音识别
Task: 3.1 - 集成阿里云ASR粤语API服务
"""

import json
import logging
import time
import base64
import threading
import queue
from typing import Dict, List, Optional, Callable, Any
from dataclasses import dataclass, asdict
from enum import Enum
import requests
import numpy as np
import wave
import io

# 导入Token管理器
try:
    from modules.asr.aliyun_nls_token_manager import AliyunNLSTokenManager
except ImportError:
    # 尝试相对导入
    try:
        from .aliyun_nls_token_manager import AliyunNLSTokenManager
    except ImportError:
        # 如果都失败了，创建一个简单的占位符
        AliyunNLSTokenManager = None
        logging.warning("AliyunNLSTokenManager不可用，ASR服务将使用模拟Token")

logger = logging.getLogger(__name__)


class ASRFormat(Enum):
    """音频格式"""
    PCM = "pcm"
    WAV = "wav"
    OPUS = "opus"


class ASRLanguage(Enum):
    """支持的语言"""
    MANDARIN = "zh"      # 普通话
    CANTONESE = "cantonese" # 粤语
    ENGLISH = "en"        # 英语


@dataclass
class ASRConfig:
    """ASR配置"""
    app_key: str
    app_secret: str
    region: str = "cn-shanghai"
    language: ASRLanguage = ASRLanguage.CANTONESE
    format: ASRFormat = ASRFormat.PCM
    sample_rate: int = 16000
    enable_punctuation: bool = True
    enable_inverse_text_normalization: bool = True
    enable_voice_detection: bool = True


@dataclass
class ASRResult:
    """ASR识别结果"""
    text: str
    confidence: float
    begin_time: int
    end_time: int
    status_code: int
    message: str
    timestamp: float


class AliyunASRService:
    """
    阿里云ASR语音识别服务

    提供粤语语音识别功能，支持实时语音转文字
    """

    def __init__(self, config: Optional[ASRConfig] = None):
        """
        初始化ASR服务

        Args:
            config: ASR配置
        """
        self.config = config
        self.token_manager = AliyunNLSTokenManager() if AliyunNLSTokenManager else None

        # 网络配置
        self.session_timeout = 30  # 会话超时时间(秒)
        self.request_timeout = 10  # 请求超时时间(秒)
        self.max_retries = 3  # 最大重试次数
        self.retry_delay = 1.0  # 重试延迟(秒)

        # 请求头配置
        self.headers = {
            "Content-Type": "application/json",
            "Accept": "application/json",
            "User-Agent": "XleRobot-ASR-Service/1.0"
        }

        # 状态管理
        self.is_running = False
        self.current_session_id = None
        self.session_queue = queue.Queue()
        self.result_queue = queue.Queue()

        # 统计信息
        self.request_count = 0
        self.success_count = 0
        self.error_count = 0
        self.total_audio_duration = 0.0

        # 线程安全
        self._lock = threading.Lock()

        logger.info(f"AliyunASRService 初始化完成")
        logger.info(f"  语言: {self.config.language.value if self.config else 'N/A'}")
        logger.info(f"  采样率: {self.config.sample_rate if self.config else 'N/A'}Hz")
        logger.info(f"  区域: {self.config.region if self.config else 'N/A'}")

    def get_access_token(self) -> Optional[str]:
        """
        获取访问令牌

        Returns:
            str: 访问令牌，如果获取失败返回None
        """
        if self.token_manager:
            return self.token_manager.get_token()
        else:
            # 模拟Token（用于测试）
            return "mock_token_for_testing"

    def start_recognition_session(self) -> Optional[str]:
        """
        开始语音识别会话

        Returns:
            str: 会话ID，如果失败返回None
        """
        try:
            # 获取访问令牌
            token = self.get_access_token()
            if not token:
                logger.error("获取访问令牌失败")
                return None

            # 创建会话请求
            url = f"https://nls-gateway-{self.config.region}.aliyuncs.com/stream/v1/asr"

            payload = {
                "appkey": self.config.app_key,
                "token": token,
                "format": self.config.format.value,
                "sample_rate": self.config.sample_rate,
                "enable_intermediate_result": True,
                "enable_punctuation_prediction": self.config.enable_punctuation,
                "enable_inverse_text_normalization": self.config.enable_inverse_text_normalization,
                "enable_voice_detection": self.config.enable_voice_detection
            }

            # 发送请求
            response = self._make_request(url, payload)

            if response and response.get("status_code") == 200:
                session_id = response.get("session_id")
                if session_id:
                    with self._lock:
                        self.current_session_id = session_id
                        self.is_running = True

                    logger.info(f"✅ ASR会话开始: {session_id}")
                    return session_id

            logger.error(f"创建ASR会话失败: {response}")
            return None

        except Exception as e:
            logger.error(f"开始语音识别会话失败: {e}")
            return None

    def stop_recognition_session(self) -> bool:
        """
        停止语音识别会话

        Returns:
            bool: 是否成功停止
        """
        try:
            with self._lock:
                if not self.is_running or not self.current_session_id:
                    return True

                session_id = self.current_session_id

            # 发送停止请求
            url = f"https://nls-gateway-{self.config.region}.aliyuncs.com/stream/v1/asr/{session_id}"
            payload = {"action": "stop"}

            response = self._make_request(url, payload)

            with self._lock:
                self.is_running = False
                self.current_session_id = None

            logger.info(f"✅ ASR会话停止: {session_id}")
            return True

        except Exception as e:
            logger.error(f"停止语音识别会话失败: {e}")
            return False

    def recognize_audio(self, audio_data: np.ndarray, sample_rate: Optional[int] = None) -> Optional[ASRResult]:
        """
        识别音频数据

        Args:
            audio_data: 音频数据
            sample_rate: 采样率

        Returns:
            ASRResult: 识别结果
        """
        try:
            # 检查会话状态
            if not self.is_running:
                logger.warning("ASR会话未启动，尝试启动会话")
                if not self.start_recognition_session():
                    return None

            # 转换音频数据
            audio_base64 = self._convert_audio_to_base64(audio_data, sample_rate)

            # 创建识别请求
            url = f"https://nls-gateway-{self.config.region}.aliyuncs.com/stream/v1/asr/{self.current_session_id}"

            payload = {
                "action": "run",
                "audio": audio_base64,
                "format": self.config.format.value,
                "sample_rate": self.config.sample_rate
            }

            # 发送请求
            response = self._make_request(url, payload)

            if response:
                # 解析响应
                result = self._parse_asr_response(response)

                # 更新统计
                with self._lock:
                    self.request_count += 1
                    if result.status_code == 200:
                        self.success_count += 1
                    else:
                        self.error_count += 1

                return result

            return None

        except Exception as e:
            logger.error(f"音频识别失败: {e}")
            with self._lock:
                self.error_count += 1
            return None

    def _convert_audio_to_base64(self, audio_data: np.ndarray, sample_rate: Optional[int] = None) -> str:
        """
        将音频数据转换为base64格式

        Args:
            audio_data: 音频数据
            sample_rate: 采样率

        Returns:
            str: base64编码的音频数据
        """
        try:
            # 使用实际采样率或配置的采样率
            actual_sample_rate = sample_rate or self.config.sample_rate

            # 创建内存中的WAV文件
            buffer = io.BytesIO()

            with wave.open(buffer, 'wb') as wav_file:
                wav_file.setnchannels(1)  # 单声道
                wav_file.setsampwidth(2)  # 16-bit
                wav_file.setframerate(actual_sample_rate)
                wav_file.writeframes(audio_data.astype(np.int16).tobytes())

            # 转换为base64
            buffer.seek(0)
            audio_base64 = base64.b64encode(buffer.read()).decode('utf-8')

            return audio_base64

        except Exception as e:
            logger.error(f"音频数据转换失败: {e}")
            return ""

    def _make_request(self, url: str, payload: Dict) -> Optional[Dict]:
        """
        发送HTTP请求

        Args:
            url: 请求URL
            payload: 请求载荷

        Returns:
            Dict: 响应数据
        """
        for attempt in range(self.max_retries):
            try:
                # 添加访问令牌到请求头
                headers = self.headers.copy()
                token = self.get_access_token()
                if token:
                    headers["X-NLS-Token"] = token

                # 发送请求
                response = requests.post(
                    url,
                    json=payload,
                    headers=headers,
                    timeout=self.request_timeout
                )

                if response.status_code == 200:
                    return response.json()
                else:
                    logger.warning(f"请求失败 (尝试 {attempt + 1}/{self.max_retries}): "
                               f"状态码 {response.status_code}, 响应: {response.text}")

            except requests.exceptions.RequestException as e:
                logger.warning(f"请求异常 (尝试 {attempt + 1}/{self.max_retries}): {e}")

            # 重试前等待
            if attempt < self.max_retries - 1:
                time.sleep(self.retry_delay * (2 ** attempt))

        logger.error(f"请求最终失败，已重试 {self.max_retries} 次")
        return None

    def _parse_asr_response(self, response: Dict) -> ASRResult:
        """
        解析ASR响应

        Args:
            response: 响应数据

        Returns:
            ASRResult: 识别结果
        """
        try:
            # 提取识别结果
            result = response.get("result", {})
            text = result.get("text", "")

            # 计算置信度
            confidence = 0.0
            if "sentences" in result and result["sentences"]:
                sentences = result["sentences"]
                if sentences:
                    confidence = sentences[0].get("confidence", 0.0) / 100.0

            # 时间信息
            begin_time = result.get("begin_time", 0)
            end_time = result.get("end_time", 0)

            # 状态信息
            status_code = response.get("status_code", 0)
            message = response.get("message", "")

            asr_result = ASRResult(
                text=text,
                confidence=confidence,
                begin_time=begin_time,
                end_time=end_time,
                status_code=status_code,
                message=message,
                timestamp=time.time()
            )

            logger.debug(f"ASR识别结果: '{text}' (置信度: {confidence:.2f})")
            return asr_result

        except Exception as e:
            logger.error(f"解析ASR响应失败: {e}")
            return ASRResult(
                text="",
                confidence=0.0,
                begin_time=0,
                end_time=0,
                status_code=500,
                message=f"解析失败: {str(e)}",
                timestamp=time.time()
            )

    def continuous_recognize(self,
                           audio_callback: Callable[[], Optional[np.ndarray]] = None,
                           result_callback: Callable[[ASRResult], None] = None) -> None:
        """
        连续语音识别

        Args:
            audio_callback: 音频数据回调函数
            result_callback: 识别结果回调函数
        """
        try:
            # 启动会话
            if not self.start_recognition_session():
                logger.error("无法启动连续识别会话")
                return

            logger.info("开始连续语音识别...")

            while self.is_running:
                # 获取音频数据
                if audio_callback:
                    audio_data = audio_callback()
                    if audio_data is not None:
                        # 识别音频
                        result = self.recognize_audio(audio_data)

                        if result and result_callback:
                            result_callback(result)

                # 短暂休眠避免CPU占用过高
                time.sleep(0.1)

            # 停止会话
            self.stop_recognition_session()
            logger.info("连续语音识别结束")

        except Exception as e:
            logger.error(f"连续语音识别失败: {e}")
            self.stop_recognition_session()

    def get_statistics(self) -> Dict[str, Any]:
        """
        获取服务统计信息

        Returns:
            Dict: 统计信息
        """
        with self._lock:
            success_rate = (self.success_count / self.request_count
                          if self.request_count > 0 else 0.0)

            return {
                "request_count": self.request_count,
                "success_count": self.success_count,
                "error_count": self.error_count,
                "success_rate": success_rate,
                "total_audio_duration": self.total_audio_duration,
                "is_running": self.is_running,
                "current_session_id": self.current_session_id,
                "language": self.config.language.value if self.config else None,
                "sample_rate": self.config.sample_rate if self.config else None
            }

    def reset_statistics(self) -> None:
        """重置统计信息"""
        with self._lock:
            self.request_count = 0
            self.success_count = 0
            self.error_count = 0
            self.total_audio_duration = 0.0
        logger.info("ASR服务统计信息已重置")

    def test_connection(self) -> bool:
        """
        测试服务连接

        Returns:
            bool: 连接是否正常
        """
        try:
            # 测试Token获取
            token = self.get_access_token()
            if not token:
                logger.error("Token获取失败")
                return False

            # 测试会话创建和关闭
            session_id = self.start_recognition_session()
            if not session_id:
                logger.error("会话创建失败")
                return False

            # 等待一小段时间
            time.sleep(0.5)

            success = self.stop_recognition_session()
            if not success:
                logger.error("会话停止失败")
                return False

            logger.info("✅ ASR服务连接测试通过")
            return True

        except Exception as e:
            logger.error(f"ASR服务连接测试失败: {e}")
            return False


# 工厂函数
def create_aliyun_asr_service(app_key: str,
                               app_secret: str,
                               language: ASRLanguage = ASRLanguage.CANTONESE,
                               **kwargs) -> AliyunASRService:
    """
    创建阿里云ASR服务的工厂函数

    Args:
        app_key: 应用密钥
        app_secret: 应用密钥
        language: 识别语言
        **kwargs: 其他配置参数

    Returns:
        AliyunASRService: ASR服务实例
    """
    config = ASRConfig(
        app_key=app_key,
        app_secret=app_secret,
        language=language,
        **kwargs
    )

    return AliyunASRService(config)


if __name__ == "__main__":
    # 测试代码
    logging.basicConfig(level=logging.INFO)

    print("🎤 阿里云ASR服务测试")
    print("=" * 50)

    # 创建配置（使用环境变量或默认值）
    import os
    app_key = os.getenv("ALIYUN_NLS_APP_KEY", "test_app_key")
    app_secret = os.getenv("ALIYUN_NLS_APP_SECRET", "test_app_secret")

    # 创建ASR服务
    service = create_aliyun_asr_service(
        app_key=app_key,
        app_secret=app_secret,
        language=ASRLanguage.CANTONESE
    )

    # 测试连接
    print(f"\n🔗 测试服务连接...")
    connection_ok = service.test_connection()
    print(f"连接状态: {'✅ 正常' if connection_ok else '❌ 异常'}")

    # 获取统计信息
    stats = service.get_statistics()
    print(f"\n📊 服务统计:")
    for key, value in stats.items():
        print(f"  {key}: {value}")

    print("✅ 测试完成")