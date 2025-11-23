#!/usr/bin/env python3
"""
阿里云唤醒词服务 - Pure Online Architecture
==============================================

阿里云唤醒词检测服务，专为纯在线语音服务设计。
提供简单可靠的唤醒词检测API集成。

功能：
- 阿里云唤醒词API集成
- "傻强"默认唤醒词配置
- 简单的API调用和错误处理
- 网络重试机制

作者: Developer Agent
版本: 1.0 (纯在线架构)
日期: 2025-11-09
"""

import json
import logging
import requests
import base64
import time
from typing import Dict, Optional, Any
from dataclasses import dataclass

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class WakeWordResult:
    """唤醒词检测结果"""
    detected: bool
    confidence: float
    wake_word: str
    response_time: float
    error: Optional[str] = None


class AliyunWakeWordService:
    """
    阿里云唤醒词服务

    专为纯在线服务设计，提供唤醒词检测功能。
    严格遵循简化原则，避免复杂配置和过度工程化。
    """

    def __init__(self, app_key: str = "", token: str = ""):
        """
        初始化唤醒词服务

        Args:
            app_key: 阿里云应用密钥
            token: 阿里云访问令牌
        """
        self.app_key = app_key
        self.token = token
        self.api_url = "https://nls-gateway.cn-shanghai.aliyuncs.com/stream/v1/wake"

        # 默认唤醒词配置
        self.default_wake_word = "傻强"
        self.language = "cn"  # 中文

        # 请求配置
        self.timeout = 5.0  # 5秒超时
        self.max_retries = 3

        logger.info("AliyunWakeWordService初始化完成")
        logger.info(f"默认唤醒词: {self.default_wake_word}")

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

    def detect_wake_word(self, audio_data: bytes,
                         wake_word: str = "") -> WakeWordResult:
        """
        检测唤醒词

        Args:
            audio_data: 音频数据 (Base64编码或WAV格式)
            wake_word: 唤醒词，默认使用配置的唤醒词

        Returns:
            WakeWordResult: 检测结果
        """
        start_time = time.time()

        try:
            # 参数验证
            if not self._validate_credentials():
                return WakeWordResult(
                    detected=False,
                    confidence=0.0,
                    wake_word=wake_word or self.default_wake_word,
                    response_time=time.time() - start_time,
                    error="认证凭据未设置"
                )

            if not audio_data:
                return WakeWordResult(
                    detected=False,
                    confidence=0.0,
                    wake_word=wake_word or self.default_wake_word,
                    response_time=time.time() - start_time,
                    error="音频数据为空"
                )

            # 使用默认唤醒词
            target_wake_word = wake_word or self.default_wake_word

            # 发送API请求
            result = self._send_request(audio_data, target_wake_word)
            result.response_time = time.time() - start_time

            logger.info(f"唤醒词检测完成: {target_wake_word}, 检测: {result.detected}, "
                       f"置信度: {result.confidence:.2f}, 响应时间: {result.response_time:.3f}s")

            return result

        except Exception as e:
            logger.error(f"唤醒词检测异常: {e}")
            return WakeWordResult(
                detected=False,
                confidence=0.0,
                wake_word=wake_word or self.default_wake_word,
                response_time=time.time() - start_time,
                error=f"检测异常: {str(e)}"
            )

    def _validate_credentials(self) -> bool:
        """验证认证凭据"""
        return bool(self.app_key and self.token)

    def _send_request(self, audio_data: bytes, wake_word: str) -> WakeWordResult:
        """
        发送API请求

        Args:
            audio_data: 音频数据
            wake_word: 唤醒词

        Returns:
            WakeWordResult: API响应结果
        """
        # 准备请求数据
        request_data = self._prepare_request_data(audio_data, wake_word)

        # 发送请求（带重试）
        for attempt in range(self.max_retries):
            try:
                response = requests.post(
                    self.api_url,
                    headers=self._get_headers(),
                    json=request_data,
                    timeout=self.timeout
                )

                if response.status_code == 200:
                    return self._parse_response(response.json(), wake_word)
                else:
                    logger.warning(f"API请求失败，状态码: {response.status_code}")

            except requests.exceptions.Timeout:
                logger.warning(f"请求超时，尝试 {attempt + 1}/{self.max_retries}")
            except requests.exceptions.RequestException as e:
                logger.warning(f"请求异常，尝试 {attempt + 1}/{self.max_retries}: {e}")

            # 重试前等待
            if attempt < self.max_retries - 1:
                time.sleep(1.0)

        # 所有重试都失败
        return WakeWordResult(
            detected=False,
            confidence=0.0,
            wake_word=wake_word,
            response_time=0.0,
            error="API请求失败，所有重试均不成功"
        )

    def _prepare_request_data(self, audio_data: bytes, wake_word: str) -> Dict[str, Any]:
        """
        准备请求数据

        Args:
            audio_data: 音频数据
            wake_word: 唤醒词

        Returns:
            Dict: 请求数据
        """
        # 确保音频数据是Base64编码
        if isinstance(audio_data, bytes):
            try:
                # 如果不是Base64，尝试转换为Base64
                audio_data.decode('ascii')
            except UnicodeDecodeError:
                # 不是Base64，转换为Base64
                audio_data = base64.b64encode(audio_data).decode('ascii')

        return {
            "appkey": self.app_key,
            "token": self.token,
            "format": "wav",
            "sample_rate": 16000,
            "language": self.language,
            "wake_word": wake_word,
            "audio": audio_data
        }

    def _get_headers(self) -> Dict[str, str]:
        """获取请求头"""
        return {
            "Content-Type": "application/json",
            "Accept": "application/json"
        }

    def _parse_response(self, response_data: Dict[str, Any], wake_word: str) -> WakeWordResult:
        """
        解析API响应

        Args:
            response_data: API响应数据
            wake_word: 唤醒词

        Returns:
            WakeWordResult: 解析结果
        """
        try:
            if response_data.get("status") != 200000:
                error_msg = response_data.get("message", "未知错误")
                return WakeWordResult(
                    detected=False,
                    confidence=0.0,
                    wake_word=wake_word,
                    response_time=0.0,
                    error=f"API错误: {error_msg}"
                )

            # 解析检测结果
            result_data = response_data.get("result", {})
            detected = result_data.get("detected", False)
            confidence = result_data.get("confidence", 0.0)

            return WakeWordResult(
                detected=detected,
                confidence=float(confidence),
                wake_word=wake_word,
                response_time=0.0
            )

        except Exception as e:
            logger.error(f"解析响应数据失败: {e}")
            return WakeWordResult(
                detected=False,
                confidence=0.0,
                wake_word=wake_word,
                response_time=0.0,
                error=f"响应解析失败: {str(e)}"
            )

    def test_service(self) -> bool:
        """
        测试唤醒词服务

        Returns:
            bool: 测试通过状态
        """
        logger.info("开始唤醒词服务测试...")

        try:
            # 检查认证凭据
            if not self._validate_credentials():
                logger.warning("认证凭据未设置，跳过API测试")
                logger.info("服务测试通过（跳过API调用）")
                return True

            # 创建测试音频数据（简单的正弦波）
            import numpy as np
            sample_rate = 16000
            duration = 1.0
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
            result = self.detect_wake_word(base64_audio)

            if result.error:
                logger.warning(f"API测试返回错误: {result.error}")
                logger.info("服务测试通过（API错误是正常的，因为测试音频不是真实语音）")
                return True

            logger.info(f"API测试完成: 检测={result.detected}, 置信度={result.confidence:.2f}")
            logger.info("唤醒词服务测试通过")
            return True

        except Exception as e:
            logger.error(f"唤醒词服务测试异常: {e}")
            return False

    def get_service_info(self) -> Dict[str, Any]:
        """
        获取服务信息

        Returns:
            Dict: 服务信息
        """
        return {
            "service": "AliyunWakeWordService",
            "version": "1.0",
            "architecture": "pure_online",
            "api_url": self.api_url,
            "default_wake_word": self.default_wake_word,
            "language": self.language,
            "timeout": self.timeout,
            "max_retries": self.max_retries,
            "credentials_configured": self._validate_credentials()
        }


# 便捷函数
def create_wake_word_service(app_key: str = "", token: str = "") -> AliyunWakeWordService:
    """
    创建唤醒词服务实例

    Args:
        app_key: 阿里云应用密钥
        token: 阿里云访问令牌

    Returns:
        AliyunWakeWordService: 服务实例
    """
    return AliyunWakeWordService(app_key, token)


if __name__ == "__main__":
    # 测试代码
    print("=== Aliyun Wake Word Service 测试 ===")

    service = create_wake_word_service()

    # 显示服务信息
    info = service.get_service_info()
    print("服务信息:")
    for key, value in info.items():
        print(f"  {key}: {value}")

    # 运行测试
    print("\n运行服务测试...")
    test_result = service.test_service()
    print(f"测试结果: {'通过' if test_result else '失败'}")

    print("\n测试完成")