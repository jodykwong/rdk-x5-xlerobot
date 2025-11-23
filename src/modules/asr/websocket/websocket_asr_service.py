#!/usr/bin/env python3.10
# -*- coding: utf-8 -*-
"""
阿里云ASR WebSocket服务 - XLeBot语音识别模块
基于阿里云智能语音服务(NLS) WebSocket API
严格遵循架构文档：必须使用WebSocket，不支持HTTP REST API
"""

import logging
import json
import os
import time
import sys
import wave
import numpy as np
import threading
import queue
import tempfile
from typing import Dict, Any, Optional, Callable, List
import asyncio
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

# 添加阿里云NLS SDK路径
sys.path.append('/home/sunrise/.local/lib/python3.10/site-packages')

try:
    from nls.token import getToken
    from nls.speech_recognizer import NlsSpeechRecognizer
    HAS_NLS_SDK = True
    logging.info("✅ 阿里云NLS SDK已加载")
except ImportError as e:
    HAS_NLS_SDK = False
    logging.warning(f"⚠️ 阿里云NLS SDK未安装: {e}")
    logging.info("请运行: pip3.10 install aliyun-nls-python-sdk")

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class AliyunASRWebSocketService:
    """阿里云ASR WebSocket服务 - 严格遵循架构文档规范"""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        初始化阿里云ASR WebSocket服务

        Args:
            config: 配置字典，包含API密钥等信息
        """
        self.config = config or {}
        self.logger = logging.getLogger(__name__)

        # 阿里云NLS配置
        self.access_key_id = os.getenv('ALIBABA_CLOUD_ACCESS_KEY_ID',
                                      self.config.get('access_key_id', ''))
        self.access_key_secret = os.getenv('ALIBABA_CLOUD_ACCESS_KEY_SECRET',
                                         self.config.get('access_key_secret', ''))
        self.app_key = os.getenv('ALIYUN_NLS_APPKEY',
                                self.config.get('app_key', 'YOUR_NLS_APPKEY'))

        # WebSocket配置（根据架构文档）
        self.endpoint = "wss://nls-gateway.cn-shanghai.aliyuncs.com/ws/v1"
        self.language = "cn-cantonese"  # 粤语
        self.format = "pcm"
        self.sample_rate = 16000
        self.enable_intermediate_result = True
        self.enable_punctuation_prediction = True
        self.enable_inverse_text_normalization = True

        # Token管理
        self.token = None
        self.token_expiry = 0
        self.recognizer = None

        # 识别结果
        self.final_result = ""
        self.intermediate_results = []
        self.recognition_complete = threading.Event()
        self.recognition_success = False
        self.error_message = ""

        # 初始化Token（延迟到实际使用时）
        self.token_initialized = False

        self.logger.info("✅ 阿里云ASR WebSocket服务初始化完成")
        self.logger.info(f"  - 端点: {self.endpoint}")
        self.logger.info(f"  - 语言: {self.language}")
        self.logger.info(f"  - 采样率: {self.sample_rate}Hz")
        self.logger.info(f"  - SDK状态: {'已加载' if HAS_NLS_SDK else '未安装'}")

    def _refresh_token(self) -> bool:
        """
        刷新Token

        Returns:
            是否成功获取Token
        """
        if not HAS_NLS_SDK:
            self.logger.error("❌ 阿里云NLS SDK未安装，无法获取Token")
            return False

        try:
            if not self.access_key_id or not self.access_key_secret:
                self.logger.error("❌ 缺少AccessKey ID或Secret")
                return False

            self.token = getToken(self.access_key_id, self.access_key_secret)
            self.token_expiry = time.time() + 23.5 * 3600  # 23.5小时后过期

            if self.token:
                self.logger.info(f"✅ Token获取成功: {self.token[:20]}...")
                return True
            else:
                self.logger.error("❌ Token获取失败，返回空值")
                return False

        except Exception as e:
            self.logger.error(f"❌ Token获取失败: {e}")
            return False

    def _check_token_validity(self) -> bool:
        """
        检查Token是否有效

        Returns:
            Token是否有效
        """
        if not self.token:
            return False
        return time.time() < self.token_expiry

    def _ensure_valid_token(self) -> bool:
        """
        确保有有效的Token

        Returns:
            是否成功获取有效Token
        """
        try:
            # 检查API密钥是否存在
            if not self.access_key_id or not self.access_key_secret:
                self.logger.error("❌ 阿里云API密钥未配置，请检查环境变量:")
                self.logger.error("   - ALIBABA_CLOUD_ACCESS_KEY_ID")
                self.logger.error("   - ALIBABA_CLOUD_ACCESS_KEY_SECRET")
                return False

            # 检查Token有效性
            if not self._check_token_validity():
                return self._refresh_token()
            return True

        except Exception as e:
            self.logger.error(f"❌ Token验证异常: {e}")
            return False

    def _on_recognition_started(self, message, *args):
        """识别开始回调"""
        self.logger.info("🎤 ASR识别开始")

    def _on_result_changed(self, message, *args):
        """
        中间结果回调 - 实时识别

        Args:
            message: 中间结果消息
        """
        try:
            result = json.loads(message) if isinstance(message, str) else message
            if 'payload' in result and 'result' in result['payload']:
                text = result['payload']['result']
                self.intermediate_results.append(text)
                self.logger.debug(f"🔄 中间结果: {text}")
        except Exception as e:
            self.logger.debug(f"中间结果解析失败: {e}")

    def _on_recognition_completed(self, message, *args):
        """
        识别完成回调 - 最终结果

        Args:
            message: 识别结果消息
        """
        try:
            result = json.loads(message) if isinstance(message, str) else message
            if 'payload' in result and 'result' in result['payload']:
                self.final_result = result['payload']['result']

                # ✅ 检查结果是否为空（防止幻觉）
                if not self.final_result or not self.final_result.strip():
                    self.logger.warning("⚠️ ASR API返回空结果，标记为识别失败")
                    self.recognition_success = False
                    return

                # 🔍 调试：记录完整的API响应结构
                self.logger.info(f"📊 API响应结构: {json.dumps(result, indent=2, ensure_ascii=False)}")
                self.logger.info(f"✅ 最终结果: '{self.final_result}'")

                self.recognition_success = True
                self.logger.info(f"✅ ASR识别成功")
            else:
                self.logger.error("❌ 识别完成回调中无结果")
                self.recognition_success = False
        except Exception as e:
            self.logger.error(f"❌ 最终结果解析失败: {e}")
            self.recognition_success = False
        finally:
            self.recognition_complete.set()

    def _on_recognition_error(self, message, *args):
        """
        识别错误回调

        Args:
            message: 错误消息
        """
        self.logger.error(f"❌ ASR识别错误: {message}")
        self.error_message = str(message)
        self.recognition_success = False
        self.recognition_complete.set()

    def _create_recognizer(self) -> Optional[NlsSpeechRecognizer]:
        """
        创建语音识别器

        Returns:
            NlsSpeechRecognizer实例或None
        """
        if not HAS_NLS_SDK:
            self.logger.error("❌ 阿里云NLS SDK未安装")
            return None

        if not self._ensure_valid_token():
            self.logger.error("❌ 无法获取有效Token")
            return None

        try:
            recognizer = NlsSpeechRecognizer(
                token=self.token,
                appkey=self.app_key,
                on_start=self._on_recognition_started,
                on_result_changed=self._on_result_changed,
                on_completed=self._on_recognition_completed,
                on_error=self._on_recognition_error
            )

            self.logger.info("✅ ASR WebSocket连接器创建成功")
            return recognizer

        except Exception as e:
            self.logger.error(f"❌ 创建ASR连接器失败: {e}")
            return None

    def _convert_audio_format(self, audio_data: bytes, source_sample_rate: int = 44100) -> bytes:
        """
        转换音频格式（44.1kHz -> 16kHz）

        Args:
            audio_data: 原始音频数据
            source_sample_rate: 源采样率

        Returns:
            转换后的音频数据
        """
        try:
            # 如果采样率已经是16kHz，直接返回
            if source_sample_rate == 16000:
                return audio_data

            # 读取音频数据
            with tempfile.NamedTemporaryFile(suffix='.wav') as temp_file:
                temp_file.write(audio_data)
                temp_file.flush()

                # 使用wave模块读取
                with wave.open(temp_file.name, 'rb') as wav_file:
                    frames = wav_file.readframes(-1)
                    channels = wav_file.getnchannels()
                    sample_width = wav_file.getsampwidth()

                # 转换为numpy数组
                audio_array = np.frombuffer(frames, dtype=np.int16)

                # 如果是立体声，转换为单声道
                if channels == 2:
                    audio_array = audio_array.reshape(-1, 2)
                    audio_array = audio_array.mean(axis=1).astype(np.int16)

                # 重采样到16kHz
                duration = len(audio_array) / source_sample_rate
                target_length = int(duration * 16000)
                indices = np.linspace(0, len(audio_array) - 1, target_length)
                resampled_array = np.interp(indices, np.arange(len(audio_array)), audio_array).astype(np.int16)

                return resampled_array.tobytes()

        except Exception as e:
            self.logger.error(f"❌ 音频格式转换失败: {e}")
            return audio_data  # 返回原始数据

    def recognize_audio(self, audio_data: bytes, **kwargs) -> Optional[str]:
        """
        识别音频 - WebSocket方式

        Args:
            audio_data: 音频数据
            **kwargs: 其他参数
                - language: 语言（默认cn-cantonese）
                - format: 音频格式（默认pcm）
                - sample_rate: 采样率（默认16000）
                - enable_intermediate_result: 是否启用中间结果（默认True）

        Returns:
            识别结果文本或None
        """
        if not audio_data:
            self.logger.warning("⚠️ 音频数据为空")
            return None

        # 重置状态
        self.final_result = ""
        self.intermediate_results = []
        self.recognition_complete.clear()
        self.recognition_success = False
        self.error_message = ""

        # 获取参数
        language = kwargs.get('language', self.language)
        format_type = kwargs.get('format', self.format)
        sample_rate = kwargs.get('sample_rate', self.sample_rate)
        enable_intermediate = kwargs.get('enable_intermediate_result', self.enable_intermediate_result)

        # 创建识别器
        recognizer = self._create_recognizer()
        if not recognizer:
            return None

        try:
            # 音频格式转换
            converted_audio = self._convert_audio_format(audio_data, sample_rate)

            self.logger.info(f"🎤 开始ASR识别: {len(converted_audio)} 字节")

            # 开始识别
            recognizer.start(
                aformat=format_type,
                sample_rate=16000,  # 固定16kHz
                enable_intermediate_result=enable_intermediate,
                enable_punctuation_prediction=self.enable_punctuation_prediction,
                enable_inverse_text_normalization=self.enable_inverse_text_normalization
            )

            # 发送音频数据 - 分块发送以避免超过64KB限制
            CHUNK_SIZE = 32000  # 32KB per chunk (safely under 64KB limit)
            total_bytes = len(converted_audio)
            chunks_sent = 0

            for i in range(0, total_bytes, CHUNK_SIZE):
                chunk = converted_audio[i:i+CHUNK_SIZE]
                recognizer.send_audio(chunk)
                chunks_sent += 1

            self.logger.info(f"📤 已发送 {chunks_sent} 个音频块 (总计 {total_bytes} 字节)")

            # 结束识别
            recognizer.stop()

            # 等待识别完成（最多10秒）
            if not self.recognition_complete.wait(timeout=10):
                self.logger.error("❌ ASR识别超时")
                return None

            if not self.recognition_success:
                self.logger.error(f"❌ ASR识别失败: {self.error_message}")
                return None

            # 关闭识别器
            recognizer.shutdown()

            result = self.final_result.strip()
            if result:
                self.logger.info(f"✅ ASR识别成功: '{result}'")
                return result
            else:
                self.logger.warning("⚠️ 识别结果为空")
                return None

        except Exception as e:
            self.logger.error(f"❌ ASR识别异常: {e}")
            try:
                recognizer.shutdown()
            except:
                pass
            return None

    def recognize_file(self, audio_file_path: str, **kwargs) -> Optional[str]:
        """
        识别音频文件

        Args:
            audio_file_path: 音频文件路径
            **kwargs: 识别参数

        Returns:
            识别结果或None
        """
        try:
            if not os.path.exists(audio_file_path):
                self.logger.error(f"❌ 音频文件不存在: {audio_file_path}")
                return None

            # 读取音频文件
            with open(audio_file_path, 'rb') as f:
                audio_data = f.read()

            self.logger.info(f"📁 读取音频文件: {audio_file_path} ({len(audio_data)} 字节)")

            return self.recognize_audio(audio_data, **kwargs)

        except Exception as e:
            self.logger.error(f"❌ 读取音频文件失败: {e}")
            return None

    def test_connection(self) -> bool:
        """
        测试WebSocket连接

        Returns:
            连接是否正常
        """
        # 创建一个空的音频数据进行测试
        test_audio = b'\x00' * 16000  # 1秒的静音
        result = self.recognize_audio(test_audio)
        return result is not None

    def get_service_info(self) -> Dict[str, Any]:
        """
        获取服务信息

        Returns:
            服务信息字典
        """
        return {
            'service_type': 'Aliyun ASR WebSocket',
            'endpoint': self.endpoint,
            'language': self.language,
            'format': self.format,
            'sample_rate': self.sample_rate,
            'sdk_available': HAS_NLS_SDK,
            'token_valid': self._check_token_validity(),
            'token_expiry': self.token_expiry,
            'app_key_configured': bool(self.app_key),
            'access_key_configured': bool(self.access_key_id and self.access_key_secret),
            'enable_intermediate_result': self.enable_intermediate_result,
            'enable_punctuation_prediction': self.enable_punctuation_prediction,
            'enable_inverse_text_normalization': self.enable_inverse_text_normalization
        }

    def __del__(self):
        """析构函数，清理资源"""
        try:
            if self.recognizer:
                self.recognizer.shutdown()
        except:
            pass

# 全局实例
_asr_service = None

def get_asr_service() -> AliyunASRWebSocketService:
    """
    获取全局ASR服务实例

    Returns:
        ASR服务实例
    """
    global _asr_service
    if _asr_service is None:
        _asr_service = AliyunASRWebSocketService()
    return _asr_service

def recognize_speech(audio_data: bytes, **kwargs) -> Optional[str]:
    """
    语音识别便捷函数

    Args:
        audio_data: 音频数据
        **kwargs: 识别参数

    Returns:
        识别结果或None
    """
    service = get_asr_service()
    return service.recognize_audio(audio_data, **kwargs)

def recognize_file(audio_file_path: str, **kwargs) -> Optional[str]:
    """
    语音文件识别便捷函数

    Args:
        audio_file_path: 音频文件路径
        **kwargs: 识别参数

    Returns:
        识别结果或None
    """
    service = get_asr_service()
    return service.recognize_file(audio_file_path, **kwargs)

# 向后兼容的别名
WebSocketASRService = AliyunASRWebSocketService
AliyunASRService = AliyunASRWebSocketService

if __name__ == "__main__":
    # 测试代码
    service = AliyunASRWebSocketService()

    print("服务信息:")
    print(json.dumps(service.get_service_info(), indent=2, ensure_ascii=False))

    # 测试连接
    print("\n测试连接...")
    if service.test_connection():
        print("✅ WebSocket连接测试成功")
    else:
        print("❌ WebSocket连接测试失败")