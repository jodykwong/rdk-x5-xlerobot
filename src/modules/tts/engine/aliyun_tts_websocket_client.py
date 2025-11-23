#!/usr/bin/env python3.10
# -*- coding: utf-8 -*-
"""
阿里云TTS WebSocket服务 - XLeBot语音合成模块
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
import base64
from typing import Dict, Any, Optional, Callable, List
import asyncio
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

# 添加阿里云NLS SDK路径
sys.path.append('/home/sunrise/.local/lib/python3.10/site-packages')

try:
    from nls.token import getToken
    from nls.speech_synthesizer import NlsSpeechSynthesizer
    HAS_NLS_SDK = True
    logging.info("✅ 阿里云NLS SDK已加载")
except ImportError as e:
    HAS_NLS_SDK = False
    logging.warning(f"⚠️ 阿里云NLS SDK未安装: {e}")
    logging.info("请运行: pip3.10 install aliyun-nls-python-sdk")

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class AliyunTTSWebSocketService:
    """阿里云TTS WebSocket服务 - 严格遵循架构文档规范"""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        初始化阿里云TTS WebSocket服务

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
        self.voice = "xiaoyun"  # 默认粤语女声
        self.format = "wav"
        self.sample_rate = 16000
        self.volume = 50
        self.speech_rate = 0
        self.pitch_rate = 0

        # 粤语音色映射
        self.cantonese_voices = {
            'xiaoxiao': '晓晓（标准女声）',
            'xiaoyun': '晓云（知性女声）',
            'xiaoyi': '晓伊（温柔女声）',
            'xiaoming': '晓峰（稳重男声）',
        }

        # Token管理
        self.token = None
        self.token_expiry = 0
        self.synthesizer = None

        # 合成结果
        self.audio_data = None
        self.synthesis_complete = threading.Event()
        self.synthesis_success = False
        self.error_message = ""

        # 初始化Token
        self._refresh_token()

        self.logger.info("✅ 阿里云TTS WebSocket服务初始化完成")
        self.logger.info(f"  - 端点: {self.endpoint}")
        self.logger.info(f"  - 默认音色: {self.voice}")
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
        if not self._check_token_validity():
            return self._refresh_token()
        return True

    def _on_synthesis_started(self, message, *args):
        """合成开始回调"""
        self.logger.info("🎵 TTS合成开始")

    def _on_audio_data_received(self, message, *args):
        """
        音频数据接收回调

        Args:
            message: 音频数据消息
        """
        try:
            # 解析音频数据
            result = json.loads(message) if isinstance(message, str) else message
            if 'payload' in result and 'audio' in result['payload']:
                audio_chunk = base64.b64decode(result['payload']['audio'])
                if self.audio_data is None:
                    self.audio_data = audio_chunk
                else:
                    self.audio_data += audio_chunk
                self.logger.debug(f"📥 接收到音频块: {len(audio_chunk)} 字节")
        except Exception as e:
            self.logger.debug(f"音频数据解析失败: {e}")

    def _on_synthesis_completed(self, message, *args):
        """
        合成完成回调

        Args:
            message: 合成完成消息
        """
        try:
            result = json.loads(message) if isinstance(message, str) else message
            if 'payload' in result and 'status' in result['payload']:
                status = result['payload']['status']
                if status == 20000000:  # 成功状态码
                    self.logger.info("✅ TTS合成完成")
                    self.synthesis_success = True
                else:
                    self.logger.error(f"❌ TTS合成失败，状态码: {status}")
                    self.synthesis_success = False
            else:
                self.logger.error("❌ 合成完成回调中无状态信息")
                self.synthesis_success = False
        except Exception as e:
            self.logger.error(f"❌ 合成完成回调解析失败: {e}")
            self.synthesis_success = False
        finally:
            self.synthesis_complete.set()

    def _on_synthesis_error(self, message, *args):
        """
        合成错误回调

        Args:
            message: 错误消息
        """
        self.logger.error(f"❌ TTS合成错误: {message}")
        self.error_message = str(message)
        self.synthesis_success = False
        self.synthesis_complete.set()

    def _create_synthesizer(self) -> Optional[NlsSpeechSynthesizer]:
        """
        创建语音合成器

        Returns:
            NlsSpeechSynthesizer实例或None
        """
        if not HAS_NLS_SDK:
            self.logger.error("❌ 阿里云NLS SDK未安装")
            return None

        if not self._ensure_valid_token():
            self.logger.error("❌ 无法获取有效Token")
            return None

        try:
            synthesizer = NlsSpeechSynthesizer(
                url=self.endpoint,
                token=self.token,
                appkey=self.app_key,
                on_metainfo=self._on_synthesis_started,
                on_data=self._on_audio_data_received,
                on_completed=self._on_synthesis_completed,
                on_error=self._on_synthesis_error
            )

            self.logger.info("✅ TTS WebSocket连接器创建成功")
            return synthesizer

        except Exception as e:
            self.logger.error(f"❌ 创建TTS连接器失败: {e}")
            return None

    def synthesize_speech(self, text: str, **kwargs) -> Optional[bytes]:
        """
        语音合成 - WebSocket方式

        Args:
            text: 待合成文本
            **kwargs: 其他参数
                - voice: 音色（默认xiaoyun）
                - format: 音频格式（默认wav）
                - sample_rate: 采样率（默认16000）
                - volume: 音量（默认50）
                - speech_rate: 语速（默认0）
                - pitch_rate: 音调（默认0）

        Returns:
            音频数据或None
        """
        if not text or not text.strip():
            self.logger.warning("⚠️ 合成文本为空")
            return None

        # 重置状态
        self.audio_data = None
        self.synthesis_complete.clear()
        self.synthesis_success = False
        self.error_message = ""

        # 获取参数
        voice = kwargs.get('voice', self.voice)
        format_type = kwargs.get('format', self.format)
        sample_rate = kwargs.get('sample_rate', self.sample_rate)
        volume = kwargs.get('volume', self.volume)
        speech_rate = kwargs.get('speech_rate', self.speech_rate)
        pitch_rate = kwargs.get('pitch_rate', self.pitch_rate)

        # 创建合成器
        synthesizer = self._create_synthesizer()
        if not synthesizer:
            return None

        try:
            self.logger.info(f"🎵 开始TTS合成: '{text[:50]}...'")

            # 开始合成
            synthesizer.start(
                voice=voice,
                format=format_type,
                sample_rate=sample_rate,
                volume=volume,
                speech_rate=speech_rate,
                pitch_rate=pitch_rate
            )

            # 发送文本数据
            synthesizer.send_text(text)
            synthesizer.stop()

            # 等待合成完成（最多15秒）
            if not self.synthesis_complete.wait(timeout=15):
                self.logger.error("❌ TTS合成超时")
                return None

            if not self.synthesis_success:
                self.logger.error(f"❌ TTS合成失败: {self.error_message}")
                return None

            # 关闭合成器
            synthesizer.shutdown()

            if self.audio_data:
                self.logger.info(f"✅ TTS合成成功: {len(self.audio_data)} 字节音频")
                return self.audio_data
            else:
                self.logger.warning("⚠️ 合成音频为空")
                return None

        except Exception as e:
            self.logger.error(f"❌ TTS合成异常: {e}")
            try:
                synthesizer.shutdown()
            except:
                pass
            return None

    def synthesize_to_file(self, text: str, output_file: str, **kwargs) -> bool:
        """
        语音合成到文件

        Args:
            text: 待合成文本
            output_file: 输出文件路径
            **kwargs: 合成参数

        Returns:
            是否成功
        """
        try:
            audio_data = self.synthesize_speech(text, **kwargs)
            if not audio_data:
                return False

            # 确保输出目录存在
            os.makedirs(os.path.dirname(output_file), exist_ok=True)

            # 写入音频文件
            with open(output_file, 'wb') as f:
                f.write(audio_data)

            self.logger.info(f"✅ 音频已保存: {output_file}")
            return True

        except Exception as e:
            self.logger.error(f"❌ 保存音频文件失败: {e}")
            return False

    def switch_voice(self, voice_id: str) -> bool:
        """
        切换音色

        Args:
            voice_id: 音色ID

        Returns:
            切换是否成功
        """
        if voice_id not in self.cantonese_voices:
            self.logger.error(f"❌ 不支持的音色: {voice_id}")
            return False

        self.voice = voice_id
        voice_name = self.cantonese_voices.get(voice_id, voice_id)
        self.logger.info(f"✅ 音色切换成功: {voice_id} ({voice_name})")
        return True

    def get_available_voices(self) -> Dict[str, str]:
        """
        获取可用音色列表

        Returns:
            音色ID到音色名称的映射
        """
        return self.cantonese_voices.copy()

    def test_connection(self) -> bool:
        """
        测试WebSocket连接

        Returns:
            连接是否正常
        """
        test_text = "这是一个测试"
        result = self.synthesize_speech(test_text)
        return result is not None

    def get_service_info(self) -> Dict[str, Any]:
        """
        获取服务信息

        Returns:
            服务信息字典
        """
        return {
            'service_type': 'Aliyun TTS WebSocket',
            'endpoint': self.endpoint,
            'voice': self.voice,
            'format': self.format,
            'sample_rate': self.sample_rate,
            'sdk_available': HAS_NLS_SDK,
            'token_valid': self._check_token_validity(),
            'token_expiry': self.token_expiry,
            'app_key_configured': bool(self.app_key),
            'access_key_configured': bool(self.access_key_id and self.access_key_secret),
            'available_voices': list(self.cantonese_voices.keys()),
            'volume': self.volume,
            'speech_rate': self.speech_rate,
            'pitch_rate': self.pitch_rate
        }

    def __del__(self):
        """析构函数，清理资源"""
        try:
            if self.synthesizer:
                self.synthesizer.shutdown()
        except:
            pass

# 全局实例
_tts_service = None

def get_tts_service() -> AliyunTTSWebSocketService:
    """
    获取全局TTS服务实例

    Returns:
        TTS服务实例
    """
    global _tts_service
    if _tts_service is None:
        _tts_service = AliyunTTSWebSocketService()
    return _tts_service

def synthesize_speech(text: str, **kwargs) -> Optional[bytes]:
    """
    语音合成便捷函数

    Args:
        text: 待合成文本
        **kwargs: 合成参数

    Returns:
        音频数据或None
    """
    service = get_tts_service()
    return service.synthesize_speech(text, **kwargs)

def synthesize_to_file(text: str, output_file: str, **kwargs) -> bool:
    """
    语音合成到文件便捷函数

    Args:
        text: 待合成文本
        output_file: 输出文件路径
        **kwargs: 合成参数

    Returns:
        是否成功
    """
    service = get_tts_service()
    return service.synthesize_to_file(text, output_file, **kwargs)

# 向后兼容的别名
AliyunTTSService = AliyunTTSWebSocketService
WebSocketTTSService = AliyunTTSWebSocketService

if __name__ == "__main__":
    # 测试代码
    service = AliyunTTSWebSocketService()

    print("服务信息:")
    print(json.dumps(service.get_service_info(), indent=2, ensure_ascii=False))

    # 测试连接
    print("\n测试连接...")
    if service.test_connection():
        print("✅ WebSocket连接测试成功")
    else:
        print("❌ WebSocket连接测试失败")