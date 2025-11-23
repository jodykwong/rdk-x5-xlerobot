#!/usr/bin/env python3.10
# -*- coding: utf-8 -*-
"""
阿里云TTS WebSocket引擎 - XLeBot语音合成模块
基于阿里云智能语音服务(NLS) WebSocket API
严格遵循架构文档：必须使用WebSocket，不支持HTTP REST API
"""

import logging
import json
import os
import time
import sys
import wave
import pygame
import tempfile
import threading
from typing import Dict, Any, Optional, Callable
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

class AliyunTTSWebSocketEngine:
    """阿里云TTS WebSocket引擎 - 严格遵循架构文档规范"""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        初始化阿里云TTS WebSocket引擎

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
        self.voice = "jiajia"  # 粤语佳佳音色
        self.format = "wav"
        self.sample_rate = 16000
        self.volume = 50
        self.speech_rate = 0  # 正常语速

        # Token管理
        self.token = None
        self.token_expiry = 0
        self.synthesizer = None
        self.audio_data = bytearray()
        self.synthesis_complete = threading.Event()
        self.synthesis_success = False

        # 初始化Token
        self._refresh_token()

        # 音频播放器
        pygame.mixer.init(frequency=16000, size=-16, channels=1, buffer=1024)

        self.logger.info("✅ 阿里云TTS WebSocket引擎初始化完成")
        self.logger.info(f"  - 端点: {self.endpoint}")
        self.logger.info(f"  - 音色: {self.voice}")
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

    def _on_synthesis_start(self, message, *args):
        """合成开始回调"""
        self.logger.info("🎵 TTS合成开始")

    def _on_audio_data(self, data, *args):
        """
        音频数据回调 - 流式接收

        Args:
            data: 音频数据片段
        """
        self.audio_data.extend(data)
        self.logger.debug(f"📥 接收音频数据: {len(data)} 字节")

    def _on_synthesis_completed(self, message, *args):
        """合成完成回调"""
        try:
            result = json.loads(message) if isinstance(message, str) else message
            self.logger.info(f"✅ TTS合成完成，总计 {len(self.audio_data)} 字节")
            self.synthesis_success = True
        except Exception as e:
            self.logger.error(f"❌ 合成完成回调解析失败: {e}")
            self.synthesis_success = False
        finally:
            self.synthesis_complete.set()

    def _on_synthesis_error(self, message, *args):
        """合成错误回调"""
        self.logger.error(f"❌ TTS合成错误: {message}")
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
                on_metainfo=self._on_synthesis_start,
                on_data=self._on_audio_data,
                on_completed=self._on_synthesis_completed,
                on_error=self._on_synthesis_error
            )

            self.logger.info("✅ TTS WebSocket连接器创建成功")
            return synthesizer

        except Exception as e:
            self.logger.error(f"❌ 创建TTS连接器失败: {e}")
            return None

    def synthesize(self, text: str, **kwargs) -> Optional[bytes]:
        """
        语音合成 - WebSocket方式

        Args:
            text: 待合成文本
            **kwargs: 其他参数
                - voice: 音色名称（默认jiajia）
                - format: 音频格式（默认wav）
                - sample_rate: 采样率（默认16000）
                - volume: 音量（默认50）
                - speech_rate: 语速（默认0）

        Returns:
            音频数据（bytes）或None
        """
        if not text or not text.strip():
            self.logger.warning("⚠️ 合成文本为空")
            return None

        # 重置状态
        self.audio_data = bytearray()
        self.synthesis_complete.clear()
        self.synthesis_success = False

        # 获取参数
        voice = kwargs.get('voice', self.voice)
        format_type = kwargs.get('format', self.format)
        sample_rate = kwargs.get('sample_rate', self.sample_rate)
        volume = kwargs.get('volume', self.volume)
        speech_rate = kwargs.get('speech_rate', self.speech_rate)

        # 创建合成器
        synthesizer = self._create_synthesizer()
        if not synthesizer:
            return None

        try:
            self.logger.info(f"🎵 开始TTS合成: {text[:50]}{'...' if len(text) > 50 else ''}")

            # 开始合成
            synthesizer.start(
                voice=voice,
                aformat=format_type,
                sample_rate=sample_rate,
                volume=volume,
                speech_rate=speech_rate,
                text=text
            )

            # 等待合成完成（最多30秒）
            if not self.synthesis_complete.wait(timeout=30):
                self.logger.error("❌ TTS合成超时")
                return None

            if not self.synthesis_success:
                self.logger.error("❌ TTS合成失败")
                return None

            # 关闭合成器
            synthesizer.shutdown()

            audio_bytes = bytes(self.audio_data)
            self.logger.info(f"✅ TTS合成成功: {len(audio_bytes)} 字节")

            return audio_bytes

        except Exception as e:
            self.logger.error(f"❌ TTS合成异常: {e}")
            try:
                synthesizer.shutdown()
            except:
                pass
            return None

    def synthesize_to_file(self, text: str, output_path: str, **kwargs) -> bool:
        """
        合成语音到文件

        Args:
            text: 待合成文本
            output_path: 输出文件路径
            **kwargs: 合成参数

        Returns:
            是否成功
        """
        try:
            # 合成音频
            audio_data = self.synthesize(text, **kwargs)
            if not audio_data:
                return False

            # 确保输出目录存在
            Path(output_path).parent.mkdir(parents=True, exist_ok=True)

            # 写入文件
            with open(output_path, 'wb') as f:
                f.write(audio_data)

            self.logger.info(f"✅ 音频已保存: {output_path}")
            return True

        except Exception as e:
            self.logger.error(f"❌ 保存音频失败: {e}")
            return False

    def play_audio(self, audio_data: bytes) -> bool:
        """
        播放音频数据

        Args:
            audio_data: 音频数据

        Returns:
            是否成功播放
        """
        if not audio_data:
            self.logger.warning("⚠️ 音频数据为空")
            return False

        try:
            # 保存到临时文件
            temp_file = tempfile.NamedTemporaryFile(suffix='.wav', delete=False)
            temp_file.write(audio_data)
            temp_file.close()

            # 播放音频
            pygame.mixer.music.load(temp_file.name)
            pygame.mixer.music.play()

            # 等待播放完成
            while pygame.mixer.music.get_busy():
                pygame.time.wait(100)

            # 清理临时文件
            os.unlink(temp_file.name)

            self.logger.info(f"✅ 音频播放成功: {len(audio_data)} 字节")
            return True

        except Exception as e:
            self.logger.error(f"❌ 音频播放失败: {e}")
            return False

    def synthesize_and_play(self, text: str, **kwargs) -> bool:
        """
        合成并播放语音

        Args:
            text: 待合成文本
            **kwargs: 合成参数

        Returns:
            是否成功
        """
        try:
            # 合成音频
            audio_data = self.synthesize(text, **kwargs)
            if not audio_data:
                return False

            # 播放音频
            return self.play_audio(audio_data)

        except Exception as e:
            self.logger.error(f"❌ 合成并播放失败: {e}")
            return False

    def test_connection(self) -> bool:
        """
        测试WebSocket连接

        Returns:
            连接是否正常
        """
        test_text = "这是一个连接测试"
        audio_data = self.synthesize(test_text)
        return audio_data is not None

    def get_engine_info(self) -> Dict[str, Any]:
        """
        获取引擎信息

        Returns:
            引擎信息字典
        """
        return {
            'engine_type': 'Aliyun TTS WebSocket',
            'endpoint': self.endpoint,
            'voice': self.voice,
            'format': self.format,
            'sample_rate': self.sample_rate,
            'sdk_available': HAS_NLS_SDK,
            'token_valid': self._check_token_validity(),
            'token_expiry': self.token_expiry,
            'app_key_configured': bool(self.app_key),
            'access_key_configured': bool(self.access_key_id and self.access_key_secret)
        }

    def __del__(self):
        """析构函数，清理资源"""
        try:
            if self.synthesizer:
                self.synthesizer.shutdown()
        except:
            pass

# 全局实例
_tts_engine = None

def get_tts_engine() -> AliyunTTSWebSocketEngine:
    """
    获取全局TTS引擎实例

    Returns:
        TTS引擎实例
    """
    global _tts_engine
    if _tts_engine is None:
        _tts_engine = AliyunTTSWebSocketEngine()
    return _tts_engine

def synthesize_speech(text: str, **kwargs) -> Optional[bytes]:
    """
    语音合成便捷函数

    Args:
        text: 待合成文本
        **kwargs: 合成参数

    Returns:
        音频数据或None
    """
    engine = get_tts_engine()
    return engine.synthesize(text, **kwargs)

def play_speech(text: str, **kwargs) -> bool:
    """
    合成并播放语音便捷函数

    Args:
        text: 待合成文本
        **kwargs: 合成参数

    Returns:
        是否成功
    """
    engine = get_tts_engine()
    return engine.synthesize_and_play(text, **kwargs)

# 向后兼容的别名
AliyunTTSEngine = AliyunTTSWebSocketEngine

if __name__ == "__main__":
    # 测试代码
    engine = AliyunTTSWebSocketEngine()

    print("引擎信息:")
    print(json.dumps(engine.get_engine_info(), indent=2, ensure_ascii=False))

    # 测试连接
    print("\n测试连接...")
    if engine.test_connection():
        print("✅ WebSocket连接测试成功")

        # 测试合成
        print("\n测试语音合成...")
        success = engine.synthesize_and_play("你好，我是XLeBot语音助手")
        print(f"合成播放结果: {'成功' if success else '失败'}")
    else:
        print("❌ WebSocket连接测试失败")