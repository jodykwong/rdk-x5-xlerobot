#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
粤语TTS模块 - 阿里云语音合成服务
Cantonese TTS Module - Aliyun NLS Service

提供粤语文本转语音功能，集成阿里云NLS TTS服务。
支持佳佳发音人，专门优化粤语语音合成。

作者: Dev Agent
Epic: 1 - TTS语音合成模块
"""

import os
import time
import logging
import requests
import json
import base64
from typing import Optional, Dict, Any
import hashlib
import hmac
import urllib.parse
from datetime import datetime

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# 导入音频播放器
try:
    from .audio_player import AudioPlayer
    AUDIO_PLAYER_AVAILABLE = True
except ImportError as e:
    logger.warning(f"⚠️ 无法导入AudioPlayer: {e}")
    AUDIO_PLAYER_AVAILABLE = False


class CantoneseTTS:
    """
    粤语TTS合成器

    功能特性:
    - 阿里云NLS TTS集成
    - 佳佳粤语发音人
    - 实时语音合成
    - 音频缓存
    - 音量控制
    """

    def __init__(self):
        """初始化粤语TTS"""
        self.tts_engine = "AliyunNLS"  # 标记TTS引擎类型
        self.audio_player: Optional[AudioPlayer] = None
        self.api_config = self._load_api_config()

        # 初始化音频播放器
        if AUDIO_PLAYER_AVAILABLE:
            try:
                self.audio_player = AudioPlayer()
                if self.audio_player.is_available():
                    logger.info("✅ TTS音频播放器初始化成功")
                else:
                    logger.warning("⚠️ TTS音频播放器不可用")
            except Exception as e:
                logger.error(f"❌ TTS音频播放器初始化失败: {e}")
                self.audio_player = None

        logger.info("粤语TTS初始化完成")

    def _load_api_config(self) -> Dict[str, Any]:
        """加载阿里云API配置"""
        try:
            # 优先从环境变量读取
            config = {
                "app_key": os.getenv("ALIYUN_NLS_APP_KEY", ""),
                "app_secret": os.getenv("ALIYUN_NLS_APP_SECRET", ""),
                "gateway_url": "https://nls-gateway.cn-shanghai.aliyuncs.com/stream/v1/tts",
                "voice": "jiajia",  # 佳佳发音人
                "format": "wav",
                "sample_rate": 16000,
                "volume": 80,
                "speech_rate": 0,
                "pitch_rate": 0
            }

            # 尝试从配置文件读取
            config_file = "/home/sunrise/xlerobot/config/aliyun_nls_config.yaml"
            if os.path.exists(config_file):
                import yaml
                with open(config_file, 'r') as f:
                    yaml_config = yaml.safe_load(f)
                    if "aliyun_nls" in yaml_config:
                        nls_config = yaml_config["aliyun_nls"]
                        config.update(nls_config.get("tts", {}))
                        logger.info("✅ 从配置文件加载TTS设置")

            # 检查必要配置
            if not config["app_key"] or not config["app_secret"]:
                logger.warning("⚠️ 阿里云NLS凭证未配置，TTS功能受限")
                config["enabled"] = False
            else:
                config["enabled"] = True
                logger.info("✅ 阿里云NLS凭证已配置")

            return config

        except Exception as e:
            logger.error(f"❌ TTS配置加载失败: {e}")
            return {"enabled": False}

    def _generate_token(self) -> Optional[str]:
        """生成阿里云API Token"""
        try:
            if not self.api_config.get("enabled"):
                return None

            # 构建Token请求
            token_url = "https://nls-gateway.cn-shanghai.aliyuncs.com/stream/v1/token"

            payload = {
                "auth": {
                    "key": self.api_config["app_key"],
                    "secret": self.api_config["app_secret"]
                },
                "type": " tts"
            }

            response = requests.post(token_url, json=payload, timeout=10)

            if response.status_code == 200:
                token_data = response.json()
                if "token" in token_data and "id" in token_data["token"]:
                    token = token_data["token"]["id"]
                    logger.debug("Token生成成功")
                    return token
                else:
                    logger.error(f"Token响应格式错误: {token_data}")
                    return None
            else:
                logger.error(f"Token请求失败: {response.status_code}, {response.text}")
                return None

        except Exception as e:
            logger.error(f"Token生成失败: {e}")
            return None

    def synthesize(self, text: str, save_path: Optional[str] = None) -> Optional[bytes]:
        """
        合成语音

        Args:
            text: 要合成的文本
            save_path: 可选的保存路径

        Returns:
            音频数据 (bytes)，失败返回None
        """
        try:
            if not self.api_config.get("enabled"):
                logger.warning("TTS服务未启用")
                return None

            if not text.strip():
                logger.warning("文本为空，跳过合成")
                return None

            # 生成Token
            token = self._generate_token()
            if not token:
                logger.error("Token生成失败，无法合成语音")
                return None

            # 构建TTS请求
            url = self.api_config["gateway_url"]

            headers = {
                "Authorization": f"Bearer {token}",
                "Content-Type": "application/json"
            }

            payload = {
                "appkey": self.api_config["app_key"],
                "text": text,
                "voice": self.api_config["voice"],
                "format": self.api_config["format"],
                "sample_rate": self.api_config["sample_rate"],
                "volume": self.api_config["volume"],
                "speech_rate": self.api_config["speech_rate"],
                "pitch_rate": self.api_config["pitch_rate"],
                "enable_subtitle": False
            }

            logger.info(f"开始TTS合成: {text[:50]}...")
            start_time = time.time()

            # 发送请求
            response = requests.post(url, json=payload, headers=headers, timeout=30)

            if response.status_code == 200:
                audio_data = response.content
                duration = time.time() - start_time

                logger.info(f"TTS合成成功: {len(audio_data)} bytes, 耗时: {duration:.2f}s")

                # 保存音频文件
                if save_path:
                    with open(save_path, 'wb') as f:
                        f.write(audio_data)
                    logger.info(f"音频已保存: {save_path}")

                return audio_data
            else:
                logger.error(f"TTS请求失败: {response.status_code}, {response.text}")
                return None

        except Exception as e:
            logger.error(f"TTS合成失败: {e}")
            return None

    def speak(self, text: str, volume: Optional[float] = None) -> bool:
        """
        直接播放语音

        Args:
            text: 要播放的文本
            volume: 可选的音量覆盖 (0.0-1.0)

        Returns:
            bool: 播放是否成功
        """
        try:
            # 合成语音
            audio_data = self.synthesize(text)
            if not audio_data:
                return False

            # 播放音频
            if self.audio_player:
                # 设置音量
                if volume is not None:
                    original_volume = self.audio_player.get_volume()
                    self.audio_player.set_volume(volume)

                # 播放音频
                success = self.audio_player.play_bytes(audio_data)

                # 恢复音量
                if volume is not None:
                    self.audio_player.set_volume(original_volume)

                return success
            else:
                logger.warning("音频播放器不可用")
                return False

        except Exception as e:
            logger.error(f"语音播放失败: {e}")
            return False

    def set_volume(self, volume: float) -> bool:
        """设置默认音量"""
        try:
            if self.audio_player:
                return self.audio_player.set_volume(volume)
            else:
                # 更新配置中的音量
                self.api_config["volume"] = int(volume * 100)
                return True
        except Exception as e:
            logger.error(f"音量设置失败: {e}")
            return False

    def get_available_voices(self) -> Dict[str, str]:
        """获取可用发音人"""
        return {
            "jiajia": "佳佳 (粤语女声)",
            "xiaoxiao": "晓晓 (标准女声)",
            "xiaoyun": "晓云 (知性女声)"
        }

    def __str__(self) -> str:
        """返回TTS的字符串表示"""
        status = "可用" if self.api_config.get("enabled") else "不可用"
        voice = self.api_config.get("voice", "jiajia")

        return (
            f"粤语TTS合成器 - {voice}\n"
            f"状态: {status}\n"
            f"引擎: {self.tts_engine}\n"
            f"采样率: {self.api_config.get('sample_rate', 16000)}Hz\n"
            f"格式: {self.api_config.get('format', 'wav')}"
        )


def create_cantonese_tts() -> CantoneseTTS:
    """创建粤语TTS实例"""
    return CantoneseTTS()


# 示例使用
if __name__ == "__main__":
    # 创建TTS
    tts = create_cantonese_tts()

    print(tts)
    print("\n可用发音人:")
    for voice_id, voice_name in tts.get_available_voices().items():
        print(f"  {voice_id}: {voice_name}")

    # 测试语音合成
    test_text = "傻强系度,老细有乜可以帮到你!"
    print(f"\n测试合成: {test_text}")

    audio_data = tts.synthesize(test_text, "test_tts_output.wav")
    if audio_data:
        print(f"✅ 合成成功: {len(audio_data)} bytes")
    else:
        print("❌ 合成失败")