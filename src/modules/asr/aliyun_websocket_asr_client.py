#!/usr/bin/env python3
"""
阿里云WebSocket ASR客户端 - 基于官方SDK

基于技术文档修复的ASR客户端，使用WebSocket而非HTTP REST API

作者: BMad Master (基于技术文档修复)
日期: 2025-11-13
参考资料: /home/sunrise/xlerobot/docs/aliyun-nls-websocket-connection-guide.md
"""

import json
import os
import logging
import time
import base64
import numpy as np
import wave
import sys
from typing import Dict, List, Optional, Callable
from dataclasses import dataclass

# 添加阿里云SDK路径
sys.path.append('/home/sunrise/.local/lib/python3.10/site-packages')

try:
    from nls.token import getToken
    from nls.speech_recognizer import NlsSpeechRecognizer
    WEBSOCKET_AVAILABLE = True
except ImportError as e:
    logging.error(f"阿里云NLS SDK未安装: {e}")
    logging.error("请运行: pip3 install alibabacloud-nls-python-sdk")
    WEBSOCKET_AVAILABLE = False

logger = logging.getLogger(__name__)

@dataclass
class ASRResult:
    """ASR识别结果"""
    text: str
    confidence: float
    session_id: str
    processing_time: float

class AliyunWebSocketASRClient:
    """阿里云WebSocket ASR客户端 - 正确实现"""

    def __init__(self):
        """初始化客户端"""
        if not WEBSOCKET_AVAILABLE:
            raise ImportError("阿里云NLS SDK未安装，无法使用WebSocket ASR")

        # 配置信息
        # 从环境变量获取配置，不再硬编码
        self.access_key_id = os.getenv("ALIBABA_CLOUD_ACCESS_KEY_ID")
        self.access_key_secret = os.getenv("ALIBABA_CLOUD_ACCESS_KEY_SECRET")
        self.app_key = os.getenv("ALIYUN_NLS_APPKEY")
        self.ws_url = "wss://nls-gateway.cn-shanghai.aliyuncs.com/ws/v1"

        if not all([self.access_key_id, self.access_key_secret, self.app_key]):
            logger.warning("⚠️ 阿里云ASR凭证未完全配置，请检查环境变量: ALIBABA_CLOUD_ACCESS_KEY_ID, ALIBABA_CLOUD_ACCESS_KEY_SECRET, ALIYUN_NLS_APPKEY")

        # 状态管理
        self.token = None
        self.result = ""
        self.completed = False
        self.recognizer = None
        self.session_id = None

        logger.info("✅ WebSocket ASR客户端初始化成功")

    def _get_token(self) -> Optional[str]:
        """获取阿里云Token"""
        try:
            token = getToken(self.access_key_id, self.access_key_secret)
            logger.info(f"✅ Token获取成功: {token[:16]}...")
            return token
        except Exception as e:
            logger.error(f"❌ Token获取失败: {e}")
            return None

    def _convert_audio_to_nls_format(self, audio_file_path: str) -> Optional[bytes]:
        """将音频文件转换为NLS要求的格式"""
        try:
            with wave.open(audio_file_path, 'rb') as wav_file:
                n_channels = wav_file.getnchannels()
                sampwidth = wav_file.getsampwidth()
                framerate = wav_file.getframerate()
                n_frames = wav_file.getnframes()
                audio_data = wav_file.readframes(n_frames)

            logger.info(f"   原始格式: {n_channels}通道, {sampwidth*8}位, {framerate}Hz")

            # 转换为单声道16kHz
            audio_array = np.frombuffer(audio_data, dtype=np.int16)
            if n_channels == 2:
                audio_array = audio_array[::2]  # 左声道

            if framerate != 16000:
                resampling_ratio = 16000 / framerate
                new_length = int(len(audio_array) * resampling_ratio)
                old_indices = np.linspace(0, len(audio_array) - 1, new_length)
                audio_array = np.interp(old_indices, np.arange(len(audio_array)), audio_array.astype(float)).astype(np.int16)

            logger.info(f"   转换后: 1通道, 16位, 16000Hz")
            return audio_array.tobytes()

        except Exception as e:
            logger.error(f"❌ 音频处理失败: {e}")
            return None

    def _on_start(self, message, *args):
        """识别开始回调"""
        logger.info("🎤 识别开始")
        logger.debug(f"   消息: {message}")

    def _on_result_changed(self, message, *args):
        """中间结果回调"""
        try:
            result = json.loads(message)
            if 'payload' in result and 'result' in result['payload']:
                text = result['payload']['result']
                logger.debug(f"🔄 中间结果: {text}")
        except Exception as e:
            logger.error(f"中间结果处理失败: {e}")

    def _on_completed(self, message, *args):
        """识别完成回调"""
        logger.info("✅ 识别完成")
        try:
            result = json.loads(message)

            if 'payload' in result and 'result' in result['payload']:
                self.result = result['payload']['result']
                confidence = result['payload'].get('confidence', 0)
                logger.info(f"🎯 最终结果: '{self.result}' (置信度: {confidence}%)")

            self.completed = True

        except Exception as e:
            logger.error(f"完成结果处理失败: {e}")
            self.completed = True

    def _on_error(self, message, *args):
        """识别错误回调"""
        logger.error(f"❌ 识别错误: {message}")
        self.completed = True

    def recognize_audio(self, audio_file_path: str) -> Optional[ASRResult]:
        """
        识别音频文件 - 基于WebSocket实现

        Args:
            audio_file_path: 音频文件路径

        Returns:
            ASRResult: 识别结果
        """
        start_time = time.time()

        try:
            # 1. 获取Token
            if not self.token:
                self.token = self._get_token()
                if not self.token:
                    logger.error("❌ Token获取失败")
                    return None

            # 2. 转换音频格式
            logger.info(f"🔄 处理音频文件: {audio_file_path}")
            audio_data = self._convert_audio_to_nls_format(audio_file_path)
            if not audio_data:
                return None

            # 3. 创建WebSocket识别器
            self.recognizer = NlsSpeechRecognizer(
                token=self.token,
                appkey=self.app_key,
                on_start=self._on_start,
                on_result_changed=self._on_result_changed,
                on_completed=self._on_completed,
                on_error=self._on_error
            )

            # 4. 启动识别
            logger.info("🚀 启动语音识别...")
            self.recognizer.start()

            # 5. 等待连接建立
            time.sleep(0.5)

            # 6. 分块发送音频数据
            chunk_size = 3200  # 每块200ms
            sent_bytes = 0

            for i in range(0, len(audio_data), chunk_size):
                chunk = audio_data[i:i + chunk_size]
                self.recognizer.send_audio(chunk)
                sent_bytes += len(chunk)

                # 模拟实时流
                if i + chunk_size < len(audio_data):
                    time.sleep(0.1)

            logger.info(f"✅ 音频数据发送完成: {sent_bytes} 字节")

            # 7. 停止识别
            logger.info("⏹️ 停止识别...")
            self.recognizer.stop()

            # 8. 等待结果
            timeout = 15
            while not self.completed and (time.time() - start_time) < timeout:
                time.sleep(0.2)

            processing_time = time.time() - start_time

            # 9. 返回结果
            if self.completed and self.result:
                return ASRResult(
                    text=self.result,
                    confidence=0.95,  # 默认置信度
                    session_id=self.session_id or "unknown",
                    processing_time=processing_time
                )
            else:
                logger.error("❌ 识别超时或无结果")
                return None

        except Exception as e:
            logger.error(f"❌ 识别异常: {e}")
            return None

        finally:
            # 10. 清理连接
            try:
                if self.recognizer:
                    self.recognizer.shutdown()
            except:
                pass

            # 重置状态
            self.result = ""
            self.completed = False
            self.recognizer = None

def test_websocket_asr():
    """测试WebSocket ASR功能"""
    try:
        client = AliyunWebSocketASRClient()

        # 测试文件路径
        test_audio = "/home/sunrise/xlerobot/cantonese_1.wav"

        if not __import__('pathlib').Path(test_audio).exists():
            logger.error(f"❌ 测试音频文件不存在: {test_audio}")
            return False

        logger.info(f"🧪 开始测试WebSocket ASR...")
        result = client.recognize_audio(test_audio)

        if result:
            logger.info(f"✅ 测试成功!")
            logger.info(f"   识别结果: {result.text}")
            logger.info(f"   处理时间: {result.processing_time:.2f}秒")
            return True
        else:
            logger.error("❌ 测试失败")
            return False

    except Exception as e:
        logger.error(f"❌ 测试异常: {e}")
        return False

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    test_websocket_asr()