#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
唤醒词检测器模块
用于检测"傻强"唤醒词 - 基于ASR的智能检测版本
"""

import asyncio
import logging
import numpy as np
import sys
import os
from typing import Optional, Dict, Any
from pathlib import Path

# 添加项目路径
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))

try:
    from modules.asr.websocket.websocket_asr_service import AliyunASRWebSocketService
    ASR_SERVICE_AVAILABLE = True
    logger = logging.getLogger(__name__)
    logger.info("✅ WebSocket ASR服务可用于唤醒词检测")
except ImportError as e:
    ASR_SERVICE_AVAILABLE = False
    logger = logging.getLogger(__name__)
    logger.warning(f"⚠️ WebSocket ASR服务不可用，使用备用检测方案: {e}")

logger = logging.getLogger(__name__)

class WakeWordDetector:
    """基于ASR的智能唤醒词检测器"""

    def __init__(self, wake_word: str = "傻强", threshold: float = 0.7):
        self.wake_word = wake_word
        self.threshold = threshold
        self.is_listening = False

        # 唤醒词变体列表（提高识别率）
        self.wake_word_variants = [
            wake_word,  # 傻强
            "傻强呀",   # 傻强呀
            "傻强啊",   # 傻强啊
            "傻强仔",   # 傻强仔
            "阿强",     # 阿强
            "强仔",     # 强仔
        ]

        # 检测状态
        self._last_detection_time = 0
        self._detection_cooldown = 2.0  # 2秒冷却时间避免重复触发
        self._detection_count = 0

        # ASR服务实例
        self._asr_service = None
        self._init_asr_service()

        # 音频能量检测阈值
        self._energy_threshold = 0.02
        self._min_audio_length = 8000  # 最小音频长度（样本数）

    def _init_asr_service(self):
        """初始化ASR服务"""
        if ASR_SERVICE_AVAILABLE:
            try:
                self._asr_service = AliyunASRWebSocketService()
                logger.info("✅ 唤醒词检测器：WebSocket ASR服务初始化成功")
            except Exception as e:
                logger.warning(f"⚠️ ASR服务初始化失败: {e}")
                self._asr_service = None
        else:
            logger.warning("⚠️ ASR服务不可用，将使用基础能量检测")
            self._asr_service = None

    def start_listening(self):
        """开始监听唤醒词"""
        self.is_listening = True
        logger.info(f"🎯 开始监听唤醒词: {self.wake_word} (变体: {len(self.wake_word_variants)}个)")

    def stop_listening(self):
        """停止监听唤醒词"""
        self.is_listening = False
        logger.info("⏹️ 停止监听唤醒词")

    def _check_audio_energy(self, audio_data: np.ndarray) -> bool:
        """检查音频能量是否足够"""
        if audio_data is None or len(audio_data) == 0:
            return False

        # 检查音频长度
        if len(audio_data) < self._min_audio_length:
            return False

        # 计算音频能量 (RMS)
        audio_energy = np.sqrt(np.mean(audio_data**2))
        return audio_energy > self._energy_threshold

    def _check_cooldown(self) -> bool:
        """检查是否在冷却期内"""
        current_time = asyncio.get_event_loop().time()
        return (current_time - self._last_detection_time) < self._detection_cooldown

    def _contains_wake_word(self, text: str) -> tuple[bool, float]:
        """
        检查文本是否包含唤醒词

        Returns:
            (is_detected, confidence): 是否检测到和置信度
        """
        if not text or not isinstance(text, str):
            return False, 0.0

        text = text.strip().lower()

        # 检查精确匹配
        for variant in self.wake_word_variants:
            if variant in text:
                # 计算置信度（基于匹配度）
                if variant == text:
                    confidence = 1.0  # 完全匹配
                elif text.startswith(variant):
                    confidence = 0.9  # 开头匹配
                else:
                    confidence = 0.8  # 包含匹配

                logger.debug(f"🎯 检测到唤醒词变体: '{variant}' in '{text}' (置信度: {confidence})")
                return True, confidence

        return False, 0.0

    async def detect(self, audio_data: Optional[np.ndarray] = None) -> bool:
        """
        检测唤醒词 - 基于ASR的智能检测

        Args:
            audio_data: 音频数据 (numpy数组)

        Returns:
            bool: 是否检测到唤醒词
        """
        if not self.is_listening or audio_data is None:
            return False

        # 检查冷却期
        if self._check_cooldown():
            return False

        # 检查音频能量（预处理）
        if not self._check_audio_energy(audio_data):
            return False

        try:
            # 方案A：使用ASR服务进行检测
            if self._asr_service is not None:
                return await self._detect_with_asr(audio_data)
            else:
                # 方案B：使用基础能量检测（备用方案）
                return await self._detect_with_energy(audio_data)

        except Exception as e:
            logger.error(f"❌ 唤醒词检测错误: {e}")
            return False

    async def _detect_with_asr(self, audio_data: np.ndarray) -> bool:
        """使用ASR服务检测唤醒词"""
        try:
            # 将numpy音频转换为ASR服务需要的格式
            import tempfile
            import wave

            # 创建临时WAV文件
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                temp_filename = temp_file.name

                # 写入WAV文件 (16kHz, 单声道, 16-bit)
                with wave.open(temp_filename, 'wb') as wav_file:
                    wav_file.setnchannels(1)  # 单声道
                    wav_file.setsampwidth(2)  # 16-bit
                    wav_file.setframerate(16000)  # 16kHz

                    # 转换音频数据格式
                    audio_int16 = (audio_data * 32767).astype(np.int16)
                    wav_file.writeframes(audio_int16.tobytes())

            # 读取音频数据
            with open(temp_filename, 'rb') as f:
                audio_bytes = f.read()

            # 删除临时文件
            os.unlink(temp_filename)

            # 调用ASR识别（使用正确的recognize_audio方法）
            result_text = self._asr_service.recognize_audio(
                audio_bytes,
                language="cn-cantonese"
            )

            if result_text:
                detected, confidence = self._contains_wake_word(result_text)

                if detected and confidence >= self.threshold:
                    self._last_detection_time = asyncio.get_event_loop().time()
                    self._detection_count += 1
                    logger.info(f"🎤 唤醒词检测成功! '{result_text}' (置信度: {confidence:.2f})")
                    return True
                else:
                    logger.debug(f"🔍 ASR结果: '{result_text}' (未检测到唤醒词)")

            return False

        except Exception as e:
            logger.error(f"❌ ASR唤醒词检测失败: {e}")
            return False

    async def _detect_with_energy(self, audio_data: np.ndarray) -> bool:
        """备用方案：基于音频特征的简单检测"""
        try:
            # 计算音频特征
            energy = np.sqrt(np.mean(audio_data**2))
            zcr = np.mean(np.abs(np.diff(np.sign(audio_data))))  # 过零率

            # 简单的启发式规则：语音信号通常有一定的能量和过零率
            if energy > self._energy_threshold * 2 and zcr > 0.05:
                # 模拟检测（用于测试ASR服务不可用的情况）
                if np.random.random() < 0.1:  # 10%概率模拟检测到
                    self._last_detection_time = asyncio.get_event_loop().time()
                    self._detection_count += 1
                    logger.warning(f"🎯 备用方案模拟检测到唤醒词 (能量: {energy:.3f}, 过零率: {zcr:.3f})")
                    return True

            return False

        except Exception as e:
            logger.error(f"❌ 备用检测方案失败: {e}")
            return False

    def is_detected(self) -> bool:
        """检查是否检测到唤醒词（兼容接口）"""
        # 这个方法为了兼容性保留，但实际检测在detect()方法中完成
        return self._detection_count > 0

    def get_detection_stats(self) -> Dict[str, Any]:
        """获取检测统计信息"""
        return {
            "detection_count": self._detection_count,
            "last_detection_time": self._last_detection_time,
            "is_listening": self.is_listening,
            "asr_service_available": self._asr_service is not None,
            "wake_word_variants": len(self.wake_word_variants)
        }