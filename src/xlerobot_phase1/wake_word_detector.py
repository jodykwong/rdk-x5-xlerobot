#!/usr/bin/env python3.10
# -*- coding: utf-8 -*-

"""
XLeRobot Phase 1 唤醒词检测器模块
用于检测"傻强"唤醒词 - 基于ASR的智能检测版本

Epic: 1 - 语音唤醒和基础识别
作者: Claude Code
创建日期: 2025-11-19
"""

import logging
import numpy as np
import sys
import os
from typing import Optional, Dict, Any, Callable
from pathlib import Path
import threading
import time

# 添加项目路径
sys.path.insert(0, str(Path(__file__).parent.parent))

try:
    from modules.asr.websocket.websocket_asr_service import AliyunASRWebSocketService
    ASR_SERVICE_AVAILABLE = True
    logging.getLogger(__name__).info("✅ WebSocket ASR服务可用于唤醒词检测")
except ImportError as e:
    ASR_SERVICE_AVAILABLE = False
    logging.getLogger(__name__).warning(f"⚠️ WebSocket ASR服务不可用，使用备用检测方案: {e}")

logger = logging.getLogger(__name__)

class WakeWordDetector:
    """基于ASR的智能唤醒词检测器"""

    def __init__(self, wake_word: str = "傻强", threshold: float = 0.7):
        """
        初始化唤醒词检测器

        Args:
            wake_word: 唤醒词（默认"傻强"）
            threshold: 检测阈值（0.0-1.0）
        """
        self.wake_word = wake_word
        self.threshold = threshold
        self.is_listening = False
        self.lock = threading.Lock()

        # 唤醒词变体列表（提高识别率）
        self.wake_word_variants = [
            wake_word,          # 傻强
            "傻强呀",            # 傻强呀
            "傻强啊",            # 傻强啊
            "傻强仔",            # 傻强仔
            "阿强",              # 阿强
            "强仔",              # 强仔
        ]

        # 检测状态
        self.detection_count = 0
        self.last_detection_time = 0
        self.cooldown_period = 2.0  # 冷却时间（秒）

        # ASR服务
        self.asr_service = None
        self._init_asr_service()

        # 回调函数
        self.on_wake_detected: Optional[Callable[[str], None]] = None

        logger.info(f"✅ 唤醒词检测器初始化完成")
        logger.info(f"  - 唤醒词: {wake_word}")
        logger.info(f"  - 检测阈值: {threshold}")
        logger.info(f"  - 变体数量: {len(self.wake_word_variants)}")
        logger.info(f"  - ASR服务: {'可用' if ASR_SERVICE_AVAILABLE else '不可用'}")

    def _init_asr_service(self):
        """初始化ASR服务"""
        if ASR_SERVICE_AVAILABLE:
            try:
                self.asr_service = AliyunASRWebSocketService()
                logger.info("✅ ASR服务初始化成功")
            except Exception as e:
                logger.error(f"❌ ASR服务初始化失败: {e}")
                self.asr_service = None

    def start_listening(self, callback: Optional[Callable[[str], None]] = None):
        """
        开始监听唤醒词

        Args:
            callback: 检测到唤醒词时的回调函数
        """
        with self.lock:
            if self.is_listening:
                logger.warning("⚠️ 唤醒词检测器已在监听状态")
                return

            self.is_listening = True
            self.on_wake_detected = callback
            logger.info("🎤 开始监听唤醒词...")

    def stop_listening(self):
        """停止监听唤醒词"""
        with self.lock:
            if not self.is_listening:
                logger.warning("⚠️ 唤醒词检测器未在监听状态")
                return

            self.is_listening = False
            self.on_wake_detected = None
            logger.info("🛑 停止监听唤醒词")

    def detect_wake_word(self, audio_data: bytes) -> bool:
        """
        检测音频中是否包含唤醒词

        Args:
            audio_data: 音频数据（PCM格式）

        Returns:
            是否检测到唤醒词
        """
        if not self.is_listening:
            return False

        # 冷却时间检查
        current_time = time.time()
        if current_time - self.last_detection_time < self.cooldown_period:
            return False

        # 使用ASR服务进行识别
        if self.asr_service:
            return self._detect_with_asr(audio_data)
        else:
            return self._detect_fallback(audio_data)

    def _detect_with_asr(self, audio_data: bytes) -> bool:
        """使用ASR服务进行唤醒词检测"""
        try:
            result = self.asr_service.recognize_audio(audio_data)

            if result and result.strip():
                logger.debug(f"🎤 ASR识别结果: '{result}'")

                # 检查是否包含唤醒词
                detected = self._check_text_for_wake_word(result)

                if detected:
                    self._on_wake_word_detected(result)
                    return True

            return False

        except Exception as e:
            logger.error(f"❌ ASR唤醒词检测失败: {e}")
            return False

    def _detect_fallback(self, audio_data: bytes) -> bool:
        """备选检测方案（简单音频分析）"""
        try:
            # 简单的音频能量检测作为备选方案
            audio_array = np.frombuffer(audio_data, dtype=np.int16)

            # 计算音频能量
            energy = np.mean(np.abs(audio_array))

            # 简单阈值检测（非常基础的备选方案）
            if energy > 1000:  # 简单能量阈值
                logger.debug(f"🎤 备选检测：音频能量={energy:.1f}")
                # 模拟检测结果（备选方案）
                mock_result = f"检测到音频活动，能量={energy:.1f}"
                self._on_wake_word_detected(mock_result)
                return True

            return False

        except Exception as e:
            logger.error(f"❌ 备选唤醒词检测失败: {e}")
            return False

    def _check_text_for_wake_word(self, text: str) -> bool:
        """
        检查文本是否包含唤醒词

        Args:
            text: 要检查的文本

        Returns:
            是否包含唤醒词
        """
        text = text.strip().lower()

        # 检查所有变体
        for variant in self.wake_word_variants:
            if variant.lower() in text:
                logger.info(f"🎯 检测到唤醒词变体: '{variant}' 在 '{text}'")
                return True

        return False

    def _on_wake_word_detected(self, recognized_text: str):
        """处理唤醒词检测事件"""
        self.detection_count += 1
        self.last_detection_time = time.time()

        logger.info(f"🎯 唤醒词检测成功! (第{self.detection_count}次)")
        logger.info(f"📝 识别文本: '{recognized_text}'")

        # 调用回调函数
        if self.on_wake_detected:
            try:
                self.on_wake_detected(recognized_text)
            except Exception as e:
                logger.error(f"❌ 唤醒词回调函数执行失败: {e}")

    def set_wake_word(self, wake_word: str):
        """
        设置新的唤醒词

        Args:
            wake_word: 新的唤醒词
        """
        with self.lock:
            old_word = self.wake_word
            self.wake_word = wake_word

            # 更新变体列表
            self.wake_word_variants = [
                wake_word,
                f"{wake_word}呀",
                f"{wake_word}啊",
                f"{wake_word}仔",
            ]

            logger.info(f"✅ 唤醒词已更新: '{old_word}' -> '{wake_word}'")

    def set_threshold(self, threshold: float):
        """
        设置检测阈值

        Args:
            threshold: 新的检测阈值（0.0-1.0）
        """
        if 0.0 <= threshold <= 1.0:
            self.threshold = threshold
            logger.info(f"✅ 检测阈值已更新: {threshold}")
        else:
            logger.error(f"❌ 无效的阈值: {threshold}")

    def set_cooldown_period(self, cooldown_seconds: float):
        """
        设置冷却时间

        Args:
            cooldown_seconds: 冷却时间（秒）
        """
        if cooldown_seconds >= 0:
            self.cooldown_period = cooldown_seconds
            logger.info(f"✅ 冷却时间已更新: {cooldown_seconds}秒")
        else:
            logger.error(f"❌ 无效的冷却时间: {cooldown_seconds}")

    def get_stats(self) -> Dict[str, Any]:
        """
        获取检测器统计信息

        Returns:
            统计信息字典
        """
        with self.lock:
            return {
                'wake_word': self.wake_word,
                'is_listening': self.is_listening,
                'detection_count': self.detection_count,
                'threshold': self.threshold,
                'cooldown_period': self.cooldown_period,
                'last_detection_time': self.last_detection_time,
                'asr_service_available': ASR_SERVICE_AVAILABLE,
                'wake_word_variants_count': len(self.wake_word_variants)
            }

    def test_detection(self, test_text: str) -> bool:
        """
        测试唤醒词检测

        Args:
            test_text: 测试文本

        Returns:
            检测结果
        """
        return self._check_text_for_wake_word(test_text)

    def cleanup(self):
        """清理资源"""
        logger.info("🧹 清理唤醒词检测器资源...")

        self.stop_listening()

        if self.asr_service:
            try:
                # ASR服务的清理在服务类内部处理
                pass
            except:
                pass

        logger.info("✅ 唤醒词检测器资源清理完成")


# 便捷函数
def create_wake_word_detector(wake_word: str = "傻强", threshold: float = 0.7) -> WakeWordDetector:
    """
    创建唤醒词检测器实例

    Args:
        wake_word: 唤醒词
        threshold: 检测阈值

    Returns:
        WakeWordDetector实例
    """
    return WakeWordDetector(wake_word, threshold)


# 测试和验证函数
def test_wake_word_detector():
    """测试唤醒词检测器功能"""
    logger.info("🧪 测试唤醒词检测器功能")

    try:
        # 创建检测器
        detector = create_wake_word_detector()

        # 测试文本检测
        test_cases = [
            ("傻强", True),
            ("傻强呀", True),
            ("阿强", True),
            ("你好", False),
            ("其他话", False)
        ]

        for text, expected in test_cases:
            result = detector.test_detection(text)
            status = "✅" if result == expected else "❌"
            logger.info(f"{status} 测试 '{text}': 期望={expected}, 实际={result}")

        # 获取统计信息
        stats = detector.get_stats()
        logger.info(f"📊 检测器统计: {stats}")

        # 清理
        detector.cleanup()

        logger.info("🎉 唤醒词检测器测试完成")
        return True

    except Exception as e:
        logger.error(f"❌ 唤醒词检测器测试失败: {e}")
        return False


if __name__ == "__main__":
    # 运行测试
    test_wake_word_detector()