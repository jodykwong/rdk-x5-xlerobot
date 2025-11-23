#!/usr/bin/env python3.10
"""
统一音频录音管理器
================

整合所有音频录音器，提供统一的录音接口。
解决循环引用问题，稳定音频录制架构。

主要功能：
- 统一音频录音器接口
- 智能录音器选择（ALSA优先，PyAudio备用）
- 循环引用问题解决
- 线程安全的录音管理
- 录音器状态监控

作者: Claude Code Agent
版本: 1.0
日期: 2025-11-19
"""

import logging
import threading
import time
import numpy as np
from typing import Optional, Dict, Any, Callable
from enum import Enum

logger = logging.getLogger(__name__)

class RecorderType(Enum):
    """录音器类型枚举"""
    ALSA = "alsa"
    THREAD_SAFE = "thread_safe"
    HYBRID = "hybrid"

class RecordingState(Enum):
    """录音状态枚举"""
    IDLE = "idle"
    STARTING = "starting"
    RECORDING = "recording"
    STOPPING = "stopping"

class UnifiedAudioRecorder:
    """
    统一音频录音管理器

    整合多种录音器实现，提供稳定的音频录制接口。
    自动选择最佳录音器，确保系统稳定性。
    """

    def __init__(self, preferred_recorder: RecorderType = RecorderType.ALSA):
        """
        初始化统一音频录音管理器

        Args:
            preferred_recorder: 首选录音器类型
        """
        self.preferred_recorder = preferred_recorder
        self._recorder = None
        self._recorder_type = None
        self._lock = threading.RLock()

        # 统计信息
        self._stats = {
            'total_attempts': 0,
            'successful_recordings': 0,
            'failed_recordings': 0,
            'recorder_switches': 0,
            'last_recorder_used': None
        }

        # 初始化录音器
        self._init_recorder()

        logger.info(f"✅ 统一音频录音管理器初始化完成")
        logger.info(f"  - 首选录音器: {preferred_recorder.value}")
        logger.info(f"  - 当前录音器: {self._recorder_type.value if self._recorder_type else 'None'}")

    def _init_recorder(self):
        """初始化录音器"""
        # 按优先级尝试不同的录音器
        recorders_to_try = [
            (RecorderType.ALSA, self._create_alsa_recorder),
            (RecorderType.THREAD_SAFE, self._create_thread_safe_recorder),
        ]

        # 如果指定了首选录音器，将其放在第一位
        if self.preferred_recorder != RecorderType.ALSA:
            recorders_to_try = [
                (self.preferred_recorder, self._get_recorder_creator(self.preferred_recorder))
            ] + recorders_to_try

        for recorder_type, creator in recorders_to_try:
            try:
                recorder = creator()
                if recorder and self._test_recorder(recorder):
                    self._recorder = recorder
                    self._recorder_type = recorder_type
                    logger.info(f"✅ 成功初始化录音器: {recorder_type.value}")
                    return
                else:
                    logger.warning(f"⚠️ 录音器测试失败: {recorder_type.value}")
            except Exception as e:
                logger.error(f"❌ 录音器初始化失败: {recorder_type.value}, 错误: {e}")
                continue

        # 如果所有录音器都失败
        logger.error("❌ 所有录音器初始化失败")
        self._recorder = None
        self._recorder_type = None

    def _get_recorder_creator(self, recorder_type: RecorderType):
        """获取录音器创建函数"""
        if recorder_type == RecorderType.ALSA:
            return self._create_alsa_recorder
        elif recorder_type == RecorderType.THREAD_SAFE:
            return self._create_thread_safe_recorder
        else:
            raise ValueError(f"不支持的录音器类型: {recorder_type}")

    def _create_alsa_recorder(self):
        """创建ALSA录音器"""
        try:
            # 延迟导入避免循环引用
            from .simple_alsa_recorder import SimpleALSARecorder
            return SimpleALSARecorder()
        except ImportError as e:
            logger.error(f"无法导入ALSA录音器: {e}")
            return None

    def _create_thread_safe_recorder(self):
        """创建线程安全录音器"""
        try:
            # 使用原始线程安全录音器（不循环引用）
            from .thread_safe_audio_recorder import ThreadSafeAudioRecorder
            return ThreadSafeAudioRecorder()
        except ImportError as e:
            logger.error(f"无法导入线程安全录音器: {e}")
            return None

    def _test_recorder(self, recorder) -> bool:
        """测试录音器是否工作正常"""
        try:
            # 简单的状态检查
            if hasattr(recorder, 'get_state'):
                state = recorder.get_state()
                logger.debug(f"录音器初始状态: {state}")

            # 检查必要的方法
            required_methods = ['start_recording', 'stop_recording']
            for method in required_methods:
                if not hasattr(recorder, method):
                    logger.error(f"录音器缺少必要方法: {method}")
                    return False

            return True
        except Exception as e:
            logger.error(f"录音器测试异常: {e}")
            return False

    def start_recording(self, duration: float = 3.0,
                       callback: Optional[Callable] = None) -> bool:
        """
        开始录音

        Args:
            duration: 录音时长（秒）
            callback: 录音完成回调函数

        Returns:
            bool: 是否成功启动录音
        """
        with self._lock:
            if not self._recorder:
                logger.error("❌ 没有可用的录音器")
                self._stats['failed_recordings'] += 1
                return False

            self._stats['total_attempts'] += 1

            try:
                result = self._recorder.start_recording(duration=duration, callback=callback)
                if result:
                    logger.debug(f"录音启动成功，时长: {duration}秒")
                else:
                    logger.warning("录音启动失败")
                    self._stats['failed_recordings'] += 1
                return result
            except Exception as e:
                logger.error(f"录音启动异常: {e}")
                self._stats['failed_recordings'] += 1
                return False

    def stop_recording(self) -> np.ndarray:
        """
        停止录音

        Returns:
            np.ndarray: 录音数据
        """
        with self._lock:
            if not self._recorder:
                logger.error("❌ 没有可用的录音器")
                return np.array([], dtype=np.int16)

            try:
                result = self._recorder.stop_recording()
                if len(result) > 0:
                    self._stats['successful_recordings'] += 1
                    self._stats['last_recorder_used'] = self._recorder_type.value
                    logger.debug(f"录音完成，数据长度: {len(result)} samples")
                else:
                    self._stats['failed_recordings'] += 1
                    logger.warning("录音完成但无数据")
                return result
            except Exception as e:
                logger.error(f"录音停止异常: {e}")
                self._stats['failed_recordings'] += 1
                return np.array([], dtype=np.int16)

    def get_state(self):
        """获取录音状态"""
        with self._lock:
            if not self._recorder:
                return RecordingState.IDLE

            try:
                return self._recorder.get_state()
            except Exception as e:
                logger.error(f"获取录音状态失败: {e}")
                return RecordingState.IDLE

    def get_stats(self) -> Dict[str, Any]:
        """获取统计信息"""
        with self._lock:
            stats = self._stats.copy()
            stats['current_recorder'] = self._recorder_type.value if self._recorder_type else None
            stats['recorder_available'] = self._recorder is not None

            # 获取底层录音器统计
            if self._recorder and hasattr(self._recorder, 'get_stats'):
                try:
                    stats['recorder_stats'] = self._recorder.get_stats()
                except Exception as e:
                    logger.warning(f"获取底层录音器统计失败: {e}")

            return stats

    def switch_recorder(self, new_recorder_type: RecorderType) -> bool:
        """
        切换录音器类型

        Args:
            new_recorder_type: 新的录音器类型

        Returns:
            bool: 切换是否成功
        """
        with self._lock:
            if new_recorder_type == self._recorder_type:
                logger.info(f"录音器类型已是 {new_recorder_type.value}，无需切换")
                return True

            logger.info(f"切换录音器: {self._recorder_type.value} -> {new_recorder_type.value}")

            # 清理当前录音器
            if self._recorder and hasattr(self._recorder, 'cleanup'):
                try:
                    self._recorder.cleanup()
                except Exception as e:
                    logger.warning(f"清理当前录音器失败: {e}")

            # 初始化新录音器
            old_type = self._recorder_type
            self.preferred_recorder = new_recorder_type
            self._init_recorder()

            if self._recorder:
                self._stats['recorder_switches'] += 1
                logger.info(f"✅ 录音器切换成功: {old_type.value} -> {self._recorder_type.value}")
                return True
            else:
                logger.error(f"❌ 录音器切换失败: 无法初始化 {new_recorder_type.value}")
                return False

    def get_audio_config(self) -> Dict[str, Any]:
        """获取音频配置"""
        with self._lock:
            if self._recorder and hasattr(self._recorder, 'get_audio_config'):
                try:
                    config = self._recorder.get_audio_config()
                    config['recorder_type'] = self._recorder_type.value
                    config['unified'] = True
                    return config
                except Exception as e:
                    logger.error(f"获取音频配置失败: {e}")

            # 默认配置
            return {
                'sample_rate': 16000,
                'channels': 1,
                'format': 'int16',
                'recorder_type': self._recorder_type.value if self._recorder_type else None,
                'unified': True
            }

    def test_recording(self) -> bool:
        """测试录音功能"""
        logger.info("开始统一录音器测试...")

        success = self.start_recording(duration=2.0)

        if success:
            time.sleep(2.5)
            audio_data = self.stop_recording()

            if len(audio_data) > 0:
                logger.info(f"✅ 统一录音器测试成功: {len(audio_data)} samples")
                logger.info(f"  - 当前录音器: {self._recorder_type.value}")
                return True
            else:
                logger.warning("⚠️ 录音完成但无数据")
                return False
        else:
            logger.error("❌ 统一录音器测试启动失败")
            return False

    def force_stop(self):
        """强制停止录音"""
        with self._lock:
            if self._recorder and hasattr(self._recorder, 'force_stop'):
                try:
                    self._recorder.force_stop()
                    logger.info("强制停止录音")
                except Exception as e:
                    logger.error(f"强制停止录音失败: {e}")

    def cleanup(self):
        """清理资源"""
        logger.info("清理统一音频录音管理器...")

        with self._lock:
            if self._recorder:
                try:
                    if hasattr(self._recorder, 'cleanup'):
                        self._recorder.cleanup()
                    elif hasattr(self._recorder, '__del__'):
                        self._recorder.__del__()
                except Exception as e:
                    logger.error(f"清理录音器失败: {e}")
                finally:
                    self._recorder = None

            logger.info("✅ 统一音频录音管理器清理完成")

# 全局实例
_recorder_instance = None
_recorder_lock = threading.Lock()

def get_unified_recorder() -> UnifiedAudioRecorder:
    """获取全局统一录音器实例"""
    global _recorder_instance

    if _recorder_instance is None:
        with _recorder_lock:
            if _recorder_instance is None:
                _recorder_instance = UnifiedAudioRecorder()

    return _recorder_instance

def create_unified_recorder(preferred_recorder: RecorderType = RecorderType.ALSA) -> UnifiedAudioRecorder:
    """
    创建新的统一录音器实例

    Args:
        preferred_recorder: 首选录音器类型

    Returns:
        UnifiedAudioRecorder: 录音器实例
    """
    return UnifiedAudioRecorder(preferred_recorder)

# 向后兼容的函数
def get_recorder():
    """获取录音器实例（向后兼容）"""
    return get_unified_recorder()

def create_recorder():
    """创建录音器实例（向后兼容）"""
    return create_unified_recorder()

if __name__ == "__main__":
    # 测试统一录音器
    logging.basicConfig(level=logging.INFO)

    recorder = create_unified_recorder()
    stats = recorder.get_stats()
    print(f"录音器统计: {stats}")

    # 运行测试
    success = recorder.test_recording()
    print(f"测试结果: {'成功' if success else '失败'}")

    # 清理
    recorder.cleanup()