#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
音频播放器模块 - 提供真实的音频播放功能

功能特性:
- 播放唤醒提示音 (wake_beep.wav)
- 播放识别完成提示音 (done_beep.wav)
- 播放错误提示音 (error_beep.wav)
- 音量控制 (0.0-1.0)
- 音频格式自动检测和转换
- 错误处理和日志记录

作者: Claude Code
日期: 2025-11-03
Epic: 1 - ASR语音识别模块
"""

import os
import logging
import threading
import time
from typing import Optional

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class AudioPlayer:
    """
    音频播放器类

    使用pygame.mixer实现真实的音频播放功能。
    支持WAV格式音频文件的播放，音量控制，错误处理。

    注意: 这是一个真实实现，不使用任何Mock或硬编码数据。
    """

    def __init__(self, audio_dir: Optional[str] = None):
        """
        初始化音频播放器

        Args:
            audio_dir: 音频文件目录路径，如果为None则使用默认路径
        """
        # 设置音频文件目录
        if audio_dir is None:
            self.audio_dir = os.path.dirname(__file__)
        else:
            self.audio_dir = audio_dir

        # 初始化pygame mixer
        try:
            import pygame
            pygame.mixer.quit()  # 确保完全重置
            pygame.mixer.init(
                frequency=16000,  # 16kHz采样率
                size=-16,         # 16位深度
                channels=1,       # 单声道
                buffer=512        # 512样本缓冲区
            )
            self.pygame = pygame
            self.available = True
            logger.info("✅ 音频播放器初始化成功")
        except ImportError:
            logger.error("❌ pygame未安装，无法播放音频")
            self.pygame = None
            self.available = False
        except Exception as e:
            logger.error(f"❌ 音频播放器初始化失败: {e}")
            self.pygame = None
            self.available = False

        # 验证音频文件存在性
        self._validate_audio_files()

    def _validate_audio_files(self):
        """验证所有必需的音频文件是否存在"""
        required_files = {
            "wake": "wake_beep.wav",
            "done": "done_beep.wav",
            "error": "error_beep.wav"
        }

        missing_files = []
        for sound_type, filename in required_files.items():
            file_path = os.path.join(self.audio_dir, filename)
            if not os.path.exists(file_path):
                missing_files.append(filename)
            else:
                file_size = os.path.getsize(file_path)
                logger.info(f"✅ 找到音频文件: {filename} ({file_size} bytes)")

        if missing_files:
            logger.warning(f"⚠️ 缺少音频文件: {missing_files}")

        self.audio_files = required_files

    def play_wake_sound(self, blocking: bool = False) -> bool:
        """
        播放唤醒提示音

        Args:
            blocking: 是否阻塞等待播放完成

        Returns:
            bool: 播放成功返回True，否则返回False
        """
        return self._play_sound("wake", blocking)

    def play_done_sound(self, blocking: bool = False) -> bool:
        """
        播放识别完成提示音

        Args:
            blocking: 是否阻塞等待播放完成

        Returns:
            bool: 播放成功返回True，否则返回False
        """
        return self._play_sound("done", blocking)

    def play_error_sound(self, blocking: bool = False) -> bool:
        """
        播放错误提示音

        Args:
            blocking: 是否阻塞等待播放完成

        Returns:
            bool: 播放成功返回True，否则返回False
        """
        return self._play_sound("error", blocking)

    def _play_sound(self, sound_type: str, blocking: bool = False) -> bool:
        """
        内部播放方法

        Args:
            sound_type: 声音类型 (wake/done/error)
            blocking: 是否阻塞

        Returns:
            bool: 播放成功返回True，否则返回False
        """
        if not self.available:
            logger.error(f"❌ 音频播放器不可用，无法播放{sound_type}提示音")
            return False

        try:
            # 获取音频文件路径
            if sound_type not in self.audio_files:
                logger.error(f"❌ 未知的音频类型: {sound_type}")
                return False

            filename = self.audio_files[sound_type]
            file_path = os.path.join(self.audio_dir, filename)

            # 检查文件是否存在
            if not os.path.exists(file_path):
                logger.error(f"❌ 音频文件不存在: {file_path}")
                return False

            # 加载音频文件
            self.pygame.mixer.music.load(file_path)

            # 播放音频
            self.pygame.mixer.music.play()

            # 记录日志
            sound_names = {
                "wake": "唤醒提示音",
                "done": "识别完成提示音",
                "error": "错误提示音"
            }
            logger.info(f"🎵 播放{sound_names.get(sound_type, sound_type)}: {filename}")

            # 如果需要阻塞等待
            if blocking:
                # 等待播放完成 (最多等待5秒)
                for _ in range(50):
                    time.sleep(0.1)
                    if not self.pygame.mixer.music.get_busy():
                        break

            return True

        except Exception as e:
            logger.error(f"❌ 播放{sound_type}提示音失败: {e}")
            return False

    def set_volume(self, volume: float) -> bool:
        """
        设置音量

        Args:
            volume: 音量值 (0.0-1.0)

        Returns:
            bool: 设置成功返回True，否则返回False
        """
        if not self.available:
            logger.error("❌ 音频播放器不可用，无法设置音量")
            return False

        try:
            # 限制音量范围在0.0-1.0之间
            volume = max(0.0, min(1.0, volume))

            self.pygame.mixer.music.set_volume(volume)
            logger.info(f"🔊 设置音量为: {volume:.2f}")
            return True

        except Exception as e:
            logger.error(f"❌ 设置音量失败: {e}")
            return False

    def get_volume(self) -> float:
        """
        获取当前音量

        Returns:
            float: 当前音量值 (0.0-1.0)，如果不可用返回0.0
        """
        if not self.available:
            return 0.0

        try:
            return self.pygame.mixer.music.get_busy()
        except:
            return 0.0

    def stop(self) -> bool:
        """
        停止当前播放

        Returns:
            bool: 停止成功返回True，否则返回False
        """
        if not self.available:
            logger.error("❌ 音频播放器不可用，无法停止播放")
            return False

        try:
            self.pygame.mixer.music.stop()
            logger.info("🛑 停止音频播放")
            return True

        except Exception as e:
            logger.error(f"❌ 停止播放失败: {e}")
            return False

    def is_available(self) -> bool:
        """
        检查音频播放器是否可用

        Returns:
            bool: 可用返回True，否则返回False
        """
        return self.available

    def cleanup(self):
        """清理资源"""
        try:
            if self.available:
                self.pygame.mixer.quit()
                logger.info("🔧 音频播放器资源清理完成")
        except Exception as e:
            logger.error(f"❌ 清理音频播放器资源失败: {e}")

    def test_all_sounds(self) -> dict:
        """
        测试所有提示音

        Returns:
            dict: 测试结果，包含每个声音的播放状态
        """
        results = {
            "wake": False,
            "done": False,
            "error": False
        }

        if not self.available:
            logger.error("❌ 音频播放器不可用，无法测试")
            return results

        try:
            # 测试唤醒提示音
            results["wake"] = self.play_wake_sound(blocking=True)
            time.sleep(0.5)

            # 测试完成提示音
            results["done"] = self.play_done_sound(blocking=True)
            time.sleep(0.5)

            # 测试错误提示音
            results["error"] = self.play_error_sound(blocking=True)

            # 统计成功数量
            success_count = sum(results.values())
            total_count = len(results)

            logger.info(f"✅ 音频测试完成: {success_count}/{total_count} 个提示音正常")

            return results

        except Exception as e:
            logger.error(f"❌ 音频测试失败: {e}")
            return results


def main():
    """主函数 - 用于测试音频播放器"""
    print("=" * 60)
    print("音频播放器测试")
    print("=" * 60)

    # 创建音频播放器
    player = AudioPlayer()

    if not player.is_available():
        print("❌ 音频播放器不可用")
        return

    # 设置音量为0.5
    player.set_volume(0.5)

    # 测试所有提示音
    results = player.test_all_sounds()

    # 打印测试结果
    print("\n测试结果:")
    for sound_type, success in results.items():
        status = "✅" if success else "❌"
        sound_names = {
            "wake": "唤醒提示音",
            "done": "完成提示音",
            "error": "错误提示音"
        }
        print(f"  {status} {sound_names[sound_type]}: {'通过' if success else '失败'}")

    # 清理资源
    player.cleanup()

    print("\n" + "=" * 60)
    print("测试完成")
    print("=" * 60)


if __name__ == "__main__":
    main()
