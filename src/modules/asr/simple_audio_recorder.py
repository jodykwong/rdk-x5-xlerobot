#!/usr/bin/env python3
"""
Simple Audio Recorder - Pure Online Architecture
===============================================

⚠️ 严禁Mock数据声明：
- 使用真实ALSA音频设备输入
- 禁止任何模拟或硬编码音频数据
- 确保所有音频数据来自真实麦克风

基础ALSA音频录制器，专为纯在线语音服务设计。
严格遵循技术边界：无本地处理，简单格式转换。

功能：
- ALSA基础音频录制
- 16kHz, 16-bit, 单声道格式
- 简单音频缓冲
- PCM原始数据输出

作者: Developer Agent
版本: 1.0 (纯在线架构)
日期: 2025-11-09
"""

import pyaudio
import numpy as np
import threading
import time
from typing import Optional, Callable
import logging

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class SimpleAudioRecorder:
    """
    简单音频录制器

    专为纯在线服务设计，提供基础的ALSA音频录制功能。
    严格遵循简化原则，避免任何复杂的音频处理。
    """

    def __init__(self):
        """初始化音频录制器"""
        # 音频参数（阿里云API兼容）
        self.FORMAT = pyaudio.paInt16  # 16-bit
        self.CHANNELS = 1  # 单声道
        self.RATE = 16000  # 16kHz
        self.CHUNK = 1024  # 缓冲区大小

        # 录制状态
        self.is_recording = False
        self.audio_data = []
        self.recording_thread = None
        self.callback = None

        # PyAudio实例
        self.audio = None
        self.stream = None

        logger.info("SimpleAudioRecorder初始化完成")
        logger.info(f"音频参数: {self.RATE}Hz, {self.CHANNELS}通道, 16-bit")

    def start_recording(self, duration: float = 3.0,
                       callback: Optional[Callable] = None) -> bool:
        """
        开始录制音频

        Args:
            duration: 录制时长（秒）
            callback: 录制完成回调函数

        Returns:
            bool: 录制启动成功状态
        """
        if self.is_recording:
            logger.warning("录制已在进行中")
            return False

        try:
            self.callback = callback
            self.audio_data = []
            self.is_recording = True

            # 启动录制线程
            self.recording_thread = threading.Thread(
                target=self._record_worker,
                args=(duration,),
                daemon=True
            )
            self.recording_thread.start()

            logger.info(f"开始录制音频，时长: {duration}秒")
            return True

        except Exception as e:
            logger.error(f"启动录制失败: {e}")
            self.is_recording = False
            return False

    def stop_recording(self) -> np.ndarray:
        """
        停止录制并返回音频数据

        Returns:
            np.ndarray: PCM音频数据
        """
        if not self.is_recording:
            logger.warning("当前没有在录制")
            return np.array([], dtype=np.int16)

        self.is_recording = False

        # 等待录制线程结束
        if self.recording_thread and self.recording_thread.is_alive():
            self.recording_thread.join(timeout=1.0)

        # 合并音频数据
        if self.audio_data:
            audio_array = np.concatenate(self.audio_data)
            logger.info(f"录制完成，数据长度: {len(audio_array)} 样本")
            return audio_array
        else:
            logger.warning("没有录制到音频数据")
            return np.array([], dtype=np.int16)

    def _record_worker(self, duration: float):
        """
        录制工作线程

        Args:
            duration: 录制时长
        """
        try:
            self.audio = pyaudio.PyAudio()

            # 打开音频流，使用sysdefault设备 (设备2，支持采样率转换)
            self.stream = self.audio.open(
                format=self.FORMAT,
                channels=self.CHANNELS,
                rate=self.RATE,
                input=True,
                input_device_index=2,  # 使用sysdefault设备支持采样率转换
                frames_per_buffer=self.CHUNK
            )

            logger.info("音频流已打开")

            # 记录开始时间
            start_time = time.time()
            frames_per_duration = int(self.RATE * duration / self.CHUNK)
            chunk_count = 0

            # 录制循环
            while self.is_recording and chunk_count < frames_per_duration:
                try:
                    # 读取音频数据
                    data = self.stream.read(self.CHUNK, exception_on_overflow=False)

                    # 转换为numpy数组
                    audio_chunk = np.frombuffer(data, dtype=np.int16)

                    # 添加幅度检查
                    max_amplitude = np.max(np.abs(audio_chunk))
                    rms = np.sqrt(np.mean(audio_chunk.astype(float)**2))

                    # 每100个块输出一次幅度信息
                    if chunk_count % 100 == 0:
                        logger.info(f"[录音调试] 块{chunk_count}: 最大幅度={max_amplitude}, RMS={rms:.1f}")

                    # 如果幅度太小，警告
                    if max_amplitude < 100 and chunk_count % 50 == 0:
                        logger.warning(f"⚠️ [录音警告] 音频信号太弱！最大幅度: {max_amplitude}")

                    self.audio_data.append(audio_chunk)

                    chunk_count += 1

                    # 检查是否超时
                    if time.time() - start_time >= duration:
                        break

                except Exception as e:
                    logger.error(f"读取音频数据失败: {e}")
                    break

            logger.info(f"录制了 {chunk_count} 个音频块")

        except Exception as e:
            logger.error(f"录制工作线程异常: {e}")
        finally:
            # 清理资源
            self._cleanup_stream()

    def _cleanup_stream(self):
        """清理音频流资源"""
        try:
            if self.stream:
                self.stream.stop_stream()
                self.stream.close()
                self.stream = None
                logger.debug("音频流已关闭")

            if self.audio:
                self.audio.terminate()
                self.audio = None
                logger.debug("PyAudio实例已终止")

        except Exception as e:
            logger.error(f"清理音频流失败: {e}")

    def get_audio_config(self) -> dict:
        """
        获取音频配置信息

        Returns:
            dict: 音频配置参数
        """
        return {
            'sample_rate': self.RATE,
            'channels': self.CHANNELS,
            'format': 'int16',
            'chunk_size': self.CHUNK,
            'bytes_per_sample': 2,  # 16-bit = 2 bytes
            'architecture': 'pure_online'
        }

    def test_recording(self) -> bool:
        """
        测试录制功能

        Returns:
            bool: 测试通过状态
        """
        logger.info("开始录制功能测试...")

        try:
            # 短时间录制测试
            success = self.start_recording(duration=0.5)
            if not success:
                logger.error("录制启动失败")
                return False

            time.sleep(0.6)  # 等待录制完成

            audio_data = self.stop_recording()

            # 验证录制结果
            if len(audio_data) == 0:
                logger.error("录制测试失败：没有数据")
                return False

            expected_samples = int(self.RATE * 0.5)  # 0.5秒的样本数
            actual_samples = len(audio_data)

            # 允许20%的误差（ALSA缓冲机制导致）
            if abs(actual_samples - expected_samples) / expected_samples > 0.2:
                logger.error(f"录制测试失败：样本数不符，期望{expected_samples}，实际{actual_samples}")
                return False

            logger.info(f"录制测试通过，样本数: {actual_samples}")
            return True

        except Exception as e:
            logger.error(f"录制测试异常: {e}")
            return False

    def __del__(self):
        """析构函数"""
        if self.is_recording:
            self.stop_recording()
        self._cleanup_stream()


# 便捷函数
def create_simple_recorder() -> SimpleAudioRecorder:
    """
    创建简单音频录制器实例

    Returns:
        SimpleAudioRecorder: 录制器实例
    """
    return SimpleAudioRecorder()


if __name__ == "__main__":
    # 测试代码
    print("=== Simple Audio Recorder 测试 ===")

    recorder = create_simple_recorder()

    # 显示配置
    config = recorder.get_audio_config()
    print("音频配置:")
    for key, value in config.items():
        print(f"  {key}: {value}")

    # 运行测试
    print("\n运行录制测试...")
    test_result = recorder.test_recording()
    print(f"测试结果: {'通过' if test_result else '失败'}")

    print("\n测试完成")