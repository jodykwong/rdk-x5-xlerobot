#!/usr/bin/env python3
"""
音频处理器单元测试 - Story 1.3

测试MVP版本的核心功能：
- PCM到WAV转换
- WAV到Base64编码
- 音频格式验证
- 音频信息获取

作者: Dev Agent
日期: 2025-11-09
Story: 1.3 - 基础语音识别 (阿里云ASR API集成)
"""

import unittest
import logging
import numpy as np
import base64
import wave
import io

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../src'))

from xlerobot.asr.audio_processor import AudioProcessor

logging.basicConfig(level=logging.INFO)


class TestAudioProcessor(unittest.TestCase):
    """音频处理器单元测试"""

    def setUp(self):
        """测试初始化"""
        self.processor = AudioProcessor(sample_rate=16000)

        # 创建测试音频数据（1秒440Hz正弦波）
        duration = 1.0
        sample_rate = 16000
        t = np.linspace(0, duration, int(sample_rate * duration))
        sine_wave = np.sin(2 * np.pi * 440 * t)
        self.test_pcm_data = (sine_wave * 32767).astype(np.int16).tobytes()

    def test_processor_initialization(self):
        """测试处理器初始化"""
        self.assertEqual(self.processor.sample_rate, 16000)
        self.assertEqual(self.processor.channels, 1)
        self.assertEqual(self.processor.sample_width, 2)

    def test_pcm_to_wav_conversion(self):
        """测试PCM到WAV转换"""
        wav_data = self.processor.pcm_to_wav(self.test_pcm_data, 16000)

        self.assertIsInstance(wav_data, bytes)
        self.assertGreater(len(wav_data), 44)  # WAV头部最少44字节

        # 验证WAV格式
        with io.BytesIO(wav_data) as wav_buffer:
            with wave.open(wav_buffer, 'rb') as wav_file:
                self.assertEqual(wav_file.getnchannels(), 1)
                self.assertEqual(wav_file.getframerate(), 16000)
                self.assertEqual(wav_file.getsampwidth(), 2)

    def test_pcm_to_wav_empty_data(self):
        """测试空PCM数据"""
        wav_data = self.processor.pcm_to_wav(b"", 16000)
        self.assertEqual(wav_data, b"")

    def test_pcm_to_wav_invalid_data(self):
        """测试无效PCM数据"""
        wav_data = self.processor.pcm_to_wav(None, 16000)
        self.assertEqual(wav_data, b"")

    def test_wav_to_base64_conversion(self):
        """测试WAV到Base64转换"""
        # 先创建WAV数据
        wav_data = self.processor.pcm_to_wav(self.test_pcm_data, 16000)

        # 转换为Base64
        base64_data = self.processor.wav_to_base64(wav_data)

        self.assertIsInstance(base64_data, str)
        self.assertGreater(len(base64_data), 0)

        # 验证Base64格式有效性
        try:
            decoded_data = base64.b64decode(base64_data)
            self.assertEqual(decoded_data, wav_data)
        except Exception:
            self.fail("Base64编码无效")

    def test_wav_to_base64_empty_data(self):
        """测试空WAV数据"""
        base64_data = self.processor.wav_to_base64(b"")
        self.assertEqual(base64_data, "")

    def test_convert_pcm_to_base64(self):
        """测试PCM直接转Base64"""
        base64_data = self.processor.convert_pcm_to_base64(self.test_pcm_data, 16000)

        self.assertIsInstance(base64_data, str)
        self.assertGreater(len(base64_data), 0)

    def test_validate_pcm_format(self):
        """测试PCM格式验证"""
        # 有效PCM数据（偶数长度）
        valid_pcm = b"\x00\x01" * 100
        self.assertTrue(self.processor.validate_audio_format(valid_pcm, "pcm"))

        # 无效PCM数据（奇数长度）
        invalid_pcm = b"\x00\x01\x00"
        self.assertFalse(self.processor.validate_audio_format(invalid_pcm, "pcm"))

        # 空数据
        self.assertFalse(self.processor.validate_audio_format(b"", "pcm"))

    def test_validate_wav_format(self):
        """测试WAV格式验证"""
        # 创建有效WAV数据
        wav_data = self.processor.pcm_to_wav(self.test_pcm_data, 16000)
        self.assertTrue(self.processor.validate_audio_format(wav_data, "wav"))

        # 无效WAV数据（太短）
        invalid_wav = b"RIFF\x24\x08\x00\x00WAVE"
        self.assertFalse(self.processor.validate_audio_format(invalid_wav, "wav"))

        # 空数据
        self.assertFalse(self.processor.validate_audio_format(b"", "wav"))

    def test_validate_unsupported_format(self):
        """测试不支持的格式验证"""
        self.assertFalse(self.processor.validate_audio_format(self.test_pcm_data, "mp3"))

    def test_get_pcm_info(self):
        """测试PCM音频信息获取"""
        info = self.processor.get_audio_info(self.test_pcm_data, "pcm")

        self.assertEqual(info["format"], "pcm")
        self.assertEqual(info["size_bytes"], len(self.test_pcm_data))
        self.assertGreater(info["duration_seconds"], 0.9)  # 约1秒
        self.assertLess(info["duration_seconds"], 1.1)

    def test_get_wav_info(self):
        """测试WAV音频信息获取"""
        wav_data = self.processor.pcm_to_wav(self.test_pcm_data, 16000)
        info = self.processor.get_audio_info(wav_data, "wav")

        self.assertEqual(info["format"], "wav")
        self.assertEqual(info["size_bytes"], len(wav_data))
        self.assertEqual(info["channels"], 1)
        self.assertEqual(info["sample_rate"], 16000)
        self.assertEqual(info["sample_width"], 2)
        self.assertGreater(info["duration_seconds"], 0.9)

    def test_get_empty_audio_info(self):
        """测试空音频信息获取"""
        info = self.processor.get_audio_info(b"", "pcm")
        self.assertIn("error", info)


if __name__ == "__main__":
    unittest.main()