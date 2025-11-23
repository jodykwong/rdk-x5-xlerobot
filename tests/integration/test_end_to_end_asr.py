#!/usr/bin/env python3
"""
端到端ASR集成测试 - Story 1.3

测试MVP版本的完整流程：
- 音频处理到识别的完整流程
- 配置管理集成
- 识别服务集成
- 错误处理流程

作者: Dev Agent
日期: 2025-11-09
Story: 1.3 - 基础语音识别 (阿里云ASR API集成)
"""

import unittest
import logging
import numpy as np
import time
import os
from unittest.mock import patch, MagicMock

import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../src'))

from xlerobot.asr.recognition_service import RecognitionService, RecognitionRequest
from xlerobot.common.config_manager import ConfigManager, ASRServiceConfig

logging.basicConfig(level=logging.INFO)


class TestEndToEndASR(unittest.TestCase):
    """端到端ASR集成测试"""

    def setUp(self):
        """测试初始化"""
        # 创建测试配置
        self.test_config = ASRServiceConfig(
            app_key="test_key",
            app_secret="test_secret",
            region="cn-shanghai"
        )

        # 创建识别服务
        self.recognition_service = RecognitionService(
            app_key=self.test_config.app_key,
            app_secret=self.test_config.app_secret,
            region=self.test_config.region
        )

        # 创建测试音频数据（1秒440Hz正弦波）
        duration = 1.0
        sample_rate = 16000
        t = np.linspace(0, duration, int(sample_rate * duration))
        sine_wave = np.sin(2 * np.pi * 440 * t)
        self.test_pcm_data = (sine_wave * 32767).astype(np.int16).tobytes()

    def test_config_manager_integration(self):
        """测试配置管理集成"""
        # 设置环境变量
        os.environ["ALIYUN_NLS_APP_KEY"] = "test_env_key"
        os.environ["ALIYUN_NLS_APP_SECRET"] = "test_env_secret"

        # 创建配置管理器
        config_manager = ConfigManager()
        config = config_manager.get_config()

        self.assertEqual(config.app_key, "test_env_key")
        self.assertEqual(config.app_secret, "test_env_secret")

        # 验证配置
        config_valid = config_manager.validate_config()
        self.assertTrue(config_valid)

        # 清理环境变量
        del os.environ["ALIYUN_NLS_APP_KEY"]
        del os.environ["ALIYUN_NLS_APP_SECRET"]

    def test_recognition_service_initialization(self):
        """测试识别服务初始化"""
        self.assertIsNotNone(self.recognition_service)
        self.assertEqual(self.recognition_service.total_requests, 0)
        self.assertEqual(self.recognition_service.successful_requests, 0)

    @patch.object(RecognitionService, 'asr_client')
    def test_successful_recognition_flow(self, mock_asr_client):
        """测试成功的识别流程"""
        # 模拟ASR客户端响应
        mock_result = MagicMock()
        mock_result.text = "打开灯光"
        mock_result.confidence = 0.95
        mock_result.status_code = 200
        mock_result.message = ""
        mock_asr_client.recognize_speech.return_value = mock_result

        # 创建识别请求
        request = RecognitionRequest(
            audio_data=self.test_pcm_data,
            format="pcm",
            sample_rate=16000
        )

        # 执行识别
        response = self.recognition_service.recognize_speech(request)

        # 验证结果
        self.assertTrue(response.success)
        self.assertEqual(response.text, "打开灯光")
        self.assertEqual(response.confidence, 0.95)
        self.assertEqual(response.error_message, "")
        self.assertLess(response.processing_time, 1.0)

        # 验证统计信息
        stats = self.recognition_service.get_statistics()
        self.assertEqual(stats["total_requests"], 1)
        self.assertEqual(stats["successful_requests"], 1)
        self.assertEqual(stats["success_rate"], 1.0)

    @patch.object(RecognitionService, 'asr_client')
    def test_failed_recognition_flow(self, mock_asr_client):
        """测试失败的识别流程"""
        # 模拟ASR客户端错误响应
        mock_result = MagicMock()
        mock_result.text = ""
        mock_result.confidence = 0.0
        mock_result.status_code = 400
        mock_result.message = "请求参数错误"
        mock_asr_client.recognize_speech.return_value = mock_result

        # 创建识别请求
        request = RecognitionRequest(
            audio_data=self.test_pcm_data,
            format="pcm",
            sample_rate=16000
        )

        # 执行识别
        response = self.recognition_service.recognize_speech(request)

        # 验证结果
        self.assertFalse(response.success)
        self.assertEqual(response.text, "")
        self.assertEqual(response.confidence, 0.0)
        self.assertIn("请求参数错误", response.error_message)

        # 验证统计信息
        stats = self.recognition_service.get_statistics()
        self.assertEqual(stats["total_requests"], 1)
        self.assertEqual(stats["successful_requests"], 0)
        self.assertEqual(stats["success_rate"], 0.0)

    def test_invalid_audio_format(self):
        """测试无效音频格式"""
        # 创建无效音频数据（奇数长度）
        invalid_audio = b"\x00\x01\x00"

        request = RecognitionRequest(
            audio_data=invalid_audio,
            format="pcm",
            sample_rate=16000
        )

        # 执行识别
        response = self.recognition_service.recognize_speech(request)

        # 验证结果
        self.assertFalse(response.success)
        self.assertIn("音频格式无效", response.error_message)

    def test_unsupported_audio_format(self):
        """测试不支持的音频格式"""
        request = RecognitionRequest(
            audio_data=self.test_pcm_data,
            format="mp3",  # 不支持的格式
            sample_rate=16000
        )

        # 执行识别
        response = self.recognition_service.recognize_speech(request)

        # 验证结果
        self.assertFalse(response.success)
        self.assertIn("不支持的音频格式", response.error_message)

    @patch.object(RecognitionService, 'asr_client')
    def test_api_exception_handling(self, mock_asr_client):
        """测试API异常处理"""
        # 模拟ASR客户端抛出异常
        mock_asr_client.recognize_speech.side_effect = Exception("网络连接失败")

        request = RecognitionRequest(
            audio_data=self.test_pcm_data,
            format="pcm",
            sample_rate=16000
        )

        # 执行识别
        response = self.recognition_service.recognize_speech(request)

        # 验证结果
        self.assertFalse(response.success)
        self.assertIn("识别异常", response.error_message)
        self.assertIn("网络连接失败", response.error_message)

    def test_service_test_functionality(self):
        """测试服务测试功能"""
        with patch.object(self.recognition_service, 'asr_client') as mock_client:
            # 模拟测试通过
            mock_client.test_connection.return_value = True
            test_result = self.recognition_service.test_service()
            self.assertTrue(test_result)

            # 模拟测试失败
            mock_client.test_connection.return_value = False
            test_result = self.recognition_service.test_service()
            self.assertFalse(test_result)

    def test_multiple_requests_statistics(self):
        """测试多个请求的统计信息"""
        with patch.object(self.recognition_service, 'asr_client') as mock_client:
            # 模拟混合成功/失败结果
            mock_client.recognize_speech.side_effect = [
                MagicMock(text="成功1", confidence=0.9, status_code=200, message=""),
                MagicMock(text="", confidence=0.0, status_code=400, message="错误"),
                MagicMock(text="成功2", confidence=0.8, status_code=200, message="")
            ]

            request = RecognitionRequest(
                audio_data=self.test_pcm_data,
                format="pcm",
                sample_rate=16000
            )

            # 执行多个请求
            for _ in range(3):
                self.recognition_service.recognize_speech(request)

            # 验证统计信息
            stats = self.recognition_service.get_statistics()
            self.assertEqual(stats["total_requests"], 3)
            self.assertEqual(stats["successful_requests"], 2)
            self.assertAlmostEqual(stats["success_rate"], 2/3, places=2)

    def test_performance_timing(self):
        """测试性能计时"""
        with patch.object(self.recognition_service, 'asr_client') as mock_client:
            # 模拟一定的处理时间
            def delayed_recognition(*args, **kwargs):
                time.sleep(0.1)  # 模拟100ms处理时间
                return MagicMock(text="测试", confidence=0.9, status_code=200, message="")

            mock_client.recognize_speech.side_effect = delayed_recognition

            request = RecognitionRequest(
                audio_data=self.test_pcm_data,
                format="pcm",
                sample_rate=16000
            )

            # 执行识别
            start_time = time.time()
            response = self.recognition_service.recognize_speech(request)
            end_time = time.time()

            # 验证结果
            self.assertTrue(response.success)
            self.assertGreater(response.processing_time, 0.1)
            self.assertGreater(end_time - start_time, 0.1)


if __name__ == "__main__":
    unittest.main()