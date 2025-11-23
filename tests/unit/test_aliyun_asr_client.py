#!/usr/bin/env python3
"""
阿里云ASR客户端单元测试 - Story 1.3

测试MVP版本的核心功能：
- ASR配置管理
- 音频格式转换
- API请求构建
- 响应解析
- 错误处理

作者: Dev Agent
日期: 2025-11-09
Story: 1.3 - 基础语音识别 (阿里云ASR API集成)
"""

import unittest
import logging
import numpy as np
from unittest.mock import patch, MagicMock

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../src'))

from xlerobot.asr.aliyun_asr_client import AliyunASRClient, ASRConfig, ASRResult

logging.basicConfig(level=logging.INFO)


class TestAliyunASRClient(unittest.TestCase):
    """阿里云ASR客户端单元测试"""

    def setUp(self):
        """测试初始化"""
        self.config = ASRConfig(
            app_key="test_key",
            app_secret="test_secret",
            region="cn-shanghai"
        )
        self.client = AliyunASRClient(self.config)

    def test_client_initialization(self):
        """测试客户端初始化"""
        self.assertEqual(self.client.config.app_key, "test_key")
        self.assertEqual(self.client.config.region, "cn-shanghai")
        self.assertEqual(self.client.max_retries, 3)

    def test_convert_to_base64(self):
        """测试Base64转换"""
        test_data = b"test_audio_data"
        base64_result = self.client._convert_to_base64(test_data)

        self.assertIsInstance(base64_result, str)
        self.assertTrue(len(base64_result) > 0)

    def test_parse_response_success(self):
        """测试成功响应解析"""
        mock_response = {
            "status_code": 200,
            "result": {
                "text": "测试识别结果",
                "sentences": [{"confidence": 85.0}]
            }
        }

        result = self.client._parse_response(mock_response)

        self.assertIsInstance(result, ASRResult)
        self.assertEqual(result.text, "测试识别结果")
        self.assertEqual(result.confidence, 0.85)
        self.assertEqual(result.status_code, 200)

    def test_parse_response_error(self):
        """测试错误响应解析"""
        mock_response = {
            "status_code": 400,
            "message": "请求参数错误"
        }

        result = self.client._parse_response(mock_response)

        self.assertEqual(result.text, "")
        self.assertEqual(result.status_code, 400)
        self.assertEqual(result.message, "请求参数错误")

    @patch('requests.post')
    def test_make_request_success(self, mock_post):
        """测试成功请求"""
        mock_response = MagicMock()
        mock_response.status_code = 200
        mock_response.json.return_value = {"status": "success"}
        mock_post.return_value = mock_response

        result = self.client._make_request_with_retry("http://test.com", {})

        self.assertIsNotNone(result)
        self.assertEqual(result["status"], "success")

    @patch('requests.post')
    def test_make_request_with_retry(self, mock_post):
        """测试请求重试机制"""
        # 模拟前两次失败，第三次成功
        mock_post.side_effect = [
            Exception("网络错误"),
            Exception("网络错误"),
            MagicMock(status_code=200, json=lambda: {"status": "success"})
        ]

        result = self.client._make_request_with_retry("http://test.com", {})

        self.assertIsNotNone(result)
        self.assertEqual(result["status"], "success")
        self.assertEqual(mock_post.call_count, 3)

    @patch('requests.post')
    def test_make_request_final_failure(self, mock_post):
        """测试请求最终失败"""
        mock_post.side_effect = Exception("网络错误")

        result = self.client._make_request_with_retry("http://test.com", {})

        self.assertIsNone(result)
        self.assertEqual(mock_post.call_count, 3)

    @patch.object(AliyunASRClient, '_make_request_with_retry')
    def test_recognize_speech_success(self, mock_request):
        """测试语音识别成功"""
        mock_request.return_value = {
            "status_code": 200,
            "result": {
                "text": "打开灯光",
                "sentences": [{"confidence": 90.0}]
            }
        }

        test_audio = b"test_pcm_data"
        result = self.client.recognize_speech(test_audio)

        self.assertIsInstance(result, ASRResult)
        self.assertEqual(result.text, "打开灯光")
        self.assertEqual(result.confidence, 0.9)

    @patch.object(AliyunASRClient, '_make_request_with_retry')
    def test_recognize_speech_failure(self, mock_request):
        """测试语音识别失败"""
        mock_request.return_value = None

        test_audio = b"test_pcm_data"
        result = self.client.recognize_speech(test_audio)

        self.assertIsNone(result)

    def test_recognize_speech_exception(self):
        """测试语音识别异常处理"""
        # 模拟异常
        with patch.object(self.client, '_convert_to_base64', side_effect=Exception("转换失败")):
            test_audio = b"test_pcm_data"
            result = self.client.recognize_speech(test_audio)

            self.assertIsInstance(result, ASRResult)
            self.assertEqual(result.status_code, 500)
            self.assertIn("转换失败", result.message)


if __name__ == "__main__":
    unittest.main()