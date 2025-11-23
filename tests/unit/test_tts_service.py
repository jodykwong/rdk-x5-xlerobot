#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
XleRobot Story 1.4 - TTS服务单元测试
BMad-Method v6 Brownfield Level 4 企业级实现
Story 1.4: 基础语音合成 (阿里云TTS API集成)

测试覆盖AC-001, AC-003, AC-006
"""

import unittest
import tempfile
import os
import sys
import numpy as np
from unittest.mock import Mock, patch, MagicMock
from pathlib import Path

# 添加项目路径
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root / 'src'))

from xlerobot.tts.aliyun_tts_client import AliyunTTSClient
from xlerobot.tts.audio_processor import AudioProcessor
from xlerobot.tts.error_handler import TTSErrorHandler, TTSErrorType, TTSRetryManager, TTSError


class TestAliyunTTSClient(unittest.TestCase):
    """阿里云TTS客户端测试"""

    def setUp(self):
        """测试设置"""
        self.config = {
            'app_key': 'test_app_key',
            'token': 'test_token',
            'region': 'cn-shanghai',
            'timeout': 10,
            'max_retries': 3
        }
        self.client = AliyunTTSClient(self.config)

    def test_client_initialization(self):
        """测试客户端初始化"""
        self.assertEqual(self.client.app_key, 'test_app_key')
        self.assertEqual(self.client.token, 'test_token')
        self.assertEqual(self.client.voice, 'jiajia')
        self.assertEqual(self.client.volume, 50)

    def test_invalid_config(self):
        """测试无效配置"""
        with self.assertRaises(ValueError):
            AliyunTTSClient({})  # 缺少app_key

    def test_synthesize_speech_success(self):
        """测试语音合成成功"""
        with patch.object(self.client, '_execute_request') as mock_execute:
            mock_execute.return_value = b'fake_wav_audio_data'

            result = self.client.synthesize_speech("测试文本")

            self.assertIsNotNone(result)
            mock_execute.assert_called_once()

    def test_synthesize_speech_failure(self):
        """测试语音合成失败"""
        with patch.object(self.client, '_execute_request') as mock_execute:
            mock_execute.return_value = None

            result = self.client.synthesize_speech("测试文本")

            self.assertIsNone(result)

    def test_parameter_validation(self):
        """测试参数验证"""
        # 测试有效参数 (不会触发调整)
        result = self.client.validate_parameters(speech_rate=50, volume=75, pitch_rate=25)
        self.assertEqual(result, {})  # 有效参数不会返回调整值

        # 测试无效参数 (会触发调整)
        result = self.client.validate_parameters(speech_rate=150, volume=200, pitch_rate=-10)
        self.assertEqual(result['speech_rate'], 100)  # 限制在0-100
        self.assertEqual(result['volume'], 150)   # 限制在50-150
        self.assertEqual(result['pitch_rate'], 0)    # 限制在0-100
        self.assertIn('speech_rate_warning', result)
        self.assertIn('volume_warning', result)
        self.assertIn('pitch_rate_warning', result)

    def test_supported_voices(self):
        """测试支持的发音人"""
        voices = self.client.get_supported_voices()
        self.assertIn('jiajia', voices)
        self.assertIsInstance(voices, dict)

    def test_connection_test(self):
        """测试连接检查"""
        with patch.object(self.client, 'synthesize_speech') as mock_synthesize:
            mock_synthesize.return_value = b'test_audio'

            result = self.client.test_connection()

            self.assertTrue(result)
            mock_synthesize.assert_called_once()


class TestAudioProcessor(unittest.TestCase):
    """音频处理器测试"""

    def setUp(self):
        """测试设置"""
        self.processor = AudioProcessor()

    def test_processor_initialization(self):
        """测试处理器初始化"""
        self.assertEqual(self.processor.wav_format, 1)
        self.assertEqual(self.processor.wav_sample_rate, 16000)
        self.assertEqual(self.processor.wav_channels, 1)
        self.assertEqual(self.processor.wav_bits_per_sample, 16)

    def test_base64_encoding_decoding(self):
        """测试Base64编解码"""
        # 测试数据
        test_data = b"test_audio_data"

        # 编码
        encoded = self.processor.encode_base64_audio(test_data)
        self.assertIsNotNone(encoded)

        # 解码
        decoded = self.processor.decode_base64_audio(encoded)
        self.assertEqual(decoded, test_data)

    def test_base64_decoding_invalid(self):
        """测试Base64解码无效数据"""
        invalid_data = "invalid_base64!"
        result = self.processor.decode_base64_audio(invalid_data)
        self.assertIsNone(result)

    def test_wav_validation_valid(self):
        """测试有效WAV验证"""
        # 创建简单的WAV文件头
        wav_header = self.processor.create_wav_header(100)
        wav_data = wav_header + b'\x00' * 100  # 添加音频数据

        result = self.processor.validate_wav_format(wav_data)

        self.assertTrue(result['valid'])
        self.assertEqual(result['format'], 'WAV')
        self.assertEqual(result['sample_rate'], 16000)
        self.assertEqual(result['channels'], 1)
        self.assertEqual(result['bits_per_sample'], 16)

    def test_wav_validation_invalid(self):
        """测试无效WAV验证"""
        # 无效的WAV数据
        invalid_data = b"invalid_wav_data"

        result = self.processor.validate_wav_format(invalid_data)

        self.assertFalse(result['valid'])
        self.assertIn('errors', result)

    def test_wav_conversion(self):
        """测试WAV格式转换"""
        import numpy as np

        # 创建测试音频数据
        test_audio = np.random.randint(-32768, 32767, 1000, dtype=np.int16)

        result = self.processor.convert_to_wav(test_audio)

        self.assertIsNotNone(result)
        self.assertTrue(result.startswith(b'RIFF'))
        self.assertEqual(len(result), 44 + len(test_audio) * 2)  # 头部 + 数据

    def test_audio_quality_analysis(self):
        """测试音频质量分析"""
        # 创建测试WAV文件
        test_audio = np.random.randint(-32768, 32767, 16000, dtype=np.int16)  # 1秒音频
        wav_data = self.processor.convert_to_wav(test_audio)

        result = self.processor.analyze_audio_quality(wav_data)

        self.assertIn('clarity_score', result)
        self.assertIn('volume_level', result)
        self.assertIn('noise_level', result)
        self.assertIn('quality_rating', result)
        self.assertEqual(result['duration'], 1.0)

    def test_audio_file_operations(self):
        """测试音频文件操作"""
        test_data = b"test_audio_content"

        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
            temp_path = temp_file.name

        # 保存文件
        success = self.processor.save_audio_file(test_data, temp_path)
        self.assertTrue(success)

        # 加载文件
        loaded_data = self.processor.load_audio_file(temp_path)
        self.assertEqual(loaded_data, test_data)


class TestTTSErrorHandler(unittest.TestCase):
    """TTS错误处理器测试"""

    def setUp(self):
        """测试设置"""
        self.error_handler = TTSErrorHandler()

    def test_error_classification(self):
        """测试错误分类"""
        # 网络错误
        network_error = ConnectionError("网络连接失败")
        tts_error = self.error_handler.classify_error(network_error)

        self.assertEqual(tts_error.error_type, TTSErrorType.NETWORK_ERROR)
        self.assertTrue(tts_error.can_retry)

        # 超时错误
        timeout_error = TimeoutError("请求超时")
        tts_error = self.error_handler.classify_error(timeout_error)

        self.assertEqual(tts_error.error_type, TTSErrorType.TIMEOUT_ERROR)
        self.assertTrue(tts_error.can_retry)

    def test_http_error_classification(self):
        """测试HTTP错误分类"""
        # 400错误
        tts_error = self.error_handler.classify_error(Exception("Bad Request"), 400)
        self.assertEqual(tts_error.error_type, TTSErrorType.INVALID_REQUEST)
        self.assertFalse(tts_error.can_retry)

        # 429错误
        tts_error = self.error_handler.classify_error(Exception("Too Many Requests"), 429)
        self.assertEqual(tts_error.error_type, TTSErrorType.API_LIMIT_ERROR)
        self.assertTrue(tts_error.can_retry)

        # 500错误
        tts_error = self.error_handler.classify_error(Exception("Internal Server Error"), 500)
        self.assertEqual(tts_error.error_type, TTSErrorType.SERVER_ERROR)
        self.assertTrue(tts_error.can_retry)

    def test_retry_delay_calculation(self):
        """测试重试延迟计算"""
        # 网络错误 - 可以重试
        network_error = TTSError(
            error_type=TTSErrorType.NETWORK_ERROR,
            error_code=None,
            error_message="网络错误",
            user_message="网络连接异常",
            can_retry=True
        )

        delay1 = self.error_handler.get_retry_delay(network_error, 0)
        delay2 = self.error_handler.get_retry_delay(network_error, 1)
        delay3 = self.error_handler.get_retry_delay(network_error, 2)

        self.assertEqual(delay1, 1.0)  # 基础延迟
        self.assertEqual(delay2, 2.0)  # 指数退避 2^1
        self.assertEqual(delay3, 4.0)  # 指数退避 2^2

    def test_can_retry_logic(self):
        """测试重试逻辑"""
        # 可重试错误
        retryable_error = TTSError(
            error_type=TTSErrorType.NETWORK_ERROR,
            error_code=None,
            error_message="网络错误",
            user_message="网络连接异常",
            can_retry=True
        )

        # 第一次尝试
        self.assertTrue(self.error_handler.can_retry(retryable_error, 0))
        # 超过最大重试次数
        self.assertFalse(self.error_handler.can_retry(retryable_error, 3))

        # 不可重试错误
        non_retryable_error = TTSError(
            error_type=TTSErrorType.INVALID_REQUEST,
            error_code=None,
            error_message="无效请求",
            user_message="请求参数错误",
            can_retry=False
        )

        self.assertFalse(self.error_handler.can_retry(non_retryable_error, 0))
        self.assertFalse(self.error_handler.can_retry(non_retryable_error, 1))


class TestAudioQualityControl(unittest.TestCase):
    """音频质量控制测试"""

    def setUp(self):
        """测试设置"""
        self.processor = AudioProcessor()
        # 创建测试WAV音频数据
        test_audio = np.random.randint(-32768, 32767, 8000, dtype=np.int16)  # 0.5秒测试音频
        self.test_audio_data = self.processor.convert_to_wav(test_audio)

    def test_speech_rate_adjustment(self):
        """测试语速调节"""
        # 测试正常语速调节
        result = self.processor.adjust_speech_rate(self.test_audio_data, 1.0)
        self.assertIsNotNone(result)

        # 测试语速加快
        result = self.processor.adjust_speech_rate(self.test_audio_data, 1.2)
        self.assertIsNotNone(result)

        # 测试语速减慢
        result = self.processor.adjust_speech_rate(self.test_audio_data, 0.8)
        self.assertIsNotNone(result)

    def test_pitch_rate_adjustment(self):
        """测试音调调节"""
        # 测试正常音调
        result = self.processor.adjust_pitch_rate(self.test_audio_data, 1.0)
        self.assertIsNotNone(result)

        # 测试音调升高
        result = self.processor.adjust_pitch_rate(self.test_audio_data, 1.2)
        self.assertIsNotNone(result)

        # 测试音调降低
        result = self.processor.adjust_pitch_rate(self.test_audio_data, 0.8)
        self.assertIsNotNone(result)

    def test_volume_adjustment(self):
        """测试音量调节"""
        # 测试正常音量
        result = self.processor.adjust_volume(self.test_audio_data, 1.0)
        self.assertIsNotNone(result)

        # 测试音量增大
        result = self.processor.adjust_volume(self.test_audio_data, 1.5)
        self.assertIsNotNone(result)

        # 测试音量减小
        result = self.processor.adjust_volume(self.test_audio_data, 0.5)
        self.assertIsNotNone(result)

    def test_emotion_style_application(self):
        """测试情感语音应用"""
        # 测试友好情感
        result = self.processor.apply_emotion_style(self.test_audio_data, "friendly")
        self.assertIsNotNone(result)

        # 测试确认情感
        result = self.processor.apply_emotion_style(self.test_audio_data, "confirm")
        self.assertIsNotNone(result)

        # 测试错误情感
        result = self.processor.apply_emotion_style(self.test_audio_data, "error")
        self.assertIsNotNone(result)

        # 测试未知情感 (应该默认为friendly)
        result = self.processor.apply_emotion_style(self.test_audio_data, "unknown")
        self.assertIsNotNone(result)

    def test_audio_quality_enhancement(self):
        """测试音频质量增强"""
        result = self.processor.enhance_audio_quality(self.test_audio_data)
        self.assertIsNotNone(result)

        # 验证增强后的音频格式仍然有效
        validation = self.processor.validate_wav_format(result)
        self.assertTrue(validation['valid'])

    def test_audio_quality_evaluation(self):
        """测试音频质量评估"""
        result = self.processor.evaluate_audio_quality(self.test_audio_data)

        # 验证评估结果包含必要字段
        self.assertIn('quality_score', result)
        self.assertIn('quality_rating', result)
        self.assertIn('snr_db', result)
        self.assertIn('dynamic_range_db', result)
        self.assertIn('rms_level', result)

        # 验证质量评分范围
        if 'quality_score' in result:
            self.assertGreaterEqual(result['quality_score'], 0)
            self.assertLessEqual(result['quality_score'], 100)

    def test_parameter_boundary_values(self):
        """测试参数边界值"""
        # 测试语速边界值
        result = self.processor.adjust_speech_rate(self.test_audio_data, 0.7)  # 低于最小值
        self.assertIsNotNone(result)

        result = self.processor.adjust_speech_rate(self.test_audio_data, 1.3)  # 高于最大值
        self.assertIsNotNone(result)

        # 测试音调边界值
        result = self.processor.adjust_pitch_rate(self.test_audio_data, 0.7)
        self.assertIsNotNone(result)

        result = self.processor.adjust_pitch_rate(self.test_audio_data, 1.3)
        self.assertIsNotNone(result)

        # 测试音量边界值
        result = self.processor.adjust_volume(self.test_audio_data, 0.4)
        self.assertIsNotNone(result)

        result = self.processor.adjust_volume(self.test_audio_data, 1.6)
        self.assertIsNotNone(result)


class TestTTSRetryManager(unittest.TestCase):
    """TTS重试管理器测试"""

    def setUp(self):
        """测试设置"""
        self.retry_manager = TTSRetryManager(max_total_retries=3)

    def test_successful_execution(self):
        """测试成功执行"""
        def successful_func():
            return "success"

        result = self.retry_manager.execute_with_retry(successful_func)
        self.assertEqual(result, "success")

    def test_retry_on_failure(self):
        """测试失败重试"""
        call_count = 0

        def failing_func():
            nonlocal call_count
            call_count += 1
            if call_count < 3:
                raise ConnectionError("模拟失败")
            return "success"

        result = self.retry_manager.execute_with_retry(failing_func)
        self.assertEqual(result, "success")
        self.assertEqual(call_count, 3)

    def test_max_retries_exceeded(self):
        """测试超过最大重试次数"""
        def always_failing_func():
            raise ConnectionError("总是失败")

        with self.assertRaises(Exception):
            self.retry_manager.execute_with_retry(always_failing_func)


if __name__ == '__main__':
    # 配置日志
    import logging
    logging.basicConfig(level=logging.INFO)

    # 运行测试
    unittest.main(verbosity=2)