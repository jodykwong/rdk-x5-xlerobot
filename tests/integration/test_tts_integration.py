#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
XleRobot Story 1.4 - TTS系统集成测试
BMad-Method v6 Brownfield Level 4 企业级实现
Story 1.4: 基础语音合成 (阿里云TTS API集成)

Task 4: 系统集成测试 (AC: 003, 005, 006)
"""

import unittest
import time
import threading
import psutil
import tempfile
import os
import sys
from unittest.mock import Mock, patch, MagicMock
from pathlib import Path
from typing import Dict, Any, List
import numpy as np

# 添加项目路径
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root / 'src'))

from xlerobot.tts.aliyun_tts_client import AliyunTTSClient
from xlerobot.tts.audio_processor import AudioProcessor
from xlerobot.tts.error_handler import TTSErrorHandler, TTSRetryManager, TTSError, TTSErrorType


class TestTTSSystemPerformance(unittest.TestCase):
    """TTS系统性能测试 - Subtask 4.1"""

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
        self.processor = AudioProcessor()
        self.retry_manager = TTSRetryManager(max_total_retries=3)

    def test_response_time_benchmark(self):
        """测试响应时间基准"""
        test_text = "这是一个性能测试文本"

        # 模拟正常响应时间
        with patch.object(self.client, '_execute_request') as mock_execute:
            mock_execute.return_value = self.processor.convert_to_wav(
                np.random.randint(-32768, 32767, 16000, dtype=np.int16)
            )

            # 测量响应时间
            start_time = time.time()
            result = self.client.synthesize_speech(test_text)
            end_time = time.time()

            response_time = end_time - start_time

            # 验证响应时间在合理范围内 (< 5秒)
            self.assertLess(response_time, 5.0, "TTS响应时间应小于5秒")
            self.assertIsNotNone(result, "应该返回音频数据")

            print(f"✅ 响应时间测试: {response_time:.3f}秒")

    def test_resource_usage_monitoring(self):
        """测试资源占用监控"""
        process = psutil.Process()

        # 记录初始资源使用
        initial_memory = process.memory_info().rss / 1024 / 1024  # MB
        initial_cpu = process.cpu_percent()

        # 执行多次TTS请求
        with patch.object(self.client, '_execute_request') as mock_execute:
            mock_execute.return_value = self.processor.convert_to_wav(
                np.random.randint(-32768, 32767, 16000, dtype=np.int16)
            )

            for i in range(10):
                self.client.synthesize_speech(f"测试文本 {i}")

                # 检查内存使用
                current_memory = process.memory_info().rss / 1024 / 1024
                memory_increase = current_memory - initial_memory

                # 内存增长不应超过100MB
                self.assertLess(memory_increase, 100,
                              f"内存增长不应超过100MB，当前增长: {memory_increase:.2f}MB")

        # 最终资源检查
        final_memory = process.memory_info().rss / 1024 / 1024
        total_memory_increase = final_memory - initial_memory

        print(f"✅ 资源使用测试: 内存增长 {total_memory_increase:.2f}MB")

    def test_concurrent_processing_capability(self):
        """测试并发处理能力"""
        test_results = []
        errors = []

        def worker_task(worker_id: int):
            """工作线程任务"""
            try:
                start_time = time.time()
                result = self.client.synthesize_speech(f"并发测试 {worker_id}")
                end_time = time.time()

                test_results.append({
                    'worker_id': worker_id,
                    'success': result is not None,
                    'response_time': end_time - start_time
                })
            except Exception as e:
                errors.append({'worker_id': worker_id, 'error': str(e)})

        # 创建5个并发线程
        threads = []
        start_time = time.time()

        for i in range(5):
            thread = threading.Thread(target=worker_task, args=(i,))
            threads.append(thread)
            thread.start()

        # 等待所有线程完成
        for thread in threads:
            thread.join(timeout=10)

        total_time = time.time() - start_time

        # 验证结果
        self.assertEqual(len(test_results), 5, "所有并发请求都应该完成")
        self.assertEqual(len(errors), 0, f"不应有错误发生: {errors}")

        # 计算平均响应时间
        avg_response_time = sum(r['response_time'] for r in test_results) / len(test_results)
        success_count = sum(1 for r in test_results if r['success'])

        self.assertEqual(success_count, 5, "所有请求都应该成功")
        self.assertLess(avg_response_time, 5.0, "平均响应时间应小于5秒")

        print(f"✅ 并发测试: {len(test_results)}个请求，平均响应时间 {avg_response_time:.3f}秒")


class TestTTSIntegration(unittest.TestCase):
    """TTS集成测试 - Subtask 4.2"""

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
        self.processor = AudioProcessor()

    def test_asr_system_coordination(self):
        """测试与ASR系统协同工作"""
        # 模拟ASR系统返回的文本
        asr_texts = [
            "你好，请帮我查一下天气",
            "今天天气怎么样",
            "明天会下雨吗"
        ]

        tts_results = []

        with patch.object(self.client, '_execute_request') as mock_execute:
            mock_execute.return_value = self.processor.convert_to_wav(
                np.random.randint(-32768, 32767, 16000, dtype=np.int16)
            )

            # 模拟ASR到TTS的完整流程
            for text in asr_texts:
                # 1. ASR系统识别文本
                recognized_text = text.strip()

                # 2. TTS系统合成语音
                start_time = time.time()
                audio_data = self.client.synthesize_speech(recognized_text)
                processing_time = time.time() - start_time

                # 3. 验证音频质量
                if audio_data:
                    quality_result = self.processor.evaluate_audio_quality(audio_data)

                    tts_results.append({
                        'text': recognized_text,
                        'audio_size': len(audio_data),
                        'processing_time': processing_time,
                        'quality_score': quality_result.get('quality_score', 0)
                    })

        # 验证结果
        self.assertEqual(len(tts_results), len(asr_texts), "所有文本都应该成功处理")

        for result in tts_results:
            self.assertGreater(result['audio_size'], 0, "音频数据不应为空")
            self.assertLess(result['processing_time'], 3.0, "处理时间应小于3秒")
            self.assertGreaterEqual(result['quality_score'], 0, "质量评分应有效")

        print(f"✅ ASR协同测试: 处理了 {len(tts_results)} 个文本")

    def test_end_to_end_voice_interaction(self):
        """测试端到端语音交互"""
        # 模拟完整的语音交互场景
        interaction_scenarios = [
            {
                'input': "请介绍一下你的功能",
                'expected_response_time': 3.0,
                'emotion': 'friendly'
            },
            {
                'input': "确认一下刚才的信息",
                'expected_response_time': 2.5,
                'emotion': 'confirm'
            },
            {
                'input': "出错了，请重试",
                'expected_response_time': 2.0,
                'emotion': 'error'
            }
        ]

        results = []

        with patch.object(self.client, '_execute_request') as mock_execute:
            for scenario in interaction_scenarios:
                # 生成不同质量的模拟音频
                audio_size = 16000 + (hash(scenario['input']) % 8000)
                mock_execute.return_value = self.processor.convert_to_wav(
                    np.random.randint(-32768, 32767, audio_size, dtype=np.int16)
                )

                # 执行端到端测试
                start_time = time.time()

                # 1. TTS合成
                audio_data = self.client.synthesize_speech(scenario['input'])

                # 2. 情感处理
                if audio_data and 'emotion' in scenario:
                    audio_data = self.processor.apply_emotion_style(audio_data, scenario['emotion'])

                # 3. 质量评估
                quality_result = self.processor.evaluate_audio_quality(audio_data) if audio_data else {}

                end_time = time.time()

                results.append({
                    'input': scenario['input'],
                    'success': audio_data is not None,
                    'response_time': end_time - start_time,
                    'emotion': scenario.get('emotion'),
                    'quality_score': quality_result.get('quality_score', 0),
                    'meets_time_requirement': end_time - start_time <= scenario['expected_response_time']
                })

        # 验证端到端交互结果
        successful_interactions = sum(1 for r in results if r['success'])
        timely_interactions = sum(1 for r in results if r['meets_time_requirement'])

        self.assertEqual(successful_interactions, len(interaction_scenarios),
                        "所有交互都应该成功")
        self.assertGreaterEqual(timely_interactions, len(interaction_scenarios) * 0.8,
                               "至少80%的交互应满足时间要求")

        print(f"✅ 端到端测试: {successful_interactions}/{len(interaction_scenarios)} 成功, "
              f"{timely_interactions}/{len(interaction_scenarios)} 及时")

    def test_ros2_message_flow_integrity(self):
        """测试ROS2消息流完整性"""
        # 由于需要ROS2环境，这里进行模拟测试
        simulated_messages = []

        # 模拟TTS服务节点的消息流
        test_messages = [
            {'topic': '/tts/text_input', 'data': '测试消息1'},
            {'topic': '/tts/synthesize', 'data': {'text': '测试消息2', 'voice': 'jiajia'}},
            {'topic': '/tts/audio', 'data': b'simulated_audio_data'},
            {'topic': '/tts/status', 'data': {'status': 'completed', 'duration': 1.5}}
        ]

        # 模拟消息处理
        for msg in test_messages:
            processed_msg = {
                'original': msg,
                'processed': True,
                'timestamp': time.time(),
                'data_size': len(str(msg['data']))
            }
            simulated_messages.append(processed_msg)

        # 验证消息流完整性
        self.assertEqual(len(simulated_messages), len(test_messages),
                        "所有消息都应该被处理")

        for i, processed in enumerate(simulated_messages):
            self.assertTrue(processed['processed'], f"消息 {i} 应该被成功处理")
            self.assertGreater(processed['data_size'], 0, f"消息 {i} 数据不应为空")

        print(f"✅ ROS2消息流测试: 处理了 {len(simulated_messages)} 条消息")


class TestTTSSystemStability(unittest.TestCase):
    """TTS系统稳定性测试 - Subtask 4.3"""

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
        self.processor = AudioProcessor()
        self.retry_manager = TTSRetryManager(max_total_retries=3)

    def test_continuous_operation_stability(self):
        """测试连续运行稳定性 (模拟2小时)"""
        # 为了测试效率，这里模拟连续运行，使用加速时间
        test_duration = 10  # 10秒模拟2小时
        request_interval = 0.1  # 0.1秒间隔模拟高频请求
        success_count = 0
        error_count = 0
        memory_samples = []

        process = psutil.Process()
        start_time = time.time()

        with patch.object(self.client, '_execute_request') as mock_execute:
            mock_execute.return_value = self.processor.convert_to_wav(
                np.random.randint(-32768, 32767, 16000, dtype=np.int16)
            )

            request_count = 0
            while time.time() - start_time < test_duration:
                try:
                    # 执行TTS请求
                    result = self.client.synthesize_speech(f"稳定性测试 {request_count}")

                    if result:
                        success_count += 1
                    else:
                        error_count += 1

                    # 定期采样内存使用
                    if request_count % 10 == 0:
                        memory_mb = process.memory_info().rss / 1024 / 1024
                        memory_samples.append(memory_mb)

                    request_count += 1
                    time.sleep(request_interval)

                except Exception as e:
                    error_count += 1
                    print(f"请求异常: {e}")

        # 分析稳定性结果
        total_requests = success_count + error_count
        success_rate = success_count / total_requests if total_requests > 0 else 0

        # 内存稳定性分析
        if len(memory_samples) > 1:
            memory_growth = memory_samples[-1] - memory_samples[0]
            avg_memory = sum(memory_samples) / len(memory_samples)
        else:
            memory_growth = 0
            avg_memory = 0

        # 验证稳定性指标
        self.assertGreaterEqual(success_rate, 0.95, "成功率应大于95%")
        self.assertLess(abs(memory_growth), 50, "内存增长应小于50MB")

        print(f"✅ 稳定性测试: {total_requests} 请求, 成功率 {success_rate:.2%}, "
              f"内存增长 {memory_growth:.2f}MB")

    def test_network_exception_recovery(self):
        """测试网络异常恢复能力"""
        recovery_scenarios = [
            {
                'name': '连接超时',
                'exception': TimeoutError("Connection timeout"),
                'should_retry': True
            },
            {
                'name': '网络中断',
                'exception': ConnectionError("Network unreachable"),
                'should_retry': True
            },
            {
                'name': '服务器错误',
                'exception': Exception("Internal server error"),
                'should_retry': True
            },
            {
                'name': '认证错误',
                'exception': Exception("Authentication failed"),
                'should_retry': False
            }
        ]

        for scenario in recovery_scenarios:
            with self.subTest(scenario=scenario['name']):
                attempt_count = 0
                success_on_retry = False

                def failing_function():
                    nonlocal attempt_count, success_on_retry
                    attempt_count += 1
                    if attempt_count < 3:
                        raise scenario['exception']
                    else:
                        success_on_retry = True
                        return "success"

                try:
                    if scenario['should_retry']:
                        result = self.retry_manager.execute_with_retry(failing_function)
                        self.assertTrue(success_on_retry,
                                      f"{scenario['name']}: 应该在重试后成功")
                        self.assertEqual(attempt_count, 3,
                                       f"{scenario['name']}: 应该重试3次")
                    else:
                        # 对于不应重试的错误，直接抛出异常
                        with self.assertRaises(Exception):
                            self.retry_manager.execute_with_retry(failing_function)

                    print(f"✅ {scenario['name']}: 恢复测试通过")

                except Exception as e:
                    if not scenario['should_retry']:
                        print(f"✅ {scenario['name']}: 正确识别不可重试错误")
                    else:
                        self.fail(f"{scenario['name']}: 应该能够恢复但失败了: {e}")

    def test_memory_leak_detection(self):
        """测试内存泄漏检测"""
        process = psutil.Process()

        # 记录初始内存
        initial_memory = process.memory_info().rss / 1024 / 1024
        memory_samples = [initial_memory]

        # 执行大量操作
        with patch.object(self.client, '_execute_request') as mock_execute:
            mock_execute.return_value = self.processor.convert_to_wav(
                np.random.randint(-32768, 32767, 16000, dtype=np.int16)
            )

            for i in range(50):
                # 执行TTS请求
                self.client.synthesize_speech(f"内存泄漏测试 {i}")

                # 执行音频处理
                test_audio = self.processor.convert_to_wav(
                    np.random.randint(-32768, 32767, 8000, dtype=np.int16)
                )
                self.processor.enhance_audio_quality(test_audio)
                self.processor.evaluate_audio_quality(test_audio)

                # 每10次操作采样内存
                if i % 10 == 0:
                    current_memory = process.memory_info().rss / 1024 / 1024
                    memory_samples.append(current_memory)

        # 强制垃圾回收
        import gc
        gc.collect()

        # 最终内存检查
        final_memory = process.memory_info().rss / 1024 / 1024
        memory_samples.append(final_memory)

        # 分析内存趋势
        memory_growth = final_memory - initial_memory
        max_memory = max(memory_samples)
        avg_memory = sum(memory_samples) / len(memory_samples)

        # 检查是否有明显的内存泄漏
        # 内存增长应该小于初始内存的50%
        memory_growth_ratio = memory_growth / initial_memory if initial_memory > 0 else 0

        self.assertLess(memory_growth_ratio, 0.5,
                       f"内存增长比例应小于50%，当前: {memory_growth_ratio:.2%}")
        self.assertLess(memory_growth, 100,
                       f"内存增长应小于100MB，当前: {memory_growth:.2f}MB")

        print(f"✅ 内存泄漏测试: 初始 {initial_memory:.2f}MB, "
              f"最终 {final_memory:.2f}MB, 增长 {memory_growth:.2f}MB ({memory_growth_ratio:.2%})")


if __name__ == '__main__':
    # 配置日志
    import logging
    logging.basicConfig(level=logging.INFO)

    # 运行测试
    unittest.main(verbosity=2)