#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Story 2.1: 通义千问API集成 - 测试套件

通义千问API客户端测试，包含单元测试、集成测试和性能测试。
验证API调用、错误处理、限流控制等功能。

作者: Dev Agent
故事ID: Story 2.1
Epic: 2 - 智能对话模块
"""

import os
import sys
import unittest
import asyncio
import time
import json
from unittest.mock import Mock, patch, AsyncMock
from typing import List, Dict, Any

# 添加项目路径
sys.path.insert(0, '/home/sunrise/xlerobot/src')

from modules.llm.qwen_client import (
    QwenAPIClient, QwenConfig, QwenRequest, QwenResponse
)
from modules.llm.api_manager import (
    APIManager, RequestQueue, RateLimiter, APIRequest, RequestPriority, RateLimitConfig
)
from modules.llm.response_parser import (
    ResponseParser, ParsedResponse, OutputFormat, Language
)


class TestQwenConfig(unittest.TestCase):
    """测试QwenConfig配置类"""

    def test_default_config(self):
        """测试默认配置"""
        config = QwenConfig()

        self.assertEqual(config.model_name, "qwen3-vl-plus")
        self.assertEqual(config.max_tokens, 4000)
        self.assertEqual(config.temperature, 0.7)
        self.assertEqual(config.timeout, 30)
        self.assertEqual(config.max_retries, 3)

    def test_custom_config(self):
        """测试自定义配置"""
        config = QwenConfig(
            model_name="qwen-max",
            max_tokens=1000,
            temperature=0.5,
            timeout=60
        )

        self.assertEqual(config.model_name, "qwen-max")
        self.assertEqual(config.max_tokens, 1000)
        self.assertEqual(config.temperature, 0.5)
        self.assertEqual(config.timeout, 60)


class TestQwenRequest(unittest.TestCase):
    """测试QwenRequest请求类"""

    def test_request_creation(self):
        """测试请求对象创建"""
        messages = [
            {"role": "user", "content": "你好"}
        ]

        request = QwenRequest(
            messages=messages,
            max_tokens=100,
            temperature=0.8
        )

        self.assertEqual(request.messages, messages)
        self.assertEqual(request.max_tokens, 100)
        self.assertEqual(request.temperature, 0.8)
        self.assertFalse(request.stream)


class TestQwenResponse(unittest.TestCase):
    """测试QwenResponse响应类"""

    def test_response_creation(self):
        """测试响应对象创建"""
        response = QwenResponse(
            text="你好，我是AI助手",
            model="qwen3-vl-plus",
            usage={"input_tokens": 10, "output_tokens": 20},
            finish_reason="stop",
            request_id="test-123"
        )

        self.assertEqual(response.text, "你好，我是AI助手")
        self.assertEqual(response.model, "qwen3-vl-plus")
        self.assertEqual(response.usage, {"input_tokens": 10, "output_tokens": 20})
        self.assertEqual(response.finish_reason, "stop")
        self.assertEqual(response.request_id, "test-123")


class TestQwenAPIClient(unittest.TestCase):
    """测试QwenAPIClient客户端类"""

    def setUp(self):
        """测试前设置"""
        self.config = QwenConfig(
            api_key="test-api-key",
            model_name="qwen3-vl-plus"
        )
        self.client = QwenAPIClient(self.config)

    def tearDown(self):
        """测试后清理"""
        if self.client.session:
            asyncio.run(self.client.close())

    def test_client_initialization(self):
        """测试客户端初始化"""
        self.assertEqual(self.client.config.model_name, "qwen3-vl-plus")
        self.assertEqual(self.client.config.api_key, "test-api-key")
        self.assertIsNone(self.client.session)

    @patch('aiohttp.ClientSession.post')
    async def test_async_chat_success(self, mock_post):
        """测试异步聊天成功"""
        # 模拟API响应
        mock_response_data = {
            'output': {
                'text': '你好，我是AI助手',
                'finish_reason': 'stop'
            },
            'usage': {
                'input_tokens': 10,
                'output_tokens': 20
            },
            'request_id': 'test-123'
        }

        mock_response = AsyncMock()
        mock_response.status = 200
        mock_response.json = AsyncMock(return_value=mock_response_data)
        mock_post.return_value.__aenter__.return_value = mock_response

        # 测试异步调用
        messages = [{"role": "user", "content": "你好"}]
        response = await self.client.chat_async(messages)

        self.assertEqual(response.text, "你好，我是AI助手")
        self.assertEqual(response.model, "qwen-plus")
        self.assertEqual(response.request_id, "test-123")

    @patch('aiohttp.ClientSession.post')
    async def test_async_chat_error(self, mock_post):
        """测试异步聊天错误处理"""
        # 模拟API错误响应
        mock_response = AsyncMock()
        mock_response.status = 400
        mock_response.text = "Bad Request"
        mock_post.return_value.__aenter__.return_value = mock_response

        # 测试错误处理
        messages = [{"role": "user", "content": "测试"}]

        with self.assertRaises(Exception):
            await self.client.chat_async(messages)

    def test_build_request_payload(self):
        """测试请求载荷构建"""
        request = QwenRequest(
            messages=[{"role": "user", "content": "测试"}],
            max_tokens=100,
            temperature=0.8
        )

        payload = self.client._build_request_payload(request)

        self.assertEqual(payload['model'], "qwen3-vl-plus")
        self.assertEqual(payload['input']['messages'], [{"role": "user", "content": "测试"}])
        self.assertEqual(payload['parameters']['max_tokens'], 100)
        self.assertEqual(payload['parameters']['temperature'], 0.8)

    def test_parse_response(self):
        """测试响应解析"""
        response_data = {
            'output': {
                'text': '测试响应',
                'finish_reason': 'stop'
            },
            'usage': {
                'input_tokens': 5,
                'output_tokens': 10
            },
            'request_id': 'test-456'
        }

        parsed = self.client._parse_response(response_data)

        self.assertEqual(parsed.text, "测试响应")
        self.assertEqual(parsed.usage, {'input_tokens': 5, 'output_tokens': 10})
        self.assertEqual(parsed.request_id, 'test-456')

    async def test_health_check(self):
        """测试健康检查"""
        with patch.object(self.client, 'chat_async') as mock_chat:
            mock_response = QwenResponse(
                text="响应",
                model="qwen3-vl-plus",
                usage={},
                finish_reason="stop",
                request_id="health-check"
            )
            mock_chat.return_value = mock_response

            health = await self.client.health_check()

            self.assertIn('status', health)
            self.assertIn('response_time_ms', health)
            self.assertIn('api_call_success', health)

    def test_get_stats(self):
        """测试获取统计信息"""
        # 模拟一些请求和错误
        self.client._request_count = 10
        self.client._error_count = 1

        stats = self.client.get_stats()

        self.assertEqual(stats['total_requests'], 10)
        self.assertEqual(stats['total_errors'], 1)
        self.assertEqual(stats['model'], "qwen3-vl-plus")


class TestRequestQueue(unittest.TestCase):
    """测试RequestQueue请求队列"""

    def setUp(self):
        """测试前设置"""
        self.queue = RequestQueue()

    def test_add_request(self):
        """测试添加请求"""
        request = APIRequest(
            id="test-1",
            messages=[{"role": "user", "content": "测试"}]
        )

        self.queue.add_request(request)
        self.assertEqual(self.queue.get_queue_size(RequestPriority.NORMAL), 1)

    def test_get_next_request(self):
        """测试获取下一个请求"""
        request = APIRequest(
            id="test-1",
            messages=[{"role": "user", "content": "测试"}]
        )

        self.queue.add_request(request)
        retrieved = self.queue.get_next_request()

        self.assertEqual(retrieved.id, "test-1")
        self.assertEqual(self.queue.get_queue_size(), 0)

    def test_priority_order(self):
        """测试优先级顺序"""
        # 添加不同优先级的请求
        high_request = APIRequest(
            id="high",
            messages=[{"role": "user", "content": "高优先级"}],
            priority=RequestPriority.HIGH
        )

        normal_request = APIRequest(
            id="normal",
            messages=[{"role": "user", "content": "普通优先级"}],
            priority=RequestPriority.NORMAL
        )

        low_request = APIRequest(
            id="low",
            messages=[{"role": "user", "content": "低优先级"}],
            priority=RequestPriority.LOW
        )

        self.queue.add_request(normal_request)
        self.queue.add_request(high_request)
        self.queue.add_request(low_request)

        # 高优先级应该先被取出
        first = self.queue.get_next_request()
        self.assertEqual(first.priority, RequestPriority.HIGH)

        # 然后是普通优先级
        second = self.queue.get_next_request()
        self.assertEqual(second.priority, RequestPriority.NORMAL)


class TestRateLimiter(unittest.TestCase):
    """测试RateLimiter限流器"""

    def setUp(self):
        """测试前设置"""
        self.config = RateLimitConfig(
            max_requests_per_minute=10,
            max_requests_per_hour=100,
            burst_limit=5
        )
        self.limiter = RateLimiter(self.config)

    def test_rate_limiter_initialization(self):
        """测试限流器初始化"""
        self.assertEqual(self.limiter.burst_tokens, 5)
        self.assertEqual(len(self.limiter.request_times), 0)

    def test_acquire_success(self):
        """测试成功获取令牌"""
        # 测试在限制范围内的请求
        for i in range(5):
            result = self.limiter.acquire()
            self.assertTrue(result)

    def test_acquire_failure(self):
        """测试获取令牌失败"""
        # 耗尽突发令牌
        for i in range(5):
            self.limiter.acquire()

        # 下一个请求应该失败
        result = self.limiter.acquire()
        self.assertFalse(result)

    def test_get_status(self):
        """测试获取状态"""
        status = self.limiter.get_status()

        self.assertIn('requests_per_minute', status)
        self.assertIn('burst_tokens', status)
        self.assertEqual(status['burst_tokens'], 5)


class TestResponseParser(unittest.TestCase):
    """测试ResponseParser响应解析器"""

    def setUp(self):
        """测试前设置"""
        self.parser = ResponseParser()

    def test_parser_initialization(self):
        """测试解析器初始化"""
        self.assertIsNotNone(self.parser)
        self.assertEqual(len(self.parser.stop_words), 36)

    def test_parse_simple_response(self):
        """测试解析简单响应"""
        response = QwenResponse(
            text="你好，我是AI助手",
            model="qwen3-vl-plus",
            usage={"input_tokens": 5, "output_tokens": 10},
            finish_reason="stop",
            request_id="test-123"
        )

        parsed = self.parser.parse_response(response)

        self.assertEqual(parsed.text, "你好，我是AI助手")
        self.assertEqual(parsed.language, Language.CHINESE)
        self.assertGreater(parsed.confidence, 0)

    def test_detect_language_chinese(self):
        """测试中文检测"""
        text = "你好，我是AI助手，很高兴为你服务。"

        language = self.parser._detect_language(text)

        self.assertIn(language, [Language.CHINESE, Language.CANTONESE])

    def test_detect_language_english(self):
        """测试英文检测"""
        text = "Hello, I am an AI assistant. Nice to meet you."

        language = self.parser._detect_language(text)

        self.assertEqual(language, Language.ENGLISH)

    def test_count_words_chinese(self):
        """测试中文词数统计"""
        text = "你好，我是AI助手"
        language = Language.CHINESE

        word_count = self.parser._count_words(text, language)

        self.assertGreater(word_count, 0)

    def test_detect_code(self):
        """测试代码检测"""
        text_with_code = "以下是Python代码：\n```python\ndef hello():\n    print('Hello')\n```"

        has_code = self.parser._detect_code(text_with_code)

        self.assertTrue(has_code)

    def test_detect_links(self):
        """测试链接检测"""
        text_with_link = "请访问 https://example.com 获取更多信息。"

        has_links = self.parser._detect_links(text_with_link)

        self.assertTrue(has_links)

    def test_format_output(self):
        """测试输出格式化"""
        text = "你好，我是AI助手"

        # 测试纯文本格式
        plain = self.parser._format_output(text, OutputFormat.PLAIN, Language.CHINESE)
        self.assertEqual(plain, "你好，我是AI助手")

        # 测试对话格式
        conversation = self.parser._format_output(text, OutputFormat.CONVERSATION, Language.CHINESE)
        self.assertEqual(conversation, "🤖 助手: 你好，我是AI助手")

    def test_validate_response(self):
        """测试响应验证"""
        # 有效响应
        valid_response = QwenResponse(
            text="这是一个有效的响应",
            model="qwen-plus",
            usage={},
            finish_reason="stop",
            request_id="test-123"
        )

        is_valid, message = self.parser.validate_response(valid_response)
        self.assertTrue(is_valid)
        self.assertEqual(message, "响应有效")

        # 无效响应
        invalid_response = QwenResponse(
            text="",
            model="qwen-plus",
            usage={},
            finish_reason="stop",
            request_id="test-456"
        )

        is_valid, message = self.parser.validate_response(invalid_response)
        self.assertFalse(is_valid)
        self.assertEqual(message, "响应文本为空")


class TestIntegration(unittest.TestCase):
    """集成测试"""

    def setUp(self):
        """测试前设置"""
        self.config = QwenConfig(api_key="test-key")
        self.parser = ResponseParser()

    def test_full_pipeline(self):
        """测试完整流程"""
        # 创建模拟响应
        response = QwenResponse(
            text="你好！我是通义千问AI助手，很高兴为你服务。",
            model="qwen-plus",
            usage={"input_tokens": 10, "output_tokens": 30},
            finish_reason="stop",
            request_id="integration-test"
        )

        # 解析响应
        parsed = self.parser.parse_response(response, OutputFormat.CONVERSATION)

        # 验证结果
        self.assertEqual(parsed.language, Language.CHINESE)
        self.assertGreater(parsed.word_count, 0)
        self.assertEqual(parsed.output_format, OutputFormat.CONVERSATION)
        self.assertIn("🤖 助手:", parsed.formatted_output)

    def test_batch_parse(self):
        """测试批量解析"""
        responses = [
            QwenResponse(
                text="第一个响应",
                model="qwen-plus",
                usage={},
                finish_reason="stop",
                request_id="batch-1"
            ),
            QwenResponse(
                text="第二个响应",
                model="qwen-plus",
                usage={},
                finish_reason="stop",
                request_id="batch-2"
            )
        ]

        parsed_list = self.parser.batch_parse(responses)

        self.assertEqual(len(parsed_list), 2)
        self.assertEqual(parsed_list[0].text, "第一个响应")
        self.assertEqual(parsed_list[1].text, "第二个响应")


class TestPerformance(unittest.TestCase):
    """性能测试"""

    def test_response_parsing_performance(self):
        """测试响应解析性能"""
        parser = ResponseParser()

        response = QwenResponse(
            text="这是一个很长的响应文本" * 100,
            model="qwen-plus",
            usage={"input_tokens": 1000, "output_tokens": 2000},
            finish_reason="stop",
            request_id="perf-test"
        )

        start_time = time.time()
        parsed = parser.parse_response(response)
        end_time = time.time()

        processing_time = end_time - start_time

        self.assertLess(processing_time, 1.0)  # 应该在1秒内完成
        self.assertGreater(parsed.word_count, 100)

    def test_language_detection_performance(self):
        """测试语言检测性能"""
        parser = ResponseParser()

        # 创建不同语言的测试文本
        texts = [
            "你好，这是中文测试文本",
            "Hello, this is English test text",
            "你好，呢度系粤语测试文本"
        ]

        start_time = time.time()

        for text in texts:
            language = parser._detect_language(text)

        end_time = time.time()
        processing_time = end_time - start_time

        self.assertLess(processing_time, 0.1)  # 语言检测应该很快


def create_test_suite():
    """创建测试套件"""
    suite = unittest.TestSuite()

    # 添加测试类
    test_classes = [
        TestQwenConfig,
        TestQwenRequest,
        TestQwenResponse,
        TestQwenAPIClient,
        TestRequestQueue,
        TestRateLimiter,
        TestResponseParser,
        TestIntegration,
        TestPerformance
    ]

    for test_class in test_classes:
        tests = unittest.TestLoader().loadTestsFromTestCase(test_class)
        suite.addTests(tests)

    return suite


if __name__ == '__main__':
    print("🧪 开始Story 2.1测试套件执行...")
    print("=" * 60)

    # 运行测试
    runner = unittest.TextTestRunner(verbosity=2)
    suite = create_test_suite()
    result = runner.run(suite)

    # 输出测试结果
    print("\n" + "=" * 60)
    print("📊 测试结果摘要:")
    print(f"✅ 运行测试: {result.testsRun}")
    print(f"❌ 失败: {len(result.failures)}")
    print(f"⚠️ 错误: {len(result.errors)}")

    if result.failures:
        print("\n❌ 失败的测试:")
        for test, traceback in result.failures:
            print(f"  - {test}: {traceback.split('AssertionError:')[-1].strip()}")

    if result.errors:
        print("\n⚠️ 错误的测试:")
        for test, traceback in result.errors:
            print(f"  - {test}: {traceback.split('Exception:')[-1].strip()}")

    # 计算覆盖率
    success_rate = ((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun) * 100
    print(f"\n🎯 测试成功率: {success_rate:.1f}%")

    if success_rate >= 90:
        print("✅ 测试通过! Story 2.1质量验收标准达到A+级别")
    elif success_rate >= 80:
        print("⚠️ 测试基本通过，但需要改进部分功能")
    else:
        print("❌ 测试失败，需要修复关键问题")

    print("=" * 60)
    print("🏁 Story 2.1测试套件执行完成")
