#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Story 2.1: 通义千问API集成 - API客户端实现

通义千问API客户端实现，支持同步和异步调用方式。
集成qwen3-vl-plus模型，支持4000 tokens上下文长度，响应时间<3秒。

作者: Dev Agent
故事ID: Story 2.1
Epic: 2 - 智能对话模块
"""

import os
import json
import logging
import asyncio
from typing import Dict, Any, Optional, List, Union, AsyncGenerator
from dataclasses import dataclass, field
import aiohttp
from aiohttp import ClientTimeout, ClientSession, ClientError
from tenacity import retry, stop_after_attempt, wait_exponential, retry_if_exception_type

# 动态导入ROS2模块
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from geometry_msgs.msg import Pose2D
    ROS2_AVAILABLE = True
except ImportError as e:
    ROS2_AVAILABLE = False
    print(f"⚠️ ROS2环境不可用，降级为纯Python模式: {e}")
    # 创建虚拟类以避免导入错误
    class Node:
        pass
    class String:
        pass
    class Pose2D:
        pass


# 配置日志
logger = logging.getLogger(__name__)


@dataclass
class QwenConfig:
    """通义千问API配置 - qwen3-vl-plus视觉多模态模型"""
    api_key: str = field(default_factory=lambda: os.getenv('QWEN_API_KEY', ''))
    api_base_url: str = "https://dashscope.aliyuncs.com/api/v1/services/aigc/multimodal-generation/generation"
    model_name: str = "qwen3-vl-plus"
    max_tokens: int = 4000
    temperature: float = 0.7
    top_p: float = 0.8
    timeout: int = 30
    max_retries: int = 3
    request_interval: float = 0.1  # 请求间隔，支持限流


@dataclass
class QwenRequest:
    """通义千问请求对象"""
    messages: List[Dict[str, str]]
    max_tokens: Optional[int] = None
    temperature: Optional[float] = None
    top_p: Optional[float] = None
    stream: bool = False


@dataclass
class QwenResponse:
    """通义千问响应对象"""
    text: str
    model: str
    usage: Dict[str, int]
    finish_reason: str
    request_id: str
    confidence: float = 1.0


class QwenAPIClient:
    """
    通义千问API客户端

    功能特性:
    - 支持同步和异步调用
    - 自动重试机制
    - 限流保护
    - 错误处理
    - 请求/响应日志
    - ROS2节点集成
    """

    def __init__(self, config: Optional[QwenConfig] = None, node: Optional[Node] = None):
        """
        初始化通义千问API客户端

        Args:
            config: API配置对象
            node: ROS2节点实例
        """
        self.config = config or QwenConfig()
        self.node = node
        self.session: Optional[ClientSession] = None
        self._rate_limiter = asyncio.Semaphore(10)  # 限制并发数
        self._last_request_time = 0
        self._request_count = 0
        self._error_count = 0

        # 验证API Key
        if not self.config.api_key:
            logger.warning("⚠️ 通义千问API Key未设置，请设置QWEN_API_KEY环境变量")

        logger.info(f"✅ 通义千问API客户端初始化完成")
        logger.info(f"   - 模型: {self.config.model_name}")
        logger.info(f"   - 最大tokens: {self.config.max_tokens}")
        logger.info(f"   - 温度: {self.config.temperature}")
        logger.info(f"   - 并发限制: 10")

    async def __aenter__(self):
        """异步上下文管理器入口"""
        await self._init_session()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """异步上下文管理器出口"""
        await self.close()

    async def _init_session(self):
        """初始化HTTP会话"""
        timeout = ClientTimeout(total=self.config.timeout)
        connector = aiohttp.TCPConnector(
            limit=100,
            limit_per_host=10,
            ttl_dns_cache=300,
            use_dns_cache=True
        )
        self.session = ClientSession(
            timeout=timeout,
            connector=connector,
            headers={
                'Authorization': f'Bearer {self.config.api_key}',
                'Content-Type': 'application/json'
            }
        )
        logger.info("✅ HTTP会话初始化完成")

    async def close(self):
        """关闭HTTP会话"""
        if self.session:
            await self.session.close()
            self.session = None
            logger.info("✅ HTTP会话已关闭")

    def _build_request_payload(self, request: QwenRequest) -> Dict[str, Any]:
        """构建请求载荷"""
        payload = {
            "model": self.config.model_name,
            "input": {
                "messages": request.messages
            },
            "parameters": {
                "max_tokens": request.max_tokens or self.config.max_tokens,
                "temperature": request.temperature or self.config.temperature,
                "top_p": request.top_p or self.config.top_p,
                "incremental_output": request.stream
            }
        }
        return payload

    def _parse_response(self, response_data: Dict[str, Any]) -> QwenResponse:
        """解析API响应"""
        try:
            output = response_data.get('output', {})
            choices = output.get('choices', [])
            text = ''

            # 解析不同格式的响应
            if choices:
                message = choices[0].get('message', {})
                content = message.get('content', [])

                if content and isinstance(content, list) and len(content) > 0:
                    # 新格式: content是列表，取第一个元素的text字段
                    text = content[0].get('text', '')
                elif isinstance(content, str):
                    # 旧格式: content直接是字符串
                    text = content
                else:
                    # 其他格式，尝试从message直接获取text
                    text = message.get('text', '')

            finish_reason = choices[0].get('finish_reason', 'stop') if choices else 'stop'

            usage = response_data.get('usage', {})
            request_id = response_data.get('request_id', '')

            return QwenResponse(
                text=text,
                model=self.config.model_name,
                usage=usage,
                finish_reason=finish_reason,
                request_id=request_id
            )
        except Exception as e:
            logger.error(f"❌ 响应解析失败: {e}")
            logger.error(f"响应数据: {json.dumps(response_data, ensure_ascii=False, indent=2)}")
            raise ValueError(f"无法解析API响应: {e}")

    async def _rate_limit(self):
        """限流控制"""
        current_time = asyncio.get_event_loop().time()
        time_since_last = current_time - self._last_request_time

        if time_since_last < self.config.request_interval:
            await asyncio.sleep(self.config.request_interval - time_since_last)

        self._last_request_time = asyncio.get_event_loop().time()
        self._request_count += 1

    @retry(
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=1, min=4, max=10),
        retry=retry_if_exception_type((ClientError, asyncio.TimeoutError))
    )
    async def chat_async(
        self,
        messages: List[Dict[str, str]],
        max_tokens: Optional[int] = None,
        temperature: Optional[float] = None,
        stream: bool = False
    ) -> QwenResponse:
        """
        异步聊天对话

        Args:
            messages: 对话消息列表
            max_tokens: 最大输出tokens
            temperature: 采样温度
            stream: 是否流式输出

        Returns:
            QwenResponse: 响应对象
        """
        if not self.session:
            await self._init_session()

        async with self._rate_limiter:
            await self._rate_limit()

            request = QwenRequest(
                messages=messages,
                max_tokens=max_tokens,
                temperature=temperature,
                stream=stream
            )

            payload = self._build_request_payload(request)

            try:
                logger.info(f"📤 发送请求: {len(messages)}条消息")
                logger.debug(f"📤 请求载荷: {json.dumps(payload, ensure_ascii=False, indent=2)}")

                async with self.session.post(
                    self.config.api_base_url,
                    json=payload
                ) as response:

                    if response.status != 200:
                        error_text = await response.text()
                        logger.error(f"❌ API请求失败: {response.status}")
                        logger.error(f"❌ 错误详情: {error_text}")
                        raise ClientError(f"API请求失败: {response.status}")

                    response_data = await response.json()
                    logger.info(f"📥 收到响应: {len(response_data)}字节")
                    logger.debug(f"📥 响应内容: {json.dumps(response_data, ensure_ascii=False, indent=2)}")

                    parsed_response = self._parse_response(response_data)

                    # ROS2发布
                    if self.node:
                        msg = String()
                        msg.data = json.dumps({
                            'type': 'llm_response',
                            'text': parsed_response.text,
                            'model': parsed_response.model,
                            'usage': parsed_response.usage,
                            'request_id': parsed_response.request_id
                        })
                        self.node.get_logger().info(f"发布LLM响应到ROS2话题")

                    return parsed_response

            except Exception as e:
                self._error_count += 1
                logger.error(f"❌ 通义千问API调用失败: {e}")
                raise

    async def chat_stream_async(
        self,
        messages: List[Dict[str, str]],
        max_tokens: Optional[int] = None,
        temperature: Optional[float] = None
    ) -> AsyncGenerator[str, None]:
        """
        异步流式聊天对话

        Args:
            messages: 对话消息列表
            max_tokens: 最大输出tokens
            temperature: 采样温度

        Yields:
            str: 流式响应的文本片段
        """
        if not self.session:
            await self._init_session()

        async with self._rate_limiter:
            await self._rate_limit()

            request = QwenRequest(
                messages=messages,
                max_tokens=max_tokens,
                temperature=temperature,
                stream=True
            )

            payload = self._build_request_payload(request)

            try:
                logger.info(f"📤 发送流式请求: {len(messages)}条消息")

                async with self.session.post(
                    self.config.api_base_url,
                    json=payload
                ) as response:

                    if response.status != 200:
                        error_text = await response.text()
                        logger.error(f"❌ 流式API请求失败: {response.status}")
                        raise ClientError(f"流式API请求失败: {response.status}")

                    async for line in response.content:
                        if line:
                            line_str = line.decode('utf-8').strip()
                            if line_str.startswith('data: '):
                                data_str = line_str[6:]  # 移除 'data: ' 前缀
                                if data_str == '[DONE]':
                                    break
                                try:
                                    data = json.loads(data_str)
                                    output = data.get('output', {})
                                    text = output.get('text', '')
                                    if text:
                                        yield text
                                except json.JSONDecodeError:
                                    continue

            except Exception as e:
                self._error_count += 1
                logger.error(f"❌ 通义千问流式API调用失败: {e}")
                raise

    def _chat_sync(
        self,
        messages: List[Dict[str, str]],
        max_tokens: Optional[int] = None,
        temperature: Optional[float] = None
    ) -> QwenResponse:
        """
        同步聊天对话 (内部方法)

        Args:
            messages: 对话消息列表
            max_tokens: 最大输出tokens
            temperature: 采样温度

        Returns:
            QwenResponse: 响应对象
        """
        return asyncio.run(self.chat_async(messages, max_tokens, temperature))

    async def chat(
        self,
        user_input: str,
        max_tokens: Optional[int] = None,
        temperature: Optional[float] = None
    ) -> str:
        """
        异步聊天对话 (简化接口)

        Args:
            user_input: 用户输入文本
            max_tokens: 最大输出tokens
            temperature: 采样温度

        Returns:
            str: 响应文本内容
        """
        # 将字符串转换为消息格式
        messages = [
            {"role": "user", "content": user_input}
        ]

        try:
            response = await self.chat_async(messages, max_tokens, temperature)
            return response.text if response else ""
        except Exception as e:
            logger.error(f"❌ LLM响应失败: {e}")
            return ""

    async def health_check(self) -> Dict[str, Any]:
        """
        健康检查

        Returns:
            Dict[str, Any]: 健康检查结果
        """
        if not self.session:
            await self._init_session()

        test_messages = [
            {"role": "user", "content": "你好"}
        ]

        try:
            start_time = asyncio.get_event_loop().time()
            response = await self.chat_async(test_messages)
            end_time = asyncio.get_event_loop().time()
            response_time = (end_time - start_time) * 1000  # 转换为毫秒

            return {
                'status': 'healthy',
                'response_time_ms': round(response_time, 2),
                'api_call_success': True,
                'error_rate': round(self._error_count / max(self._request_count, 1) * 100, 2),
                'total_requests': self._request_count,
                'total_errors': self._error_count,
                'model': response.model
            }
        except Exception as e:
            return {
                'status': 'unhealthy',
                'error': str(e),
                'api_call_success': False,
                'error_rate': round(self._error_count / max(self._request_count, 1) * 100, 2),
                'total_requests': self._request_count,
                'total_errors': self._error_count
            }

    def get_stats(self) -> Dict[str, Any]:
        """
        获取API调用统计信息

        Returns:
            Dict[str, Any]: 统计信息
        """
        return {
            'total_requests': self._request_count,
            'total_errors': self._error_count,
            'error_rate': round(self._error_count / max(self._request_count, 1) * 100, 2),
            'success_rate': round((self._request_count - self._error_count) / max(self._request_count, 1) * 100, 2),
            'model': self.config.model_name,
            'config': {
                'max_tokens': self.config.max_tokens,
                'temperature': self.config.temperature,
                'timeout': self.config.timeout,
                'max_retries': self.config.max_retries
            }
        }


# ROS2节点集成示例
class QwenLLMNode(Node):
    """通义千问LLM ROS2节点"""

    def __init__(self):
        super().__init__('qwen_llm_node')

        # 初始化API客户端
        self.config = QwenConfig()
        self.client = QwenAPIClient(self.config, self)

        # ROS2订阅者和发布者
        self.subscription = self.create_subscription(
            String,
            '/llm/input',
            self.llm_input_callback,
            10
        )

        self.publisher = self.create_publisher(
            String,
            '/llm/output',
            10
        )

        # 定时健康检查
        self.timer = self.create_timer(30.0, self.health_check_callback)

        self.get_logger().info("✅ 通义千问LLM节点初始化完成")

    async def llm_input_callback(self, msg):
        """处理LLM输入消息"""
        try:
            # 解析输入消息
            input_data = json.loads(msg.data)

            if input_data.get('type') == 'chat':
                messages = input_data.get('messages', [])
                max_tokens = input_data.get('max_tokens')
                temperature = input_data.get('temperature')

                # 调用API
                response = await self.client.chat_async(messages, max_tokens, temperature)

                # 发布响应
                output_msg = String()
                output_msg.data = json.dumps({
                    'type': 'chat_response',
                    'text': response.text,
                    'model': response.model,
                    'usage': response.usage,
                    'request_id': response.request_id
                })
                self.publisher.publish(output_msg)

        except Exception as e:
            self.get_logger().error(f"❌ LLM处理失败: {e}")

    def health_check_callback(self):
        """定时健康检查回调"""
        # 这里需要异步执行健康检查
        # 在实际应用中，可以使用rclpy.spin_until_future_complete
        pass


if __name__ == '__main__':
    # 示例用法
    async def main():
        config = QwenConfig(
            model_name="qwen3-vl-plus",
            max_tokens=2000,
            temperature=0.7
        )

        async with QwenAPIClient(config) as client:
            # 测试API调用
            messages = [
                {"role": "user", "content": "你好，请介绍一下你自己"}
            ]

            response = await client.chat_async(messages)
            print(f"🤖 回答: {response.text}")
            print(f"📊 使用情况: {response.usage}")

            # 健康检查
            health = await client.health_check()
            print(f"❤️  健康状态: {health}")

    asyncio.run(main())
