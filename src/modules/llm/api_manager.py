#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Story 2.1: 通义千问API集成 - API调用管理器

API调用管理和限流模块，实现请求队列管理、并发控制和配额管理。
支持连接池复用、异步调用支持、性能监控和限流保护。

作者: Dev Agent
故事ID: Story 2.1
Epic: 2 - 智能对话模块
"""

import os
import time
import asyncio
import logging
from typing import Dict, Any, List, Optional, Callable, Union
from dataclasses import dataclass, field
from enum import Enum
import json
import weakref
from collections import deque, defaultdict
import threading

# 动态导入ROS2模块
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
except ImportError as e:
    ROS2_AVAILABLE = False
    print(f"⚠️ ROS2环境不可用，降级为纯Python模式: {e}")
    # 创建虚拟类以避免导入错误
    class Node:
        pass
    class String:
        pass

from .qwen_client import QwenAPIClient, QwenConfig, QwenRequest, QwenResponse


logger = logging.getLogger(__name__)


class RequestPriority(Enum):
    """请求优先级"""
    LOW = 1
    NORMAL = 2
    HIGH = 3
    CRITICAL = 4


@dataclass
class RateLimitConfig:
    """限流配置"""
    max_requests_per_minute: int = 60
    max_requests_per_hour: int = 3600
    max_concurrent_requests: int = 10
    burst_limit: int = 20  # 突发请求限制
    cooldown_period: float = 1.0  # 限流冷却时间(秒)


@dataclass
class APIRequest:
    """API请求对象"""
    id: str
    messages: List[Dict[str, str]]
    callback: Optional[Callable] = None
    priority: RequestPriority = RequestPriority.NORMAL
    max_tokens: Optional[int] = None
    temperature: Optional[float] = None
    created_at: float = field(default_factory=time.time)
    retry_count: int = 0
    max_retries: int = 3
    timeout: float = 30.0


@dataclass
class APIResponse:
    """API响应对象"""
    request_id: str
    response: QwenResponse
    success: bool
    error: Optional[str] = None
    response_time: float = 0.0
    timestamp: float = field(default_factory=time.time)


class RequestQueue:
    """请求队列管理"""

    def __init__(self):
        self.queues: Dict[RequestPriority, deque] = {
            priority: deque() for priority in RequestPriority
        }
        self.active_requests: Dict[str, APIRequest] = {}
        self.lock = threading.RLock()

    def add_request(self, request: APIRequest):
        """添加请求到队列"""
        with self.lock:
            self.queues[request.priority].append(request)
            logger.debug(f"📝 添加请求到队列: {request.id}, 优先级: {request.priority.name}")

    def get_next_request(self) -> Optional[APIRequest]:
        """获取下一个要处理的请求"""
        with self.lock:
            # 按优先级顺序处理
            for priority in [RequestPriority.CRITICAL, RequestPriority.HIGH,
                           RequestPriority.NORMAL, RequestPriority.LOW]:
                if self.queues[priority]:
                    request = self.queues[priority].popleft()
                    self.active_requests[request.id] = request
                    logger.debug(f"📤 从队列获取请求: {request.id}")
                    return request
            return None

    def mark_completed(self, request_id: str):
        """标记请求已完成"""
        with self.lock:
            if request_id in self.active_requests:
                del self.active_requests[request_id]
                logger.debug(f"✅ 请求已完成: {request_id}")

    def get_queue_size(self, priority: Optional[RequestPriority] = None) -> int:
        """获取队列大小"""
        with self.lock:
            if priority:
                return len(self.queues[priority])
            return sum(len(queue) for queue in self.queues.values())

    def get_active_count(self) -> int:
        """获取活跃请求数量"""
        with self.lock:
            return len(self.active_requests)


class RateLimiter:
    """限流器"""

    def __init__(self, config: RateLimitConfig):
        self.config = config
        self.request_times: deque = deque()
        self.hourly_requests: deque = deque()
        self.burst_tokens: int = config.burst_limit
        self.last_refill = time.time()
        self.lock = threading.RLock()

    def acquire(self) -> bool:
        """
        获取请求令牌

        Returns:
            bool: 是否获取成功
        """
        with self.lock:
            current_time = time.time()

            # 清理过期的请求记录
            self._cleanup_old_requests(current_time)

            # 检查每分钟限制
            if len(self.request_times) >= self.config.max_requests_per_minute:
                logger.warning(f"⚠️ 达到每分钟请求限制: {self.config.max_requests_per_minute}")
                return False

            # 检查每小时限制
            if len(self.hourly_requests) >= self.config.max_requests_per_hour:
                logger.warning(f"⚠️ 达到每小时请求限制: {self.config.max_requests_per_hour}")
                return False

            # 检查突发令牌
            if self.burst_tokens <= 0:
                logger.warning(f"⚠️ 突发令牌耗尽")
                return False

            # 记录请求
            self.request_times.append(current_time)
            self.hourly_requests.append(current_time)
            self.burst_tokens -= 1

            logger.debug(f"✅ 获取请求令牌成功，剩余: {self.burst_tokens}")
            return True

    def _cleanup_old_requests(self, current_time: float):
        """清理过期的请求记录"""
        # 清理每分钟记录
        minute_ago = current_time - 60
        while self.request_times and self.request_times[0] < minute_ago:
            self.request_times.popleft()

        # 清理每小时记录
        hour_ago = current_time - 3600
        while self.hourly_requests and self.hourly_requests[0] < hour_ago:
            self.hourly_requests.popleft()

        # 重置突发令牌
        if current_time - self.last_refill >= self.config.cooldown_period:
            self.burst_tokens = min(self.config.burst_limit,
                                  self.burst_tokens + 1)
            self.last_refill = current_time

    def get_status(self) -> Dict[str, Any]:
        """获取限流状态"""
        with self.lock:
            return {
                'requests_per_minute': len(self.request_times),
                'requests_per_hour': len(self.hourly_requests),
                'burst_tokens': self.burst_tokens,
                'max_requests_per_minute': self.config.max_requests_per_minute,
                'max_requests_per_hour': self.config.max_requests_per_hour,
                'max_burst': self.config.burst_limit
            }


class APIManager:
    """
    API调用管理器

    功能特性:
    - 请求队列管理
    - 并发控制
    - 限流保护
    - 自动重试
    - 性能监控
    - 错误处理
    - 配额管理
    """

    def __init__(
        self,
        config: QwenConfig,
        rate_config: Optional[RateLimitConfig] = None,
        node: Optional[Node] = None
    ):
        """
        初始化API管理器

        Args:
            config: API配置
            rate_config: 限流配置
            node: ROS2节点实例
        """
        self.config = config
        self.rate_config = rate_config or RateLimitConfig()
        self.node = node
        self.client: Optional[QwenAPIClient] = None

        # 核心组件
        self.request_queue = RequestQueue()
        self.rate_limiter = RateLimiter(self.rate_config)

        # 统计信息
        self.stats = {
            'total_requests': 0,
            'successful_requests': 0,
            'failed_requests': 0,
            'total_response_time': 0.0,
            'average_response_time': 0.0,
            'last_request_time': 0.0,
            'peak_concurrent_requests': 0
        }

        # 回调函数
        self.response_callbacks: List[Callable] = []

        # 控制标志
        self.is_running = False
        self.worker_task: Optional[asyncio.Task] = None
        self.lock = threading.RLock()

        logger.info("✅ API管理器初始化完成")
        logger.info(f"   - 最大并发: {self.rate_config.max_concurrent_requests}")
        logger.info(f"   - 限流: {self.rate_config.max_requests_per_minute}/分钟")

    async def start(self):
        """启动API管理器"""
        if self.is_running:
            logger.warning("⚠️ API管理器已在运行")
            return

        self.is_running = True
        self.client = QwenAPIClient(self.config, self.node)
        await self.client._init_session()

        # 启动工作协程
        self.worker_task = asyncio.create_task(self._worker_loop())

        logger.info("🚀 API管理器已启动")

    async def stop(self):
        """停止API管理器"""
        if not self.is_running:
            return

        self.is_running = False

        if self.worker_task:
            self.worker_task.cancel()
            try:
                await self.worker_task
            except asyncio.CancelledError:
                pass

        if self.client:
            await self.client.close()

        logger.info("🛑 API管理器已停止")

    async def _worker_loop(self):
        """工作协程循环"""
        semaphore = asyncio.Semaphore(self.rate_config.max_concurrent_requests)

        while self.is_running:
            try:
                # 获取下一个请求
                request = self.request_queue.get_next_request()
                if not request:
                    await asyncio.sleep(0.1)
                    continue

                # 检查限流
                if not self.rate_limiter.acquire():
                    # 限流失败，重新加入队列
                    self.request_queue.add_request(request)
                    await asyncio.sleep(1)
                    continue

                # 异步处理请求
                asyncio.create_task(self._process_request(request, semaphore))

            except Exception as e:
                logger.error(f"❌ 工作循环错误: {e}")
                await asyncio.sleep(1)

    async def _process_request(self, request: APIRequest, semaphore: asyncio.Semaphore):
        """处理单个请求"""
        start_time = time.time()

        try:
            async with semaphore:
                # 更新并发统计
                with self.lock:
                    self.stats['total_requests'] += 1
                    active_count = self.request_queue.get_active_count()
                    if active_count > self.stats['peak_concurrent_requests']:
                        self.stats['peak_concurrent_requests'] = active_count

                # 调用API
                response = await self.client.chat_async(
                    messages=request.messages,
                    max_tokens=request.max_tokens,
                    temperature=request.temperature
                )

                # 处理响应
                response_time = time.time() - start_time
                api_response = APIResponse(
                    request_id=request.id,
                    response=response,
                    success=True,
                    response_time=response_time
                )

                # 更新统计
                self._update_stats(response_time, True)

                # 调用回调
                await self._handle_response(api_response)

        except Exception as e:
            # 处理错误和重试
            error_time = time.time() - start_time
            error_msg = str(e)

            if request.retry_count < request.max_retries:
                logger.warning(f"⚠️ 请求失败，准备重试: {request.id}, 错误: {error_msg}")
                request.retry_count += 1
                await asyncio.sleep(2 ** request.retry_count)  # 指数退避
                self.request_queue.add_request(request)
            else:
                logger.error(f"❌ 请求最终失败: {request.id}, 错误: {error_msg}")
                api_response = APIResponse(
                    request_id=request.id,
                    response=None,
                    success=False,
                    error=error_msg,
                    response_time=error_time
                )

                self._update_stats(error_time, False)
                await self._handle_response(api_response)

        finally:
            self.request_queue.mark_completed(request.id)

    def _update_stats(self, response_time: float, success: bool):
        """更新统计信息"""
        with self.lock:
            if success:
                self.stats['successful_requests'] += 1
            else:
                self.stats['failed_requests'] += 1

            self.stats['total_response_time'] += response_time
            self.stats['average_response_time'] = (
                self.stats['total_response_time'] / self.stats['total_requests']
            )
            self.stats['last_request_time'] = time.time()

    async def _handle_response(self, api_response: APIResponse):
        """处理API响应"""
        # 调用注册回调
        for callback in self.response_callbacks:
            try:
                if asyncio.iscoroutinefunction(callback):
                    await callback(api_response)
                else:
                    callback(api_response)
            except Exception as e:
                logger.error(f"❌ 回调函数执行失败: {e}")

        # 发布ROS2消息
        if self.node:
            msg = String()
            msg.data = json.dumps({
                'type': 'api_response',
                'request_id': api_response.request_id,
                'success': api_response.success,
                'error': api_response.error,
                'response_time': api_response.response_time,
                'timestamp': api_response.timestamp
            })
            self.node.get_logger().info(f"发布API响应到ROS2话题")

    async def submit_request(
        self,
        messages: List[Dict[str, str]],
        callback: Optional[Callable] = None,
        priority: RequestPriority = RequestPriority.NORMAL,
        max_tokens: Optional[int] = None,
        temperature: Optional[float] = None,
        timeout: float = 30.0
    ) -> str:
        """
        提交API请求

        Args:
            messages: 对话消息列表
            callback: 响应回调函数
            priority: 请求优先级
            max_tokens: 最大输出tokens
            temperature: 采样温度
            timeout: 超时时间

        Returns:
            str: 请求ID
        """
        if not self.is_running:
            raise RuntimeError("API管理器未启动")

        import uuid
        request_id = str(uuid.uuid4())[:8]

        request = APIRequest(
            id=request_id,
            messages=messages,
            callback=callback,
            priority=priority,
            max_tokens=max_tokens,
            temperature=temperature,
            timeout=timeout
        )

        self.request_queue.add_request(request)

        logger.info(f"📝 提交请求: {request_id}, 优先级: {priority.name}")
        return request_id

    def add_response_callback(self, callback: Callable):
        """添加响应回调函数"""
        self.response_callbacks.append(callback)
        logger.info(f"📋 添加响应回调: {callback.__name__}")

    def get_status(self) -> Dict[str, Any]:
        """获取管理器状态"""
        with self.lock:
            queue_status = {
                'total_queue_size': self.request_queue.get_queue_size(),
                'active_requests': self.request_queue.get_active_count(),
                'peak_concurrent': self.stats['peak_concurrent_requests']
            }

            rate_status = self.rate_limiter.get_status()

            success_rate = (
                self.stats['successful_requests'] / max(self.stats['total_requests'], 1)
            ) * 100

            return {
                'is_running': self.is_running,
                'queue': queue_status,
                'rate_limiter': rate_status,
                'stats': {
                    **self.stats,
                    'success_rate': round(success_rate, 2),
                    'failure_rate': round(100 - success_rate, 2)
                }
            }

    def get_performance_metrics(self) -> Dict[str, Any]:
        """获取性能指标"""
        with self.lock:
            return {
                'total_requests': self.stats['total_requests'],
                'successful_requests': self.stats['successful_requests'],
                'failed_requests': self.stats['failed_requests'],
                'success_rate': round(
                    self.stats['successful_requests'] / max(self.stats['total_requests'], 1) * 100,
                    2
                ),
                'average_response_time_ms': round(
                    self.stats['average_response_time'] * 1000, 2
                ),
                'peak_concurrent_requests': self.stats['peak_concurrent_requests'],
                'last_request_time': self.stats['last_request_time']
            }


# ROS2节点集成
class QwenAPIManagerNode(Node):
    """通义千问API管理器ROS2节点"""

    def __init__(self):
        super().__init__('qwen_api_manager_node')

        # 初始化配置
        self.config = QwenConfig()
        self.rate_config = RateLimitConfig(
            max_requests_per_minute=60,
            max_concurrent_requests=10
        )

        # 初始化API管理器
        self.manager = APIManager(self.config, self.rate_config, self)

        # ROS2订阅者和发布者
        self.input_subscription = self.create_subscription(
            String,
            '/llm/api/input',
            self.api_input_callback,
            10
        )

        self.output_publisher = self.create_publisher(
            String,
            '/llm/api/output',
            10
        )

        self.status_publisher = self.create_publisher(
            String,
            '/llm/api/status',
            10
        )

        # 定时器
        self.status_timer = self.create_timer(10.0, self.status_callback)

        # 启动管理器
        asyncio.create_task(self._start_manager())

        self.get_logger().info("✅ 通义千问API管理器节点初始化完成")

    async def _start_manager(self):
        """启动API管理器"""
        try:
            await self.manager.start()
        except Exception as e:
            self.get_logger().error(f"❌ API管理器启动失败: {e}")

    async def api_input_callback(self, msg):
        """处理API输入消息"""
        try:
            input_data = json.loads(msg.data)

            if input_data.get('type') == 'submit_request':
                messages = input_data.get('messages', [])
                priority = RequestPriority(input_data.get('priority', 2))
                max_tokens = input_data.get('max_tokens')
                temperature = input_data.get('temperature')

                request_id = await self.manager.submit_request(
                    messages=messages,
                    priority=priority,
                    max_tokens=max_tokens,
                    temperature=temperature
                )

                # 响应请求ID
                response_msg = String()
                response_msg.data = json.dumps({
                    'type': 'request_submitted',
                    'request_id': request_id,
                    'status': 'queued'
                })
                self.output_publisher.publish(response_msg)

        except Exception as e:
            self.get_logger().error(f"❌ API输入处理失败: {e}")

    def status_callback(self):
        """发布状态消息"""
        try:
            status = self.manager.get_status()
            status_msg = String()
            status_msg.data = json.dumps(status)
            self.status_publisher.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f"❌ 状态发布失败: {e}")


if __name__ == '__main__':
    # 示例用法
    async def main():
        config = QwenConfig(model_name="qwen3-vl-plus")
        rate_config = RateLimitConfig(max_concurrent_requests=5)

        manager = APIManager(config, rate_config)

        # 启动管理器
        await manager.start()

        # 提交测试请求
        async def test_callback(response: APIResponse):
            if response.success:
                print(f"✅ 请求成功: {response.request_id}")
                print(f"📝 回答: {response.response.text}")
            else:
                print(f"❌ 请求失败: {response.request_id}, 错误: {response.error}")

        request_id = await manager.submit_request(
            messages=[{"role": "user", "content": "你好，请介绍一下你自己"}],
            callback=test_callback,
            priority=RequestPriority.NORMAL
        )

        print(f"📝 提交请求: {request_id}")

        # 等待处理
        await asyncio.sleep(5)

        # 获取状态
        status = manager.get_status()
        print(f"📊 状态: {json.dumps(status, indent=2, ensure_ascii=False)}")

        # 停止管理器
        await manager.stop()

    asyncio.run(main())
