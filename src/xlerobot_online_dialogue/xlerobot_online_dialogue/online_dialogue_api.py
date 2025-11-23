"""
OnlineDialogueAPI - 阿里云多模态对话API包装器
Story 1.7: 多模态在线对话API集成
严格遵循Epic 1纯在线架构原则 - 100%云端处理
"""

import asyncio
import base64
import json
import time
from typing import Dict, Optional, Any
from dataclasses import dataclass
import uuid
import aiohttp
import logging

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class DialogueRequest:
    """对话请求参数"""
    audio_base64: Optional[str] = None
    image_base64: Optional[str] = None
    session_id: str = ""
    user_input: str = ""
    context: Optional[Dict[str, Any]] = None

@dataclass
class DialogueResponse:
    """对话响应参数"""
    text_response: str
    session_id: str
    success: bool = True
    error_message: str = ""
    response_time_ms: int = 0
    api_metadata: Optional[Dict[str, Any]] = None

class OnlineDialogueAPI:
    """
    纯在线多模态对话API包装器
    严格遵循Epic 1纯在线架构 - 100%使用阿里云API
    """

    def __init__(self, api_key: str = None, model: str = "qwen-max"):
        """
        初始化在线对话API客户端

        Args:
            api_key: 阿里云API密钥
            model: 使用的模型名称，默认qwen-max
        """
        logger.info("🌐 初始化OnlineDialogueAPI - 纯在线架构")

        self.api_key = api_key
        self.model = model
        self.api_endpoint = "https://dashscope.aliyuncs.com/api/v1/services/aigc/multimodal-generation/generation"

        # API调用统计
        self.call_stats = {
            "total_calls": 0,
            "successful_calls": 0,
            "failed_calls": 0,
            "total_response_time_ms": 0,
            "average_response_time_ms": 0.0
        }

        # API配置 - 性能优化
        self.max_retries = 3
        self.retry_delay = 1.0  # 秒
        self.timeout = 30  # 秒

        # 新增性能配置
        self.circuit_breaker_threshold = 5  # 连续失败阈值
        self.circuit_breaker_timeout = 60  # 断路器恢复时间（秒）
        self.adaptive_retry_delay = True  # 自适应重试延迟
        self.connection_pool_size = 10  # 连接池大小

        # 断路器状态
        self.circuit_breaker_failures = 0
        self.circuit_breaker_last_failure_time = 0

        # 连接器配置（性能优化）- 延迟初始化
        self.connector = None

        logger.info(f"✅ API客户端初始化完成 - 模型: {model}")

    async def process_dialogue(self, request: DialogueRequest) -> DialogueResponse:
        """
        处理多模态对话请求 - 纯API调用，无本地处理

        Args:
            request: 对话请求参数

        Returns:
            DialogueResponse: API响应结果
        """
        start_time = time.time()
        self.call_stats["total_calls"] += 1

        try:
            logger.info(f"🤖 处理对话请求 - 会话ID: {request.session_id}")

            # 生成会话ID（如果未提供）
            if not request.session_id:
                request.session_id = str(uuid.uuid4())

            # 构建API请求
            api_request = await self._build_api_request(request)

            # 调用阿里云API（带重试）
            api_response = await self._call_api_with_retry(api_request)

            # 解析API响应
            response = await self._parse_api_response(api_response, request.session_id)

            # 更新统计信息
            response_time_ms = int((time.time() - start_time) * 1000)
            response.response_time_ms = response_time_ms
            self.call_stats["successful_calls"] += 1
            self._update_response_time_stats(response_time_ms)

            logger.info(f"✅ 对话处理成功 - 响应时间: {response_time_ms}ms")
            return response

        except Exception as e:
            logger.error(f"❌ 对话处理失败: {str(e)}")
            self.call_stats["failed_calls"] += 1

            return DialogueResponse(
                text_response=f"唔好意思，对话服务暂时唔能响应: {str(e)}",
                session_id=request.session_id,
                success=False,
                error_message=str(e),
                response_time_ms=int((time.time() - start_time) * 1000)
            )

    async def _build_api_request(self, request: DialogueRequest) -> Dict[str, Any]:
        """
        构建阿里云API请求
        严格按照阿里云DashScope API格式
        """
        # 构建消息列表
        messages = []

        # 添加用户输入文本
        if request.user_input:
            messages.append({
                "role": "user",
                "content": request.user_input
            })

        # 添加多模态内容（如果有图像）
        if request.image_base64:
            content_parts = []

            if request.user_input:
                content_parts.append({
                    "text": request.user_input
                })

            content_parts.append({
                "image": f"data:image/jpeg;base64,{request.image_base64}"
            })

            messages.append({
                "role": "user",
                "content": content_parts
            })

        # 构建完整API请求
        api_request = {
            "model": self.model,
            "input": {
                "messages": messages
            },
            "parameters": {
                "top_p": 0.8,
                "temperature": 0.7,
                "max_tokens": 2000
            }
        }

        # 添加会话ID到参数（如果支持）
        if request.session_id:
            api_request["parameters"]["session_id"] = request.session_id

        return api_request

    async def _call_api_with_retry(self, api_request: Dict[str, Any]) -> Dict[str, Any]:
        """
        带重试机制的API调用 - 性能优化版本
        包含断路器模式和自适应重试延迟
        """
        # 检查断路器状态
        if self._is_circuit_breaker_open():
            raise Exception("断路器开启：API服务暂时不可用")

        # 延迟初始化连接器
        if self.connector is None:
            self.connector = aiohttp.TCPConnector(
                limit=self.connection_pool_size,
                ttl_dns_cache=300,  # DNS缓存5分钟
                use_dns_cache=True,
                keepalive_timeout=60,  # 连接保持时间
                enable_cleanup_closed=True
            )

        last_exception = None

        for attempt in range(self.max_retries):
            try:
                logger.debug(f"📡 API调用尝试 {attempt + 1}/{self.max_retries}")

                # 自适应超时配置
                timeout = self.timeout * (1 + attempt * 0.5)  # 递增超时

                async with aiohttp.ClientSession(
                    headers={
                        "Authorization": f"Bearer {self.api_key}",
                        "Content-Type": "application/json",
                        "User-Agent": "XleRobot-OnlineDialogue/1.0"
                    },
                    timeout=aiohttp.ClientTimeout(total=timeout),
                    connector=self.connector
                ) as session:
                    async with session.post(
                        self.api_endpoint,
                        json=api_request
                    ) as response:

                        if response.status == 200:
                            result = await response.json()
                            logger.debug("✅ API调用成功")
                            # 重置断路器失败计数
                            self.circuit_breaker_failures = 0
                            return result
                        else:
                            error_text = await response.text()
                            raise Exception(f"API错误 {response.status}: {error_text}")

            except Exception as e:
                last_exception = e
                logger.warning(f"⚠️ API调用失败 (尝试 {attempt + 1}): {str(e)}")

                # 更新断路器状态
                self.circuit_breaker_failures += 1
                self.circuit_breaker_last_failure_time = time.time()

                if attempt < self.max_retries - 1:
                    # 自适应重试延迟
                    if self.adaptive_retry_delay:
                        delay = self._calculate_adaptive_delay(attempt)
                    else:
                        delay = self.retry_delay * (2 ** attempt)  # 指数退避

                    logger.debug(f"⏱️ 等待 {delay:.2f}秒后重试...")
                    await asyncio.sleep(delay)
                    continue

        # 所有重试都失败了
        raise Exception(f"API调用失败，已尝试{self.max_retries}次: {str(last_exception)}")

    async def _parse_api_response(self, api_response: Dict[str, Any], session_id: str) -> DialogueResponse:
        """
        解析阿里云API响应
        """
        try:
            # 检查API响应格式
            if "output" not in api_response:
                raise Exception("API响应格式错误：缺少output字段")

            output = api_response["output"]

            if "choices" not in output or not output["choices"]:
                raise Exception("API响应格式错误：缺少choices字段")

            choice = output["choices"][0]

            if "message" not in choice or "content" not in choice["message"]:
                raise Exception("API响应格式错误：缺少message.content字段")

            text_response = choice["message"]["content"]

            return DialogueResponse(
                text_response=text_response,
                session_id=session_id,
                success=True,
                api_metadata={
                    "model": api_response.get("model", self.model),
                    "request_id": api_response.get("request_id", ""),
                    "usage": api_response.get("usage", {})
                }
            )

        except Exception as e:
            logger.error(f"❌ API响应解析失败: {str(e)}")
            raise Exception(f"API响应解析失败: {str(e)}")

    def _update_response_time_stats(self, response_time_ms: int):
        """更新响应时间统计"""
        total_calls = self.call_stats["successful_calls"]
        if total_calls == 1:
            self.call_stats["average_response_time_ms"] = response_time_ms
        else:
            current_avg = self.call_stats["average_response_time_ms"]
            new_avg = ((current_avg * (total_calls - 1)) + response_time_ms) / total_calls
            self.call_stats["average_response_time_ms"] = new_avg

    def get_api_statistics(self) -> Dict[str, Any]:
        """
        获取API调用统计信息
        """
        stats = self.call_stats.copy()

        # 计算成功率
        if stats["total_calls"] > 0:
            stats["success_rate"] = stats["successful_calls"] / stats["total_calls"]
        else:
            stats["success_rate"] = 0.0

        return stats

    def reset_statistics(self):
        """重置统计信息"""
        self.call_stats = {
            "total_calls": 0,
            "successful_calls": 0,
            "failed_calls": 0,
            "total_response_time_ms": 0,
            "average_response_time_ms": 0.0
        }

    def _is_circuit_breaker_open(self) -> bool:
        """
        检查断路器是否开启
        """
        if self.circuit_breaker_failures >= self.circuit_breaker_threshold:
            # 检查是否超过恢复时间
            time_since_last_failure = time.time() - self.circuit_breaker_last_failure_time
            if time_since_last_failure < self.circuit_breaker_timeout:
                logger.warning(f"🚫 断路器开启：连续失败 {self.circuit_breaker_failures} 次")
                return True
            else:
                # 超过恢复时间，重置断路器
                self.circuit_breaker_failures = 0
                logger.info("🔄 断路器已重置")
        return False

    def _calculate_adaptive_delay(self, attempt: int) -> float:
        """
        计算自适应重试延迟
        考虑网络状况和历史响应时间
        """
        # 基础指数退避
        base_delay = self.retry_delay * (2 ** attempt)

        # 添加随机抖动，避免雷群效应
        import random
        jitter = random.uniform(0.8, 1.2)

        # 根据历史成功率调整延迟
        if self.call_stats["total_calls"] > 0:
            success_rate = self.call_stats["successful_calls"] / self.call_stats["total_calls"]
            if success_rate < 0.5:  # 成功率低于50%时增加延迟
                adaptive_factor = 1.5
            elif success_rate < 0.8:  # 成功率50-80%时略微增加延迟
                adaptive_factor = 1.2
            else:  # 成功率高时保持正常延迟
                adaptive_factor = 1.0
        else:
            adaptive_factor = 1.0

        calculated_delay = base_delay * jitter * adaptive_factor

        # 限制最大延迟
        max_delay = self.retry_delay * 8
        return min(calculated_delay, max_delay)

    def get_performance_metrics(self) -> Dict[str, Any]:
        """
        获取性能指标
        """
        stats = self.get_api_statistics()

        # 添加断路器状态
        stats.update({
            "circuit_breaker_failures": self.circuit_breaker_failures,
            "circuit_breaker_open": self._is_circuit_breaker_open(),
            "circuit_breaker_threshold": self.circuit_breaker_threshold,
            "connection_pool_size": self.connection_pool_size,
            "adaptive_retry_delay": self.adaptive_retry_delay
        })

        return stats

    async def health_check(self) -> bool:
        """
        健康检查 - 测试API连接
        """
        try:
            test_request = DialogueRequest(
                user_input="测试连接",
                session_id=str(uuid.uuid4())
            )

            response = await self.process_dialogue(test_request)
            return response.success

        except Exception as e:
            logger.error(f"❌ API健康检查失败: {str(e)}")
            return False

# 全局API实例
_global_api_instance = None

def get_online_dialogue_api(api_key: str = None) -> OnlineDialogueAPI:
    """
    获取全局在线对话API实例（单例模式）
    """
    global _global_api_instance

    if _global_api_instance is None:
        _global_api_instance = OnlineDialogueAPI(api_key=api_key)

    return _global_api_instance