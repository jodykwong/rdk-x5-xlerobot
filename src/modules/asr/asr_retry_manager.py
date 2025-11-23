#!/usr/bin/env python3
"""
ASR API重试和容错机制模块

专门为ASR服务优化的重试机制：
- 智能重试策略
- 断路器模式
- 熔断恢复机制
- 降级处理

作者: Dev Agent
日期: 2025-11-08
Epic: 1 - ASR语音识别模块
Story: 1.1 - 粤语语音识别基础功能
Phase: 3 - 基础语音识别
Task: 3.4 - 添加API重试和容错机制
"""

import logging
import time
import threading
from typing import Dict, List, Optional, Callable, Any, Union
from dataclasses import dataclass, field
from enum import Enum
from queue import Queue, Empty
import json
from modules.asr.network_config import ErrorType, ErrorInfo

logger = logging.getLogger(__name__)


class CircuitState(Enum):
    """断路器状态"""
    CLOSED = "closed"      # 关闭状态，正常工作
    OPEN = "open"          # 打开状态，熔断中
    HALF_OPEN = "half_open"  # 半开状态，尝试恢复


class RetryStrategy(Enum):
    """重试策略"""
    FIXED_DELAY = "fixed_delay"      # 固定延迟
    EXPONENTIAL_BACKOFF = "exponential_backoff"  # 指数退避
    LINEAR_BACKOFF = "linear_backoff"  # 线性退避
    ADAPTIVE = "adaptive"            # 自适应


class FallbackAction(Enum):
    """降级行为"""
    RETURN_EMPTY = "return_empty"      # 返回空结果
    RETURN_CACHED = "return_cached"    # 返回缓存结果
    RETURN_DEFAULT = "return_default"  # 返回默认结果
    THROW_ERROR = "throw_error"        # 抛出错误


@dataclass
class RetryConfig:
    """重试配置"""
    max_retries: int = 3                    # 最大重试次数
    strategy: RetryStrategy = RetryStrategy.EXPONENTIAL_BACKOFF  # 重试策略
    base_delay: float = 1.0                 # 基础延迟(秒)
    max_delay: float = 60.0                 # 最大延迟(秒)
    backoff_factor: float = 2.0             # 退避因子
    jitter: bool = True                     # 添加随机抖动
    retry_on_status: List[int] = field(default_factory=lambda: [408, 429, 500, 502, 503, 504])
    retry_on_errors: List[str] = field(default_factory=lambda: ["timeout", "connection", "server"])


@dataclass
class CircuitBreakerConfig:
    """断路器配置"""
    failure_threshold: int = 5              # 失败阈值
    recovery_timeout: float = 60.0          # 恢复超时(秒)
    expected_exception: List[str] = field(default_factory=lambda: ["timeout", "connection"])
    success_threshold: int = 2              # 成功阈值(半开状态)
    monitor_period: float = 10.0            # 监控周期(秒)


@dataclass
class ASRRequestResult:
    """ASR请求结果"""
    success: bool
    data: Any = None
    error: Optional[Exception] = None
    attempt_count: int = 1
    total_time: float = 0.0
    retry_count: int = 0
    from_cache: bool = False
    from_fallback: bool = False


class CircuitBreaker:
    """断路器实现"""

    def __init__(self, config: CircuitBreakerConfig):
        """
        初始化断路器

        Args:
            config: 断路器配置
        """
        self.config = config
        self.state = CircuitState.CLOSED
        self.failure_count = 0
        self.success_count = 0
        self.last_failure_time = 0.0
        self.last_success_time = 0.0
        self.monitor_start_time = time.time()

        # 线程安全
        self._lock = threading.Lock()

        logger.info(f"CircuitBreaker 初始化完成")
        logger.info(f"  失败阈值: {config.failure_threshold}")
        logger.info(f"  恢复超时: {config.recovery_timeout}s")

    def call(self, func: Callable, *args, **kwargs) -> Any:
        """
        通过断路器调用函数

        Args:
            func: 要调用的函数
            *args: 函数参数
            **kwargs: 函数关键字参数

        Returns:
            Any: 函数执行结果

        Raises:
            Exception: 断路器打开时的异常
        """
        with self._lock:
            if self.state == CircuitState.OPEN:
                if self._should_attempt_reset():
                    self.state = CircuitState.HALF_OPEN
                    logger.info("断路器进入半开状态")
                else:
                    raise Exception("断路器打开，拒绝请求")

        try:
            result = func(*args, **kwargs)

            # 记录成功
            self._on_success()

            return result

        except Exception as e:
            # 记录失败
            self._on_failure(str(e))
            raise e

    def _should_attempt_reset(self) -> bool:
        """判断是否应该尝试重置"""
        return time.time() - self.last_failure_time >= self.config.recovery_timeout

    def _on_success(self) -> None:
        """处理成功调用"""
        self.success_count += 1
        self.last_success_time = time.time()

        if self.state == CircuitState.HALF_OPEN:
            if self.success_count >= self.config.success_threshold:
                self.state = CircuitState.CLOSED
                self.failure_count = 0
                self.success_count = 0
                logger.info("断路器恢复正常关闭状态")

    def _on_failure(self, error_msg: str) -> None:
        """处理失败调用"""
        self.failure_count += 1
        self.last_failure_time = time.time()

        # 检查是否是预期异常
        is_expected = any(expected in error_msg.lower()
                         for expected in self.config.expected_exception)

        if self.state == CircuitState.CLOSED and is_expected:
            if self.failure_count >= self.config.failure_threshold:
                self.state = CircuitState.OPEN
                logger.warning(f"断路器打开，失败次数: {self.failure_count}")

        elif self.state == CircuitState.HALF_OPEN:
            self.state = CircuitState.OPEN
            logger.warning("半开状态下失败，断路器重新打开")

    def get_state(self) -> Dict[str, Any]:
        """获取断路器状态"""
        with self._lock:
            return {
                "state": self.state.value,
                "failure_count": self.failure_count,
                "success_count": self.success_count,
                "last_failure_time": self.last_failure_time,
                "last_success_time": self.last_success_time,
                "uptime": time.time() - self.monitor_start_time
            }


class ASRRetryManager:
    """ASR重试管理器"""

    def __init__(self,
                 retry_config: Optional[RetryConfig] = None,
                 circuit_config: Optional[CircuitBreakerConfig] = None):
        """
        初始化ASR重试管理器

        Args:
            retry_config: 重试配置
            circuit_config: 断路器配置
        """
        self.retry_config = retry_config or RetryConfig()
        self.circuit_config = circuit_config or CircuitBreakerConfig()
        self.circuit_breaker = CircuitBreaker(self.circuit_config)

        # 缓存
        self.cache: Dict[str, Any] = {}
        self.cache_timeout = 300.0  # 5分钟缓存
        self.cache_lock = threading.Lock()

        # 统计信息
        self.total_requests = 0
        self.successful_requests = 0
        self.failed_requests = 0
        self.retry_count = 0
        self.cache_hits = 0
        self.fallback_usage = 0

        # 降级处理
        self.fallback_handlers: Dict[FallbackAction, Callable] = {}

        logger.info(f"ASRRetryManager 初始化完成")
        logger.info(f"  最大重试: {self.retry_config.max_retries}")
        logger.info(f"  重试策略: {self.retry_config.strategy.value}")

    def execute_with_retry(self,
                          func: Callable,
                          *args,
                          cache_key: Optional[str] = None,
                          fallback_action: FallbackAction = FallbackAction.THROW_ERROR,
                          fallback_result: Any = None,
                          **kwargs) -> ASRRequestResult:
        """
        执行带重试机制的函数调用

        Args:
            func: 要执行的函数
            *args: 函数参数
            cache_key: 缓存键
            fallback_action: 降级行为
            fallback_result: 降级结果
            **kwargs: 函数关键字参数

        Returns:
            ASRRequestResult: 执行结果
        """
        start_time = time.time()
        self.total_requests += 1

        # 检查缓存
        if cache_key:
            cached_result = self._get_from_cache(cache_key)
            if cached_result is not None:
                self.cache_hits += 1
                return ASRRequestResult(
                    success=True,
                    data=cached_result,
                    attempt_count=1,
                    total_time=time.time() - start_time,
                    from_cache=True
                )

        attempt_count = 0
        last_exception = None

        while attempt_count <= self.retry_config.max_retries:
            attempt_count += 1

            try:
                # 通过断路器执行
                result = self.circuit_breaker.call(func, *args, **kwargs)

                # 成功执行
                execution_time = time.time() - start_time

                # 缓存结果
                if cache_key:
                    self._store_in_cache(cache_key, result)

                self.successful_requests += 1

                return ASRRequestResult(
                    success=True,
                    data=result,
                    attempt_count=attempt_count,
                    total_time=execution_time,
                    retry_count=attempt_count - 1
                )

            except Exception as e:
                last_exception = e
                error_msg = str(e)

                logger.warning(f"ASR请求失败 (尝试 {attempt_count}/{self.retry_config.max_retries + 1}): {error_msg}")

                # 判断是否应该重试
                if attempt_count <= self.retry_config.max_retries and self._should_retry(error_msg):
                    # 计算延迟
                    delay = self._calculate_delay(attempt_count - 1)
                    time.sleep(delay)
                    self.retry_count += 1
                else:
                    break

        # 所有尝试都失败了，执行降级处理
        execution_time = time.time() - start_time
        self.failed_requests += 1

        fallback_result_data = self._execute_fallback(
            fallback_action, fallback_result, last_exception
        )

        return ASRRequestResult(
            success=False,
            data=fallback_result_data,
            error=last_exception,
            attempt_count=attempt_count,
            total_time=execution_time,
            retry_count=attempt_count - 1,
            from_fallback=True
        )

    def _should_retry(self, error_msg: str) -> bool:
        """判断是否应该重试"""
        error_lower = error_msg.lower()

        # 检查错误类型
        for retry_error in self.retry_config.retry_on_errors:
            if retry_error in error_lower:
                return True

        # 检查HTTP状态码
        import re
        status_match = re.search(r'status[_\s-]?code[:\s]+(\d+)', error_lower)
        if status_match:
            status_code = int(status_match.group(1))
            return status_code in self.retry_config.retry_on_status

        return False

    def _calculate_delay(self, retry_count: int) -> float:
        """计算重试延迟"""
        if self.retry_config.strategy == RetryStrategy.FIXED_DELAY:
            delay = self.retry_config.base_delay

        elif self.retry_config.strategy == RetryStrategy.LINEAR_BACKOFF:
            delay = self.retry_config.base_delay * (1 + retry_count)

        elif self.retry_config.strategy == RetryStrategy.EXPONENTIAL_BACKOFF:
            delay = self.retry_config.base_delay * (self.retry_config.backoff_factor ** retry_count)

        elif self.retry_config.strategy == RetryStrategy.ADAPTIVE:
            # 基于成功率的自适应延迟
            if self.total_requests > 0:
                success_rate = self.successful_requests / self.total_requests
                if success_rate < 0.5:
                    delay = self.retry_config.base_delay * (self.retry_config.backoff_factor ** retry_count)
                else:
                    delay = self.retry_config.base_delay
            else:
                delay = self.retry_config.base_delay

        else:
            delay = self.retry_config.base_delay

        # 限制最大延迟
        delay = min(delay, self.retry_config.max_delay)

        # 添加随机抖动
        if self.retry_config.jitter:
            import random
            jitter_factor = random.uniform(0.8, 1.2)
            delay *= jitter_factor

        return delay

    def _get_from_cache(self, cache_key: str) -> Optional[Any]:
        """从缓存获取数据"""
        with self.cache_lock:
            if cache_key in self.cache:
                cached_data, timestamp = self.cache[cache_key]
                if time.time() - timestamp < self.cache_timeout:
                    return cached_data
                else:
                    del self.cache[cache_key]
        return None

    def _store_in_cache(self, cache_key: str, data: Any) -> None:
        """存储数据到缓存"""
        with self.cache_lock:
            self.cache[cache_key] = (data, time.time())

            # 限制缓存大小
            if len(self.cache) > 100:
                # 删除最旧的条目
                oldest_key = min(self.cache.keys(),
                               key=lambda k: self.cache[k][1])
                del self.cache[oldest_key]

    def _execute_fallback(self,
                         action: FallbackAction,
                         default_result: Any,
                         error: Exception) -> Any:
        """执行降级处理"""
        self.fallback_usage += 1

        logger.warning(f"执行降级处理: {action.value}")

        if action == FallbackAction.RETURN_EMPTY:
            return None

        elif action == FallbackAction.RETURN_DEFAULT:
            return default_result

        elif action == FallbackAction.RETURN_CACHED:
            # 返回最近的成功结果
            with self.cache_lock:
                if self.cache:
                    latest_key = max(self.cache.keys(),
                                   key=lambda k: self.cache[k][1])
                    return self.cache[latest_key][0]
            return default_result

        elif action == FallbackAction.THROW_ERROR:
            raise error

        return default_result

    def register_fallback_handler(self,
                                action: FallbackAction,
                                handler: Callable[[Exception], Any]) -> None:
        """注册降级处理器"""
        self.fallback_handlers[action] = handler

    def get_statistics(self) -> Dict[str, Any]:
        """获取统计信息"""
        circuit_state = self.circuit_breaker.get_state()

        success_rate = (self.successful_requests / self.total_requests
                       if self.total_requests > 0 else 0.0)

        return {
            "total_requests": self.total_requests,
            "successful_requests": self.successful_requests,
            "failed_requests": self.failed_requests,
            "success_rate": success_rate,
            "retry_count": self.retry_count,
            "cache_hits": self.cache_hits,
            "fallback_usage": self.fallback_usage,
            "circuit_breaker": circuit_state,
            "cache_size": len(self.cache)
        }

    def reset_statistics(self) -> None:
        """重置统计信息"""
        self.total_requests = 0
        self.successful_requests = 0
        self.failed_requests = 0
        self.retry_count = 0
        self.cache_hits = 0
        self.fallback_usage = 0

        with self.cache_lock:
            self.cache.clear()

        logger.info("ASR重试管理器统计信息已重置")


# 工厂函数
def create_asr_retry_manager(max_retries: int = 3,
                            strategy: RetryStrategy = RetryStrategy.EXPONENTIAL_BACKOFF,
                            **kwargs) -> ASRRetryManager:
    """
    创建ASR重试管理器的工厂函数

    Args:
        max_retries: 最大重试次数
        strategy: 重试策略
        **kwargs: 其他配置参数

    Returns:
        ASRRetryManager: ASR重试管理器实例
    """
    retry_config = RetryConfig(
        max_retries=max_retries,
        strategy=strategy,
        **kwargs
    )

    return ASRRetryManager(retry_config=retry_config)


if __name__ == "__main__":
    # 测试代码
    logging.basicConfig(level=logging.INFO)

    print("🔄 ASR重试管理器测试")
    print("=" * 50)

    # 创建重试管理器
    retry_manager = create_asr_retry_manager(
        max_retries=2,
        strategy=RetryStrategy.EXPONENTIAL_BACKOFF
    )

    def test_function(success_rate: float = 0.7):
        """测试函数"""
        import random
        if random.random() < success_rate:
            return "成功结果"
        else:
            raise Exception("模拟失败")

    print(f"\n🧪 测试重试机制...")

    # 测试成功调用
    result = retry_manager.execute_with_retry(
        test_function,
        success_rate=0.8,
        cache_key="test_key"
    )

    print(f"结果: {result}")
    print(f"成功: {result.success}")
    print(f"重试次数: {result.retry_count}")

    # 获取统计信息
    stats = retry_manager.get_statistics()
    print(f"\n📊 统计信息:")
    for key, value in stats.items():
        print(f"  {key}: {value}")

    print("\n✅ 测试完成")