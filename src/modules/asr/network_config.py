#!/usr/bin/env python3
"""
ASR网络配置和错误处理模块

专门为ASR服务优化的网络配置：
- 网络连接管理
- 超时和重试配置
- 错误处理和恢复
- 网络状态监控

作者: Dev Agent
日期: 2025-11-08
Epic: 1 - ASR语音识别模块
Story: 1.1 - 粤语语音识别基础功能
Phase: 3 - 基础语音识别
Task: 3.3 - 配置网络连接和错误处理
"""

import logging
import time
import socket
import requests
from typing import Dict, List, Optional, Callable, Any
from dataclasses import dataclass, field
from enum import Enum
import threading
from queue import Queue, Empty
import urllib3

# 禁用SSL警告
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

logger = logging.getLogger(__name__)


class NetworkStatus(Enum):
    """网络状态枚举"""
    CONNECTED = "connected"
    DISCONNECTED = "disconnected"
    UNSTABLE = "unstable"
    TIMEOUT = "timeout"
    ERROR = "error"


class ErrorType(Enum):
    """错误类型枚举"""
    TIMEOUT_ERROR = "timeout_error"
    CONNECTION_ERROR = "connection_error"
    HTTP_ERROR = "http_error"
    AUTHENTICATION_ERROR = "auth_error"
    RATE_LIMIT_ERROR = "rate_limit_error"
    SERVER_ERROR = "server_error"
    UNKNOWN_ERROR = "unknown_error"


@dataclass
class NetworkConfig:
    """网络配置"""
    timeout: float = 10.0  # 默认超时时间(秒)
    connect_timeout: float = 5.0  # 连接超时时间(秒)
    read_timeout: float = 15.0  # 读取超时时间(秒)
    max_retries: int = 3  # 最大重试次数
    retry_delay: float = 1.0  # 重试延迟(秒)
    backoff_factor: float = 2.0  # 退避因子
    enable_keep_alive: bool = True  # 启用Keep-Alive
    keep_alive_timeout: float = 30.0  # Keep-Alive超时
    max_connections: int = 10  # 最大连接数
    pool_block: bool = False  # 连接池阻塞模式
    verify_ssl: bool = True  # SSL验证
    user_agent: str = "XleRobot-ASR-Service/1.0"  # 用户代理
    check_interval: float = 30.0  # 网络检查间隔(秒)
    error_threshold: int = 5  # 错误阈值


@dataclass
class ErrorInfo:
    """错误信息"""
    error_type: ErrorType
    error_code: Optional[int] = None
    error_message: str = ""
    timestamp: float = field(default_factory=time.time)
    retry_count: int = 0
    context: Dict[str, Any] = field(default_factory=dict)


class NetworkMonitor:
    """网络监控器"""

    def __init__(self, config: NetworkConfig):
        """
        初始化网络监控器

        Args:
            config: 网络配置
        """
        self.config = config
        self.status = NetworkStatus.DISCONNECTED
        self.last_check_time = 0.0
        self.check_interval = config.check_interval  # 检查间隔(秒)
        self.error_history: List[ErrorInfo] = []
        self.error_threshold = config.error_threshold  # 错误阈值

        # 统计信息
        self.total_requests = 0
        self.successful_requests = 0
        self.failed_requests = 0
        self.total_response_time = 0.0
        self.min_response_time = float('inf')
        self.max_response_time = 0.0

        # 线程安全
        self._lock = threading.Lock()

        # 监控线程
        self._monitor_thread = None
        self._monitoring = False

        logger.info(f"NetworkMonitor 初始化完成")
        logger.info(f"  检查间隔: {self.check_interval}s")
        logger.info(f"  错误阈值: {self.error_threshold}")

    def start_monitoring(self) -> None:
        """开始网络监控"""
        with self._lock:
            if self._monitoring:
                logger.warning("网络监控已在运行")
                return

            self._monitoring = True
            self._monitor_thread = threading.Thread(
                target=self._monitor_loop,
                daemon=True,
                name="NetworkMonitor"
            )
            self._monitor_thread.start()

        logger.info("✅ 网络监控已启动")

    def stop_monitoring(self) -> None:
        """停止网络监控"""
        with self._lock:
            self._monitoring = False
            if self._monitor_thread and self._monitor_thread.is_alive():
                self._monitor_thread.join(timeout=2.0)

        logger.info("⏹️ 网络监控已停止")

    def _monitor_loop(self) -> None:
        """监控循环"""
        while self._monitoring:
            try:
                self._check_network_status()
                time.sleep(self.check_interval)
            except Exception as e:
                logger.error(f"网络监控循环异常: {e}")
                time.sleep(5.0)  # 异常时短暂休眠

    def _check_network_status(self) -> None:
        """检查网络状态"""
        try:
            # 使用阿里云API端点进行连通性测试
            test_url = "https://nls-gateway-cn-shanghai.aliyuncs.com"

            start_time = time.time()
            response = requests.get(
                test_url,
                timeout=self.config.connect_timeout,
                headers={"User-Agent": self.config.user_agent}
            )
            response_time = time.time() - start_time

            with self._lock:
                self.last_check_time = time.time()

                if response.status_code == 200:
                    if self.status != NetworkStatus.CONNECTED:
                        logger.info(f"✅ 网络连接正常 (响应时间: {response_time:.3f}s)")
                    self.status = NetworkStatus.CONNECTED
                else:
                    if self.status != NetworkStatus.ERROR:
                        logger.warning(f"⚠️ 网络响应异常: {response.status_code}")
                    self.status = NetworkStatus.ERROR

        except requests.exceptions.Timeout:
            with self._lock:
                self.last_check_time = time.time()
                if self.status != NetworkStatus.TIMEOUT:
                    logger.warning("⏰ 网络连接超时")
                self.status = NetworkStatus.TIMEOUT

        except requests.exceptions.ConnectionError:
            with self._lock:
                self.last_check_time = time.time()
                if self.status != NetworkStatus.DISCONNECTED:
                    logger.error("❌ 网络连接断开")
                self.status = NetworkStatus.DISCONNECTED

        except Exception as e:
            with self._lock:
                self.last_check_time = time.time()
                if self.status != NetworkStatus.ERROR:
                    logger.error(f"🚫 网络检查异常: {e}")
                self.status = NetworkStatus.ERROR

    def record_request(self, success: bool, response_time: float,
                      error_info: Optional[ErrorInfo] = None) -> None:
        """
        记录请求结果

        Args:
            success: 是否成功
            response_time: 响应时间
            error_info: 错误信息(如果有)
        """
        with self._lock:
            self.total_requests += 1

            if success:
                self.successful_requests += 1
                self.total_response_time += response_time
                self.min_response_time = min(self.min_response_time, response_time)
                self.max_response_time = max(self.max_response_time, response_time)
            else:
                self.failed_requests += 1

                if error_info:
                    self.error_history.append(error_info)
                    # 保持最近100个错误记录
                    if len(self.error_history) > 100:
                        self.error_history = self.error_history[-100:]

    def get_status(self) -> NetworkStatus:
        """获取当前网络状态"""
        with self._lock:
            return self.status

    def get_statistics(self) -> Dict[str, Any]:
        """获取网络统计信息"""
        with self._lock:
            success_rate = (self.successful_requests / self.total_requests
                          if self.total_requests > 0 else 0.0)

            avg_response_time = (self.total_response_time / self.successful_requests
                               if self.successful_requests > 0 else 0.0)

            return {
                "status": self.status.value,
                "total_requests": self.total_requests,
                "successful_requests": self.successful_requests,
                "failed_requests": self.failed_requests,
                "success_rate": success_rate,
                "avg_response_time": avg_response_time,
                "min_response_time": self.min_response_time,
                "max_response_time": self.max_response_time,
                "last_check_time": self.last_check_time,
                "error_count": len(self.error_history)
            }

    def reset_statistics(self) -> None:
        """重置统计信息"""
        with self._lock:
            self.total_requests = 0
            self.successful_requests = 0
            self.failed_requests = 0
            self.total_response_time = 0.0
            self.min_response_time = float('inf')
            self.max_response_time = 0.0
            self.error_history.clear()

        logger.info("网络监控统计信息已重置")


class NetworkErrorHandler:
    """网络错误处理器"""

    def __init__(self, config: NetworkConfig):
        """
        初始化网络错误处理器

        Args:
            config: 网络配置
        """
        self.config = config
        self.error_callbacks: Dict[ErrorType, List[Callable]] = {}
        self.global_callbacks: List[Callable] = []
        self.retry_queue = Queue()

        logger.info(f"NetworkErrorHandler 初始化完成")
        logger.info(f"  最大重试次数: {config.max_retries}")
        logger.info(f"  重试延迟: {config.retry_delay}s")

    def register_error_callback(self, error_type: ErrorType,
                              callback: Callable[[ErrorInfo], None]) -> None:
        """
        注册错误回调函数

        Args:
            error_type: 错误类型
            callback: 回调函数
        """
        if error_type not in self.error_callbacks:
            self.error_callbacks[error_type] = []
        self.error_callbacks[error_type].append(callback)

        logger.debug(f"注册错误回调: {error_type.value}")

    def register_global_callback(self, callback: Callable[[ErrorInfo], None]) -> None:
        """
        注册全局错误回调函数

        Args:
            callback: 回调函数
        """
        self.global_callbacks.append(callback)
        logger.debug("注册全局错误回调")

    def handle_error(self, error_info: ErrorInfo) -> bool:
        """
        处理网络错误

        Args:
            error_info: 错误信息

        Returns:
            bool: 是否应该重试
        """
        logger.warning(f"处理网络错误: {error_info.error_type.value} - {error_info.error_message}")

        # 调用特定类型的错误回调
        if error_info.error_type in self.error_callbacks:
            for callback in self.error_callbacks[error_info.error_type]:
                try:
                    callback(error_info)
                except Exception as e:
                    logger.error(f"错误回调执行失败: {e}")

        # 调用全局错误回调
        for callback in self.global_callbacks:
            try:
                callback(error_info)
            except Exception as e:
                logger.error(f"全局错误回调执行失败: {e}")

        # 根据错误类型决定是否重试
        return self._should_retry(error_info)

    def _should_retry(self, error_info: ErrorInfo) -> bool:
        """判断是否应该重试"""
        if error_info.retry_count >= self.config.max_retries:
            return False

        # 超时错误和连接错误可以重试
        if error_info.error_type in [ErrorType.TIMEOUT_ERROR, ErrorType.CONNECTION_ERROR]:
            return True

        # 5xx服务器错误可以重试
        if (error_info.error_type == ErrorType.SERVER_ERROR and
            error_info.error_code and 500 <= error_info.error_code < 600):
            return True

        # 429限流错误可以重试
        if (error_info.error_type == ErrorType.RATE_LIMIT_ERROR and
            error_info.error_code == 429):
            return True

        return False

    def calculate_retry_delay(self, retry_count: int) -> float:
        """
        计算重试延迟

        Args:
            retry_count: 重试次数

        Returns:
            float: 延迟时间(秒)
        """
        # 指数退避算法
        delay = self.config.retry_delay * (self.config.backoff_factor ** retry_count)

        # 添加随机抖动，避免同时重试
        import random
        jitter = random.uniform(0.8, 1.2)

        return delay * jitter


class ASRNetworkManager:
    """ASR网络管理器"""

    def __init__(self, config: Optional[NetworkConfig] = None):
        """
        初始化ASR网络管理器

        Args:
            config: 网络配置
        """
        self.config = config or NetworkConfig()
        self.monitor = NetworkMonitor(self.config)
        self.error_handler = NetworkErrorHandler(self.config)

        # HTTP会话
        self.session = self._create_session()

        logger.info(f"ASRNetworkManager 初始化完成")
        logger.info(f"  超时时间: {self.config.timeout}s")
        logger.info(f"  最大重试: {self.config.max_retries}")

    def _create_session(self) -> requests.Session:
        """创建HTTP会话"""
        session = requests.Session()

        # 设置超时
        session.timeout = (self.config.connect_timeout, self.config.read_timeout)

        # 设置连接池
        adapter = requests.adapters.HTTPAdapter(
            max_retries=0  # 我们自己处理重试
        )

        session.mount('http://', adapter)
        session.mount('https://', adapter)

        # 设置默认头部
        session.headers.update({
            'User-Agent': self.config.user_agent,
            'Connection': 'keep-alive' if self.config.enable_keep_alive else 'close'
        })

        # SSL设置
        session.verify = self.config.verify_ssl

        return session

    def start(self) -> None:
        """启动网络管理器"""
        self.monitor.start_monitoring()
        logger.info("ASR网络管理器已启动")

    def stop(self) -> None:
        """停止网络管理器"""
        self.monitor.stop_monitoring()
        if self.session:
            self.session.close()
        logger.info("ASR网络管理器已停止")

    def make_request(self, method: str, url: str, **kwargs) -> Optional[requests.Response]:
        """
        发送HTTP请求

        Args:
            method: HTTP方法
            url: 请求URL
            **kwargs: 其他请求参数

        Returns:
            requests.Response: 响应对象，失败时返回None
        """
        start_time = time.time()
        retry_count = 0

        while retry_count <= self.config.max_retries:
            try:
                # 发送请求
                response = self.session.request(method, url, **kwargs)
                response_time = time.time() - start_time

                # 检查响应状态
                if response.status_code == 200:
                    # 记录成功请求
                    self.monitor.record_request(True, response_time)
                    return response
                else:
                    # 处理HTTP错误
                    error_info = self._create_http_error_info(response, retry_count)
                    should_retry = self.error_handler.handle_error(error_info)

                    if should_retry:
                        retry_count += 1
                        delay = self.error_handler.calculate_retry_delay(retry_count)
                        time.sleep(delay)
                        continue
                    else:
                        # 记录失败请求
                        self.monitor.record_request(False, response_time, error_info)
                        return response

            except requests.exceptions.Timeout as e:
                response_time = time.time() - start_time
                error_info = ErrorInfo(
                    error_type=ErrorType.TIMEOUT_ERROR,
                    error_message=str(e),
                    retry_count=retry_count,
                    context={"url": url, "method": method}
                )

                should_retry = self.error_handler.handle_error(error_info)
                if should_retry and retry_count < self.config.max_retries:
                    retry_count += 1
                    delay = self.error_handler.calculate_retry_delay(retry_count)
                    time.sleep(delay)
                    continue
                else:
                    self.monitor.record_request(False, response_time, error_info)
                    return None

            except requests.exceptions.ConnectionError as e:
                response_time = time.time() - start_time
                error_info = ErrorInfo(
                    error_type=ErrorType.CONNECTION_ERROR,
                    error_message=str(e),
                    retry_count=retry_count,
                    context={"url": url, "method": method}
                )

                should_retry = self.error_handler.handle_error(error_info)
                if should_retry and retry_count < self.config.max_retries:
                    retry_count += 1
                    delay = self.error_handler.calculate_retry_delay(retry_count)
                    time.sleep(delay)
                    continue
                else:
                    self.monitor.record_request(False, response_time, error_info)
                    return None

            except Exception as e:
                response_time = time.time() - start_time
                error_info = ErrorInfo(
                    error_type=ErrorType.UNKNOWN_ERROR,
                    error_message=str(e),
                    retry_count=retry_count,
                    context={"url": url, "method": method}
                )

                self.monitor.record_request(False, response_time, error_info)
                return None

        return None

    def _create_http_error_info(self, response: requests.Response,
                               retry_count: int) -> ErrorInfo:
        """创建HTTP错误信息"""
        # 根据状态码确定错误类型
        if response.status_code == 401:
            error_type = ErrorType.AUTHENTICATION_ERROR
        elif response.status_code == 429:
            error_type = ErrorType.RATE_LIMIT_ERROR
        elif 500 <= response.status_code < 600:
            error_type = ErrorType.SERVER_ERROR
        else:
            error_type = ErrorType.HTTP_ERROR

        return ErrorInfo(
            error_type=error_type,
            error_code=response.status_code,
            error_message=response.text[:200],  # 限制错误消息长度
            retry_count=retry_count,
            context={"url": response.url, "status_code": response.status_code}
        )

    def get_network_status(self) -> Dict[str, Any]:
        """获取网络状态"""
        stats = self.monitor.get_statistics()
        stats["is_connected"] = self.monitor.get_status() == NetworkStatus.CONNECTED
        return stats

    def reset_statistics(self) -> None:
        """重置统计信息"""
        self.monitor.reset_statistics()
        logger.info("ASR网络管理器统计信息已重置")


# 工厂函数
def create_asr_network_manager(timeout: float = 10.0,
                               max_retries: int = 3,
                               **kwargs) -> ASRNetworkManager:
    """
    创建ASR网络管理器的工厂函数

    Args:
        timeout: 超时时间
        max_retries: 最大重试次数
        **kwargs: 其他配置参数

    Returns:
        ASRNetworkManager: ASR网络管理器实例
    """
    config = NetworkConfig(
        timeout=timeout,
        max_retries=max_retries,
        **kwargs
    )

    return ASRNetworkManager(config)


if __name__ == "__main__":
    # 测试代码
    logging.basicConfig(level=logging.INFO)

    print("🌐 ASR网络管理器测试")
    print("=" * 50)

    # 创建网络管理器
    manager = create_asr_network_manager(
        timeout=5.0,
        max_retries=2
    )

    # 启动管理器
    manager.start()

    print(f"\n🔗 测试网络连接...")

    # 测试连接
    response = manager.make_request(
        "GET",
        "https://nls-gateway-cn-shanghai.aliyuncs.com"
    )

    if response:
        print(f"✅ 连接成功 (状态码: {response.status_code})")
    else:
        print("❌ 连接失败")

    # 获取网络状态
    status = manager.get_network_status()
    print(f"\n📊 网络状态:")
    for key, value in status.items():
        print(f"  {key}: {value}")

    # 停止管理器
    manager.stop()

    print("\n✅ 测试完成")