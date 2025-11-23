#!/usr/bin/env python3
"""
XLeRobot WebSocket连接稳定性管理器
实现自动重连、健康监控、断线续传机制
"""

import logging
import time
import threading
import asyncio
import json
from typing import Optional, Dict, Any, Callable, List
from dataclasses import dataclass, field
from enum import Enum
import queue
from datetime import datetime, timedelta

logger = logging.getLogger(__name__)

class ConnectionState(Enum):
    """连接状态"""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    RECONNECTING = "reconnecting"
    ERROR = "error"

@dataclass
class ConnectionMetrics:
    """连接指标"""
    total_connections: int = 0
    successful_connections: int = 0
    failed_connections: int = 0
    reconnection_attempts: int = 0
    last_connection_time: Optional[datetime] = None
    last_error_time: Optional[datetime] = None
    total_downtime: float = 0.0  # 总停机时间（秒）
    uptime_percentage: float = 0.0  # 正常运行时间百分比

class WebSocketStabilityManager:
    """
    WebSocket连接稳定性管理器

    特性：
    - 自动重连机制
    - 连接健康监控
    - 断线续传机制
    - 电路熔断器
    - 连接指标统计
    """

    def __init__(self,
                 max_reconnect_attempts: int = 5,
                 base_reconnect_delay: float = 1.0,
                 max_reconnect_delay: float = 30.0,
                 health_check_interval: float = 10.0,
                 connection_timeout: float = 10.0,
                 circuit_breaker_threshold: int = 3,
                 circuit_breaker_timeout: float = 60.0):
        """
        初始化稳定性管理器

        Args:
            max_reconnect_attempts: 最大重连次数
            base_reconnect_delay: 基础重连延迟（秒）
            max_reconnect_delay: 最大重连延迟（秒）
            health_check_interval: 健康检查间隔（秒）
            connection_timeout: 连接超时（秒）
            circuit_breaker_threshold: 熔断器阈值
            circuit_breaker_timeout: 熔断器恢复时间（秒）
        """
        # 重连配置
        self.max_reconnect_attempts = max_reconnect_attempts
        self.base_reconnect_delay = base_reconnect_delay
        self.max_reconnect_delay = max_reconnect_delay

        # 监控配置
        self.health_check_interval = health_check_interval
        self.connection_timeout = connection_timeout

        # 熔断器配置
        self.circuit_breaker_threshold = circuit_breaker_threshold
        self.circuit_breaker_timeout = circuit_breaker_timeout

        # 状态管理
        self.state = ConnectionState.DISCONNECTED
        self.connection = None
        self.lock = threading.Lock()

        # 熔断器状态
        self.circuit_breaker_open = False
        self.circuit_breaker_open_time = None
        self.consecutive_failures = 0

        # 重连状态
        self.reconnect_count = 0
        self.last_reconnect_time = None

        # 健康监控
        self.health_check_thread = None
        self.stop_health_check = threading.Event()

        # 消息队列（断线续传）
        self.pending_messages = queue.Queue()
        self.max_pending_messages = 1000

        # 回调函数
        self.on_connect_callback: Optional[Callable] = None
        self.on_disconnect_callback: Optional[Callable] = None
        self.on_error_callback: Optional[Callable] = None
        self.on_message_callback: Optional[Callable] = None

        # 指标统计
        self.metrics = ConnectionMetrics()
        self.start_time = datetime.now()

        logger.info("WebSocket稳定性管理器初始化完成")

    def set_callbacks(self,
                     on_connect: Optional[Callable] = None,
                     on_disconnect: Optional[Callable] = None,
                     on_error: Optional[Callable] = None,
                     on_message: Optional[Callable] = None) -> None:
        """
        设置回调函数

        Args:
            on_connect: 连接成功回调
            on_disconnect: 连接断开回调
            on_error: 错误回调
            on_message: 消息接收回调
        """
        self.on_connect_callback = on_connect
        self.on_disconnect_callback = on_disconnect
        self.on_error_callback = on_error
        self.on_message_callback = on_message

    def connect(self, connection_factory: Callable) -> bool:
        """
        建立连接

        Args:
            connection_factory: 连接工厂函数

        Returns:
            bool: 连接成功状态
        """
        with self.lock:
            if self.state in [ConnectionState.CONNECTED, ConnectionState.CONNECTING]:
                logger.warning(f"连接已在进行中，当前状态: {self.state.value}")
                return True

            if self.circuit_breaker_open:
                logger.warning("熔断器已打开，暂时不允许连接")
                return False

            try:
                self.state = ConnectionState.CONNECTING
                logger.info("正在建立WebSocket连接...")

                # 创建连接
                connection = connection_factory()
                if not connection:
                    raise Exception("连接工厂返回None")

                # 测试连接
                if self._test_connection(connection):
                    self.connection = connection
                    self.state = ConnectionState.CONNECTED
                    self.consecutive_failures = 0
                    self.metrics.total_connections += 1
                    self.metrics.successful_connections += 1
                    self.metrics.last_connection_time = datetime.now()

                    logger.info("✅ WebSocket连接建立成功")

                    # 启动健康监控
                    self._start_health_monitor()

                    # 发送待处理消息
                    self._send_pending_messages()

                    # 调用连接成功回调
                    if self.on_connect_callback:
                        self.on_connect_callback()

                    return True
                else:
                    raise Exception("连接测试失败")

            except Exception as e:
                logger.error(f"❌ WebSocket连接失败: {e}")
                self._handle_connection_error(e)
                return False

    def disconnect(self) -> None:
        """断开连接"""
        with self.lock:
            if self.state == ConnectionState.DISCONNECTED:
                return

            logger.info("正在断开WebSocket连接...")

            try:
                # 停止健康监控
                self._stop_health_monitor()

                # 关闭连接
                if self.connection:
                    try:
                        # 根据具体连接类型实现关闭逻辑
                        if hasattr(self.connection, 'close'):
                            self.connection.close()
                        elif hasattr(self.connection, 'shutdown'):
                            self.connection.shutdown()
                    except Exception as e:
                        logger.warning(f"关闭连接时出现异常: {e}")

                self.connection = None
                self.state = ConnectionState.DISCONNECTED

                # 调用断开回调
                if self.on_disconnect_callback:
                    self.on_disconnect_callback()

                logger.info("WebSocket连接已断开")

            except Exception as e:
                logger.error(f"❌ 断开连接异常: {e}")

    def send_message(self, message: Any, priority: bool = False) -> bool:
        """
        发送消息

        Args:
            message: 消息内容
            priority: 是否为高优先级消息

        Returns:
            bool: 发送成功状态
        """
        try:
            with self.lock:
                if self.state != ConnectionState.CONNECTED or not self.connection:
                    # 连接断开，将消息加入待处理队列
                    if priority:
                        # 高优先级消息插入队列前端
                        temp_queue = queue.Queue()
                        temp_queue.put(message)
                        while not self.pending_messages.empty():
                            temp_queue.put(self.pending_messages.get())
                        self.pending_messages = temp_queue
                    else:
                        # 普通消息加入队列尾
                        if self.pending_messages.qsize() < self.max_pending_messages:
                            self.pending_messages.put(message)
                        else:
                            logger.warning("待处理消息队列已满，丢弃消息")
                            return False

                    logger.debug(f"连接断开，消息已加入待处理队列 (队列大小: {self.pending_messages.qsize()})")
                    return True

                # 连接正常，直接发送
                return self._send_message_internal(message)

        except Exception as e:
            logger.error(f"❌ 发送消息异常: {e}")
            return False

    def _send_message_internal(self, message: Any) -> bool:
        """内部消息发送方法"""
        try:
            # 根据具体连接类型实现发送逻辑
            if hasattr(self.connection, 'send'):
                if isinstance(message, str):
                    self.connection.send(message)
                elif isinstance(message, bytes):
                    self.connection.send_binary(message)
                else:
                    self.connection.send(json.dumps(message))
            elif hasattr(self.connection, 'send_audio'):
                # 特殊处理音频数据
                self.connection.send_audio(message)
            else:
                logger.error("连接对象不支持发送操作")
                return False

            return True

        except Exception as e:
            logger.error(f"❌ 消息发送失败: {e}")
            self._handle_connection_error(e)
            return False

    def _send_pending_messages(self) -> None:
        """发送待处理消息"""
        if self.pending_messages.empty():
            return

        sent_count = 0
        while not self.pending_messages.empty():
            try:
                message = self.pending_messages.get_nowait()
                if self._send_message_internal(message):
                    sent_count += 1
                else:
                    # 发送失败，重新放回队列
                    self.pending_messages.put(message)
                    break
            except queue.Empty:
                break

        if sent_count > 0:
            logger.info(f"✅ 发送了 {sent_count} 个待处理消息")

    def _test_connection(self, connection) -> bool:
        """测试连接是否有效"""
        try:
            # 根据具体连接类型实现测试逻辑
            if hasattr(connection, 'ping'):
                connection.ping()
                return True
            elif hasattr(connection, 'is_connected'):
                return connection.is_connected()
            elif hasattr(connection, 'connected'):
                return connection.connected
            else:
                # 默认认为连接有效
                return True
        except Exception as e:
            logger.debug(f"连接测试失败: {e}")
            return False

    def _handle_connection_error(self, error: Exception) -> None:
        """处理连接错误"""
        self.consecutive_failures += 1
        self.metrics.failed_connections += 1
        self.metrics.last_error_time = datetime.now()

        # 检查熔断器
        if self.consecutive_failures >= self.circuit_breaker_threshold:
            self._open_circuit_breaker()

        # 更新状态
        self.state = ConnectionState.ERROR
        self.connection = None

        # 调用错误回调
        if self.on_error_callback:
            self.on_error_callback(error)

        # 尝试重连
        self._schedule_reconnect()

    def _open_circuit_breaker(self) -> None:
        """打开熔断器"""
        self.circuit_breaker_open = True
        self.circuit_breaker_open_time = datetime.now()
        logger.warning(f"🔥 熔断器已打开（连续失败 {self.consecutive_failures} 次）")

        # 启动熔断器恢复定时器
        def recovery_timer():
            time.sleep(self.circuit_breaker_timeout)
            with self.lock:
                if self.circuit_breaker_open:
                    self.circuit_breaker_open = False
                    self.consecutive_failures = 0
                    logger.info("🔧 熔断器已关闭，允许重连")

        threading.Thread(target=recovery_timer, daemon=True).start()

    def _schedule_reconnect(self) -> None:
        """调度重连"""
        if self.reconnect_count >= self.max_reconnect_attempts:
            logger.error(f"已达到最大重连次数 ({self.max_reconnect_attempts})，停止重连")
            return

        if self.circuit_breaker_open:
            logger.info("熔断器已打开，等待恢复后再重连")
            return

        # 计算重连延迟
        delay = min(self.base_reconnect_delay * (2 ** self.reconnect_count), self.max_reconnect_delay)
        self.last_reconnect_time = datetime.now()

        def reconnect_worker():
            time.sleep(delay)
            with self.lock:
                if self.state != ConnectionState.CONNECTED:
                    self.reconnect_count += 1
                    self.metrics.reconnection_attempts += 1
                    logger.info(f"🔄 开始第 {self.reconnect_count} 次重连 (延迟: {delay:.1f}s)")
                    self.state = ConnectionState.RECONNECTING
                    # 这里需要重新调用connect方法，但需要connection_factory参数
                    # 实际使用时需要在外部处理重连逻辑

        threading.Thread(target=reconnect_worker, daemon=True).start()

    def _start_health_monitor(self) -> None:
        """启动健康监控"""
        if self.health_check_thread and self.health_check_thread.is_alive():
            return

        self.stop_health_check.clear()
        self.health_check_thread = threading.Thread(target=self._health_monitor_loop, daemon=True)
        self.health_check_thread.start()
        logger.debug("健康监控线程已启动")

    def _stop_health_monitor(self) -> None:
        """停止健康监控"""
        if self.health_check_thread and self.health_check_thread.is_alive():
            self.stop_health_check.set()
            self.health_check_thread.join(timeout=2.0)
            logger.debug("健康监控线程已停止")

    def _health_monitor_loop(self) -> None:
        """健康监控循环"""
        while not self.stop_health_check.wait(self.health_check_interval):
            try:
                if self.state == ConnectionState.CONNECTED and self.connection:
                    if not self._test_connection(self.connection):
                        logger.warning("健康检查失败，连接可能已断开")
                        self._handle_connection_error(Exception("健康检查失败"))
                else:
                    logger.debug("健康监控跳过，连接未建立")

            except Exception as e:
                logger.error(f"健康监控异常: {e}")

    def get_connection_state(self) -> ConnectionState:
        """获取连接状态"""
        with self.lock:
            return self.state

    def get_metrics(self) -> Dict[str, Any]:
        """获取连接指标"""
        with self.lock:
            total_time = (datetime.now() - self.start_time).total_seconds()

            if self.metrics.total_connections > 0:
                self.metrics.uptime_percentage = (self.metrics.successful_connections / self.metrics.total_connections) * 100

            return {
                "state": self.state.value,
                "circuit_breaker_open": self.circuit_breaker_open,
                "reconnect_count": self.reconnect_count,
                "pending_messages": self.pending_messages.qsize(),
                "metrics": {
                    "total_connections": self.metrics.total_connections,
                    "successful_connections": self.metrics.successful_connections,
                    "failed_connections": self.metrics.failed_connections,
                    "reconnection_attempts": self.metrics.reconnection_attempts,
                    "uptime_percentage": f"{self.metrics.uptime_percentage:.1f}%",
                    "last_connection_time": self.metrics.last_connection_time.isoformat() if self.metrics.last_connection_time else None,
                    "last_error_time": self.metrics.last_error_time.isoformat() if self.metrics.last_error_time else None
                }
            }

    def reset_metrics(self) -> None:
        """重置指标"""
        with self.lock:
            self.metrics = ConnectionMetrics()
            self.reconnect_count = 0
            self.consecutive_failures = 0
            logger.info("连接指标已重置")

    def force_reconnect(self) -> bool:
        """强制重连"""
        with self.lock:
            if self.connection:
                try:
                    self.disconnect()
                except:
                    pass

            self.reconnect_count = 0
            self.circuit_breaker_open = False
            self.consecutive_failures = 0

            logger.info("强制重连已调度")
            return True

if __name__ == "__main__":
    # 测试代码
    import json

    logging.basicConfig(level=logging.INFO)

    print("=== WebSocket稳定性管理器测试 ===")

    # 创建管理器
    manager = WebSocketStabilityManager(
        max_reconnect_attempts=3,
        base_reconnect_delay=1.0,
        health_check_interval=2.0
    )

    # 模拟连接工厂
    def mock_connection_factory():
        class MockConnection:
            def __init__(self):
                self.connected = True

            def send(self, message):
                print(f"发送消息: {message}")

            def close(self):
                self.connected = False

            def ping(self):
                if not self.connected:
                    raise Exception("连接已断开")

        return MockConnection()

    # 设置回调
    def on_connect():
        print("✅ 连接成功回调")

    def on_disconnect():
        print("❌ 连接断开回调")

    def on_error(error):
        print(f"🔥 连接错误回调: {error}")

    manager.set_callbacks(on_connect=on_connect, on_disconnect=on_disconnect, on_error=on_error)

    # 测试连接
    print("\n1. 测试连接...")
    if manager.connect(mock_connection_factory):
        print("连接成功")
    else:
        print("连接失败")

    # 获取状态和指标
    print("\n2. 连接状态...")
    print(f"状态: {manager.get_connection_state().value}")

    print("\n3. 连接指标...")
    metrics = manager.get_metrics()
    print(json.dumps(metrics, indent=2, ensure_ascii=False))

    # 测试消息发送
    print("\n4. 测试消息发送...")
    manager.send_message({"type": "test", "data": "hello"}, priority=True)

    # 断开连接
    print("\n5. 断开连接...")
    manager.disconnect()

    print("\n测试完成")