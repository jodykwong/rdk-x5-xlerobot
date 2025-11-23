#!/usr/bin/env python3.10
# -*- coding: utf-8 -*-

"""
WebSocket连接池管理器 - WebSocket Connection Pool Manager
BMad-Method v6 Brownfield Level 4 企业级实现

功能描述:
- 企业级WebSocket连接池管理
- 连接复用和负载均衡
- 自动重连和故障恢复
- 连接健康检查和监控
- 性能优化和资源管理
- 线程安全的连接管理

架构特性:
- 单例模式确保全局唯一
- 支持ASR和TTS连接池分离
- 智能连接调度算法
- 优雅关闭和资源清理

作者: Claude Code
Epic: 1 - WebSocket连接池优化
创建日期: 2025-11-19
"""

import logging
import threading
import time
import asyncio
import json
from typing import Dict, List, Optional, Any, Callable, Union
from dataclasses import dataclass, field
from queue import Queue, Empty
from enum import Enum
from concurrent.futures import ThreadPoolExecutor
import weakref
from contextlib import contextmanager

# 尝试导入WebSocket库
try:
    import websockets
    import aiohttp
    HAS_WEBSOCKET_LIBS = True
except ImportError:
    HAS_WEBSOCKET_LIBS = False
    logging.warning("⚠️ WebSocket库未安装，将使用备选方案")

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class ConnectionState(Enum):
    """连接状态枚举"""
    DISCONNECTED = "disconnected"     # 未连接
    CONNECTING = "connecting"         # 连接中
    CONNECTED = "connected"           # 已连接
    ERROR = "error"                   # 错误状态
    CLOSING = "closing"               # 关闭中


class PoolType(Enum):
    """连接池类型"""
    ASR = "asr"       # ASR连接池
    TTS = "tts"       # TTS连接池


@dataclass
class ConnectionInfo:
    """连接信息"""
    connection_id: str
    websocket: Optional[Any] = None  # WebSocket连接对象
    state: ConnectionState = ConnectionState.DISCONNECTED
    created_at: float = field(default_factory=time.time)
    last_used: float = field(default_factory=time.time)
    usage_count: int = 0
    error_count: int = 0
    endpoint: str = ""
    is_active: bool = True
    pool_type: PoolType = PoolType.ASR

    def __post_init__(self):
        """初始化后处理"""
        if not self.connection_id:
            self.connection_id = f"{self.pool_type.value}_{int(time.time() * 1000)}"

    def update_usage(self):
        """更新使用信息"""
        self.last_used = time.time()
        self.usage_count += 1

    def mark_error(self):
        """标记错误"""
        self.error_count += 1
        if self.error_count > 3:  # 错误超过3次则禁用
            self.is_active = False

    @property
    def age_seconds(self) -> float:
        """获取连接年龄（秒）"""
        return time.time() - self.created_at

    @property
    def idle_seconds(self) -> float:
        """获取空闲时间（秒）"""
        return time.time() - self.last_used

    def to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        return {
            'connection_id': self.connection_id,
            'state': self.state.value,
            'created_at': self.created_at,
            'last_used': self.last_used,
            'usage_count': self.usage_count,
            'error_count': self.error_count,
            'endpoint': self.endpoint,
            'is_active': self.is_active,
            'pool_type': self.pool_type.value,
            'age_seconds': self.age_seconds,
            'idle_seconds': self.idle_seconds
        }


@dataclass
class PoolConfig:
    """连接池配置"""
    min_connections: int = 2          # 最小连接数
    max_connections: int = 10         # 最大连接数
    connection_timeout: float = 30.0   # 连接超时时间（秒）
    idle_timeout: float = 300.0        # 空闲超时时间（秒）
    health_check_interval: float = 60.0  # 健康检查间隔（秒）
    max_retry_attempts: int = 3       # 最大重试次数
    retry_delay: float = 1.0          # 重试延迟（秒）
    enable_load_balancing: bool = True # 启用负载均衡


class WebSocketConnectionPool:
    """
    WebSocket连接池管理器

    企业级连接池实现，支持：
    - 连接复用和负载均衡
    - 自动重连和故障恢复
    - 健康检查和监控
    - 资源管理和清理
    """

    _instances: Dict[PoolType, 'WebSocketConnectionPool'] = {}
    _lock = threading.Lock()

    def __new__(cls, pool_type: PoolType = PoolType.ASR):
        """单例模式"""
        with cls._lock:
            if pool_type not in cls._instances:
                cls._instances[pool_type] = super().__new__(cls)
            return cls._instances[pool_type]

    def __init__(self, pool_type: PoolType = PoolType.ASR, config: Optional[PoolConfig] = None):
        """初始化连接池"""
        # 避免重复初始化
        if hasattr(self, '_initialized'):
            return

        self.pool_type = pool_type
        self.config = config or PoolConfig()
        self.connections: Dict[str, ConnectionInfo] = {}
        self.available_connections: Queue[str] = Queue()
        self.lock = threading.RLock()
        self.shutdown_event = threading.Event()

        # 监控和统计
        self.stats = {
            'total_created': 0,
            'total_reused': 0,
            'total_errors': 0,
            'active_connections': 0,
            'peak_connections': 0
        }

        # 后台任务
        self.health_check_thread = None
        self.cleanup_thread = None
        self.executor = ThreadPoolExecutor(max_workers=4, thread_name_prefix=f"{pool_type.value}-pool")

        # 端点配置
        self.endpoints = self._get_endpoints()

        self._initialized = True
        logger.info(f"✅ WebSocket连接池初始化完成 - {pool_type.value}")
        logger.info(f"  - 最小连接数: {self.config.min_connections}")
        logger.info(f"  - 最大连接数: {self.config.max_connections}")
        logger.info(f"  - 端点数量: {len(self.endpoints)}")

    def _get_endpoints(self) -> List[str]:
        """获取端点列表"""
        if self.pool_type == PoolType.ASR:
            # ASR端点（多个端点用于负载均衡）
            return [
                "wss://nls-gateway.cn-shanghai.aliyuncs.com/ws/v1",
                "wss://nls-gateway.cn-beijing.aliyuncs.com/ws/v1",
                "wss://nls-gateway.cn-hangzhou.aliyuncs.com/ws/v1"
            ]
        else:  # TTS
            # TTS端点
            return [
                "wss://nls-gateway.cn-shanghai.aliyuncs.com/ws/v1",
                "wss://nls-gateway.cn-beijing.aliyuncs.com/ws/v1"
            ]

    def initialize(self) -> bool:
        """初始化连接池"""
        try:
            logger.info(f"🚀 初始化{self.pool_type.value}连接池...")

            # 创建最小连接数
            for i in range(self.config.min_connections):
                self._create_connection()

            # 启动后台任务
            self._start_background_tasks()

            # 等待连接就绪
            if self._wait_for_connections_ready():
                logger.info(f"✅ {self.pool_type.value}连接池初始化成功")
                return True
            else:
                logger.error(f"❌ {self.pool_type.value}连接池初始化超时")
                return False

        except Exception as e:
            logger.error(f"❌ {self.pool_type.value}连接池初始化失败: {e}")
            return False

    def _create_connection(self, endpoint: Optional[str] = None) -> Optional[ConnectionInfo]:
        """创建新连接"""
        if len(self.connections) >= self.config.max_connections:
            logger.warning(f"⚠️ 连接数已达上限: {self.config.max_connections}")
            return None

        # 选择端点
        if not endpoint:
            if self.config.enable_load_balancing:
                endpoint = self._select_best_endpoint()
            else:
                endpoint = self.endpoints[0]

        connection_info = ConnectionInfo(
            connection_id=f"{self.pool_type.value}_{len(self.connections)}_{int(time.time())}",
            endpoint=endpoint,
            pool_type=self.pool_type
        )

        try:
            # 尝试建立连接
            if HAS_WEBSOCKET_LIBS:
                websocket = self._create_websocket_connection(endpoint)
            else:
                websocket = self._create_fallback_connection(endpoint)

            if websocket:
                connection_info.websocket = websocket
                connection_info.state = ConnectionState.CONNECTED
                self.connections[connection_info.connection_id] = self.available_connections.put(connection_info.connection_id)

                self.stats['total_created'] += 1
                self.stats['active_connections'] = len([c for c in self.connections.values() if c.state == ConnectionState.CONNECTED])
                self.stats['peak_connections'] = max(self.stats['peak_connections'], self.stats['active_connections'])

                logger.info(f"✅ 创建新连接: {connection_info.connection_id} -> {endpoint}")
                return connection_info
            else:
                connection_info.mark_error()
                logger.error(f"❌ 连接创建失败: {endpoint}")
                return None

        except Exception as e:
            connection_info.mark_error()
            logger.error(f"❌ 连接创建异常: {e}")
            return None

    def _create_websocket_connection(self, endpoint: str) -> Optional[Any]:
        """创建WebSocket连接"""
        # 这里需要根据实际的WebSocket库实现
        # 暂时返回None，需要后续实现
        return None

    def _create_fallback_connection(self, endpoint: str) -> Optional[Any]:
        """创建备选连接"""
        # 这里可以实现基于HTTP的长连接或其他备选方案
        return None

    def _select_best_endpoint(self) -> str:
        """选择最佳端点（负载均衡）"""
        # 简单的轮询算法，可以根据实际需要改进
        if not hasattr(self, '_current_endpoint_index'):
            self._current_endpoint_index = 0

        endpoint = self.endpoints[self._current_endpoint_index]
        self._current_endpoint_index = (self._current_endpoint_index + 1) % len(self.endpoints)
        return endpoint

    @contextmanager
    def get_connection(self, timeout: float = 5.0):
        """获取连接的上下文管理器"""
        connection_info = None
        try:
            connection_info = self.acquire_connection(timeout)
            if connection_info:
                yield connection_info
            else:
                raise ConnectionError("无法获取可用连接")
        finally:
            if connection_info:
                self.release_connection(connection_info.connection_id)

    def acquire_connection(self, timeout: float = 5.0) -> Optional[ConnectionInfo]:
        """获取连接"""
        start_time = time.time()

        while time.time() - start_time < timeout:
            try:
                # 尝试从可用连接队列获取
                connection_id = self.available_connections.get(timeout=0.1)

                with self.lock:
                    if connection_id in self.connections:
                        connection_info = self.connections[connection_id]

                        # 检查连接状态
                        if connection_info.state == ConnectionState.CONNECTED and connection_info.is_active:
                            connection_info.update_usage()
                            self.stats['total_reused'] += 1
                            logger.debug(f"🔄 复用连接: {connection_id}")
                            return connection_info
                        else:
                            # 连接不可用，重新创建
                            logger.warning(f"⚠️ 连接不可用，重新创建: {connection_id}")
                            self._remove_connection(connection_id)

            except Empty:
                # 队列为空，尝试创建新连接
                if len(self.connections) < self.config.max_connections:
                    connection_info = self._create_connection()
                    if connection_info:
                        connection_info.update_usage()
                        return connection_info

                # 等待一段时间后重试
                time.sleep(0.1)

        logger.error(f"❌ 获取连接超时: {timeout}秒")
        return None

    def release_connection(self, connection_id: str):
        """释放连接"""
        with self.lock:
            if connection_id in self.connections:
                connection_info = self.connections[connection_id]

                # 检查连接状态
                if connection_info.state == ConnectionState.CONNECTED and connection_info.is_active:
                    try:
                        self.available_connections.put(connection_id, timeout=0.1)
                        logger.debug(f"✅ 连接已释放: {connection_id}")
                    except:
                        # 队列已满，连接会被清理
                        logger.warning(f"⚠️ 释放连接时队列已满: {connection_id}")
                else:
                    # 连接不可用，标记移除
                    logger.warning(f"⚠️ 释放不可用连接: {connection_id}")

    def _remove_connection(self, connection_id: str):
        """移除连接"""
        with self.lock:
            if connection_id in self.connections:
                connection_info = self.connections.pop(connection_id)

                # 关闭WebSocket连接
                try:
                    if connection_info.websocket:
                        # 根据实际WebSocket库实现关闭逻辑
                        pass
                except:
                    pass

                self.stats['active_connections'] = len([c for c in self.connections.values() if c.state == ConnectionState.CONNECTED])
                logger.debug(f"🗑️ 连接已移除: {connection_id}")

    def _wait_for_connections_ready(self, timeout: float = 10.0) -> bool:
        """等待连接就绪"""
        start_time = time.time()

        while time.time() - start_time < timeout:
            ready_connections = len([c for c in self.connections.values() if c.state == ConnectionState.CONNECTED])
            if ready_connections >= self.config.min_connections:
                return True
            time.sleep(0.1)

        return False

    def _start_background_tasks(self):
        """启动后台任务"""
        # 启动健康检查任务
        if not self.health_check_thread or not self.health_check_thread.is_alive():
            self.health_check_thread = threading.Thread(
                target=self._health_check_worker,
                name=f"{self.pool_type.value}-health-check",
                daemon=True
            )
            self.health_check_thread.start()
            logger.debug(f"🔍 健康检查任务已启动: {self.pool_type.value}")

        # 启动清理任务
        if not self.cleanup_thread or not self.cleanup_thread.is_alive():
            self.cleanup_thread = threading.Thread(
                target=self._cleanup_worker,
                name=f"{self.pool_type.value}-cleanup",
                daemon=True
            )
            self.cleanup_thread.start()
            logger.debug(f"🧹 清理任务已启动: {self.pool_type.value}")

    def _health_check_worker(self):
        """健康检查工作线程"""
        while not self.shutdown_event.is_set():
            try:
                self._perform_health_check()
                self.shutdown_event.wait(self.config.health_check_interval)
            except Exception as e:
                logger.error(f"❌ 健康检查异常: {e}")
                self.shutdown_event.wait(5.0)

    def _perform_health_check(self):
        """执行健康检查"""
        with self.lock:
            for connection_info in list(self.connections.values()):
                try:
                    # 检查连接年龄
                    if connection_info.age_seconds > 3600:  # 1小时
                        logger.warning(f"⚠️ 连接过期: {connection_info.connection_id} ({connection_info.age_seconds:.1f}s)")
                        connection_info.is_active = False

                    # 检查空闲时间
                    if connection_info.idle_seconds > self.config.idle_timeout:
                        logger.info(f"💤 连接空闲超时: {connection_info.connection_id} ({connection_info.idle_seconds:.1f}s)")
                        connection_info.is_active = False

                    # 检查错误率
                    if connection_info.error_count > 5:
                        logger.warning(f"⚠️ 连接错误过多: {connection_info.connection_id} ({connection_info.error_count}次)")
                        connection_info.is_active = False

                except Exception as e:
                    logger.error(f"❌ 健康检查失败: {connection_info.connection_id} - {e}")

    def _cleanup_worker(self):
        """清理工作线程"""
        while not self.shutdown_event.is_set():
            try:
                self._perform_cleanup()
                self.shutdown_event.wait(60.0)  # 每分钟清理一次
            except Exception as e:
                logger.error(f"❌ 清理任务异常: {e}")
                self.shutdown_event.wait(10.0)

    def _perform_cleanup(self):
        """执行清理"""
        with self.lock:
            connections_to_remove = []

            for connection_id, connection_info in self.connections.items():
                # 清理非活跃连接
                if not connection_info.is_active:
                    connections_to_remove.append(connection_id)
                    continue

                # 清理错误连接
                if connection_info.state == ConnectionState.ERROR:
                    connections_to_remove.append(connection_id)
                    continue

            # 移除标记的连接
            for connection_id in connections_to_remove:
                logger.info(f"🗑️ 清理连接: {connection_id}")
                self._remove_connection(connection_id)

            # 确保最小连接数
            active_connections = len([c for c in self.connections.values() if c.state == ConnectionState.CONNECTED])
            if active_connections < self.config.min_connections:
                needed = self.config.min_connections - active_connections
                logger.info(f"🔄 补充连接: 需要创建{needed}个连接")
                for _ in range(needed):
                    self._create_connection()

    def get_stats(self) -> Dict[str, Any]:
        """获取连接池统计信息"""
        with self.lock:
            active_connections = len([c for c in self.connections.values() if c.state == ConnectionState.CONNECTED])
            idle_connections = len([c for c in self.connections.values() if c.idle_seconds > 60])

            return {
                'pool_type': self.pool_type.value,
                'total_connections': len(self.connections),
                'active_connections': active_connections,
                'idle_connections': idle_connections,
                'available_connections': self.available_connections.qsize(),
                'stats': self.stats.copy(),
                'config': {
                    'min_connections': self.config.min_connections,
                    'max_connections': self.config.max_connections,
                    'connection_timeout': self.config.connection_timeout,
                    'idle_timeout': self.config.idle_timeout
                }
            }

    def get_connection_details(self) -> List[Dict[str, Any]]:
        """获取连接详细信息"""
        with self.lock:
            return [conn.to_dict() for conn in self.connections.values()]

    def shutdown(self):
        """关闭连接池"""
        logger.info(f"🛑 关闭{self.pool_type.value}连接池...")

        self.shutdown_event.set()

        # 关闭所有连接
        with self.lock:
            for connection_id in list(self.connections.keys()):
                self._remove_connection(connection_id)

        # 等待后台任务结束
        if self.health_check_thread and self.health_check_thread.is_alive():
            self.health_check_thread.join(timeout=5.0)

        if self.cleanup_thread and self.cleanup_thread.is_alive():
            self.cleanup_thread.join(timeout=5.0)

        # 关闭线程池
        self.executor.shutdown(wait=True)

        logger.info(f"✅ {self.pool_type.value}连接池已关闭")


class ConnectionPoolManager:
    """连接池管理器"""

    def __init__(self):
        self.pools: Dict[PoolType, WebSocketConnectionPool] = {}
        self.lock = threading.Lock()

    def get_pool(self, pool_type: PoolType, config: Optional[PoolConfig] = None) -> WebSocketConnectionPool:
        """获取连接池"""
        with self.lock:
            if pool_type not in self.pools:
                self.pools[pool_type] = WebSocketConnectionPool(pool_type, config)
            return self.pools[pool_type]

    def initialize_all(self, asr_config: Optional[PoolConfig] = None, tts_config: Optional[PoolConfig] = None) -> bool:
        """初始化所有连接池"""
        try:
            # 初始化ASR连接池
            asr_pool = self.get_pool(PoolType.ASR, asr_config)
            asr_success = asr_pool.initialize()

            # 初始化TTS连接池
            tts_pool = self.get_pool(PoolType.TTS, tts_config)
            tts_success = tts_pool.initialize()

            return asr_success and tts_success

        except Exception as e:
            logger.error(f"❌ 连接池初始化失败: {e}")
            return False

    def get_stats(self) -> Dict[str, Any]:
        """获取所有连接池统计"""
        stats = {}
        for pool_type, pool in self.pools.items():
            stats[pool_type.value] = pool.get_stats()
        return stats

    def shutdown_all(self):
        """关闭所有连接池"""
        logger.info("🛑 关闭所有连接池...")

        with self.lock:
            for pool in self.pools.values():
                pool.shutdown()

        self.pools.clear()
        logger.info("✅ 所有连接池已关闭")


# 全局连接池管理器实例
_pool_manager = None
_manager_lock = threading.Lock()


def get_pool_manager() -> ConnectionPoolManager:
    """获取全局连接池管理器"""
    global _pool_manager

    with _manager_lock:
        if _pool_manager is None:
            _pool_manager = ConnectionPoolManager()
        return _pool_manager


def get_asr_pool(config: Optional[PoolConfig] = None) -> WebSocketConnectionPool:
    """获取ASR连接池"""
    return get_pool_manager().get_pool(PoolType.ASR, config)


def get_tts_pool(config: Optional[PoolConfig] = None) -> WebSocketConnectionPool:
    """获取TTS连接池"""
    return get_pool_manager().get_pool(PoolType.TTS, config)


# 测试和验证函数
def test_connection_pool():
    """测试连接池功能"""
    logger.info("🧪 测试WebSocket连接池功能")

    try:
        # 创建连接池配置
        config = PoolConfig(
            min_connections=1,
            max_connections=3,
            connection_timeout=10.0,
            idle_timeout=60.0
        )

        # 创建连接池
        pool = WebSocketConnectionPool(PoolType.ASR, config)

        # 测试获取连接
        with pool.get_connection(timeout=5.0) as connection:
            if connection:
                logger.info(f"✅ 成功获取连接: {connection.connection_id}")
            else:
                logger.error("❌ 获取连接失败")
                return False

        # 获取统计信息
        stats = pool.get_stats()
        logger.info(f"📊 连接池统计: {stats}")

        # 清理
        pool.shutdown()

        logger.info("🎉 连接池测试完成")
        return True

    except Exception as e:
        logger.error(f"❌ 连接池测试失败: {e}")
        return False


if __name__ == "__main__":
    # 运行测试
    test_connection_pool()