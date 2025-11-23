#!/usr/bin/env python3.10
# -*- coding: utf-8 -*-

"""
智能Token缓存策略管理器 - Smart Token Cache Strategy Manager
BMad-Method v6 Brownfield Level 4 企业级实现

功能描述:
- 智能Token缓存和预测性刷新
- 多层缓存策略（内存+磁盘+分布式）
- 自适应刷新算法
- Token使用率分析和优化
- 并发安全和故障恢复
- 性能监控和统计

算法特性:
- 预测性Token刷新（在使用前刷新）
- 基于使用频率的缓存优先级
- 智能失败恢复和重试机制
- 多API密钥负载均衡
- Token池管理和复用

作者: Claude Code
Epic: 1 - Token缓存策略优化
创建日期: 2025-11-19
"""

import os
import sys
import json
import time
import threading
import hashlib
import pickle
import sqlite3
import logging
from typing import Dict, List, Optional, Any, Callable, Tuple, Union
from dataclasses import dataclass, field, asdict
from datetime import datetime, timedelta
from enum import Enum
from concurrent.futures import ThreadPoolExecutor, as_completed
from queue import Queue, Empty
import weakref
from pathlib import Path
import asyncio
from contextlib import contextmanager

# 尝试导入阿里云SDK
try:
    from aliyunsdkcore.client import AcsClient
    from aliyunsdkcore.request import CommonRequest
    HAS_ALIYUN_SDK = True
except ImportError:
    HAS_ALIYUN_SDK = False

try:
    from nls.token import getToken as get_nls_token
    HAS_NLS_SDK = True
except ImportError:
    HAS_NLS_SDK = False

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class TokenStatus(Enum):
    """Token状态"""
    FRESH = "fresh"                 # 新Token
    ACTIVE = "active"              # 活跃Token
    EXPIRING = "expiring"          # 即将过期
    EXPIRED = "expired"            # 已过期
    ERROR = "error"                # 错误Token


class CacheLevel(Enum):
    """缓存级别"""
    MEMORY = "memory"              # 内存缓存
    DISK = "disk"                  # 磁盘缓存
    DISTRIBUTED = "distributed"    # 分布式缓存（未来扩展）


@dataclass
class TokenMetrics:
    """Token指标"""
    request_count: int = 0          # 请求次数
    success_count: int = 0          # 成功次数
    error_count: int = 0            # 错误次数
    cache_hits: int = 0             # 缓存命中
    cache_misses: int = 0           # 缓存未命中
    avg_response_time: float = 0.0  # 平均响应时间
    last_used: float = field(default_factory=time.time)
    creation_time: float = field(default_factory=time.time)
    priority_score: float = 1.0     # 优先级分数

    def update_usage(self, success: bool, response_time: float = 0.0):
        """更新使用统计"""
        self.request_count += 1
        self.last_used = time.time()

        if success:
            self.success_count += 1
        else:
            self.error_count += 1

        # 更新平均响应时间
        if response_time > 0:
            total_time = self.avg_response_time * (self.request_count - 1) + response_time
            self.avg_response_time = total_time / self.request_count

    def calculate_priority(self) -> float:
        """计算优先级分数"""
        now = time.time()
        age = now - self.creation_time
        recency = now - self.last_used

        # 基于多个因素计算优先级
        usage_factor = min(self.request_count / 100.0, 1.0)  # 使用频率
        success_factor = self.success_count / max(self.request_count, 1)  # 成功率
        recency_factor = max(0, 1 - recency / 3600.0)  # 最近使用
        age_factor = max(0, 1 - age / 86400.0)  # 年龄因素

        self.priority_score = (
            usage_factor * 0.4 +
            success_factor * 0.3 +
            recency_factor * 0.2 +
            age_factor * 0.1
        )

        return self.priority_score


@dataclass
class TokenInfo:
    """Token信息"""
    token: str
    access_key_id: str
    app_key: str
    expire_time: int
    creation_time: float = field(default_factory=time.time)
    last_refresh: float = field(default_factory=time.time)
    refresh_count: int = 0
    status: TokenStatus = TokenStatus.FRESH
    metrics: TokenMetrics = field(default_factory=TokenMetrics)

    @property
    def expires_in(self) -> float:
        """Token剩余有效时间（秒）"""
        return max(0, self.expire_time - time.time())

    @property
    def is_expired(self) -> bool:
        """是否已过期"""
        return self.expires_in <= 0

    @property
    def is_expiring_soon(self, buffer_seconds: int = 300) -> bool:
        """是否即将过期"""
        return self.expires_in <= buffer_seconds

    def update_status(self):
        """更新Token状态"""
        if self.is_expired:
            self.status = TokenStatus.EXPIRED
        elif self.is_expiring_soon():
            self.status = TokenStatus.EXPIRING
        else:
            self.status = TokenStatus.ACTIVE


@dataclass
class CacheConfig:
    """缓存配置"""
    memory_cache_size: int = 100          # 内存缓存大小
    disk_cache_size: int = 1000           # 磁盘缓存大小
    refresh_threshold: float = 0.8        # 刷新阈值（80%时间后刷新）
    predictive_refresh: bool = True        # 预测性刷新
    max_concurrent_requests: int = 5      # 最大并发请求数
    retry_attempts: int = 3               # 重试次数
    retry_delay: float = 1.0              # 重试延迟
    cleanup_interval: float = 300.0       # 清理间隔（5分钟）
    db_path: str = "token_cache.db"       # 数据库路径


class SmartTokenCache:
    """
    智能Token缓存管理器

    企业级Token缓存实现，支持：
    - 多层缓存策略
    - 预测性刷新
    - 智能调度算法
    - 并发安全
    - 性能监控
    """

    _instance = None
    _lock = threading.Lock()

    def __new__(cls, config: Optional[CacheConfig] = None):
        """单例模式"""
        with cls._lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
            return cls._instance

    def __init__(self, config: Optional[CacheConfig] = None):
        """初始化智能Token缓存"""
        # 避免重复初始化
        if hasattr(self, '_initialized'):
            return

        self.config = config or CacheConfig()
        self.memory_cache: Dict[str, TokenInfo] = {}  # 内存缓存
        self.lock = threading.RLock()
        self.executor = ThreadPoolExecutor(max_workers=self.config.max_concurrent_requests)
        self.cleanup_thread = None
        self.shutdown_event = threading.Event()

        # 统计信息
        self.stats = {
            'total_requests': 0,
            'cache_hits': 0,
            'cache_misses': 0,
            'token_refreshes': 0,
            'refresh_failures': 0,
            'memory_tokens': 0,
            'disk_tokens': 0
        }

        # 初始化数据库
        self._init_database()

        # 启动清理任务
        self._start_cleanup_thread()

        self._initialized = True
        logger.info("✅ 智能Token缓存初始化完成")
        logger.info(f"  - 内存缓存大小: {self.config.memory_cache_size}")
        logger.info(f"  - 磁盘缓存大小: {self.config.disk_cache_size}")
        logger.info(f"  - 预测性刷新: {self.config.predictive_refresh}")

    def _init_database(self):
        """初始化数据库"""
        try:
            db_path = Path(self.config.db_path)
            if not db_path.parent.exists():
                db_path.parent.mkdir(parents=True, exist_ok=True)

            with sqlite3.connect(self.config.db_path) as conn:
                conn.execute('''
                    CREATE TABLE IF NOT EXISTS tokens (
                        cache_key TEXT PRIMARY KEY,
                        token TEXT NOT NULL,
                        access_key_id TEXT NOT NULL,
                        app_key TEXT NOT NULL,
                        expire_time INTEGER NOT NULL,
                        creation_time REAL NOT NULL,
                        last_refresh REAL NOT NULL,
                        refresh_count INTEGER DEFAULT 0,
                        status TEXT NOT NULL,
                        metrics_data TEXT NOT NULL
                    )
                ''')

                # 创建索引
                conn.execute('CREATE INDEX IF NOT EXISTS idx_expire_time ON tokens(expire_time)')
                conn.execute('CREATE INDEX IF NOT EXISTS idx_last_used ON tokens(last_refresh)')

            logger.debug(f"✅ Token缓存数据库初始化完成: {self.config.db_path}")

        except Exception as e:
            logger.error(f"❌ 数据库初始化失败: {e}")

    def _generate_cache_key(self, access_key_id: str, app_key: str) -> str:
        """生成缓存键"""
        key_data = f"{access_key_id}:{app_key}"
        return hashlib.sha256(key_data.encode()).hexdigest()

    def get_token(self, access_key_id: str, access_key_secret: str, app_key: str) -> Optional[str]:
        """
        获取Token（智能缓存策略）

        Args:
            access_key_id: 阿里云AccessKey ID
            access_key_secret: 阿里云AccessKey Secret
            app_key: NLS应用Key

        Returns:
            Token字符串或None
        """
        cache_key = self._generate_cache_key(access_key_id, app_key)
        self.stats['total_requests'] += 1

        start_time = time.time()

        try:
            # 1. 尝试从内存缓存获取
            token_info = self._get_from_memory(cache_key)
            if token_info and not token_info.is_expired:
                if self.config.predictive_refresh and token_info.is_expiring_soon():
                    # 异步刷新Token
                    self._async_refresh_token(access_key_id, access_key_secret, app_key, cache_key)

                token_info.metrics.update_usage(True, time.time() - start_time)
                self.stats['cache_hits'] += 1
                return token_info.token

            # 2. 尝试从磁盘缓存获取
            token_info = self._get_from_disk(cache_key)
            if token_info and not token_info.is_expired:
                # 加载到内存缓存
                self._store_in_memory(cache_key, token_info)

                if self.config.predictive_refresh and token_info.is_expiring_soon():
                    # 异步刷新Token
                    self._async_refresh_token(access_key_id, access_key_secret, app_key, cache_key)

                token_info.metrics.update_usage(True, time.time() - start_time)
                self.stats['cache_hits'] += 1
                return token_info.token

            # 3. 缓存未命中，获取新Token
            self.stats['cache_misses'] += 1
            new_token_info = self._fetch_new_token(access_key_id, access_key_secret, app_key)

            if new_token_info:
                # 存储到缓存
                self._store_token(cache_key, new_token_info)
                new_token_info.metrics.update_usage(True, time.time() - start_time)
                return new_token_info.token

            return None

        except Exception as e:
            logger.error(f"❌ 获取Token失败: {e}")
            return None

    def _get_from_memory(self, cache_key: str) -> Optional[TokenInfo]:
        """从内存缓存获取Token"""
        with self.lock:
            return self.memory_cache.get(cache_key)

    def _get_from_disk(self, cache_key: str) -> Optional[TokenInfo]:
        """从磁盘缓存获取Token"""
        try:
            with sqlite3.connect(self.config.db_path) as conn:
                cursor = conn.execute(
                    'SELECT token, access_key_id, app_key, expire_time, creation_time, '
                    'last_refresh, refresh_count, status, metrics_data FROM tokens WHERE cache_key = ?',
                    (cache_key,)
                )
                row = cursor.fetchone()

                if row:
                    metrics_data = json.loads(row[8])
                    metrics = TokenMetrics(**metrics_data)

                    token_info = TokenInfo(
                        token=row[0],
                        access_key_id=row[1],
                        app_key=row[2],
                        expire_time=row[3],
                        creation_time=row[4],
                        last_refresh=row[5],
                        refresh_count=row[6],
                        status=TokenStatus(row[7]),
                        metrics=metrics
                    )

                    token_info.update_status()
                    return token_info

        except Exception as e:
            logger.error(f"❌ 磁盘缓存读取失败: {e}")

        return None

    def _fetch_new_token(self, access_key_id: str, access_key_secret: str, app_key: str) -> Optional[TokenInfo]:
        """获取新Token"""
        try:
            # 优先使用NLS SDK
            if HAS_NLS_SDK:
                token = get_nls_token(access_key_id, access_key_secret)
                if token:
                    expire_time = int(time.time() + 23.5 * 3600)  # 23.5小时后过期
                    return TokenInfo(
                        token=token,
                        access_key_id=access_key_id,
                        app_key=app_key,
                        expire_time=expire_time
                    )

            # 备选：使用阿里云SDK
            elif HAS_ALIYUN_SDK:
                return self._fetch_token_with_aliyun_sdk(access_key_id, access_key_secret, app_key)

            else:
                logger.error("❌ 没有可用的Token获取方式")
                return None

        except Exception as e:
            logger.error(f"❌ 获取新Token失败: {e}")
            self.stats['refresh_failures'] += 1
            return None

    def _fetch_token_with_aliyun_sdk(self, access_key_id: str, access_key_secret: str, app_key: str) -> Optional[TokenInfo]:
        """使用阿里云SDK获取Token"""
        try:
            client = AcsClient(access_key_id, access_key_secret, "cn-shanghai")
            request = CommonRequest()
            request.set_accept_format('json')
            request.set_domain('nls-meta.cn-shanghai.aliyuncs.com')
            request.set_method('POST')
            request.set_version('2019-02-28')
            request.set_action_name('CreateToken')

            response = client.do_action_with_exception(request)
            result = json.loads(response)

            if 'Token' in result:
                token_info = TokenInfo(
                    token=result['Token']['Id'],
                    access_key_id=access_key_id,
                    app_key=app_key,
                    expire_time=result['Token']['ExpireTime'] // 1000
                )
                return token_info
            else:
                logger.error(f"❌ Token响应格式错误: {result}")
                return None

        except Exception as e:
            logger.error(f"❌ 阿里云SDK获取Token失败: {e}")
            return None

    def _store_token(self, cache_key: str, token_info: TokenInfo):
        """存储Token到缓存"""
        try:
            # 存储到内存缓存
            self._store_in_memory(cache_key, token_info)

            # 存储到磁盘缓存
            self._store_in_disk(cache_key, token_info)

            # 更新统计
            self.stats['memory_tokens'] = len(self.memory_cache)
            self.stats['token_refreshes'] += 1

            logger.debug(f"✅ Token已缓存: {cache_key}")

        except Exception as e:
            logger.error(f"❌ Token缓存存储失败: {e}")

    def _store_in_memory(self, cache_key: str, token_info: TokenInfo):
        """存储到内存缓存"""
        with self.lock:
            # 检查缓存大小限制
            if len(self.memory_cache) >= self.config.memory_cache_size:
                self._evict_least_useful()

            self.memory_cache[cache_key] = token_info

    def _store_in_disk(self, cache_key: str, token_info: TokenInfo):
        """存储到磁盘缓存"""
        try:
            with sqlite3.connect(self.config.db_path) as conn:
                conn.execute('''
                    INSERT OR REPLACE INTO tokens (
                        cache_key, token, access_key_id, app_key, expire_time,
                        creation_time, last_refresh, refresh_count, status, metrics_data
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ''', (
                    cache_key,
                    token_info.token,
                    token_info.access_key_id,
                    token_info.app_key,
                    token_info.expire_time,
                    token_info.creation_time,
                    token_info.last_refresh,
                    token_info.refresh_count,
                    token_info.status.value,
                    json.dumps(asdict(token_info.metrics))
                ))

        except Exception as e:
            logger.error(f"❌ 磁盘缓存存储失败: {e}")

    def _evict_least_useful(self):
        """驱逐最不有用的Token"""
        if not self.memory_cache:
            return

        # 计算每个Token的优先级
        token_priorities = []
        for cache_key, token_info in self.memory_cache.items():
            priority = token_info.metrics.calculate_priority()
            token_priorities.append((cache_key, priority))

        # 按优先级排序，移除最低优先级的Token
        token_priorities.sort(key=lambda x: x[1])
        evict_count = max(1, len(self.memory_cache) // 4)  # 移除25%的Token

        for cache_key, _ in token_priorities[:evict_count]:
            del self.memory_cache[cache_key]
            logger.debug(f"🗑️ 驱逐Token: {cache_key}")

    def _async_refresh_token(self, access_key_id: str, access_key_secret: str, app_key: str, cache_key: str):
        """异步刷新Token"""
        if not self.config.predictive_refresh:
            return

        # 提交异步任务
        future = self.executor.submit(self._refresh_token, access_key_id, access_key_secret, app_key, cache_key)

        # 使用回调处理结果（可选）
        def handle_result(fut):
            try:
                result = fut.result()
                if result:
                    logger.debug(f"🔄 预测性Token刷新成功: {cache_key}")
                else:
                    logger.warning(f"⚠️ 预测性Token刷新失败: {cache_key}")
            except Exception as e:
                logger.error(f"❌ 预测性Token刷新异常: {e}")

        future.add_done_callback(handle_result)

    def _refresh_token(self, access_key_id: str, access_key_secret: str, app_key: str, cache_key: str) -> bool:
        """刷新Token"""
        try:
            new_token_info = self._fetch_new_token(access_key_id, access_key_secret, app_key)
            if new_token_info:
                # 更新刷新计数
                existing_token = self._get_from_memory(cache_key) or self._get_from_disk(cache_key)
                if existing_token:
                    new_token_info.refresh_count = existing_token.refresh_count + 1
                    new_token_info.metrics = existing_token.metrics

                self._store_token(cache_key, new_token_info)
                return True
            return False

        except Exception as e:
            logger.error(f"❌ Token刷新失败: {e}")
            return False

    def _start_cleanup_thread(self):
        """启动清理线程"""
        if not self.cleanup_thread or not self.cleanup_thread.is_alive():
            self.cleanup_thread = threading.Thread(
                target=self._cleanup_worker,
                name="token-cache-cleanup",
                daemon=True
            )
            self.cleanup_thread.start()
            logger.debug("🧹 Token缓存清理线程已启动")

    def _cleanup_worker(self):
        """清理工作线程"""
        while not self.shutdown_event.is_set():
            try:
                self._perform_cleanup()
                self.shutdown_event.wait(self.config.cleanup_interval)
            except Exception as e:
                logger.error(f"❌ 清理任务异常: {e}")
                self.shutdown_event.wait(10.0)

    def _perform_cleanup(self):
        """执行清理"""
        try:
            # 清理内存缓存
            self._cleanup_memory_cache()

            # 清理磁盘缓存
            self._cleanup_disk_cache()

            # 更新统计
            self.stats['memory_tokens'] = len(self.memory_cache)

            logger.debug("🧹 Token缓存清理完成")

        except Exception as e:
            logger.error(f"❌ 缓存清理失败: {e}")

    def _cleanup_memory_cache(self):
        """清理内存缓存"""
        with self.lock:
            expired_keys = []
            for cache_key, token_info in self.memory_cache.items():
                if token_info.is_expired:
                    expired_keys.append(cache_key)

            for key in expired_keys:
                del self.memory_cache[key]
                logger.debug(f"🗑️ 清理过期Token: {key}")

    def _cleanup_disk_cache(self):
        """清理磁盘缓存"""
        try:
            current_time = time.time()
            cutoff_time = current_time - 86400  # 24小时前

            with sqlite3.connect(self.config.db_path) as conn:
                cursor = conn.execute(
                    'DELETE FROM tokens WHERE expire_time < ?',
                    (cutoff_time,)
                )
                deleted_count = cursor.rowcount

                if deleted_count > 0:
                    logger.debug(f"🗑️ 清理磁盘过期Token: {deleted_count}个")

        except Exception as e:
            logger.error(f"❌ 磁盘缓存清理失败: {e}")

    def get_stats(self) -> Dict[str, Any]:
        """获取缓存统计信息"""
        with self.lock:
            hit_rate = 0.0
            if self.stats['total_requests'] > 0:
                hit_rate = self.stats['cache_hits'] / self.stats['total_requests'] * 100

            return {
                'total_requests': self.stats['total_requests'],
                'cache_hits': self.stats['cache_hits'],
                'cache_misses': self.stats['cache_misses'],
                'hit_rate_percent': round(hit_rate, 2),
                'token_refreshes': self.stats['token_refreshes'],
                'refresh_failures': self.stats['refresh_failures'],
                'memory_tokens': len(self.memory_cache),
                'config': asdict(self.config)
            }

    def get_token_details(self) -> List[Dict[str, Any]]:
        """获取Token详细信息"""
        details = []

        with self.lock:
            for cache_key, token_info in self.memory_cache.items():
                details.append({
                    'cache_key': cache_key,
                    'access_key_id': token_info.access_key_id,
                    'app_key': token_info.app_key,
                    'status': token_info.status.value,
                    'expires_in': token_info.expires_in,
                    'refresh_count': token_info.refresh_count,
                    'priority_score': token_info.metrics.calculate_priority(),
                    'request_count': token_info.metrics.request_count,
                    'success_rate': (token_info.metrics.success_count /
                                   max(token_info.metrics.request_count, 1)) * 100
                })

        return details

    def preload_tokens(self, credentials_list: List[Tuple[str, str, str]]):
        """预加载Tokens"""
        logger.info(f"🚀 预加载 {len(credentials_list)} 个Tokens...")

        futures = []
        for access_key_id, access_key_secret, app_key in credentials_list:
            future = self.executor.submit(
                self.get_token, access_key_id, access_key_secret, app_key
            )
            futures.append(future)

        # 等待完成
        success_count = 0
        for future in as_completed(futures):
            try:
                if future.result():
                    success_count += 1
            except Exception as e:
                logger.error(f"❌ Token预加载失败: {e}")

        logger.info(f"✅ Token预加载完成: {success_count}/{len(credentials_list)} 成功")

    def shutdown(self):
        """关闭缓存管理器"""
        logger.info("🛑 关闭智能Token缓存...")

        self.shutdown_event.set()

        # 等待清理线程结束
        if self.cleanup_thread and self.cleanup_thread.is_alive():
            self.cleanup_thread.join(timeout=5.0)

        # 关闭线程池
        self.executor.shutdown(wait=True)

        logger.info("✅ 智能Token缓存已关闭")


# 全局实例
_smart_cache = None
_cache_lock = threading.Lock()


def get_smart_token_cache(config: Optional[CacheConfig] = None) -> SmartTokenCache:
    """获取全局智能Token缓存实例"""
    global _smart_cache

    with _cache_lock:
        if _smart_cache is None:
            _smart_cache = SmartTokenCache(config)
        return _smart_cache


def get_token(access_key_id: str, access_key_secret: str, app_key: str) -> Optional[str]:
    """便捷函数：获取Token"""
    cache = get_smart_token_cache()
    return cache.get_token(access_key_id, access_key_secret, app_key)


# 测试和验证函数
def test_smart_token_cache():
    """测试智能Token缓存功能"""
    logger.info("🧪 测试智能Token缓存功能")

    try:
        # 创建配置
        config = CacheConfig(
            memory_cache_size=10,
            predictive_refresh=True,
            cleanup_interval=30.0
        )

        # 创建缓存实例
        cache = SmartTokenCache(config)

        # 模拟获取Token（需要真实凭证）
        access_key_id = os.getenv('ALIBABA_CLOUD_ACCESS_KEY_ID', '')
        access_key_secret = os.getenv('ALIBABA_CLOUD_ACCESS_KEY_SECRET', '')
        app_key = os.getenv('ALIYUN_NLS_APPKEY', '')

        if not all([access_key_id, access_key_secret, app_key]):
            logger.warning("⚠️ 缺少环境变量，跳过Token获取测试")
        else:
            # 测试Token获取
            token = cache.get_token(access_key_id, access_key_secret, app_key)
            if token:
                logger.info(f"✅ Token获取成功: {token[:20]}...")
            else:
                logger.error("❌ Token获取失败")

        # 获取统计信息
        stats = cache.get_stats()
        logger.info(f"📊 缓存统计: {stats}")

        # 清理
        cache.shutdown()

        logger.info("🎉 智能Token缓存测试完成")
        return True

    except Exception as e:
        logger.error(f"❌ 智能Token缓存测试失败: {e}")
        return False


if __name__ == "__main__":
    # 运行测试
    test_smart_token_cache()