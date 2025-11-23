#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
阿里云NLS Token自动管理器
负责Token的获取、缓存、自动刷新和过期处理
XleRobot Story 1.1 - 语音唤醒和基础识别
"""

import os
import json
import time
import threading
import asyncio
from datetime import datetime, timedelta
import logging
from pathlib import Path
from typing import Optional, Dict, Any, Callable
from dataclasses import dataclass, field

try:
    import yaml
    from aliyunsdkcore.client import AcsClient
    from aliyunsdkcore.request import CommonRequest
except ImportError as e:
    print(f"❌ 缺少必要依赖: {e}")
    print("请运行: pip3 install aliyun-python-sdk-core==2.15.1 PyYAML")
    exit(1)

# 尝试导入官方NLS SDK作为备选方案
try:
    from nls.token import getToken as get_nls_token
    NLS_SDK_AVAILABLE = True
except ImportError:
    NLS_SDK_AVAILABLE = False

@dataclass
class TokenInfo:
    """Token信息"""
    token: str
    expire_time: int
    request_time: int
    refresh_count: int = 0
    last_used: int = field(default_factory=lambda: int(time.time()))

@dataclass
class TokenStats:
    """Token统计信息"""
    total_requests: int = 0
    successful_requests: int = 0
    failed_requests: int = 0
    cache_hits: int = 0
    auto_refreshes: int = 0
    last_refresh_time: int = field(default_factory=lambda: int(time.time()))
    average_response_time: float = 0.0

class AliyunNLSTokenManager:
    """增强的阿里云NLS Token管理器 - 支持WebSocket架构"""

    def __init__(self,
                 config_path=None,
                 enable_websocket_sdk: bool = True,
                 auto_refresh: bool = True,
                 refresh_threshold: int = 300):
        """
        初始化Token管理器

        Args:
            config_path (str): 配置文件路径，如果为None则自动查找
            enable_websocket_sdk (bool): 是否优先使用官方WebSocket SDK
            auto_refresh (bool): 是否启用自动刷新
            refresh_threshold (int): 提前刷新阈值（秒）
        """
        if config_path is None:
            # 智能查找配置文件
            candidates = [
                os.getenv("ALIYUN_CONFIG_PATH"),
                os.path.join(os.getcwd(), "config/aliyun_nls_config.yaml"),
                "/home/sunrise/xlerobot/config/aliyun_nls_config.yaml",
                os.path.join(os.path.dirname(__file__), "../config/aliyun_nls_config.yaml")
            ]
            config_path = None
            for path in candidates:
                if path and os.path.exists(path):
                    config_path = path
                    break

            if config_path is None:
                # 如果所有路径都不存在，使用默认路径并创建目录
                config_path = "/home/sunrise/xlerobot/config/aliyun_nls_config.yaml"
                config_dir = os.path.dirname(config_path)
                os.makedirs(config_dir, exist_ok=True)

        self.config_path = config_path
        self.config = self._load_config()

        # 配置参数
        cache_config = self.config.get('authentication', {}).get('token', {}).get('cache', {})
        self.cache_file = Path(cache_config.get('cache_file', '/tmp/aliyun_nls_token.cache'))
        self.buffer_seconds = cache_config.get('buffer_seconds', refresh_threshold)

        # 功能开关
        self.enable_websocket_sdk = enable_websocket_sdk and NLS_SDK_AVAILABLE
        self.auto_refresh = auto_refresh

        # Token信息（使用新的数据类）
        self._token_info: Optional[TokenInfo] = None

        # 统计信息
        self.stats = TokenStats()

        # 线程锁和线程控制
        self._lock = threading.RLock()  # 使用可重入锁
        self._refresh_thread_started = False
        self._refresh_event = threading.Event()

        # 回调函数
        self._token_refresh_callbacks: List[Callable] = []

        # 设置日志
        self._setup_logging()

        # 初始化时尝试加载缓存
        self._load_cached_token()

        self.logger.info("🔧 增强版阿里云NLS Token管理器初始化完成")
        self.logger.info(f"  - WebSocket SDK: {'启用' if self.enable_websocket_sdk else '禁用'}")
        self.logger.info(f"  - 自动刷新: {'启用' if self.auto_refresh else '禁用'}")
        self.logger.info(f"  - 刷新阈值: {self.buffer_seconds}秒")

    def _setup_logging(self):
        """设置日志"""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger('AliyunNLSTokenManager')

    def _load_config(self):
        """加载配置文件"""
        try:
            with open(self.config_path, 'r', encoding='utf-8') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.logger.error(f"❌ 配置文件加载失败: {e}")
            raise

    def _get_aliyun_client(self):
        """获取阿里云客户端"""
        token_config = self.config.get('authentication', {}).get('token', {})
        access_key_id = token_config.get('access_key_id', '')
        access_key_secret = token_config.get('access_key_secret', '')
        region_id = token_config.get('region_id', 'cn-shanghai')

        if not access_key_id or not access_key_secret:
            raise ValueError("❌ AccessKey ID或Secret未配置")

        return AcsClient(access_key_id, access_key_secret, region_id)

    def _request_new_token(self) -> TokenInfo:
        """请求新的Token（支持WebSocket SDK和HTTP API）"""
        start_time = time.time()

        try:
            # 优先使用WebSocket SDK
            if self.enable_websocket_sdk:
                self.logger.info("🚀 使用官方WebSocket SDK获取Token...")
                token = self._get_token_via_websocket_sdk()
                if token:
                    response_time = time.time() - start_time
                    self._update_response_time_stats(response_time)

                    token_info = TokenInfo(
                        token=token,
                        expire_time=int(time.time()) + 3600,  # WebSocket SDK通常1小时有效期
                        request_time=int(start_time)
                    )
                    self.logger.info(f"✅ WebSocket SDK Token获取成功，耗时: {response_time:.2f}s")
                    return token_info
                else:
                    self.logger.warning("⚠️ WebSocket SDK获取失败，回退到HTTP API")

            # 回退到HTTP API
            self.logger.info("📡 使用HTTP API获取Token...")
            token_info = self._get_token_via_http_api()
            response_time = time.time() - start_time
            self._update_response_time_stats(response_time)
            self.logger.info(f"✅ HTTP API Token获取成功，耗时: {response_time:.2f}s")
            return token_info

        except Exception as e:
            response_time = time.time() - start_time
            self.stats.failed_requests += 1
            self.logger.error(f"❌ Token请求失败: {e} (耗时: {response_time:.2f}s)")
            raise

    def _get_token_via_websocket_sdk(self) -> Optional[str]:
        """通过WebSocket SDK获取Token"""
        try:
            # 获取认证信息
            auth_config = self.config.get('authentication', {}).get('token', {})
            access_key_id = auth_config.get('access_key_id') or os.environ.get('ALIBABA_CLOUD_ACCESS_KEY_ID')
            access_key_secret = auth_config.get('access_key_secret') or os.environ.get('ALIBABA_CLOUD_ACCESS_KEY_SECRET')

            if not access_key_id or not access_key_secret:
                self.logger.warning("⚠️ WebSocket SDK需要AccessKey配置")
                return None

            # 调用官方SDK
            token = get_nls_token(access_key_id, access_key_secret)

            if token:
                self.logger.info(f"✅ WebSocket SDK Token获取成功")
                return token
            else:
                self.logger.warning("⚠️ WebSocket SDK返回空Token")
                return None

        except Exception as e:
            self.logger.warning(f"⚠️ WebSocket SDK获取失败: {e}")
            return None

    def _get_token_via_http_api(self) -> TokenInfo:
        """通过HTTP API获取Token"""
        self.logger.info("📡 正在创建阿里云客户端...")
        client = self._get_aliyun_client()

        self.logger.info("📝 构建Token请求...")
        request = CommonRequest()
        request.set_method('POST')
        request.set_domain('nls-meta.cn-shanghai.aliyuncs.com')
        request.set_version('2019-02-28')
        request.set_action_name('CreateToken')

        self.logger.info("📤 发送Token请求到阿里云...")
        import socket
        # 设置socket超时
        socket.setdefaulttimeout(30)  # 30秒超时

        response = client.do_action_with_exception(request)
        self.logger.info(f"📥 收到响应，长度: {len(response)} 字节")

        token_data = json.loads(response)
        self.logger.info("📋 响应数据解析成功")

        if 'Token' in token_data and 'Id' in token_data['Token']:
            return TokenInfo(
                token=token_data['Token']['Id'],
                expire_time=token_data['Token']['ExpireTime'],
                request_time=int(time.time())
            )
        else:
            raise ValueError(f"❌ Token响应格式错误: {token_data}")

    def _update_response_time_stats(self, response_time: float):
        """更新响应时间统计"""
        self.stats.total_requests += 1
        self.stats.successful_requests += 1

        # 计算平均响应时间
        if self.stats.total_requests == 1:
            self.stats.average_response_time = response_time
        else:
            self.stats.average_response_time = (
                (self.stats.average_response_time * (self.stats.total_requests - 1) + response_time) /
                self.stats.total_requests
            )

    def add_token_refresh_callback(self, callback: Callable[[TokenInfo], None]):
        """
        添加Token刷新回调函数

        Args:
            callback: 回调函数，接收TokenInfo参数
        """
        self._token_refresh_callbacks.append(callback)
        self.logger.info("✅ 已添加Token刷新回调函数")

    def remove_token_refresh_callback(self, callback: Callable):
        """
        移除Token刷新回调函数

        Args:
            callback: 要移除的回调函数
        """
        if callback in self._token_refresh_callbacks:
            self._token_refresh_callbacks.remove(callback)
            self.logger.info("✅ 已移除Token刷新回调函数")

    def _notify_token_refresh_callbacks(self, token_info: TokenInfo):
        """通知所有Token刷新回调函数"""
        for callback in self._token_refresh_callbacks:
            try:
                callback(token_info)
            except Exception as e:
                self.logger.warning(f"⚠️ Token刷新回调函数执行失败: {e}")

    def async_get_token(self) -> asyncio.Future:
        """
        异步获取Token

        Returns:
            asyncio.Future: Token获取的Future对象
        """
        loop = asyncio.get_event_loop()
        return loop.run_in_executor(None, self.get_token)

    def get_token_with_fallback(self, max_retries: int = 3) -> Optional[str]:
        """
        带降级机制的Token获取

        Args:
            max_retries: 最大重试次数

        Returns:
            Token或None
        """
        last_error = None

        for attempt in range(max_retries):
            try:
                token = self.get_token()
                if token:
                    self.logger.info(f"✅ Token获取成功 (尝试 {attempt + 1}/{max_retries})")
                    return token

            except Exception as e:
                last_error = e
                self.logger.warning(f"⚠️ Token获取失败 (尝试 {attempt + 1}/{max_retries}): {e}")

                if attempt < max_retries - 1:
                    # 指数退避重试
                    wait_time = (2 ** attempt) + 1
                    self.logger.info(f"⏳ 等待 {wait_time}s 后重试...")
                    time.sleep(wait_time)

        self.logger.error(f"❌ Token获取最终失败，最大重试次数: {max_retries}")
        if last_error:
            self.logger.error(f"❌ 最后错误: {last_error}")
        return None

    def force_refresh_token(self) -> bool:
        """
        强制刷新Token（忽略缓存）

        Returns:
            是否刷新成功
        """
        self.logger.info("🔄 强制刷新Token...")
        with self._lock:
            try:
                # 清除当前Token
                self._token_info = None

                # 请求新Token
                token_info = self._request_new_token()
                self._token_info = token_info
                self._save_cached_token(token_info)

                # 通知回调函数
                self._notify_token_refresh_callbacks(token_info)

                self.logger.info("✅ 强制刷新Token成功")
                return True

            except Exception as e:
                self.logger.error(f"❌ 强制刷新Token失败: {e}")
                return False

    def _save_cached_token(self, token_info: TokenInfo):
        """保存Token到缓存文件"""
        try:
            self.cache_file.parent.mkdir(parents=True, exist_ok=True)
            token_data = {
                'token': token_info.token,
                'expire_time': token_info.expire_time,
                'request_time': token_info.request_time,
                'refresh_count': token_info.refresh_count,
                'last_used': token_info.last_used
            }
            with open(self.cache_file, 'w', encoding='utf-8') as f:
                json.dump(token_data, f, ensure_ascii=False, indent=2)
            self.logger.debug(f"✅ Token已缓存到: {self.cache_file}")
        except Exception as e:
            self.logger.warning(f"⚠️ Token缓存保存失败: {e}")

    def _load_cached_token(self):
        """从缓存文件加载Token"""
        try:
            if not self.cache_file.exists():
                self.logger.info("📝 缓存文件不存在，将创建新的Token")
                return

            with open(self.cache_file, 'r', encoding='utf-8') as f:
                token_data = json.load(f)

            current_time = int(time.time())
            expire_time = token_data.get('expire_time', 0)

            # 检查Token是否仍然有效（考虑缓冲时间）
            if current_time < (expire_time - self.buffer_seconds):
                self._token_info = TokenInfo(
                    token=token_data.get('token'),
                    expire_time=expire_time,
                    request_time=token_data.get('request_time', current_time),
                    refresh_count=token_data.get('refresh_count', 0),
                    last_used=token_data.get('last_used', current_time)
                )
                self.stats.cache_hits += 1

                remaining_time = expire_time - current_time
                remaining_hours = remaining_time // 3600

                self.logger.info(f"✅ 从缓存加载Token成功，剩余有效时间: {remaining_hours}小时")
                return
            else:
                self.logger.info("⏰ 缓存的Token已过期，将重新获取")

        except Exception as e:
            self.logger.warning(f"⚠️ 缓存Token加载失败: {e}")

    def refresh_token(self, force=False, skip_lock=False):
        """
        刷新Token

        Args:
            force (bool): 是否强制刷新
            skip_lock (bool): 是否跳过锁获取（用于内部调用）

        Returns:
            bool: 刷新是否成功
        """
        if not skip_lock:
            with self._lock:
                return self._refresh_token_internal(force)
        else:
            return self._refresh_token_internal(force)

    def _refresh_token_internal(self, force: bool) -> bool:
        """内部Token刷新方法（已获取锁）"""
        current_time = int(time.time())

        # 检查是否需要刷新
        if not force and self._token_info and current_time < (self._token_info.expire_time - self.buffer_seconds):
            remaining_time = self._token_info.expire_time - current_time
            remaining_hours = remaining_time // 3600
            self.logger.debug(f"🔄 Token仍然有效，剩余时间: {remaining_hours}小时，无需刷新")
            return True

        try:
            self.logger.info("🔄 正在刷新阿里云NLS Token...")
            token_info = self._request_new_token()

            # 更新刷新计数
            if self._token_info:
                token_info.refresh_count = self._token_info.refresh_count + 1

            self._token_info = token_info
            self.stats.last_refresh_time = int(current_time)

            # 保存到缓存
            self._save_cached_token(token_info)

            # 通知回调函数
            self._notify_token_refresh_callbacks(token_info)

            # 计算剩余时间
            remaining_time = token_info.expire_time - current_time
            remaining_hours = remaining_time // 3600

            self.logger.info(f"✅ Token刷新成功！新Token有效期: {remaining_hours}小时")
            return True

        except Exception as e:
            self.logger.error(f"❌ Token刷新失败: {e}")
            return False

    def get_token(self):
        """
        获取有效的Token

        Returns:
            str: 有效的Token，如果获取失败返回None
        """
        self.logger.info("🔍 开始获取Token...")
        self.logger.info(f"🔒 尝试获取锁...")

        with self._lock:
            self.logger.info("🔒 锁获取成功")
            current_time = int(time.time())
            self.logger.info(f"🕐 当前时间: {current_time}")

            # 检查当前Token是否有效
            if self._token_info and current_time < (self._token_info.expire_time - self.buffer_seconds):
                self._token_info.last_used = current_time
                self.logger.info("✅ 使用现有有效Token")
                return self._token_info.token

            self.logger.info("🔄 Token无效或不存在，开始刷新...")
            # 尝试刷新Token（跳过锁，避免死锁）
            if self._refresh_token_internal(False):
                # 第一次成功获取Token后，启动自动刷新线程
                if not self._refresh_thread_started and self.auto_refresh:
                    self.logger.info("🚀 启动自动刷新线程...")
                    self._start_auto_refresh()
                    self._refresh_thread_started = True
                self.logger.info("✅ Token刷新成功")
                return self._token_info.token

            self.logger.error("❌ 无法获取有效的Token")
            return None

    def is_token_valid(self):
        """
        检查当前Token是否有效

        Returns:
            bool: Token是否有效
        """
        if not self._token_info:
            return False

        current_time = int(time.time())
        return current_time < (self._token_info.expire_time - self.buffer_seconds)

    def get_token_info(self):
        """
        获取Token信息

        Returns:
            dict: Token信息
        """
        if not self._token_info:
            return {
                'token': None,
                'expire_time': 0,
                'remaining_seconds': 0,
                'remaining_hours': 0,
                'last_refresh_time': 0,
                'is_valid': False,
                'cache_file': str(self.cache_file),
                'refresh_count': 0,
                'last_used': 0
            }

        current_time = int(time.time())
        remaining_time = max(0, self._token_info.expire_time - current_time)

        return {
            'token': self._token_info.token[:20] + "..." + self._token_info.token[-20:] if self._token_info.token else None,
            'expire_time': self._token_info.expire_time,
            'remaining_seconds': remaining_time,
            'remaining_hours': remaining_time // 3600,
            'last_refresh_time': self._token_info.request_time,
            'is_valid': self.is_token_valid(),
            'cache_file': str(self.cache_file),
            'refresh_count': self._token_info.refresh_count,
            'last_used': self._token_info.last_used
        }

    def get_stats(self) -> TokenStats:
        """获取统计信息"""
        return self.stats

    def reset_stats(self):
        """重置统计信息"""
        self.stats = TokenStats()
        self.logger.info("📊 统计信息已重置")

    def refresh_token(self, force=False, skip_lock=False):
        """
        刷新Token

        Args:
            force (bool): 是否强制刷新
            skip_lock (bool): 是否跳过锁获取（用于内部调用）

        Returns:
            bool: 刷新是否成功
        """
        if not skip_lock:
            with self._lock:
                return self._refresh_token_internal(force)
        else:
            return self._refresh_token_internal(force)

    def _auto_refresh_worker(self):
        """自动刷新工作线程"""
        self.logger.info("🔄 Token自动刷新线程已启动")

        while not self._refresh_event.is_set():
            try:
                # 等待检查间隔或直到停止事件
                if self._refresh_event.wait(60):  # 每分钟检查一次
                    break

                if not self.is_token_valid():
                    self.logger.info("⏰ Token即将过期或已过期，开始自动刷新...")
                    self._refresh_token_internal(False)
                    self.stats.auto_refreshes += 1
                else:
                    # 检查是否需要提前刷新
                    current_time = int(time.time())
                    if self._token_info:
                        time_until_refresh = self._token_info.expire_time - current_time - self.buffer_seconds

                        if time_until_refresh <= 0:
                            self.logger.info("⏰ 到达刷新缓冲时间，开始自动刷新...")
                            self._refresh_token_internal(False)
                            self.stats.auto_refreshes += 1

            except Exception as e:
                self.logger.error(f"❌ 自动刷新异常: {e}")
                time.sleep(300)  # 出错时等待5分钟再重试

    def _start_auto_refresh(self):
        """启动自动刷新线程"""
        refresh_thread = threading.Thread(target=self._auto_refresh_worker, daemon=True)
        refresh_thread.start()

    def cleanup(self):
        """清理资源"""
        try:
            # 停止自动刷新
            self._refresh_event.set()

            if self.cache_file.exists():
                self.cache_file.unlink()
                self.logger.info("🗑️ 缓存文件已清理")
        except Exception as e:
            self.logger.warning(f"⚠️ 缓存清理失败: {e}")

    # 全局增强Token管理器实例
_enhanced_token_manager = None

def get_enhanced_token_manager() -> AliyunNLSTokenManager:
    """获取增强版全局Token管理器实例"""
    global _enhanced_token_manager
    if _enhanced_token_manager is None:
        _enhanced_token_manager = AliyunNLSTokenManager(
            enable_websocket_sdk=True,
            auto_refresh=True,
            refresh_threshold=300
        )
    return _enhanced_token_manager

def get_valid_token() -> Optional[str]:
    """获取有效的Token（便捷函数）"""
    manager = get_enhanced_token_manager()
    return manager.get_token()

# 向后兼容的全局Token管理器实例
_token_manager = None

def get_token_manager():
    """获取全局Token管理器实例（向后兼容）"""
    global _token_manager
    if _token_manager is None:
        _token_manager = AliyunNLSTokenManager()
    return _token_manager

if __name__ == "__main__":
    # 测试增强版Token管理器
    print("🚀 增强版阿里云NLS Token管理器测试")
    print("=" * 50)

    try:
        # 创建增强版管理器
        manager = AliyunNLSTokenManager(
            enable_websocket_sdk=True,
            auto_refresh=True,
            refresh_threshold=300
        )

        # 显示Token信息
        token_info = manager.get_token_info()
        print(f"📋 Token信息:")
        print(f"  Token: {token_info['token']}")
        print(f"  剩余时间: {token_info['remaining_hours']} 小时")
        print(f"  是否有效: {token_info['is_valid']}")
        print(f"  刷新次数: {token_info['refresh_count']}")

        # 显示统计信息
        stats = manager.get_stats()
        print(f"\n📊 统计信息:")
        print(f"  总请求: {stats.total_requests}")
        print(f"  成功请求: {stats.successful_requests}")
        print(f"  缓存命中: {stats.cache_hits}")
        print(f"  平均响应时间: {stats.average_response_time:.2f}s")

        # 测试获取Token
        print(f"\n🔑 获取Token测试:")
        token = manager.get_token()
        if token:
            print(f"✅ Token获取成功: {token[:20]}...{token[-20:]}")
        else:
            print("❌ Token获取失败")

        print(f"\n🎯 增强版Token管理器运行中，按Ctrl+C退出...")

        # 保持运行，测试自动刷新
        try:
            while True:
                time.sleep(30)
                info = manager.get_token_info()
                stats = manager.get_stats()
                print(f"⏰ Token状态 - 剩余: {info['remaining_hours']}h, 有效: {info['is_valid']}, 统计: {stats.total_requests}请求")
        except KeyboardInterrupt:
            print(f"\n👋 增强版Token管理器已停止")

    except Exception as e:
        print(f"❌ 增强版Token管理器初始化失败: {e}")
        import traceback
        traceback.print_exc()

# 全局Token管理器实例
_token_manager = None

def get_token_manager():
    """获取全局Token管理器实例"""
    global _token_manager
    if _token_manager is None:
        _token_manager = AliyunNLSTokenManager()
    return _token_manager

def get_valid_token():
    """获取有效的Token（便捷函数）"""
    manager = get_token_manager()
    return manager.get_token()

if __name__ == "__main__":
    # 测试Token管理器
    print("🚀 阿里云NLS Token管理器测试")
    print("=" * 50)

    try:
        manager = AliyunNLSTokenManager()

        # 显示Token信息
        token_info = manager.get_token_info()
        print(f"📋 Token信息:")
        print(f"  Token: {token_info['token']}")
        print(f"  剩余时间: {token_info['remaining_hours']} 小时")
        print(f"  是否有效: {token_info['is_valid']}")
        print(f"  缓存文件: {token_info['cache_file']}")

        # 测试获取Token
        print(f"\n🔑 获取Token测试:")
        token = manager.get_token()
        if token:
            print(f"✅ Token获取成功: {token[:20]}...{token[-20:]}")
        else:
            print("❌ Token获取失败")

        print(f"\n🎯 Token管理器运行中，按Ctrl+C退出...")

        # 保持运行，测试自动刷新
        try:
            while True:
                time.sleep(30)
                info = manager.get_token_info()
                print(f"⏰ Token状态 - 剩余: {info['remaining_hours']}h, 有效: {info['is_valid']}")
        except KeyboardInterrupt:
            print(f"\n👋 Token管理器已停止")

    except Exception as e:
        print(f"❌ Token管理器初始化失败: {e}")