#!/usr/bin/env python3
"""
XLeRobot 统一Token管理器
解决多重Token管理冲突，使用阿里云官方SDK作为唯一Token源
"""

import logging
import json
import time
import os
import threading
from typing import Optional, Dict, Any
from dataclasses import dataclass, asdict
from pathlib import Path
import hashlib

logger = logging.getLogger(__name__)

@dataclass
class TokenInfo:
    """Token信息"""
    token: str
    expire_time: int
    request_time: int
    access_key_id: str
    app_key: str

class UnifiedTokenManager:
    """
    统一Token管理器

    特性：
    - 唯一使用阿里云官方SDK
    - 自动刷新机制
    - 线程安全
    - 本地缓存
    - 健康检查
    """

    def __init__(self, cache_file: Optional[str] = None):
        """
        初始化Token管理器

        Args:
            cache_file: 缓存文件路径
        """
        self.cache_file = cache_file or "/tmp/xlerobot_token_cache.json"
        self.lock = threading.Lock()
        self.token_info: Optional[TokenInfo] = None
        self.buffer_seconds = 300  # 提前5分钟刷新
        self.auto_refresh_enabled = False  # 禁用自动刷新
        self.refresh_thread = None
        self.stop_refresh = threading.Event()

        # 环境变量
        self.access_key_id = os.environ.get("ALIBABA_CLOUD_ACCESS_KEY_ID", "")
        self.access_key_secret = os.environ.get("ALIBABA_CLOUD_ACCESS_KEY_SECRET", "")
        self.app_key = os.environ.get("ALIYUN_NLS_APPKEY", "")

        # 统计信息
        self.stats = {
            "total_requests": 0,
            "cache_hits": 0,
            "refresh_count": 0,
            "last_refresh_time": 0,
            "errors": 0
        }

        # 尝试加载官方SDK
        self._load_official_sdk()

        # 加载缓存（不自动获取Token）
        self._load_cache()

        logger.info("统一Token管理器初始化完成（按需Token获取）")

    def _load_official_sdk(self) -> None:
        """加载阿里云官方SDK"""
        try:
            # 导入官方SDK
            from nls.token import getToken
            self._official_get_token = getToken
            logger.info("✅ 阿里云官方SDK加载成功")
        except ImportError as e:
            logger.error(f"❌ 阿里云官方SDK加载失败: {e}")
            self._official_get_token = None

        # 设置socket超时，避免长时间阻塞
        import socket
        socket.setdefaulttimeout(10)  # 10秒超时

    def _load_cache(self) -> None:
        """加载Token缓存"""
        try:
            if os.path.exists(self.cache_file):
                with open(self.cache_file, 'r', encoding='utf-8') as f:
                    cache_data = json.load(f)

                # 验证缓存数据
                if self._validate_cache_data(cache_data):
                    self.token_info = TokenInfo(**cache_data)
                    current_time = int(time.time())

                    # 检查Token是否仍然有效
                    if current_time < (self.token_info.expire_time - self.buffer_seconds):
                        logger.info(f"✅ 从缓存加载有效Token: {self.token_info.token[:20]}...")
                    else:
                        logger.warning("⚠️ 缓存Token已过期，将重新获取")
                        self.token_info = None
                else:
                    logger.warning("⚠️ 缓存数据格式错误，将重新获取")
                    self.token_info = None

        except Exception as e:
            logger.error(f"❌ 加载Token缓存失败: {e}")
            self.token_info = None

    def _validate_cache_data(self, data: Dict[str, Any]) -> bool:
        """验证缓存数据格式"""
        required_fields = ["token", "expire_time", "request_time", "access_key_id", "app_key"]
        return all(field in data for field in required_fields)

    def _save_cache(self) -> None:
        """保存Token缓存"""
        try:
            if self.token_info:
                cache_data = asdict(self.token_info)

                # 确保目录存在
                cache_dir = os.path.dirname(self.cache_file)
                os.makedirs(cache_dir, exist_ok=True)

                with open(self.cache_file, 'w', encoding='utf-8') as f:
                    json.dump(cache_data, f, indent=2)

                logger.debug("Token缓存保存成功")

        except Exception as e:
            logger.error(f"❌ 保存Token缓存失败: {e}")

    def _refresh_token(self, force: bool = False) -> bool:
        """
        刷新Token

        Args:
            force: 是否强制刷新

        Returns:
            bool: 刷新成功状态
        """
        with self.lock:
            current_time = int(time.time())

            # 检查是否需要刷新
            if not force and self.token_info:
                time_until_expire = self.token_info.expire_time - current_time
                if time_until_expire > self.buffer_seconds:
                    logger.debug(f"Token仍然有效，剩余时间: {time_until_expire}s")
                    return True

            # 执行刷新
            try:
                logger.info("🔄 开始刷新Token...")
                self.stats["refresh_count"] += 1

                # 检查配置
                if not all([self.access_key_id, self.access_key_secret, self.app_key]):
                    logger.error("❌ 缺少必要的认证配置")
                    self.stats["errors"] += 1
                    return False

                # 使用官方SDK获取Token（带超时）
                if self._official_get_token:
                    import signal

                    def timeout_handler(signum, frame):
                        raise TimeoutError("Token获取超时")

                    # 设置10秒超时
                    signal.signal(signal.SIGALRM, timeout_handler)
                    signal.alarm(10)

                    try:
                        token = self._official_get_token(self.access_key_id, self.access_key_secret)
                        signal.alarm(0)  # 取消超时
                    except TimeoutError:
                        logger.error("❌ Token获取超时")
                        self.stats["errors"] += 1
                        return False
                    except Exception as e:
                        logger.error(f"❌ Token获取异常: {e}")
                        self.stats["errors"] += 1
                        return False
                    finally:
                        signal.alarm(0)  # 确保取消超时
                else:
                    logger.error("❌ 官方SDK不可用")
                    return False

                if not token:
                    logger.error("❌ Token获取失败，返回空值")
                    self.stats["errors"] += 1
                    return False

                # 计算过期时间（Token通常有效期24小时）
                expire_time = current_time + 24 * 3600  # 24小时后过期

                # 更新Token信息
                self.token_info = TokenInfo(
                    token=token,
                    expire_time=expire_time,
                    request_time=current_time,
                    access_key_id=self.access_key_id,
                    app_key=self.app_key
                )

                # 保存缓存
                self._save_cache()

                # 更新统计
                self.stats["last_refresh_time"] = current_time

                logger.info(f"✅ Token刷新成功: {token[:20]}...")
                return True

            except Exception as e:
                logger.error(f"❌ Token刷新异常: {e}")
                self.stats["errors"] += 1
                return False

    def _start_auto_refresh(self) -> None:
        """启动自动刷新线程"""
        if not self.auto_refresh_enabled:
            return

        def refresh_worker():
            """自动刷新工作线程"""
            while not self.stop_refresh.wait(60):  # 每分钟检查一次
                try:
                    current_time = int(time.time())

                    if self.token_info:
                        time_until_expire = self.token_info.expire_time - current_time

                        # 提前5分钟刷新
                        if time_until_expire <= self.buffer_seconds:
                            logger.info("⏰ Token即将过期，开始自动刷新...")
                            self._refresh_token(force=True)

                except Exception as e:
                    logger.error(f"❌ 自动刷新异常: {e}")

        self.refresh_thread = threading.Thread(target=refresh_worker, daemon=True)
        self.refresh_thread.start()
        logger.info("✅ 自动刷新线程已启动")

    def get_token(self, force_refresh: bool = False) -> Optional[str]:
        """
        获取有效Token

        Args:
            force_refresh: 是否强制刷新

        Returns:
            str: Token，失败返回None
        """
        self.stats["total_requests"] += 1

        with self.lock:
            # 检查Token是否存在
            if not self.token_info:
                logger.info("🔄 Token不存在，开始获取...")
                if self._refresh_token(force=True):
                    self.stats["cache_hits"] += 1
                    return self.token_info.token
                else:
                    return None

            # 检查Token是否过期
            current_time = int(time.time())
            time_until_expire = self.token_info.expire_time - current_time

            if time_until_expire <= self.buffer_seconds or force_refresh:
                logger.info(f"🔄 Token需要刷新，剩余时间: {time_until_expire}s")
                if self._refresh_token(force=True):
                    self.stats["cache_hits"] += 1
                    return self.token_info.token
                else:
                    logger.error("❌ Token刷新失败")
                    return None

            # Token有效，直接返回
            self.stats["cache_hits"] += 1
            logger.debug(f"✅ 使用缓存Token，剩余时间: {time_until_expire}s")
            return self.token_info.token

    def get_token_info(self) -> Optional[Dict[str, Any]]:
        """
        获取Token详细信息

        Returns:
            Dict: Token信息
        """
        if not self.token_info:
            return None

        current_time = int(time.time())
        return {
            "token": self.token_info.token[:20] + "..." if self.token_info.token else None,
            "expire_time": self.token_info.expire_time,
            "request_time": self.token_info.request_time,
            "access_key_id": self.token_info.access_key_id,
            "app_key": self.token_info.app_key,
            "time_until_expire": max(0, self.token_info.expire_time - current_time),
            "is_valid": current_time < (self.token_info.expire_time - self.buffer_seconds)
        }

    def health_check(self) -> Dict[str, Any]:
        """
        健康检查

        Returns:
            Dict: 健康状态
        """
        try:
            # 检查配置
            config_ok = all([self.access_key_id, self.access_key_secret, self.app_key])

            # 检查Token
            token_ok = bool(self.get_token())

            # 检查SDK
            sdk_ok = self._official_get_token is not None

            # 计算成功率
            cache_hit_rate = 0
            if self.stats["total_requests"] > 0:
                cache_hit_rate = (self.stats["cache_hits"] / self.stats["total_requests"]) * 100

            status = "healthy" if all([config_ok, token_ok, sdk_ok]) else "unhealthy"

            return {
                "status": status,
                "config_ok": config_ok,
                "token_ok": token_ok,
                "sdk_ok": sdk_ok,
                "auto_refresh_enabled": self.auto_refresh_enabled,
                "statistics": {
                    "total_requests": self.stats["total_requests"],
                    "cache_hit_rate": f"{cache_hit_rate:.1f}%",
                    "refresh_count": self.stats["refresh_count"],
                    "errors": self.stats["errors"],
                    "last_refresh_time": self.stats["last_refresh_time"]
                },
                "token_info": self.get_token_info()
            }

        except Exception as e:
            return {
                "status": "error",
                "error": str(e)
            }

    def invalidate_cache(self) -> None:
        """使缓存失效"""
        with self.lock:
            self.token_info = None
            try:
                if os.path.exists(self.cache_file):
                    os.remove(self.cache_file)
                logger.info("✅ Token缓存已清除")
            except Exception as e:
                logger.error(f"❌ 清除缓存失败: {e}")

    def shutdown(self) -> None:
        """关闭Token管理器"""
        logger.info("正在关闭Token管理器...")

        # 停止自动刷新
        self.stop_refresh.set()
        if self.refresh_thread and self.refresh_thread.is_alive():
            self.refresh_thread.join(timeout=2.0)

        # 保存最终状态
        if self.token_info:
            self._save_cache()

        logger.info("Token管理器已关闭")

    def __del__(self):
        """析构函数"""
        try:
            self.shutdown()
        except:
            pass

# 全局Token管理器实例
_token_manager = None

def get_unified_token_manager() -> UnifiedTokenManager:
    """获取全局统一Token管理器实例"""
    global _token_manager
    if _token_manager is None:
        _token_manager = UnifiedTokenManager()
    return _token_manager

def get_valid_token(force_refresh: bool = False) -> Optional[str]:
    """
    获取有效Token（便捷函数）

    Args:
        force_refresh: 是否强制刷新

    Returns:
        str: Token，失败返回None
    """
    manager = get_unified_token_manager()
    return manager.get_token(force_refresh=force_refresh)

# 兼容性函数 - 替代旧的token管理器
def get_token_manager() -> UnifiedTokenManager:
    """获取Token管理器（兼容性函数）"""
    return get_unified_token_manager()

if __name__ == "__main__":
    # 测试代码
    import json

    logging.basicConfig(level=logging.INFO)

    print("=== 统一Token管理器测试 ===")

    # 创建管理器
    manager = get_unified_token_manager()

    # 健康检查
    print("\n1. 健康检查...")
    health = manager.health_check()
    print(json.dumps(health, indent=2, ensure_ascii=False))

    # 获取Token
    print("\n2. 获取Token...")
    token = manager.get_token()
    if token:
        print(f"Token获取成功: {token[:20]}...")
    else:
        print("Token获取失败")

    # Token信息
    print("\n3. Token详细信息...")
    token_info = manager.get_token_info()
    if token_info:
        print(json.dumps(token_info, indent=2, ensure_ascii=False))

    # 统计信息
    print("\n4. 统计信息...")
    print(json.dumps(health.get("statistics", {}), indent=2, ensure_ascii=False))

    # 测试多次获取
    print("\n5. 测试多次获取...")
    for i in range(3):
        token = get_valid_token()
        print(f"第{i+1}次获取: {'成功' if token else '失败'}")
        time.sleep(1)

    print("\n测试完成")