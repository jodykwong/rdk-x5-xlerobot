"""
SimpleSessionManager - 简单会话管理器
Story 1.7: 多模态在线对话API集成
严格遵循Epic 1纯在线架构 - 仅管理API会话ID，无本地对话逻辑
"""

import time
import uuid
from typing import Dict, Optional, Set
import logging

logger = logging.getLogger(__name__)

class SimpleSessionManager:
    """
    简单会话管理器
    严格遵循Epic 1纯在线架构 - 仅管理API会话ID
    严禁：本地对话逻辑、状态机、上下文管理
    """

    def __init__(self, session_timeout: float = 300.0):
        """
        初始化简单会话管理器

        Args:
            session_timeout: 会话超时时间（秒），默认5分钟
        """
        logger.info("🔄 初始化SimpleSessionManager - 纯在线架构")

        self.session_timeout = session_timeout
        self.active_sessions: Dict[str, float] = {}  # session_id -> 最后活动时间
        self.session_stats = {
            "total_sessions_created": 0,
            "active_sessions_count": 0,
            "expired_sessions_count": 0
        }

        logger.info(f"✅ 会话管理器初始化完成 - 超时时间: {session_timeout}秒")

    def create_session(self) -> str:
        """
        创建新的API会话ID
        仅生成ID，不进行任何本地会话逻辑

        Returns:
            str: 会话ID
        """
        session_id = str(uuid.uuid4())
        self.active_sessions[session_id] = time.time()
        self.session_stats["total_sessions_created"] += 1

        logger.info(f"🆕 创建会话: {session_id}")
        return session_id

    def is_session_active(self, session_id: str) -> bool:
        """
        检查会话是否活跃
        仅检查会话ID是否存在且未超时

        Args:
            session_id: 会话ID

        Returns:
            bool: 是否活跃
        """
        if session_id not in self.active_sessions:
            return False

        # 检查是否超时
        last_activity = self.active_sessions[session_id]
        if time.time() - last_activity > self.session_timeout:
            # 自动清理过期会话
            del self.active_sessions[session_id]
            self.session_stats["expired_sessions_count"] += 1
            logger.info(f"🧹 会话已自动过期: {session_id}")
            return False

        return True

    def update_session_activity(self, session_id: str) -> bool:
        """
        更新会话活动时间
        仅更新时间戳，不进行任何状态管理

        Args:
            session_id: 会话ID

        Returns:
            bool: 是否更新成功
        """
        if session_id in self.active_sessions:
            self.active_sessions[session_id] = time.time()
            return True
        else:
            # 如果会话不存在，自动创建
            logger.warning(f"⚠️ 会话不存在，自动创建: {session_id}")
            self.active_sessions[session_id] = time.time()
            return False

    def get_session_age(self, session_id: str) -> Optional[float]:
        """
        获取会话年龄（秒）

        Args:
            session_id: 会话ID

        Returns:
            Optional[float]: 会话年龄，如果会话不存在返回None
        """
        if session_id not in self.active_sessions:
            return None

        return time.time() - self.active_sessions[session_id]

    def get_active_sessions_count(self) -> int:
        """
        获取活跃会话数量
        会自动清理过期会话

        Returns:
            int: 活跃会话数量
        """
        self._cleanup_expired_sessions()
        self.session_stats["active_sessions_count"] = len(self.active_sessions)
        return len(self.active_sessions)

    def get_all_active_sessions(self) -> Set[str]:
        """
        获取所有活跃会话ID
        会自动清理过期会话

        Returns:
            Set[str]: 活跃会话ID集合
        """
        self._cleanup_expired_sessions()
        return set(self.active_sessions.keys())

    def expire_session(self, session_id: str) -> bool:
        """
        手动过期会话
        仅从活跃列表中移除，不进行任何本地状态清理

        Args:
            session_id: 会话ID

        Returns:
            bool: 是否成功过期
        """
        if session_id in self.active_sessions:
            del self.active_sessions[session_id]
            logger.info(f"⏰ 手动过期会话: {session_id}")
            return True
        return False

    def _cleanup_expired_sessions(self):
        """
        清理所有过期会话
        自动调用，定期清理超时会话
        """
        current_time = time.time()
        expired_sessions = []

        for session_id, last_activity in self.active_sessions.items():
            if current_time - last_activity > self.session_timeout:
                expired_sessions.append(session_id)

        # 移除过期会话
        for session_id in expired_sessions:
            del self.active_sessions[session_id]

        if expired_sessions:
            self.session_stats["expired_sessions_count"] += len(expired_sessions)
            logger.info(f"🧹 清理了{len(expired_sessions)}个过期会话")

    def get_session_statistics(self) -> Dict[str, any]:
        """
        获取会话管理统计信息

        Returns:
            Dict[str, any]: 统计信息
        """
        self._cleanup_expired_sessions()

        stats = self.session_stats.copy()
        stats["active_sessions_count"] = len(self.active_sessions)

        # 计算会话详情
        if self.active_sessions:
            current_time = time.time()
            session_ages = [
                current_time - last_activity
                for last_activity in self.active_sessions.values()
            ]
            stats["average_session_age"] = sum(session_ages) / len(session_ages)
            stats["oldest_session_age"] = max(session_ages)
            stats["newest_session_age"] = min(session_ages)
        else:
            stats["average_session_age"] = 0.0
            stats["oldest_session_age"] = 0.0
            stats["newest_session_age"] = 0.0

        return stats

    def reset_all_sessions(self):
        """
        重置所有会话
        清空所有活跃会话，重置统计信息
        """
        cleared_count = len(self.active_sessions)
        self.active_sessions.clear()
        self.session_stats = {
            "total_sessions_created": 0,
            "active_sessions_count": 0,
            "expired_sessions_count": 0
        }

        logger.info(f"🔄 重置所有会话 - 清除了{cleared_count}个活跃会话")

    def cleanup_all_sessions(self):
        """
        清理所有会话（兼容性方法）
        与reset_all_sessions功能相同
        """
        self.reset_all_sessions()

# 全局会话管理器实例
_global_session_manager = None

def get_session_manager() -> SimpleSessionManager:
    """
    获取全局会话管理器实例（单例模式）
    """
    global _global_session_manager

    if _global_session_manager is None:
        _global_session_manager = SimpleSessionManager()

    return _global_session_manager