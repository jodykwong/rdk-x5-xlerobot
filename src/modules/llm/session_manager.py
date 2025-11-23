#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Story 2.2: 对话上下文管理 - 会话管理器

会话管理模块，实现用户会话ID生成、元数据管理、多用户并发支持和会话状态追踪。
提供完整的会话生命周期管理和性能监控。

作者: Dev Agent
故事ID: Story 2.2
Epic: 2 - 智能对话模块
"""

import os
import time
import uuid
import asyncio
import logging
from typing import Dict, List, Optional, Any, Set, Tuple
from dataclasses import dataclass, field, asdict
from datetime import datetime, timedelta
from enum import Enum
import threading
import json
from collections import defaultdict, deque

from .dialogue_context import DialogueContext, MessageRole, ContextType


logger = logging.getLogger(__name__)


class SessionStatus(Enum):
    """会话状态"""
    ACTIVE = "active"        # 活跃状态
    IDLE = "idle"           # 空闲状态
    EXPIRED = "expired"     # 已过期
    ARCHIVED = "archived"   # 已归档
    DELETED = "deleted"     # 已删除


class UserRole(Enum):
    """用户角色"""
    ADMIN = "admin"         # 管理员
    PREMIUM = "premium"     # 高级用户
    STANDARD = "standard"   # 标准用户
    GUEST = "guest"         # 访客用户


@dataclass
class UserProfile:
    """用户档案"""
    user_id: str
    username: str
    email: Optional[str]
    role: UserRole = UserRole.STANDARD
    created_at: float = field(default_factory=time.time)
    last_login: float = field(default_factory=time.time)
    session_count: int = 0
    preferences: Dict[str, Any] = field(default_factory=dict)
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class SessionConfig:
    """会话配置"""
    max_concurrent_sessions: int = 10
    default_ttl: int = 3600  # 1小时
    max_messages: int = 20
    max_tokens: int = 4000
    enable_compression: bool = True
    auto_cleanup: bool = True
    cleanup_interval: int = 300  # 5分钟清理一次


@dataclass
class SessionInfo:
    """会话信息"""
    session_id: str
    user_id: str
    status: SessionStatus
    created_at: float = field(default_factory=time.time)
    last_activity: float = field(default_factory=time.time)
    message_count: int = 0
    config: SessionConfig = field(default_factory=SessionConfig)
    metadata: Dict[str, Any] = field(default_factory=dict)


class SessionManager:
    """
    会话管理器

    功能特性:
    - 用户会话ID生成和管理
    - 会话元数据管理
    - 多用户并发支持
    - 会话状态追踪
    - 会话清理机制
    - 性能监控和统计
    """

    def __init__(self, config: Optional[SessionConfig] = None):
        """
        初始化会话管理器

        Args:
            config: 会话配置
        """
        self.config = config or SessionConfig()

        # 存储
        self.users: Dict[str, UserProfile] = {}
        self.sessions: Dict[str, SessionInfo] = {}
        self.user_sessions: Dict[str, Set[str]] = defaultdict(set)
        self.session_queue = deque()

        # 上下文管理器
        self.context_manager = DialogueContext(
            max_messages=self.config.max_messages
        )

        # 并发控制
        self.lock = threading.RLock()

        # 统计信息
        self.stats = {
            'total_users': 0,
            'active_users': 0,
            'total_sessions': 0,
            'active_sessions': 0,
            'expired_sessions': 0,
            'total_messages': 0,
            'peak_concurrent_sessions': 0
        }

        # 清理任务
        self.cleanup_task: Optional[asyncio.Task] = None
        self._start_cleanup_task()

        logger.info("✅ 会话管理器初始化完成")
        logger.info(f"   - 最大并发会话: {self.config.max_concurrent_sessions}")
        logger.info(f"   - 默认TTL: {self.config.default_ttl}秒")
        logger.info(f"   - 自动清理: {self.config.auto_cleanup}")

    def _start_cleanup_task(self):
        """启动清理任务"""
        if self.config.auto_cleanup:
            try:
                self.cleanup_task = asyncio.create_task(self._cleanup_loop())
            except RuntimeError:
                # 没有运行的事件循环，延迟启动
                self.cleanup_task = None
                logger.warning("⚠️ 没有事件循环，清理任务将在异步上下文中延迟启动")

    async def _cleanup_loop(self):
        """清理循环"""
        while True:
            try:
                await asyncio.sleep(self.config.cleanup_interval)
                await self._cleanup_expired_sessions()
                await self._cleanup_inactive_users()
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"❌ 清理循环错误: {e}")

    async def _cleanup_expired_sessions(self):
        """清理过期会话"""
        current_time = time.time()
        expired_sessions = []

        with self.lock:
            for session_id, session_info in self.sessions.items():
                if (current_time - session_info.last_activity > session_info.config.default_ttl and
                    session_info.status == SessionStatus.EXPIRED):
                    expired_sessions.append(session_id)

        for session_id in expired_sessions:
            await self.archive_session(session_id)

        if expired_sessions:
            logger.info(f"🧹 清理过期会话: {len(expired_sessions)}个")

    async def _cleanup_inactive_users(self):
        """清理非活跃用户"""
        current_time = time.time()
        inactive_threshold = 86400 * 7  # 7天

        inactive_users = []
        with self.lock:
            for user_id, user_profile in self.users.items():
                if current_time - user_profile.last_login > inactive_threshold:
                    inactive_users.append(user_id)

        if inactive_users:
            logger.info(f"🧹 清理非活跃用户: {len(inactive_users)}个")

    def create_user(
        self,
        username: str,
        email: Optional[str] = None,
        role: UserRole = UserRole.STANDARD,
        preferences: Optional[Dict[str, Any]] = None
    ) -> str:
        """
        创建用户

        Args:
            username: 用户名
            email: 邮箱
            role: 用户角色
            preferences: 用户偏好

        Returns:
            str: 用户ID
        """
        user_id = str(uuid.uuid4())[:12]

        user_profile = UserProfile(
            user_id=user_id,
            username=username,
            email=email,
            role=role,
            preferences=preferences or {}
        )

        with self.lock:
            self.users[user_id] = user_profile
            self.user_sessions[user_id] = set()

            # 更新统计
            self.stats['total_users'] += 1
            self.stats['active_users'] += 1

        logger.info(f"👤 创建用户: {user_id}, 用户名: {username}")
        return user_id

    def get_user(self, user_id: str) -> Optional[UserProfile]:
        """获取用户信息"""
        return self.users.get(user_id)

    def update_user(self, user_id: str, **kwargs) -> bool:
        """更新用户信息"""
        if user_id not in self.users:
            return False

        user_profile = self.users[user_id]

        # 更新允许的字段
        allowed_fields = ['username', 'email', 'role', 'preferences', 'metadata']
        for key, value in kwargs.items():
            if key in allowed_fields:
                setattr(user_profile, key, value)

        logger.debug(f"👤 更新用户: {user_id}")
        return True

    def create_session(
        self,
        user_id: str,
        session_config: Optional[SessionConfig] = None
    ) -> str:
        """
        创建会话

        Args:
            user_id: 用户ID
            session_config: 会话配置

        Returns:
            str: 会话ID
        """
        if user_id not in self.users:
            raise ValueError(f"用户不存在: {user_id}")

        config = session_config or self.config

        # 检查并发限制
        user_active_sessions = len(self.user_sessions.get(user_id, set()))
        if user_active_sessions >= config.max_concurrent_sessions:
            raise ValueError(f"用户 {user_id} 达到最大并发会话数限制: {config.max_concurrent_sessions}")

        # 生成会话ID
        session_id = str(uuid.uuid4())[:12]

        session_info = SessionInfo(
            session_id=session_id,
            user_id=user_id,
            status=SessionStatus.ACTIVE,
            config=config
        )

        with self.lock:
            # 创建会话
            self.sessions[session_id] = session_info
            self.user_sessions[user_id].add(session_id)

            # 创建上下文
            context_session_id = self.context_manager.create_session(
                user_id,
                {'session_id': session_id, 'config': asdict(config)}
            )

            # 更新统计
            self.stats['total_sessions'] += 1
            self.stats['active_sessions'] += 1

            # 更新峰值
            if self.stats['active_sessions'] > self.stats['peak_concurrent_sessions']:
                self.stats['peak_concurrent_sessions'] = self.stats['active_sessions']

            # 更新用户统计
            user_profile = self.users[user_id]
            user_profile.session_count += 1
            user_profile.last_login = time.time()

        logger.info(f"📝 创建会话: {session_id}, 用户: {user_id}")
        return session_id

    def get_session(self, session_id: str) -> Optional[SessionInfo]:
        """获取会话信息"""
        return self.sessions.get(session_id)

    def update_session_activity(self, session_id: str):
        """更新会话活动时间"""
        if session_id in self.sessions:
            self.sessions[session_id].last_activity = time.time()
            self.sessions[session_id].status = SessionStatus.ACTIVE

    async def add_message_to_session(
        self,
        session_id: str,
        role: MessageRole,
        content: str
    ) -> bool:
        """
        添加消息到会话

        Args:
            session_id: 会话ID
            role: 消息角色
            content: 消息内容

        Returns:
            bool: 是否成功
        """
        if session_id not in self.sessions:
            return False

        try:
            # 添加消息到上下文管理器
            message = self.context_manager.add_message(session_id, role, content)

            # 更新会话统计
            with self.lock:
                session_info = self.sessions[session_id]
                session_info.message_count += 1
                session_info.last_activity = time.time()
                session_info.status = SessionStatus.ACTIVE

                self.stats['total_messages'] += 1

            logger.debug(f"📨 添加消息到会话 {session_id}: {role.value}")
            return True

        except Exception as e:
            logger.error(f"❌ 添加消息失败: {e}")
            return False

    def get_session_context(
        self,
        session_id: str,
        include_summary: bool = True
    ) -> List[Dict[str, Any]]:
        """
        获取会话上下文

        Args:
            session_id: 会话ID
            include_summary: 是否包含摘要

        Returns:
            List[Dict[str, Any]]: 上下文消息列表
        """
        try:
            messages = self.context_manager.get_context(session_id, include_summary)
            return [asdict(msg) for msg in messages]
        except Exception as e:
            logger.error(f"❌ 获取会话上下文失败: {e}")
            return []

    async def deactivate_session(self, session_id: str):
        """停用会话"""
        if session_id not in self.sessions:
            return

        with self.lock:
            session_info = self.sessions[session_id]
            session_info.status = SessionStatus.IDLE
            self.stats['active_sessions'] -= 1

        logger.info(f"⏸️ 停用会话: {session_id}")

    async def archive_session(self, session_id: str):
        """归档会话"""
        if session_id not in self.sessions:
            return

        with self.lock:
            session_info = self.sessions[session_id]
            session_info.status = SessionStatus.ARCHIVED

            # 从用户会话列表中移除
            user_id = session_info.user_id
            if user_id in self.user_sessions:
                self.user_sessions[user_id].discard(session_id)

            # 更新统计
            self.stats['active_sessions'] -= 1
            self.stats['expired_sessions'] += 1

        logger.info(f"📦 归档会话: {session_id}")

    async def delete_session(self, session_id: str) -> bool:
        """删除会话"""
        if session_id not in self.sessions:
            return False

        with self.lock:
            session_info = self.sessions[session_id]
            user_id = session_info.user_id

            # 删除会话
            del self.sessions[session_id]

            # 从用户会话列表中移除
            if user_id in self.user_sessions:
                self.user_sessions[user_id].discard(session_id)

            # 更新统计
            if session_info.status == SessionStatus.ACTIVE:
                self.stats['active_sessions'] -= 1

        # 删除上下文
        try:
            self.context_manager.delete_session(session_id)
        except Exception as e:
            logger.warning(f"⚠️ 删除上下文失败: {e}")

        logger.info(f"🗑️ 删除会话: {session_id}")
        return True

    def get_user_sessions(self, user_id: str) -> List[str]:
        """获取用户的所有会话"""
        return list(self.user_sessions.get(user_id, set()))

    def get_user_active_sessions(self, user_id: str) -> List[str]:
        """获取用户的活跃会话"""
        active_sessions = []
        for session_id in self.user_sessions.get(user_id, set()):
            session_info = self.sessions.get(session_id)
            if session_info and session_info.status == SessionStatus.ACTIVE:
                active_sessions.append(session_id)
        return active_sessions

    def search_sessions(
        self,
        user_id: Optional[str] = None,
        status: Optional[SessionStatus] = None,
        start_time: Optional[float] = None,
        end_time: Optional[float] = None,
        limit: int = 100
    ) -> List[Dict[str, Any]]:
        """
        搜索会话

        Args:
            user_id: 用户ID过滤
            status: 状态过滤
            start_time: 开始时间过滤
            end_time: 结束时间过滤
            limit: 限制返回数量

        Returns:
            List[Dict[str, Any]]: 匹配的会话列表
        """
        results = []

        with self.lock:
            for session_id, session_info in self.sessions.items():
                # 应用过滤条件
                if user_id and session_info.user_id != user_id:
                    continue

                if status and session_info.status != status:
                    continue

                if start_time and session_info.created_at < start_time:
                    continue

                if end_time and session_info.created_at > end_time:
                    continue

                results.append({
                    'session_id': session_id,
                    'user_id': session_info.user_id,
                    'status': session_info.status.value,
                    'created_at': session_info.created_at,
                    'last_activity': session_info.last_activity,
                    'message_count': session_info.message_count,
                    'duration': time.time() - session_info.created_at
                })

                if len(results) >= limit:
                    break

        # 按创建时间排序
        results.sort(key=lambda x: x['created_at'], reverse=True)
        return results

    def get_session_stats(self, session_id: str) -> Dict[str, Any]:
        """获取会话统计信息"""
        if session_id not in self.sessions:
            return {}

        session_info = self.sessions[session_id]
        user_profile = self.users.get(session_info.user_id)

        # 获取上下文统计
        context_stats = self.context_manager.get_session_stats(session_id)

        return {
            'session_id': session_id,
            'user_id': session_info.user_id,
            'username': user_profile.username if user_profile else 'Unknown',
            'status': session_info.status.value,
            'created_at': session_info.created_at,
            'last_activity': session_info.last_activity,
            'duration': time.time() - session_info.created_at,
            'message_count': session_info.message_count,
            'context_stats': context_stats,
            'user_role': user_profile.role.value if user_profile else 'unknown'
        }

    def get_user_stats(self, user_id: str) -> Dict[str, Any]:
        """获取用户统计信息"""
        if user_id not in self.users:
            return {}

        user_profile = self.users[user_id]
        active_sessions = self.get_user_active_sessions(user_id)

        return {
            'user_id': user_id,
            'username': user_profile.username,
            'email': user_profile.email,
            'role': user_profile.role.value,
            'created_at': user_profile.created_at,
            'last_login': user_profile.last_login,
            'total_sessions': user_profile.session_count,
            'active_sessions': len(active_sessions),
            'preferences': user_profile.preferences,
            'total_messages': sum(
                self.sessions.get(sid).message_count
                for sid in active_sessions
                if self.sessions.get(sid)
            )
        }

    def get_global_stats(self) -> Dict[str, Any]:
        """获取全局统计信息"""
        with self.lock:
            # 计算活跃用户数
            active_users = len([
                uid for uid, sessions in self.user_sessions.items()
                if any(
                    self.sessions.get(sid).status == SessionStatus.ACTIVE
                    for sid in sessions
                    if sid in self.sessions
                )
            ])

            # 计算平均会话时长
            current_time = time.time()
            session_durations = [
                current_time - session_info.created_at
                for session_info in self.sessions.values()
                if session_info.status in [SessionStatus.ACTIVE, SessionStatus.IDLE]
            ]
            avg_session_duration = sum(session_durations) / len(session_durations) if session_durations else 0

            return {
                **self.stats,
                'active_users': active_users,
                'total_memory_usage': len(self.sessions) * 1000,  # 估算
                'average_session_duration': avg_session_duration,
                'cleanup_enabled': self.config.auto_cleanup,
                'context_manager_stats': {
                    'active_contexts': len(self.sessions),
                    'memory_usage': getattr(self.context_manager, 'memory_usage', 0)
                }
            }

    async def shutdown(self):
        """关闭会话管理器"""
        if self.cleanup_task:
            self.cleanup_task.cancel()
            try:
                await self.cleanup_task
            except asyncio.CancelledError:
                pass

        logger.info("🛑 会话管理器已关闭")


# ROS2节点集成
class SessionManagerNode:
    """会话管理器ROS2节点"""

    def __init__(self, node):
        """
        初始化会话管理器节点

        Args:
            node: ROS2节点实例
        """
        config = SessionConfig(
            max_concurrent_sessions=5,
            default_ttl=3600,
            auto_cleanup=True
        )
        self.session_manager = SessionManager(config)
        self.node = node

    async def create_user_and_session(self, username: str, email: Optional[str] = None) -> Tuple[str, str]:
        """
        创建用户和会话

        Args:
            username: 用户名
            email: 邮箱

        Returns:
            Tuple[str, str]: (用户ID, 会话ID)
        """
        try:
            # 创建用户
            user_id = self.session_manager.create_user(username, email)

            # 创建会话
            session_id = self.session_manager.create_session(user_id)

            self.node.get_logger().info(f"👤 创建用户和会话: 用户{user_id}, 会话{session_id}")
            return user_id, session_id

        except Exception as e:
            self.node.get_logger().error(f"❌ 创建用户和会话失败: {e}")
            raise

    async def get_user_sessions_and_stats(self, user_id: str) -> Dict[str, Any]:
        """
        获取用户会话和统计

        Args:
            user_id: 用户ID

        Returns:
            Dict[str, Any]: 用户信息和会话列表
        """
        try:
            user_profile = self.session_manager.get_user(user_id)
            sessions = self.session_manager.get_user_sessions(user_id)
            user_stats = self.session_manager.get_user_stats(user_id)

            result = {
                'user_profile': asdict(user_profile) if user_profile else None,
                'sessions': sessions,
                'stats': user_stats
            }

            self.node.get_logger().info(f"📊 获取用户信息: {user_id}, {len(sessions)}个会话")
            return result

        except Exception as e:
            self.node.get_logger().error(f"❌ 获取用户信息失败: {e}")
            return {}


if __name__ == '__main__':
    # 示例用法
    async def main():
        # 创建会话管理器
        config = SessionConfig(max_concurrent_sessions=5)
        manager = SessionManager(config)

        try:
            # 创建用户
            user_id = manager.create_user("test_user", "test@example.com")
            print(f"👤 创建用户: {user_id}")

            # 创建会话
            session_id = manager.create_session(user_id)
            print(f"📝 创建会话: {session_id}")

            # 添加消息
            await manager.add_message_to_session(session_id, MessageRole.USER, "你好")
            await manager.add_message_to_session(session_id, MessageRole.ASSISTANT, "你好！很高兴见到你")

            # 获取上下文
            context = manager.get_session_context(session_id)
            print(f"📖 获取上下文: {len(context)}条消息")

            # 获取统计信息
            user_stats = manager.get_user_stats(user_id)
            print(f"📊 用户统计: {user_stats}")

            global_stats = manager.get_global_stats()
            print(f"🌍 全局统计: {global_stats}")

        finally:
            await manager.shutdown()

    # 运行示例
    # asyncio.run(main())
