#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
对话上下文管理模块 - 内存优化版本

提供智能内存管理、对话历史压缩和性能优化功能。
"""

from enum import Enum
from typing import List, Dict, Any, Optional, Set
from dataclasses import dataclass, field
import datetime
import threading
import weakref
from collections import deque
import gc
import logging

logger = logging.getLogger(__name__)

class MessageRole(Enum):
    """消息角色枚举"""
    USER = "user"
    ASSISTANT = "assistant"
    SYSTEM = "system"

class ContextType(Enum):
    """上下文类型枚举"""
    SHORT_TERM = "short_term"
    LONG_TERM = "long_term"
    EPISODIC = "episodic"

@dataclass
class ContextMetadata:
    """上下文元数据"""
    session_id: str
    user_id: Optional[str] = None
    location: Optional[str] = None
    timestamp: Optional[datetime.datetime] = None
    extra_data: Optional[Dict[str, Any]] = None

@dataclass
class Message:
    """消息数据类 - 内存优化版本"""
    role: MessageRole
    content: str
    timestamp: datetime.datetime = field(default_factory=datetime.datetime.now)
    metadata: Optional[Dict[str, Any]] = field(default=None)
    message_id: str = field(default_factory=lambda: f"msg_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')}")
    size_bytes: int = field(init=False)

    def __post_init__(self):
        # 计算消息大小用于内存管理
        self.size_bytes = len(self.content.encode('utf-8')) + 100  # 基础开销
        # 优化存储 - 移除不必要的元数据
        if self.metadata and len(str(self.metadata)) > 500:
            self.metadata = {k: v for k, v in list(self.metadata.items())[:5]}

@dataclass
class SessionSummary:
    """会话摘要数据类"""
    session_id: str
    start_time: datetime.datetime
    end_time: datetime.datetime
    messages: List[Message]
    summary: str
    key_topics: List[str]

class DialogueContext:
    """
    内存优化的对话上下文管理器

    功能特性:
    - 智能内存管理
    - 自动对话压缩
    - 性能监控
    - 垃圾回收优化
    """

    def __init__(
        self,
        max_messages: int = 50,
        max_memory_mb: int = 10,
        enable_compression: bool = True,
        compression_threshold: int = 30,
        cleanup_interval: int = 100
    ):
        """
        初始化对话上下文管理器

        Args:
            max_messages: 最大消息数量
            max_memory_mb: 最大内存使用(MB)
            enable_compression: 是否启用对话压缩
            compression_threshold: 压缩阈值(消息数量)
            cleanup_interval: 清理间隔(消息数量)
        """
        self.max_messages = max_messages
        self.max_memory_bytes = max_memory_mb * 1024 * 1024
        self.enable_compression = enable_compression
        self.compression_threshold = compression_threshold
        self.cleanup_interval = cleanup_interval

        # 内存优化的存储结构
        self.messages: deque = deque(maxlen=max_messages)
        self.current_session_id = None
        self.context_type = ContextType.SHORT_TERM

        # 内存管理
        self.total_memory_usage = 0
        self.compression_count = 0
        self.cleanup_count = 0
        self.message_count = 0

        # 线程安全
        self.lock = threading.RLock()

        # 弱引用缓存
        self._message_cache: Dict[str, weakref.ref] = {}
        self._recent_messages: List[str] = []

        # 性能监控
        self.stats = {
            'total_messages': 0,
            'compressed_messages': 0,
            'memory_saved_mb': 0.0,
            'avg_message_size': 0.0,
            'last_cleanup': None
        }

        logger.info(f"✅ 对话上下文管理器初始化 (内存限制: {max_memory_mb}MB)")

    def add_message(self, role: MessageRole, content: str, metadata: Optional[Dict[str, Any]] = None):
        """添加消息到上下文 - 内存优化版本"""
        with self.lock:
            # 创建优化后的消息对象
            message = Message(
                role=role,
                content=self._compress_content(content) if len(content) > 1000 else content,
                metadata=self._optimize_metadata(metadata),
                message_id=f"msg_{self.message_count}_{datetime.datetime.now().strftime('%H%M%S')}"
            )

            # 添加到消息队列
            self.messages.append(message)
            self.message_count += 1
            self.total_memory_usage += message.size_bytes

            # 更新缓存
            self._update_cache(message)

            # 检查是否需要内存管理
            if self._should_trigger_memory_management():
                self._manage_memory()

            logger.debug(f"📝 添加消息: {message.message_id}, 总消息数: {len(self.messages)}")

    def _compress_content(self, content: str) -> str:
        """压缩消息内容"""
        if len(content) <= 200:
            return content

        # 保留开头和结尾，中间用省略号
        return content[:100] + "...[已压缩]..." + content[-50:]

    def _optimize_metadata(self, metadata: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        """优化元数据"""
        if not metadata:
            return None

        # 只保留重要的元数据字段
        important_keys = {'intent', 'confidence', 'entities', 'timestamp'}
        return {k: v for k, v in metadata.items() if k in important_keys}

    def _update_cache(self, message: Message):
        """更新消息缓存"""
        # 维护最近消息ID列表
        self._recent_messages.append(message.message_id)
        if len(self._recent_messages) > 20:
            self._recent_messages.pop(0)

        # 添加到弱引用缓存
        self._message_cache[message.message_id] = weakref.ref(message)

    def _should_trigger_memory_management(self) -> bool:
        """检查是否需要触发内存管理"""
        return (
            len(self.messages) > self.compression_threshold or
            self.total_memory_usage > self.max_memory_bytes or
            self.message_count % self.cleanup_interval == 0
        )

    def _manage_memory(self):
        """执行内存管理"""
        original_size = len(self.messages)
        original_memory = self.total_memory_usage

        # 1. 对话压缩
        if self.enable_compression and len(self.messages) > self.compression_threshold:
            self._compress_dialogue()

        # 2. 清理过期消息
        if len(self.messages) > self.max_messages:
            self._cleanup_old_messages()

        # 3. 垃圾回收
        if self.message_count % (self.cleanup_interval * 2) == 0:
            self._force_garbage_collection()

        # 更新统计
        messages_removed = original_size - len(self.messages)
        memory_saved = (original_memory - self.total_memory_usage) / (1024 * 1024)

        self.stats['compressed_messages'] += messages_removed
        self.stats['memory_saved_mb'] += memory_saved
        self.stats['last_cleanup'] = datetime.datetime.now()

        if messages_removed > 0:
            logger.info(f"🧹 内存管理完成: 清理 {messages_removed} 条消息, 节省 {memory_saved:.2f}MB")

    def _compress_dialogue(self):
        """压缩对话历史"""
        if len(self.messages) <= 10:
            return

        # 保留最近的重要消息
        recent_messages = list(self.messages)[-10:]

        # 创建压缩摘要
        compressed_summary = self._create_dialogue_summary()

        # 清空并重建消息列表
        self.messages.clear()
        self.total_memory_usage = 0

        # 添加压缩摘要作为系统消息
        summary_message = Message(
            role=MessageRole.SYSTEM,
            content=f"[对话历史摘要] {compressed_summary}",
            message_id=f"summary_{datetime.datetime.now().strftime('%H%M%S')}"
        )
        self.messages.append(summary_message)
        self.total_memory_usage += summary_message.size_bytes

        # 添加最近的消息
        for msg in recent_messages:
            self.messages.append(msg)
            self.total_memory_usage += msg.size_bytes

        self.compression_count += 1

    def _create_dialogue_summary(self) -> str:
        """创建对话摘要"""
        if not self.messages:
            return "空对话"

        # 统计对话内容
        user_messages = [m for m in self.messages if m.role == MessageRole.USER]
        assistant_messages = [m for m in self.messages if m.role == MessageRole.ASSISTANT]

        # 提取关键信息
        topics = set()
        for msg in user_messages[-5:]:  # 分析最近5条用户消息
            if "天气" in msg.content:
                topics.add("天气询问")
            elif "时间" in msg.content:
                topics.add("时间询问")
            elif "你好" in msg.content or "hi" in msg.content.lower():
                topics.add("问候")
            else:
                topics.add("一般对话")

        return f"对话包含 {len(user_messages)} 条用户消息和 {len(assistant_messages)} 条助手回复，主要话题: {', '.join(topics)}"

    def _cleanup_old_messages(self):
        """清理过期消息"""
        # 保留最近的消息
        messages_to_keep = list(self.messages)[-self.max_messages:]

        # 重新计算内存使用
        self.total_memory_usage = sum(msg.size_bytes for msg in messages_to_keep)

        # 重建消息队列
        self.messages.clear()
        for msg in messages_to_keep:
            self.messages.append(msg)

        self.cleanup_count += 1

    def _force_garbage_collection(self):
        """强制垃圾回收"""
        collected = gc.collect()
        logger.debug(f"🗑️ 垃圾回收完成，清理对象数: {collected}")

    def get_context_messages(self, limit: Optional[int] = None) -> List[Message]:
        """获取上下文消息 - 内存优化版本"""
        with self.lock:
            if limit:
                return list(self.messages)[-limit:]
            return list(self.messages)

    def clear_context(self):
        """清空上下文"""
        with self.lock:
            self.messages.clear()
            self.total_memory_usage = 0
            self._message_cache.clear()
            self._recent_messages.clear()
            gc.collect()
            logger.info("🧹 对话上下文已清空")

    def get_recent_context(self, num_messages: int = 5) -> str:
        """获取最近的上下文字符串"""
        recent_messages = self.get_context_messages(num_messages)
        context_parts = []
        for msg in recent_messages:
            role_prefix = {
                MessageRole.USER: "用户",
                MessageRole.ASSISTANT: "助手",
                MessageRole.SYSTEM: "系统"
            }.get(msg.role, "未知")
            context_parts.append(f"{role_prefix}: {msg.content}")
        return "\n".join(context_parts)

    def get_memory_stats(self) -> Dict[str, Any]:
        """获取内存使用统计"""
        with self.lock:
            return {
                **self.stats,
                'current_messages': len(self.messages),
                'memory_usage_mb': self.total_memory_usage / (1024 * 1024),
                'memory_limit_mb': self.max_memory_bytes / (1024 * 1024),
                'memory_usage_percent': (self.total_memory_usage / self.max_memory_bytes) * 100,
                'avg_message_size': self.total_memory_usage / max(len(self.messages), 1),
                'compression_count': self.compression_count,
                'cleanup_count': self.cleanup_count
            }

@dataclass
class DialogueContextNode:
    """对话上下文节点"""
    message: Message
    parent_id: Optional[str] = None
    child_ids: List[str] = None
    node_id: str = ""
    context_type: ContextType = ContextType.SHORT_TERM
    metadata: Optional[ContextMetadata] = None

    def __post_init__(self):
        if self.child_ids is None:
            self.child_ids = []
        if not self.node_id:
            import uuid
            self.node_id = str(uuid.uuid4())

# 创建全局对话上下文实例
global_dialogue_context = DialogueContext()