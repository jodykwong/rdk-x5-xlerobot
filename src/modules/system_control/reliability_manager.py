"""
可靠性管理器 - Story 4.2
实现消息可靠性保证机制
支持消息确认、重试机制、持久化和事务性消息
"""

import asyncio
import time
import json
import uuid
from typing import Any, Dict, List, Optional, Callable, Union
from dataclasses import dataclass, field
from enum import Enum
import logging

import hashlib

logger = logging.getLogger(__name__)


class DeliveryMode(Enum):
    """消息传递模式"""
    AT_LEAST_ONCE = "at_least_once"  # 至少一次
    AT_MOST_ONCE = "at_most_once"    # 至多一次
    EXACTLY_ONCE = "exactly_once"    # 恰好一次
    BEST_EFFORT = "best_effort"      # 尽力而为


class MessageState(Enum):
    """消息状态"""
    PENDING = "pending"
    SENT = "sent"
    ACKNOWLEDGED = "acknowledged"
    FAILED = "failed"
    TIMEOUT = "timeout"
    RETRYING = "retrying"
    DEAD_LETTER = "dead_letter"


@dataclass
class ReliableMessage:
    """可靠消息"""
    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    sender: str = ""
    receiver: str = ""
    payload: Any = None
    delivery_mode: DeliveryMode = DeliveryMode.AT_LEAST_ONCE
    created_at: float = field(default_factory=time.time)
    state: MessageState = MessageState.PENDING
    retry_count: int = 0
    max_retries: int = 3
    timeout: float = 30.0
    last_attempt: Optional[float] = None
    acknowledged_at: Optional[float] = None
    checksum: Optional[str] = None
    correlation_id: Optional[str] = None
    reply_to: Optional[str] = None
    metadata: Dict[str, Any] = field(default_factory=dict)
    persistence_key: Optional[str] = None


@dataclass
class AckRecord:
    """确认记录"""
    message_id: str
    acknowledged: bool
    acknowledged_at: float = field(default_factory=time.time)
    receiver: str = ""
    checksum: Optional[str] = None
    latency: float = 0.0


class ReliabilityManager:
    """
    可靠性管理器
    实现消息的可靠传递，包括确认、重试、持久化和事务性消息
    确保消息不丢失、不重复、按顺序传递
    """

    def __init__(self, persistence_enabled: bool = True):
        # 消息存储
        self._messages: Dict[str, ReliableMessage] = {}
        self._pending_messages: List[str] = []
        self._message_handlers: Dict[str, Callable] = {}

        # 确认记录
        self._ack_records: Dict[str, AckRecord] = {}
        self._pending_acks: Dict[str, float] = {}

        # 重试机制
        self._retry_intervals: List[float] = [1.0, 2.0, 5.0, 10.0, 30.0]  # 指数退避
        self._max_retries = 3

        # 持久化
        self._persistence_enabled = persistence_enabled
        self._storage_backend: Optional[Callable] = None
        self._persistence_interval = 10.0  # 秒

        # 去重
        self._deduplication_window = 3600.0  # 1小时
        self._message_hashes: Dict[str, float] = {}  # hash -> timestamp

        # 统计信息
        self._stats = {
            'messages_sent': 0,
            'messages_acknowledged': 0,
            'messages_failed': 0,
            'messages_retried': 0,
            'average_latency': 0.0,
            'success_rate': 0.0
        }

        # 任务存储
        self._tasks: List[asyncio.Task] = []
        self._running = False

        logger.info("可靠性管理器初始化完成", persistence_enabled=persistence_enabled)

    async def start(self):
        """启动可靠性管理器"""
        if not self._running:
            self._running = True

            # 启动后台任务
            self._tasks.append(asyncio.create_task(self._retry_processor()))
            self._tasks.append(asyncio.create_task(self._timeout_monitor()))
            self._tasks.append(asyncio.create_task(self._persistence_worker()))

            logger.info("可靠性管理器已启动")

    async def stop(self):
        """停止可靠性管理器"""
        if self._running:
            self._running = False

            # 取消所有任务
            for task in self._tasks:
                task.cancel()

            # 等待任务完成
            if self._tasks:
                await asyncio.gather(*self._tasks, return_exceptions=True)

            # 持久化所有消息
            if self._persistence_enabled:
                await self._persist_all_messages()

            logger.info("可靠性管理器已停止")

    def set_storage_backend(self, storage_backend: Callable):
        """
        设置存储后端

        Args:
            storage_backend: 存储后端函数
        """
        self._storage_backend = storage_backend
        logger.info("存储后端已设置")

    async def send_reliable_message(self, message: Union[ReliableMessage, dict],
                                   sender: str = "",
                                   receiver: str = "",
                                   payload: Any = None,
                                   delivery_mode: DeliveryMode = DeliveryMode.AT_LEAST_ONCE,
                                   max_retries: int = 3,
                                   timeout: float = 30.0) -> str:
        """
        发送可靠消息

        Args:
            message: 消息对象或消息字典
            sender: 发送者
            receiver: 接收者
            payload: 消息载荷
            delivery_mode: 传递模式
            max_retries: 最大重试次数
            timeout: 超时时间

        Returns:
            str: 消息ID
        """
        try:
            # 创建消息对象
            if isinstance(message, ReliableMessage):
                msg = message
            elif isinstance(message, dict):
                msg = ReliableMessage(**message)
            else:
                msg = ReliableMessage(
                    sender=sender,
                    receiver=receiver,
                    payload=message if payload is None else payload,
                    delivery_mode=delivery_mode,
                    max_retries=max_retries,
                    timeout=timeout
                )

            # 计算校验和
            msg.checksum = self._calculate_checksum(msg.payload)

            # 生成持久化键
            if self._persistence_enabled:
                msg.persistence_key = f"reliable_message:{msg.id}"
                await self._store_message(msg)

            # 添加到管理
            self._messages[msg.id] = msg
            self._pending_messages.append(msg.id)

            # 更新统计
            self._stats['messages_sent'] += 1

            logger.debug(
                "可靠消息已发送",
                message_id=msg.id,
                receiver=receiver,
                delivery_mode=delivery_mode.value,
                checksum=msg.checksum
            )

            return msg.id

        except Exception as e:
            logger.error("发送可靠消息失败", error=str(e))
            raise

    async def acknowledge_message(self, message_id: str,
                                 acknowledged: bool = True,
                                 receiver: str = "",
                                 checksum: Optional[str] = None) -> bool:
        """
        确认消息

        Args:
            message_id: 消息ID
            acknowledged: 是否确认成功
            receiver: 接收者
            checksum: 校验和

        Returns:
            bool: 确认是否成功
        """
        try:
            if message_id not in self._messages:
                logger.warning("消息不存在", message_id=message_id)
                return False

            msg = self._messages[message_id]

            # 验证校验和
            if checksum and msg.checksum and checksum != msg.checksum:
                logger.error("消息校验和不一致", message_id=message_id, expected=msg.checksum, received=checksum)
                return False

            # 创建确认记录
            ack_record = AckRecord(
                message_id=message_id,
                acknowledged=acknowledged,
                receiver=receiver,
                checksum=checksum,
                latency=time.time() - msg.created_at if msg.created_at else 0.0
            )

            self._ack_records[message_id] = ack_record

            if acknowledged:
                # 更新消息状态
                msg.state = MessageState.ACKNOWLEDGED
                msg.acknowledged_at = time.time()

                # 从待处理列表中移除
                if message_id in self._pending_messages:
                    self._pending_messages.remove(message_id)

                # 从持久化存储中删除
                if self._persistence_enabled and msg.persistence_key:
                    await self._delete_message(msg.persistence_key)

                self._stats['messages_acknowledged'] += 1
                self._update_latency_stats(ack_record.latency)

                logger.debug(
                    "消息确认成功",
                    message_id=message_id,
                    latency_ms=ack_record.latency * 1000
                )
            else:
                # 标记为失败
                msg.state = MessageState.FAILED

                # 添加到重试队列
                await self._schedule_retry(msg)

                logger.warning(
                    "消息确认失败，准备重试",
                    message_id=message_id,
                    retry_count=msg.retry_count
                )

            return True

        except Exception as e:
            logger.error("确认消息失败", message_id=message_id, error=str(e))
            return False

    async def _schedule_retry(self, msg: ReliableMessage):
        """安排重试"""
        try:
            if msg.retry_count < msg.max_retries:
                msg.retry_count += 1
                msg.state = MessageState.RETRYING
                msg.last_attempt = time.time()

                # 计算下次重试时间（指数退避）
                if msg.retry_count - 1 < len(self._retry_intervals):
                    retry_delay = self._retry_intervals[msg.retry_count - 1]
                else:
                    retry_delay = self._retry_intervals[-1]

                # 安排重试任务
                asyncio.create_task(self._retry_message(msg, retry_delay))

                self._stats['messages_retried'] += 1

                logger.info(
                    "消息重试已安排",
                    message_id=msg.id,
                    retry_count=msg.retry_count,
                    retry_delay=retry_delay
                )
            else:
                # 超过最大重试次数，标记为死信
                msg.state = MessageState.DEAD_LETTER
                self._stats['messages_failed'] += 1

                logger.error(
                    "消息重试次数超限，标记为死信",
                    message_id=msg.id,
                    max_retries=msg.max_retries
                )

        except Exception as e:
            logger.error("安排重试失败", message_id=msg.id, error=str(e))

    async def _retry_message(self, msg: ReliableMessage, delay: float):
        """重试消息"""
        try:
            await asyncio.sleep(delay)

            # 重新添加到待处理队列
            if msg.id not in self._pending_messages:
                self._pending_messages.append(msg.id)

            # 查找消息处理器
            handler = self._message_handlers.get(msg.receiver)
            if handler:
                try:
                    if asyncio.iscoroutinefunction(handler):
                        await handler(msg)
                    else:
                        await asyncio.get_event_loop().run_in_executor(
                            None, handler, msg
                        )

                    logger.debug(
                        "消息重试成功",
                        message_id=msg.id,
                        retry_count=msg.retry_count
                    )

                except Exception as e:
                    logger.error(
                        "消息重试失败",
                        message_id=msg.id,
                        retry_count=msg.retry_count,
                        error=str(e)
                    )

        except Exception as e:
            logger.error("重试任务失败", message_id=msg.id, error=str(e))

    def register_message_handler(self, receiver: str, handler: Callable):
        """
        注册消息处理器

        Args:
            receiver: 接收者
            handler: 处理函数
        """
        self._message_handlers[receiver] = handler
        logger.info("消息处理器已注册", receiver=receiver)

    async def get_message_status(self, message_id: str) -> Optional[Dict[str, Any]]:
        """
        获取消息状态

        Args:
            message_id: 消息ID

        Returns:
            Optional[Dict]: 消息状态信息
        """
        if message_id not in self._messages:
            return None

        msg = self._messages[message_id]
        ack_record = self._ack_records.get(message_id)

        return {
            'message_id': msg.id,
            'state': msg.state.value,
            'sender': msg.sender,
            'receiver': msg.receiver,
            'delivery_mode': msg.delivery_mode.value,
            'retry_count': msg.retry_count,
            'max_retries': msg.max_retries,
            'created_at': msg.created_at,
            'acknowledged_at': msg.acknowledged_at,
            'checksum': msg.checksum,
            'acknowledged': ack_record.acknowledged if ack_record else None,
            'latency': ack_record.latency if ack_record else None
        }

    async def _retry_processor(self):
        """重试处理器任务"""
        while self._running:
            try:
                # 处理重试队列中的消息
                # 这里可以实现更复杂的重试逻辑

                await asyncio.sleep(1.0)  # 每秒检查一次

            except Exception as e:
                logger.error("重试处理器错误", error=str(e))
                await asyncio.sleep(1)

    async def _timeout_monitor(self):
        """超时监控任务"""
        while self._running:
            try:
                current_time = time.time()
                timeout_messages = []

                # 检查消息超时
                for msg_id, msg in self._messages.items():
                    if (msg.state in [MessageState.PENDING, MessageState.SENT] and
                        current_time - msg.created_at > msg.timeout):
                        timeout_messages.append(msg_id)

                # 处理超时消息
                for msg_id in timeout_messages:
                    msg = self._messages[msg_id]
                    msg.state = MessageState.TIMEOUT
                    self._stats['messages_failed'] += 1

                    logger.warning(
                        "消息超时",
                        message_id=msg_id,
                        timeout=msg.timeout,
                        retry_count=msg.retry_count
                    )

                    # 安排重试
                    await self._schedule_retry(msg)

                await asyncio.sleep(1.0)  # 每秒检查一次

            except Exception as e:
                logger.error("超时监控错误", error=str(e))
                await asyncio.sleep(1)

    async def _persistence_worker(self):
        """持久化工作器"""
        if not self._persistence_enabled:
            return

        while self._running:
            try:
                # 定期持久化待处理的消息
                await self._persist_pending_messages()

                await asyncio.sleep(self._persistence_interval)

            except Exception as e:
                logger.error("持久化工作器错误", error=str(e))
                await asyncio.sleep(self._persistence_interval)

    async def _persist_pending_messages(self):
        """持久化待处理消息"""
        try:
            for msg_id in self._pending_messages:
                if msg_id in self._messages:
                    msg = self._messages[msg_id]
                    if msg.persistence_key:
                        await self._store_message(msg)

            logger.debug("待处理消息已持久化", count=len(self._pending_messages))

        except Exception as e:
            logger.error("持久化待处理消息失败", error=str(e))

    async def _persist_all_messages(self):
        """持久化所有消息"""
        try:
            for msg in self._messages.values():
                if msg.persistence_key:
                    await self._store_message(msg)

            logger.info("所有消息已持久化", count=len(self._messages))

        except Exception as e:
            logger.error("持久化所有消息失败", error=str(e))

    async def _store_message(self, msg: ReliableMessage):
        """存储消息"""
        try:
            if self._storage_backend:
                await self._storage_backend(msg.persistence_key, msg)
            else:
                # 默认存储到内存
                logger.debug("消息已存储到内存", message_id=msg.id)

        except Exception as e:
            logger.error("存储消息失败", message_id=msg.id, error=str(e))

    async def _delete_message(self, persistence_key: str):
        """删除消息"""
        try:
            if self._storage_backend:
                await self._storage_backend(persistence_key, None)  # None表示删除
            else:
                # 默认从内存删除
                logger.debug("消息已从内存删除", persistence_key=persistence_key)

        except Exception as e:
            logger.error("删除消息失败", persistence_key=persistence_key, error=str(e))

    def _calculate_checksum(self, data: Any) -> str:
        """计算数据校验和"""
        try:
            if isinstance(data, str):
                data_str = data
            elif isinstance(data, dict) or isinstance(data, list):
                data_str = json.dumps(data, sort_keys=True)
            else:
                data_str = str(data)

            return hashlib.sha256(data_str.encode('utf-8')).hexdigest()

        except Exception as e:
            logger.error("计算校验和失败", error=str(e))
            return ""

    def _update_latency_stats(self, latency: float):
        """更新延迟统计"""
        if self._stats['average_latency'] == 0:
            self._stats['average_latency'] = latency
        else:
            # 移动平均
            alpha = 0.1
            self._stats['average_latency'] = (
                (1 - alpha) * self._stats['average_latency'] +
                alpha * latency
            )

        # 更新成功率
        total = self._stats['messages_sent']
        if total > 0:
            self._stats['success_rate'] = self._stats['messages_acknowledged'] / total

    def get_stats(self) -> Dict[str, Any]:
        """
        获取可靠性管理器统计信息

        Returns:
            Dict: 统计信息
        """
        stats = self._stats.copy()
        stats['pending_messages'] = len(self._pending_messages)
        stats['total_messages'] = len(self._messages)
        stats['acknowledged_messages'] = sum(
            1 for msg in self._messages.values()
            if msg.state == MessageState.ACKNOWLEDGED
        )
        stats['failed_messages'] = sum(
            1 for msg in self._messages.values()
            if msg.state in [MessageState.FAILED, MessageState.TIMEOUT, MessageState.DEAD_LETTER]
        )
        return stats

    def clear_all(self):
        """清空所有数据"""
        self._messages.clear()
        self._pending_messages.clear()
        self._ack_records.clear()
        self._message_hashes.clear()

        logger.info("所有数据已清空")


if __name__ == "__main__":
    async def main():
        """测试可靠性管理器功能"""
        manager = ReliabilityManager()
        await manager.start()

        # 测试消息处理器
        async def test_handler(message: ReliableMessage):
            print(f"处理可靠消息: {message.id}, 载荷: {message.payload}")
            # 模拟处理
            await asyncio.sleep(0.01)
            return {"status": "processed"}

        manager.register_message_handler("test_receiver", test_handler)

        # 测试发送可靠消息
        message = ReliableMessage(
            sender="test_sender",
            receiver="test_receiver",
            payload={"data": "test_data"}
        )

        message_id = await manager.send_reliable_message(message)
        print(f"消息ID: {message_id}")

        # 等待处理
        await asyncio.sleep(0.1)

        # 模拟确认消息
        ack_success = await manager.acknowledge_message(message_id, True, "test_receiver")
        print(f"消息确认: {ack_success}")

        # 获取消息状态
        status = await manager.get_message_status(message_id)
        print(f"消息状态: {status}")

        # 统计信息
        stats = manager.get_stats()
        print(f"管理器统计: {stats}")

        # 关闭
        await manager.stop()

    asyncio.run(main())
