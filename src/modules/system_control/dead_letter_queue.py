"""
死信队列 - Story 4.2
实现死信队列和重试机制
处理无法正常投递或处理的消息
支持多种死信处理策略
"""

import asyncio
import time
import json
import uuid
from typing import Any, Dict, List, Optional, Callable, Union
from dataclasses import dataclass, field
from enum import Enum
import logging


logger = logging.getLogger(__name__)


class DeadLetterReason(Enum):
    """死信原因枚举"""
    MAX_RETRIES_EXCEEDED = "max_retries_exceeded"
    TIMEOUT = "timeout"
    INVALID_MESSAGE = "invalid_message"
    PROCESSING_ERROR = "processing_error"
    NETWORK_ERROR = "network_error"
    QUEUE_FULL = "queue_full"
    SCHEDULED_RETRY = "scheduled_retry"
    MANUAL_INTERVENTION = "manual_intervention"


class DeadLetterAction(Enum):
    """死信处理动作"""
    DISCARD = "discard"                    # 丢弃
    RETRY = "retry"                        # 重试
    ARCHIVE = "archive"                    # 归档
    NOTIFY = "notify"                      # 通知
    CUSTOM = "custom"                      # 自定义


@dataclass
class DeadLetterMessage:
    """死信消息"""
    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    original_message_id: Optional[str] = None
    sender: str = ""
    receiver: str = ""
    payload: Any = None
    reason: DeadLetterReason = DeadLetterReason.MAX_RETRIES_EXCEEDED
    error_message: Optional[str] = None
    error_details: Dict[str, Any] = field(default_factory=dict)
    retry_count: int = 0
    max_retries: int = 3
    created_at: float = field(default_factory=time.time)
    first_failed_at: Optional[float] = None
    last_retry_at: Optional[float] = None
    dead_letter_at: float = field(default_factory=time.time)
    action: DeadLetterAction = DeadLetterAction.DISCARD
    handler_result: Optional[Any] = None
    metadata: Dict[str, Any] = field(default_factory=dict)
    correlation_id: Optional[str] = None


@dataclass
class RetryPolicy:
    """重试策略"""
    max_retries: int = 3
    initial_delay: float = 1.0
    max_delay: float = 60.0
    backoff_multiplier: float = 2.0
    jitter: bool = True
    max_total_delay: float = 300.0  # 5分钟


class DeadLetterQueue:
    """
    死信队列
    收集无法正常处理的消息，提供多种处理策略
    支持消息重试、归档、通知等功能
    """

    def __init__(self, default_retry_policy: Optional[RetryPolicy] = None):
        # 死信存储
        self._dead_letters: Dict[str, DeadLetterMessage] = {}
        self._dead_letter_queue: List[str] = []

        # 处理策略
        self._default_retry_policy = default_retry_policy or RetryPolicy()
        self._action_handlers: Dict[DeadLetterAction, Callable] = {}
        self._custom_handlers: Dict[str, Callable] = {}

        # 重试队列
        self._retry_queue: List[DeadLetterMessage] = []
        self._retry_schedule: Dict[str, float] = {}  # message_id -> retry_at

        # 统计信息
        self._stats = {
            'total_dead_letters': 0,
            'processed_dead_letters': 0,
            'retried_messages': 0,
            'discarded_messages': 0,
            'archived_messages': 0,
            'average_retry_count': 0.0
        }

        # 告警和通知
        self._notification_handlers: List[Callable] = []
        self._alert_threshold = 100  # 死信数量告警阈值

        # 任务存储
        self._tasks: List[asyncio.Task] = []
        self._running = False

        # 注册默认处理器
        self._register_default_handlers()

        logger.info("死信队列初始化完成")

    def _register_default_handlers(self):
        """注册默认处理器"""
        self._action_handlers[DeadLetterAction.DISCARD] = self._handle_discard
        self._action_handlers[DeadLetterAction.RETRY] = self._handle_retry
        self._action_handlers[DeadLetterAction.ARCHIVE] = self._handle_archive
        self._action_handlers[DeadLetterAction.NOTIFY] = self._handle_notify
        self._action_handlers[DeadLetterAction.CUSTOM] = self._handle_custom

    async def start(self):
        """启动死信队列"""
        if not self._running:
            self._running = True

            # 启动后台任务
            self._tasks.append(asyncio.create_task(self._retry_processor()))
            self._tasks.append(asyncio.create_task(self._monitor_dead_letters()))
            self._tasks.append(asyncio.create_task(self._alert_checker()))

            logger.info("死信队列已启动")

    async def stop(self):
        """停止死信队列"""
        if self._running:
            self._running = False

            # 取消所有任务
            for task in self._tasks:
                task.cancel()

            # 等待任务完成
            if self._tasks:
                await asyncio.gather(*self._tasks, return_exceptions=True)

            logger.info("死信队列已停止")

    def add_dead_letter(self,
                       original_message_id: Optional[str],
                       sender: str,
                       receiver: str,
                       payload: Any,
                       reason: DeadLetterReason,
                       error_message: Optional[str] = None,
                       error_details: Optional[Dict[str, Any]] = None,
                       retry_count: int = 0,
                       metadata: Optional[Dict[str, Any]] = None) -> str:
        """
        添加死信

        Args:
            original_message_id: 原始消息ID
            sender: 发送者
            receiver: 接收者
            payload: 消息载荷
            reason: 死信原因
            error_message: 错误信息
            error_details: 错误详情
            retry_count: 重试次数
            metadata: 元数据

        Returns:
            str: 死信ID
        """
        try:
            # 创建死信消息
            dead_letter = DeadLetterMessage(
                original_message_id=original_message_id,
                sender=sender,
                receiver=receiver,
                payload=payload,
                reason=reason,
                error_message=error_message,
                error_details=error_details or {},
                retry_count=retry_count,
                first_failed_at=time.time() if retry_count == 0 else None,
                last_retry_at=time.time(),
                metadata=metadata or {}
            )

            # 添加到死信队列
            dead_letter_id = dead_letter.id
            self._dead_letters[dead_letter_id] = dead_letter
            self._dead_letter_queue.append(dead_letter_id)

            # 更新统计
            self._stats['total_dead_letters'] += 1

            logger.warning(
                "死信已添加",
                dead_letter_id=dead_letter_id,
                original_message_id=original_message_id,
                reason=reason.value,
                error_message=error_message
            )

            # 检查是否需要告警
            if len(self._dead_letter_queue) >= self._alert_threshold:
                logger.error(
                    "死信队列告警：数量超过阈值",
                    current_count=len(self._dead_letter_queue),
                    threshold=self._alert_threshold
                )

            return dead_letter_id

        except Exception as e:
            logger.error("添加死信失败", error=str(e))
            raise

    def set_dead_letter_action(self, dead_letter_id: str,
                              action: DeadLetterAction,
                              custom_handler: Optional[str] = None) -> bool:
        """
        设置死信处理动作

        Args:
            dead_letter_id: 死信ID
            action: 处理动作
            custom_handler: 自定义处理器名称

        Returns:
            bool: 设置是否成功
        """
        try:
            if dead_letter_id not in self._dead_letters:
                logger.warning("死信不存在", dead_letter_id=dead_letter_id)
                return False

            dead_letter = self._dead_letters[dead_letter_id]
            dead_letter.action = action

            if custom_handler:
                dead_letter.metadata['custom_handler'] = custom_handler

            logger.info(
                "死信处理动作已设置",
                dead_letter_id=dead_letter_id,
                action=action.value,
                custom_handler=custom_handler
            )

            return True

        except Exception as e:
            logger.error("设置死信处理动作失败", dead_letter_id=dead_letter_id, error=str(e))
            return False

    async def retry_dead_letter(self, dead_letter_id: str,
                               retry_policy: Optional[RetryPolicy] = None) -> bool:
        """
        重试死信

        Args:
            dead_letter_id: 死信ID
            retry_policy: 重试策略

        Returns:
            bool: 重试是否成功
        """
        try:
            if dead_letter_id not in self._dead_letters:
                logger.warning("死信不存在", dead_letter_id=dead_letter_id)
                return False

            dead_letter = self._dead_letters[dead_letter_id]
            policy = retry_policy or self._default_retry_policy

            # 检查重试次数限制
            if dead_letter.retry_count >= policy.max_retries:
                logger.warning(
                    "重试次数超过限制",
                    dead_letter_id=dead_letter_id,
                    retry_count=dead_letter.retry_count,
                    max_retries=policy.max_retries
                )
                return False

            # 计算重试延迟
            delay = self._calculate_retry_delay(
                dead_letter.retry_count,
                policy
            )

            # 安排重试
            self._retry_queue.append(dead_letter)
            self._retry_schedule[dead_letter.id] = time.time() + delay

            dead_letter.metadata['retry_scheduled_at'] = time.time()
            dead_letter.metadata['retry_delay'] = delay

            logger.info(
                "死信重试已安排",
                dead_letter_id=dead_letter_id,
                retry_count=dead_letter.retry_count,
                delay=delay
            )

            return True

        except Exception as e:
            logger.error("重试死信失败", dead_letter_id=dead_letter_id, error=str(e))
            return False

    def _calculate_retry_delay(self, retry_count: int, policy: RetryPolicy) -> float:
        """计算重试延迟"""
        # 指数退避
        delay = policy.initial_delay * (policy.backoff_multiplier ** retry_count)

        # 应用最大延迟限制
        delay = min(delay, policy.max_delay)

        # 添加抖动
        if policy.jitter:
            import random
            jitter = random.uniform(0, delay * 0.1)
            delay += jitter

        return delay

    async def _retry_processor(self):
        """重试处理器任务"""
        while self._running:
            try:
                current_time = time.time()

                # 检查需要重试的死信
                retry_messages = []
                for dead_letter in list(self._retry_queue):
                    retry_at = self._retry_schedule.get(dead_letter.id, 0)
                    if current_time >= retry_at:
                        retry_messages.append(dead_letter)

                # 执行重试
                for dead_letter in retry_messages:
                    if dead_letter.id in self._retry_queue:
                        self._retry_queue.remove(dead_letter)
                    if dead_letter.id in self._retry_schedule:
                        del self._retry_schedule[dead_letter.id]

                    # 更新重试计数
                    dead_letter.retry_count += 1
                    dead_letter.last_retry_at = time.time()

                    # 更新统计
                    self._stats['retried_messages'] += 1

                    logger.info(
                        "执行死信重试",
                        dead_letter_id=dead_letter.id,
                        retry_count=dead_letter.retry_count
                    )

                    # 这里可以触发原始消息的重新处理
                    # 具体实现依赖于消息系统架构

                await asyncio.sleep(1.0)  # 每秒检查一次

            except Exception as e:
                logger.error("重试处理器错误", error=str(e))
                await asyncio.sleep(1)

    async def _monitor_dead_letters(self):
        """监控死信队列"""
        while self._running:
            try:
                # 处理死信队列中的消息
                processed = 0
                max_process_per_cycle = 10  # 每周期最大处理数量

                for _ in range(max_process_per_cycle):
                    if not self._dead_letter_queue:
                        break

                    # 获取下一个死信
                    dead_letter_id = self._dead_letter_queue.pop(0)
                    if dead_letter_id not in self._dead_letters:
                        continue

                    dead_letter = self._dead_letters[dead_letter_id]

                    # 执行处理动作
                    handler = self._action_handlers.get(dead_letter.action)
                    if handler:
                        try:
                            if asyncio.iscoroutinefunction(handler):
                                result = await handler(dead_letter)
                            else:
                                result = await asyncio.get_event_loop().run_in_executor(
                                    None, handler, dead_letter
                                )

                            dead_letter.handler_result = result
                            self._stats['processed_dead_letters'] += 1

                            logger.debug(
                                "死信处理成功",
                                dead_letter_id=dead_letter_id,
                                action=dead_letter.action.value
                            )

                        except Exception as e:
                            logger.error(
                                "死信处理失败",
                                dead_letter_id=dead_letter_id,
                                action=dead_letter.action.value,
                                error=str(e)
                            )

                    processed += 1

                if processed == 0:
                    await asyncio.sleep(0.1)  # 没有消息处理时短暂休眠

            except Exception as e:
                logger.error("监控死信错误", error=str(e))
                await asyncio.sleep(1)

    async def _alert_checker(self):
        """告警检查器"""
        while self._running:
            try:
                # 检查死信队列长度
                queue_length = len(self._dead_letter_queue)

                if queue_length >= self._alert_threshold:
                    # 发送告警通知
                    await self._send_alert(
                        f"死信队列长度达到 {queue_length}，超过告警阈值 {self._alert_threshold}"
                    )

                await asyncio.sleep(60.0)  # 每分钟检查一次

            except Exception as e:
                logger.error("告警检查错误", error=str(e))
                await asyncio.sleep(60)

    async def _send_alert(self, message: str):
        """发送告警"""
        for handler in self._notification_handlers:
            try:
                if asyncio.iscoroutinefunction(handler):
                    await handler(message)
                else:
                    handler(message)
            except Exception as e:
                logger.error("发送告警失败", error=str(e))

    async def _handle_discard(self, dead_letter: DeadLetterMessage) -> bool:
        """处理丢弃动作"""
        self._stats['discarded_messages'] += 1
        logger.info("死信已丢弃", dead_letter_id=dead_letter.id)
        return True

    async def _handle_retry(self, dead_letter: DeadLetterMessage) -> bool:
        """处理重试动作"""
        success = await self.retry_dead_letter(dead_letter.id)
        if success:
            logger.info("死信重试已安排", dead_letter_id=dead_letter.id)
        return success

    async def _handle_archive(self, dead_letter: DeadLetterMessage) -> bool:
        """处理归档动作"""
        # 归档死信（实际实现中可能保存到文件或数据库）
        self._stats['archived_messages'] += 1

        logger.info(
            "死信已归档",
            dead_letter_id=dead_letter.id,
            reason=dead_letter.reason.value
        )

        return True

    async def _handle_notify(self, dead_letter: DeadLetterMessage) -> bool:
        """处理通知动作"""
        notification = {
            'dead_letter_id': dead_letter.id,
            'original_message_id': dead_letter.original_message_id,
            'reason': dead_letter.reason.value,
            'error_message': dead_letter.error_message,
            'retry_count': dead_letter.retry_count,
            'timestamp': dead_letter.dead_letter_at
        }

        await self._send_alert(f"死信通知: {json.dumps(notification)}")

        logger.info("死信通知已发送", dead_letter_id=dead_letter.id)
        return True

    async def _handle_custom(self, dead_letter: DeadLetterMessage) -> bool:
        """处理自定义动作"""
        custom_handler_name = dead_letter.metadata.get('custom_handler')
        if not custom_handler_name:
            logger.warning("自定义处理器未指定", dead_letter_id=dead_letter.id)
            return False

        handler = self._custom_handlers.get(custom_handler_name)
        if not handler:
            logger.warning("自定义处理器不存在", handler=custom_handler_name)
            return False

        try:
            if asyncio.iscoroutinefunction(handler):
                result = await handler(dead_letter)
            else:
                result = handler(dead_letter)

            logger.info(
                "自定义处理器执行成功",
                dead_letter_id=dead_letter.id,
                handler=custom_handler_name
            )

            return True

        except Exception as e:
            logger.error(
                "自定义处理器执行失败",
                dead_letter_id=dead_letter.id,
                handler=custom_handler_name,
                error=str(e)
            )

            return False

    def register_action_handler(self, action: DeadLetterAction, handler: Callable):
        """
        注册处理动作处理器

        Args:
            action: 处理动作
            handler: 处理函数
        """
        self._action_handlers[action] = handler
        logger.info("处理动作处理器已注册", action=action.value)

    def register_custom_handler(self, name: str, handler: Callable):
        """
        注册自定义处理器

        Args:
            name: 处理器名称
            handler: 处理函数
        """
        self._custom_handlers[name] = handler
        logger.info("自定义处理器已注册", name=name)

    def register_notification_handler(self, handler: Callable):
        """
        注册通知处理器

        Args:
            handler: 通知处理函数
        """
        self._notification_handlers.append(handler)
        logger.info("通知处理器已注册")

    def get_dead_letter(self, dead_letter_id: str) -> Optional[DeadLetterMessage]:
        """
        获取死信

        Args:
            dead_letter_id: 死信ID

        Returns:
            Optional[DeadLetterMessage]: 死信消息
        """
        return self._dead_letters.get(dead_letter_id)

    def get_dead_letters_by_reason(self, reason: DeadLetterReason) -> List[DeadLetterMessage]:
        """
        按原因获取死信

        Args:
            reason: 死信原因

        Returns:
            List[DeadLetterMessage]: 死信列表
        """
        return [
            dead_letter for dead_letter in self._dead_letters.values()
            if dead_letter.reason == reason
        ]

    def get_dead_letters_by_receiver(self, receiver: str) -> List[DeadLetterMessage]:
        """
        按接收者获取死信

        Args:
            receiver: 接收者

        Returns:
            List[DeadLetterMessage]: 死信列表
        """
        return [
            dead_letter for dead_letter in self._dead_letters.values()
            if dead_letter.receiver == receiver
        ]

    def get_stats(self) -> Dict[str, Any]:
        """
        获取死信队列统计信息

        Returns:
            Dict: 统计信息
        """
        stats = self._stats.copy()
        stats['current_dead_letters'] = len(self._dead_letter_queue)
        stats['queued_for_retry'] = len(self._retry_queue)
        stats['dead_letters_by_reason'] = {
            reason.value: len(self.get_dead_letters_by_reason(reason))
            for reason in DeadLetterReason
        }
        stats['average_retry_count'] = (
            sum(dl.retry_count for dl in self._dead_letters.values()) /
            max(len(self._dead_letters), 1)
        )
        return stats

    def clear_all(self):
        """清空所有死信"""
        self._dead_letters.clear()
        self._dead_letter_queue.clear()
        self._retry_queue.clear()
        self._retry_schedule.clear()

        logger.info("所有死信已清空")


if __name__ == "__main__":
    async def main():
        """测试死信队列功能"""
        dlq = DeadLetterQueue()
        await dlq.start()

        # 测试添加死信
        dead_letter_id = dlq.add_dead_letter(
            original_message_id="msg123",
            sender="sender1",
            receiver="receiver1",
            payload={"data": "test"},
            reason=DeadLetterReason.MAX_RETRIES_EXCEEDED,
            error_message="Maximum retries exceeded",
            retry_count=3
        )
        print(f"死信ID: {dead_letter_id}")

        # 设置重试动作
        dlq.set_dead_letter_action(dead_letter_id, DeadLetterAction.RETRY)

        # 获取死信
        dead_letter = dlq.get_dead_letter(dead_letter_id)
        print(f"死信信息: {dead_letter.reason.value}")

        # 统计信息
        stats = dlq.get_stats()
        print(f"死信队列统计: {stats}")

        # 等待处理
        await asyncio.sleep(2)

        # 关闭
        await dlq.stop()

    asyncio.run(main())
