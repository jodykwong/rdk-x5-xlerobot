"""
异步通信模块 - Story 4.2
实现高效的异步通信机制
支持Promise/Future模式、消息确认机制、事务性消息
"""

import asyncio
import time
import uuid
from typing import Any, Dict, List, Optional, Callable, Union, Awaitable
from dataclasses import dataclass, field
from enum import Enum
from collections import defaultdict
import logging


logger = logging.getLogger(__name__)


class MessagePriority(Enum):
    """消息优先级"""
    LOW = 0
    NORMAL = 1
    HIGH = 2
    CRITICAL = 3


class TransactionStatus(Enum):
    """事务状态"""
    PENDING = "pending"
    IN_PROGRESS = "in_progress"
    COMMITTED = "committed"
    ABORTED = "aborted"
    TIMEOUT = "timeout"


@dataclass
class AsyncMessage:
    """异步消息"""
    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    sender: str = ""
    receiver: str = ""
    message_type: str = "generic"
    payload: Any = None
    priority: MessagePriority = MessagePriority.NORMAL
    timestamp: float = field(default_factory=time.time)
    timeout: float = 30.0
    correlation_id: Optional[str] = None
    reply_to: Optional[str] = None
    metadata: Dict[str, Any] = field(default_factory=dict)
    retry_count: int = 0
    max_retries: int = 3


@dataclass
class MessageAcknowledgement:
    """消息确认"""
    message_id: str
    acknowledged: bool
    acknowledged_at: float = field(default_factory=time.time)
    error: Optional[str] = None


@dataclass
class Transaction:
    """事务对象"""
    transaction_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    status: TransactionStatus = TransactionStatus.PENDING
    messages: List[AsyncMessage] = field(default_factory=list)
    created_at: float = field(default_factory=time.time)
    timeout: float = 60.0
    result: Any = None
    error: Optional[str] = None


class Future:
    """
    Future/Promise模式实现
    用于异步操作的结果处理
    """

    def __init__(self):
        self._loop = asyncio.get_event_loop()
        self._result = None
        self._exception = None
        self._done = False
        self._waiters: List[asyncio.Future] = []
        self._callbacks: List[Callable] = []

    def set_result(self, result):
        """设置结果"""
        if self._done:
            raise RuntimeError("Future already done")
        self._result = result
        self._done = True
        self._notify_waiters()
        self._run_callbacks()

    def set_exception(self, exception):
        """设置异常"""
        if self._done:
            raise RuntimeError("Future already done")
        self._exception = exception
        self._done = True
        self._notify_waiters()
        self._run_callbacks()

    def result(self) -> Any:
        """获取结果（阻塞）"""
        if self._done:
            if self._exception:
                raise self._exception
            return self._result
        else:
            # 在同步上下文中获取结果
            fut = self._loop.create_future()
            self._waiters.append(fut)
            return self._loop.run_until_complete(fut)

    async def result_async(self) -> Any:
        """异步获取结果"""
        if self._done:
            if self._exception:
                raise self._exception
            return self._result
        else:
            self._waiters.append(self._loop.create_future())
            return await self._waiters[-1]

    def done(self) -> bool:
        """检查是否完成"""
        return self._done

    def add_done_callback(self, callback: Callable):
        """添加完成回调"""
        if self._done:
            callback(self)
        else:
            self._callbacks.append(callback)

    def _notify_waiters(self):
        """通知等待者"""
        for waiter in self._waiters:
            if not waiter.done():
                if self._exception:
                    waiter.set_exception(self._exception)
                else:
                    waiter.set_result(self._result)
        self._waiters.clear()

    def _run_callbacks(self):
        """运行回调"""
        for callback in self._callbacks:
            try:
                callback(self)
            except Exception as e:
                logger.error("Future回调执行失败", error=str(e))
        self._callbacks.clear()


class AsyncCommunicator:
    """
    异步通信器
    实现高效的异步消息传递、Promise/Future模式、消息确认和事务性消息
    """

    def __init__(self):
        # 消息存储
        self._pending_messages: Dict[str, AsyncMessage] = {}
        self._message_handlers: Dict[str, Callable] = {}
        self._message_queues: Dict[str, List[AsyncMessage]] = defaultdict(list)

        # Future存储
        self._futures: Dict[str, Future] = {}

        # 消息确认
        self._acknowledgements: Dict[str, MessageAcknowledgement] = {}

        # 事务管理
        self._transactions: Dict[str, Transaction] = {}

        # 统计信息
        self._stats = {
            'messages_sent': 0,
            'messages_received': 0,
            'messages_acknowledged': 0,
            'messages_failed': 0,
            'transactions_created': 0,
            'transactions_committed': 0,
            'average_latency': 0.0
        }

        # 任务存储
        self._tasks: List[asyncio.Task] = []
        self._running = False

        logger.info("异步通信器初始化完成")

    async def start(self):
        """启动异步通信器"""
        if not self._running:
            self._running = True

            # 启动后台任务
            self._tasks.append(asyncio.create_task(self._message_processor()))
            self._tasks.append(asyncio.create_task(self._timeout_monitor()))

            logger.info("异步通信器已启动")

    async def stop(self):
        """停止异步通信器"""
        if self._running:
            self._running = False

            # 取消所有任务
            for task in self._tasks:
                task.cancel()

            # 等待任务完成
            if self._tasks:
                await asyncio.gather(*self._tasks, return_exceptions=True)

            logger.info("异步通信器已停止")

    async def send(self, message: Union[AsyncMessage, dict, str],
                   receiver: str = "",
                   payload: Any = None,
                   message_type: str = "generic",
                   priority: MessagePriority = MessagePriority.NORMAL,
                   timeout: float = 30.0,
                   require_acknowledgement: bool = True) -> Future:
        """
        发送异步消息

        Args:
            message: 消息对象、消息字符串或消息字典
            receiver: 接收者
            payload: 消息载荷
            message_type: 消息类型
            priority: 消息优先级
            timeout: 超时时间
            require_acknowledgement: 是否需要确认

        Returns:
            Future: 异步结果
        """
        try:
            # 创建消息对象
            if isinstance(message, AsyncMessage):
                msg = message
            elif isinstance(message, str):
                msg = AsyncMessage(
                    sender="",
                    receiver=receiver,
                    message_type=message_type,
                    payload=message if payload is None else payload,
                    priority=priority,
                    timeout=timeout
                )
            elif isinstance(message, dict):
                msg = AsyncMessage(**message)
            else:
                msg = AsyncMessage(
                    sender="",
                    receiver=receiver,
                    message_type=message_type,
                    payload=message,
                    priority=priority,
                    timeout=timeout
                )

            # 创建Future
            future = Future()

            # 添加到待处理消息
            self._pending_messages[msg.id] = msg

            # 查找消息处理器
            handler = self._message_handlers.get(msg.receiver)
            if handler:
                try:
                    # 异步或同步处理器
                    if asyncio.iscoroutinefunction(handler):
                        result = await handler(msg)
                    else:
                        result = await asyncio.get_event_loop().run_in_executor(
                            None, handler, msg
                        )
                    future.set_result(result)

                    # 更新统计
                    self._stats['messages_sent'] += 1
                    self._stats['messages_received'] += 1

                except Exception as e:
                    future.set_exception(e)
                    self._stats['messages_failed'] += 1
                    logger.error("消息处理失败", message_id=msg.id, error=str(e))
            else:
                # 放入消息队列
                self._message_queues[msg.receiver].append(msg)
                future.set_result({"status": "queued", "message_id": msg.id})

                # 更新统计
                self._stats['messages_sent'] += 1

            logger.debug(
                "异步消息发送成功",
                message_id=msg.id,
                receiver=receiver,
                message_type=message_type
            )

            return future

        except Exception as e:
            logger.error("异步消息发送失败", error=str(e))
            future = Future()
            future.set_exception(e)
            return future

    async def receive(self, receiver: str, timeout: Optional[float] = None) -> Optional[AsyncMessage]:
        """
        接收异步消息

        Args:
            receiver: 接收者
            timeout: 超时时间

        Returns:
            Optional[AsyncMessage]: 接收到的消息
        """
        try:
            start_time = time.time()

            while True:
                # 检查消息队列
                if self._message_queues[receiver]:
                    msg = self._message_queues[receiver].pop(0)
                    self._stats['messages_received'] += 1
                    return msg

                # 检查超时
                if timeout and (time.time() - start_time) > timeout:
                    logger.debug("消息接收超时", receiver=receiver, timeout=timeout)
                    return None

                # 短暂休眠
                await asyncio.sleep(0.001)  # 1ms

        except Exception as e:
            logger.error("消息接收失败", receiver=receiver, error=str(e))
            return None

    def register_handler(self, receiver: str, handler: Callable):
        """
        注册消息处理器

        Args:
            receiver: 接收者
            handler: 处理函数
        """
        self._message_handlers[receiver] = handler
        logger.info("消息处理器已注册", receiver=receiver)

    def acknowledge(self, message_id: str, acknowledged: bool = True, error: Optional[str] = None) -> bool:
        """
        确认消息

        Args:
            message_id: 消息ID
            acknowledged: 是否确认成功
            error: 错误信息

        Returns:
            bool: 确认是否成功
        """
        try:
            ack = MessageAcknowledgement(
                message_id=message_id,
                acknowledged=acknowledged,
                error=error
            )

            self._acknowledgements[message_id] = ack

            if acknowledged:
                # 移除待处理消息
                if message_id in self._pending_messages:
                    del self._pending_messages[message_id]

                self._stats['messages_acknowledged'] += 1

                logger.debug("消息确认成功", message_id=message_id)
            else:
                logger.warning("消息确认失败", message_id=message_id, error=error)

            return True

        except Exception as e:
            logger.error("消息确认失败", message_id=message_id, error=str(e))
            return False

    async def send_with_confirmation(self, message: AsyncMessage,
                                    timeout: float = 5.0) -> bool:
        """
        发送消息并等待确认

        Args:
            message: 消息对象
            timeout: 确认超时时间

        Returns:
            bool: 是否确认成功
        """
        try:
            # 发送消息
            future = await self.send(message, require_acknowledgement=True)

            # 等待确认
            start_time = time.time()
            while time.time() - start_time < timeout:
                if message.id in self._acknowledgements:
                    ack = self._acknowledgements[message.id]
                    return ack.acknowledged

                await asyncio.sleep(0.01)  # 10ms

            logger.warning("消息确认超时", message_id=message.id, timeout=timeout)
            return False

        except Exception as e:
            logger.error("发送消息并确认失败", message_id=message.id, error=str(e))
            return False

    async def create_transaction(self, messages: List[AsyncMessage],
                                timeout: float = 60.0) -> str:
        """
        创建事务

        Args:
            messages: 事务中的消息列表
            timeout: 事务超时时间

        Returns:
            str: 事务ID
        """
        try:
            transaction_id = str(uuid.uuid4())

            transaction = Transaction(
                transaction_id=transaction_id,
                messages=messages,
                timeout=timeout
            )

            self._transactions[transaction_id] = transaction
            self._stats['transactions_created'] += 1

            logger.info(
                "事务已创建",
                transaction_id=transaction_id,
                message_count=len(messages),
                timeout=timeout
            )

            return transaction_id

        except Exception as e:
            logger.error("创建事务失败", error=str(e))
            raise

    async def commit_transaction(self, transaction_id: str) -> bool:
        """
        提交事务

        Args:
            transaction_id: 事务ID

        Returns:
            bool: 是否提交成功
        """
        try:
            if transaction_id not in self._transactions:
                logger.error("事务不存在", transaction_id=transaction_id)
                return False

            transaction = self._transactions[transaction_id]

            # 发送事务中的所有消息
            success_count = 0
            for message in transaction.messages:
                success = await self.send_with_confirmation(message)
                if success:
                    success_count += 1
                else:
                    logger.error("事务消息发送失败", message_id=message.id)

            # 检查是否所有消息都成功
            if success_count == len(transaction.messages):
                transaction.status = TransactionStatus.COMMITTED
                transaction.result = {"status": "committed", "messages": success_count}
                self._stats['transactions_committed'] += 1

                logger.info(
                    "事务提交成功",
                    transaction_id=transaction_id,
                    messages_count=len(transaction.messages)
                )

                return True
            else:
                transaction.status = TransactionStatus.ABORTED
                transaction.error = f"部分消息发送失败: {success_count}/{len(transaction.messages)}"

                logger.error(
                    "事务提交失败",
                    transaction_id=transaction_id,
                    success_count=success_count,
                    total_count=len(transaction.messages)
                )

                return False

        except Exception as e:
            logger.error("提交事务失败", transaction_id=transaction_id, error=str(e))
            return False

    async def abort_transaction(self, transaction_id: str) -> bool:
        """
        中止事务

        Args:
            transaction_id: 事务ID

        Returns:
            bool: 是否中止成功
        """
        try:
            if transaction_id not in self._transactions:
                logger.error("事务不存在", transaction_id=transaction_id)
                return False

            transaction = self._transactions[transaction_id]
            transaction.status = TransactionStatus.ABORTED
            transaction.error = "事务被中止"

            logger.info("事务已中止", transaction_id=transaction_id)
            return True

        except Exception as e:
            logger.error("中止事务失败", transaction_id=transaction_id, error=str(e))
            return False

    async def _message_processor(self):
        """消息处理器任务"""
        while self._running:
            try:
                processed = False

                # 处理每个接收者的队列
                for receiver, queue in self._message_queues.items():
                    if queue:
                        handler = self._message_handlers.get(receiver)
                        if handler:
                            msg = queue.pop(0)
                            try:
                                if asyncio.iscoroutinefunction(handler):
                                    await handler(msg)
                                else:
                                    await asyncio.get_event_loop().run_in_executor(
                                        None, handler, msg
                                    )
                                processed = True
                                self._stats['messages_received'] += 1
                            except Exception as e:
                                logger.error("消息处理错误", message_id=msg.id, error=str(e))
                                self._stats['messages_failed'] += 1

                if not processed:
                    await asyncio.sleep(0.001)  # 1ms

            except Exception as e:
                logger.error("消息处理器错误", error=str(e))
                await asyncio.sleep(0.1)

    async def _timeout_monitor(self):
        """超时监控任务"""
        while self._running:
            try:
                current_time = time.time()

                # 检查消息超时
                expired_messages = []
                for msg_id, msg in self._pending_messages.items():
                    if current_time - msg.timestamp > msg.timeout:
                        expired_messages.append(msg_id)

                for msg_id in expired_messages:
                    msg = self._pending_messages[msg_id]
                    logger.warning("消息超时", message_id=msg_id, timeout=msg.timeout)
                    del self._pending_messages[msg_id]
                    self._stats['messages_failed'] += 1

                # 检查事务超时
                expired_transactions = []
                for trans_id, transaction in self._transactions.items():
                    if current_time - transaction.created_at > transaction.timeout:
                        if transaction.status == TransactionStatus.PENDING:
                            transaction.status = TransactionStatus.TIMEOUT
                            expired_transactions.append(trans_id)

                for trans_id in expired_transactions:
                    logger.warning("事务超时", transaction_id=trans_id)

                await asyncio.sleep(1.0)  # 每秒检查一次

            except Exception as e:
                logger.error("超时监控错误", error=str(e))
                await asyncio.sleep(1)

    def get_stats(self) -> Dict[str, Any]:
        """
        获取通信器统计信息

        Returns:
            Dict: 统计信息
        """
        stats = self._stats.copy()
        stats['pending_messages'] = len(self._pending_messages)
        stats['active_handlers'] = len(self._message_handlers)
        stats['queue_sizes'] = {k: len(v) for k, v in self._message_queues.items()}
        stats['active_transactions'] = sum(
            1 for t in self._transactions.values()
            if t.status in [TransactionStatus.PENDING, TransactionStatus.IN_PROGRESS]
        )
        return stats

    def clear_queues(self):
        """清空所有队列"""
        self._message_queues.clear()
        self._pending_messages.clear()
        logger.info("所有队列已清空")


if __name__ == "__main__":
    async def main():
        """测试异步通信器功能"""
        communicator = AsyncCommunicator()
        await communicator.start()

        # 测试消息处理器
        async def test_handler(message: AsyncMessage):
            print(f"处理消息: {message.message_type}, 载荷: {message.payload}")
            await asyncio.sleep(0.001)  # 模拟处理时间
            return {"status": "processed", "message_id": message.id}

        communicator.register_handler("test_receiver", test_handler)

        # 测试发送消息
        message = AsyncMessage(
            sender="test_sender",
            receiver="test_receiver",
            message_type="test",
            payload={"data": "test_data"}
        )

        future = await communicator.send(message)
        result = await future.result_async()
        print(f"消息发送结果: {result}")

        # 测试消息确认
        ack_success = communicator.acknowledge(message.id, True)
        print(f"消息确认: {ack_success}")

        # 测试事务
        messages = [
            AsyncMessage(sender="test", receiver="test", payload={"msg": 1}),
            AsyncMessage(sender="test", receiver="test", payload={"msg": 2})
        ]

        trans_id = await communicator.create_transaction(messages)
        print(f"事务ID: {trans_id}")

        success = await communicator.commit_transaction(trans_id)
        print(f"事务提交: {success}")

        # 统计信息
        stats = communicator.get_stats()
        print(f"通信器统计: {stats}")

        # 关闭
        await communicator.stop()

    asyncio.run(main())
