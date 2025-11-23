"""
消息队列模块 - Story 4.2
实现基于ROS2 Topic的高效消息队列系统
支持发布订阅模式，满足<10ms延迟要求
"""

import asyncio
import time
from typing import Any, Callable, Dict, List, Optional, Union
from dataclasses import dataclass, field
from enum import Enum
import logging


# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
    from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
    ROS2_AVAILABLE = True
except ImportError:
    # Mock classes for development
    class Node:
        pass
    QoSProfile = dict
    ROS2_AVAILABLE = False


logger = logging.getLogger(__name__)


class MessagePriority(Enum):
    """消息优先级枚举"""
    LOW = 0
    NORMAL = 1
    HIGH = 2
    CRITICAL = 3


@dataclass
class QueuedMessage:
    """队列消息数据结构"""
    id: str
    topic: str
    data: Any
    priority: MessagePriority = MessagePriority.NORMAL
    timestamp: float = field(default_factory=time.time)
    retry_count: int = 0
    metadata: Dict[str, Any] = field(default_factory=dict)


class MessageQueue(Node if ROS2_AVAILABLE else object):
    """
    高性能消息队列实现
    基于ROS2 Topic实现，满足<10ms延迟要求
    支持消息优先级、QoS策略、异步处理
    """

    def __init__(self, node_name: str = "message_queue"):
        if ROS2_AVAILABLE and rclpy.ok():
            super().__init__(node_name)
            self.callback_group = ReentrantCallbackGroup()
            self._qos_profile = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1000,
                reliability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                durability=QoSDurabilityPolicy.VOLATILE
            )
            self.node_name = node_name
        else:
            self.node_name = node_name
            self.callback_group = None
            self._qos_profile = None

        # 消息队列存储
        self._queues: Dict[str, List[QueuedMessage]] = {}
        self._subscriptions: Dict[str, Any] = {}
        self._publishers: Dict[str, Any] = {}

        # 统计信息
        self._stats = {
            'messages_published': 0,
            'messages_received': 0,
            'messages_failed': 0,
            'average_latency': 0.0,
            'peak_latency': 0.0
        }

        # 异步任务存储
        self._processing_tasks: Dict[str, asyncio.Task] = {}

        logger.info("消息队列初始化完成", node=self.node_name)

    async def publish(self, topic: str, message: Any,
                     qos: int = 10,
                     priority: MessagePriority = MessagePriority.NORMAL,
                     timeout: float = 5.0) -> bool:
        """
        发布消息到指定主题

        Args:
            topic: 主题名称
            message: 消息内容
            qos: 服务质量等级
            priority: 消息优先级
            timeout: 超时时间

        Returns:
            bool: 发布是否成功
        """
        start_time = time.perf_counter()

        try:
            # 生成消息ID
            msg_id = f"{topic}:{start_time:.6f}"

            # 创建队列消息
            queued_msg = QueuedMessage(
                id=msg_id,
                topic=topic,
                data=message,
                priority=priority,
                metadata={'qos': qos, 'timeout': timeout}
            )

            # 添加到队列
            if topic not in self._queues:
                self._queues[topic] = []

            self._queues[topic].append(queued_msg)

            # 按优先级排序
            self._queues[topic].sort(key=lambda m: m.priority.value, reverse=True)

            # 发布到ROS2 Topic
            if ROS2_AVAILABLE and rclpy.ok():
                if topic not in self._publishers:
                    callback_group = self.callback_group or ReentrantCallbackGroup()
                    self._publishers[topic] = self.create_publisher(
                        type(message), topic, qos, callback_group=callback_group
                    )

                self._publishers[topic].publish(message)
            else:
                # Mock发布
                logger.debug("Mock发布消息", topic=topic, message=message)

            # 更新统计
            self._stats['messages_published'] += 1
            latency = (time.perf_counter() - start_time) * 1000  # ms
            self._update_latency_stats(latency)

            logger.debug(
                "消息发布成功",
                topic=topic,
                msg_id=msg_id,
                priority=priority.name,
                latency_ms=latency
            )

            return True

        except Exception as e:
            self._stats['messages_failed'] += 1
            logger.error("消息发布失败", topic=topic, error=str(e))
            return False

    def subscribe(self, topic: str, callback: Callable,
                  qos: int = 10,
                  message_type: type = None) -> str:
        """
        订阅主题消息

        Args:
            topic: 主题名称
            callback: 消息处理回调函数
            qos: 服务质量等级
            message_type: 消息类型（用于ROS2）

        Returns:
            str: 订阅ID
        """
        subscription_id = f"{topic}:{id(callback)}"

        try:
            if ROS2_AVAILABLE and rclpy.ok():
                # ROS2订阅
                def _ros2_callback(msg):
                    start_time = time.perf_counter()
                    try:
                        callback(msg)
                        latency = (time.perf_counter() - start_time) * 1000
                        logger.debug(
                            "ROS2消息接收",
                            topic=topic,
                            callback=callback.__name__,
                            latency_ms=latency
                        )
                    except Exception as e:
                        logger.error("ROS2消息处理错误", topic=topic, error=str(e))

                callback_group = self.callback_group or ReentrantCallbackGroup()
                self._subscriptions[subscription_id] = self.create_subscription(
                    message_type or type(None),
                    topic,
                    _ros2_callback,
                    qos,
                    callback_group=callback_group
                )
            else:
                # Mock订阅
                self._subscriptions[subscription_id] = callback
                logger.info("Mock订阅创建", topic=topic, subscription_id=subscription_id)

            logger.info("主题订阅成功", topic=topic, subscription_id=subscription_id)
            return subscription_id

        except Exception as e:
            logger.error("主题订阅失败", topic=topic, error=str(e))
            raise

    def unsubscribe(self, subscription_id: str) -> bool:
        """
        取消订阅

        Args:
            subscription_id: 订阅ID

        Returns:
            bool: 取消订阅是否成功
        """
        try:
            if subscription_id in self._subscriptions:
                if ROS2_AVAILABLE and hasattr(self._subscriptions[subscription_id], 'destroy'):
                    self._subscriptions[subscription_id].destroy()
                del self._subscriptions[subscription_id]

                logger.info("取消订阅成功", subscription_id=subscription_id)
                return True
            else:
                logger.warning("订阅ID不存在", subscription_id=subscription_id)
                return False

        except Exception as e:
            logger.error("取消订阅失败", subscription_id=subscription_id, error=str(e))
            return False

    def get_queue_depth(self, topic: str) -> int:
        """
        获取队列深度

        Args:
            topic: 主题名称

        Returns:
            int: 队列中待处理消息数量
        """
        return len(self._queues.get(topic, []))

    def get_queue_stats(self) -> Dict[str, Any]:
        """
        获取队列统计信息

        Returns:
            Dict: 统计信息
        """
        stats = self._stats.copy()
        stats['active_topics'] = list(self._queues.keys())
        stats['total_queue_depth'] = sum(len(q) for q in self._queues.values())
        stats['subscriptions_count'] = len(self._subscriptions)
        stats['publishers_count'] = len(self._publishers)
        return stats

    def _update_latency_stats(self, latency_ms: float):
        """更新延迟统计信息"""
        if self._stats['average_latency'] == 0:
            self._stats['average_latency'] = latency_ms
        else:
            # 移动平均
            alpha = 0.1
            self._stats['average_latency'] = (1 - alpha) * self._stats['average_latency'] + alpha * latency_ms

        # 更新峰值延迟
        if latency_ms > self._stats['peak_latency']:
            self._stats['peak_latency'] = latency_ms

    async def start_processing(self, topics: List[str] = None):
        """
        开始处理消息队列

        Args:
            topics: 要处理的主题列表，None表示处理所有主题
        """
        target_topics = topics or list(self._queues.keys())

        for topic in target_topics:
            if topic not in self._processing_tasks:
                task = asyncio.create_task(self._process_queue(topic))
                self._processing_tasks[topic] = task
                logger.info("开始处理队列", topic=topic)

    async def _process_queue(self, topic: str):
        """处理队列中的消息"""
        while True:
            try:
                if topic in self._queues and self._queues[topic]:
                    msg = self._queues[topic].pop(0)

                    # 处理消息
                    start_time = time.perf_counter()
                    # 这里可以添加消息处理逻辑
                    processing_time = time.perf_counter() - start_time

                    logger.debug(
                        "处理队列消息",
                        topic=topic,
                        msg_id=msg.id,
                        processing_time_ms=processing_time * 1000
                    )

                # 短暂休眠避免忙等
                await asyncio.sleep(0.001)  # 1ms

            except Exception as e:
                logger.error("队列处理错误", topic=topic, error=str(e))
                await asyncio.sleep(0.1)

    async def shutdown(self):
        """优雅关闭消息队列"""
        logger.info("开始关闭消息队列")

        # 取消所有处理任务
        for task in self._processing_tasks.values():
            task.cancel()

        if self._processing_tasks:
            await asyncio.gather(*self._processing_tasks.values(), return_exceptions=True)

        # 清理ROS2资源
        if ROS2_AVAILABLE:
            for pub in self._publishers.values():
                if hasattr(pub, 'destroy'):
                    pub.destroy()
            for sub in self._subscriptions.values():
                if hasattr(sub, 'destroy'):
                    sub.destroy()

        logger.info("消息队列已关闭")


# IMessageQueue接口实现
class IMessageQueue:
    """消息队列接口"""

    def __init__(self):
        self._implementation = MessageQueue()

    async def publish(self, topic: str, message: Any, qos: int = 10) -> bool:
        """发布消息"""
        return await self._implementation.publish(topic, message, qos)

    def subscribe(self, topic: str, callback: Callable, qos: int = 10) -> str:
        """订阅消息"""
        return self._implementation.subscribe(topic, callback, qos)

    def unsubscribe(self, subscription_id: str) -> bool:
        """取消订阅"""
        return self._implementation.unsubscribe(subscription_id)

    def get_queue_depth(self, topic: str) -> int:
        """获取队列深度"""
        return self._implementation.get_queue_depth(topic)


if __name__ == "__main__":
    async def main():
        """测试消息队列功能"""
        queue = MessageQueue("test_queue")

        # 发布测试消息
        await queue.publish("test_topic", "Hello World", priority=MessagePriority.NORMAL)

        # 订阅测试
        def test_callback(msg):
            print(f"收到消息: {msg}")

        sub_id = queue.subscribe("test_topic", test_callback)
        print(f"订阅ID: {sub_id}")

        # 统计信息
        stats = queue.get_queue_stats()
        print(f"队列统计: {stats}")

        # 启动处理
        await queue.start_processing(["test_topic"])

        # 等待处理
        await asyncio.sleep(1)

        # 关闭
        await queue.shutdown()

    asyncio.run(main())
