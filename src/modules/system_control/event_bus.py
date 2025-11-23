"""
事件总线模块 - Story 4.2
实现事件驱动的发布订阅架构
支持事件监听、过滤、广播等功能
"""

import asyncio
import time
import uuid
from typing import Any, Callable, Dict, List, Optional, Set, Iterator, Union
from dataclasses import dataclass, field
from enum import Enum
import logging


logger = logging.getLogger(__name__)


class EventType(Enum):
    """事件类型枚举"""
    SYSTEM_START = "system_start"
    SYSTEM_STOP = "system_stop"
    MODULE_REGISTER = "module_register"
    MODULE_UNREGISTER = "module_unregister"
    MESSAGE_RECEIVED = "message_received"
    MESSAGE_SENT = "message_sent"
    ERROR_OCCURRED = "error_occurred"
    PERFORMANCE_ALERT = "performance_alert"
    HEALTH_CHECK = "health_check"
    CUSTOM = "custom"


@dataclass
class Event:
    """事件数据结构"""
    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    event_type: EventType = EventType.CUSTOM
    data: Any = None
    timestamp: float = field(default_factory=time.time)
    source: str = ""
    metadata: Dict[str, Any] = field(default_factory=dict)
    correlation_id: Optional[str] = None

    def __post_init__(self):
        """事件后处理"""
        if not self.source:
            self.source = "unknown"


@dataclass
class EventHandler:
    """事件处理器"""
    id: str
    handler_func: Callable
    event_types: Set[EventType]
    filter_func: Optional[Callable] = None
    is_active: bool = True
    created_at: float = field(default_factory=time.time)
    last_triggered: Optional[float] = None
    trigger_count: int = 0


class EventBus:
    """
    高性能事件总线
    支持事件发布、订阅、过滤、广播
    实现发布订阅模式，支持事件驱动架构
    """

    def __init__(self):
        # 事件处理器存储
        self._handlers: Dict[str, EventHandler] = {}
        self._event_subscribers: Dict[EventType, Set[str]] = {}

        # 事件历史（可选，用于调试和监控）
        self._event_history: List[Event] = []
        self._max_history_size = 1000

        # 统计数据
        self._stats = {
            'events_published': 0,
            'events_delivered': 0,
            'events_failed': 0,
            'active_handlers': 0,
            'average_delivery_time': 0.0
        }

        # 异步队列
        self._event_queue = asyncio.Queue(maxsize=10000)
        self._processing_task: Optional[asyncio.Task] = None
        self._running = False

        logger.info("事件总线初始化完成")

    async def start(self):
        """启动事件总线"""
        if not self._running:
            self._running = True
            self._processing_task = asyncio.create_task(self._process_events())
            logger.info("事件总线已启动")

    async def stop(self):
        """停止事件总线"""
        if self._running:
            self._running = False

            # 等待处理任务完成
            if self._processing_task:
                self._processing_task.cancel()
                try:
                    await self._processing_task
                except asyncio.CancelledError:
                    pass

            logger.info("事件总线已停止")

    def emit(self, event: Union[Event, EventType, str],
             data: Any = None,
             source: str = "",
             metadata: Dict[str, Any] = None) -> str:
        """
        发出事件

        Args:
            event: 事件对象、事件类型或事件类型字符串
            data: 事件数据
            source: 事件源
            metadata: 事件元数据

        Returns:
            str: 事件ID
        """
        # 创建事件对象
        if isinstance(event, Event):
            event_obj = event
        elif isinstance(event, EventType):
            event_obj = Event(
                event_type=event,
                data=data,
                source=source or "unknown",
                metadata=metadata or {}
            )
        elif isinstance(event, str):
            try:
                event_type = EventType(event)
            except ValueError:
                event_type = EventType.CUSTOM
            event_obj = Event(
                event_type=event_type,
                data=data,
                source=source or "unknown",
                metadata=metadata or {}
            )
        else:
            raise ValueError(f"不支持的事件类型: {type(event)}")

        # 添加到队列进行处理
        try:
            self._event_queue.put_nowait(event_obj)
            self._stats['events_published'] += 1

            logger.debug(
                "事件已发布",
                event_id=event_obj.id,
                event_type=event_obj.event_type.value,
                source=event_obj.source
            )

            return event_obj.id

        except asyncio.QueueFull:
            self._stats['events_failed'] += 1
            logger.error("事件队列已满，事件发布失败", event_type=event_obj.event_type.value)
            raise

    def on(self, event_type: Union[EventType, str],
           handler: Callable,
           filter_func: Optional[Callable] = None,
           source_filter: Optional[str] = None) -> str:
        """
        注册事件处理器

        Args:
            event_type: 事件类型
            handler: 处理函数
            filter_func: 事件过滤函数
            source_filter: 事件源过滤器

        Returns:
            str: 处理器ID
        """
        # 转换事件类型
        if isinstance(event_type, str):
            try:
                event_enum = EventType(event_type)
            except ValueError:
                event_enum = EventType.CUSTOM
        else:
            event_enum = event_type

        # 创建处理器
        handler_id = str(uuid.uuid4())

        # 应用源过滤器
        if source_filter:
            def _source_filter_wrapper(event: Event) -> bool:
                return event.source == source_filter

            if filter_func:
                original_filter = filter_func
                filter_func = lambda e: original_filter(e) and _source_filter_wrapper(e)
            else:
                filter_func = _source_filter_wrapper

        handler_obj = EventHandler(
            id=handler_id,
            handler_func=handler,
            event_types={event_enum},
            filter_func=filter_func
        )

        # 注册处理器
        self._handlers[handler_id] = handler_obj

        # 更新事件订阅索引
        if event_enum not in self._event_subscribers:
            self._event_subscribers[event_enum] = set()
        self._event_subscribers[event_enum].add(handler_id)

        # 更新统计
        self._stats['active_handlers'] = len(self._handlers)

        logger.info(
            "事件处理器已注册",
            handler_id=handler_id,
            event_type=event_enum.value,
            source_filter=source_filter
        )

        return handler_id

    def off(self, handler_id: str) -> bool:
        """
        注销事件处理器

        Args:
            handler_id: 处理器ID

        Returns:
            bool: 注销是否成功
        """
        if handler_id not in self._handlers:
            logger.warning("处理器ID不存在", handler_id=handler_id)
            return False

        handler = self._handlers[handler_id]

        # 从订阅索引中移除
        for event_type in handler.event_types:
            if event_type in self._event_subscribers:
                self._event_subscribers[event_type].discard(handler_id)
                # 清理空的事件类型
                if not self._event_subscribers[event_type]:
                    del self._event_subscribers[event_type]

        # 删除处理器
        del self._handlers[handler_id]

        # 更新统计
        self._stats['active_handlers'] = len(self._handlers)

        logger.info("事件处理器已注销", handler_id=handler_id)
        return True

    def filter(self, event_type: EventType,
               predicate: Callable[[Event], bool]) -> Iterator[Event]:
        """
        事件过滤迭代器

        Args:
            event_type: 事件类型
            predicate: 过滤谓词函数

        Returns:
            Iterator[Event]: 过滤后的事件迭代器
        """
        # 从历史中筛选匹配的事件
        for event in self._event_history:
            if event.event_type == event_type and predicate(event):
                yield event

    def get_event_history(self,
                         event_type: Optional[EventType] = None,
                         limit: int = 100) -> List[Event]:
        """
        获取事件历史

        Args:
            event_type: 事件类型过滤
            limit: 返回数量限制

        Returns:
            List[Event]: 事件列表
        """
        events = self._event_history

        if event_type:
            events = [e for e in events if e.event_type == event_type]

        return events[-limit:]

    def get_stats(self) -> Dict[str, Any]:
        """
        获取事件总线统计信息

        Returns:
            Dict: 统计信息
        """
        stats = self._stats.copy()
        stats['event_types_registered'] = list(self._event_subscribers.keys())
        stats['queue_size'] = self._event_queue.qsize()
        stats['history_size'] = len(self._event_history)
        return stats

    async def _process_events(self):
        """异步处理事件队列"""
        while self._running:
            try:
                # 从队列获取事件
                event = await asyncio.wait_for(self._event_queue.get(), timeout=0.1)

                start_time = time.perf_counter()

                # 添加到历史
                self._add_to_history(event)

                # 找到匹配的处理器
                matching_handlers = self._find_matching_handlers(event)

                # 并发执行处理器
                if matching_handlers:
                    tasks = []
                    for handler in matching_handlers:
                        if handler.is_active:
                            task = asyncio.create_task(self._execute_handler(handler, event))
                            tasks.append(task)

                    if tasks:
                        # 等待所有处理器完成
                        await asyncio.gather(*tasks, return_exceptions=True)

                # 更新延迟统计
                delivery_time = (time.perf_counter() - start_time) * 1000  # ms
                self._update_delivery_stats(delivery_time)
                self._stats['events_delivered'] += 1

            except asyncio.TimeoutError:
                # 超时正常，继续循环
                continue
            except Exception as e:
                self._stats['events_failed'] += 1
                logger.error("事件处理错误", error=str(e))

    def _find_matching_handlers(self, event: Event) -> List[EventHandler]:
        """查找匹配的事件处理器"""
        matching = []

        # 直接匹配的事件类型
        if event.event_type in self._event_subscribers:
            handler_ids = self._event_subscribers[event.event_type]
            for handler_id in handler_ids:
                handler = self._handlers.get(handler_id)
                if handler and handler.is_active:
                    # 检查过滤器
                    if not handler.filter_func or handler.filter_func(event):
                        matching.append(handler)

        # 检查通配符处理器（CUSTOM或其他）
        for handler in self._handlers.values():
            if (handler.is_active and
                EventType.CUSTOM in handler.event_types and
                handler not in matching):
                if not handler.filter_func or handler.filter_func(event):
                    matching.append(handler)

        return matching

    async def _execute_handler(self, handler: EventHandler, event: Event):
        """执行事件处理器"""
        try:
            # 记录触发时间
            handler.last_triggered = time.time()
            handler.trigger_count += 1

            # 同步处理器
            if asyncio.iscoroutinefunction(handler.handler_func):
                await handler.handler_func(event)
            else:
                handler.handler_func(event)

            logger.debug(
                "事件处理器执行成功",
                handler_id=handler.id,
                event_type=event.event_type.value,
                trigger_count=handler.trigger_count
            )

        except Exception as e:
            logger.error(
                "事件处理器执行失败",
                handler_id=handler.id,
                event_type=event.event_type.value,
                error=str(e)
            )
            self._stats['events_failed'] += 1

    def _add_to_history(self, event: Event):
        """添加事件到历史"""
        self._event_history.append(event)

        # 限制历史大小
        if len(self._event_history) > self._max_history_size:
            self._event_history.pop(0)

    def _update_delivery_stats(self, delivery_time_ms: float):
        """更新事件投递统计"""
        if self._stats['average_delivery_time'] == 0:
            self._stats['average_delivery_time'] = delivery_time_ms
        else:
            # 移动平均
            alpha = 0.1
            self._stats['average_delivery_time'] = (
                (1 - alpha) * self._stats['average_delivery_time'] +
                alpha * delivery_time_ms
            )

    def broadcast(self, events: List[Event],
                  parallel: bool = True) -> List[str]:
        """
        批量广播事件

        Args:
            events: 事件列表
            parallel: 是否并发执行

        Returns:
            List[str]: 事件ID列表
        """
        if parallel:
            # 并发广播
            tasks = []
            for event in events:
                event_id = self.emit(event)
                tasks.append(event_id)
            return tasks
        else:
            # 顺序广播
            return [self.emit(event) for event in events]

    def clear_handlers(self, event_type: Optional[EventType] = None):
        """
        清除处理器

        Args:
            event_type: 要清除的事件类型，None表示清除所有
        """
        if event_type is None:
            # 清除所有处理器
            self._handlers.clear()
            self._event_subscribers.clear()
            logger.info("所有事件处理器已清除")
        else:
            # 清除特定事件类型的处理器
            if event_type in self._event_subscribers:
                handler_ids = self._event_subscribers[event_type].copy()
                for handler_id in handler_ids:
                    self.off(handler_id)
                logger.info("事件类型处理器已清除", event_type=event_type.value)


# IEventBus接口实现
class IEventBus:
    """事件总线接口"""

    def __init__(self):
        self._implementation = EventBus()

    async def start(self):
        """启动事件总线"""
        await self._implementation.start()

    async def stop(self):
        """停止事件总线"""
        await self._implementation.stop()

    def emit(self, event_type: str, data: Any) -> str:
        """发出事件"""
        return self._implementation.emit(event_type, data)

    def on(self, event_type: str, handler: Callable) -> str:
        """注册事件处理器"""
        return self._implementation.on(event_type, handler)

    def off(self, handler_id: str) -> None:
        """注销事件处理器"""
        self._implementation.off(handler_id)

    def filter(self, event_type: EventType, predicate: Callable) -> Iterator:
        """事件过滤迭代器"""
        return self._implementation.filter(event_type, predicate)


if __name__ == "__main__":
    async def main():
        """测试事件总线功能"""
        event_bus = EventBus()
        await event_bus.start()

        # 测试处理器
        def test_handler(event: Event):
            print(f"收到事件: {event.event_type.value}, 数据: {event.data}")

        # 注册处理器
        handler_id = event_bus.on(EventType.SYSTEM_START, test_handler)
        print(f"处理器ID: {handler_id}")

        # 发出事件
        event_id = event_bus.emit(EventType.SYSTEM_START, "系统启动", "test_source")
        print(f"事件ID: {event_id}")

        # 等待处理
        await asyncio.sleep(0.1)

        # 统计信息
        stats = event_bus.get_stats()
        print(f"事件总线统计: {stats}")

        # 关闭
        await event_bus.stop()

    asyncio.run(main())
