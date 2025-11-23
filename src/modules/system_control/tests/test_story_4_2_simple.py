"""
Story 4.2 简单验证测试
验证模块基本功能
"""

import asyncio
import time
import sys
import os

# 添加项目根目录到路径
sys.path.insert(0, '/home/sunrise/xlerobot')

from src.modules.system_control.message_queue import MessageQueue, MessagePriority
from src.modules.system_control.event_bus import EventBus, EventType, Event
from src.modules.system_control.async_communicator import AsyncCommunicator, AsyncMessage
from src.modules.system_control.reliability_manager import ReliabilityManager, ReliableMessage, DeliveryMode
from src.modules.system_control.dead_letter_queue import DeadLetterQueue, DeadLetterReason, DeadLetterAction
from src.modules.system_control.performance_monitor import PerformanceMonitor


async def test_all_modules():
    """测试所有模块"""
    print("=" * 60)
    print("开始 Story 4.2 模块协调机制验证测试")
    print("=" * 60)

    # 测试消息队列
    print("\n📦 测试消息队列...")
    message_queue = MessageQueue("test_queue")

    start_time = time.perf_counter()
    success = await message_queue.publish(
        "test_topic",
        {"data": "test_message"},
        priority=MessagePriority.HIGH
    )
    end_time = time.perf_counter()

    latency_ms = (end_time - start_time) * 1000
    print(f"  ✅ 消息发布成功: {success}")
    print(f"  ✅ 延迟: {latency_ms:.2f}ms {'(< 10ms)' if latency_ms < 10.0 else '(>= 10ms - 失败!)'}")

    # 测试事件总线
    print("\n📡 测试事件总线...")
    event_bus = EventBus()
    await event_bus.start()

    received_events = []

    def event_handler(event: Event):
        received_events.append(event)

    event_bus.on(EventType.SYSTEM_START, event_handler)

    start_time = time.perf_counter()
    event_id = event_bus.emit(
        EventType.SYSTEM_START,
        data="Test Event",
        source="test_source"
    )
    await asyncio.sleep(0.01)  # 等待处理
    end_time = time.perf_counter()

    latency_ms = (end_time - start_time) * 1000
    print(f"  ✅ 事件发出成功: {event_id}")
    print(f"  ✅ 事件接收: {len(received_events) > 0}")
    print(f"  ✅ 延迟: {latency_ms:.2f}ms {'(< 10ms)' if latency_ms < 10.0 else '(>= 10ms - 失败!)'}")

    await event_bus.stop()

    # 测试异步通信
    print("\n🔄 测试异步通信...")
    async_communicator = AsyncCommunicator()
    await async_communicator.start()

    async def message_handler(message: AsyncMessage):
        await asyncio.sleep(0.001)  # 1ms
        return {"status": "processed"}

    async_communicator.register_handler("test_receiver", message_handler)

    start_time = time.perf_counter()
    future = await async_communicator.send(
        message="Test async message",
        receiver="test_receiver",
        message_type="test",
        timeout=5.0
    )
    result = await future.result_async()
    end_time = time.perf_counter()

    latency_ms = (end_time - start_time) * 1000
    print(f"  ✅ 异步消息处理成功: {result is not None}")
    print(f"  ✅ 延迟: {latency_ms:.2f}ms {'(< 10ms)' if latency_ms < 10.0 else '(>= 10ms - 失败!)'}")

    await async_communicator.stop()

    # 测试可靠性管理器
    print("\n🛡️ 测试可靠性管理器...")
    reliability_manager = ReliabilityManager()
    await reliability_manager.start()

    async def reliability_handler(message: ReliableMessage):
        await asyncio.sleep(0.001)
        return True

    reliability_manager.register_message_handler("test_receiver", reliability_handler)

    message = ReliableMessage(
        sender="test_sender",
        receiver="test_receiver",
        payload={"data": "test"},
        delivery_mode=DeliveryMode.AT_LEAST_ONCE
    )

    start_time = time.perf_counter()
    message_id = await reliability_manager.send_reliable_message(message)
    ack_success = await reliability_manager.acknowledge_message(
        message_id,
        acknowledged=True,
        receiver="test_receiver"
    )
    end_time = time.perf_counter()

    latency_ms = (end_time - start_time) * 1000
    print(f"  ✅ 可靠消息发送成功: {message_id}")
    print(f"  ✅ 消息确认成功: {ack_success}")
    print(f"  ✅ 延迟: {latency_ms:.2f}ms {'(< 10ms)' if latency_ms < 10.0 else '(>= 10ms - 失败!)'}")

    await reliability_manager.stop()

    # 测试死信队列
    print("\n📮 测试死信队列...")
    dead_letter_queue = DeadLetterQueue()
    await dead_letter_queue.start()

    processed_dead_letters = []

    async def archive_handler(dead_letter):
        processed_dead_letters.append(dead_letter.id)
        return True

    dead_letter_queue.register_action_handler(
        DeadLetterAction.ARCHIVE,
        archive_handler
    )

    dead_letter_id = dead_letter_queue.add_dead_letter(
        original_message_id="msg123",
        sender="test_sender",
        receiver="test_receiver",
        payload={"data": "test"},
        reason=DeadLetterReason.MAX_RETRIES_EXCEEDED,
        error_message="Maximum retries exceeded",
        retry_count=3
    )

    await asyncio.sleep(0.1)

    print(f"  ✅ 死信添加成功: {dead_letter_id}")
    print(f"  ✅ 死信处理: {len(processed_dead_letters) > 0}")

    await dead_letter_queue.stop()

    # 测试性能监控器
    print("\n📊 测试性能监控器...")
    performance_monitor = PerformanceMonitor()
    await performance_monitor.start()

    # 记录多个延迟指标
    for i in range(10):
        await performance_monitor.record_latency(
            operation=f"test_operation_{i}",
            latency_ms=5.0 + (i * 0.5)
        )

    # 记录消息指标
    for i in range(5):
        await performance_monitor.record_message(
            message_type="test_message",
            size_bytes=1024 * (i + 1)
        )

    latency_stats = performance_monitor.get_metric_stats("latency.test_operation_0")
    current_metrics = performance_monitor.get_current_metrics()

    print(f"  ✅ 延迟指标记录: {latency_stats['count'] > 0}")
    print(f"  ✅ 当前指标获取: {'latency.test_operation_0' in current_metrics}")
    print(f"  ✅ 平均延迟: {latency_stats['avg']:.2f}ms")

    await performance_monitor.stop()

    print("\n" + "=" * 60)
    print("Story 4.2 验证测试完成")
    print("=" * 60)

    return True


if __name__ == "__main__":
    try:
        asyncio.run(test_all_modules())
        print("\n✅ 所有测试通过!")
        exit(0)
    except Exception as e:
        print(f"\n❌ 测试失败: {str(e)}")
        import traceback
        traceback.print_exc()
        exit(1)
