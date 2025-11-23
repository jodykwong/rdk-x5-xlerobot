"""
Story 4.2 集成测试 - 模块协调机制
测试所有模块的协作和<10ms延迟要求
"""

import asyncio
import time
import pytest
from typing import List, Dict, Any

# 导入所有模块
from ..message_queue import MessageQueue, MessagePriority
from ..event_bus import EventBus, EventType, Event
from ..async_communicator import AsyncCommunicator, AsyncMessage, MessagePriority as AsyncMessagePriority
from ..reliability_manager import ReliabilityManager, ReliableMessage, DeliveryMode, MessageState
from ..dead_letter_queue import DeadLetterQueue, DeadLetterReason, DeadLetterAction
from ..performance_monitor import PerformanceMonitor, MetricType, AlertLevel


class TestStory42Integration:
    """Story 4.2集成测试类"""

    @pytest.fixture
    async def setup_modules(self):
        """设置所有模块"""
        # 创建消息队列
        message_queue = MessageQueue("test_queue")

        # 创建事件总线
        event_bus = EventBus()

        # 创建异步通信器
        async_communicator = AsyncCommunicator()

        # 创建可靠性管理器
        reliability_manager = ReliabilityManager()

        # 创建死信队列
        dead_letter_queue = DeadLetterQueue()

        # 创建性能监控器
        performance_monitor = PerformanceMonitor()

        # 启动所有模块
        await message_queue.start()
        await event_bus.start()
        await async_communicator.start()
        await reliability_manager.start()
        await dead_letter_queue.start()
        await performance_monitor.start()

        yield {
            'message_queue': message_queue,
            'event_bus': event_bus,
            'async_communicator': async_communicator,
            'reliability_manager': reliability_manager,
            'dead_letter_queue': dead_letter_queue,
            'performance_monitor': performance_monitor
        }

        # 关闭所有模块
        await message_queue.shutdown()
        await event_bus.stop()
        await async_communicator.stop()
        await reliability_manager.stop()
        await dead_letter_queue.stop()
        await performance_monitor.stop()

    @pytest.mark.asyncio
    async def test_message_queue_basic_functionality(self, setup_modules):
        """测试消息队列基本功能"""
        message_queue = setup_modules['message_queue']

        # 发布消息
        start_time = time.perf_counter()
        success = await message_queue.publish(
            "test_topic",
            {"data": "test_message"},
            priority=MessagePriority.HIGH
        )
        end_time = time.perf_counter()

        # 验证延迟 < 10ms
        latency_ms = (end_time - start_time) * 1000
        assert success, "消息发布失败"
        assert latency_ms < 10.0, f"消息队列延迟超标: {latency_ms:.2f}ms"

        # 获取队列深度
        depth = message_queue.get_queue_depth("test_topic")
        assert depth >= 0, "队列深度获取失败"

        print(f"✅ 消息队列基本功能测试通过，延迟: {latency_ms:.2f}ms")

    @pytest.mark.asyncio
    async def test_event_bus_functionality(self, setup_modules):
        """测试事件总线功能"""
        event_bus = setup_modules['event_bus']

        # 创建事件处理器
        received_events = []

        def event_handler(event: Event):
            received_events.append(event)

        # 注册事件处理器
        handler_id = event_bus.on(EventType.SYSTEM_START, event_handler)

        # 发出事件
        start_time = time.perf_counter()
        event_id = event_bus.emit(
            EventType.SYSTEM_START,
            data="Test Event",
            source="test_source"
        )
        end_time = time.perf_counter()

        # 等待事件处理
        await asyncio.sleep(0.01)  # 10ms

        # 验证延迟 < 10ms
        latency_ms = (end_time - start_time) * 1000
        assert len(received_events) > 0, "事件未收到"
        assert latency_ms < 10.0, f"事件总线延迟超标: {latency_ms:.2f}ms"

        # 注销处理器
        event_bus.off(handler_id)

        print(f"✅ 事件总线功能测试通过，延迟: {latency_ms:.2f}ms")

    @pytest.mark.asyncio
    async def test_async_communication(self, setup_modules):
        """测试异步通信"""
        async_communicator = setup_modules['async_communicator']

        # 注册消息处理器
        async def message_handler(message: AsyncMessage):
            await asyncio.sleep(0.001)  # 1ms处理时间
            return {"status": "processed"}

        async_communicator.register_handler("test_receiver", message_handler)

        # 发送异步消息
        start_time = time.perf_counter()
        future = await async_communicator.send(
            message="Test async message",
            receiver="test_receiver",
            message_type="test",
            priority=AsyncMessagePriority.HIGH,
            timeout=5.0
        )
        end_time = time.perf_counter()

        # 等待结果
        result = await future.result_async()

        # 验证延迟 < 10ms
        latency_ms = (end_time - start_time) * 1000
        assert result is not None, "异步消息处理失败"
        assert latency_ms < 10.0, f"异步通信延迟超标: {latency_ms:.2f}ms"

        print(f"✅ 异步通信测试通过，延迟: {latency_ms:.2f}ms")

    @pytest.mark.asyncio
    async def test_reliability_manager(self, setup_modules):
        """测试可靠性管理器"""
        reliability_manager = setup_modules['reliability_manager']

        # 注册消息处理器
        async def reliability_handler(message: ReliableMessage):
            # 模拟成功处理
            await asyncio.sleep(0.001)  # 1ms
            return True

        reliability_manager.register_message_handler("test_receiver", reliability_handler)

        # 发送可靠消息
        message = ReliableMessage(
            sender="test_sender",
            receiver="test_receiver",
            payload={"data": "test"},
            delivery_mode=DeliveryMode.AT_LEAST_ONCE
        )

        start_time = time.perf_counter()
        message_id = await reliability_manager.send_reliable_message(message)
        end_time = time.perf_counter()

        # 确认消息
        ack_success = await reliability_manager.acknowledge_message(
            message_id,
            acknowledged=True,
            receiver="test_receiver"
        )

        # 获取消息状态
        status = await reliability_manager.get_message_status(message_id)

        # 验证
        assert message_id, "可靠消息发送失败"
        assert ack_success, "消息确认失败"
        assert status['acknowledged'], "消息状态不正确"

        # 验证延迟 < 10ms
        latency_ms = (end_time - start_time) * 1000
        assert latency_ms < 10.0, f"可靠性管理延迟超标: {latency_ms:.2f}ms"

        print(f"✅ 可靠性管理器测试通过，延迟: {latency_ms:.2f}ms")

    @pytest.mark.asyncio
    async def test_dead_letter_queue(self, setup_modules):
        """测试死信队列"""
        dead_letter_queue = setup_modules['dead_letter_queue']

        # 注册处理器
        processed_dead_letters = []

        async def archive_handler(dead_letter):
            processed_dead_letters.append(dead_letter.id)
            return True

        dead_letter_queue.register_action_handler(
            DeadLetterAction.ARCHIVE,
            archive_handler
        )

        # 添加死信
        dead_letter_id = dead_letter_queue.add_dead_letter(
            original_message_id="msg123",
            sender="test_sender",
            receiver="test_receiver",
            payload={"data": "test"},
            reason=DeadLetterReason.MAX_RETRIES_EXCEEDED,
            error_message="Maximum retries exceeded",
            retry_count=3
        )

        # 设置处理动作
        success = dead_letter_queue.set_dead_letter_action(
            dead_letter_id,
            DeadLetterAction.ARCHIVE
        )

        # 等待处理
        await asyncio.sleep(0.1)

        # 验证
        assert success, "死信动作设置失败"
        assert dead_letter_id in dead_letter_queue._dead_letters, "死信未添加"
        assert len(processed_dead_letters) > 0, "死信未处理"

        print(f"✅ 死信队列测试通过")

    @pytest.mark.asyncio
    async def test_performance_monitor(self, setup_modules):
        """测试性能监控器"""
        performance_monitor = setup_modules['performance_monitor']

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

        # 验证指标记录
        latency_stats = performance_monitor.get_metric_stats("latency.test_operation_0")
        assert latency_stats['count'] > 0, "延迟指标记录失败"

        current_metrics = performance_monitor.get_current_metrics()
        assert "latency.test_operation_0" in current_metrics, "当前指标获取失败"

        print(f"✅ 性能监控器测试通过")

    @pytest.mark.asyncio
    async def test_end_to_end_message_flow(self, setup_modules):
        """端到端消息流测试"""
        modules = setup_modules

        # 注册消息处理器
        async def message_handler(message: AsyncMessage):
            # 记录性能指标
            await modules['performance_monitor'].record_latency(
                "message_processing",
                3.0  # 3ms处理时间
            )

            # 发出事件
            modules['event_bus'].emit(
                EventType.MESSAGE_RECEIVED,
                data={"message_id": message.id},
                source="test_handler"
            )

            return {"status": "processed"}

        modules['async_communicator'].register_handler("test_receiver", message_handler)

        # 发送消息并等待处理
        start_time = time.perf_counter()

        future = await modules['async_communicator'].send(
            message="End-to-end test message",
            receiver="test_receiver",
            message_type="e2e_test",
            priority=AsyncMessagePriority.HIGH
        )

        result = await future.result_async()

        end_time = time.perf_counter()

        # 验证端到端延迟 < 10ms
        e2e_latency_ms = (end_time - start_time) * 1000
        assert result is not None, "端到端消息流失败"
        assert e2e_latency_ms < 10.0, f"端到端延迟超标: {e2e_latency_ms:.2f}ms"

        # 验证性能指标
        performance_stats = modules['performance_monitor'].get_stats()
        assert performance_stats['total_metrics_collected'] > 0, "性能指标未记录"

        print(f"✅ 端到端消息流测试通过，延迟: {e2e_latency_ms:.2f}ms")

    @pytest.mark.asyncio
    async def test_high_throughput_message_processing(self, setup_modules):
        """高吞吐量消息处理测试"""
        modules = setup_modules

        # 注册快速消息处理器
        async def fast_handler(message: AsyncMessage):
            return {"status": "fast_processed"}

        modules['async_communicator'].register_handler("fast_receiver", fast_handler)

        # 发送大量消息
        message_count = 100
        start_time = time.perf_counter()

        futures = []
        for i in range(message_count):
            future = await modules['async_communicator'].send(
                message=f"Message {i}",
                receiver="fast_receiver",
                message_type="throughput_test"
            )
            futures.append(future)

        # 等待所有消息处理完成
        for future in futures:
            await future.result_async()

        end_time = time.perf_counter()

        # 计算吞吐量
        total_time = end_time - start_time
        throughput = message_count / total_time

        # 验证高吞吐量 (> 1000 msg/s)
        assert throughput > 1000, f"吞吐量不足: {throughput:.2f} msg/s"

        # 验证平均延迟 < 10ms
        average_latency_ms = (total_time * 1000) / message_count
        assert average_latency_ms < 10.0, f"平均延迟超标: {average_latency_ms:.2f}ms"

        print(f"✅ 高吞吐量测试通过: {throughput:.2f} msg/s, 平均延迟: {average_latency_ms:.2f}ms")

    @pytest.mark.asyncio
    async def test_system_stability_under_load(self, setup_modules):
        """系统稳定性测试（负载下）"""
        modules = setup_modules

        # 注册消息处理器
        processed_messages = []

        async def load_handler(message: AsyncMessage):
            processed_messages.append(message.id)
            await asyncio.sleep(0.0005)  # 0.5ms处理时间
            return {"status": "processed"}

        modules['async_communicator'].register_handler("load_receiver", load_handler)

        # 模拟高负载
        concurrent_senders = 5
        messages_per_sender = 50
        total_messages = concurrent_senders * messages_per_sender

        start_time = time.perf_counter()

        # 并发发送消息
        tasks = []
        for sender in range(concurrent_senders):
            task = asyncio.create_task(self._send_batch_messages(
                modules,
                sender,
                messages_per_sender
            ))
            tasks.append(task)

        # 等待所有发送完成
        await asyncio.gather(*tasks)

        # 等待所有消息处理完成
        while len(processed_messages) < total_messages:
            await asyncio.sleep(0.01)

        end_time = time.perf_counter()

        # 验证处理完成
        assert len(processed_messages) == total_messages, "消息处理不完整"

        # 验证系统稳定性（没有异常或崩溃）
        system_stats = modules['performance_monitor'].get_stats()
        assert system_stats['total_alerts_triggered'] == 0, "系统产生告警"

        # 验证性能
        total_time = end_time - start_time
        average_latency_ms = (total_time * 1000) / total_messages
        assert average_latency_ms < 10.0, f"负载下延迟超标: {average_latency_ms:.2f}ms"

        print(f"✅ 系统稳定性测试通过，处理 {total_messages} 条消息，"
              f"平均延迟: {average_latency_ms:.2f}ms")

    async def _send_batch_messages(self, modules, sender_id: int, count: int):
        """发送批量消息的辅助函数"""
        for i in range(count):
            future = await modules['async_communicator'].send(
                message=f"Sender {sender_id} Message {i}",
                receiver="load_receiver",
                message_type="load_test"
            )
            await future.result_async()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
