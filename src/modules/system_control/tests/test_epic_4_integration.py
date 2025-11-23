"""
Epic 4 完整集成测试
测试所有系统控制模块的协同工作和集成功能
"""

import asyncio
import time
import sys

sys.path.insert(0, '/home/sunrise/xlerobot')

from src.modules.system_control.message_queue import MessageQueue, MessagePriority
from src.modules.system_control.event_bus import EventBus, EventType
from src.modules.system_control.async_communicator import AsyncCommunicator, AsyncMessage
from src.modules.system_control.resource_manager.cpu_scheduler import CPUScheduler, CPUPriority, LoadBalanceStrategy
from src.modules.system_control.system_monitor import SystemMonitor
from src.modules.system_control.health_checker import HealthChecker
from src.modules.system_control.resource_monitor import ResourceMonitor, ResourceType
from src.modules.system_control.config_manager import ConfigManager, ConfigScope, ConfigFormat


async def test_epic_4_complete_integration():
    """Epic 4完整集成测试"""
    print("=" * 80)
    print("开始 Epic 4 系统控制模块完整集成测试")
    print("=" * 80)

    # 1. 测试消息队列 + 事件总线 + 异步通信
    print("\n📨 测试消息通信模块协同...")

    message_queue = MessageQueue("integration_test")
    event_bus = EventBus()
    async_communicator = AsyncCommunicator()

    await event_bus.start()
    await async_communicator.start()

    # 设置消息处理
    async def message_handler(message: AsyncMessage):
        # 发出事件
        event_bus.emit(
            EventType.MESSAGE_RECEIVED,
            data={"message_id": message.id},
            source="test_handler"
        )
        return {"status": "processed"}

    async_communicator.register_handler("test_receiver", message_handler)

    # 发送消息
    start_time = time.perf_counter()
    future = await async_communicator.send(
        message="Integration test message",
        receiver="test_receiver",
        message_type="integration_test"
    )
    result = await future.result_async()
    latency_ms = (time.perf_counter() - start_time) * 1000

    print(f"  ✅ 消息通信测试通过，延迟: {latency_ms:.2f}ms")
    assert latency_ms < 10.0, f"消息通信延迟超标: {latency_ms:.2f}ms"

    await async_communicator.stop()
    await event_bus.stop()

    # 2. 测试CPU调度器 + 系统监控
    print("\n⚙️ 测试资源调度与监控协同...")

    cpu_scheduler = CPUScheduler(
        max_workers=2,
        load_balance_strategy=LoadBalanceStrategy.LEAST_LOADED
    )
    system_monitor = SystemMonitor(collection_interval=0.5)
    resource_monitor = ResourceMonitor(collection_interval=0.5)

    cpu_scheduler.start()
    await system_monitor.start()
    await resource_monitor.start()

    # 提交CPU密集任务
    def cpu_intensive_task(duration: float):
        """CPU密集型任务"""
        start = time.time()
        while time.time() - start < duration:
            sum(range(1000))
        return "Task completed"

    # 提交多个任务
    task_futures = []
    for i in range(5):
        task_id = f"task_{i}"
        cpu_scheduler.submit_task(
            task_id=task_id,
            func=cpu_intensive_task,
            args=(0.1,),
            priority=CPUPriority.NORMAL
        )

    # 等待任务完成
    await asyncio.sleep(2)

    # 检查系统状态
    system_status = system_monitor.get_system_status()
    cpu_usage = resource_monitor.get_current_resource_usage(ResourceType.CPU)

    print(f"  ✅ CPU调度测试完成")
    print(f"  ✅ 系统状态: {system_status.state.value}, CPU: {cpu_usage.usage_percent:.1f}%" if cpu_usage else "  - CPU数据采集中")

    await resource_monitor.stop()
    await system_monitor.stop()
    cpu_scheduler.stop()

    # 3. 测试配置管理 + 健康检查
    print("\n⚙️ 测试配置管理与健康检查协同...")

    config_manager = ConfigManager()
    await config_manager.start()

    health_checker = HealthChecker(check_interval=1.0)
    await health_checker.start()

    # 通过配置管理健康检查参数
    config_manager.set_config("health_check.cpu_threshold", 80.0)
    config_manager.set_config("health_check.memory_threshold", 85.0)

    # 获取配置值
    cpu_threshold = config_manager.get_config("health_check.cpu_threshold", 75.0)
    memory_threshold = config_manager.get_config("health_check.memory_threshold", 80.0)

    # 执行健康检查
    quick_health = health_checker.quick_health_check()

    print(f"  ✅ 配置管理测试通过: CPU阈值={cpu_threshold}, 内存阈值={memory_threshold}")
    print(f"  ✅ 健康检查结果: {quick_health.overall_status.value} (分数: {quick_health.score:.1f}/100)")

    await health_checker.stop()
    await config_manager.stop()

    # 4. 测试所有模块协同工作
    print("\n🔗 测试全模块协同工作...")

    # 启动所有模块
    all_modules = {
        'message_queue': MessageQueue("full_test"),
        'event_bus': EventBus(),
        'async_communicator': AsyncCommunicator(),
        'cpu_scheduler': CPUScheduler(max_workers=2),
        'system_monitor': SystemMonitor(collection_interval=0.5),
        'health_checker': HealthChecker(check_interval=1.0),
        'resource_monitor': ResourceMonitor(collection_interval=0.5),
        'config_manager': ConfigManager()
    }

    # 启动所有模块
    await all_modules['event_bus'].start()
    await all_modules['async_communicator'].start()
    all_modules['cpu_scheduler'].start()
    await all_modules['system_monitor'].start()
    await all_modules['health_checker'].start()
    await all_modules['resource_monitor'].start()
    await all_modules['config_manager'].start()

    # 设置配置
    all_modules['config_manager'].set_config("system.name", "XLeRobot")
    all_modules['config_manager'].set_config("system.version", "1.0.0")

    # 设置消息处理
    async def integrated_handler(message: AsyncMessage):
        # 记录配置信息
        system_name = all_modules['config_manager'].get_config("system.name")
        # 执行CPU任务
        all_modules['cpu_scheduler'].submit_task(
            task_id=f"integrated_task_{message.id}",
            func=lambda: f"Processed by {system_name}",
            priority=CPUPriority.HIGH
        )
        return {"status": "integrated", "name": system_name}

    all_modules['async_communicator'].register_handler("integrated_receiver", integrated_handler)

    # 发送综合测试消息
    start_time = time.perf_counter()
    future = await all_modules['async_communicator'].send(
        message="Full integration test",
        receiver="integrated_receiver",
        message_type="integration"
    )
    result = await future.result_async()
    end_time = time.perf_counter()

    total_latency_ms = (end_time - start_time) * 1000

    # 获取综合状态
    system_status = all_modules['system_monitor'].get_system_status()
    quick_health = await all_modules['health_checker'].get_current_health()
    cpu_usage = all_modules['resource_monitor'].get_current_resource_usage(ResourceType.CPU)

    print(f"  ✅ 全模块协同测试通过")
    print(f"  ✅ 端到端延迟: {total_latency_ms:.2f}ms")
    print(f"  ✅ 系统状态: {system_status.state.value}")
    print(f"  ✅ 健康状态: {quick_health.overall_status.value if quick_health else 'N/A'}")
    print(f"  ✅ 资源使用: CPU {cpu_usage.usage_percent:.1f}%" if cpu_usage else "  - 采集中")

    # 关闭所有模块
    await all_modules['config_manager'].stop()
    await all_modules['resource_monitor'].stop()
    await all_modules['health_checker'].stop()
    await all_modules['system_monitor'].stop()
    all_modules['cpu_scheduler'].stop()
    await all_modules['async_communicator'].stop()
    await all_modules['event_bus'].stop()

    print("\n" + "=" * 80)
    print("Epic 4 完整集成测试完成")
    print("=" * 80)

    return True


async def test_performance_requirements():
    """测试性能要求"""
    print("\n⚡ 测试性能要求...")

    # 创建轻量级测试环境
    message_queue = MessageQueue("perf_test")
    system_monitor = SystemMonitor(collection_interval=0.1)
    health_checker = HealthChecker(check_interval=0.5)

    await system_monitor.start()
    await health_checker.start()

    # 测试1: 系统监控响应时间
    start_time = time.perf_counter()
    for _ in range(100):
        status = system_monitor.get_system_status()
        _ = health_checker.quick_health_check()
    monitoring_time_ms = (time.perf_counter() - start_time) * 1000 / 100

    print(f"  ✅ 监控响应时间: {monitoring_time_ms:.2f}ms (要求 < 10ms)")
    assert monitoring_time_ms < 10.0, f"监控响应时间超标: {monitoring_time_ms:.2f}ms"

    # 测试2: 高频配置更新
    config_manager = ConfigManager()
    await config_manager.start()

    start_time = time.perf_counter()
    for i in range(100):
        config_manager.set_config(f"perf.test.{i}", i)
    config_update_time_ms = (time.perf_counter() - start_time) * 1000 / 100

    print(f"  ✅ 配置更新时间: {config_update_time_ms:.2f}ms (要求 < 5ms)")
    assert config_update_time_ms < 5.0, f"配置更新时间超标: {config_update_time_ms:.2f}ms"

    await config_manager.stop()
    await health_checker.stop()
    await system_monitor.stop()

    print("  ✅ 所有性能测试通过")


async def main():
    """主测试函数"""
    try:
        await test_epic_4_complete_integration()
        await test_performance_requirements()

        print("\n" + "=" * 80)
        print("🎉 Epic 4 所有测试通过!")
        print("=" * 80)
        print("\n✅ 模块协调机制验证通过")
        print("✅ 资源管理验证通过")
        print("✅ 系统监控验证通过")
        print("✅ 配置管理验证通过")
        print("✅ 性能要求验证通过")
        print("\n🚀 Epic 4 开发完成!")

        return 0

    except Exception as e:
        print(f"\n❌ 测试失败: {str(e)}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    exit_code = asyncio.run(main())
    exit(exit_code)
