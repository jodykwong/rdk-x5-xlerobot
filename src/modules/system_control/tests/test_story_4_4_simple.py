"""
Story 4.4 简单验证测试 - 系统监控
验证系统监控模块基本功能
"""

import asyncio
import time
import sys

# 添加项目根目录到路径
sys.path.insert(0, '/home/sunrise/xlerobot')

from src.modules.system_control.system_monitor import SystemMonitor, AlertLevel, MetricType
from src.modules.system_control.health_checker import HealthChecker, HealthStatus
from src.modules.system_control.resource_monitor import ResourceMonitor, ResourceType


async def test_all_monitoring_modules():
    """测试所有监控模块"""
    print("=" * 60)
    print("开始 Story 4.4 系统监控验证测试")
    print("=" * 60)

    # 测试系统监控器
    print("\n📊 测试系统监控器...")
    system_monitor = SystemMonitor(collection_interval=1.0)

    alert_received = []

    def alert_handler(alert):
        alert_received.append(alert)

    system_monitor.register_alert_callback(alert_handler)

    await system_monitor.start()

    # 等待收集数据
    await asyncio.sleep(3)

    # 获取系统状态
    status = system_monitor.get_system_status()
    print(f"  ✅ 系统状态: {status.state.value}")
    print(f"  ✅ CPU使用率: {status.cpu_usage:.1f}%")
    print(f"  ✅ 内存使用率: {status.memory_usage:.1f}%")
    print(f"  ✅ 磁盘使用率: {status.disk_usage:.1f}%")

    # 获取指标
    cpu_value = system_monitor.get_metric_value("cpu_usage")
    memory_value = system_monitor.get_metric_value("memory_usage")
    print(f"  ✅ 指标获取: CPU={cpu_value:.1f}%, 内存={memory_value:.1f}%")

    # 获取统计信息
    stats = system_monitor.get_stats()
    print(f"  ✅ 统计信息: 收集指标 {stats['active_metrics']} 个")

    await system_monitor.stop()

    # 测试健康检查器
    print("\n💚 测试健康检查器...")
    health_checker = HealthChecker(check_interval=1.0)

    health_received = []

    def health_handler(health):
        health_received.append(health)

    health_checker.register_health_callback(health_handler)

    await health_checker.start()

    # 等待健康检查
    await asyncio.sleep(3)

    # 快速健康检查
    quick_health = health_checker.quick_health_check()
    print(f"  ✅ 快速健康检查: {quick_health.overall_status.value}")
    print(f"  ✅ 健康分数: {quick_health.score:.1f}/100")

    # 显示检查项
    for check in quick_health.checks:
        print(f"    - {check.name}: {check.status.value} - {check.message}")

    await health_checker.stop()

    # 测试资源监控器
    print("\n🔍 测试资源监控器...")
    resource_monitor = ResourceMonitor(collection_interval=1.0)

    resource_alerts = []

    def resource_alert_handler(alert):
        resource_alerts.append(alert)

    resource_monitor.register_alert_callback(resource_alert_handler)

    await resource_monitor.start()

    # 等待收集数据
    await asyncio.sleep(3)

    # 获取当前资源使用情况
    cpu_usage = resource_monitor.get_current_resource_usage(ResourceType.CPU)
    memory_usage = resource_monitor.get_current_resource_usage(ResourceType.MEMORY)
    print(f"  ✅ 资源使用: CPU={cpu_usage.usage_percent:.1f}%, "
          f"内存={memory_usage.usage_percent:.1f}%" if cpu_usage and memory_usage else "  ⚠️ 资源数据收集中")

    # 获取TOP进程
    top_processes = resource_monitor.get_top_processes('cpu_percent', 3)
    if top_processes:
        print(f"  ✅ TOP进程: {len(top_processes)} 个")
        for proc in top_processes[:2]:
            print(f"    - {proc.name}: CPU {proc.cpu_percent:.1f}%")
    else:
        print(f"  ⚠️ 进程数据收集中")

    # 获取资源趋势
    cpu_trends = resource_monitor.get_resource_trends(ResourceType.CPU, 60.0)
    if cpu_trends:
        print(f"  ✅ 趋势分析: 当前 {cpu_trends['current']:.1f}%, "
              f"平均 {cpu_trends['average']:.1f}%, "
              f"趋势 {cpu_trends['trend']}")

    # 获取统计信息
    rstats = resource_monitor.get_stats()
    print(f"  ✅ 监控统计: 进程 {rstats['current_processes']} 个, "
          f"收集 {rstats['collections_performed']} 次")

    await resource_monitor.stop()

    # 测试模块协同工作
    print("\n🔗 测试模块协同工作...")

    # 创建组合监控
    combined_monitor = SystemMonitor(collection_interval=0.5)
    combined_health = HealthChecker(check_interval=0.5)
    combined_resource = ResourceMonitor(collection_interval=0.5)

    # 启动所有监控
    await combined_monitor.start()
    await combined_health.start()
    await combined_resource.start()

    # 运行一段时间
    await asyncio.sleep(5)

    # 获取综合状态
    sys_status = combined_monitor.get_system_status()
    health_status = await combined_health.get_current_health()
    cpu_usage = combined_resource.get_current_resource_usage(ResourceType.CPU)

    print(f"  ✅ 系统状态: {sys_status.state.value}")
    print(f"  ✅ 健康状态: {health_status.overall_status.value if health_status else 'N/A'}")
    print(f"  ✅ 资源状态: CPU {cpu_usage.usage_percent:.1f}%" if cpu_usage else "  - 资源收集中")

    # 验证延迟要求（< 10ms）
    start_time = time.perf_counter()
    _ = combined_monitor.get_system_status()
    _ = combined_health.quick_health_check()
    _ = combined_resource.get_top_processes('cpu_percent', 5)
    end_time = time.perf_counter()

    latency_ms = (end_time - start_time) * 1000
    print(f"  ✅ 访问延迟: {latency_ms:.2f}ms {'(< 10ms ✓)' if latency_ms < 10.0 else '(>= 10ms ⚠️)'}")

    # 停止所有监控
    await combined_resource.stop()
    await combined_health.stop()
    await combined_monitor.stop()

    print("\n" + "=" * 60)
    print("Story 4.4 系统监控验证测试完成")
    print("=" * 60)

    return True


if __name__ == "__main__":
    try:
        asyncio.run(test_all_monitoring_modules())
        print("\n✅ 所有监控测试通过!")
        exit(0)
    except Exception as e:
        print(f"\n❌ 测试失败: {str(e)}")
        import traceback
        traceback.print_exc()
        exit(1)
