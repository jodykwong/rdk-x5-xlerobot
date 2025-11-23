#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
健康监控模块

提供系统健康监控、故障检测和自动恢复功能。
包括服务状态监控、性能监控和异常恢复机制。

作者: Dev Agent
功能: 健康检查、自动恢复、性能监控
Version: 1.0.0
"""

__version__ = "1.0.0"
__author__ = "Dev Agent"
__description__ = "健康监控和自动恢复模块"

# 导入核心组件
from .health_monitor import (
    HealthMonitor,
    HealthStatus,
    ServiceType,
    HealthCheck,
    ServiceStatus,
    global_health_monitor
)

# 导出的公共API
__all__ = [
    # 版本信息
    '__version__',
    '__author__',
    '__description__',

    # 核心类
    'HealthMonitor',
    'HealthStatus',
    'ServiceType',
    'HealthCheck',
    'ServiceStatus',

    # 全局实例
    'global_health_monitor',
]

# 便捷的初始化函数
def create_health_monitor(**kwargs) -> HealthMonitor:
    """创建健康监控器实例"""
    return HealthMonitor(**kwargs)


def get_global_health_monitor() -> HealthMonitor:
    """获取全局健康监控器实例"""
    return global_health_monitor


if __name__ == '__main__':
    # 健康监控模块测试
    print("🧪 健康监控模块测试")
    print("=" * 50)

    # 创建健康监控器
    monitor = create_health_monitor(
        check_interval=5.0,
        enable_auto_recovery=True
    )

    print(f"📦 健康监控模块版本: {__version__}")
    print(f"👤 开发者: {__author__}")
    print(f"📝 描述: {__description__}")

    print("\n🎯 可用功能:")
    print("  ✅ 实时健康监控")
    print("  ✅ 自动故障检测")
    print("  ✅ 服务自动重启")
    print("  ✅ 性能监控")
    print("  ✅ 异常处理和恢复")

    print("\n📊 系统状态:")
    status = monitor.get_system_status()
    print(f"  监控活跃: {status['monitoring_active']}")
    print(f"  注册服务: {len(status['services'])}")
    print(f"  检查次数: {status['stats']['total_checks']}")

    print("\n✅ 健康监控模块准备就绪！")
    print("=" * 50)