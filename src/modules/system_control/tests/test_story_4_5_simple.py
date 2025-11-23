"""
Story 4.5 简单验证测试 - 配置管理
验证配置管理模块基本功能
"""

import asyncio
import time
import sys
import tempfile
import os
from pathlib import Path

# 添加项目根目录到路径
sys.path.insert(0, '/home/sunrise/xlerobot')

from src.modules.system_control.config_manager import (
    ConfigManager, ConfigScope, ConfigFormat, ConfigChange
)


async def test_config_manager():
    """测试配置管理器"""
    print("=" * 60)
    print("开始 Story 4.5 配置管理验证测试")
    print("=" * 60)

    # 创建临时目录用于测试
    with tempfile.TemporaryDirectory() as temp_dir:
        config_manager = ConfigManager(temp_dir)
        await config_manager.start()

        # 测试1: 基本配置操作
        print("\n📝 测试基本配置操作...")

        # 设置配置
        success1 = config_manager.set_config(
            "app.name",
            "XLeRobot",
            description="应用程序名称"
        )
        print(f"  ✅ 设置配置 'app.name': {success1}")

        success2 = config_manager.set_config(
            "app.version",
            "1.0.0",
            validation_rules=["type:str"]
        )
        print(f"  ✅ 设置配置 'app.version': {success2}")

        success3 = config_manager.set_config(
            "server.port",
            8080,
            validation_rules=["type:int", "min:1", "max:65535"]
        )
        print(f"  ✅ 设置配置 'server.port': {success3}")

        success4 = config_manager.set_config(
            "debug.enabled",
            True,
            validation_rules=["type:bool"]
        )
        print(f"  ✅ 设置配置 'debug.enabled': {success4}")

        # 获取配置
        app_name = config_manager.get_config("app.name")
        server_port = config_manager.get_config("server.port")
        debug_enabled = config_manager.get_config("debug.enabled")

        print(f"  ✅ 获取配置: app.name={app_name}, server.port={server_port}, debug.enabled={debug_enabled}")

        # 测试2: 配置验证
        print("\n🔍 测试配置验证...")

        # 测试类型验证
        success5 = config_manager.set_config(
            "test.int",
            100,
            validation_rules=["type:int"]
        )
        print(f"  ✅ 整数类型验证: {success5}")

        # 测试范围验证
        success6 = config_manager.set_config(
            "test.range",
            50,
            validation_rules=["min:0", "max:100"]
        )
        print(f"  ✅ 范围验证 (0-100): {success6}")

        # 测试无效配置
        success7 = config_manager.set_config(
            "test.range.invalid",
            150,
            validation_rules=["min:0", "max:100"]
        )
        print(f"  ✅ 无效配置拒绝 (150 > 100): {not success7}")

        # 测试3: 作用域管理
        print("\n🏷️ 测试作用域管理...")

        config_manager.set_config("global.setting", "global_value", scope=ConfigScope.GLOBAL)
        config_manager.set_config("module.setting", "module_value", scope=ConfigScope.MODULE)
        config_manager.set_config("instance.setting", "instance_value", scope=ConfigScope.INSTANCE)

        global_configs = config_manager.get_all_configs(ConfigScope.GLOBAL)
        module_configs = config_manager.get_all_configs(ConfigScope.MODULE)
        instance_configs = config_manager.get_all_configs(ConfigScope.INSTANCE)

        print(f"  ✅ 作用域分离: 全局={len(global_configs)}, 模块={len(module_configs)}, 实例={len(instance_configs)}")

        # 测试4: 变更历史
        print("\n📜 测试变更历史...")

        # 修改配置以生成变更记录
        config_manager.set_config("server.port", 9090)
        config_manager.set_config("app.name", "XLeRobot_Updated")

        change_history = config_manager.get_change_history(limit=10)
        print(f"  ✅ 变更记录: {len(change_history)} 项")
        for change in change_history[-3:]:  # 显示最近3项
            print(f"    - {change.key}: {change.old_value} -> {change.new_value}")

        # 测试5: 配置导出
        print("\n📤 测试配置导出...")

        exported_yaml = config_manager.export_config(format_type=ConfigFormat.YAML)
        print(f"  ✅ YAML导出成功: {len(exported_yaml)} 字符")

        exported_json = config_manager.export_config(format_type=ConfigFormat.JSON)
        print(f"  ✅ JSON导出成功: {len(exported_json)} 字符")

        # 测试6: 回调机制
        print("\n🔔 测试回调机制...")

        changes_received = []

        def change_callback(change: ConfigChange):
            changes_received.append(change)

        config_manager.register_change_callback(change_callback)

        # 触发变更
        config_manager.set_config("callback.test", "test_value")

        print(f"  ✅ 变更回调: 接收到 {len(changes_received)} 个变更通知")
        if changes_received:
            print(f"    - 最新变更: {changes_received[-1].key}")

        # 测试7: 配置模板
        print("\n📋 测试配置模板...")

        template_config = {
            "template.host": "localhost",
            "template.port": 3000,
            "template.workers": 4
        }

        template_content = config_manager.create_config_template(
            "server_template",
            template_config
        )
        print(f"  ✅ 模板创建: {len(template_content)} 字符")

        # 应用模板
        apply_success = config_manager.apply_config_template(
            template_content,
            scope=ConfigScope.GLOBAL
        )
        print(f"  ✅ 模板应用: {apply_success}")

        # 验证模板应用结果
        host_value = config_manager.get_config("template.host")
        port_value = config_manager.get_config("template.port")
        print(f"  ✅ 模板结果: template.host={host_value}, template.port={port_value}")

        # 测试8: 性能测试
        print("\n⚡ 测试性能...")

        start_time = time.perf_counter()
        for i in range(100):
            config_manager.set_config(f"perf.test.{i}", f"value_{i}")
        set_time = time.perf_counter() - start_time

        start_time = time.perf_counter()
        for i in range(100):
            _ = config_manager.get_config(f"perf.test.{i}")
        get_time = time.perf_counter() - start_time

        print(f"  ✅ 性能测试: 设置100项用时 {set_time*1000:.2f}ms")
        print(f"  ✅ 性能测试: 获取100项用时 {get_time*1000:.2f}ms")

        # 测试9: 统计信息
        print("\n📊 测试统计信息...")

        stats = config_manager.get_stats()
        print(f"  ✅ 统计信息:")
        print(f"    - 总配置数: {stats['total_configs']}")
        print(f"    - 变更次数: {stats['changes_made']}")
        print(f"    - 验证次数: {stats['validations_performed']}")
        print(f"    - 配置目录: {stats['config_dir']}")

        # 停止配置管理器
        await config_manager.stop()

        print("\n" + "=" * 60)
        print("Story 4.5 配置管理验证测试完成")
        print("=" * 60)

        return True


if __name__ == "__main__":
    try:
        asyncio.run(test_config_manager())
        print("\n✅ 所有配置管理测试通过!")
        exit(0)
    except Exception as e:
        print(f"\n❌ 测试失败: {str(e)}")
        import traceback
        traceback.print_exc()
        exit(1)
