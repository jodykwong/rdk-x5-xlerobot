#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
XleRobot Story 1.4 - TTS服务启动文件
BMad-Method v6 Brownfield Level 4 企业级实现
Story 1.4: 基础语音合成 (阿里云TTS API集成)

功能描述:
- TTS服务节点启动
- 音频播放节点启动
- 参数配置加载
- 严格遵循纯在线服务设计原则
"""

import os
import sys
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成启动描述"""

    # 获取包路径
    xlerobot_dir = Path(__file__).parent.parent
    config_dir = xlerobot_dir / "config"

    # 创建启动描述
    ld = LaunchDescription()

    # 添加TTS服务节点
    tts_node = Node(
        package='xlerobot',
        executable='python3',
        name='tts_service_node',
        output='screen',
        arguments=[
            str(xlerobot_dir / 'src' / 'xlerobot' / 'nodes' / 'tts_service_node.py')
        ],
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time', default_value='false'),
                'log_level': LaunchConfiguration('log_level', default_value='info'),
            }
        ],
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0
    )
    ld.add_action(tts_node)

    # 添加音频播放节点
    player_node = Node(
        package='xlerobot',
        executable='python3',
        name='audio_player_node',
        output='screen',
        arguments=[
            str(xlerobot_dir / 'src' / 'xlerobot' / 'nodes' / 'audio_player_node.py')
        ],
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time', default_value='false'),
                'log_level': LaunchConfiguration('log_level', default_value='info'),
                'alsa_device': LaunchConfiguration('alsa_device', default_value='default'),
                'sample_rate': LaunchConfiguration('sample_rate', default_value='16000'),
                'channels': LaunchConfiguration('channels', default_value='1'),
            }
        ],
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0
    )
    ld.add_action(player_node)

    return ld


def main(argv=None):
    """主函数"""
    try:
        # 生成启动描述
        ld = generate_launch_description()

        # 启动
        print("🚀 启动 XleRobot TTS 服务...")
        print("📋 包含节点:")
        print("  - tts_service_node (TTS服务节点)")
        print("  - audio_player_node (音频播放节点)")

        return ld

    except Exception as e:
        print(f"❌ 启动失败: {e}")
        return None


if __name__ == '__main__':
    try:
        from launch import launch_main
        launch_main(sys.argv, main)
    except ImportError:
        print("❌ launch模块未找到，请安装ROS2 launch工具")
        sys.exit(1)