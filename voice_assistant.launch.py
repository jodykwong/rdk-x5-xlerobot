#!/usr/bin/env python3.10
# -*- coding: utf-8 -*-
"""
XLeRobot语音助手Launch文件

启动所有核心ROS2节点，包括ASR、LLM、TTS服务，
以及主控协调节点，实现完整的语音交互流程。

作者: Claude Code
故事ID: Epic 1 ASR→LLM→TTS串联修复
"""

import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成Launch描述"""

    # 获取包路径
    try:
        package_dir = get_package_share_directory('xlerobot')
    except:
        package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    # 声明启动参数
    launch_arguments = [
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='日志级别 (debug, info, warn, error)'
        ),
        DeclareLaunchArgument(
            'enable_debug',
            default_value='false',
            description='是否启用调试模式'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='是否使用仿真时间'
        ),
        DeclareLaunchArgument(
            'qwen_api_key',
            default_value='',
            description='通义千问API密钥'
        ),
        DeclareLaunchArgument(
            'tts_voice',
            default_value='xiaoyun',
            description='TTS音色选择'
        ),
    ]

    # 设置环境变量
    # 注意：API密钥类环境变量已通过父进程env命令注入，不在此处重复设置以避免覆盖为空值
    env_variables = [
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT',
                             '[{severity}] [{name}]: {message}'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
    ]

    # 仅在环境变量存在时才添加设置（避免覆盖为空值）
    # AMENT_PREFIX_PATH 和 PYTHONPATH 由父进程设置，不在此处覆盖
    # API密钥通过env命令传递，保持原值不覆盖

    # 设置TTS语音环境变量
    tts_voice_env = SetEnvironmentVariable(
        'TTS_VOICE',
        LaunchConfiguration('tts_voice')
    )

    # 核心节点列表
    nodes = []

    # 1. 主控协调节点 - 最先启动
    coordinator_executable = os.path.join(package_dir, '..', '..', 'lib', 'xlerobot', 'voice_assistant_coordinator.py')
    coordinator_node = Node(
        executable='/usr/bin/python3.10',
        name='voice_assistant_coordinator',
        namespace='xlerobot',
        arguments=[coordinator_executable,
                  '--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[{
            'log_level': LaunchConfiguration('log_level'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen',
        emulate_tty=True
    )
    nodes.append(coordinator_node)

    # 2. LLM服务节点
    llm_executable = os.path.join(package_dir, '..', '..', 'lib', 'xlerobot', 'llm_service_node.py')
    llm_node = Node(
        executable='/usr/bin/python3.10',
        name='llm_service_node',
        namespace='xlerobot',
        arguments=[llm_executable,
                  '--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[{
            'log_level': LaunchConfiguration('log_level'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'model_name': 'qwen-turbo',
            'max_tokens': 2000,
            'temperature': 0.7,
            'qwen_api_key': LaunchConfiguration('qwen_api_key')
        }],
        output='screen',
        emulate_tty=True,
        # 条件启动：需要QWEN_API_KEY
        # condition=IfCondition(LaunchConfiguration('qwen_api_key'))
    )
    nodes.append(llm_node)

    # 3. TTS服务节点
    tts_executable = os.path.join(package_dir, '..', '..', 'lib', 'xlerobot', 'tts_service_node.py')
    tts_node = Node(
        executable='/usr/bin/python3.10',
        name='tts_service_node',
        namespace='xlerobot',
        arguments=[tts_executable,
                  '--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[{
            'log_level': LaunchConfiguration('log_level'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'voice': LaunchConfiguration('tts_voice'),
            'output_dir': '/tmp/xlerobot_tts',
            'max_queue_length': 5,
            'audio_device': 'default'
        }],
        output='screen',
        emulate_tty=True
    )
    nodes.append(tts_node)

    # 延迟启动节点（确保依赖关系）
    delayed_nodes = []

    # 调整启动顺序和延迟：
    # 1. voice_assistant_coordinator - 延迟0秒（监控节点）
    # 2. tts_service_node - 延迟1秒（音频播放能力）
    # 3. llm_service_node - 延迟2秒（推理能力）
    # 4. asr_bridge_node - 延迟3秒（确保下游服务就绪）

    # 延迟启动TTS节点（1秒延迟，音频播放能力）
    delayed_tts_node = TimerAction(
        period=1.0,  # 1秒延迟
        actions=[tts_node]
    )
    delayed_nodes.append(delayed_tts_node)

    # 延迟启动LLM节点（2秒延迟，推理能力）
    delayed_llm_node = TimerAction(
        period=2.0,  # 2秒延迟
        actions=[llm_node]
    )
    delayed_nodes.append(delayed_llm_node)

    # 4. ASR桥接节点 - 延迟启动（3秒延迟，确保下游服务就绪）
    asr_executable = os.path.join(package_dir, '..', '..', 'lib', 'xlerobot', 'asr_bridge_node.py')
    asr_bridge_node = Node(
        executable='/usr/bin/python3.10',
        name='asr_bridge_node',
        namespace='xlerobot',
        arguments=[asr_executable,
                  '--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[{
            'log_level': LaunchConfiguration('log_level'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'sample_rate': 16000,
            'channels': 1,
            'chunk_size': 1024,
            'qos_reliability': 'reliable',
            'qos_history_depth': 10,
        }],
        output='screen',
        emulate_tty=True
    )

    # 延迟启动ASR节点（3秒延迟，确保下游服务就绪）
    delayed_asr_node = TimerAction(
        period=3.0,  # 3秒延迟
        actions=[asr_bridge_node]
    )
    delayed_nodes.append(delayed_asr_node)

    return LaunchDescription(
        launch_arguments +
        env_variables +
        [tts_voice_env] +
        # 1. 协调节点（立即启动，监控其他节点）
        [coordinator_node] +
        # 2-4. 延迟启动的节点（按正确顺序）
        delayed_nodes
    )


def main():
    """主函数 - 直接运行Launch文件"""
    try:
        from launch import LaunchService
        from launch_ros.actions import Node

        # 创建Launch描述
        ld = generate_launch_description()

        # 创建Launch服务
        ls = LaunchService(debug=False)
        ls.include_launch_description(ld)

        # 启动Launch服务
        return ls.run()

    except ImportError:
        print("❌ 缺少Launch相关模块，请安装:")
        print("sudo apt install ros-humble-launch-ros")
        print("sudo apt install ros-humble-launch-testing-ament-cmake")
        return 1
    except Exception as e:
        print(f"❌ Launch文件执行失败: {e}")
        return 1


if __name__ == '__main__':
    sys.exit(main())