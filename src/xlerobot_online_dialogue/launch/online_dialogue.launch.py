"""
在线对话服务启动文件
Story 1.7: 多模态在线对话API集成
严格遵循Epic 1纯在线架构
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """生成启动描述"""
    package_name = 'xlerobot_online_dialogue'

    # 声明启动参数
    api_key_arg = DeclareLaunchArgument(
        'dashscope_api_key',
        default_value='',
        description='阿里云DashScope API密钥'
    )

    enable_debug_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='是否启用调试模式'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='日志级别 (debug, info, warn, error)'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='qwen-max',
        description='对话模型名称'
    )

    session_timeout_arg = DeclareLaunchArgument(
        'session_timeout',
        default_value='300.0',
        description='会话超时时间（秒）'
    )

    # 获取配置文件路径
    api_key = LaunchConfiguration('dashscope_api_key')
    debug_mode = LaunchConfiguration('debug_mode')
    log_level = LaunchConfiguration('log_level')
    model = LaunchConfiguration('model')
    session_timeout = LaunchConfiguration('session_timeout')

    config_file = LaunchConfiguration(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare(package_name),
            'config',
            'online_dialogue_config.yaml'
        ])
    )

    # 在线对话节点
    dialogue_node = Node(
        package=package_name,
        executable='online_dialogue_node',
        name='online_dialogue_node',
        output='screen',
        parameters=[
            config_file,
            {
                'dashscope_api_key': api_key,
                'debug_mode': debug_mode,
                'log_level': log_level,
                'model': model,
                'session_timeout': session_timeout
            }
        ],
        arguments=['--ros-args', '--log-level', log_level],
        emulate_tty=True
    )

    # 如果启用调试模式，启动调试工具
    debug_tools = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rqt_console',
                executable='rqt_console',
                name='online_dialogue_debug_console',
                condition=IfCondition(debug_mode)
            )
        ]
    )

    # 启动描述
    ld = LaunchDescription()

    # 添加参数声明
    ld.add_action(api_key_arg)
    ld.add_action(enable_debug_arg)
    ld.add_action(log_level_arg)
    ld.add_action(model_arg)
    ld.add_action(session_timeout_arg)

    # 添加节点
    ld.add_action(dialogue_node)
    ld.add_action(debug_tools)

    return ld

if __name__ == '__main__':
    generate_launch_description()