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

    # 获取项目根路径 - 支持环境变量覆盖，避免硬编码
    xlerobot_root = os.environ.get('XLEROBOT_ROOT', '/home/sunrise/xlerobot')
    # 如果环境变量未设置，尝试从当前文件位置推断
    if xlerobot_root == '/home/sunrise/xlerobot' and not os.path.exists(xlerobot_root):
        # 从 src/xlerobot/launch/ 向上三级找到项目根目录
        inferred_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
        if os.path.exists(os.path.join(inferred_root, 'start_voice_assistant.sh')):
            xlerobot_root = inferred_root

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
            'tts_voice',
            default_value='xiaoyun',
            description='TTS音色选择'
        ),
    ]

    # 设置环境变量 - 传递关键的API密钥和路径配置
    env_variables = [
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT',
                             '[{severity}] [{name}]: {message}'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        # 确保关键的API密钥被传递到ROS2子进程
        SetEnvironmentVariable('QWEN_API_KEY', os.environ.get('QWEN_API_KEY', '')),
        SetEnvironmentVariable('ALIBABA_CLOUD_ACCESS_KEY_ID', os.environ.get('ALIBABA_CLOUD_ACCESS_KEY_ID', '')),
        SetEnvironmentVariable('ALIBABA_CLOUD_ACCESS_KEY_SECRET', os.environ.get('ALIBABA_CLOUD_ACCESS_KEY_SECRET', '')),
        SetEnvironmentVariable('ALIYUN_NLS_APPKEY', os.environ.get('ALIYUN_NLS_APPKEY', '')),
        # 确保Python路径正确传递
        SetEnvironmentVariable('PYTHONPATH', os.environ.get('PYTHONPATH', '')),
        # 确保可执行文件路径正确
        SetEnvironmentVariable('PYTHON_EXECUTABLE', os.environ.get('PYTHON_EXECUTABLE', '/usr/bin/python3.10')),
    ]

    # 设置TTS语音环境变量
    tts_voice_env = SetEnvironmentVariable(
        'TTS_VOICE',
        LaunchConfiguration('tts_voice')
    )

    # 核心节点列表
    nodes = []

    # 1. 主控协调节点 - 最先启动
    coordinator_node = Node(
        package='xlerobot',
        executable='/usr/bin/python3.10',
        arguments=[f'{xlerobot_root}/src/xlerobot/nodes/voice_assistant_coordinator.py'],
        name='voice_assistant_coordinator',
        parameters=[{
            'log_level': LaunchConfiguration('log_level'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen',
        emulate_tty=True
    )
    nodes.append(coordinator_node)

    # 2. LLM服务节点
    llm_node = Node(
        package='xlerobot',
        executable='/usr/bin/python3.10',
        arguments=[f'{xlerobot_root}/src/xlerobot/nodes/llm_service_node.py'],
        name='llm_service_node',
        parameters=[{
            'log_level': LaunchConfiguration('log_level'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'model_name': 'qwen-turbo',
            'max_tokens': 2000,
            'temperature': 0.7
        }],
        output='screen',
        emulate_tty=True
        # 条件启动：需要QWEN_API_KEY
        # condition=IfCondition(LaunchConfiguration('qwen_api_key'))
    )
    nodes.append(llm_node)

    # 3. TTS服务节点
    tts_node = Node(
        package='xlerobot',
        executable='/usr/bin/python3.10',
        arguments=[f'{xlerobot_root}/src/xlerobot/nodes/tts_service_node.py'],
        name='tts_service_node',
        parameters=[{
            'log_level': LaunchConfiguration('log_level'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'voice': LaunchConfiguration('tts_voice'),
            'output_dir': os.environ.get('XLEROBOT_TTS_OUTPUT_DIR', '/tmp/xlerobot_tts'),
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
    asr_bridge_node = Node(
        package='xlerobot',
        executable='/usr/bin/python3.10',
        arguments=[f'{xlerobot_root}/src/xlerobot/nodes/asr_bridge_node.py'],
        name='asr_bridge_node',
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