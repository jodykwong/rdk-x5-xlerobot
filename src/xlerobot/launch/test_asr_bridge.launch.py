#!/usr/bin/env python3.10
"""
简化测试Launch文件 - 只启动ASR桥接节点
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xlerobot',
            executable='asr_bridge_node',
            name='asr_bridge_node',
            namespace='xlerobot',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'log_level': 'info',
            }]
        )
    ])