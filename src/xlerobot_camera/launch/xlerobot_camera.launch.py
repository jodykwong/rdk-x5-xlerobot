#!/usr/bin/env python3
"""
XleRobot IMX219 Camera Launch File
基于D-Robotics RDK X5的摄像头启动文件
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成启动描述"""
    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('xlerobot_camera'),
            'config',
            'imx219_config.yaml'
        ]),
        description='Path to camera configuration file'
    )

    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='0',
        description='Camera ID (0-3)'
    )

    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='imx219',
        description='Camera name'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_link',
        description='Camera frame ID'
    )

    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='1920',
        description='Image width'
    )

    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='1080',
        description='Image height'
    )

    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='Frames per second'
    )

    pixel_format_arg = DeclareLaunchArgument(
        'pixel_format',
        default_value='rgb8',
        description='Pixel format (rgb8, bgr8, yuyv, gray)'
    )

    publish_raw_arg = DeclareLaunchArgument(
        'publish_raw',
        default_value='true',
        description='Publish raw images'
    )

    publish_compressed_arg = DeclareLaunchArgument(
        'publish_compressed',
        default_value='true',
        description='Publish compressed images'
    )

    enable_debug_arg = DeclareLaunchArgument(
        'enable_debug',
        default_value='false',
        description='Enable debug output'
    )

    # 摄像头节点
    camera_node = Node(
        package='xlerobot_camera',
        executable='camera_node',
        name='xlerobot_camera_node',
        namespace='',
        parameters=[
            {'camera_id': LaunchConfiguration('camera_id')},
            {'camera_name': LaunchConfiguration('camera_name')},
            {'frame_id': LaunchConfiguration('frame_id')},
            {'image_width': LaunchConfiguration('image_width')},
            {'image_height': LaunchConfiguration('image_height')},
            {'fps': LaunchConfiguration('fps')},
            {'pixel_format': LaunchConfiguration('pixel_format')},
            {'publish_raw': LaunchConfiguration('publish_raw')},
            {'publish_compressed': LaunchConfiguration('publish_compressed')},
        ],
        output='screen',
        emulate_tty=True,
        log_level='DEBUG' if IfCondition(LaunchConfiguration('enable_debug')) else 'INFO'
    )

    # 图像查看节点 (可选)
    image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='image_view',
        namespace='',
        parameters=[
            {'image_topic': '/' + LaunchConfiguration('camera_name') + '/image_compressed'}
        ],
        condition=IfCondition(LaunchConfiguration('enable_debug')),
        output='screen'
    )

    return LaunchDescription([
        config_file_arg,
        camera_id_arg,
        camera_name_arg,
        frame_id_arg,
        image_width_arg,
        image_height_arg,
        fps_arg,
        pixel_format_arg,
        publish_raw_arg,
        publish_compressed_arg,
        enable_debug_arg,
        camera_node,
        image_view_node
    ])