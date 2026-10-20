#!/usr/bin/env python3
"""
Production Launch File for ROS 2 Pipeline Orchestration.
Demonstrates:
- Parameter file loading
- Namespace isolation
- Topic remapping
- Multi-language node composition (C++ Talker + Python Listener)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('launch_demo')
    param_file = os.path.join(pkg_share, 'config', 'params.yaml')

    freq_arg = DeclareLaunchArgument(
        'publish_frequency',
        default_value='2.5',
        description='Publish frequency for the C++ talker node'
    )

    publish_frequency = LaunchConfiguration('publish_frequency')

    # C++ Talker 节点
    cpp_talker = Node(
        package='cpp_topic_demo',
        executable='cpp_talker',
        name='cpp_talker_node',
        namespace='sensor_pipeline',
        parameters=[
            param_file,
            {'publish_frequency': publish_frequency}
        ],
        remappings=[
            ('chatter', '/telemetry/chatter')
        ],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    # Python Listener 节点
    py_listener = Node(
        package='py_topic_demo',
        executable='py_listener',
        name='py_listener_node',
        namespace='sensor_pipeline',
        parameters=[param_file],
        remappings=[
            ('chatter', '/telemetry/chatter')
        ],
        output='screen'
    )

    return LaunchDescription([
        freq_arg,
        LogInfo(msg=">>> Launching ROS 2 Sensor Telemetry Pipeline <<<"),
        cpp_talker,
        py_listener,
    ])
