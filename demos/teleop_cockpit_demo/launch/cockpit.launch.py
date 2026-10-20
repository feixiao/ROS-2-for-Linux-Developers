#!/usr/bin/env python3
"""
Launch file for Browser-based Teleoperation Cockpit Demo.
Spawns:
1. cockpit_bridge_node (Web server on port 8080 + HUD video renderer + /cmd_vel publisher)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8080',
        description='Port for Cockpit Web Server'
    )

    port = LaunchConfiguration('port')

    cockpit_node = Node(
        package='teleop_cockpit_demo',
        executable='cockpit_bridge_node',
        name='cockpit_bridge_node',
        parameters=[{'port': port}],
        output='screen'
    )

    return LaunchDescription([
        port_arg,
        LogInfo(msg=">>> Launching ROS 2 Browser Teleoperation Cockpit on port 8080 <<<"),
        cockpit_node
    ])
