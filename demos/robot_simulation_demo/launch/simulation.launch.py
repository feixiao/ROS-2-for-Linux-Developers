#!/usr/bin/env python3
"""
Launch file for Zero-Hardware Robot Simulation Demo.
Spawns:
1. simulated_robot_node (50Hz physics kinematics + 10Hz 360-ray LiDAR)
"""

from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    sim_robot_node = Node(
        package='robot_simulation_demo',
        executable='simulated_robot_node',
        name='simulated_robot_node',
        output='screen'
    )

    return LaunchDescription([
        LogInfo(msg=">>> Launching Zero-Hardware Robot Simulation (Kinematics + LiDAR) <<<"),
        sim_robot_node
    ])
