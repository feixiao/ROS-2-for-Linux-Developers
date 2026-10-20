#!/usr/bin/env python3
"""
Production Nav2 Autonomous Navigation Pipeline Launch.
Orchestrates:
- map_server (loading 2D occupancy grid)
- amcl (particle filter localization)
- controller_server (local trajectory tracking)
- planner_server (global A* path planning)
- bt_navigator (Behavior Tree mission navigator)
- lifecycle_manager (deterministic state transitions)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('nav2_costmap_demo')
    default_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_map_file = os.path.join(pkg_share, 'maps', 'warehouse_map.yaml')

    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_map_file = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map yaml file to load'
    )

    lifecycle_nodes = [
        'map_server',
        'amcl',
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator'
    ]

    # 1. 地图服务器节点
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': map_file}]
    )

    # 2. AMCL 蒙特卡洛定位节点
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file]
    )

    # 3. 控制器服务器 (DWB / MPPI)
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file]
    )

    # 4. 全局规划器服务器 (Smac / NavFn)
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )

    # 5. Nav2 托管生命周期管理器 (保证所有导航节点时序就绪)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': lifecycle_nodes}
        ]
    )

    return LaunchDescription([
        declare_params_file,
        declare_map_file,
        map_server_node,
        amcl_node,
        controller_node,
        planner_node,
        lifecycle_manager_node
    ])
