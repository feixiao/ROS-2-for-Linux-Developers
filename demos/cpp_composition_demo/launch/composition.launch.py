#!/usr/bin/env python3
"""
Component Composition Launch File.
Spawns a single ComposableNodeContainer process and dynamically loads
TalkerComponent and ListenerComponent shared libraries via dlopen.
Enables zero-copy intra-process message delivery without DDS serialization.
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name='zero_copy_pipeline_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='cpp_composition_demo',
                plugin='cpp_composition_demo::TalkerComponent',
                name='talker_comp',
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='cpp_composition_demo',
                plugin='cpp_composition_demo::ListenerComponent',
                name='listener_comp',
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
