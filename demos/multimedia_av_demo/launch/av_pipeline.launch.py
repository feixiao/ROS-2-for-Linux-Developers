#!/usr/bin/env python3
"""
Audio-Video Multimodal Pipeline Launch File.
Launches:
1. camera_publisher (raw image and CameraInfo publishing via image_transport)
2. gst_h264_streamer (hardware-accelerated low-latency H.264 video streaming)
3. audio_capture_node (16kHz 16-bit linear PCM audio streaming)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fps_arg = DeclareLaunchArgument(
        'frame_rate',
        default_value='30.0',
        description='Camera frame rate (FPS)'
    )

    sample_rate_arg = DeclareLaunchArgument(
        'sample_rate',
        default_value='16000',
        description='Audio sampling rate in Hz'
    )

    frame_rate = LaunchConfiguration('frame_rate')
    sample_rate = LaunchConfiguration('sample_rate')

    # 1. 通用相机发布节点 (image_transport)
    camera_node = Node(
        package='multimedia_av_demo',
        executable='camera_publisher',
        name='camera_publisher',
        parameters=[{'frame_rate': frame_rate}],
        output='screen'
    )

    # 2. GStreamer H.264 硬件硬编推流节点
    gst_streamer_node = Node(
        package='multimedia_av_demo',
        executable='gst_h264_streamer',
        name='gst_h264_streamer',
        output='screen'
    )

    # 3. 麦克风音频采集节点
    audio_node = Node(
        package='multimedia_av_demo',
        executable='audio_capture_node.py',
        name='audio_capture_node',
        parameters=[{'sample_rate': sample_rate}],
        output='screen'
    )

    return LaunchDescription([
        fps_arg,
        sample_rate_arg,
        LogInfo(msg=">>> Launching ROS 2 Audio-Video Multimedia Pipeline <<<"),
        camera_node,
        gst_streamer_node,
        audio_node
    ])
