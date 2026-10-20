#!/usr/bin/env python3
"""
Integration Test for ROS 2 Pipeline using launch_testing.
Tests the end-to-end multi-process execution:
1. Spawns cpp_talker node with remapped topics.
2. Creates an ephemeral ROS 2 subscriber to verify message delivery.
3. Asserts timely receipt, message payload integrity, and clean process termination.
"""

import unittest
import pytest
import launch
import launch_ros
import launch_testing
import launch_testing.actions
import rclpy
from std_msgs.msg import String


@pytest.mark.launch_test
def generate_test_description():
    talker_node = launch_ros.actions.Node(
        package='cpp_topic_demo',
        executable='cpp_talker',
        name='integration_test_talker',
        parameters=[{'publish_frequency': 10.0}],
        remappings=[('chatter', '/test/integration_chatter')],
        output='screen'
    )

    return (
        launch.LaunchDescription([
            talker_node,
            launch_testing.actions.ReadyToTest()
        ]),
        {
            'talker': talker_node
        }
    )


class TestPipelineIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('pipeline_test_evaluator')

    def tearDown(self):
        self.node.destroy_node()

    def test_topic_data_flow(self):
        """Verifies that published messages are received within deadline."""
        received_messages = []
        sub = self.node.create_subscription(
            String,
            '/test/integration_chatter',
            lambda msg: received_messages.append(msg.data),
            10
        )

        # 循环抽样等待消息
        end_time = self.node.get_clock().now() + rclpy.time.Duration(seconds=3.0)
        while self.node.get_clock().now() < end_time and len(received_messages) == 0:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertGreater(
            len(received_messages), 0,
            "Integration Test Failure: No message received on '/test/integration_chatter' within 3s."
        )
        self.assertTrue(
            "Hello from Linux ROS 2" in received_messages[0],
            f"Unexpected payload content: {received_messages[0]}"
        )


@launch_testing.post_shutdown_test()
class TestPipelinePostShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Checks that all processes exited gracefully."""
        launch_testing.asserts.assertExitCodes(proc_info)
