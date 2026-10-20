#!/usr/bin/env python3
"""
Unit test for Python Listener node using pytest.
Tests node creation, subscription declaration, and callback dispatch.
"""

import unittest
import rclpy
from std_msgs.msg import String
from py_topic_demo.listener import MinimalPySubscriber


class TestMinimalPySubscriber(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = MinimalPySubscriber()

    def tearDown(self):
        self.node.destroy_node()

    def test_subscriber_creation(self):
        self.assertIsNotNone(self.node.subscription)
        self.assertEqual(self.node.subscription.topic_name, '/chatter')

    def test_callback_execution(self):
        # 模拟调用回调并断言不抛出异常
        msg = String()
        msg.data = "Test Mock Payload"
        try:
            self.node.listener_callback(msg)
        except Exception as e:
            self.fail(f"Callback raised exception unexpectedly: {e}")


if __name__ == '__main__':
    unittest.main()
