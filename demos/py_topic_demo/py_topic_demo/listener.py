#!/usr/bin/env python3
"""
Minimal Python Subscriber Node for ROS 2.
Illustrates subscriber callback, QoS matching, and graceful shutdown.
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String


class MinimalPySubscriber(Node):

    def __init__(self):
        super().__init__('minimal_py_subscriber')

        # 匹配发布者的 QoS
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            qos_profile
        )
        self.get_logger().info('Python MinimalPySubscriber listening on "chatter"...')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received message: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = MinimalPySubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
