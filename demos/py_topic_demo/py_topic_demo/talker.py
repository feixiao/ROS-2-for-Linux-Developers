#!/usr/bin/env python3
"""
Minimal Python Publisher Node for ROS 2.
Illustrates object-oriented Node inheritance, parameter declaration, QoS, and timers.
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String


class MinimalPyPublisher(Node):

    def __init__(self):
        super().__init__('minimal_py_publisher')

        # 声明参数: 发布频率
        self.declare_parameter('frequency', 1.0)
        freq = self.get_parameter('frequency').get_parameter_value().double_value

        # 配置与 C++ 端兼容的 QoS Profile
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.publisher_ = self.create_publisher(String, 'chatter', qos_profile)
        self.timer = self.create_timer(1.0 / freq, self.timer_callback)
        self.i = 0
        self.get_logger().info(f'Python MinimalPyPublisher started at {freq} Hz')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from Linux ROS 2 Python node! Index: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = MinimalPyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
