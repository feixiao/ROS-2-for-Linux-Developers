#!/usr/bin/env python3
"""
Python Service Server Node for ROS 2.
Illustrates create_service, request/response lifecycle, and thread execution.
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class MinimalPyServiceServer(Node):

    def __init__(self):
        super().__init__('minimal_py_service_server')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )
        self.get_logger().info('Python AddTwoInts Service Server ready on "add_two_ints"')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b} -> sum={response.sum}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MinimalPyServiceServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
