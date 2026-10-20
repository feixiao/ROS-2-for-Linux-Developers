#!/usr/bin/env python3
"""
Zero-Hardware Robot Simulation Node (Kinematics, Raycasting LiDAR & Odometry).
Simulates:
1. Two-wheeled differential drive kinematics积分 (/cmd_vel -> /odom & /tf).
2. 360-degree Raycasting LiDAR sensor simulation in a virtual room (/scan LaserScan).
3. Continuous broadcast of odom -> base_link -> laser_frame transforms.
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf2_ros


class SimulatedRobotNode(Node):

    def __init__(self):
        super().__init__('simulated_robot_node')

        self.declare_parameter('update_rate', 50.0)      # 50 Hz 运动学积分
        self.declare_parameter('scan_rate', 10.0)        # 10 Hz 激光雷达扫描
        self.declare_parameter('lidar_num_beams', 360)   # 360 线光线投射
        self.declare_parameter('lidar_max_range', 12.0)  # 最大探测距离 12m

        self.dt = 1.0 / self.get_parameter('update_rate').value
        self.num_beams = self.get_parameter('lidar_num_beams').value
        self.max_range = self.get_parameter('lidar_max_range').value

        # 机器人当前物理状态 (x, y, theta)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_v = 0.0
        self.angular_w = 0.0

        # 模拟虚拟房间墙体几何约束 [x1, y1, x2, y2] (单位: 米)
        # 10m x 10m 房间，中央带若干障碍物立柱
        self.walls = [
            (-5.0, -5.0, 5.0, -5.0),   # 南墙
            (5.0, -5.0, 5.0, 5.0),     # 东墙
            (5.0, 5.0, -5.0, 5.0),     # 北墙
            (-5.0, 5.0, -5.0, -5.0),   # 西墙
            (1.5, 1.5, 2.5, 1.5),      # 障碍物立柱 A
            (2.5, 1.5, 2.5, 2.5),
            (2.5, 2.5, 1.5, 2.5),
            (1.5, 2.5, 1.5, 1.5),
            (-2.0, -1.0, -1.0, -1.0),  # 障碍物立柱 B
            (-1.0, -1.0, -1.0, -2.0),
            (-1.0, -2.0, -2.0, -2.0),
            (-2.0, -2.0, -2.0, -1.0)
        ]

        # 1. 发布与订阅通道
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        # 2. tf 坐标变换广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 3. 50Hz 运动学物理模拟定时器
        self.physics_timer = self.create_timer(self.dt, self.physics_step)

        # 4. 10Hz 激光雷达光线追踪模拟定时器
        self.scan_timer = self.create_timer(1.0 / self.get_parameter('scan_rate').value, self.publish_laser_scan)

        self.get_logger().info('===========================================================')
        self.get_logger().info('>>> Zero-Hardware Robot Physics & LiDAR Simulator ONLINE <<<')
        self.get_logger().info('    Publishing: /odom, /scan (360 beams), /tf')
        self.get_logger().info('    Subscribing: /cmd_vel')
        self.get_logger().info('===========================================================')

    def on_cmd_vel(self, msg):
        self.linear_v = msg.linear.x
        self.angular_w = msg.angular.z

    def physics_step(self):
        # 1. 差速底盘运动学正向积分 (Forward Kinematics Integration)
        self.theta += self.angular_w * self.dt
        self.x += self.linear_v * math.cos(self.theta) * self.dt
        self.y += self.linear_v * math.sin(self.theta) * self.dt

        now = self.get_clock().now().to_msg()

        # 2. 构造四元数旋转
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        # 3. 广播 odom -> base_link 动态变换
        t_odom = TransformStamped()
        t_odom.header.stamp = now
        t_odom.header.frame_id = 'odom'
        t_odom.child_frame_id = 'base_link'
        t_odom.transform.translation.x = float(self.x)
        t_odom.transform.translation.y = float(self.y)
        t_odom.transform.translation.z = 0.0
        t_odom.transform.rotation.x = 0.0
        t_odom.transform.rotation.y = 0.0
        t_odom.transform.rotation.z = float(qz)
        t_odom.transform.rotation.w = float(qw)
        self.tf_broadcaster.sendTransform(t_odom)

        # 4. 广播 base_link -> laser_frame 刚体安装变换
        t_laser = TransformStamped()
        t_laser.header.stamp = now
        t_laser.header.frame_id = 'base_link'
        t_laser.child_frame_id = 'laser_frame'
        t_laser.transform.translation.x = 0.15
        t_laser.transform.translation.y = 0.0
        t_laser.transform.translation.z = 0.20
        t_laser.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_laser)

        # 5. 发布 /odom 话题
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.orientation.z = float(qz)
        odom.pose.pose.orientation.w = float(qw)
        odom.twist.twist.linear.x = float(self.linear_v)
        odom.twist.twist.angular.z = float(self.angular_w)
        self.odom_pub.publish(odom)

    def publish_laser_scan(self):
        # 360 度光线投射模拟激光雷达测距
        now = self.get_clock().now().to_msg()
        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (2.0 * math.pi) / float(self.num_beams)
        scan.time_increment = (1.0 / 10.0) / float(self.num_beams)
        scan.range_min = 0.12
        scan.range_max = float(self.max_range)

        ranges = []
        for i in range(self.num_beams):
            angle = scan.angle_min + i * scan.angle_increment
            beam_world_angle = self.theta + angle
            dx = math.cos(beam_world_angle)
            dy = math.sin(beam_world_angle)
            min_dist = self.max_range

            # 计算射线与所有墙体线段的几何交点
            for (x1, y1, x2, y2) in self.walls:
                denom = (y2 - y1) * dx - (x2 - x1) * dy
                if abs(denom) > 1e-6:
                    ua = ((x2 - x1) * (self.y - y1) - (y2 - y1) * (self.x - x1)) / denom
                    ub = (dx * (self.y - y1) - dy * (self.x - x1)) / denom
                    if ua > 0 and 0.0 <= ub <= 1.0:
                        if ua < min_dist:
                            min_dist = ua
            ranges.append(float(min_dist))

        scan.ranges = ranges
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
