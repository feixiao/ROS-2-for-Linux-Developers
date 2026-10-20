#!/usr/bin/env python3
"""
Prometheus Metrics Exporter Node for ROS 2.
Bridges ROS 2 topic telemetry to Prometheus scraping endpoint (port 9100).
Exposes:
- ros2_messages_total (counter)
- ros2_topic_frequency_hz (gauge)
- ros2_node_uptime_seconds (gauge)
"""

import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MetricsData:
    message_count = 0
    last_msg_time = 0.0
    frequency_hz = 0.0
    start_time = time.time()
    lock = threading.Lock()


class PrometheusHttpHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        if self.path == '/metrics':
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain; version=0.0.4; charset=utf-8')
            self.end_headers()

            with MetricsData.lock:
                uptime = time.time() - MetricsData.start_time
                count = MetricsData.message_count
                hz = MetricsData.frequency_hz

            payload = (
                "# HELP ros2_messages_total Total count of ROS 2 messages received\n"
                "# TYPE ros2_messages_total counter\n"
                f'ros2_messages_total{{topic="/chatter"}} {count}\n\n'
                "# HELP ros2_topic_frequency_hz Current publishing frequency in Hz\n"
                "# TYPE ros2_topic_frequency_hz gauge\n"
                f'ros2_topic_frequency_hz{{topic="/chatter"}} {hz:.2f}\n\n'
                "# HELP ros2_node_uptime_seconds Total runtime of exporter in seconds\n"
                "# TYPE ros2_node_uptime_seconds gauge\n"
                f'ros2_node_uptime_seconds {uptime:.2f}\n'
            )
            self.wfile.write(payload.encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        # 静默 HTTP 访问日志以保持 ROS 控制台纯净
        pass


class Ros2PrometheusExporter(Node):

    def __init__(self):
        super().__init__('ros2_prometheus_exporter')
        self.sub = self.create_subscription(
            String,
            'chatter',
            self.on_message,
            10
        )
        self.msg_window = []
        self.timer = self.create_timer(1.0, self.compute_stats)
        self.get_logger().info('Prometheus Exporter subscribed to "chatter", listening on port 9100/metrics')

    def on_message(self, msg):
        now = time.time()
        with MetricsData.lock:
            MetricsData.message_count += 1
            MetricsData.last_msg_time = now
        self.msg_window.append(now)

    def compute_stats(self):
        now = time.time()
        # 保留最近 5 秒的消息时间戳计算瞬时频率
        self.msg_window = [t for t in self.msg_window if now - t <= 5.0]
        with MetricsData.lock:
            if len(self.msg_window) > 1:
                dt = self.msg_window[-1] - self.msg_window[0]
                MetricsData.frequency_hz = (len(self.msg_window) - 1) / dt if dt > 0 else 0.0
            else:
                MetricsData.frequency_hz = 0.0


def start_http_server(port=9100):
    server = HTTPServer(('0.0.0.0', port), PrometheusHttpHandler)
    server.serve_forever()


def main(args=None):
    rclpy.init(args=args)

    # 启动 HTTP 服务线程
    http_thread = threading.Thread(target=start_http_server, daemon=True)
    http_thread.start()

    node = Ros2PrometheusExporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
