#!/usr/bin/env python3
"""
ROS 2 Teleoperation Cockpit Bridge Node (Pure Linux Stream & Control).
Features:
1. Dynamic 3D Perspective Ground Grid & Artificial Horizon HUD Synthesizer (1280x720 @ 30 FPS).
2. Embedded multi-threaded HTTP Server on port 8080 serving cockpit UI and low-latency MJPEG video stream.
3. Teleop control channel: parses keyboard/joystick commands and publishes to /cmd_vel.
4. Auto-braking safety watchdog (resets velocity to zero on connection loss or timeout).
"""

import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False


class SharedState:
    current_linear = 0.0
    current_angular = 0.0
    last_cmd_time = time.time()
    latest_jpeg = None
    lock = threading.Lock()
    grid_offset = 0.0
    horizon_angle = 0.0


class CockpitHttpHandler(BaseHTTPRequestHandler):

    def do_HEAD(self):
        self.do_GET()

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == '/' or path == '/index.html':
            html_content = self.load_html()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(html_content)))
            self.end_headers()
            self.wfile.write(html_content)

        elif path == '/stream':
            # 低延迟连续 MJPEG 视频流传输
            self.send_response(200)
            self.send_header('Age', '0')
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()

            while True:
                with SharedState.lock:
                    jpeg_bytes = SharedState.latest_jpeg

                if jpeg_bytes is not None:
                    try:
                        self.wfile.write(b'--frame\r\n')
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', str(len(jpeg_bytes)))
                        self.end_headers()
                        self.wfile.write(jpeg_bytes)
                        self.wfile.write(b'\r\n')
                    except (BrokenPipeError, ConnectionResetError, Exception):
                        break
                time.sleep(0.033)

        elif path == '/cmd':
            # 解析浏览器发送的速度控制指令
            query = parse_qs(parsed.query)
            v = float(query.get('v', [0.0])[0])
            w = float(query.get('w', [0.0])[0])

            with SharedState.lock:
                SharedState.current_linear = v
                SharedState.current_angular = w
                SharedState.last_cmd_time = time.time()

            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"status":"ok"}')

        elif path == '/ping':
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'pong')

        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass

    def load_html(self):
        possible_paths = [
            os.path.join(os.path.dirname(__file__), '..', 'web', 'index.html'),
            os.path.join(os.path.dirname(__file__), 'web', 'index.html'),
            '/opt/ros2_app/teleop_cockpit_demo/share/teleop_cockpit_demo/web/index.html',
            os.path.abspath('demos/teleop_cockpit_demo/web/index.html')
        ]
        for p in possible_paths:
            if os.path.exists(p):
                with open(p, 'rb') as f:
                    return f.read()
        return b"<h1>Teleop Cockpit HTML file not found</h1>"


class TeleopCockpitBridgeNode(Node):

    def __init__(self):
        super().__init__('teleop_cockpit_bridge_node')

        self.declare_parameter('port', 8080)
        self.port = self.get_parameter('port').value

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.bridge = CvBridge() if HAS_CV_BRIDGE else None
        self.external_frame = None
        self.sub_cam = self.create_subscription(
            Image,
            'camera/image_raw',
            self.on_camera_image,
            10
        )

        # 1. 启动嵌入式多线程 HTTP Web 服务线程
        self.http_thread = threading.Thread(target=self.run_server, daemon=True)
        self.http_thread.start()

        # 2. 定时器 (30Hz): 驱动 HUD 画面渲染与图像编码
        self.render_timer = self.create_timer(1.0 / 30.0, self.render_hud_frame)

        # 3. 定时器 (20Hz): 执行底盘控制发布与安全看门狗检测
        self.cmd_timer = self.create_timer(1.0 / 20.0, self.control_loop)

        self.get_logger().info('======================================================')
        self.get_logger().info(f'>>> Teleoperation Cockpit Ready at http://localhost:{self.port} <<<')
        self.get_logger().info('======================================================')

    def on_camera_image(self, msg):
        if self.bridge is not None:
            try:
                self.external_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception:
                pass

    def run_server(self):
        server = ThreadingHTTPServer(('0.0.0.0', self.port), CockpitHttpHandler)
        server.serve_forever()

    def control_loop(self):
        now = time.time()
        with SharedState.lock:
            # 500ms 安全看门狗
            if now - SharedState.last_cmd_time > 0.5:
                SharedState.current_linear = 0.0
                SharedState.current_angular = 0.0

            v = SharedState.current_linear
            w = SharedState.current_angular

        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.cmd_pub.publish(twist)

    def render_hud_frame(self):
        width, height = 1280, 720
        with SharedState.lock:
            v = SharedState.current_linear
            w = SharedState.current_angular
            SharedState.grid_offset = (SharedState.grid_offset + v * 8.0) % 60.0
            SharedState.horizon_angle += w * 1.5
            grid_offset = SharedState.grid_offset

        if self.external_frame is not None:
            frame = cv2.resize(self.external_frame, (width, height))
        else:
            frame = np.zeros((height, width, 3), dtype=np.uint8)
            frame[:] = (12, 16, 24)

            horizon_y = int(height * 0.55)
            cv2.rectangle(frame, (0, horizon_y), (width, height), (8, 12, 18), -1)

            center_x = width // 2
            for x_line in range(-800, 801, 80):
                pt_far = (center_x + int(x_line * 0.15), horizon_y)
                pt_near = (center_x + int(x_line * 1.8), height)
                cv2.line(frame, pt_far, pt_near, (28, 42, 60), 1, cv2.LINE_AA)

            for d in range(0, height - horizon_y, 40):
                curr_y = horizon_y + int(((d + grid_offset) % (height - horizon_y)) ** 1.35 * 0.28)
                if curr_y < height:
                    alpha = (curr_y - horizon_y) / float(height - horizon_y)
                    color = (int(30 * alpha), int(60 * alpha), int(75 * alpha))
                    cv2.line(frame, (0, curr_y), (width, curr_y), color, 1, cv2.LINE_AA)

        hud_color = (204, 255, 0)
        amber_color = (0, 180, 255)

        cx, cy = width // 2, height // 2

        cv2.circle(frame, (cx, cy), 28, hud_color, 1, cv2.LINE_AA)
        cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1, cv2.LINE_AA)
        cv2.line(frame, (cx - 45, cy), (cx - 15, cy), hud_color, 1, cv2.LINE_AA)
        cv2.line(frame, (cx + 15, cy), (cx + 45, cy), hud_color, 1, cv2.LINE_AA)
        cv2.line(frame, (cx, cy - 35), (cx, cy - 12), hud_color, 1, cv2.LINE_AA)
        cv2.line(frame, (cx, cy + 12), (cx, cy + 35), hud_color, 1, cv2.LINE_AA)

        rad = math.radians(-w * 15.0)
        cos_r = math.cos(rad)
        sin_r = math.sin(rad)
        h_len = 160
        pt1 = (int(cx - h_len * cos_r), int(cy - h_len * sin_r))
        pt2 = (int(cx + h_len * cos_r), int(cy + h_len * sin_r))
        cv2.line(frame, pt1, pt2, (0, 255, 120), 2, cv2.LINE_AA)

        box_x = int(cx + 180 * math.sin(time.time() * 0.8))
        box_y = int(cy - 60 + 20 * math.cos(time.time() * 1.2))
        cv2.rectangle(frame, (box_x - 35, box_y - 35), (box_x + 35, box_y + 35), amber_color, 1, cv2.LINE_AA)
        cv2.putText(frame, "TARGET: WAYPOINT", (box_x - 45, box_y - 42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, amber_color, 1, cv2.LINE_AA)

        cv2.putText(frame, "ROS 2 TELEOP // COCKPIT LIVE STREAM", (30, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, f"LINEAR SPD : {v:+.2f} m/s", (30, 85),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 204), 2, cv2.LINE_AA)
        cv2.putText(frame, f"ANGULAR YAW: {w:+.2f} rad/s", (30, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 204), 2, cv2.LINE_AA)

        cv2.putText(frame, "LATENCY: <30ms | FPS: 30.0", (width - 340, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 100), 2, cv2.LINE_AA)
        cv2.putText(frame, time.strftime("UTC %Y-%m-%d %H:%M:%S"), (width - 340, 85),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (160, 180, 200), 1, cv2.LINE_AA)

        ret, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ret:
            with SharedState.lock:
                SharedState.latest_jpeg = jpeg.tobytes()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopCockpitBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
