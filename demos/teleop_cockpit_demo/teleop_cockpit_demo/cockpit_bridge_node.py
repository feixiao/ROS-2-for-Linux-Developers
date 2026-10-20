#!/usr/bin/env python3
"""
ROS 2 Teleoperation Cockpit Bridge Node.
Features:
1. Dynamic 3D Perspective Ground Grid & Artificial Horizon HUD Synthesizer (1280x720 @ 30 FPS).
2. Embedded HTTP Server on port 8080 serving cockpit UI and low-latency MJPEG video stream.
3. Dual-channel teleop control: parses keyboard/joystick commands and publishes to /cmd_vel.
4. Auto-braking safety watchdog (resets velocity to zero on connection loss or timeout).
"""

import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
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
            # 查找并返回 index.html
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
                    except (BrokenPipeError, ConnectionResetError):
                        break
                time.sleep(0.033)  # 约 30 FPS 刷新率

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
        # 静默 HTTP 访问日志以保持 ROS 控制台纯净
        pass

    def load_html(self):
        # 依次从工作空间源码目录或安装目录查找 index.html
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

        # 发布底盘速度指令 (/cmd_vel)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # 订阅外部真实相机图像 (如果有)
        self.bridge = CvBridge() if HAS_CV_BRIDGE else None
        self.external_frame = None
        self.sub_cam = self.create_subscription(
            Image,
            'camera/image_raw',
            self.on_camera_image,
            10
        )

        # 1. 启动嵌入式 HTTP Web 服务线程
        self.http_thread = threading.Thread(target=self.run_server, daemon=True)
        self.http_thread.start()

        # 2. 定时器 (30Hz): 驱动 HUD 画面渲染与图像编码
        self.render_timer = self.create_timer(1.0 / 30.0, self.render_hud_frame)

        # 3. 定时器 (20Hz): 执行底盘控制发布与安全看门狗检测
        self.cmd_timer = self.create_timer(1.0 / 20.0, self.control_loop)

        self.get_logger().info(f'======================================================')
        self.get_logger().info(f'>>> Teleoperation Cockpit Ready at http://localhost:{self.port} <<<')
        self.get_logger().info(f'======================================================')

    def on_camera_image(self, msg):
        if self.bridge is not None:
            try:
                self.external_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception:
                pass

    def run_server(self):
        server = HTTPServer(('0.0.0.0', self.port), CockpitHttpHandler)
        server.serve_forever()

    def control_loop(self):
        now = time.time()
        with SharedState.lock:
            # 安全看门狗: 若超过 500ms 没有收到网页端按键或摇杆指令，自动刹车
            if now - SharedState.last_cmd_time > 0.5:
                SharedState.current_linear = 0.0
                SharedState.current_angular = 0.0

            v = SharedState.current_linear
            w = SharedState.current_angular

        # 持续发布 Twist 指令到底盘
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
            roll = SharedState.horizon_angle

        # 1. 底图选择：使用外部物理摄像头画面或动态合成暗黑科技背景
        if self.external_frame is not None:
            frame = cv2.resize(self.external_frame, (width, height))
        else:
            # 纯数字孪生合成画布
            frame = np.zeros((height, width, 3), dtype=np.uint8)
            frame[:] = (12, 16, 24) # 深邃夜间底色

            # 绘制 3D 动态透视地平面网格 (赛车与飞机座舱视角)
            horizon_y = int(height * 0.55)
            # 地面半透明暗色
            cv2.rectangle(frame, (0, horizon_y), (width, height), (8, 12, 18), -1)

            # 透视纵向网格线 (向无穷远地平线收敛)
            center_x = width // 2
            for x_line in range(-800, 801, 80):
                pt_far = (center_x + int(x_line * 0.15), horizon_y)
                pt_near = (center_x + int(x_line * 1.8), height)
                cv2.line(frame, pt_far, pt_near, (28, 42, 60), 1, cv2.LINE_AA)

            # 横向动态位移网格线 (随前进速度向后倒退)
            for d in range(0, height - horizon_y, 40):
                curr_y = horizon_y + int(((d + grid_offset) % (height - horizon_y)) ** 1.35 * 0.28)
                if curr_y < height:
                    alpha = (curr_y - horizon_y) / float(height - horizon_y)
                    color = (int(30 * alpha), int(60 * alpha), int(75 * alpha))
                    cv2.line(frame, (0, curr_y), (width, curr_y), color, 1, cv2.LINE_AA)

        # 2. 叠加绿色高科技 HUD 航电仪表 (Heads-Up Display)
        hud_color = (204, 255, 0)  # 青绿霓虹色 (BGR: 0, 255, 204)
        amber_color = (0, 180, 255) # 琥珀金

        cx, cy = width // 2, height // 2

        # 准星 Crosshair 与中心指示圆
        cv2.circle(frame, (cx, cy), 28, hud_color, 1, cv2.LINE_AA)
        cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1, cv2.LINE_AA)
        cv2.line(frame, (cx - 45, cy), (cx - 15, cy), hud_color, 1, cv2.LINE_AA)
        cv2.line(frame, (cx + 15, cy), (cx + 45, cy), hud_color, 1, cv2.LINE_AA)
        cv2.line(frame, (cx, cy - 35), (cx, cy - 12), hud_color, 1, cv2.LINE_AA)
        cv2.line(frame, (cx, cy + 12), (cx, cy + 35), hud_color, 1, cv2.LINE_AA)

        # 人工水平仪 (Artificial Horizon) - 随转向角速度动态旋转倾斜
        rad = math.radians(-w * 15.0)
        cos_r = math.cos(rad)
        sin_r = math.sin(rad)
        h_len = 160
        pt1 = (int(cx - h_len * cos_r), int(cy - h_len * sin_r))
        pt2 = (int(cx + h_len * cos_r), int(cy + h_len * sin_r))
        cv2.line(frame, pt1, pt2, (0, 255, 120), 2, cv2.LINE_AA)

        # 虚拟目标识别框 (模拟具身目标检测 AI 算法)
        box_x = int(cx + 180 * math.sin(time.time() * 0.8))
        box_y = int(cy - 60 + 20 * math.cos(time.time() * 1.2))
        cv2.rectangle(frame, (box_x - 35, box_y - 35), (box_x + 35, box_y + 35), amber_color, 1, cv2.LINE_AA)
        cv2.putText(frame, "TARGET: WAYPOINT", (box_x - 45, box_y - 42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, amber_color, 1, cv2.LINE_AA)

        # 顶部与边角遥测文本状态
        cv2.putText(frame, "ROS 2 TELEOP FPV // LIVE FEED", (30, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, f"LINEAR SPD : {v:+.2f} m/s", (30, 85),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 204), 2, cv2.LINE_AA)
        cv2.putText(frame, f"ANGULAR YAW: {w:+.2f} rad/s", (30, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 204), 2, cv2.LINE_AA)

        cv2.putText(frame, f"LATENCY: <30ms | FPS: 30.0", (width - 340, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 100), 2, cv2.LINE_AA)
        cv2.putText(frame, time.strftime("UTC %Y-%m-%d %H:%M:%S"), (width - 340, 85),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (160, 180, 200), 1, cv2.LINE_AA)

        # 3. 编码为极速 JPEG 放入共享内存缓冲区
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
