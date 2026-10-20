#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class CameraPublisherNode : public rclcpp::Node {
public:
  CameraPublisherNode() : Node("camera_publisher_node"), count_(0) {
    // 1. 声明参数
    this->declare_parameter<int>("device_id", 0);
    this->declare_parameter<int>("width", 1280);
    this->declare_parameter<int>("height", 720);
    this->declare_parameter<double>("frame_rate", 30.0);
    this->declare_parameter<std::string>("frame_id", "camera_optical_frame");

    int width = this->get_parameter("width").as_int();
    int height = this->get_parameter("height").as_int();
    double fps = this->get_parameter("frame_rate").as_double();
    frame_id_ = this->get_parameter("frame_id").as_string();

    // 2. 初始化 image_transport 专用发布器 (自动生成 raw / compressed 话题)
    image_transport::ImageTransport it(shared_from_this());
    camera_pub_ = it.advertiseCamera("camera/image_raw", 10);

    // 3. 构建模拟相机标定内参 (CameraInfo)
    setup_camera_info(width, height);

    // 4. 定时器周期推流 (30 FPS)
    auto timer_period = std::chrono::duration<double>(1.0 / fps);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&CameraPublisherNode::publish_frame, this));

    RCLCPP_INFO(get_logger(), "CameraPublisher started: %dx%d @ %.1f FPS, frame_id='%s'",
                width, height, fps, frame_id_.c_str());
  }

private:
  void setup_camera_info(int width, int height) {
    camera_info_.header.frame_id = frame_id_;
    camera_info_.width = width;
    camera_info_.height = height;
    camera_info_.distortion_model = "plumb_bob";

    // 畸变参数 D [k1, k2, p1, p2, k3]
    camera_info_.d = {0.0, 0.0, 0.0, 0.0, 0.0};

    // 3x3 内参矩阵 K: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    double fx = 600.0, fy = 600.0, cx = width / 2.0, cy = height / 2.0;
    camera_info_.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};

    // 3x4 投影矩阵 P
    camera_info_.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  }

  void publish_frame() {
    auto stamp = this->get_clock()->now();

    // 构建图像数据 (在无物理摄像头的 CI/容器环境下，生成渐变彩条测试图)
    int width = camera_info_.width;
    int height = camera_info_.height;
    auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
    img_msg->header.stamp = stamp;
    img_msg->header.frame_id = frame_id_;
    img_msg->height = height;
    img_msg->width = width;
    img_msg->encoding = "rgb8";
    img_msg->is_bigendian = 0;
    img_msg->step = width * 3; // stride 步长 = width * 3 channels

    img_msg->data.resize(img_msg->step * height);
    uint8_t base_color = static_cast<uint8_t>((count_ * 5) % 255);
    for (size_t y = 0; y < static_cast<size_t>(height); ++y) {
      for (size_t x = 0; x < static_cast<size_t>(width); ++x) {
        size_t idx = y * img_msg->step + x * 3;
        img_msg->data[idx] = static_cast<uint8_t>((x + base_color) % 256);     // R
        img_msg->data[idx + 1] = static_cast<uint8_t>((y + base_color) % 256); // G
        img_msg->data[idx + 2] = base_color;                                   // B
      }
    }

    camera_info_.header.stamp = stamp;

    // 同时原子发布图像与相机内参
    camera_pub_.publish(*img_msg, camera_info_);
    count_++;
  }

  std::string frame_id_;
  image_transport::CameraPublisher camera_pub_;
  sensor_msgs::msg::CameraInfo camera_info_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
