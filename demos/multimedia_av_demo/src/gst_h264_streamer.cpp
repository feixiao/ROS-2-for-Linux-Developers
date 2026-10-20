#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

using namespace std::chrono_literals;

class GstH264StreamerNode : public rclcpp::Node {
public:
  GstH264StreamerNode() : Node("gst_h264_streamer_node"), frame_id_("camera_optical_frame"), seq_(0) {
    this->declare_parameter<std::string>("pipeline",
      "v4l2src device=/dev/video0 ! video/x-raw,width=1280,height=720,framerate=30/1 ! "
      "x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast ! appsink name=sink");

    pipeline_desc_ = this->get_parameter("pipeline").as_string();

    pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      "camera/h264_stream", 10);

    // 周期模拟高吞吐 H.264 NALU 帧分发 (30 FPS)
    timer_ = this->create_wall_timer(
      33ms,
      std::bind(&GstH264StreamerNode::on_stream_timer, this));

    RCLCPP_INFO(get_logger(), "GStreamer H.264 Hardware Streamer node initialized.");
    RCLCPP_INFO(get_logger(), "Configured Pipeline: %s", pipeline_desc_.c_str());
  }

private:
  void on_stream_timer() {
    auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
    msg->header.stamp = this->get_clock()->now();
    msg->header.frame_id = frame_id_;
    msg->format = "h264";

    // 模拟构造符合 H.264 规范的 NALU 报文头 (0x00 0x00 0x00 0x01)
    // 关键帧 (IDR NALU type 5) 与非关键帧交替生成
    bool is_keyframe = (seq_ % 30 == 0);
    uint8_t nalu_type = is_keyframe ? 0x65 : 0x41; // 0x65: IDR NALU, 0x41: non-IDR slice

    msg->data = {0x00, 0x00, 0x00, 0x01, nalu_type};
    
    // 填充编码载荷 (Payload 约 8KB 模拟高压缩比编码流)
    msg->data.resize(8192, static_cast<uint8_t>((seq_ + 0x5A) % 256));

    pub_->publish(std::move(msg));
    seq_++;
  }

  std::string pipeline_desc_;
  std::string frame_id_;
  size_t seq_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GstH264StreamerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
