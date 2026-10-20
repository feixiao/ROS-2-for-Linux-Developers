#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node {
public:
  FrameListener() : Node("frame_listener") {
    // 1. 初始化 tf2 缓冲区 (默认保存最近 10 秒的时空变换)
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    // 2. 挂载监听器，异步消费 /tf 与 /tf_static 并填入 buffer
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 3. 周期查询跨坐标系变换
    timer_ = this->create_wall_timer(
      500ms,
      std::bind(&FrameListener::on_timer, this));

    RCLCPP_INFO(get_logger(), "FrameListener started: querying map -> laser_frame");
  }

private:
  void on_timer() {
    geometry_msgs::msg::TransformStamped t;
    try {
      // 查询此时刻两个任意坐标系间的链式相对变换 (SE(3) 树状路径自动合成)
      t = tf_buffer_->lookupTransform(
        "map",
        "laser_frame",
        tf2::TimePointZero);  // 查询最新可用的几何变换

      RCLCPP_INFO(
        get_logger(),
        "Resolved Transform [map -> laser_frame]: Translation=(%.2f, %.2f, %.2f)",
        t.transform.translation.x,
        t.transform.translation.y,
        t.transform.translation.z);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        get_logger(),
        "Could not transform map to laser_frame: %s", ex.what());
    }
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FrameListener>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
