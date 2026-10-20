#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class DynamicFrameBroadcaster : public rclcpp::Node {
public:
  DynamicFrameBroadcaster() : Node("dynamic_frame_broadcaster"), angle_(0.0) {
    // 实例化动态坐标变换广播器 (发布到 /tf 话题)
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = create_wall_timer(
      50ms,  // 20Hz 广播频率
      std::bind(&DynamicFrameBroadcaster::broadcast_transform, this));

    RCLCPP_INFO(get_logger(), "DynamicFrameBroadcaster started: base_link -> laser_frame");
  }

private:
  void broadcast_transform() {
    geometry_msgs::msg::TransformStamped t;

    // 1. 设置时间戳与父子坐标系
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "base_link";
    t.child_frame_id = "laser_frame";

    // 2. 模拟底盘前进时的位移平移 (Translation)
    t.transform.translation.x = 0.5 * std::cos(angle_);
    t.transform.translation.y = 0.5 * std::sin(angle_);
    t.transform.translation.z = 0.25;

    // 3. 欧拉角 (Roll, Pitch, Yaw) 转四元数 (Quaternion), 杜绝万向节死锁
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, angle_);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // 4. 发送动态变换
    tf_broadcaster_->sendTransform(t);

    angle_ += 0.05;
  }

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  double angle_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DynamicFrameBroadcaster>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
