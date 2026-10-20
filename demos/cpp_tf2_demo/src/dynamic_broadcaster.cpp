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
    auto now = this->get_clock()->now();

    // 1. 广播动态里程计变换: odom -> base_link (底盘动态运动)
    geometry_msgs::msg::TransformStamped t_odom;
    t_odom.header.stamp = now;
    t_odom.header.frame_id = "odom";
    t_odom.child_frame_id = "base_link";
    t_odom.transform.translation.x = 0.5 * std::cos(angle_);
    t_odom.transform.translation.y = 0.5 * std::sin(angle_);
    t_odom.transform.translation.z = 0.0;

    tf2::Quaternion q_odom;
    q_odom.setRPY(0.0, 0.0, angle_);
    t_odom.transform.rotation.x = q_odom.x();
    t_odom.transform.rotation.y = q_odom.y();
    t_odom.transform.rotation.z = q_odom.z();
    t_odom.transform.rotation.w = q_odom.w();
    tf_broadcaster_->sendTransform(t_odom);

    // 2. 广播传感器安装变换: base_link -> laser_frame
    geometry_msgs::msg::TransformStamped t_laser;
    t_laser.header.stamp = now;
    t_laser.header.frame_id = "base_link";
    t_laser.child_frame_id = "laser_frame";
    t_laser.transform.translation.x = 0.2;
    t_laser.transform.translation.y = 0.0;
    t_laser.transform.translation.z = 0.25;

    tf2::Quaternion q_laser;
    q_laser.setRPY(0.0, 0.0, 0.0);
    t_laser.transform.rotation.x = q_laser.x();
    t_laser.transform.rotation.y = q_laser.y();
    t_laser.transform.rotation.z = q_laser.z();
    t_laser.transform.rotation.w = q_laser.w();
    tf_broadcaster_->sendTransform(t_laser);

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
