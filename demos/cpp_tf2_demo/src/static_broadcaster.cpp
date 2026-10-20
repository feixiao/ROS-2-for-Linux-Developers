#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class StaticFrameBroadcaster : public rclcpp::Node {
public:
  StaticFrameBroadcaster() : Node("static_frame_broadcaster") {
    // 实例化静态坐标变换广播器 (发布到 /tf_static 话题, 具备 Transient Local 持久化 QoS)
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "odom";

    // 静态刚体相对安装位姿: 地图原点到里程计原点
    t.transform.translation.x = 1.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // 静态变换仅需广播一次，底层 DDS 保证后来者订阅自动补发
    static_broadcaster_->sendTransform(t);

    RCLCPP_INFO(get_logger(), "Static transform broadcasted: map -> odom (1.0, 0.0, 0.0)");
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StaticFrameBroadcaster>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
