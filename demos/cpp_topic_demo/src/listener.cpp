#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber() : Node("minimal_subscriber") {
    // 匹配发布者的 QoS 配置
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "chatter",
      qos_profile,
      std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "C++ MinimalSubscriber started, waiting for messages on 'chatter'...");
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
