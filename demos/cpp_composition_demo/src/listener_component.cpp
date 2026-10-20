#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

namespace cpp_composition_demo {

class ListenerComponent : public rclcpp::Node {
public:
  explicit ListenerComponent(const rclcpp::NodeOptions & options)
  : Node("listener_component", options) {
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "chatter",
      10,
      std::bind(&ListenerComponent::on_message, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "ListenerComponent loaded into container!");
  }

private:
  void on_message(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Component received message directly: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

}  // namespace cpp_composition_demo

RCLCPP_COMPONENTS_REGISTER_NODE(cpp_composition_demo::ListenerComponent)
