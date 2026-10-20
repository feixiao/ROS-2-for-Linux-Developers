#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace cpp_composition_demo {

class TalkerComponent : public rclcpp::Node {
public:
  explicit TalkerComponent(const rclcpp::NodeOptions & options)
  : Node("talker_component", options), count_(0) {
    // 启用进程内零拷贝支持
    pub_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

    timer_ = this->create_wall_timer(
      500ms,
      std::bind(&TalkerComponent::on_timer, this));

    RCLCPP_INFO(this->get_logger(), "TalkerComponent loaded into container!");
  }

private:
  void on_timer() {
    // 使用 std::unique_ptr 触发 Intra-Process 零拷贝传递
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Zero-copy message #" + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Component publishing (Intra-Process): '%s'", msg->data.c_str());
    pub_->publish(std::move(msg));
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

}  // namespace cpp_composition_demo

RCLCPP_COMPONENTS_REGISTER_NODE(cpp_composition_demo::TalkerComponent)
