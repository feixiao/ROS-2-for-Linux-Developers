#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    // 声明参数: 发布频率 (默认 2.0 Hz)
    this->declare_parameter<double>("publish_frequency", 2.0);
    double freq = this->get_parameter("publish_frequency").as_double();

    // 配置 QoS: 历史深度 10, 可靠传输 (Reliable), 易失性 (Volatile)
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", qos_profile);

    auto timer_period = std::chrono::duration<double>(1.0 / freq);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&MinimalPublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "C++ MinimalPublisher initialized at %.1f Hz", freq);
  }

private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello from Linux ROS 2 C++ node! Count: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
