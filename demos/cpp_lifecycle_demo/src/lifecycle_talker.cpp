#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ManagedLifecycleTalker : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit ManagedLifecycleTalker(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : LifecycleNode("managed_lifecycle_talker", options), count_(0) {
    RCLCPP_INFO(get_logger(), "Lifecycle Node instantiated: state is UNCONFIGURED");
  }

  // 1. 配置阶段: 从 Unconfigured 跃迁至 Inactive
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override {
    (void)state;
    RCLCPP_INFO(get_logger(), "on_configure() called: allocating resources, parsing parameters...");

    // 创建受生命周期管辖的发布者 (处于 Inactive 状态时即使 publish 也不会发出任何报文)
    pub_ = this->create_publisher<std_msgs::msg::String>("lifecycle_chatter", 10);

    timer_ = this->create_wall_timer(
      1s,
      std::bind(&ManagedLifecycleTalker::on_timer, this));

    return CallbackReturn::SUCCESS;
  }

  // 2. 激活阶段: 从 Inactive 跃迁至 Active
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override {
    (void)state;
    RCLCPP_INFO(get_logger(), "on_activate() called: enabling publisher, starting active transmission!");

    // 显式激活发布者通道
    pub_->on_activate();

    return CallbackReturn::SUCCESS;
  }

  // 3. 停用阶段: 从 Active 回退至 Inactive
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override {
    (void)state;
    RCLCPP_INFO(get_logger(), "on_deactivate() called: pausing transmission...");

    // 禁用发布者
    pub_->on_deactivate();

    return CallbackReturn::SUCCESS;
  }

  // 4. 清理阶段: 从 Inactive 回退至 Unconfigured
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override {
    (void)state;
    RCLCPP_INFO(get_logger(), "on_cleanup() called: releasing timers, sockets, and buffers...");

    timer_.reset();
    pub_.reset();

    return CallbackReturn::SUCCESS;
  }

  // 5. 销毁阶段: 彻底终结
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override {
    (void)state;
    RCLCPP_INFO(get_logger(), "on_shutdown() called: finalizing node...");

    timer_.reset();
    pub_.reset();

    return CallbackReturn::SUCCESS;
  }

private:
  void on_timer() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Lifecycle telemetry frame #" + std::to_string(count_++);

    // 检查发布者当前是否处于激活状态
    if (!pub_->is_activated()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Publisher is inactive; message discarded locally.");
      return;
    }

    RCLCPP_INFO(get_logger(), "Publishing active message: '%s'", msg->data.c_str());
    pub_->publish(std::move(msg));
  }

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManagedLifecycleTalker>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
