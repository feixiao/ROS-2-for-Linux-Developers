#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "example_interfaces/action/fibonacci.hpp"

class FibonacciActionServer : public rclcpp::Node {
public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  FibonacciActionServer() : Node("fibonacci_action_server") {
    action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      "fibonacci",
      std::bind(&FibonacciActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&FibonacciActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&FibonacciActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Fibonacci Action Server initialized.");
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal) {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    if (goal->order < 0 || goal->order > 45) {
      RCLCPP_WARN(this->get_logger(), "Rejecting goal: order must be in [0, 45]");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    // 启动独立工作线程执行耗时任务，避免阻塞主事件循环
    std::thread{std::bind(&FibonacciActionServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal in worker thread...");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & sequence = feedback->sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // 检查客户端是否发起了取消请求
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled by client");
        return;
      }
      // 计算下一个斐波那契数
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // 推送中间进度 Feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Published feedback progress: len=%zu", sequence.size());

      loop_rate.sleep();
    }

    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
    }
  }

  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FibonacciActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
