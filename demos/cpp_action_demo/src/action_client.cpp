#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "example_interfaces/action/fibonacci.hpp"

class FibonacciActionClient : public rclcpp::Node {
public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  FibonacciActionClient() : Node("fibonacci_action_client") {
    client_ptr_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");
  }

  void send_goal(int order) {
    if (!client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after 5s.");
      return;
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = order;

    RCLCPP_INFO(this->get_logger(), "Sending goal: calculate Fibonacci up to order %d", order);

    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    
    // 1. 目标响应回调
    send_goal_options.goal_response_callback =
      [this](const GoalHandleFibonacci::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result...");
        }
      };

    // 2. 过程反馈回调
    send_goal_options.feedback_callback =
      [this](GoalHandleFibonacci::SharedPtr, const std::shared_ptr<const Fibonacci::Feedback> feedback) {
        std::stringstream ss;
        ss << "Next number in sequence: ";
        for (auto number : feedback->sequence) {
          ss << number << " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
      };

    // 3. 最终结果回调
    send_goal_options.result_callback =
      [this](const GoalHandleFibonacci::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Goal was canceled");
            return;
          default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
        std::stringstream ss;
        ss << "Result received: ";
        for (auto number : result.result->sequence) {
          ss << number << " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        rclcpp::shutdown();
      };

    client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int order = 10;
  if (argc > 1) {
    order = std::atoi(argv[1]);
  }

  auto action_client = std::make_shared<FibonacciActionClient>();
  action_client->send_goal(order);
  rclcpp::spin(action_client);
  return 0;
}
