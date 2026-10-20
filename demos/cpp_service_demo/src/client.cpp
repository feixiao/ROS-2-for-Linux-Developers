#include <chrono>
#include <cstdlib>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  if (argc != 3) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Usage: cpp_service_client X Y");
    return 1;
  }

  auto node = rclcpp::Node::make_shared("add_two_ints_client");
  auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);

  // 等待服务端上线 (超时 3 秒)
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "Service 'add_two_ints' not available, waiting again...");
  }

  // 发起异步 RPC 请求
  auto result_future = client->async_send_request(request);

  // 在独立的事件循环中等待结果，避免单线程死锁
  if (rclcpp::spin_until_future_complete(node, result_future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    RCLCPP_INFO(node->get_logger(), "Result of %ld + %ld = %ld",
                request->a, request->b, result->sum);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}
