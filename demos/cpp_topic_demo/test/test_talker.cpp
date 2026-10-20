#include <gtest/gtest.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TalkerUnitTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override {
    rclcpp::shutdown();
  }
};

TEST_F(TalkerUnitTest, NodeInitializationAndParameterCheck) {
  auto node = std::make_shared<rclcpp::Node>("test_node");
  node->declare_parameter<double>("frequency", 5.0);

  double freq = node->get_parameter("frequency").as_double();
  EXPECT_DOUBLE_EQ(freq, 5.0);

  auto pub = node->create_publisher<std_msgs::msg::String>("chatter", 10);
  ASSERT_NE(pub, nullptr);
  EXPECT_EQ(pub->get_topic_name(), std::string("/chatter"));
}
