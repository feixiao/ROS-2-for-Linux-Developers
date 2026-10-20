#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cpp_ros2_control_demo {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class DiffDriveHardware : public hardware_interface::SystemInterface {
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }

    hw_positions_.resize(info_.joints.size(), 0.0);
    hw_velocities_.resize(info_.joints.size(), 0.0);
    hw_commands_.resize(info_.joints.size(), 0.0);

    for (const hardware_interface::ComponentInfo & joint : info_.joints) {
      // 校验关节命令接口包含 velocity
      if (joint.command_interfaces.size() != 1 ||
          joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_FATAL(
          rclcpp::get_logger("DiffDriveHardware"),
          "Joint '%s' requires single command interface '%s'",
          joint.name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return CallbackReturn::ERROR;
      }

      // 校验关节状态接口包含 position 和 velocity
      if (joint.state_interfaces.size() != 2) {
        RCLCPP_FATAL(
          rclcpp::get_logger("DiffDriveHardware"),
          "Joint '%s' requires 2 state interfaces (position, velocity)",
          joint.name.c_str());
        return CallbackReturn::ERROR;
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Hardware SystemInterface successfully initialized.");
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }
    return command_interfaces;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override {
    (void)previous_state;
    // 激活阶段: 硬件通信通道建立 (如打开 CAN Socket 或串口句柄)
    for (size_t i = 0; i < hw_commands_.size(); i++) {
      hw_commands_[i] = 0.0;
      hw_positions_[i] = 0.0;
      hw_velocities_[i] = 0.0;
    }
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Hardware activated. Ready for real-time control loop.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override {
    (void)previous_state;
    // 停用阶段: 向电机下发刹车零速指令并关闭句柄
    for (size_t i = 0; i < hw_commands_.size(); i++) {
      hw_commands_[i] = 0.0;
    }
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Hardware deactivated safely.");
    return CallbackReturn::SUCCESS;
  }

  // 硬实时控制周期读取函数 (严禁任何动态内存分配与阻塞系统调用)
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override {
    (void)time;
    double dt = period.seconds();

    // 模拟从硬件编码器读取真实轮速并积分更新位置
    for (size_t i = 0; i < hw_commands_.size(); i++) {
      hw_velocities_[i] = hw_commands_[i]; // 理想电机响应
      hw_positions_[i] += hw_velocities_[i] * dt;
    }

    return hardware_interface::return_type::OK;
  }

  // 硬实时控制周期写入函数 (向驱动器下发目标速度)
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override {
    (void)time;
    (void)period;

    // 在实际生产中，在此处通过已打开的 CAN 文件描述符写入 can_frame
    return hardware_interface::return_type::OK;
  }

private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
};

}  // namespace cpp_ros2_control_demo

PLUGINLIB_EXPORT_CLASS(
  cpp_ros2_control_demo::DiffDriveHardware,
  hardware_interface::SystemInterface
)
