#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// ============================================================================
// BehaviorTree.CPP 核心抽象框架与数据结构 (自包含现代 C++ 设计模式)
// ============================================================================
namespace BT {

enum class NodeStatus {
  IDLE,
  RUNNING,
  SUCCESS,
  FAILURE
};

// 黑板机制 (Blackboard): 行为树跨节点全局共享键值字典
class Blackboard {
public:
  using Ptr = std::shared_ptr<Blackboard>;
  static Ptr create() { return std::make_shared<Blackboard>(); }

  template <typename T>
  void set(const std::string & key, const T & value) {
    storage_[key] = std::to_string(value);
  }

  double get_double(const std::string & key, double default_val = 0.0) const {
    auto it = storage_.find(key);
    if (it != storage_.end()) {
      return std::stod(it->second);
    }
    return default_val;
  }

  std::string get_string(const std::string & key, const std::string & default_val = "") const {
    auto it = storage_.find(key);
    if (it != storage_.end()) {
      return it->second;
    }
    return default_val;
  }

private:
  std::unordered_map<std::string, std::string> storage_;
};

// 树节点基类
class TreeNode {
public:
  explicit TreeNode(const std::string & name) : name_(name), status_(NodeStatus::IDLE) {}
  virtual ~TreeNode() = default;

  virtual NodeStatus executeTick() = 0;
  NodeStatus status() const { return status_; }
  const std::string & name() const { return name_; }

protected:
  std::string name_;
  NodeStatus status_;
};

// 条件节点: 瞬时返回 SUCCESS 或 FAILURE
class ConditionNode : public TreeNode {
public:
  using TreeNode::TreeNode;
};

// 异步动作节点: 支持周期返回 RUNNING 并与 ROS 2 异步事件联动
class AsyncActionNode : public TreeNode {
public:
  using TreeNode::TreeNode;
};

// 控制流节点: 回退选择器 (Fallback / Selector)
// 只要有一个子节点成功则返回 SUCCESS，全部失败返回 FAILURE
class FallbackNode : public TreeNode {
public:
  explicit FallbackNode(const std::string & name) : TreeNode(name) {}

  void addChild(std::shared_ptr<TreeNode> child) {
    children_.push_back(child);
  }

  NodeStatus executeTick() override {
    for (auto & child : children_) {
      auto status = child->executeTick();
      if (status == NodeStatus::RUNNING) {
        return NodeStatus::RUNNING;
      }
      if (status == NodeStatus::SUCCESS) {
        return NodeStatus::SUCCESS;
      }
    }
    return NodeStatus::FAILURE;
  }

private:
  std::vector<std::shared_ptr<TreeNode>> children_;
};

// 控制流节点: 顺序执行器 (Sequence)
// 只要有一个子节点失败则返回 FAILURE，全部成功返回 SUCCESS
class SequenceNode : public TreeNode {
public:
  explicit SequenceNode(const std::string & name) : TreeNode(name) {}

  void addChild(std::shared_ptr<TreeNode> child) {
    children_.push_back(child);
  }

  NodeStatus executeTick() override {
    for (auto & child : children_) {
      auto status = child->executeTick();
      if (status == NodeStatus::RUNNING) {
        return NodeStatus::RUNNING;
      }
      if (status == NodeStatus::FAILURE) {
        return NodeStatus::FAILURE;
      }
    }
    return NodeStatus::SUCCESS;
  }

private:
  std::vector<std::shared_ptr<TreeNode>> children_;
};

}  // namespace BT

// ============================================================================
// 自定义业务行为树节点 (Custom Robotics BT Nodes)
// ============================================================================

// 1. 电量检查条件节点
class CheckBatteryCondition : public BT::ConditionNode {
public:
  CheckBatteryCondition(const std::string & name, BT::Blackboard::Ptr bb)
  : BT::ConditionNode(name), bb_(bb) {}

  BT::NodeStatus executeTick() override {
    double battery = bb_->get_double("battery_level", 100.0);
    if (battery < 20.0) {
      std::cout << "[BT Condition] Battery is LOW (" << battery << "% < 20.0%) -> RETURN SUCCESS\n";
      return BT::NodeStatus::SUCCESS;
    }
    std::cout << "[BT Condition] Battery is OK (" << battery << "%) -> RETURN FAILURE\n";
    return BT::NodeStatus::FAILURE;
  }

private:
  BT::Blackboard::Ptr bb_;
};

// 2. 导航到目标点异步动作节点
class NavigateToGoalAction : public BT::AsyncActionNode {
public:
  NavigateToGoalAction(const std::string & name, const std::string & goal, BT::Blackboard::Ptr bb)
  : BT::AsyncActionNode(name), goal_(goal), bb_(bb), step_(0) {}

  BT::NodeStatus executeTick() override {
    if (step_ < 3) {
      std::cout << "[BT Action] Navigating to '" << goal_ << "' [Progress: " << (step_ + 1) * 33 << "%] -> RUNNING\n";
      step_++;
      return BT::NodeStatus::RUNNING;
    }
    std::cout << "[BT Action] Successfully arrived at '" << goal_ << "'! -> SUCCESS\n";
    step_ = 0;
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::string goal_;
  BT::Blackboard::Ptr bb_;
  int step_;
};

// 3. 充电对接动作节点
class DockAndChargeAction : public BT::AsyncActionNode {
public:
  DockAndChargeAction(const std::string & name, BT::Blackboard::Ptr bb)
  : BT::AsyncActionNode(name), bb_(bb) {}

  BT::NodeStatus executeTick() override {
    std::cout << "[BT Action] Docked at charging station. Recharging battery to 100%...\n";
    bb_->set("battery_level", 100.0);
    return BT::NodeStatus::SUCCESS;
  }

private:
  BT::Blackboard::Ptr bb_;
};

// ============================================================================
// ROS 2 任务决策总控节点 (Mission Executive Node)
// ============================================================================
class MissionExecutiveNode : public rclcpp::Node {
public:
  MissionExecutiveNode() : Node("mission_executive_node"), tick_count_(0) {
    // 1. 初始化黑板
    blackboard_ = BT::Blackboard::create();
    blackboard_->set("battery_level", 15.0); // 初始模拟低电量以触发自愈充电

    // 2. 组装行为树 (按 robot_mission_tree.xml 拓扑)
    build_behavior_tree();

    // 3. 周期 Tick 调度循环 (1Hz)
    timer_ = create_wall_timer(
      1s,
      std::bind(&MissionExecutiveNode::tick_tree, this));

    RCLCPP_INFO(get_logger(), "BehaviorTree.CPP Mission Executive started.");
  }

private:
  void build_behavior_tree() {
    root_fallback_ = std::make_shared<BT::FallbackNode>("root_safety_fallback");

    // 分支 1: 低电量自愈
    auto charge_seq = std::make_shared<BT::SequenceNode>("charge_sequence");
    charge_seq->addChild(std::make_shared<CheckBatteryCondition>("is_battery_low", blackboard_));
    charge_seq->addChild(std::make_shared<NavigateToGoalAction>("nav_to_dock", "Dock_01", blackboard_));
    charge_seq->addChild(std::make_shared<DockAndChargeAction>("dock_charge", blackboard_));

    // 分支 2: 常规作业
    auto patrol_seq = std::make_shared<BT::SequenceNode>("patrol_sequence");
    patrol_seq->addChild(std::make_shared<NavigateToGoalAction>("nav_zone_a", "Inspection_Zone_A", blackboard_));
    patrol_seq->addChild(std::make_shared<NavigateToGoalAction>("nav_zone_b", "Inspection_Zone_B", blackboard_));

    root_fallback_->addChild(charge_seq);
    root_fallback_->addChild(patrol_seq);
  }

  void tick_tree() {
    RCLCPP_INFO(get_logger(), ">>> Ticking Behavior Tree (Cycle #%zu) <<<", ++tick_count_);
    auto status = root_fallback_->executeTick();
    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(get_logger(), "Behavior Tree execution: Cycle SUCCEEDED");
    } else if (status == BT::NodeStatus::RUNNING) {
      RCLCPP_INFO(get_logger(), "Behavior Tree execution: Nodes still RUNNING");
    } else {
      RCLCPP_WARN(get_logger(), "Behavior Tree execution: Cycle FAILED");
    }
  }

  BT::Blackboard::Ptr blackboard_;
  std::shared_ptr<BT::FallbackNode> root_fallback_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t tick_count_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionExecutiveNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
