// Copyright 2026 bird
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_behavior_tree/behavior_tree_engine.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace behavior_ext_plugins
{

class FollowPathRecoveryBtNode : public rclcpp::Node
{
public:
  // 初始化可复用的 FollowPath 恢复行为树执行器，行为树本身在 action server 就绪后延迟创建。
  FollowPathRecoveryBtNode()
  : Node("follow_path_recovery_bt_node"),
    last_path_stamp_(0, 0, get_clock()->get_clock_type())
  {
    declareParameters();
    loadParameters();

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, 10,
      std::bind(&FollowPathRecoveryBtNode::pathCallback, this, std::placeholders::_1));
    if (!path_valid_topic_.empty()) {
      path_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
        path_valid_topic_, 10,
        std::bind(&FollowPathRecoveryBtNode::pathValidCallback, this, std::placeholders::_1));
    }
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

    const auto tick_period = std::chrono::milliseconds(std::max(10, tick_period_ms_));
    timer_ = create_wall_timer(
      tick_period, std::bind(&FollowPathRecoveryBtNode::tickTree, this));
  }

  // 在 ROS 上下文销毁前释放行为树和黑板，打破黑板持有节点共享指针形成的引用环。
  void shutdownTree()
  {
    timer_.reset();
    if (rclcpp::ok() && tree_) {
      tree_->haltTree();
    }
    tree_.reset();
    blackboard_.reset();
    engine_.reset();
  }

private:
  // 声明路径输入、动作名称以及脱困距离等运行参数。
  void declareParameters()
  {
    declare_parameter<std::string>("path_topic", "/follow_path");
    declare_parameter<std::string>("status_topic", "/follow/recovery_status");
    declare_parameter<std::string>("path_valid_topic", "");
    declare_parameter<std::string>("bt_xml", "");
    declare_parameter<std::string>("controller_id", "FollowPath");
    declare_parameter<std::string>("goal_checker_id", "general_goal_checker");
    declare_parameter<std::string>("follow_path_action", "/follow_path");
    declare_parameter<std::string>("backup_action", "/backup");
    declare_parameter<int>("tick_period_ms", 50);
    declare_parameter<int>("server_timeout_ms", 20);
    declare_parameter<int>("wait_for_service_timeout_ms", 1000);
    declare_parameter<int>("recovery_retries", 2);
    declare_parameter<double>("recovery_distance_m", 0.30);
    declare_parameter<double>("recovery_speed_mps", 0.12);
    declare_parameter<double>("recovery_time_allowance_sec", 5.0);
    declare_parameter<double>("path_timeout_sec", 0.0);
    declare_parameter<double>("goal_update_distance_m", 0.08);
    declare_parameter<double>("goal_update_angle_rad", 0.10);
  }

  // 读取并约束参数，空 XML 路径自动使用本包随附行为树。
  void loadParameters()
  {
    path_topic_ = get_parameter("path_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
    path_valid_topic_ = get_parameter("path_valid_topic").as_string();
    bt_xml_ = get_parameter("bt_xml").as_string();
    controller_id_ = get_parameter("controller_id").as_string();
    goal_checker_id_ = get_parameter("goal_checker_id").as_string();
    follow_path_action_ = get_parameter("follow_path_action").as_string();
    backup_action_ = get_parameter("backup_action").as_string();
    tick_period_ms_ = static_cast<int>(get_parameter("tick_period_ms").as_int());
    server_timeout_ms_ = static_cast<int>(get_parameter("server_timeout_ms").as_int());
    wait_for_service_timeout_ms_ =
      static_cast<int>(get_parameter("wait_for_service_timeout_ms").as_int());
    recovery_retries_ = static_cast<int>(get_parameter("recovery_retries").as_int());
    recovery_distance_m_ = std::max(0.0, get_parameter("recovery_distance_m").as_double());
    recovery_speed_mps_ = std::max(0.0, get_parameter("recovery_speed_mps").as_double());
    recovery_time_allowance_sec_ =
      std::max(0.1, get_parameter("recovery_time_allowance_sec").as_double());
    path_timeout_sec_ = std::max(0.0, get_parameter("path_timeout_sec").as_double());
    goal_update_distance_m_ =
      std::max(0.0, get_parameter("goal_update_distance_m").as_double());
    goal_update_angle_rad_ =
      std::max(0.0, get_parameter("goal_update_angle_rad").as_double());

    if (bt_xml_.empty()) {
      bt_xml_ = ament_index_cpp::get_package_share_directory("behavior_ext_plugins") +
        "/behavior_trees/follow_path_with_free_space_recovery.xml";
    }
  }

  // 接收最新路径；空路径会立即撤销控制或恢复动作，避免继续执行旧目标。
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    last_path_stamp_ = now();
    if (msg->poses.empty()) {
      stopTree("idle: empty path");
      terminal_goal_.reset();
      return;
    }

    if (terminal_goal_ && !goalChanged(*terminal_goal_, *msg)) {
      return;
    }

    if (active_path_ && !goalChanged(*active_path_, *msg)) {
      return;
    }

    active_path_ = *msg;
    terminal_goal_.reset();
    if (blackboard_) {
      // 只在目标端点发生有效变化时更新黑板，避免路径时间戳导致 FollowPath 高频抢占。
      blackboard_->set<nav_msgs::msg::Path>("path", *active_path_);
    }
  }

  // 可选路径有效性为 false 时立即撤销行为树，供带独立有效性话题的规划器使用。
  void pathValidCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) {
      stopTree("idle: path marked invalid");
      terminal_goal_.reset();
    }
  }

  // 比较路径目标端点和终点朝向，忽略机器人当前位置引起的起点变化。
  bool goalChanged(const nav_msgs::msg::Path & lhs, const nav_msgs::msg::Path & rhs) const
  {
    if (lhs.poses.empty() || rhs.poses.empty()) {
      return lhs.poses.size() != rhs.poses.size();
    }
    const auto & lhs_goal = lhs.poses.back().pose;
    const auto & rhs_goal = rhs.poses.back().pose;
    const double distance = std::hypot(
      lhs_goal.position.x - rhs_goal.position.x,
      lhs_goal.position.y - rhs_goal.position.y);
    const double angle = std::abs(
      normalizeAngle(
        tf2::getYaw(lhs_goal.orientation) - tf2::getYaw(rhs_goal.orientation)));
    return distance >= goal_update_distance_m_ || angle >= goal_update_angle_rad_;
  }

  // 在 action server 启动后创建行为树，并把运行参数写入黑板。
  bool initializeTree()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    if (
      last_tree_init_attempt_ != std::chrono::steady_clock::time_point{} &&
      steady_now - last_tree_init_attempt_ < std::chrono::seconds(1))
    {
      return false;
    }
    // 使用稳态时钟节流初始化，避免 use_sim_time 尚无 /clock 时永远无法创建行为树。
    last_tree_init_attempt_ = steady_now;

    try {
      const std::vector<std::string> plugin_libraries = {
        "nav2_follow_path_action_bt_node",
        "nav2_back_up_action_bt_node",
        "nav2_recovery_node_bt_node",
      };
      engine_ = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(plugin_libraries);
      blackboard_ = BT::Blackboard::create();
      blackboard_->set<rclcpp::Node::SharedPtr>("node", shared_from_this());
      blackboard_->set<std::chrono::milliseconds>(
        "bt_loop_duration", std::chrono::milliseconds(std::max(10, tick_period_ms_)));
      blackboard_->set<std::chrono::milliseconds>(
        "server_timeout", std::chrono::milliseconds(std::max(1, server_timeout_ms_)));
      blackboard_->set<std::chrono::milliseconds>(
        "wait_for_service_timeout",
        std::chrono::milliseconds(std::max(1, wait_for_service_timeout_ms_)));
      blackboard_->set<int>("recovery_retries", std::max(0, recovery_retries_));
      blackboard_->set<double>("recovery_distance_m", recovery_distance_m_);
      blackboard_->set<double>("recovery_speed_mps", recovery_speed_mps_);
      blackboard_->set<double>("recovery_time_allowance_sec", recovery_time_allowance_sec_);
      blackboard_->set<std::string>("controller_id", controller_id_);
      blackboard_->set<std::string>("goal_checker_id", goal_checker_id_);
      blackboard_->set<std::string>("follow_path_action", follow_path_action_);
      blackboard_->set<std::string>("backup_action", backup_action_);
      if (active_path_) {
        blackboard_->set<nav_msgs::msg::Path>("path", *active_path_);
      }
      tree_ = std::make_unique<BT::Tree>(engine_->createTreeFromFile(bt_xml_, blackboard_));
      publishStatus("ready: recovery behavior tree");
      return true;
    } catch (const std::exception & exception) {
      tree_.reset();
      blackboard_.reset();
      engine_.reset();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "恢复行为树等待 action server: %s", exception.what());
      return false;
    }
  }

  // 周期执行行为树：控制失败时由 RecoveryNode 自动运行空闲方向 BackUp，再重试路径。
  void tickTree()
  {
    if (!active_path_) {
      return;
    }
    if (path_timeout_sec_ > 0.0 && (now() - last_path_stamp_).seconds() > path_timeout_sec_) {
      stopTree("stop: path timeout");
      return;
    }
    if (!tree_ && !initializeTree()) {
      return;
    }

    try {
      const BT::NodeStatus result = tree_->tickRoot();
      if (recoveryRunning()) {
        publishStatus("recovering: moving toward local free space");
      } else if (result == BT::NodeStatus::RUNNING) {
        publishStatus("tracking: FollowPath active");
      } else if (result == BT::NodeStatus::SUCCESS) {
        terminal_goal_ = active_path_;
        active_path_.reset();
        tree_->haltTree();
        publishStatus("succeeded: FollowPath completed");
      } else if (result == BT::NodeStatus::FAILURE) {
        terminal_goal_ = active_path_;
        active_path_.reset();
        tree_->haltTree();
        publishStatus("failed: recovery retries exhausted");
      }
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "恢复行为树执行失败: %s", exception.what());
      stopTree("failed: behavior tree exception");
    }
  }

  // 检查 BackUp 节点是否正在执行，用于输出可观测的恢复状态。
  bool recoveryRunning() const
  {
    if (!tree_) {
      return false;
    }
    for (const auto & node : tree_->nodes) {
      if (node->name() == "BackUpFreeSpace" && node->status() == BT::NodeStatus::RUNNING) {
        return true;
      }
    }
    return false;
  }

  // 撤销行为树内仍在运行的 action，并清空当前路径。
  void stopTree(const std::string & status)
  {
    if (tree_) {
      tree_->haltTree();
    }
    active_path_.reset();
    publishStatus(status);
  }

  // 仅在内容变化时发布状态，降低日志和话题噪声。
  void publishStatus(const std::string & status)
  {
    if (status == last_status_) {
      return;
    }
    last_status_ = status;
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "%s", status.c_str());
  }

  // 将角度归一化到 [-pi, pi]，用于判断终点朝向是否变化。
  static double normalizeAngle(const double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  std::string path_topic_;
  std::string status_topic_;
  std::string path_valid_topic_;
  std::string bt_xml_;
  std::string controller_id_;
  std::string goal_checker_id_;
  std::string follow_path_action_;
  std::string backup_action_;
  int tick_period_ms_{50};
  int server_timeout_ms_{20};
  int wait_for_service_timeout_ms_{1000};
  int recovery_retries_{2};
  double recovery_distance_m_{0.30};
  double recovery_speed_mps_{0.12};
  double recovery_time_allowance_sec_{5.0};
  double path_timeout_sec_{0.0};
  double goal_update_distance_m_{0.08};
  double goal_update_angle_rad_{0.10};

  rclcpp::Time last_path_stamp_;
  std::chrono::steady_clock::time_point last_tree_init_attempt_;
  std::optional<nav_msgs::msg::Path> active_path_;
  std::optional<nav_msgs::msg::Path> terminal_goal_;
  std::string last_status_;
  BT::Blackboard::Ptr blackboard_;
  std::unique_ptr<nav2_behavior_tree::BehaviorTreeEngine> engine_;
  std::unique_ptr<BT::Tree> tree_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr path_valid_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace behavior_ext_plugins

// 创建行为树执行节点并进入 ROS 事件循环。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<behavior_ext_plugins::FollowPathRecoveryBtNode>();
  rclcpp::spin(node);
  node->shutdownTree();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
