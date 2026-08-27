// Copyright 2026 OpenAI
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
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "go2_uwb_local_follow/local_planner_core.hpp"

namespace go2_uwb_local_follow
{
namespace
{

// 把浮点数格式化为诊断话题使用的短字符串。
std::string formatDouble(double value, int precision = 3)
{
  if (!std::isfinite(value)) {
    return "inf";
  }
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

// 将数值限制在给定闭区间内。
double clampValue(double value, double minimum, double maximum)
{
  return std::max(minimum, std::min(value, maximum));
}

}  // namespace

class LocalCollisionMonitorNode : public rclcpp::Node
{
public:
  // 初始化名义速度、当前帧障碍物、轨迹碰撞检查和隔离输出接口。
  explicit LocalCollisionMonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("local_collision_monitor_node", options)
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    nominal_cmd_topic_ = declare_parameter<std::string>(
      "nominal_cmd_topic", "/go2_uwb_local_follow/nominal_cmd");
    obstacle_topic_ = declare_parameter<std::string>(
      "obstacle_topic", "/local_grid_obstacle");
    planned_cmd_topic_ = declare_parameter<std::string>(
      "planned_cmd_topic", "/go2_uwb_local_follow/collision_checked_cmd");
    cmd_vel_topic_ = declare_parameter<std::string>(
      "cmd_vel_topic", "/cmd_vel_avoidance");
    predicted_path_topic_ = declare_parameter<std::string>(
      "predicted_path_topic", "/go2_uwb_local_follow/evaluated_path");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/go2_uwb_local_follow/collision_diagnostics");

    enable_motion_ = declare_parameter<bool>("enable_motion", false);
    control_frequency_ = declare_parameter<double>("control_frequency", 20.0);
    diagnostic_frequency_ = declare_parameter<double>("diagnostic_frequency", 2.0);
    nominal_timeout_sec_ = declare_parameter<double>("nominal_timeout_sec", 0.20);
    obstacle_timeout_sec_ = declare_parameter<double>("obstacle_timeout_sec", 0.30);
    max_linear_speed_ = declare_parameter<double>("max_linear_speed", 0.80);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 2.00);

    trajectory_config_.prediction_time = declare_parameter<double>("prediction_time", 1.20);
    trajectory_config_.simulation_dt = declare_parameter<double>("simulation_dt", 0.05);
    footprint_config_.robot_length = declare_parameter<double>("robot_length", 0.70);
    footprint_config_.robot_width = declare_parameter<double>("robot_width", 0.40);
    footprint_config_.safety_margin = declare_parameter<double>("safety_margin", 0.08);
    emergency_front_distance_ = declare_parameter<double>(
      "emergency_front_distance", 0.25);
    emergency_half_width_ = declare_parameter<double>("emergency_half_width", 0.30);

    obstacle_x_min_ = declare_parameter<double>("obstacle_x_min", -0.50);
    obstacle_x_max_ = declare_parameter<double>("obstacle_x_max", 3.00);
    obstacle_y_abs_max_ = declare_parameter<double>("obstacle_y_abs_max", 2.00);
    self_filter_x_min_ = declare_parameter<double>("self_filter_x_min", -0.35);
    self_filter_x_max_ = declare_parameter<double>("self_filter_x_max", 0.35);
    self_filter_y_abs_ = declare_parameter<double>("self_filter_y_abs", 0.20);
    max_obstacle_points_ = declare_parameter<int>("max_obstacle_points", 5000);
    validateParameters();

    nominal_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      nominal_cmd_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      std::bind(&LocalCollisionMonitorNode::nominalCallback, this, std::placeholders::_1));
    obstacle_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      obstacle_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalCollisionMonitorNode::obstacleCallback, this, std::placeholders::_1));

    planned_cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      planned_cmd_topic_, 10);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    predicted_path_pub_ = create_publisher<nav_msgs::msg::Path>(predicted_path_topic_, 10);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, 10);

    const auto control_period = std::chrono::duration<double>(1.0 / control_frequency_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(control_period),
      std::bind(&LocalCollisionMonitorNode::controlTick, this));
    diagnostic_period_ = std::chrono::duration<double>(1.0 / diagnostic_frequency_);

    RCLCPP_INFO(
      get_logger(),
      "Local collision monitor started: nominal=%s obstacles=%s output=%s enable_motion=%s",
      nominal_cmd_topic_.c_str(), obstacle_topic_.c_str(), cmd_vel_topic_.c_str(),
      enable_motion_ ? "true" : "false");
  }

  // 节点正常销毁前尽力发布一次零速度。
  ~LocalCollisionMonitorNode() override
  {
    if (cmd_vel_pub_) {
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    }
  }

private:
  struct NominalSnapshot
  {
    PlannerVelocity2D velocity;
    std::chrono::steady_clock::time_point receipt_time{};
    bool valid{false};
  };

  struct ObstacleSnapshot
  {
    std::shared_ptr<const std::vector<ObstaclePoint2D>> points;
    std::chrono::steady_clock::time_point receipt_time{};
    bool valid{false};
    std::string rejection_reason;
  };

  struct StatusSnapshot
  {
    std::string state{"WAIT_NOMINAL"};
    PlannerVelocity2D nominal;
    PlannerVelocity2D planned;
    double nominal_age{0.0};
    double obstacle_age{0.0};
    double min_clearance{std::numeric_limits<double>::infinity()};
    std::size_t obstacle_count{0U};
    bool collision{false};
    bool emergency{false};
  };

  // 检查话题、频率、时效、足迹和障碍过滤参数。
  void validateParameters()
  {
    std::string reason;
    if (base_frame_.empty() || nominal_cmd_topic_.empty() || obstacle_topic_.empty() ||
      planned_cmd_topic_.empty() || cmd_vel_topic_.empty())
    {
      throw std::invalid_argument("frame and topic names must not be empty");
    }
    if (!validateTrajectoryConfig(trajectory_config_, &reason) ||
      !validateFootprintConfig(footprint_config_, &reason))
    {
      throw std::invalid_argument(reason);
    }
    if (!std::isfinite(control_frequency_) || control_frequency_ <= 0.0 ||
      !std::isfinite(diagnostic_frequency_) || diagnostic_frequency_ <= 0.0)
    {
      throw std::invalid_argument("control and diagnostic frequencies must be positive");
    }
    if (!std::isfinite(nominal_timeout_sec_) || nominal_timeout_sec_ <= 0.0 ||
      !std::isfinite(obstacle_timeout_sec_) || obstacle_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument("input timeouts must be positive");
    }
    if (!std::isfinite(max_linear_speed_) || max_linear_speed_ < 0.0 ||
      !std::isfinite(max_angular_speed_) || max_angular_speed_ < 0.0)
    {
      throw std::invalid_argument("velocity limits must be finite and non-negative");
    }
    if (!std::isfinite(emergency_front_distance_) || emergency_front_distance_ < 0.0 ||
      !std::isfinite(emergency_half_width_) || emergency_half_width_ < 0.0)
    {
      throw std::invalid_argument("emergency region dimensions must be non-negative");
    }
    if (!std::isfinite(obstacle_x_min_) || !std::isfinite(obstacle_x_max_) ||
      obstacle_x_min_ >= obstacle_x_max_ || !std::isfinite(obstacle_y_abs_max_) ||
      obstacle_y_abs_max_ <= 0.0)
    {
      throw std::invalid_argument("obstacle filter bounds are invalid");
    }
    if (!std::isfinite(self_filter_x_min_) || !std::isfinite(self_filter_x_max_) ||
      self_filter_x_min_ >= self_filter_x_max_ || !std::isfinite(self_filter_y_abs_) ||
      self_filter_y_abs_ < 0.0 || max_obstacle_points_ <= 0)
    {
      throw std::invalid_argument("self filter or maximum obstacle count is invalid");
    }
  }

  // 保存最新名义速度，只接受有限的前进和转向分量。
  void nominalCallback(const geometry_msgs::msg::TwistStamped::SharedPtr message)
  {
    NominalSnapshot snapshot;
    const double linear_x = message->twist.linear.x;
    const double angular_z = message->twist.angular.z;
    snapshot.valid = std::isfinite(linear_x) && std::isfinite(angular_z);
    snapshot.velocity.linear_x = snapshot.valid ?
      clampValue(linear_x, 0.0, max_linear_speed_) : 0.0;
    snapshot.velocity.angular_z = snapshot.valid ?
      clampValue(angular_z, -max_angular_speed_, max_angular_speed_) : 0.0;
    snapshot.receipt_time = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(input_mutex_);
    nominal_snapshot_ = snapshot;
  }

  // 将当前帧 PointCloud2 过滤成 base_footprint 下的二维障碍快照。
  void obstacleCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    ObstacleSnapshot snapshot;
    snapshot.receipt_time = std::chrono::steady_clock::now();
    if (message->header.frame_id != base_frame_) {
      snapshot.rejection_reason = "obstacle frame differs from base_frame";
      storeObstacleSnapshot(std::move(snapshot));
      return;
    }

    auto points = std::make_shared<std::vector<ObstaclePoint2D>>();
    points->reserve(
      std::min<std::size_t>(
        static_cast<std::size_t>(max_obstacle_points_),
        static_cast<std::size_t>(message->width) * message->height));
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x_iterator(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y_iterator(*message, "y");
      for (; x_iterator != x_iterator.end(); ++x_iterator, ++y_iterator) {
        const double x = static_cast<double>(*x_iterator);
        const double y = static_cast<double>(*y_iterator);
        if (!std::isfinite(x) || !std::isfinite(y) ||
          x < obstacle_x_min_ || x > obstacle_x_max_ ||
          std::abs(y) > obstacle_y_abs_max_)
        {
          continue;
        }
        const bool inside_self_filter =
          x >= self_filter_x_min_ && x <= self_filter_x_max_ &&
          std::abs(y) <= self_filter_y_abs_;
        if (inside_self_filter) {
          continue;
        }
        points->push_back(ObstaclePoint2D{x, y});
        if (points->size() >= static_cast<std::size_t>(max_obstacle_points_)) {
          break;
        }
      }
    } catch (const std::runtime_error & exception) {
      snapshot.rejection_reason = exception.what();
      storeObstacleSnapshot(std::move(snapshot));
      return;
    }

    snapshot.points = std::move(points);
    snapshot.valid = true;
    storeObstacleSnapshot(std::move(snapshot));
  }

  // 用一次短锁替换共享障碍快照，避免控制循环复制整帧点云。
  void storeObstacleSnapshot(ObstacleSnapshot snapshot)
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    obstacle_snapshot_ = std::move(snapshot);
  }

  // 固定频率执行输入时效、紧急区和名义轨迹碰撞检查。
  void controlTick()
  {
    NominalSnapshot nominal;
    ObstacleSnapshot obstacle;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      nominal = nominal_snapshot_;
      obstacle = obstacle_snapshot_;
    }

    const auto current = std::chrono::steady_clock::now();
    StatusSnapshot status;
    status.nominal = nominal.velocity;
    status.nominal_age = nominal.valid ?
      std::chrono::duration<double>(current - nominal.receipt_time).count() :
      std::numeric_limits<double>::infinity();
    status.obstacle_age = obstacle.valid ?
      std::chrono::duration<double>(current - obstacle.receipt_time).count() :
      std::numeric_limits<double>::infinity();

    if (!nominal.valid || status.nominal_age > nominal_timeout_sec_) {
      publishDecision(status, "NOMINAL_TIMEOUT", {}, {});
      return;
    }
    if (!obstacle.valid) {
      const std::string state = obstacle.rejection_reason.empty() ?
        "WAIT_OBSTACLE" : "OBSTACLE_INVALID";
      publishDecision(status, state, {}, {});
      return;
    }
    if (status.obstacle_age > obstacle_timeout_sec_) {
      publishDecision(status, "SENSOR_TIMEOUT", {}, {});
      return;
    }

    const auto & points = *obstacle.points;
    status.obstacle_count = points.size();
    const auto trajectory = predictTrajectory(nominal.velocity, trajectory_config_);
    const CollisionResult collision = checkTrajectoryCollision(
      trajectory, points, footprint_config_);
    status.collision = collision.collision;
    status.min_clearance = collision.min_clearance;
    status.emergency = hasEmergencyFrontObstacle(
      points, footprint_config_, emergency_front_distance_, emergency_half_width_);

    if (status.emergency) {
      publishDecision(status, "EMERGENCY_STOP", {}, trajectory);
      return;
    }
    if (status.collision) {
      publishDecision(status, "NOMINAL_COLLISION", {}, trajectory);
      return;
    }

    status.planned = nominal.velocity;
    publishDecision(
      status, enable_motion_ ? "CLEAR" : "CLEAR_DEBUG", nominal.velocity, trajectory);
  }

  // 发布碰撞检查后的速度、隔离实机输出、预测 Path 和节流诊断。
  void publishDecision(
    StatusSnapshot status,
    const std::string & state,
    const PlannerVelocity2D & planned,
    const std::vector<PlannerPose2D> & trajectory)
  {
    status.state = state;
    status.planned = planned;

    geometry_msgs::msg::TwistStamped planned_message;
    planned_message.header.stamp = now();
    planned_message.header.frame_id = base_frame_;
    planned_message.twist.linear.x = std::max(0.0, planned.linear_x);
    planned_message.twist.angular.z = planned.angular_z;
    planned_cmd_pub_->publish(planned_message);

    geometry_msgs::msg::Twist output_message;
    if (enable_motion_) {
      output_message.linear.x = std::max(0.0, planned.linear_x);
      output_message.angular.z = planned.angular_z;
    }
    cmd_vel_pub_->publish(output_message);
    publishPath(trajectory, planned_message.header.stamp);

    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      status_snapshot_ = status;
    }
    publishDiagnosticIfDue();
  }

  // 将当前周期评估的局部预测姿态发布成 base_footprint 下的调试 Path。
  void publishPath(
    const std::vector<PlannerPose2D> & trajectory,
    const builtin_interfaces::msg::Time & stamp)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = base_frame_;
    path.poses.reserve(trajectory.size());
    for (const auto & pose : trajectory) {
      geometry_msgs::msg::PoseStamped pose_message;
      pose_message.header = path.header;
      pose_message.pose.position.x = pose.x;
      pose_message.pose.position.y = pose.y;
      pose_message.pose.orientation.z = std::sin(pose.yaw * 0.5);
      pose_message.pose.orientation.w = std::cos(pose.yaw * 0.5);
      path.poses.push_back(std::move(pose_message));
    }
    predicted_path_pub_->publish(path);
  }

  // 按限制频率发布输入时效、碰撞结果、障碍数量和速度决策。
  void publishDiagnosticIfDue()
  {
    const auto current = std::chrono::steady_clock::now();
    if (have_diagnostic_time_ && current - last_diagnostic_time_ < diagnostic_period_) {
      return;
    }
    have_diagnostic_time_ = true;
    last_diagnostic_time_ = current;

    StatusSnapshot status;
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      status = status_snapshot_;
    }
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus diagnostic;
    diagnostic.level = status.state == "CLEAR" || status.state == "CLEAR_DEBUG" ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diagnostic.name = get_fully_qualified_name() + std::string(": local collision monitor");
    diagnostic.hardware_id = "go2_stereo_local_planner";
    diagnostic.message = status.state;
    const std::pair<std::string, std::string> entries[] = {
      {"enable_motion", enable_motion_ ? "true" : "false"},
      {"nominal_age_sec", formatDouble(status.nominal_age)},
      {"obstacle_age_sec", formatDouble(status.obstacle_age)},
      {"obstacle_count", std::to_string(status.obstacle_count)},
      {"collision", status.collision ? "true" : "false"},
      {"emergency", status.emergency ? "true" : "false"},
      {"min_clearance", formatDouble(status.min_clearance)},
      {"nominal_v", formatDouble(status.nominal.linear_x)},
      {"nominal_w", formatDouble(status.nominal.angular_z)},
      {"planned_v", formatDouble(status.planned.linear_x)},
      {"planned_w", formatDouble(status.planned.angular_z)}};
    for (const auto & entry : entries) {
      diagnostic_msgs::msg::KeyValue value;
      value.key = entry.first;
      value.value = entry.second;
      diagnostic.values.push_back(std::move(value));
    }
    array.status.push_back(std::move(diagnostic));
    diagnostics_pub_->publish(array);
  }

  std::string base_frame_;
  std::string nominal_cmd_topic_;
  std::string obstacle_topic_;
  std::string planned_cmd_topic_;
  std::string cmd_vel_topic_;
  std::string predicted_path_topic_;
  std::string diagnostics_topic_;

  bool enable_motion_{false};
  double control_frequency_{20.0};
  double diagnostic_frequency_{2.0};
  double nominal_timeout_sec_{0.20};
  double obstacle_timeout_sec_{0.30};
  double max_linear_speed_{0.80};
  double max_angular_speed_{2.00};
  TrajectoryConfig trajectory_config_;
  FootprintConfig footprint_config_;
  double emergency_front_distance_{0.25};
  double emergency_half_width_{0.30};
  double obstacle_x_min_{-0.50};
  double obstacle_x_max_{3.00};
  double obstacle_y_abs_max_{2.00};
  double self_filter_x_min_{-0.35};
  double self_filter_x_max_{0.35};
  double self_filter_y_abs_{0.20};
  int max_obstacle_points_{5000};

  std::mutex input_mutex_;
  NominalSnapshot nominal_snapshot_;
  ObstacleSnapshot obstacle_snapshot_;
  std::mutex status_mutex_;
  StatusSnapshot status_snapshot_;
  std::chrono::duration<double> diagnostic_period_{0.5};
  bool have_diagnostic_time_{false};
  std::chrono::steady_clock::time_point last_diagnostic_time_{};

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr nominal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr planned_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_path_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace go2_uwb_local_follow

// 启动局部碰撞监视节点并进入 ROS 事件循环。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<go2_uwb_local_follow::LocalCollisionMonitorNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("local_collision_monitor_node"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
