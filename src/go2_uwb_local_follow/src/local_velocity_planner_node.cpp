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
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
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

class LocalVelocityPlannerNode : public rclcpp::Node
{
public:
  // 初始化名义速度、实测速度、障碍点云、速度采样和隔离输出接口。
  explicit LocalVelocityPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("local_velocity_planner_node", options)
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    odom_child_frame_ = declare_parameter<std::string>(
      "odom_child_frame", "base_footprint");
    nominal_cmd_topic_ = declare_parameter<std::string>(
      "nominal_cmd_topic", "/go2_uwb_local_follow/nominal_cmd");
    obstacle_topic_ = declare_parameter<std::string>(
      "obstacle_topic", "/local_grid_obstacle");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom_leg");
    planned_cmd_topic_ = declare_parameter<std::string>(
      "planned_cmd_topic", "/go2_uwb_local_follow/planned_cmd");
    final_cmd_topic_ = declare_parameter<std::string>(
      "final_cmd_topic", "/go2_uwb_local_follow/final_cmd");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_planned");
    selected_path_topic_ = declare_parameter<std::string>(
      "selected_path_topic", "/go2_uwb_local_follow/selected_path");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/go2_uwb_local_follow/planner_diagnostics");

    target_topic_ = declare_parameter<std::string>("target_topic", "/uwb/target_point");
    target_timeout_sec_ = declare_parameter<double>("target_timeout_sec", 0.50);
    recovery_config_.heading_tolerance = declare_parameter<double>(
      "recovery_heading_tolerance",
      0.15);
    recovery_config_.settled_command_angular = declare_parameter<double>(
      "recovery_command_angular",
      0.10);
    recovery_config_.settled_measured_angular = declare_parameter<double>(
      "recovery_measured_angular", 0.10);
    recovery_config_.turn_command_threshold = declare_parameter<double>(
      "turn_command_threshold",
      0.30);
    recovery_config_.turn_measured_threshold = declare_parameter<double>(
      "turn_measured_threshold",
      0.10);
    recovery_config_.turn_tracking_timeout =
      declare_parameter<double>("turn_tracking_timeout", 0.35);
    recovery_config_.clear_observations = declare_parameter<int>("recovery_clear_observations", 3);
    prediction_config_.response_delay = declare_parameter<double>("command_response_delay", 0.15);
    prediction_config_.angular_response_gain = declare_parameter<double>(
      "angular_response_gain",
      1.0);

    enable_motion_ = declare_parameter<bool>("enable_motion", false);
    control_frequency_ = declare_parameter<double>("control_frequency", 20.0);
    diagnostic_frequency_ = declare_parameter<double>("diagnostic_frequency", 2.0);
    nominal_timeout_sec_ = declare_parameter<double>("nominal_timeout_sec", 0.20);
    obstacle_timeout_sec_ = declare_parameter<double>("obstacle_timeout_sec", 1.00);
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 0.10);
    odom_linear_deadband_ = declare_parameter<double>("odom_linear_deadband", 0.02);
    odom_angular_deadband_ = declare_parameter<double>("odom_angular_deadband", 0.05);
    angular_stabilization_config_.velocity_tracking_kp = declare_parameter<double>(
      "angular_velocity_tracking_kp", 1.0);
    angular_stabilization_config_.command_deadband = declare_parameter<double>(
      "angular_command_deadband", 0.08);
    angular_stabilization_config_.reverse_speed_threshold = declare_parameter<double>(
      "angular_reverse_speed_threshold", 0.15);
    avoidance_direction_hold_sec_ = declare_parameter<double>(
      "avoidance_direction_hold_sec", 1.50);

    trajectory_config_.prediction_time = declare_parameter<double>("prediction_time", 1.20);
    trajectory_config_.simulation_dt = declare_parameter<double>("simulation_dt", 0.05);
    footprint_config_.robot_length = declare_parameter<double>("robot_length", 0.70);
    footprint_config_.robot_width = declare_parameter<double>("robot_width", 0.40);
    footprint_config_.safety_margin = declare_parameter<double>("safety_margin", 0.08);

    motion_limits_.min_linear_speed = declare_parameter<double>(
      "min_linear_speed", 0.23);
    motion_limits_.max_linear_speed = declare_parameter<double>(
      "max_linear_speed", 0.80);
    motion_limits_.max_reverse_speed = declare_parameter<double>(
      "max_reverse_speed", 0.23);
    motion_limits_.min_angular_speed = declare_parameter<double>(
      "min_follow_angular_speed", 0.0);
    motion_limits_.max_angular_speed = declare_parameter<double>(
      "max_angular_speed", 2.00);
    motion_limits_.max_linear_accel = declare_parameter<double>(
      "max_linear_accel", 0.80);
    motion_limits_.max_linear_decel = declare_parameter<double>(
      "max_linear_decel", 0.80);
    motion_limits_.max_angular_accel = declare_parameter<double>(
      "max_angular_accel", 2.00);

    sampling_config_.linear_samples = declare_parameter<int>("linear_samples", 9);
    sampling_config_.angular_samples = declare_parameter<int>("angular_samples", 25);
    sampling_config_.min_avoidance_angular_speed = declare_parameter<double>(
      "min_avoidance_angular_speed", 0.25);
    sampling_config_.max_avoidance_angular_speed = declare_parameter<double>(
      "max_avoidance_angular_speed", 1.50);
    sampling_config_.linear_speed_step = declare_parameter<double>("linear_speed_step", 0.10);
    sampling_config_.linear_speed_priority_scales = declare_parameter<std::vector<double>>(
      "linear_speed_priority_scales", {1.0, 0.85, 0.70, 0.50, 0.0});
    sampling_config_.obstacle_influence_distance = declare_parameter<double>(
      "obstacle_influence_distance", 0.35);
    sampling_config_.minimum_safe_clearance = declare_parameter<double>(
      "minimum_safe_clearance", 0.05);
    sampling_config_.minimum_ttc = declare_parameter<double>("minimum_ttc", 0.20);
    sampling_config_.weight_follow_linear = declare_parameter<double>(
      "weight_follow_linear", 8.0);
    sampling_config_.weight_follow_angular = declare_parameter<double>(
      "weight_follow_angular", 12.0);
    sampling_config_.weight_smooth_linear = declare_parameter<double>(
      "weight_smooth_linear", 4.0);
    sampling_config_.weight_smooth_angular = declare_parameter<double>(
      "weight_smooth_angular", 8.0);
    sampling_config_.weight_obstacle = declare_parameter<double>("weight_obstacle", 6.0);
    sampling_config_.weight_progress = declare_parameter<double>("weight_progress", 0.80);

    emergency_front_distance_ = declare_parameter<double>(
      "emergency_front_distance", 0.25);
    emergency_half_width_ = declare_parameter<double>("emergency_half_width", 0.30);
    emergency_confirm_frames_ = static_cast<int>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("emergency_confirm_frames", 2)));
    emergency_reverse_config_.enabled = declare_parameter<bool>(
      "enable_emergency_reverse", true);
    emergency_reverse_config_.speed = declare_parameter<double>(
      "emergency_reverse_speed", 0.23);
    emergency_reverse_config_.distance = declare_parameter<double>(
      "emergency_reverse_distance", 0.20);
    emergency_reverse_config_.stop_hold_sec = declare_parameter<double>(
      "emergency_reverse_stop_hold_sec", 0.0);
    emergency_reverse_config_.start_linear_speed_threshold = declare_parameter<double>(
      "emergency_reverse_start_linear_speed", 0.04);
    emergency_reverse_config_.start_angular_speed_threshold = declare_parameter<double>(
      "emergency_reverse_start_angular_speed", 0.10);
    emergency_reverse_config_.extra_safety_margin = declare_parameter<double>(
      "emergency_reverse_extra_margin", 0.05);
    obstacle_x_min_ = declare_parameter<double>("obstacle_x_min", -0.75);
    obstacle_x_max_ = declare_parameter<double>("obstacle_x_max", 3.00);
    obstacle_y_abs_max_ = declare_parameter<double>("obstacle_y_abs_max", 2.00);
    enable_self_filter_ = declare_parameter<bool>("enable_self_filter", true);
    self_filter_x_min_ = declare_parameter<double>("self_filter_x_min", -0.35);
    self_filter_x_max_ = declare_parameter<double>("self_filter_x_max", 0.35);
    self_filter_y_abs_ = declare_parameter<double>("self_filter_y_abs", 0.20);
    max_obstacle_points_ = declare_parameter<int>("max_obstacle_points", 5000);
    prediction_config_.control_period = 1.0 / control_frequency_;
    prediction_config_.control_dt = prediction_config_.control_period;
    validateParameters();

    target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      target_topic_, rclcpp::QoS(10),
      std::bind(&LocalVelocityPlannerNode::targetCallback, this, std::placeholders::_1));
    nominal_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      nominal_cmd_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      std::bind(&LocalVelocityPlannerNode::nominalCallback, this, std::placeholders::_1));
    obstacle_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      obstacle_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalVelocityPlannerNode::obstacleCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalVelocityPlannerNode::odomCallback, this, std::placeholders::_1));

    planned_cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      planned_cmd_topic_, 10);
    final_cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(final_cmd_topic_, 10);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    selected_path_pub_ = create_publisher<nav_msgs::msg::Path>(selected_path_topic_, 10);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, 10);

    const auto control_period = std::chrono::duration<double>(1.0 / control_frequency_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(control_period),
      std::bind(&LocalVelocityPlannerNode::controlTick, this));
    diagnostic_period_ = std::chrono::duration<double>(1.0 / diagnostic_frequency_);
    last_control_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(
      get_logger(),
      "Local velocity planner started: nominal=%s obstacles=%s odom=%s output=%s "
      "enable_motion=%s",
      nominal_cmd_topic_.c_str(), obstacle_topic_.c_str(), odom_topic_.c_str(),
      cmd_vel_topic_.c_str(), enable_motion_ ? "true" : "false");
  }

  // 节点正常销毁前尽力发布一次零速度。
  ~LocalVelocityPlannerNode() override
  {
    if (cmd_vel_pub_) {
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    }
  }

private:
  enum class EmergencyRecoveryState
  {
    IDLE,
    BRAKING,
    REVERSING,
    COMPLETE,
    BLOCKED,
    LIMIT_REACHED
  };

  struct NominalSnapshot
  {
    PlannerVelocity2D velocity;
    std::chrono::steady_clock::time_point receipt_time{};
    bool valid{false};
  };

  struct TargetSnapshot
  {
    double heading{0.0};
    int64_t stamp_ns{0};
    std::chrono::steady_clock::time_point receipt_time{};
    bool valid{false};
  };

  struct ObstacleSnapshot
  {
    std::shared_ptr<const std::vector<ObstaclePoint2D>> points;
    int64_t stamp_ns{0};
    std::chrono::steady_clock::time_point receipt_time{};
    bool valid{false};
    std::string rejection_reason;
  };

  struct OdomSnapshot
  {
    PlannerVelocity2D velocity;
    std::chrono::steady_clock::time_point receipt_time{};
    bool valid{false};
    std::string rejection_reason;
  };

  struct StatusSnapshot
  {
    std::string state{"WAIT_INPUT"};
    double target_heading{0.0};
    double target_age{std::numeric_limits<double>::infinity()};
    FollowRecoveryState recovery;
    PlannerVelocity2D nominal;
    PlannerVelocity2D stabilized_nominal;
    PlannerVelocity2D effective_nominal;
    PlannerVelocity2D measured;
    PlannerVelocity2D planned;
    PlannerVelocity2D final_command;
    PlannerCost cost;
    double nominal_age{std::numeric_limits<double>::infinity()};
    double obstacle_age{std::numeric_limits<double>::infinity()};
    double odom_age{std::numeric_limits<double>::infinity()};
    double min_clearance{std::numeric_limits<double>::infinity()};
    double required_clearance{0.0};
    double clearance_ttc{std::numeric_limits<double>::infinity()};
    double planning_time_ms{0.0};
    std::size_t obstacle_count{0U};
    std::size_t evaluated_count{0U};
    std::size_t collision_count{0U};
    std::size_t marginal_count{0U};
    int avoidance_turn_direction{0};
    double selected_speed_scale{0.0};
    double emergency_reverse_elapsed{0.0};
    double emergency_reverse_remaining{0.0};
    bool avoidance_active{false};
    bool emergency_detected{false};
    bool emergency{false};
    bool emergency_recovery_active{false};
  };

  // 检查话题、时效、运动学、采样权重、足迹和障碍过滤参数。
  void validateParameters()
  {
    std::string reason;
    if (target_topic_.empty() || !std::isfinite(target_timeout_sec_) ||
      target_timeout_sec_ <= 0.0 ||
      !validateCommandPredictionConfig(prediction_config_) ||
      !validateFollowRecoveryConfig(recovery_config_))
    {
      throw std::invalid_argument("invalid command prediction, target or recovery configuration");
    }
    if (base_frame_.empty() || odom_child_frame_.empty() || nominal_cmd_topic_.empty() ||
      obstacle_topic_.empty() || odom_topic_.empty() || planned_cmd_topic_.empty() ||
      final_cmd_topic_.empty() || cmd_vel_topic_.empty() || selected_path_topic_.empty() ||
      diagnostics_topic_.empty())
    {
      throw std::invalid_argument("frame and topic names must not be empty");
    }
    if (!validateTrajectoryConfig(trajectory_config_, &reason) ||
      !validateFootprintConfig(footprint_config_, &reason) ||
      !validateMotionLimits(motion_limits_, &reason) ||
      !validateAngularStabilizationConfig(angular_stabilization_config_, &reason) ||
      !validateEmergencyReverseConfig(emergency_reverse_config_, motion_limits_, &reason) ||
      !validateVelocitySamplingConfig(sampling_config_, &reason))
    {
      throw std::invalid_argument(reason);
    }
    if (sampling_config_.max_avoidance_angular_speed > motion_limits_.max_angular_speed ||
      motion_limits_.min_angular_speed > sampling_config_.max_avoidance_angular_speed)
    {
      throw std::invalid_argument(
              "avoidance angular limits must stay within follow angular speed limits");
    }
    if (!std::isfinite(control_frequency_) || control_frequency_ <= 0.0 ||
      !std::isfinite(diagnostic_frequency_) || diagnostic_frequency_ <= 0.0)
    {
      throw std::invalid_argument("control and diagnostic frequencies must be positive");
    }
    if (!std::isfinite(nominal_timeout_sec_) || nominal_timeout_sec_ <= 0.0 ||
      !std::isfinite(obstacle_timeout_sec_) || obstacle_timeout_sec_ <= 0.0 ||
      !std::isfinite(odom_timeout_sec_) || odom_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument("input timeouts must be positive");
    }
    if (!std::isfinite(odom_linear_deadband_) || odom_linear_deadband_ < 0.0 ||
      !std::isfinite(odom_angular_deadband_) || odom_angular_deadband_ < 0.0)
    {
      throw std::invalid_argument("odom velocity deadbands must be non-negative");
    }
    if (!std::isfinite(avoidance_direction_hold_sec_) || avoidance_direction_hold_sec_ < 0.0) {
      throw std::invalid_argument("avoidance direction hold time must be non-negative");
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

  void targetCallback(const geometry_msgs::msg::PointStamped::SharedPtr message)
  {
    TargetSnapshot snapshot;
    snapshot.receipt_time = std::chrono::steady_clock::now();
    snapshot.stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
    snapshot.valid = message->header.frame_id == base_frame_ && snapshot.stamp_ns > 0 &&
      std::isfinite(message->point.x) && std::isfinite(message->point.y);
    if (snapshot.valid) {
      snapshot.heading = std::atan2(message->point.y, message->point.x);
    }
    std::lock_guard<std::mutex> lock(input_mutex_);
    target_snapshot_ = snapshot;
  }

  // 保存最新名义跟随速度，只接受有限的前进和转向分量。
  void nominalCallback(const geometry_msgs::msg::TwistStamped::SharedPtr message)
  {
    NominalSnapshot snapshot;
    const double linear_x = message->twist.linear.x;
    const double angular_z = message->twist.angular.z;
    snapshot.valid = std::isfinite(linear_x) && std::isfinite(angular_z);
    snapshot.velocity.linear_x = snapshot.valid ?
      clampValue(linear_x, 0.0, motion_limits_.max_linear_speed) : 0.0;
    snapshot.velocity.angular_z = snapshot.valid ?
      clampValue(
      angular_z, -motion_limits_.max_angular_speed, motion_limits_.max_angular_speed) :
      0.0;
    snapshot.receipt_time = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(input_mutex_);
    nominal_snapshot_ = snapshot;
  }

  // 从 /odom_leg 读取 base_footprint 下的真实速度，保留恢复倒退所需的负线速度。
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    OdomSnapshot snapshot;
    snapshot.receipt_time = std::chrono::steady_clock::now();
    if (message->child_frame_id != odom_child_frame_) {
      snapshot.rejection_reason = "odom child_frame_id differs from configured frame";
      storeOdomSnapshot(std::move(snapshot));
      return;
    }

    double linear_x = message->twist.twist.linear.x;
    double angular_z = message->twist.twist.angular.z;
    if (!std::isfinite(linear_x) || !std::isfinite(angular_z)) {
      snapshot.rejection_reason = "odom twist contains non-finite values";
      storeOdomSnapshot(std::move(snapshot));
      return;
    }
    if (std::abs(linear_x) < odom_linear_deadband_) {
      linear_x = 0.0;
    }
    if (std::abs(angular_z) < odom_angular_deadband_) {
      angular_z = 0.0;
    }
    snapshot.velocity.linear_x = clampValue(
      linear_x, -motion_limits_.max_reverse_speed, motion_limits_.max_linear_speed);
    snapshot.velocity.angular_z = clampValue(
      angular_z, -motion_limits_.max_angular_speed, motion_limits_.max_angular_speed);
    snapshot.valid = true;
    storeOdomSnapshot(std::move(snapshot));
  }

  // 用一次短锁替换共享实测速度快照。
  void storeOdomSnapshot(OdomSnapshot snapshot)
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    odom_snapshot_ = std::move(snapshot);
  }

  // 将当前帧 PointCloud2 过滤成 base_footprint 下的二维障碍快照。
  void obstacleCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    ObstacleSnapshot snapshot;
    snapshot.stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
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
          enable_self_filter_ && x >= self_filter_x_min_ && x <= self_filter_x_max_ &&
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

  // 计算快照相对当前单调时钟的接收年龄。
  template<typename SnapshotT>
  double snapshotAge(
    const SnapshotT & snapshot,
    const std::chrono::steady_clock::time_point & current) const
  {
    return snapshot.valid ?
           std::chrono::duration<double>(current - snapshot.receipt_time).count() :
           std::numeric_limits<double>::infinity();
  }

  // 返回仍在保持期内的避障转向方向，短时点云丢失或 UWB 小角度变化不会清除方向。
  int activeAvoidanceTurnDirection(const std::chrono::steady_clock::time_point & current)
  {
    if (!have_avoidance_turn_time_ || avoidance_turn_direction_ == 0) {
      return 0;
    }
    const double age = std::chrono::duration<double>(
      current - last_avoidance_turn_time_).count();
    if (age > avoidance_direction_hold_sec_) {
      avoidance_turn_direction_ = 0;
      have_avoidance_turn_time_ = false;
      return 0;
    }
    return avoidance_turn_direction_;
  }

  // 按新点云帧确认紧急障碍，过滤只出现一帧的近场伪点。
  bool confirmEmergencyObstacle(
    bool detected,
    const std::chrono::steady_clock::time_point & obstacle_receipt)
  {
    // 控制频率高于点云频率，同一帧只能计数一次，否则确认机制会退化成单帧触发。
    if (have_confirm_receipt_ && obstacle_receipt == last_confirm_receipt_) {
      return emergency_latched_;
    }
    last_confirm_receipt_ = obstacle_receipt;
    have_confirm_receipt_ = true;

    if (detected) {
      emergency_clear_count_ = 0;
      emergency_hit_count_ = std::min(emergency_hit_count_ + 1, emergency_confirm_frames_);
    } else {
      emergency_hit_count_ = 0;
      emergency_clear_count_ = std::min(emergency_clear_count_ + 1, emergency_confirm_frames_);
    }
    if (emergency_hit_count_ >= emergency_confirm_frames_) {
      emergency_latched_ = true;
    } else if (emergency_clear_count_ >= emergency_confirm_frames_) {
      emergency_latched_ = false;
    }
    return emergency_latched_;
  }

  // 清除单次急停恢复状态；关键输入失效后必须重新完成急停确认和停稳阶段。
  void resetEmergencyRecovery()
  {
    emergency_recovery_state_ = EmergencyRecoveryState::IDLE;
    emergency_brake_start_time_ = std::chrono::steady_clock::time_point{};
    emergency_reverse_start_time_ = std::chrono::steady_clock::time_point{};
  }

  // 急停后先确认底盘停稳，再安全后退到触发障碍离开当前急停区域。
  bool handleEmergencyRecovery(
    StatusSnapshot & status,
    const OdomSnapshot & odom,
    const std::vector<ObstaclePoint2D> & obstacles,
    const std::chrono::steady_clock::time_point & current)
  {
    if (!emergency_reverse_config_.enabled) {
      resetEmergencyRecovery();
      return false;
    }

    if (emergency_recovery_state_ == EmergencyRecoveryState::IDLE) {
      if (!status.emergency) {
        return false;
      }
      emergency_recovery_state_ = EmergencyRecoveryState::BRAKING;
      emergency_brake_start_time_ = current;
      status.emergency_recovery_active = true;
      status.emergency_reverse_remaining = emergency_reverse_config_.distance;
      // 急停触发周期必须先明确发出一次零速，下一周期才允许检查停稳并切换方向。
      const auto braking_path = predictAcceleratingTrajectory(
        odom.velocity, PlannerVelocity2D{}, trajectory_config_, motion_limits_, true);
      publishStop(status, "EMERGENCY_BRAKING", braking_path);
      return true;
    }

    if (emergency_recovery_state_ == EmergencyRecoveryState::BRAKING) {
      // 短暂伪障碍在停稳阶段已经解除时，不执行没有必要的盲区倒退。
      if (!status.emergency) {
        resetEmergencyRecovery();
        return false;
      }
      if (!status.emergency_detected) {
        emergency_recovery_state_ = EmergencyRecoveryState::COMPLETE;
        publishStop(status, "EMERGENCY_ZONE_CLEARED", {});
        return true;
      }
      const double braking_elapsed = std::chrono::duration<double>(
        current - emergency_brake_start_time_).count();
      status.emergency_recovery_active = true;
      status.emergency_reverse_elapsed = braking_elapsed;
      status.emergency_reverse_remaining = emergency_reverse_config_.distance;
      const bool hold_finished = braking_elapsed >= emergency_reverse_config_.stop_hold_sec;
      const bool chassis_stopped =
        std::abs(odom.velocity.linear_x) <=
        emergency_reverse_config_.start_linear_speed_threshold &&
        std::abs(odom.velocity.angular_z) <=
        emergency_reverse_config_.start_angular_speed_threshold;
      if (!hold_finished || !chassis_stopped) {
        const auto braking_path = predictAcceleratingTrajectory(
          odom.velocity, PlannerVelocity2D{}, trajectory_config_, motion_limits_, true);
        publishStop(status, "EMERGENCY_BRAKING", braking_path);
        return true;
      }

      const auto initial_plan = planEmergencyReverse(
        obstacles, trajectory_config_, footprint_config_, motion_limits_,
        emergency_reverse_config_, emergency_reverse_config_.distance);
      if (!initial_plan.valid) {
        emergency_recovery_state_ = EmergencyRecoveryState::BLOCKED;
        status.min_clearance = initial_plan.collision.min_clearance;
        publishStop(status, "EMERGENCY_REVERSE_BLOCKED", initial_plan.selected_trajectory);
        return true;
      }
      emergency_recovery_state_ = EmergencyRecoveryState::REVERSING;
      emergency_reverse_start_time_ = current;
    }

    if (emergency_recovery_state_ == EmergencyRecoveryState::REVERSING) {
      const double reverse_elapsed = std::chrono::duration<double>(
        current - emergency_reverse_start_time_).count();
      const EmergencyReverseProgress progress = evaluateEmergencyReverseProgress(
        status.emergency_detected, emergency_reverse_config_, reverse_elapsed);
      status.emergency_recovery_active = true;
      status.emergency_reverse_elapsed = reverse_elapsed;
      status.emergency_reverse_remaining = progress.remaining_distance;
      if (progress.zone_cleared) {
        emergency_recovery_state_ = EmergencyRecoveryState::COMPLETE;
        publishStop(status, "EMERGENCY_ZONE_CLEARED", {});
        return true;
      }
      if (progress.distance_limit_reached) {
        // 区域仍被占用却达到后退上限，按异常阻塞停车，禁止无限退入后方盲区。
        emergency_recovery_state_ = EmergencyRecoveryState::LIMIT_REACHED;
        publishStop(status, "EMERGENCY_REVERSE_LIMIT_REACHED", {});
        return true;
      }

      const auto reverse_plan = planEmergencyReverse(
        obstacles, trajectory_config_, footprint_config_, motion_limits_,
        emergency_reverse_config_, progress.remaining_distance);
      status.min_clearance = reverse_plan.collision.min_clearance;
      status.required_clearance = emergency_reverse_config_.extra_safety_margin;
      if (std::isfinite(reverse_plan.collision.min_clearance)) {
        status.clearance_ttc = reverse_plan.collision.min_clearance /
          emergency_reverse_config_.speed;
      }
      if (!reverse_plan.valid) {
        emergency_recovery_state_ = EmergencyRecoveryState::BLOCKED;
        publishStop(status, "EMERGENCY_REVERSE_BLOCKED", reverse_plan.selected_trajectory);
        return true;
      }

      status.avoidance_active = true;
      status.planned = reverse_plan.selected_velocity;
      status.final_command = reverse_plan.selected_velocity;
      // 已经过停稳门槛，恢复阶段直接跨过 Go2 执行死区，且禁止边退边转。
      status.final_command.angular_z = 0.0;
      publishDecision(status, "EMERGENCY_REVERSING", reverse_plan.selected_trajectory);
      return true;
    }

    if (emergency_recovery_state_ == EmergencyRecoveryState::BLOCKED) {
      if (!status.emergency) {
        resetEmergencyRecovery();
        return false;
      }
      publishStop(status, "EMERGENCY_REVERSE_BLOCKED", {});
      return true;
    }

    if (emergency_recovery_state_ == EmergencyRecoveryState::LIMIT_REACHED) {
      if (!status.emergency) {
        resetEmergencyRecovery();
        return false;
      }
      publishStop(status, "EMERGENCY_REVERSE_LIMIT_REACHED", {});
      return true;
    }

    if (emergency_recovery_state_ == EmergencyRecoveryState::COMPLETE) {
      if (status.emergency_detected) {
        // 清空帧只是短暂抖动时恢复倒退，但先保留一个零速周期再重新进入恢复。
        emergency_recovery_state_ = EmergencyRecoveryState::REVERSING;
        publishStop(status, "EMERGENCY_ZONE_REENTERED", {});
        return true;
      }
      if (!status.emergency) {
        resetEmergencyRecovery();
        return false;
      }
      publishStop(status, "EMERGENCY_ZONE_CLEARED", {});
      return true;
    }
    return false;
  }

  // 生成仅供候选评分使用的历史速度，让短暂停车后仍优先沿原方向绕障。
  PlannerVelocity2D makeScoringPreviousCommand(
    const PlannerVelocity2D & previous_command,
    int preferred_direction) const
  {
    PlannerVelocity2D scoring_previous = previous_command;
    if (preferred_direction == 0) {
      return scoring_previous;
    }

    // 这里只改变评分参考，不改变加速度限幅使用的真实上一条指令。
    const double reference_magnitude = std::max(
      std::abs(scoring_previous.angular_z), sampling_config_.min_avoidance_angular_speed);
    scoring_previous.angular_z = std::copysign(
      reference_magnitude, static_cast<double>(preferred_direction));
    return scoring_previous;
  }

  // 仅在名义轨迹进入障碍影响区且规划结果发生转向时记录绕障方向。
  void rememberAvoidanceTurnDirection(
    bool avoidance_active,
    const PlannerVelocity2D & selected_velocity,
    const std::chrono::steady_clock::time_point & current)
  {
    constexpr double angular_tolerance = 1e-6;
    if (!avoidance_active || std::abs(selected_velocity.angular_z) <= angular_tolerance) {
      return;
    }
    avoidance_turn_direction_ = selected_velocity.angular_z > 0.0 ? 1 : -1;
    last_avoidance_turn_time_ = current;
    have_avoidance_turn_time_ = true;
  }

  // 固定频率执行输入时效、速度采样、制动轨迹碰撞和最终指令限幅。
  void controlTick()
  {
    NominalSnapshot nominal;
    TargetSnapshot target;
    ObstacleSnapshot obstacle;
    OdomSnapshot odom;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      nominal = nominal_snapshot_;
      target = target_snapshot_;
      obstacle = obstacle_snapshot_;
      odom = odom_snapshot_;
    }

    const auto current = std::chrono::steady_clock::now();
    const double measured_dt = std::chrono::duration<double>(current - last_control_time_).count();
    last_control_time_ = current;
    const double control_dt = clampValue(measured_dt, 0.0, 2.0 / control_frequency_);

    StatusSnapshot status;
    status.target_heading = target.heading;
    status.target_age = std::max(
      snapshotAge(target, current),
      static_cast<double>(now().nanoseconds() - target.stamp_ns) * 1e-9);
    status.nominal = nominal.velocity;
    status.measured = odom.velocity;
    status.nominal_age = snapshotAge(nominal, current);
    status.obstacle_age = snapshotAge(obstacle, current);
    status.odom_age = snapshotAge(odom, current);
    status.avoidance_turn_direction = activeAvoidanceTurnDirection(current);

    if (!nominal.valid || status.nominal_age > nominal_timeout_sec_) {
      resetEmergencyRecovery();
      publishStop(status, "NOMINAL_TIMEOUT", {});
      return;
    }
    if (!obstacle.valid) {
      resetEmergencyRecovery();
      const std::string state = obstacle.rejection_reason.empty() ?
        "WAIT_OBSTACLE" : "OBSTACLE_INVALID";
      publishStop(status, state, {});
      return;
    }
    if (status.obstacle_age > obstacle_timeout_sec_) {
      resetEmergencyRecovery();
      publishStop(status, "SENSOR_TIMEOUT", {});
      return;
    }
    if (!odom.valid) {
      resetEmergencyRecovery();
      const std::string state = odom.rejection_reason.empty() ? "WAIT_ODOM" : "ODOM_INVALID";
      publishStop(status, state, {});
      return;
    }
    if (status.odom_age > odom_timeout_sec_) {
      resetEmergencyRecovery();
      publishStop(status, "ODOM_TIMEOUT", {});
      return;
    }

    if (!target.valid || status.target_age > target_timeout_sec_ ||
      target.stamp_ns > now().nanoseconds() + 50000000LL)
    {
      resetEmergencyRecovery();
      publishStop(status, "TARGET_INVALID_OR_TIMEOUT", {});
      return;
    }

    status.stabilized_nominal = correctNominalAngularVelocity(
      nominal.velocity, odom.velocity, angular_stabilization_config_, motion_limits_);

    const auto & points = *obstacle.points;
    status.obstacle_count = points.size();
    status.emergency_detected = hasEmergencyFrontObstacle(
      points, footprint_config_, emergency_front_distance_, emergency_half_width_);
    status.emergency = confirmEmergencyObstacle(
      status.emergency_detected, obstacle.receipt_time);
    if (handleEmergencyRecovery(status, odom, points, current)) {
      return;
    }
    const PlannerVelocity2D previous_command = last_command_valid_ ?
      last_command_ : odom.velocity;
    const PlannerVelocity2D scoring_previous_command = makeScoringPreviousCommand(
      previous_command, status.avoidance_turn_direction);

    const auto planning_start = std::chrono::steady_clock::now();
    auto prediction = prediction_config_;
    prediction.control_dt = control_dt;
    const bool new_observation = obstacle.stamp_ns > last_recovery_observation_stamp_;
    if (new_observation) {
      last_recovery_observation_stamp_ = obstacle.stamp_ns;
    }
    const FollowRecoveryInput recovery_input{target.heading, true, new_observation};
    const LocalPlanResult result = planRecoveringVelocity(
      odom.velocity, previous_command, status.stabilized_nominal, points,
      trajectory_config_, footprint_config_, motion_limits_, sampling_config_,
      prediction, recovery_config_, recovery_input, recovery_state_, status.emergency,
      &scoring_previous_command);
    status.recovery = recovery_state_;
    status.planning_time_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - planning_start).count();
    status.effective_nominal = result.effective_nominal;
    status.evaluated_count = result.evaluated_count;
    status.collision_count = result.collision_count;
    status.marginal_count = result.marginal_count;
    status.avoidance_active = result.avoidance_active;
    status.selected_speed_scale = result.selected_speed_scale;

    if (!result.valid) {
      const auto braking_path = predictAcceleratingTrajectory(
        odom.velocity, PlannerVelocity2D{}, trajectory_config_, motion_limits_, true);
      publishStop(status, "BLOCKED", braking_path);
      return;
    }

    status.planned = result.selected_velocity;
    rememberAvoidanceTurnDirection(
      result.avoidance_active, result.selected_velocity, current);
    status.avoidance_turn_direction = avoidance_turn_direction_;
    status.cost = result.cost;
    status.min_clearance = result.min_clearance;
    status.required_clearance = result.required_clearance;
    status.clearance_ttc = result.clearance_ttc;
    // first_command 已参与所选轨迹预测；此处不得再次限幅或改写。
    status.final_command = result.first_command;
    const std::string phase = recovery_state_.phase == FollowRecoveryPhase::AVOIDING ? "AVOIDING" :
      (recovery_state_.phase == FollowRecoveryPhase::RECOVERING ? "RECOVERING" : "PLANNING");
    const std::string state = enable_motion_ ? phase : phase + "_DEBUG";
    publishDecision(status, state, result.selected_trajectory);
  }

  // 在任一关键输入失效或无安全轨迹时发布零速度和可选制动轨迹。
  void publishStop(
    StatusSnapshot status,
    const std::string & state,
    const std::vector<PlannerPose2D> & trajectory)
  {
    // 本次实际发布全零指令，恢复阶段从零上限开始确认。
    recovery_state_.speed_cap = 0.0;
    recovery_state_.phase = FollowRecoveryPhase::RECOVERING;
    recovery_state_.clear_count = 0;
    recovery_state_.turn_mismatch_sec = 0.0;
    recovery_state_.turn_unestablished = false;
    status.recovery = recovery_state_;
    status.planned = PlannerVelocity2D{};
    status.final_command = PlannerVelocity2D{};
    publishDecision(std::move(status), state, trajectory, true);
  }

  // 发布规划目标、最终限幅结果、实机隔离输出、选中轨迹和诊断状态。
  void publishDecision(
    StatusSnapshot status,
    const std::string & state,
    const std::vector<PlannerPose2D> & trajectory,
    bool force_stop = false)
  {
    status.state = state;
    if (force_stop) {
      status.planned = PlannerVelocity2D{};
      status.final_command = PlannerVelocity2D{};
    }

    const auto stamp = now();
    publishStampedVelocity(planned_cmd_pub_, status.planned, stamp);
    publishStampedVelocity(final_cmd_pub_, status.final_command, stamp);

    geometry_msgs::msg::Twist output_message;
    if (enable_motion_ && !force_stop) {
      // 负线速度只可能由经过状态机和后向碰撞检查的急停恢复阶段产生。
      output_message.linear.x = clampValue(
        status.final_command.linear_x,
        -motion_limits_.max_reverse_speed,
        motion_limits_.max_linear_speed);
      output_message.angular.z = status.final_command.angular_z;
      last_command_ = status.final_command;
      last_command_valid_ = true;
    } else if (enable_motion_) {
      last_command_ = PlannerVelocity2D{};
      last_command_valid_ = true;
    }
    cmd_vel_pub_->publish(output_message);
    publishPath(trajectory, stamp);

    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      status_snapshot_ = std::move(status);
    }
    publishDiagnosticIfDue();
  }

  // 发布一个带 base_footprint 时间戳的二维速度调试消息，并保留恢复倒退指令。
  void publishStampedVelocity(
    const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr & publisher,
    const PlannerVelocity2D & velocity,
    const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::TwistStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = base_frame_;
    message.twist.linear.x = clampValue(
      velocity.linear_x, -motion_limits_.max_reverse_speed, motion_limits_.max_linear_speed);
    message.twist.angular.z = velocity.angular_z;
    publisher->publish(message);
  }

  // 将当前周期选中并包含制动尾段的局部轨迹发布到 base_footprint。
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
    selected_path_pub_->publish(path);
  }

  // 按限制频率发布输入时效、实测速度、候选数量、代价和最终速度。
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
    diagnostic.level = status.state == "PLANNING" || status.state == "PLANNING_DEBUG" ||
      status.state == "AVOIDING" || status.state == "AVOIDING_DEBUG" ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diagnostic.name = get_fully_qualified_name() + std::string(": local velocity planner");
    diagnostic.hardware_id = "go2_stereo_local_planner";
    diagnostic.message = status.state;
    const std::pair<std::string, std::string> entries[] = {
      {"target_heading", formatDouble(status.target_heading)},
      {"target_age_sec", formatDouble(status.target_age)},
      {"recovery_speed_cap", formatDouble(status.recovery.speed_cap)},
      {"recovery_clear_count", std::to_string(status.recovery.clear_count)},
      {"turn_mismatch_sec", formatDouble(status.recovery.turn_mismatch_sec)},
      {"turn_unestablished", status.recovery.turn_unestablished ? "true" : "false"},
      {"recovery_reversing", status.recovery.reversing ? "true" : "false"},
      {"enable_motion", enable_motion_ ? "true" : "false"},
      {"nominal_age_sec", formatDouble(status.nominal_age)},
      {"obstacle_age_sec", formatDouble(status.obstacle_age)},
      {"odom_age_sec", formatDouble(status.odom_age)},
      {"obstacle_count", std::to_string(status.obstacle_count)},
      {"evaluated_count", std::to_string(status.evaluated_count)},
      {"collision_count", std::to_string(status.collision_count)},
      {"marginal_count", std::to_string(status.marginal_count)},
      {"avoidance_turn_direction", std::to_string(status.avoidance_turn_direction)},
      {"avoidance_active", status.avoidance_active ? "true" : "false"},
      {"selected_speed_scale", formatDouble(status.selected_speed_scale)},
      {"emergency_detected", status.emergency_detected ? "true" : "false"},
      {"emergency", status.emergency ? "true" : "false"},
      {"emergency_recovery_active", status.emergency_recovery_active ? "true" : "false"},
      {"emergency_reverse_elapsed_sec", formatDouble(status.emergency_reverse_elapsed)},
      {"emergency_reverse_remaining_m", formatDouble(status.emergency_reverse_remaining)},
      {"min_clearance", formatDouble(status.min_clearance)},
      {"required_clearance", formatDouble(status.required_clearance)},
      {"clearance_ttc", formatDouble(status.clearance_ttc)},
      {"planning_time_ms", formatDouble(status.planning_time_ms)},
      {"measured_v", formatDouble(status.measured.linear_x)},
      {"measured_w", formatDouble(status.measured.angular_z)},
      {"nominal_v", formatDouble(status.nominal.linear_x)},
      {"nominal_w", formatDouble(status.nominal.angular_z)},
      {"angular_velocity_error", formatDouble(
          status.nominal.angular_z - status.measured.angular_z)},
      {"angular_velocity_tracking_kp", formatDouble(
          angular_stabilization_config_.velocity_tracking_kp)},
      {"stabilized_nominal_v", formatDouble(status.stabilized_nominal.linear_x)},
      {"stabilized_nominal_w", formatDouble(status.stabilized_nominal.angular_z)},
      {"corrected_nominal_w", formatDouble(status.stabilized_nominal.angular_z)},
      {"effective_nominal_v", formatDouble(status.effective_nominal.linear_x)},
      {"effective_nominal_w", formatDouble(status.effective_nominal.angular_z)},
      {"planned_v", formatDouble(status.planned.linear_x)},
      {"planned_w", formatDouble(status.planned.angular_z)},
      {"final_v", formatDouble(status.final_command.linear_x)},
      {"final_w", formatDouble(status.final_command.angular_z)},
      {"cost_total", formatDouble(status.cost.total)},
      {"cost_follow", formatDouble(status.cost.follow)},
      {"cost_smooth", formatDouble(status.cost.smooth)},
      {"cost_obstacle", formatDouble(status.cost.obstacle)},
      {"cost_progress", formatDouble(status.cost.progress)}};
    for (const auto & entry : entries) {
      diagnostic_msgs::msg::KeyValue value;
      value.key = entry.first;
      value.value = entry.second;
      diagnostic.values.push_back(std::move(value));
    }
    array.status.push_back(std::move(diagnostic));
    diagnostics_pub_->publish(array);
  }

  std::string target_topic_;
  double target_timeout_sec_{0.50};
  CommandPredictionConfig prediction_config_;
  FollowRecoveryConfig recovery_config_;
  FollowRecoveryState recovery_state_;
  TargetSnapshot target_snapshot_;
  int64_t last_recovery_observation_stamp_{0};
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
  std::string base_frame_;
  std::string odom_child_frame_;
  std::string nominal_cmd_topic_;
  std::string obstacle_topic_;
  std::string odom_topic_;
  std::string planned_cmd_topic_;
  std::string final_cmd_topic_;
  std::string cmd_vel_topic_;
  std::string selected_path_topic_;
  std::string diagnostics_topic_;

  bool enable_motion_{false};
  double control_frequency_{20.0};
  double diagnostic_frequency_{2.0};
  double nominal_timeout_sec_{0.20};
  double obstacle_timeout_sec_{1.00};
  double odom_timeout_sec_{0.10};
  double odom_linear_deadband_{0.02};
  double odom_angular_deadband_{0.05};
  AngularStabilizationConfig angular_stabilization_config_;
  double avoidance_direction_hold_sec_{1.50};
  TrajectoryConfig trajectory_config_;
  FootprintConfig footprint_config_;
  MotionLimits motion_limits_;
  VelocitySamplingConfig sampling_config_;
  EmergencyReverseConfig emergency_reverse_config_;
  double emergency_front_distance_{0.25};
  double emergency_half_width_{0.30};
  int emergency_confirm_frames_{2};
  double obstacle_x_min_{-0.75};
  double obstacle_x_max_{3.00};
  double obstacle_y_abs_max_{2.00};
  bool enable_self_filter_{true};
  double self_filter_x_min_{-0.35};
  double self_filter_x_max_{0.35};
  double self_filter_y_abs_{0.20};
  int max_obstacle_points_{5000};

  std::mutex input_mutex_;
  NominalSnapshot nominal_snapshot_;
  ObstacleSnapshot obstacle_snapshot_;
  OdomSnapshot odom_snapshot_;
  PlannerVelocity2D last_command_;
  bool last_command_valid_{false};
  int avoidance_turn_direction_{0};
  bool have_avoidance_turn_time_{false};
  int emergency_hit_count_{0};
  int emergency_clear_count_{0};
  bool emergency_latched_{false};
  EmergencyRecoveryState emergency_recovery_state_{EmergencyRecoveryState::IDLE};
  bool have_confirm_receipt_{false};
  std::chrono::steady_clock::time_point last_confirm_receipt_{};
  std::chrono::steady_clock::time_point last_avoidance_turn_time_{};
  std::chrono::steady_clock::time_point emergency_brake_start_time_{};
  std::chrono::steady_clock::time_point emergency_reverse_start_time_{};
  std::chrono::steady_clock::time_point last_control_time_{};

  std::mutex status_mutex_;
  StatusSnapshot status_snapshot_;
  std::chrono::duration<double> diagnostic_period_{0.5};
  bool have_diagnostic_time_{false};
  std::chrono::steady_clock::time_point last_diagnostic_time_{};

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr nominal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr planned_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr final_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr selected_path_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace go2_uwb_local_follow

// 启动局部速度规划节点并进入 ROS 事件循环。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<go2_uwb_local_follow::LocalVelocityPlannerNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("local_velocity_planner_node"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
