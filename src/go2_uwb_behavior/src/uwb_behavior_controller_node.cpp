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
#include <cstdio>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "go2_uwb_behavior/action/random_roam.hpp"
#include "go2_uwb_behavior/behavior_core.hpp"
#include "go2_uwb_behavior/srv/set_behavior.hpp"
#include "go2_uwb_local_follow/follow_control_core.hpp"

namespace go2_uwb_behavior
{
namespace
{

using SteadyTime = std::chrono::steady_clock::time_point;
using RandomRoam = go2_uwb_behavior::action::RandomRoam;
using GoalHandleRandomRoam = rclcpp_action::ServerGoalHandle<RandomRoam>;
using SetBehavior = go2_uwb_behavior::srv::SetBehavior;
using FollowResult = go2_uwb_local_follow::FollowResult;
using FollowConfig = go2_uwb_local_follow::FollowConfig;

constexpr double kMinimumAge = 0.0;

// 将浮点数格式化为固定三位小数的诊断文本。
std::string formatDouble(double value)
{
  if (!std::isfinite(value)) {
    return "n/a";
  }
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.3f", value);
  return std::string(buffer);
}

// 将现有跟随核心速度转换为行为层统一速度结构。
Velocity2D fromFollowVelocity(const go2_uwb_local_follow::Velocity2D & velocity)
{
  return Velocity2D{velocity.linear_x, velocity.angular_z};
}

}  // namespace

class UwbBehaviorControllerNode : public rclcpp::Node
{
public:
  // 初始化统一跟随、随机漫游、Action 接口与最终速度安全门控。
  explicit UwbBehaviorControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("uwb_behavior_controller_node", options),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declareParameters();
    validateParameters();
    owner_filter_ = std::make_unique<OwnerCenterFilter>(owner_filter_config_);
    progress_monitor_ = std::make_unique<ProgressMonitor>(
      required_progress_, progress_window_sec_);

    target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      target_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      std::bind(&UwbBehaviorControllerNode::targetCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&UwbBehaviorControllerNode::odomCallback, this, std::placeholders::_1));
    obstacle_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      obstacle_topic_, rclcpp::SensorDataQoS(),
      std::bind(&UwbBehaviorControllerNode::obstacleCallback, this, std::placeholders::_1));
    planner_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      planner_cmd_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      std::bind(&UwbBehaviorControllerNode::plannerCommandCallback, this, std::placeholders::_1));
    planner_diagnostics_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      planner_diagnostics_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      std::bind(
        &UwbBehaviorControllerNode::plannerDiagnosticsCallback, this,
        std::placeholders::_1));

    nominal_cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(nominal_cmd_topic_, 10);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, 10);
    follow_diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      follow_diagnostics_topic_, 10);
    const auto marker_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    roam_target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      roam_target_topic_, marker_qos);
    play_center_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      play_center_topic_, marker_qos);

    behavior_service_ = create_service<SetBehavior>(
      behavior_service_name_,
      std::bind(
        &UwbBehaviorControllerNode::behaviorServiceCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    roam_action_server_ = rclcpp_action::create_server<RandomRoam>(
      this,
      roam_action_name_,
      std::bind(
        &UwbBehaviorControllerNode::handleRoamGoal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(
        &UwbBehaviorControllerNode::handleRoamCancel, this,
        std::placeholders::_1),
      std::bind(
        &UwbBehaviorControllerNode::handleRoamAccepted, this,
        std::placeholders::_1));

    const auto control_period = std::chrono::duration<double>(1.0 / control_frequency_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(control_period),
      std::bind(&UwbBehaviorControllerNode::controlTick, this));
    diagnostic_period_ = std::chrono::duration<double>(1.0 / diagnostic_frequency_);
    feedback_period_ = std::chrono::duration<double>(1.0 / feedback_frequency_);
    last_control_time_ = std::chrono::steady_clock::now();

    current_mode_ = default_mode_ == "IDLE" ? Mode::IDLE : Mode::FOLLOW;
    state_ = current_mode_ == Mode::FOLLOW ? "FOLLOW_WAIT_TARGET" : "IDLE";
    RCLCPP_INFO(
      get_logger(),
      "UWB behavior controller started: mode=%s target=%s odom=%s nominal=%s planner=%s cmd=%s",
      modeName(current_mode_).c_str(), target_topic_.c_str(), odom_topic_.c_str(),
      nominal_cmd_topic_.c_str(), planner_cmd_topic_.c_str(), cmd_vel_topic_.c_str());
  }

  // 节点正常销毁前尽力补发零名义速度和零底盘速度。
  ~UwbBehaviorControllerNode() override
  {
    if (nominal_cmd_pub_) {
      publishNominal(Velocity2D{});
    }
    if (cmd_vel_pub_) {
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    }
  }

private:
  enum class Mode : std::uint8_t
  {
    IDLE = SetBehavior::Request::IDLE,
    FOLLOW = SetBehavior::Request::FOLLOW,
    STOP = SetBehavior::Request::STOP,
    ROAM = SetBehavior::Request::ROAM
  };

  enum class RoamPhase
  {
    INACTIVE,
    PREPARING_STOP,
    SELECTING_GOAL,
    NAVIGATING,
    RETURNING,
    ARRIVAL_STOP,
    RETRY_STOP,
    FINAL_STOP
  };

  enum class CompletionDisposition
  {
    NONE,
    SUCCEED,
    ABORT,
    CANCEL
  };

  struct TargetSnapshot
  {
    Point2D point_base;
    builtin_interfaces::msg::Time source_stamp;
    SteadyTime receipt_time{};
    std::uint64_t version{0U};
    bool valid{false};
  };

  struct OdomSnapshot
  {
    Pose2D pose;
    Velocity2D velocity;
    SteadyTime receipt_time{};
    bool valid{false};
  };

  struct ObstacleSnapshot
  {
    std::vector<Point2D> points_odom;
    SteadyTime receipt_time{};
    bool points_ready_for_roam{false};
    bool valid{false};
  };

  struct PlannerCommandSnapshot
  {
    Velocity2D velocity;
    SteadyTime receipt_time{};
    bool valid{false};
  };

  // 声明话题、控制、跟随、漫游、围栏和完成条件参数。
  void declareParameters()
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    target_topic_ = declare_parameter<std::string>("target_topic", "/uwb/target_point");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom_leg");
    obstacle_topic_ = declare_parameter<std::string>(
      "obstacle_topic", "/local_rolling_obstacle");
    nominal_cmd_topic_ = declare_parameter<std::string>(
      "nominal_cmd_topic", "/go2_uwb_local_follow/nominal_cmd");
    planner_cmd_topic_ = declare_parameter<std::string>(
      "planner_cmd_topic", "/go2_uwb_behavior/planner_cmd_vel");
    planner_diagnostics_topic_ = declare_parameter<std::string>(
      "planner_diagnostics_topic", "/go2_uwb_local_follow/planner_diagnostics");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/go2/behavior_diagnostics");
    follow_diagnostics_topic_ = declare_parameter<std::string>(
      "follow_diagnostics_topic", "/go2_uwb_local_follow/follow_diagnostics");
    roam_target_topic_ = declare_parameter<std::string>(
      "roam_target_topic", "/go2/random_roam/target");
    play_center_topic_ = declare_parameter<std::string>(
      "play_center_topic", "/go2/random_roam/play_center");
    behavior_service_name_ = declare_parameter<std::string>(
      "behavior_service_name", "/go2/set_behavior");
    roam_action_name_ = declare_parameter<std::string>(
      "roam_action_name", "/go2/random_roam");
    default_mode_ = declare_parameter<std::string>("default_mode", "FOLLOW");
    enable_motion_ = declare_parameter<bool>("enable_motion", true);

    control_frequency_ = declare_parameter<double>("control_frequency", 20.0);
    diagnostic_frequency_ = declare_parameter<double>("diagnostic_frequency", 2.0);
    feedback_frequency_ = declare_parameter<double>("feedback_frequency", 5.0);
    target_timeout_sec_ = declare_parameter<double>("target_timeout_sec", 0.50);
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 0.20);
    obstacle_timeout_sec_ = declare_parameter<double>("obstacle_timeout_sec", 0.70);
    planner_cmd_timeout_sec_ = declare_parameter<double>("planner_cmd_timeout_sec", 0.20);
    planner_diagnostics_timeout_sec_ = declare_parameter<double>(
      "planner_diagnostics_timeout_sec", 0.75);
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.10);
    readiness_timeout_sec_ = declare_parameter<double>("readiness_timeout_sec", 2.0);

    follow_config_.follow_distance = declare_parameter<double>("follow_distance", 1.0);
    follow_config_.distance_deadband = declare_parameter<double>("distance_deadband", 0.08);
    follow_config_.angle_deadband = declare_parameter<double>("angle_deadband", 0.20);
    follow_config_.angle_reengage = declare_parameter<double>("angle_reengage", 0.45);
    follow_config_.turn_response_delay = declare_parameter<double>("turn_response_delay", 0.10);
    follow_config_.angular_braking_accel = declare_parameter<double>(
      "angular_braking_accel", 1.50);
    follow_config_.angular_brake_release_speed = declare_parameter<double>(
      "angular_brake_release_speed", 0.06);
    follow_config_.angular_reverse_speed_threshold = declare_parameter<double>(
      "angular_reverse_speed_threshold", 0.15);
    follow_config_.linear_kp = declare_parameter<double>("linear_kp", 0.60);
    follow_config_.angular_kp = declare_parameter<double>("angular_kp", 1.0);
    follow_config_.min_linear_speed = declare_parameter<double>("min_linear_speed", 0.23);
    follow_config_.max_linear_speed = declare_parameter<double>("max_linear_speed", 0.80);
    follow_config_.max_angular_speed = declare_parameter<double>("max_angular_speed", 2.0);
    follow_config_.heading_slowdown_start = declare_parameter<double>(
      "heading_slowdown_start", 0.50);
    follow_config_.heading_stop_angle = declare_parameter<double>("heading_stop_angle", 1.40);
    follow_config_.blind_rotation_max_speed = declare_parameter<double>(
      "blind_rotation_max_speed", 2.0);
    follow_config_.max_linear_accel = declare_parameter<double>("max_linear_accel", 0.80);
    follow_config_.max_linear_decel = declare_parameter<double>("max_linear_decel", 0.80);
    follow_config_.max_angular_accel = declare_parameter<double>("max_angular_accel", 2.0);

    roam_control_config_ = follow_config_;
    roam_control_config_.follow_distance = 0.0;
    roam_control_config_.distance_deadband = declare_parameter<double>(
      "roam_goal_tolerance", 0.30);
    roam_control_config_.angle_deadband = declare_parameter<double>(
      "roam_angle_deadband", 0.08);
    roam_control_config_.angle_reengage = declare_parameter<double>(
      "roam_angle_reengage", 0.15);
    roam_control_config_.max_linear_speed = declare_parameter<double>(
      "roam_max_linear_speed", 0.35);
    roam_control_config_.max_angular_speed = declare_parameter<double>(
      "roam_max_angular_speed", 1.0);
    roam_control_config_.blind_rotation_max_speed = roam_control_config_.max_angular_speed;

    owner_filter_config_.median_window = static_cast<std::size_t>(
      declare_parameter<int>("uwb_median_window", 5));
    owner_filter_config_.low_pass_alpha = declare_parameter<double>("uwb_filter_alpha", 0.25);
    owner_filter_config_.center_deadband = declare_parameter<double>(
      "play_center_deadband", 0.40);
    owner_filter_config_.center_move_confirm_sec = declare_parameter<double>(
      "center_move_confirm_sec", 1.0);
    owner_filter_config_.center_max_speed = declare_parameter<double>(
      "center_max_speed", 0.30);
    minimum_owner_samples_ = static_cast<std::size_t>(
      declare_parameter<int>("minimum_owner_samples", 5));

    sampling_config_.owner_keepout_radius = declare_parameter<double>(
      "owner_keepout_radius", 1.50);
    sampling_config_.random_goal_radius_max = declare_parameter<double>(
      "random_goal_radius_max", 4.50);
    sampling_config_.random_step_min = declare_parameter<double>("random_step_min", 0.80);
    sampling_config_.random_step_max = declare_parameter<double>("random_step_max", 1.80);
    sampling_config_.goal_obstacle_clearance = declare_parameter<double>(
      "goal_obstacle_clearance", 0.70);
    sampling_config_.max_sample_attempts = static_cast<std::size_t>(
      declare_parameter<int>("max_sample_attempts", 50));

    geofence_config_.return_trigger_radius = declare_parameter<double>(
      "return_trigger_radius", 5.00);
    geofence_config_.return_release_radius = declare_parameter<double>(
      "return_release_radius", 4.50);
    geofence_config_.restrictive_radius = declare_parameter<double>(
      "restrictive_radius", 5.60);
    geofence_config_.absolute_radius = declare_parameter<double>("absolute_radius", 6.00);
    geofence_config_.prediction_sec = declare_parameter<double>(
      "geofence_prediction_sec", 1.0);
    geofence_config_.simulation_dt = declare_parameter<double>(
      "geofence_simulation_dt", 0.05);
    geofence_reject_timeout_sec_ = declare_parameter<double>(
      "geofence_reject_timeout_sec", 1.0);

    default_roam_timeout_sec_ = declare_parameter<double>("default_roam_timeout_sec", 30.0);
    maximum_roam_timeout_sec_ = declare_parameter<double>("maximum_roam_timeout_sec", 120.0);
    required_progress_ = declare_parameter<double>("required_progress", 0.15);
    progress_window_sec_ = declare_parameter<double>("progress_window_sec", 3.0);
    planner_blocked_timeout_sec_ = declare_parameter<double>(
      "planner_blocked_timeout_sec", 1.0);
    max_retries_ = declare_parameter<int>("max_retries", 2);
    stop_linear_threshold_ = declare_parameter<double>("stop_linear_threshold", 0.04);
    stop_angular_threshold_ = declare_parameter<double>("stop_angular_threshold", 0.08);
    stop_confirm_sec_ = declare_parameter<double>("stop_confirm_sec", 0.30);
    stop_confirmation_timeout_sec_ = declare_parameter<double>(
      "stop_confirmation_timeout_sec", 2.0);
    robot_clearance_radius_ = declare_parameter<double>("robot_clearance_radius", 0.40);
  }

  // 校验全部参数及现有跟随核心的配置约束。
  void validateParameters()
  {
    std::string reason;
    if (!validateOwnerFilterConfig(owner_filter_config_, &reason) ||
      !validateRoamSamplingConfig(sampling_config_, &reason) ||
      !validateGeofenceConfig(geofence_config_, &reason) ||
      !go2_uwb_local_follow::validateFollowConfig(follow_config_, &reason) ||
      !go2_uwb_local_follow::validateFollowConfig(roam_control_config_, &reason))
    {
      throw std::invalid_argument(reason);
    }
    const std::vector<std::string> required_strings = {
      base_frame_, odom_frame_, target_topic_, odom_topic_, obstacle_topic_, nominal_cmd_topic_,
      planner_cmd_topic_, cmd_vel_topic_, behavior_service_name_, roam_action_name_};
    if (std::any_of(
        required_strings.begin(), required_strings.end(),
        [](const std::string & value) {return value.empty();}))
    {
      throw std::invalid_argument("behavior topic, frame and interface names must not be empty");
    }
    if (default_mode_ != "FOLLOW" && default_mode_ != "IDLE") {
      throw std::invalid_argument("default_mode must be FOLLOW or IDLE");
    }
    const double positive_values[] = {
      control_frequency_, diagnostic_frequency_, feedback_frequency_, target_timeout_sec_,
      odom_timeout_sec_, obstacle_timeout_sec_, planner_cmd_timeout_sec_,
      planner_diagnostics_timeout_sec_, transform_timeout_sec_, readiness_timeout_sec_,
      geofence_reject_timeout_sec_, default_roam_timeout_sec_, maximum_roam_timeout_sec_,
      required_progress_, progress_window_sec_, planner_blocked_timeout_sec_,
      stop_linear_threshold_, stop_angular_threshold_, stop_confirm_sec_,
      stop_confirmation_timeout_sec_, robot_clearance_radius_};
    if (std::any_of(
        std::begin(positive_values), std::end(positive_values),
        [](double value) {return !std::isfinite(value) || value <= 0.0;}))
    {
      throw std::invalid_argument("behavior timing and threshold parameters must be positive");
    }
    if (minimum_owner_samples_ == 0U ||
      minimum_owner_samples_ > owner_filter_config_.median_window || max_retries_ < 0 ||
      default_roam_timeout_sec_ > maximum_roam_timeout_sec_)
    {
      throw std::invalid_argument("behavior sample, retry or timeout bounds are invalid");
    }
    if (sampling_config_.random_goal_radius_max > geofence_config_.return_release_radius) {
      throw std::invalid_argument("random goal radius must not exceed return release radius");
    }
  }

  // 接收并转换最新 UWB 目标点，保留与原跟随节点一致的 TF 行为。
  void targetCallback(const geometry_msgs::msg::PointStamped::SharedPtr message)
  {
    if (!std::isfinite(message->point.x) || !std::isfinite(message->point.y) ||
      !std::isfinite(message->point.z) || message->header.frame_id.empty())
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Reject invalid UWB target point");
      return;
    }

    tf2::Vector3 target(message->point.x, message->point.y, message->point.z);
    if (message->header.frame_id != base_frame_) {
      try {
        const auto transform = tf_buffer_.lookupTransform(
          base_frame_, message->header.frame_id, rclcpp::Time(message->header.stamp),
          rclcpp::Duration::from_seconds(transform_timeout_sec_));
        tf2::Quaternion quaternion(
          transform.transform.rotation.x, transform.transform.rotation.y,
          transform.transform.rotation.z, transform.transform.rotation.w);
        if (quaternion.length2() <= std::numeric_limits<double>::epsilon()) {
          throw std::runtime_error("target TF quaternion has zero length");
        }
        quaternion.normalize();
        target = tf2::Matrix3x3(quaternion) * target + tf2::Vector3(
          transform.transform.translation.x,
          transform.transform.translation.y,
          transform.transform.translation.z);
      } catch (const std::exception & exception) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "UWB target TF failed: %s", exception.what());
        return;
      }
    }

    target_snapshot_.point_base = Point2D{target.x(), target.y()};
    target_snapshot_.source_stamp = message->header.stamp;
    target_snapshot_.receipt_time = std::chrono::steady_clock::now();
    target_snapshot_.valid = std::isfinite(target.x()) && std::isfinite(target.y());
    ++target_snapshot_.version;
  }

  // 保存连续 odom 位姿和实测二维速度，供中心估计与停车确认使用。
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    tf2::Quaternion quaternion(
      message->pose.pose.orientation.x,
      message->pose.pose.orientation.y,
      message->pose.pose.orientation.z,
      message->pose.pose.orientation.w);
    if (quaternion.length2() <= std::numeric_limits<double>::epsilon()) {
      odom_snapshot_.valid = false;
      return;
    }
    quaternion.normalize();
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    const double x = message->pose.pose.position.x;
    const double y = message->pose.pose.position.y;
    const double linear_x = message->twist.twist.linear.x;
    const double linear_y = message->twist.twist.linear.y;
    const double angular_z = message->twist.twist.angular.z;
    odom_snapshot_.valid = std::isfinite(x) && std::isfinite(y) && std::isfinite(yaw) &&
      std::isfinite(linear_x) && std::isfinite(linear_y) && std::isfinite(angular_z);
    odom_snapshot_.pose = Pose2D{x, y, yaw};
    // 停车确认使用平面线速度模长，防止仍在横移时误判为已经停稳。
    odom_snapshot_.velocity = Velocity2D{std::hypot(linear_x, linear_y), angular_z};
    odom_snapshot_.receipt_time = std::chrono::steady_clock::now();
  }

  // 将滚动障碍点从当前机身坐标转换到 odom 并缓存用于目标净空检查。
  void obstacleCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    const auto current = std::chrono::steady_clock::now();
    obstacle_snapshot_.receipt_time = current;
    obstacle_snapshot_.points_odom.clear();
    obstacle_snapshot_.points_ready_for_roam = false;
    if (!odom_snapshot_.valid ||
      elapsedSince(odom_snapshot_.receipt_time, current) > odom_timeout_sec_ ||
      message->header.frame_id != base_frame_)
    {
      obstacle_snapshot_.valid = false;
      return;
    }

    if (current_mode_ != Mode::ROAM) {
      // FOLLOW/IDLE 只维护点云时效，避免空闲时重复转换整幅滚动地图。
      obstacle_snapshot_.valid = true;
      return;
    }

    try {
      sensor_msgs::PointCloud2ConstIterator<float> x_iterator(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y_iterator(*message, "y");
      obstacle_snapshot_.points_odom.reserve(
        std::min<std::size_t>(message->width * message->height, 5000U));
      for (; x_iterator != x_iterator.end(); ++x_iterator, ++y_iterator) {
        const double x = static_cast<double>(*x_iterator);
        const double y = static_cast<double>(*y_iterator);
        if (!std::isfinite(x) || !std::isfinite(y)) {
          continue;
        }
        obstacle_snapshot_.points_odom.push_back(
          transformBasePointToOdom(Point2D{x, y}, odom_snapshot_.pose));
      }
      obstacle_snapshot_.points_ready_for_roam = true;
      obstacle_snapshot_.valid = true;
    } catch (const std::exception & exception) {
      obstacle_snapshot_.valid = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Reject invalid obstacle cloud: %s", exception.what());
    }
  }

  // 保存局部规划器已经完成避障和限幅的最终内部速度。
  void plannerCommandCallback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    const double linear_x = message->linear.x;
    const double angular_z = message->angular.z;
    planner_command_snapshot_.valid = std::isfinite(linear_x) && std::isfinite(angular_z);
    planner_command_snapshot_.velocity = planner_command_snapshot_.valid ?
      Velocity2D{linear_x, angular_z} : Velocity2D{};
    planner_command_snapshot_.receipt_time = std::chrono::steady_clock::now();
  }

  // 提取规划器状态，用于比纯距离超时更快识别持续 BLOCKED。
  void plannerDiagnosticsCallback(
    const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message)
  {
    for (const auto & status : message->status) {
      if (status.hardware_id == "go2_stereo_local_planner") {
        planner_state_ = status.message;
        planner_diagnostics_time_ = std::chrono::steady_clock::now();
        planner_diagnostics_valid_ = true;
        return;
      }
    }
  }

  // 接收上层持久模式请求，并在抢占漫游时先进入受控停车。
  void behaviorServiceCallback(
    const std::shared_ptr<SetBehavior::Request> request,
    std::shared_ptr<SetBehavior::Response> response)
  {
    if (request->mode > SetBehavior::Request::STOP ||
      request->mode == SetBehavior::Request::ROAM)
    {
      response->accepted = false;
      response->current_mode = modeValue(current_mode_);
      response->message = "ROAM 必须通过 /go2/random_roam Action 启动";
      return;
    }

    const Mode requested = static_cast<Mode>(request->mode);
    if (roam_goal_handle_) {
      const auto code = RandomRoam::Result::PREEMPTED_BY_MODE;
      beginFinalStop(
        code, "漫游被行为模式请求抢占", CompletionDisposition::ABORT, requested);
      response->accepted = true;
      response->current_mode = modeValue(current_mode_);
      response->message = "已接受，正在停车并切换模式";
      return;
    }

    setMode(requested);
    response->accepted = true;
    response->current_mode = modeValue(current_mode_);
    response->message = "行为模式已切换为 " + modeName(current_mode_);
  }

  // 校验漫游 Goal 的超时范围、STOP 锁存和单任务互斥条件。
  rclcpp_action::GoalResponse handleRoamGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const RandomRoam::Goal> goal)
  {
    if (current_mode_ == Mode::STOP || roam_goal_handle_) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!std::isfinite(goal->timeout_sec) || goal->timeout_sec < 0.0 ||
      goal->timeout_sec > maximum_roam_timeout_sec_ ||
      (goal->timeout_sec > 0.0 && goal->timeout_sec < 5.0))
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // 将上层取消请求转换为停车后返回 CANCELED 的状态转换。
  rclcpp_action::CancelResponse handleRoamCancel(
    const std::shared_ptr<GoalHandleRandomRoam> goal_handle)
  {
    if (!roam_goal_handle_ || goal_handle != roam_goal_handle_) {
      return rclcpp_action::CancelResponse::REJECT;
    }
    beginFinalStop(
      RandomRoam::Result::CANCELED, "上层取消随机漫游",
      CompletionDisposition::CANCEL, Mode::IDLE);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // 初始化一次漫游任务的随机源、计时器和准备停车阶段。
  void handleRoamAccepted(const std::shared_ptr<GoalHandleRandomRoam> goal_handle)
  {
    roam_goal_handle_ = goal_handle;
    const auto goal = goal_handle->get_goal();
    const std::uint32_t seed = goal->random_seed == 0U ?
      std::random_device{}() : goal->random_seed;
    random_generator_.seed(seed);
    active_roam_timeout_sec_ = goal->timeout_sec > 0.0 ?
      goal->timeout_sec : default_roam_timeout_sec_;
    roam_started_time_ = std::chrono::steady_clock::now();
    phase_started_time_ = roam_started_time_;
    current_mode_ = Mode::ROAM;
    roam_phase_ = RoamPhase::PREPARING_STOP;
    // 必须等待进入 ROAM 后收到的新点云，不能把 FOLLOW 阶段未解析的空缓存当成无障碍。
    obstacle_snapshot_.points_ready_for_roam = false;
    retry_count_ = 0;
    roam_goal_valid_ = false;
    stop_stable_elapsed_sec_ = 0.0;
    blocked_elapsed_sec_ = 0.0;
    geofence_reject_elapsed_sec_ = 0.0;
    completion_disposition_ = CompletionDisposition::NONE;
    state_ = "ROAM_PREPARING";
  }

  // 固定频率执行当前模式、发布唯一名义速度并门控规划器最终速度。
  void controlTick()
  {
    const auto current = std::chrono::steady_clock::now();
    const double measured_dt = std::chrono::duration<double>(current - last_control_time_).count();
    last_control_time_ = current;
    const double dt = std::clamp(measured_dt, 0.0, 2.0 / control_frequency_);

    updateOwnerFilter(current);
    Velocity2D nominal;
    nominal_allows_motion_ = false;
    if (current_mode_ == Mode::FOLLOW) {
      nominal = computeFollowNominal(current);
    } else if (current_mode_ == Mode::ROAM && roam_goal_handle_) {
      nominal = processRoam(current, dt);
    } else {
      state_ = current_mode_ == Mode::STOP ? "STOP" : "IDLE";
    }
    publishNominal(nominal);

    const Velocity2D output = selectSafePlannerCommand(current, dt);
    publishVelocity(output);
    publishRoamFeedback(current);
    publishDiagnosticsIfDue(current);
  }

  // 使用每个新 UWB 样本和当前 odom 位姿更新实时主人位置及迟滞中心。
  void updateOwnerFilter(const SteadyTime & current)
  {
    if (!target_snapshot_.valid || !odom_snapshot_.valid ||
      target_snapshot_.version == processed_target_version_)
    {
      return;
    }
    double dt = 1.0 / control_frequency_;
    if (have_owner_filter_time_) {
      dt = std::max(
        kMinimumAge,
        std::chrono::duration<double>(
          target_snapshot_.receipt_time - last_owner_filter_time_).count());
    }
    const Point2D owner_odom = transformBasePointToOdom(
      target_snapshot_.point_base, odom_snapshot_.pose);
    if (owner_filter_->update(owner_odom, dt)) {
      processed_target_version_ = target_snapshot_.version;
      last_owner_filter_time_ = target_snapshot_.receipt_time;
      have_owner_filter_time_ = true;
      publishPlayCenter(current);
    }
  }

  // 复用现有跟随核心计算 FOLLOW 名义速度并保持原超时停车语义。
  Velocity2D computeFollowNominal(const SteadyTime & current)
  {
    if (!targetFresh(current)) {
      resetFollowAngularState();
      state_ = target_snapshot_.valid ? "FOLLOW_TARGET_TIMEOUT" : "FOLLOW_WAIT_TARGET";
      follow_result_valid_ = false;
      return Velocity2D{};
    }
    if (!odomFresh(current)) {
      resetFollowAngularState();
      state_ = odom_snapshot_.valid ? "FOLLOW_ODOM_TIMEOUT" : "FOLLOW_WAIT_ODOM";
      follow_result_valid_ = false;
      return Velocity2D{};
    }

    last_follow_result_ = computeControlledTarget(
      target_snapshot_.point_base, follow_config_, follow_turn_direction_,
      follow_brake_latched_);
    follow_result_valid_ = true;
    nominal_allows_motion_ = true;
    if (last_follow_result_.blind_rotation) {
      state_ = "FOLLOW_BLIND_ROTATE";
    } else if (last_follow_result_.within_follow_distance) {
      state_ = "FOLLOW_HOLD_DISTANCE";
    } else {
      state_ = "FOLLOWING";
    }
    return fromFollowVelocity(last_follow_result_.target_velocity);
  }

  // 推进随机漫游状态机并返回本周期应送入避障规划器的名义速度。
  Velocity2D processRoam(const SteadyTime & current, double dt)
  {
    if (roam_phase_ != RoamPhase::FINAL_STOP &&
      elapsedSince(roam_started_time_, current) > active_roam_timeout_sec_)
    {
      beginFinalStop(
        RandomRoam::Result::TIMEOUT, "随机漫游超过总超时",
        CompletionDisposition::ABORT, Mode::IDLE);
    }

    switch (roam_phase_) {
      case RoamPhase::PREPARING_STOP:
        state_ = "ROAM_PREPARING";
        if (elapsedSince(phase_started_time_, current) > readiness_timeout_sec_ &&
          !roamInputsReady(current))
        {
          beginFinalStop(
            RandomRoam::Result::NOT_READY, "UWB、里程计、障碍或规划器尚未就绪",
            CompletionDisposition::ABORT, Mode::IDLE);
          return Velocity2D{};
        }
        if (roamInputsReady(current) && updateStopped(dt, current)) {
          setRoamPhase(RoamPhase::SELECTING_GOAL, current);
        }
        return Velocity2D{};

      case RoamPhase::SELECTING_GOAL:
        state_ = "ROAM_SELECTING_GOAL";
        if (!movingInputsHealthy(current)) {
          beginInputTimeoutStop("选择目标时关键输入失效");
          return Velocity2D{};
        }
        if (!selectRandomGoal(current)) {
          beginFinalStop(
            RandomRoam::Result::NO_VALID_GOAL, "未找到满足半径、步长和净空的随机目标",
            CompletionDisposition::ABORT, Mode::IDLE);
        }
        return Velocity2D{};

      case RoamPhase::NAVIGATING:
        return processNavigating(current, dt);

      case RoamPhase::RETURNING:
        return processReturning(current, dt);

      case RoamPhase::ARRIVAL_STOP:
        state_ = "ROAM_ARRIVAL_STOP";
        if (updateStopped(dt, current)) {
          beginFinalStop(
            RandomRoam::Result::SUCCESS, "随机目标已到达且机器人已停稳",
            CompletionDisposition::SUCCEED, Mode::IDLE);
        }
        return Velocity2D{};

      case RoamPhase::RETRY_STOP:
        state_ = "ROAM_RETRY_STOP";
        if (updateStopped(dt, current)) {
          if (retry_count_ < max_retries_) {
            ++retry_count_;
            setRoamPhase(RoamPhase::SELECTING_GOAL, current);
          } else {
            beginFinalStop(
              RandomRoam::Result::BLOCKED, "三个随机目标均无进展或被阻断",
              CompletionDisposition::ABORT, Mode::IDLE);
          }
        }
        return Velocity2D{};

      case RoamPhase::FINAL_STOP:
        state_ = "ROAM_FINAL_STOP";
        processFinalStop(current, dt);
        return Velocity2D{};

      case RoamPhase::INACTIVE:
      default:
        return Velocity2D{};
    }
  }

  // 控制固定随机目标，处理到达、主人外移、输入超时和受阻重采样。
  Velocity2D processNavigating(const SteadyTime & current, double dt)
  {
    state_ = "ROAM_NAVIGATING";
    if (!movingInputsHealthy(current) || !roam_goal_valid_) {
      beginInputTimeoutStop("随机目标导航期间关键输入失效");
      return Velocity2D{};
    }
    const double owner_distance = currentOwnerDistance();
    if (owner_distance > geofence_config_.return_trigger_radius) {
      roam_goal_valid_ = false;
      progress_monitor_->reset(owner_distance);
      setRoamPhase(RoamPhase::RETURNING, current);
      return Velocity2D{};
    }

    const Point2D goal_base = transformOdomPointToBase(roam_goal_odom_, odom_snapshot_.pose);
    const double goal_distance = std::hypot(goal_base.x, goal_base.y);
    last_goal_distance_ = goal_distance;
    if (goal_distance <= roam_control_config_.distance_deadband) {
      setRoamPhase(RoamPhase::ARRIVAL_STOP, current);
      return Velocity2D{};
    }
    if (updateBlockedState(current, dt) || progress_monitor_->update(goal_distance, dt)) {
      setRoamPhase(RoamPhase::RETRY_STOP, current);
      return Velocity2D{};
    }

    const FollowResult result = computeControlledTarget(
      goal_base, roam_control_config_, roam_turn_direction_, roam_brake_latched_);
    nominal_allows_motion_ = true;
    return fromFollowVelocity(result.target_velocity);
  }

  // 在主人超出软边界后优先回到释放半径，再重新选择随机目标。
  Velocity2D processReturning(const SteadyTime & current, double dt)
  {
    state_ = "ROAM_RETURNING";
    if (!movingInputsHealthy(current)) {
      beginInputTimeoutStop("返回主人期间关键输入失效");
      return Velocity2D{};
    }
    const double owner_distance = currentOwnerDistance();
    last_goal_distance_ = std::max(0.0, owner_distance - geofence_config_.return_release_radius);
    if (owner_distance <= geofence_config_.return_release_radius) {
      resetRoamAngularState();
      setRoamPhase(RoamPhase::SELECTING_GOAL, current);
      return Velocity2D{};
    }
    if (updateBlockedState(current, dt) || progress_monitor_->update(owner_distance, dt)) {
      beginFinalStop(
        RandomRoam::Result::BLOCKED, "返回主人方向的安全轨迹持续受阻",
        CompletionDisposition::ABORT, Mode::IDLE);
      return Velocity2D{};
    }

    FollowConfig return_config = roam_control_config_;
    return_config.follow_distance = geofence_config_.return_release_radius;
    return_config.distance_deadband = 0.0;
    const Point2D owner_base = transformOdomPointToBase(
      owner_filter_->filteredOwner(), odom_snapshot_.pose);
    const FollowResult result = computeControlledTarget(
      owner_base, return_config, roam_turn_direction_, roam_brake_latched_);
    nominal_allows_motion_ = true;
    return fromFollowVelocity(result.target_velocity);
  }

  // 统一调用原跟随核心和动态角速度刹车，并更新各模式独立锁存状态。
  FollowResult computeControlledTarget(
    const Point2D & target_base,
    const FollowConfig & config,
    int & turn_direction,
    bool & brake_latched)
  {
    FollowResult result = go2_uwb_local_follow::computeFollowTarget(
      target_base.x, target_base.y, config);
    const auto brake = go2_uwb_local_follow::applyDynamicAngularBrake(
      result.heading, result.target_velocity.angular_z, odom_snapshot_.velocity.angular_z,
      config, turn_direction, brake_latched);
    turn_direction = brake.turn_direction;
    brake_latched = brake.brake_latched;
    result.turn_direction = brake.turn_direction;
    result.actual_angular_z = odom_snapshot_.velocity.angular_z;
    result.brake_angle = brake.brake_angle;
    result.dynamic_stop_angle = brake.dynamic_stop_angle;
    result.angular_braking = brake.braking;
    result.angular_brake_latched = brake.brake_latched;
    result.target_velocity.angular_z = brake.angular_z;
    return result;
  }

  // 在当前主人圆环、机器人单步范围与障碍净空交集中选择固定 odom 目标。
  bool selectRandomGoal(const SteadyTime & current)
  {
    const auto goal = sampleRandomGoal(
      owner_filter_->playCenter(),
      Point2D{odom_snapshot_.pose.x, odom_snapshot_.pose.y},
      obstacle_snapshot_.points_odom,
      sampling_config_, random_generator_);
    if (!goal.has_value()) {
      return false;
    }
    roam_goal_odom_ = *goal;
    roam_goal_valid_ = true;
    last_goal_distance_ = distanceBetween(
      roam_goal_odom_, Point2D{odom_snapshot_.pose.x, odom_snapshot_.pose.y});
    progress_monitor_->reset(last_goal_distance_);
    blocked_elapsed_sec_ = 0.0;
    resetRoamAngularState();
    publishRoamTarget();
    setRoamPhase(RoamPhase::NAVIGATING, current);
    return true;
  }

  // 根据规划器诊断累计 BLOCKED 持续时间并返回是否达到重试门限。
  bool updateBlockedState(const SteadyTime & current, double dt)
  {
    const bool fresh = planner_diagnostics_valid_ &&
      elapsedSince(planner_diagnostics_time_, current) <= planner_diagnostics_timeout_sec_;
    if (fresh && planner_state_.find("BLOCKED") != std::string::npos) {
      blocked_elapsed_sec_ += dt;
    } else {
      blocked_elapsed_sec_ = 0.0;
    }
    return blocked_elapsed_sec_ >= planner_blocked_timeout_sec_;
  }

  // 从规划器内部输出中选择本周期可发布的速度，并执行模式、超时和围栏门控。
  Velocity2D selectSafePlannerCommand(const SteadyTime & current, double dt)
  {
    const bool active_motion_phase = current_mode_ == Mode::FOLLOW ||
      (current_mode_ == Mode::ROAM &&
      (roam_phase_ == RoamPhase::NAVIGATING || roam_phase_ == RoamPhase::RETURNING));
    if (!enable_motion_ || !active_motion_phase || !nominal_allows_motion_ ||
      !obstacleFresh(current) || !plannerCommandFresh(current))
    {
      geofence_reject_elapsed_sec_ = 0.0;
      return Velocity2D{};
    }

    const Velocity2D command = planner_command_snapshot_.velocity;
    if (current_mode_ != Mode::ROAM || !owner_filter_->valid() || !odomFresh(current)) {
      return command;
    }

    last_geofence_result_ = evaluateGeofenceCommand(
      odom_snapshot_.pose, owner_filter_->filteredOwner(), command, geofence_config_);
    if (last_geofence_result_.allowed) {
      geofence_reject_elapsed_sec_ = 0.0;
      return command;
    }

    // 只拒绝整条已规划速度，不拼接未经避障验证的新非零速度。
    geofence_reject_elapsed_sec_ += dt;
    if (geofence_reject_elapsed_sec_ >= geofence_reject_timeout_sec_ && roam_goal_handle_) {
      beginFinalStop(
        RandomRoam::Result::GEOFENCE_STOP, "规划器速度持续触发 6 米围栏",
        CompletionDisposition::ABORT, Mode::IDLE);
    }
    return Velocity2D{};
  }

  // 将漫游失败原因转换为停车后统一返回的输入超时结果。
  void beginInputTimeoutStop(const std::string & message)
  {
    beginFinalStop(
      RandomRoam::Result::INPUT_TIMEOUT, message,
      CompletionDisposition::ABORT, Mode::IDLE);
  }

  // 进入最终停车阶段并保存完成方式、结果及停车后的目标模式。
  void beginFinalStop(
    std::uint8_t result_code,
    const std::string & result_message,
    CompletionDisposition disposition,
    Mode next_mode)
  {
    if (!roam_goal_handle_) {
      setMode(next_mode);
      return;
    }
    pending_result_code_ = result_code;
    pending_result_message_ = result_message;
    completion_disposition_ = disposition;
    pending_mode_ = next_mode;
    roam_phase_ = RoamPhase::FINAL_STOP;
    phase_started_time_ = std::chrono::steady_clock::now();
    stop_stable_elapsed_sec_ = 0.0;
    nominal_allows_motion_ = false;
  }

  // 等待里程计确认停稳；成功结果绝不在未确认停车时返回。
  void processFinalStop(const SteadyTime & current, double dt)
  {
    if (updateStopped(dt, current)) {
      finishRoamAction();
      return;
    }
    if (elapsedSince(phase_started_time_, current) > stop_confirmation_timeout_sec_) {
      if (completion_disposition_ == CompletionDisposition::SUCCEED) {
        pending_result_code_ = RandomRoam::Result::STOP_UNCONFIRMED;
        pending_result_message_ = "到达目标但未能从里程计确认停稳";
        completion_disposition_ = CompletionDisposition::ABORT;
      }
      finishRoamAction();
    }
  }

  // 构造最终结果并按成功、失败或取消终态结束 Action。
  void finishRoamAction()
  {
    if (!roam_goal_handle_) {
      return;
    }
    auto result = std::make_shared<RandomRoam::Result>();
    result->code = pending_result_code_;
    result->message = pending_result_message_;
    result->reached_pose = currentRobotPoseMessage();
    result->owner_distance = currentOwnerDistance();
    result->min_clearance = currentMinimumClearance();

    if (completion_disposition_ == CompletionDisposition::SUCCEED) {
      roam_goal_handle_->succeed(result);
    } else if (completion_disposition_ == CompletionDisposition::CANCEL) {
      roam_goal_handle_->canceled(result);
    } else {
      roam_goal_handle_->abort(result);
    }
    roam_goal_handle_.reset();
    roam_goal_valid_ = false;
    roam_phase_ = RoamPhase::INACTIVE;
    completion_disposition_ = CompletionDisposition::NONE;
    setMode(pending_mode_);
  }

  // 更新连续停车确认时间，任一速度或里程计异常都会清零计时。
  bool updateStopped(double dt, const SteadyTime & current)
  {
    if (odomFresh(current) && isRobotStopped(
        odom_snapshot_.velocity, stop_linear_threshold_, stop_angular_threshold_))
    {
      stop_stable_elapsed_sec_ += dt;
    } else {
      stop_stable_elapsed_sec_ = 0.0;
    }
    return stop_stable_elapsed_sec_ >= stop_confirm_sec_;
  }

  // 切换漫游子状态并重置该阶段独立的停车和阻塞计时。
  void setRoamPhase(RoamPhase phase, const SteadyTime & current)
  {
    roam_phase_ = phase;
    phase_started_time_ = current;
    stop_stable_elapsed_sec_ = 0.0;
    blocked_elapsed_sec_ = 0.0;
  }

  // 切换非漫游模式并清除与上一控制目标相关的角速度锁存。
  void setMode(Mode mode)
  {
    current_mode_ = mode;
    roam_phase_ = RoamPhase::INACTIVE;
    roam_goal_valid_ = false;
    resetFollowAngularState();
    resetRoamAngularState();
    nominal_allows_motion_ = false;
    state_ = mode == Mode::FOLLOW ? "FOLLOW_WAIT_TARGET" :
      (mode == Mode::STOP ? "STOP" : "IDLE");
  }

  // 清空 FOLLOW 独立的转向方向与动态刹车锁存。
  void resetFollowAngularState()
  {
    follow_turn_direction_ = 0;
    follow_brake_latched_ = false;
  }

  // 清空 ROAM 独立的转向方向与动态刹车锁存。
  void resetRoamAngularState()
  {
    roam_turn_direction_ = 0;
    roam_brake_latched_ = false;
  }

  // 返回漫游启动所需的 UWB、里程计、障碍、规划器及滤波样本是否齐备。
  bool roamInputsReady(const SteadyTime & current) const
  {
    return movingInputsHealthy(current) &&
           owner_filter_->sampleCount() >= minimum_owner_samples_;
  }

  // 返回漫游过程中不可缺失的传感输入、障碍缓存和规划器输出是否新鲜有效。
  bool movingInputsHealthy(const SteadyTime & current) const
  {
    return targetFresh(current) && odomFresh(current) && obstacleFresh(current) &&
           obstacle_snapshot_.points_ready_for_roam && plannerCommandFresh(current) &&
           owner_filter_->valid();
  }

  // 返回 UWB 目标是否在允许时限内。
  bool targetFresh(const SteadyTime & current) const
  {
    return target_snapshot_.valid &&
           elapsedSince(target_snapshot_.receipt_time, current) <= target_timeout_sec_;
  }

  // 返回 odom 位姿和速度是否在允许时限内。
  bool odomFresh(const SteadyTime & current) const
  {
    return odom_snapshot_.valid &&
           elapsedSince(odom_snapshot_.receipt_time, current) <= odom_timeout_sec_;
  }

  // 返回滚动障碍点云是否在允许时限内。
  bool obstacleFresh(const SteadyTime & current) const
  {
    return obstacle_snapshot_.valid &&
           elapsedSince(obstacle_snapshot_.receipt_time, current) <= obstacle_timeout_sec_;
  }

  // 返回规划器内部速度是否在允许时限内。
  bool plannerCommandFresh(const SteadyTime & current) const
  {
    return planner_command_snapshot_.valid &&
           elapsedSince(planner_command_snapshot_.receipt_time, current) <=
           planner_cmd_timeout_sec_;
  }

  // 计算稳态时间点到当前时刻的非负秒数。
  double elapsedSince(const SteadyTime & start, const SteadyTime & current) const
  {
    return std::max(kMinimumAge, std::chrono::duration<double>(current - start).count());
  }

  // 返回当前实时主人距离，无有效中心时返回无穷大。
  double currentOwnerDistance() const
  {
    if (!owner_filter_ || !owner_filter_->valid() || !odom_snapshot_.valid) {
      return std::numeric_limits<double>::infinity();
    }
    return distanceBetween(
      owner_filter_->filteredOwner(), Point2D{odom_snapshot_.pose.x, odom_snapshot_.pose.y});
  }

  // 返回机器人足迹边缘到最近缓存障碍点的净空。
  double currentMinimumClearance() const
  {
    if (!obstacle_snapshot_.valid || !odom_snapshot_.valid) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    if (obstacle_snapshot_.points_odom.empty()) {
      return std::numeric_limits<double>::infinity();
    }
    const Point2D robot{odom_snapshot_.pose.x, odom_snapshot_.pose.y};
    double minimum = std::numeric_limits<double>::infinity();
    for (const auto & obstacle : obstacle_snapshot_.points_odom) {
      minimum = std::min(minimum, distanceBetween(robot, obstacle));
    }
    return std::max(0.0, minimum - robot_clearance_radius_);
  }

  // 发布带时间戳的唯一名义速度，供原局部规划器执行避障。
  void publishNominal(const Velocity2D & velocity)
  {
    geometry_msgs::msg::TwistStamped message;
    message.header.stamp = now();
    message.header.frame_id = base_frame_;
    message.twist.linear.x = velocity.linear_x;
    message.twist.angular.z = velocity.angular_z;
    nominal_cmd_pub_->publish(message);
  }

  // 发布最终底盘速度并将所有未使用自由度保持为零。
  void publishVelocity(const Velocity2D & velocity)
  {
    geometry_msgs::msg::Twist message;
    message.linear.x = velocity.linear_x;
    message.angular.z = velocity.angular_z;
    cmd_vel_pub_->publish(message);
  }

  // 发布固定随机目标供 RViz 检查目标选择和到达过程。
  void publishRoamTarget()
  {
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = now();
    message.header.frame_id = odom_frame_;
    message.pose.position.x = roam_goal_odom_.x;
    message.pose.position.y = roam_goal_odom_.y;
    message.pose.orientation.w = 1.0;
    roam_target_pub_->publish(message);
  }

  // 在主人中心收到新样本后发布迟滞中心供可视化。
  void publishPlayCenter(const SteadyTime &)
  {
    if (!owner_filter_->valid()) {
      return;
    }
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = now();
    message.header.frame_id = odom_frame_;
    message.pose.position.x = owner_filter_->playCenter().x;
    message.pose.position.y = owner_filter_->playCenter().y;
    message.pose.orientation.w = 1.0;
    play_center_pub_->publish(message);
  }

  // 构造当前机器人 odom 位姿用于漫游结果返回。
  geometry_msgs::msg::PoseStamped currentRobotPoseMessage() const
  {
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = now();
    message.header.frame_id = odom_frame_;
    if (!odom_snapshot_.valid) {
      message.pose.orientation.w = 1.0;
      return message;
    }
    message.pose.position.x = odom_snapshot_.pose.x;
    message.pose.position.y = odom_snapshot_.pose.y;
    message.pose.orientation.z = std::sin(odom_snapshot_.pose.yaw * 0.5);
    message.pose.orientation.w = std::cos(odom_snapshot_.pose.yaw * 0.5);
    return message;
  }

  // 按限制频率向当前 Action 客户端发布状态、目标和剩余距离。
  void publishRoamFeedback(const SteadyTime & current)
  {
    if (!roam_goal_handle_ ||
      (have_feedback_time_ && current - last_feedback_time_ < feedback_period_))
    {
      return;
    }
    have_feedback_time_ = true;
    last_feedback_time_ = current;
    auto feedback = std::make_shared<RandomRoam::Feedback>();
    feedback->state = feedbackState();
    if (roam_goal_valid_) {
      feedback->target_pose.header.stamp = now();
      feedback->target_pose.header.frame_id = odom_frame_;
      feedback->target_pose.pose.position.x = roam_goal_odom_.x;
      feedback->target_pose.pose.position.y = roam_goal_odom_.y;
      feedback->target_pose.pose.orientation.w = 1.0;
    }
    feedback->distance_remaining = last_goal_distance_;
    feedback->owner_distance = currentOwnerDistance();
    feedback->elapsed_sec = elapsedSince(roam_started_time_, current);
    roam_goal_handle_->publish_feedback(feedback);
  }

  // 将内部漫游子状态转换为 Action Feedback 公共状态值。
  std::uint8_t feedbackState() const
  {
    switch (roam_phase_) {
      case RoamPhase::PREPARING_STOP:
        return RandomRoam::Feedback::PREPARING;
      case RoamPhase::SELECTING_GOAL:
        return RandomRoam::Feedback::SELECTING_GOAL;
      case RoamPhase::NAVIGATING:
        return RandomRoam::Feedback::NAVIGATING;
      case RoamPhase::RETURNING:
        return RandomRoam::Feedback::RETURNING;
      case RoamPhase::RETRY_STOP:
        return RandomRoam::Feedback::RETRYING;
      case RoamPhase::ARRIVAL_STOP:
      case RoamPhase::FINAL_STOP:
      default:
        return RandomRoam::Feedback::STOPPING;
    }
  }

  // 周期发布行为模式、输入时效、围栏和漫游进度诊断。
  void publishDiagnosticsIfDue(const SteadyTime & current)
  {
    if (have_diagnostic_time_ && current - last_diagnostic_time_ < diagnostic_period_) {
      return;
    }
    have_diagnostic_time_ = true;
    last_diagnostic_time_ = current;

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    const bool healthy = current_mode_ == Mode::IDLE || current_mode_ == Mode::STOP ||
      (current_mode_ == Mode::FOLLOW && targetFresh(current) && odomFresh(current)) ||
      (current_mode_ == Mode::ROAM && movingInputsHealthy(current));
    status.level = healthy ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.name = get_fully_qualified_name() + std::string(": UWB behavior controller");
    status.hardware_id = "go2_uwb_behavior";
    status.message = state_;
    const std::pair<std::string, std::string> entries[] = {
      {"mode", modeName(current_mode_)},
      {"enable_motion", enable_motion_ ? "true" : "false"},
      {"target_age_sec",
        snapshotAge(target_snapshot_.valid, target_snapshot_.receipt_time, current)},
      {"odom_age_sec", snapshotAge(odom_snapshot_.valid, odom_snapshot_.receipt_time, current)},
      {"obstacle_age_sec",
        snapshotAge(obstacle_snapshot_.valid, obstacle_snapshot_.receipt_time, current)},
      {"planner_cmd_age_sec",
        snapshotAge(
          planner_command_snapshot_.valid, planner_command_snapshot_.receipt_time, current)},
      {"owner_distance", formatDouble(currentOwnerDistance())},
      {"goal_distance", roam_goal_valid_ ? formatDouble(last_goal_distance_) : "n/a"},
      {"retry_count", std::to_string(retry_count_)},
      {"planner_state", planner_state_},
      {"geofence_allowed", last_geofence_result_.allowed ? "true" : "false"},
      {"geofence_predicted_max", formatDouble(last_geofence_result_.maximum_distance)}};
    for (const auto & entry : entries) {
      diagnostic_msgs::msg::KeyValue value;
      value.key = entry.first;
      value.value = entry.second;
      status.values.push_back(std::move(value));
    }
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(array);
    publishFollowDiagnostics(current);
  }

  // 在 FOLLOW 模式继续发布兼容的话题和关键字段，便于沿用现有监控命令。
  void publishFollowDiagnostics(const SteadyTime & current)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = current_mode_ == Mode::FOLLOW && targetFresh(current) && odomFresh(current) ?
      diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.name = get_fully_qualified_name() + std::string(": UWB follow compatibility");
    status.hardware_id = "go2_base";
    status.message = state_;
    const std::pair<std::string, std::string> entries[] = {
      {"target_age_sec",
        snapshotAge(target_snapshot_.valid, target_snapshot_.receipt_time, current)},
      {"odom_age_sec", snapshotAge(odom_snapshot_.valid, odom_snapshot_.receipt_time, current)},
      {"distance", follow_result_valid_ ? formatDouble(last_follow_result_.distance) : "n/a"},
      {"heading", follow_result_valid_ ? formatDouble(last_follow_result_.heading) : "n/a"},
      {"nominal_v", follow_result_valid_ ?
        formatDouble(last_follow_result_.target_velocity.linear_x) : "0.000"},
      {"nominal_w", follow_result_valid_ ?
        formatDouble(last_follow_result_.target_velocity.angular_z) : "0.000"}};
    for (const auto & entry : entries) {
      diagnostic_msgs::msg::KeyValue value;
      value.key = entry.first;
      value.value = entry.second;
      status.values.push_back(std::move(value));
    }
    array.status.push_back(std::move(status));
    follow_diagnostics_pub_->publish(array);
  }

  // 将快照有效性和接收时刻转换为诊断年龄文本。
  std::string snapshotAge(bool valid, const SteadyTime & receipt, const SteadyTime & current) const
  {
    return valid ? formatDouble(elapsedSince(receipt, current)) : "n/a";
  }

  // 返回模式对应的公共接口数值。
  std::uint8_t modeValue(Mode mode) const
  {
    return static_cast<std::uint8_t>(mode);
  }

  // 返回供日志和诊断使用的模式名称。
  std::string modeName(Mode mode) const
  {
    switch (mode) {
      case Mode::FOLLOW:
        return "FOLLOW";
      case Mode::STOP:
        return "STOP";
      case Mode::ROAM:
        return "ROAM";
      case Mode::IDLE:
      default:
        return "IDLE";
    }
  }

  std::string base_frame_;
  std::string odom_frame_;
  std::string target_topic_;
  std::string odom_topic_;
  std::string obstacle_topic_;
  std::string nominal_cmd_topic_;
  std::string planner_cmd_topic_;
  std::string planner_diagnostics_topic_;
  std::string cmd_vel_topic_;
  std::string diagnostics_topic_;
  std::string follow_diagnostics_topic_;
  std::string roam_target_topic_;
  std::string play_center_topic_;
  std::string behavior_service_name_;
  std::string roam_action_name_;
  std::string default_mode_;
  bool enable_motion_{true};

  double control_frequency_{20.0};
  double diagnostic_frequency_{2.0};
  double feedback_frequency_{5.0};
  double target_timeout_sec_{0.50};
  double odom_timeout_sec_{0.20};
  double obstacle_timeout_sec_{0.70};
  double planner_cmd_timeout_sec_{0.20};
  double planner_diagnostics_timeout_sec_{0.75};
  double transform_timeout_sec_{0.10};
  double readiness_timeout_sec_{2.0};

  FollowConfig follow_config_;
  FollowConfig roam_control_config_;
  OwnerFilterConfig owner_filter_config_;
  RoamSamplingConfig sampling_config_;
  GeofenceConfig geofence_config_;
  std::size_t minimum_owner_samples_{5U};
  double geofence_reject_timeout_sec_{1.0};
  double default_roam_timeout_sec_{30.0};
  double maximum_roam_timeout_sec_{120.0};
  double required_progress_{0.15};
  double progress_window_sec_{3.0};
  double planner_blocked_timeout_sec_{1.0};
  int max_retries_{2};
  double stop_linear_threshold_{0.04};
  double stop_angular_threshold_{0.08};
  double stop_confirm_sec_{0.30};
  double stop_confirmation_timeout_sec_{2.0};
  double robot_clearance_radius_{0.40};

  Mode current_mode_{Mode::FOLLOW};
  Mode pending_mode_{Mode::IDLE};
  RoamPhase roam_phase_{RoamPhase::INACTIVE};
  CompletionDisposition completion_disposition_{CompletionDisposition::NONE};
  std::string state_{"FOLLOW_WAIT_TARGET"};
  std::uint8_t pending_result_code_{RandomRoam::Result::SUCCESS};
  std::string pending_result_message_;
  bool nominal_allows_motion_{false};

  TargetSnapshot target_snapshot_;
  OdomSnapshot odom_snapshot_;
  ObstacleSnapshot obstacle_snapshot_;
  PlannerCommandSnapshot planner_command_snapshot_;
  std::uint64_t processed_target_version_{0U};
  bool have_owner_filter_time_{false};
  SteadyTime last_owner_filter_time_{};
  std::unique_ptr<OwnerCenterFilter> owner_filter_;
  std::unique_ptr<ProgressMonitor> progress_monitor_;

  Point2D roam_goal_odom_;
  bool roam_goal_valid_{false};
  double last_goal_distance_{0.0};
  int retry_count_{0};
  double blocked_elapsed_sec_{0.0};
  double geofence_reject_elapsed_sec_{0.0};
  double stop_stable_elapsed_sec_{0.0};
  double active_roam_timeout_sec_{30.0};
  std::mt19937 random_generator_;
  SteadyTime roam_started_time_{};
  SteadyTime phase_started_time_{};

  int follow_turn_direction_{0};
  bool follow_brake_latched_{false};
  int roam_turn_direction_{0};
  bool roam_brake_latched_{false};
  FollowResult last_follow_result_;
  bool follow_result_valid_{false};
  GeofenceResult last_geofence_result_;

  std::string planner_state_{"WAIT_PLANNER"};
  bool planner_diagnostics_valid_{false};
  SteadyTime planner_diagnostics_time_{};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr planner_cmd_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    planner_diagnostics_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr nominal_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr follow_diagnostics_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr roam_target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr play_center_pub_;
  rclcpp::Service<SetBehavior>::SharedPtr behavior_service_;
  rclcpp_action::Server<RandomRoam>::SharedPtr roam_action_server_;
  std::shared_ptr<GoalHandleRandomRoam> roam_goal_handle_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::chrono::duration<double> diagnostic_period_{0.5};
  std::chrono::duration<double> feedback_period_{0.2};
  bool have_diagnostic_time_{false};
  bool have_feedback_time_{false};
  SteadyTime last_diagnostic_time_{};
  SteadyTime last_feedback_time_{};
  SteadyTime last_control_time_{};
};

}  // namespace go2_uwb_behavior

// 启动统一 UWB 跟随与随机漫游节点并进入 ROS 事件循环。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<go2_uwb_behavior::UwbBehaviorControllerNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("uwb_behavior_controller_node"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
