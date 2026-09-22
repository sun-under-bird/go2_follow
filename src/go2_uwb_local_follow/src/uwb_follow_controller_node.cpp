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

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "go2_uwb_local_follow/follow_control_core.hpp"
#include "go2_uwb_local_follow/target_motion_core.hpp"
#include "go2_uwb_local_follow/rolling_obstacle_map_core.hpp"
#include "go2_uwb_local_follow/observation_time.hpp"

namespace go2_uwb_local_follow
{
namespace
{

// 把浮点数格式化为诊断话题使用的短字符串。
std::string formatDouble(double value, int precision = 3)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

}  // namespace

class UwbFollowControllerNode : public rclcpp::Node
{
public:
  // 初始化最新目标快照、固定频率跟随控制及隔离速度输出接口。
  explicit UwbFollowControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("uwb_follow_controller_node", options),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    target_topic_ = declare_parameter<std::string>("target_topic", "/uwb/target_point");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom_leg");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_follow");
    nominal_cmd_topic_ = declare_parameter<std::string>(
      "nominal_cmd_topic", "/go2_uwb_local_follow/nominal_cmd");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/go2_uwb_local_follow/follow_diagnostics");
    enable_motion_ = declare_parameter<bool>("enable_motion", true);
    control_frequency_ = declare_parameter<double>("control_frequency", 20.0);
    diagnostic_frequency_ = declare_parameter<double>("diagnostic_frequency", 2.0);
    target_timeout_sec_ = declare_parameter<double>("target_timeout_sec", 0.50);
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 0.20);
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.10);

    config_.follow_distance = declare_parameter<double>("follow_distance", 1.0);
    config_.distance_deadband = declare_parameter<double>("distance_deadband", 0.08);
    config_.angle_deadband = declare_parameter<double>("angle_deadband", 0.20);
    config_.angle_reengage = declare_parameter<double>("angle_reengage", 0.45);
    config_.turn_response_delay = declare_parameter<double>("turn_response_delay", 0.10);
    config_.angular_braking_accel = declare_parameter<double>("angular_braking_accel", 1.50);
    config_.angular_brake_release_speed = declare_parameter<double>(
      "angular_brake_release_speed", 0.06);
    config_.angular_reverse_speed_threshold = declare_parameter<double>(
      "angular_reverse_speed_threshold", 0.15);
    config_.linear_kp = declare_parameter<double>("linear_kp", 0.6);
    config_.angular_kp = declare_parameter<double>("angular_kp", 1.0);
    config_.min_linear_speed = declare_parameter<double>("min_linear_speed", 0.23);
    config_.max_linear_speed = declare_parameter<double>("max_linear_speed", 0.80);
    config_.max_angular_speed = declare_parameter<double>("max_angular_speed", 2.00);
    config_.heading_slowdown_start = declare_parameter<double>(
      "heading_slowdown_start", 0.50);
    config_.heading_stop_angle = declare_parameter<double>("heading_stop_angle", 1.40);
    config_.blind_rotation_max_speed = declare_parameter<double>(
      "blind_rotation_max_speed", 2.00);
    config_.max_linear_accel = declare_parameter<double>("max_linear_accel", 0.80);
    config_.max_linear_decel = declare_parameter<double>("max_linear_decel", 0.80);
    config_.max_angular_accel = declare_parameter<double>("max_angular_accel", 2.00);
    enable_target_estimation_ = declare_parameter<bool>("enable_target_estimation", true);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    target_prediction_sec_ = declare_parameter<double>("target_prediction_sec", 0.20);
    TargetMotionConfig tracking;
    tracking.window_sec = declare_parameter<double>("target_window_sec", tracking.window_sec);
    tracking.reset_gap_sec = declare_parameter<double>(
      "target_reset_gap_sec",
      tracking.reset_gap_sec);
    tracking.max_human_speed = declare_parameter<double>(
      "target_max_human_speed",
      tracking.max_human_speed);
    tracking.jump_margin = declare_parameter<double>("target_jump_margin", tracking.jump_margin);
    tracking.innovation_limit = declare_parameter<double>(
      "target_innovation_limit",
      tracking.innovation_limit);
    tracking.velocity_filter_sec = declare_parameter<double>(
      "target_velocity_filter_sec",
      tracking.velocity_filter_sec);
    tracking.minimum_span_sec = declare_parameter<double>(
      "target_minimum_span_sec",
      tracking.minimum_span_sec);
    tracking.stop_speed = declare_parameter<double>("target_stop_speed", tracking.stop_speed);
    tracking.moving_speed = declare_parameter<double>("target_moving_speed", tracking.moving_speed);
    tracking.turn_angle = declare_parameter<double>("target_turn_angle", tracking.turn_angle);
    tracking.return_angle = declare_parameter<double>("target_return_angle", tracking.return_angle);
    tracking.direction_history_sec = declare_parameter<double>(
      "target_direction_history_sec",
      tracking.direction_history_sec);
    tracking.direction_consistency_angle = declare_parameter<double>(
      "target_direction_consistency_angle", tracking.direction_consistency_angle);
    tracking.minimum_samples = declare_parameter<int>(
      "target_minimum_samples",
      tracking.minimum_samples);
    tracking.confirmation_samples = declare_parameter<int>(
      "target_confirmation_samples",
      tracking.confirmation_samples);
    walk_config_.start_distance_error = declare_parameter<double>(
      "walk_start_distance_error",
      walk_config_.start_distance_error);
    walk_config_.stop_distance_error = declare_parameter<double>(
      "walk_stop_distance_error",
      walk_config_.stop_distance_error);
    walk_config_.close_distance_margin = declare_parameter<double>(
      "walk_close_distance_margin",
      walk_config_.close_distance_margin);
    walk_config_.stop_command_speed = declare_parameter<double>(
      "walk_stop_command_speed",
      walk_config_.stop_command_speed);
    walk_config_.approach_speed = declare_parameter<double>(
      "walk_approach_speed",
      walk_config_.approach_speed);
    walk_config_.confirmation_sec = declare_parameter<double>(
      "walk_confirmation_sec",
      walk_config_.confirmation_sec);
    walk_config_.minimum_hold_sec = declare_parameter<double>(
      "walk_minimum_hold_sec",
      walk_config_.minimum_hold_sec);

    RollingMapConfig pose_config;
    pose_config.odom_buffer_duration_sec = declare_parameter<double>(
      "odom_buffer_duration_sec",
      pose_config.odom_buffer_duration_sec);
    pose_config.max_pose_extrapolation_sec = declare_parameter<double>(
      "max_pose_extrapolation_sec",
      pose_config.max_pose_extrapolation_sec);
    pose_config.max_pose_interpolation_gap_sec = declare_parameter<double>(
      "max_pose_interpolation_gap_sec", pose_config.max_pose_interpolation_gap_sec);
    pose_config.odom_jump_distance = declare_parameter<double>(
      "odom_jump_distance",
      pose_config.odom_jump_distance);
    pose_config.odom_jump_yaw =
      declare_parameter<double>("odom_jump_yaw", pose_config.odom_jump_yaw);
    pose_config.odom_jump_check_interval_sec = declare_parameter<double>(
      "odom_jump_check_interval_sec", pose_config.odom_jump_check_interval_sec);
    estimator_ = std::make_unique<TargetMotionEstimator>(tracking);
    if (!validateRollingMapConfig(pose_config) || !validateWalkHysteresisConfig(walk_config_) ||
      odom_frame_.empty() || !std::isfinite(target_prediction_sec_) ||
      target_prediction_sec_ < 0.0 || target_prediction_sec_ > target_timeout_sec_)
    {throw std::invalid_argument("invalid target estimation or walking configuration");}
    pose_buffer_ = std::make_unique<OdomPoseBuffer>(pose_config);
    target_state_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      declare_parameter<std::string>("target_state_topic", "/uwb/target_state"), 10);
    validateParameters();

    target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      target_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      std::bind(&UwbFollowControllerNode::targetCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&UwbFollowControllerNode::odomCallback, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    nominal_cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(nominal_cmd_topic_, 10);
    enable_cycle_telemetry_ = declare_parameter<bool>("enable_cycle_telemetry", true);
    telemetry_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/go2_uwb_local_follow/follow_cycle", 10);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, 10);

    const auto control_period = std::chrono::duration<double>(1.0 / control_frequency_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(control_period),
      std::bind(&UwbFollowControllerNode::controlTick, this));
    const auto diagnostic_period = std::chrono::duration<double>(1.0 / diagnostic_frequency_);
    diagnostic_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(diagnostic_period),
      std::bind(&UwbFollowControllerNode::diagnosticTick, this));
    last_control_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(
      get_logger(), "UWB follow controller started: target=%s odom=%s cmd=%s enable_motion=%s",
      target_topic_.c_str(), odom_topic_.c_str(), cmd_vel_topic_.c_str(),
      enable_motion_ ? "true" : "false");
  }

  // 节点正常销毁前尽力补发一次零速度。
  ~UwbFollowControllerNode() override
  {
    if (cmd_pub_) {
      cmd_pub_->publish(geometry_msgs::msg::Twist());
    }
  }

private:
  struct TargetSnapshot
  {
    double x{0.0};
    double y{0.0};
    double vx{0.0};
    double vy{0.0};
    builtin_interfaces::msg::Time source_stamp{};
    std::chrono::steady_clock::time_point receipt_time{};
    bool valid{false};
  };

  struct OdomSnapshot
  {
    double angular_z{0.0};
    double linear_x{0.0};
    int64_t stamp_ns{0};
    std::chrono::steady_clock::time_point receipt_time{};
    bool valid{false};
  };

  // 检查控制频率、超时和跟随控制参数。
  void validateParameters()
  {
    std::string reason;
    if (base_frame_.empty() || odom_topic_.empty()) {
      throw std::invalid_argument("base_frame and odom_topic must not be empty");
    }
    if (!validateFollowConfig(config_, &reason)) {
      throw std::invalid_argument(reason);
    }
    if (!std::isfinite(control_frequency_) || control_frequency_ <= 0.0 ||
      !std::isfinite(diagnostic_frequency_) || diagnostic_frequency_ <= 0.0)
    {
      throw std::invalid_argument("control and diagnostic frequencies must be positive");
    }
    if (!std::isfinite(target_timeout_sec_) || target_timeout_sec_ <= 0.0 ||
      !std::isfinite(odom_timeout_sec_) || odom_timeout_sec_ <= 0.0 ||
      !std::isfinite(transform_timeout_sec_) || transform_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument("target, odom and transform timeouts must be positive");
    }
  }

  static int64_t stampNs(const builtin_interfaces::msg::Time & stamp)
  {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000U) {return 0;}
    return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  }

  // All callbacks use the default mutually exclusive callback group.
  void resetTracking(int64_t epoch)
  {
    estimator_->reset();
    walk_state_ = WalkState{};
    latest_target_ = TargetSnapshot{};
    target_epoch_ns_ = epoch;
    last_update_ = "RESET";
  }

  void checkClock()
  {
    if (!enable_target_estimation_) {return;}
    const auto stamp = now().nanoseconds();
    if (last_clock_ns_ > 0 && stamp < last_clock_ns_) {
      resetTracking(stamp);
      pose_buffer_->clear();
      latest_odom_ = OdomSnapshot{};
      last_odom_stamp_ns_ = 0;
    }
    last_clock_ns_ = stamp;
  }

  // 将目标点按其自身时间戳转换到 base_footprint；回调只覆盖最新目标快照。
  void targetCallback(const geometry_msgs::msg::PointStamped::SharedPtr message)
  {
    checkClock();
    const auto source = stampNs(message->header.stamp);
    if (enable_target_estimation_ &&
      (source < target_epoch_ns_ || sourceAge(source, now().nanoseconds()) > target_timeout_sec_))
    {last_update_ = "INVALID_SOURCE_TIME"; return;}
    if (!std::isfinite(message->point.x) || !std::isfinite(message->point.y) ||
      !std::isfinite(message->point.z) || message->header.frame_id.empty())
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Reject invalid target point");
      setState("INVALID_TARGET");
      return;
    }

    tf2::Vector3 target(message->point.x, message->point.y, message->point.z);
    if (message->header.frame_id != base_frame_) {
      try {
        const auto transform = tf_buffer_.lookupTransform(
          base_frame_, message->header.frame_id, rclcpp::Time(message->header.stamp),
          rclcpp::Duration::from_seconds(transform_timeout_sec_));
        const auto & rotation_message = transform.transform.rotation;
        tf2::Quaternion quaternion(
          rotation_message.x, rotation_message.y, rotation_message.z, rotation_message.w);
        if (quaternion.length2() <= std::numeric_limits<double>::epsilon()) {
          throw std::runtime_error("target TF quaternion has zero length");
        }
        quaternion.normalize();
        const tf2::Matrix3x3 rotation(quaternion);
        const tf2::Vector3 translation(
          transform.transform.translation.x,
          transform.transform.translation.y,
          transform.transform.translation.z);
        target = rotation * target + translation;
      } catch (const std::exception & exception) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Target TF failed: %s", exception.what());
        setState("TF_ERROR");
        return;
      }
    }

    TargetSnapshot snapshot;
    snapshot.x = target.x();
    snapshot.y = target.y();
    snapshot.source_stamp = message->header.stamp;
    snapshot.receipt_time = std::chrono::steady_clock::now();
    snapshot.valid = std::isfinite(snapshot.x) && std::isfinite(snapshot.y);
    if (enable_target_estimation_) {
      TimedPose2D pose;
      if (!pose_buffer_->lookup(source, &pose)) {last_update_ = "NO_SOURCE_POSE"; return;}
      const auto world = transformRollingPointToOdom({snapshot.x, snapshot.y, 0.0, source}, pose);
      const auto update = estimator_->observe(world.x, world.y, source);
      last_update_ = targetUpdateName(update);
      if (update == TargetUpdate::OUTLIER) {++outlier_count_;}
      if (update == TargetUpdate::WARMING) {
        latest_target_ = TargetSnapshot{};
        walk_state_ = WalkState{};
      }
      if (update != TargetUpdate::ACCEPTED) {return;}
      const auto & estimate = estimator_->estimate();
      snapshot.x = estimate.x;
      snapshot.y = estimate.y;
      snapshot.vx = estimate.vx;
      snapshot.vy = estimate.vy;
      nav_msgs::msg::Odometry state;
      state.header.stamp = message->header.stamp;
      state.header.frame_id = odom_frame_;
      state.child_frame_id = odom_frame_;  // Human velocity is in the fixed frame, too.
      state.pose.pose.position.x = estimate.x;
      state.pose.pose.position.y = estimate.y;
      state.pose.pose.orientation.z = std::sin(estimate.heading / 2.0);
      state.pose.pose.orientation.w = std::cos(estimate.heading / 2.0);
      state.twist.twist.linear.x = estimate.vx;
      state.twist.twist.linear.y = estimate.vy;
      // This deterministic regression does not estimate covariance; do not claim certainty.
      for (size_t i = 0; i < 6; ++i) {
        state.pose.covariance[i * 7] = 1e6;
        state.twist.covariance[i * 7] = 1e6;
      }
      target_state_pub_->publish(state);
    }
    std::lock_guard<std::mutex> lock(target_mutex_);
    latest_target_ = snapshot;
  }

  // 返回当前最新目标快照，不保留或插值更早的控制样本。
  TargetSnapshot targetSnapshot()
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    return latest_target_;
  }

  // 从 /odom_leg 保存未经命令死区处理的真实角速度，供动态停止角计算。
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    checkClock();
    OdomSnapshot snapshot;
    snapshot.stamp_ns = stampNs(message->header.stamp);
    if (enable_target_estimation_) {
      if (snapshot.stamp_ns <= last_odom_stamp_ns_) {return;}
      const auto & q = message->pose.pose.orientation;
      const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
      TimedPose2D pose{snapshot.stamp_ns, message->pose.pose.position.x,
        message->pose.pose.position.y, 0.0};
      if (message->header.frame_id != odom_frame_ || message->child_frame_id != base_frame_ ||
        !std::isfinite(norm) || norm <= 1e-12 || !std::isfinite(pose.x) || !std::isfinite(pose.y) ||
        !std::isfinite(message->twist.twist.angular.z) ||
        !std::isfinite(message->twist.twist.linear.x) ||
        sourceAge(snapshot.stamp_ns, now().nanoseconds()) > odom_timeout_sec_)
      {latest_odom_ = snapshot; return;}
      pose.yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y) / norm,
        1.0 - 2.0 * (q.y * q.y + q.z * q.z) / norm);
      const auto appended = pose_buffer_->append(pose);
      if (appended == PoseAppendResult::kRejected) {latest_odom_ = snapshot; return;}
      if (appended == PoseAppendResult::kResetDetected) {resetTracking(snapshot.stamp_ns);}
      last_odom_stamp_ns_ = snapshot.stamp_ns;
    }
    snapshot.receipt_time = std::chrono::steady_clock::now();
    snapshot.linear_x = message->twist.twist.linear.x;
    snapshot.angular_z = message->twist.twist.angular.z;
    snapshot.valid = std::isfinite(snapshot.angular_z);
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_odom_ = snapshot;
  }

  // 返回当前最新里程计角速度快照，避免控制周期持锁执行计算。
  OdomSnapshot odomSnapshot()
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    return latest_odom_;
  }

  // 固定频率计算名义速度、执行变化率限制并发布隔离跟随速度。
  void controlTick()
  {
    checkClock();
    controlTickImpl();
    if (enable_cycle_telemetry_) {publishDiagnostic(true);}
  }

  void controlTickImpl()
  {
    const auto current_time = std::chrono::steady_clock::now();
    const double measured_dt =
      std::chrono::duration<double>(current_time - last_control_time_).count();
    last_control_time_ = current_time;
    const double dt = std::clamp(measured_dt, 0.0, 2.0 / control_frequency_);

    const TargetSnapshot target = targetSnapshot();
    if (!target.valid) {
      publishImmediateStop("WAIT_TARGET");
      return;
    }
    const double target_age = std::max(
      std::chrono::duration<double>(current_time - target.receipt_time).count(),
      enable_target_estimation_ ? sourceAge(
        stampNs(target.source_stamp),
        now().nanoseconds()) : 0.0);
    last_target_age_ = target_age;
    if (target_age > target_timeout_sec_) {
      publishImmediateStop("TARGET_LOST");
      return;
    }

    const OdomSnapshot odom = odomSnapshot();
    if (!odom.valid) {
      publishImmediateStop("WAIT_ODOM");
      return;
    }
    const double odom_age = std::max(
      std::chrono::duration<double>(current_time - odom.receipt_time).count(),
      enable_target_estimation_ ? sourceAge(odom.stamp_ns, now().nanoseconds()) : 0.0);
    last_odom_age_ = odom_age;
    if (odom_age > odom_timeout_sec_) {
      publishImmediateStop("ODOM_TIMEOUT");
      return;
    }

    FollowResult result;
    if (enable_target_estimation_) {
      const auto control_stamp = now().nanoseconds();
      TimedPose2D pose;
      if (!pose_buffer_->lookup(control_stamp, &pose)) {
        publishImmediateStop("TARGET_ALIGNMENT_FAILED"); return;
      }
      const double horizon = std::min(
        target_prediction_sec_,
        sourceAge(stampNs(target.source_stamp), control_stamp));
      const auto local = transformRollingPointToBase(
        {target.x + target.vx * horizon, target.y + target.vy * horizon, 0.0, 0}, pose);
      const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
      last_predictive_ = computePredictiveFollow(
        local.x, local.y,
        c * target.vx + s * target.vy, -s * target.vx + c * target.vy,
        stampNs(
          target.source_stamp), control_stamp, config_, walk_config_, walk_state_, odom.linear_x);
      result = last_predictive_.follow;
    } else {
      result = computeFollowTarget(target.x, target.y, config_);
    }
    const DynamicAngularBrakeResult brake = applyDynamicAngularBrake(
      result.heading, result.target_velocity.angular_z, odom.angular_z, config_, turn_direction_,
      angular_brake_latched_);
    // 动态刹车直接覆盖 UWB 名义角速度，确保后级 Kp 不会在刹车区继续补偿。
    turn_direction_ = brake.turn_direction;
    angular_brake_latched_ = brake.brake_latched;
    result.turn_direction = brake.turn_direction;
    result.actual_angular_z = odom.angular_z;
    result.brake_angle = brake.brake_angle;
    result.dynamic_stop_angle = brake.dynamic_stop_angle;
    result.angular_braking = brake.braking;
    result.angular_brake_latched = brake.brake_latched;
    result.target_velocity.angular_z = brake.angular_z;
    Velocity2D previous_output;
    {
      // 即使未来切换到多线程执行器，也只在锁内读取跨回调共享的上一周期速度。
      std::lock_guard<std::mutex> lock(status_mutex_);
      previous_output = last_output_;
    }
    const Velocity2D limited = limitVelocityRate(
      previous_output, result.target_velocity, config_, dt);
    publishNominal(result.target_velocity);

    Velocity2D output = limited;
    if (!enable_motion_) {
      output = Velocity2D{};
      setState("OUTPUT_DISABLED");
    } else if (result.blind_rotation) {
      setState("BLIND_ROTATE");
    } else if (result.within_follow_distance) {
      setState("HOLD_DISTANCE");
    } else {
      setState("FOLLOWING");
    }
    publishVelocity(output);

    std::lock_guard<std::mutex> lock(status_mutex_);
    last_output_ = output;
    last_result_ = result;
    last_target_age_ = target_age;
    last_odom_age_ = odom_age;
    have_result_ = true;
  }

  // 超时或无目标时绕过普通平滑，立即发布零速度并清除历史输出。
  void publishImmediateStop(const std::string & state)
  {
    walk_state_ = WalkState{};
    publishVelocity(Velocity2D{});
    publishNominal(Velocity2D{});
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      last_output_ = Velocity2D{};
      turn_direction_ = 0;
      // 超时期间虽然持续输出零速度，但刹车锁存只能由有效里程计确认停稳后解除。
      have_result_ = false;
      state_ = state;
    }
  }

  // 发布 geometry_msgs/Twist，并强制所有未使用自由度为零。
  void publishVelocity(const Velocity2D & velocity)
  {
    geometry_msgs::msg::Twist message;
    message.linear.x = std::max(0.0, velocity.linear_x);
    message.angular.z = velocity.angular_z;
    cmd_pub_->publish(message);
  }

  // 发布带时间戳的未限变化率名义速度，供后续避障规划和 PlotJuggler 使用。
  void publishNominal(const Velocity2D & velocity)
  {
    geometry_msgs::msg::TwistStamped message;
    message.header.stamp = now();
    message.header.frame_id = base_frame_;
    message.twist.linear.x = std::max(0.0, velocity.linear_x);
    message.twist.angular.z = velocity.angular_z;
    nominal_cmd_pub_->publish(message);
  }

  // 线程安全地更新用于诊断的控制状态名称。
  void setState(const std::string & state)
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    state_ = state;
  }

  // 周期发布目标距离、角度、速度和最新样本年龄。
  void diagnosticTick()
  {
    publishDiagnostic(false);
  }

  void publishDiagnostic(bool cycle)
  {
    std::string state;
    Velocity2D output;
    FollowResult result;
    double target_age = 0.0;
    double odom_age = 0.0;
    bool have_result = false;
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      state = state_;
      output = last_output_;
      result = last_result_;
      target_age = last_target_age_;
      odom_age = last_odom_age_;
      have_result = have_result_;
    }

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = state == "FOLLOWING" || state == "HOLD_DISTANCE" ||
      state == "BLIND_ROTATE" ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.name = get_fully_qualified_name() + std::string(": UWB follow controller");
    status.hardware_id = "go2_base";
    status.message = state;
    const std::pair<std::string, std::string> entries[] = {
      {"enable_target_estimation", enable_target_estimation_ ? "true" : "false"},
      {"target_update", last_update_},
      {"target_state_stamp_ns", std::to_string(estimator_->estimate().stamp_ns)},
      {"target_state_age_sec", formatDouble(
          sourceAge(
            estimator_->estimate().stamp_ns, now().nanoseconds()))},
      {"target_estimate_valid", estimator_->estimate().valid && latest_target_.valid &&
        sourceAge(estimator_->estimate().stamp_ns, now().nanoseconds()) <= target_timeout_sec_ ?
        "true" : "false"},
      {"target_outliers", std::to_string(outlier_count_)},
      {"human_motion", humanMotionName(estimator_->estimate().motion)},
      {"human_x_odom", formatDouble(estimator_->estimate().x)},
      {"human_y_odom", formatDouble(estimator_->estimate().y)},
      {"human_vx_odom", formatDouble(estimator_->estimate().vx)},
      {"human_vy_odom", formatDouble(estimator_->estimate().vy)},
      {"human_heading", formatDouble(estimator_->estimate().heading)},
      {"direction_valid", estimator_->estimate().direction_valid ? "true" : "false"},
      {"walking", walk_state_.walking ? "true" : "false"},
      {"walk_reason", !enable_target_estimation_ ? "LEGACY_DISTANCE" :
        have_result ? last_predictive_.reason : state},
      {"feedforward_v", have_result ? formatDouble(last_predictive_.feedforward) : "0.000"},
      {"desired_v", have_result ? formatDouble(last_predictive_.desired_speed) : "0.000"},
      {"within_follow_distance", have_result && result.within_follow_distance ? "true" : "false"},
      {"blind_rotation", have_result && result.blind_rotation ? "true" : "false"},
      {"have_result", have_result ? "true" : "false"},
      {"target_age_sec", formatDouble(target_age)},
      {"odom_age_sec", formatDouble(odom_age)},
      {"distance", have_result ? formatDouble(result.distance) : "n/a"},
      {"heading", have_result ? formatDouble(result.heading) : "n/a"},
      {"actual_wz", have_result ? formatDouble(result.actual_angular_z) : "n/a"},
      {"brake_angle", have_result ? formatDouble(result.brake_angle) : "n/a"},
      {"dynamic_stop_angle", have_result ? formatDouble(result.dynamic_stop_angle) : "n/a"},
      {"angular_braking", have_result && result.angular_braking ? "true" : "false"},
      {"angular_brake_latched",
        have_result && result.angular_brake_latched ? "true" : "false"},
      {"turn_direction", have_result ? std::to_string(result.turn_direction) : "0"},
      {"heading_scale", have_result ? formatDouble(result.heading_scale) : "n/a"},
      {"nominal_v", have_result ? formatDouble(result.target_velocity.linear_x) : "0.000"},
      {"nominal_w", have_result ? formatDouble(result.target_velocity.angular_z) : "0.000"},
      {"output_v", formatDouble(output.linear_x)},
      {"output_w", formatDouble(output.angular_z)}};
    for (const auto & entry : entries) {
      diagnostic_msgs::msg::KeyValue value;
      value.key = entry.first;
      value.value = entry.second;
      status.values.push_back(std::move(value));
    }
    array.status.push_back(std::move(status));
    if (cycle) {telemetry_pub_->publish(array);} else {diagnostics_pub_->publish(array);}
  }

  bool enable_target_estimation_{true};
  std::string odom_frame_;
  double target_prediction_sec_{0.20};
  std::unique_ptr<TargetMotionEstimator> estimator_;
  std::unique_ptr<OdomPoseBuffer> pose_buffer_;
  WalkHysteresisConfig walk_config_;
  WalkState walk_state_;
  PredictiveFollowResult last_predictive_;
  int64_t target_epoch_ns_{0}, last_clock_ns_{0}, last_odom_stamp_ns_{0};
  size_t outlier_count_{0};
  std::string last_update_{"WAIT_TARGET"};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr target_state_pub_;
  bool enable_cycle_telemetry_{true};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr telemetry_pub_;
  std::string base_frame_;
  std::string target_topic_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string nominal_cmd_topic_;
  std::string diagnostics_topic_;
  bool enable_motion_{true};
  double control_frequency_{20.0};
  double diagnostic_frequency_{2.0};
  double target_timeout_sec_{0.50};
  double odom_timeout_sec_{0.20};
  double transform_timeout_sec_{0.10};
  FollowConfig config_;
  int turn_direction_{0};
  bool angular_brake_latched_{false};

  std::mutex target_mutex_;
  TargetSnapshot latest_target_;

  std::mutex odom_mutex_;
  OdomSnapshot latest_odom_;

  std::mutex status_mutex_;
  Velocity2D last_output_;
  FollowResult last_result_;
  double last_target_age_{0.0};
  double last_odom_age_{0.0};
  bool have_result_{false};
  std::string state_{"WAIT_TARGET"};
  std::chrono::steady_clock::time_point last_control_time_{};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr nominal_cmd_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

}  // namespace go2_uwb_local_follow

// 启动 UWB 纯跟随控制节点并进入 ROS 事件循环。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<go2_uwb_local_follow::UwbFollowControllerNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("uwb_follow_controller_node"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
