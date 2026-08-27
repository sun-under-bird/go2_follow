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
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "go2_uwb_local_follow/follow_control_core.hpp"

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
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_follow");
    nominal_cmd_topic_ = declare_parameter<std::string>(
      "nominal_cmd_topic", "/go2_uwb_local_follow/nominal_cmd");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/go2_uwb_local_follow/follow_diagnostics");
    enable_motion_ = declare_parameter<bool>("enable_motion", true);
    control_frequency_ = declare_parameter<double>("control_frequency", 20.0);
    diagnostic_frequency_ = declare_parameter<double>("diagnostic_frequency", 2.0);
    target_timeout_sec_ = declare_parameter<double>("target_timeout_sec", 0.50);
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.10);

    config_.follow_distance = declare_parameter<double>("follow_distance", 1.0);
    config_.distance_deadband = declare_parameter<double>("distance_deadband", 0.08);
    config_.angle_deadband = declare_parameter<double>("angle_deadband", 0.20);
    config_.angle_reengage = declare_parameter<double>("angle_reengage", 0.45);
    config_.linear_kp = declare_parameter<double>("linear_kp", 0.6);
    config_.angular_kp = declare_parameter<double>("angular_kp", 1.0);
    config_.min_linear_speed = declare_parameter<double>("min_linear_speed", 0.12);
    config_.max_linear_speed = declare_parameter<double>("max_linear_speed", 0.80);
    config_.max_angular_speed = declare_parameter<double>("max_angular_speed", 2.00);
    config_.heading_slowdown_start = declare_parameter<double>(
      "heading_slowdown_start", 0.50);
    config_.heading_stop_angle = declare_parameter<double>("heading_stop_angle", 1.05);
    config_.blind_rotation_max_speed = declare_parameter<double>(
      "blind_rotation_max_speed", 2.00);
    config_.max_linear_accel = declare_parameter<double>("max_linear_accel", 0.80);
    config_.max_linear_decel = declare_parameter<double>("max_linear_decel", 0.80);
    config_.max_angular_accel = declare_parameter<double>("max_angular_accel", 2.00);
    validateParameters();

    target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      target_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      std::bind(&UwbFollowControllerNode::targetCallback, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    nominal_cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(nominal_cmd_topic_, 10);
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
      get_logger(), "UWB follow controller started: target=%s cmd=%s enable_motion=%s",
      target_topic_.c_str(), cmd_vel_topic_.c_str(), enable_motion_ ? "true" : "false");
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
    builtin_interfaces::msg::Time source_stamp;
    std::chrono::steady_clock::time_point receipt_time{};
    bool valid{false};
  };

  // 检查控制频率、超时和跟随控制参数。
  void validateParameters()
  {
    std::string reason;
    if (base_frame_.empty()) {
      throw std::invalid_argument("base_frame must not be empty");
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
      !std::isfinite(transform_timeout_sec_) || transform_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument("target and transform timeouts must be positive");
    }
  }

  // 将目标点按其自身时间戳转换到 base_footprint；回调只覆盖最新目标快照。
  void targetCallback(const geometry_msgs::msg::PointStamped::SharedPtr message)
  {
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
    std::lock_guard<std::mutex> lock(target_mutex_);
    latest_target_ = snapshot;
  }

  // 返回当前最新目标快照，不保留或插值更早的控制样本。
  TargetSnapshot targetSnapshot()
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    return latest_target_;
  }

  // 固定频率计算名义速度、执行变化率限制并发布隔离跟随速度。
  void controlTick()
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
    const double target_age =
      std::chrono::duration<double>(current_time - target.receipt_time).count();
    if (target_age > target_timeout_sec_) {
      publishImmediateStop("TARGET_LOST");
      return;
    }

    FollowResult result = computeFollowTarget(target.x, target.y, config_);
    turn_direction_ = updateTurnDirection(
      result.heading, config_.angle_deadband, config_.angle_reengage, turn_direction_);
    result.turn_direction = turn_direction_;
    if (turn_direction_ == 0) {
      // 进入停止门限后保持零角速度，直到误差越过更大的重启门限。
      result.target_velocity.angular_z = 0.0;
    } else {
      result.target_velocity.angular_z = std::copysign(
        std::abs(result.target_velocity.angular_z), static_cast<double>(turn_direction_));
    }
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
    have_result_ = true;
  }

  // 超时或无目标时绕过普通平滑，立即发布零速度并清除历史输出。
  void publishImmediateStop(const std::string & state)
  {
    publishVelocity(Velocity2D{});
    publishNominal(Velocity2D{});
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      last_output_ = Velocity2D{};
      turn_direction_ = 0;
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
    std::string state;
    Velocity2D output;
    FollowResult result;
    double target_age = 0.0;
    bool have_result = false;
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      state = state_;
      output = last_output_;
      result = last_result_;
      target_age = last_target_age_;
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
      {"target_age_sec", formatDouble(target_age)},
      {"distance", have_result ? formatDouble(result.distance) : "n/a"},
      {"heading", have_result ? formatDouble(result.heading) : "n/a"},
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
    diagnostics_pub_->publish(array);
  }

  std::string base_frame_;
  std::string target_topic_;
  std::string cmd_vel_topic_;
  std::string nominal_cmd_topic_;
  std::string diagnostics_topic_;
  bool enable_motion_{true};
  double control_frequency_{20.0};
  double diagnostic_frequency_{2.0};
  double target_timeout_sec_{0.50};
  double transform_timeout_sec_{0.10};
  FollowConfig config_;
  int turn_direction_{0};

  std::mutex target_mutex_;
  TargetSnapshot latest_target_;

  std::mutex status_mutex_;
  Velocity2D last_output_;
  FollowResult last_result_;
  double last_target_age_{0.0};
  bool have_result_{false};
  std::string state_{"WAIT_TARGET"};
  std::chrono::steady_clock::time_point last_control_time_{};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
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
