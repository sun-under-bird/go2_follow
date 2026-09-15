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

#include "go2_uwb_local_follow/follow_control_core.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace go2_uwb_local_follow
{
namespace
{

// 吸收十进制距离边界在二进制浮点表示中的微小舍入误差。
constexpr double kBoundaryTolerance = 1e-9;

// 将数值限制在给定闭区间内。
double clampValue(double value, double minimum, double maximum)
{
  return std::max(minimum, std::min(value, maximum));
}

// 按最大变化率让一个标量平滑逼近目标值。
double approachValue(double current, double target, double maximum_rate, double dt)
{
  const double maximum_step = std::max(0.0, maximum_rate) * std::max(0.0, dt);
  return current + clampValue(target - current, -maximum_step, maximum_step);
}

// 在需要时写入参数校验失败原因。
bool rejectWithReason(const std::string & message, std::string * reason)
{
  if (reason != nullptr) {
    *reason = message;
  }
  return false;
}

}  // namespace

// 校验跟随控制和速度变化率参数之间的约束关系。
bool validateFollowConfig(const FollowConfig & config, std::string * reason)
{
  const bool finite =
    std::isfinite(config.follow_distance) && std::isfinite(config.distance_deadband) &&
    std::isfinite(config.angle_deadband) && std::isfinite(config.angle_reengage) &&
    std::isfinite(config.turn_response_delay) &&
    std::isfinite(config.angular_braking_accel) &&
    std::isfinite(config.angular_brake_release_speed) &&
    std::isfinite(config.angular_reverse_speed_threshold) &&
    std::isfinite(config.linear_kp) &&
    std::isfinite(config.angular_kp) && std::isfinite(config.min_linear_speed) &&
    std::isfinite(config.max_linear_speed) &&
    std::isfinite(config.max_angular_speed) &&
    std::isfinite(config.heading_slowdown_start) &&
    std::isfinite(config.heading_stop_angle) &&
    std::isfinite(config.heading_alignment_hysteresis) &&
    std::isfinite(config.avoidance_heading_stop_angle) &&
    std::isfinite(config.avoidance_heading_max_linear_speed) &&
    std::isfinite(config.blind_rotation_max_speed) &&
    std::isfinite(config.max_linear_accel) && std::isfinite(config.max_linear_decel) &&
    std::isfinite(config.max_angular_accel);
  if (!finite) {
    return rejectWithReason("follow config contains non-finite values", reason);
  }
  if (config.follow_distance < 0.0 || config.distance_deadband < 0.0 ||
    config.angle_deadband < 0.0)
  {
    return rejectWithReason("follow distance and deadbands must be non-negative", reason);
  }
  if (config.angle_reengage < config.angle_deadband) {
    return rejectWithReason("angle reengage threshold must not be below deadband", reason);
  }
  if (config.turn_response_delay < 0.0 || config.angular_braking_accel <= 0.0 ||
    config.angular_brake_release_speed < 0.0 ||
    config.angular_reverse_speed_threshold < config.angular_brake_release_speed)
  {
    return rejectWithReason("angular braking parameters are invalid", reason);
  }
  if (config.linear_kp < 0.0 || config.angular_kp < 0.0 || config.min_linear_speed < 0.0 ||
    config.max_linear_speed < 0.0 || config.max_angular_speed < 0.0)
  {
    return rejectWithReason("control gains and speed limits must be non-negative", reason);
  }
  if (config.min_linear_speed > config.max_linear_speed) {
    return rejectWithReason("minimum linear speed must not exceed maximum linear speed", reason);
  }
  if (config.heading_slowdown_start < 0.0 ||
    config.heading_stop_angle <= config.heading_slowdown_start)
  {
    return rejectWithReason("heading angles must satisfy 0 <= slowdown < stop", reason);
  }
  if (config.blind_rotation_max_speed < 0.0 ||
    config.blind_rotation_max_speed > config.max_angular_speed)
  {
    return rejectWithReason("blind rotation speed must be within angular speed limit", reason);
  }
  if (config.heading_alignment_hysteresis < 0.0 ||
    config.heading_alignment_hysteresis >= config.heading_stop_angle)
  {
    return rejectWithReason("heading hysteresis must be within the stop angle", reason);
  }
  if (config.enable_avoidance_heading_relaxation &&
    (config.avoidance_heading_stop_angle <= config.heading_stop_angle ||
    config.avoidance_heading_stop_angle >= 1.5707963267948966 ||
    config.avoidance_heading_max_linear_speed < config.min_linear_speed ||
    config.avoidance_heading_max_linear_speed > config.max_linear_speed))
  {
    return rejectWithReason("avoidance heading angle or forward speed limit is invalid", reason);
  }
  if (config.max_linear_accel <= 0.0 || config.max_linear_decel <= 0.0 ||
    config.max_angular_accel <= 0.0)
  {
    return rejectWithReason("velocity rate limits must be positive", reason);
  }
  return true;
}

// 在持续安全绕障时临时放宽角度限制，距离停车与对准滞回始终生效。
FollowResult computeFollowTarget(
  double target_x,
  double target_y,
  const FollowConfig & config,
  bool safe_forward_avoidance,
  bool previous_heading_alignment)
{
  FollowResult result;
  result.distance = std::hypot(target_x, target_y);
  result.heading = std::atan2(target_y, target_x);

  const double distance_error = result.distance - config.follow_distance;
  result.within_follow_distance =
    distance_error <= config.distance_deadband + kBoundaryTolerance;
  if (!result.within_follow_distance) {
    // 离开距离死区后跨过 MCF 的低速无效区，避免持续前倾但无法迈步。
    const double proportional_speed =
      config.linear_kp * (distance_error - config.distance_deadband);
    result.target_velocity.linear_x = clampValue(
      std::max(config.min_linear_speed, proportional_speed),
      0.0, config.max_linear_speed);
  }

  const double absolute_heading = std::abs(result.heading);
  // 一旦已经进入对准，延后到达的旧绕障许可也不能提前解除停车滞回。
  const bool allow_relaxation = config.enable_avoidance_heading_relaxation &&
    safe_forward_avoidance && !previous_heading_alignment;
  const double stop_angle = allow_relaxation ?
    config.avoidance_heading_stop_angle : config.heading_stop_angle;
  result.blind_rotation = absolute_heading >= stop_angle ||
    (previous_heading_alignment &&
    absolute_heading > stop_angle - config.heading_alignment_hysteresis);
  result.avoidance_heading_relaxed = allow_relaxation && !result.blind_rotation &&
    !result.within_follow_distance &&
    absolute_heading >= config.heading_stop_angle - config.heading_alignment_hysteresis;
  const double signed_angle_error = std::copysign(
    std::max(0.0, absolute_heading - config.angle_deadband), result.heading);
  result.target_velocity.angular_z = clampValue(
    config.angular_kp * signed_angle_error,
    -config.max_angular_speed, config.max_angular_speed);

  // 停止角以内保持距离控制得到的线速度，实现边走边转；越过门槛才原地对准。
  result.heading_scale = result.blind_rotation ? 0.0 : 1.0;
  if (result.blind_rotation) {
    result.target_velocity.linear_x = 0.0;
    result.target_velocity.angular_z = clampValue(
      result.target_velocity.angular_z,
      -config.blind_rotation_max_speed, config.blind_rotation_max_speed);
  }
  if (result.avoidance_heading_relaxed) {
    result.target_velocity.linear_x = std::min(
      result.target_velocity.linear_x, config.avoidance_heading_max_linear_speed);
  }
  return result;
}

// 根据停止与重启角度门限更新带滞回的转向方向。
int updateTurnDirection(
  double heading,
  double stop_angle,
  double reengage_angle,
  int previous_direction)
{
  if (!std::isfinite(heading) || !std::isfinite(stop_angle) ||
    !std::isfinite(reengage_angle) || stop_angle < 0.0 || reengage_angle < stop_angle)
  {
    return 0;
  }

  constexpr double half_turn = 1.5707963267948966;
  if (previous_direction > 0) {
    // 目标接近正后方时 atan2 可能在 ±pi 间跳变，此时继续已选方向。
    return heading > stop_angle || heading < -half_turn ? 1 : 0;
  }
  if (previous_direction < 0) {
    return heading < -stop_angle || heading > half_turn ? -1 : 0;
  }
  if (heading >= reengage_angle) {
    return 1;
  }
  if (heading <= -reengage_angle) {
    return -1;
  }
  return 0;
}

// 根据实测角速度计算动态停止角，并在需要时优先撤销 UWB 名义转向。
DynamicAngularBrakeResult applyDynamicAngularBrake(
  double heading,
  double desired_angular_z,
  double actual_angular_z,
  const FollowConfig & config,
  int previous_direction,
  bool brake_latched)
{
  DynamicAngularBrakeResult result;
  result.dynamic_stop_angle = config.angle_deadband;
  if (!std::isfinite(heading) || !std::isfinite(desired_angular_z) ||
    !std::isfinite(actual_angular_z) || !validateFollowConfig(config))
  {
    result.braking = true;
    result.brake_latched = true;
    return result;
  }

  const double actual_speed = std::abs(actual_angular_z);
  result.brake_angle = actual_speed * config.turn_response_delay +
    actual_speed * actual_speed / (2.0 * config.angular_braking_accel);
  result.dynamic_stop_angle = config.angle_deadband + result.brake_angle;
  if (!std::isfinite(result.dynamic_stop_angle)) {
    // 异常大的实测值按需要立即刹车处理，禁止向后级输出不可控角速度。
    result.brake_angle = 0.0;
    result.dynamic_stop_angle = config.angle_deadband;
    result.braking = true;
    result.brake_latched = true;
    return result;
  }

  if (brake_latched && actual_speed > config.angular_brake_release_speed) {
    // 锁存后不随动态停止角缩小而重新加速，必须等真实角速度接近停止。
    result.braking = true;
    result.brake_latched = true;
    return result;
  }

  // 动态停止角可能大于固定重启角，取两者较大值避免刹车边界附近逐周期启停。
  const double dynamic_reengage_angle = std::max(
    config.angle_reengage, result.dynamic_stop_angle);
  result.turn_direction = updateTurnDirection(
    heading, result.dynamic_stop_angle, dynamic_reengage_angle, previous_direction);

  const bool reversing = result.turn_direction * actual_angular_z < 0.0;
  if (reversing && actual_speed > config.angular_reverse_speed_threshold) {
    // 机器人尚未停稳时保持零命令，实际角速度降到门槛后才允许反向纠偏。
    result.turn_direction = 0;
    result.braking = true;
    result.brake_latched = true;
    return result;
  }

  if (result.turn_direction == 0) {
    result.braking = std::abs(desired_angular_z) > 0.0 || actual_speed > 0.0;
    result.brake_latched = result.braking && actual_speed > config.angular_brake_release_speed &&
      (previous_direction != 0 || std::abs(desired_angular_z) > 0.0);
    return result;
  }
  result.angular_z = std::copysign(
    std::abs(desired_angular_z), static_cast<double>(result.turn_direction));
  return result;
}

// 按线加速、线减速和角加速度限制一个控制周期，并跳过实机无效线速度区间。
Velocity2D limitVelocityRate(
  const Velocity2D & previous,
  const Velocity2D & target,
  const FollowConfig & config,
  double dt)
{
  const double linear_rate = target.linear_x >= previous.linear_x ?
    config.max_linear_accel : config.max_linear_decel;
  Velocity2D output;
  output.linear_x = approachValue(previous.linear_x, target.linear_x, linear_rate, dt);
  output.angular_z = approachValue(
    previous.angular_z, target.angular_z, config.max_angular_accel, dt);
  output.linear_x = clampValue(output.linear_x, 0.0, config.max_linear_speed);
  output.angular_z = clampValue(
    output.angular_z, -config.max_angular_speed, config.max_angular_speed);
  if (target.linear_x <= 0.0) {
    if (output.linear_x < config.min_linear_speed) {
      output.linear_x = 0.0;
    }
  } else if (output.linear_x < config.min_linear_speed) {
    // 起步时直接给出可执行速度，避免连续 MOVE 小指令只造成重心调整。
    output.linear_x = config.min_linear_speed;
  }
  return output;
}

}  // namespace go2_uwb_local_follow
