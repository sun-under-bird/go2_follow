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

// 生成首尾一阶导数为零的平滑插值比例。
double smoothStep(double value)
{
  const double ratio = clampValue(value, 0.0, 1.0);
  return ratio * ratio * (3.0 - 2.0 * ratio);
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
    std::isfinite(config.linear_kp) &&
    std::isfinite(config.angular_kp) && std::isfinite(config.min_linear_speed) &&
    std::isfinite(config.max_linear_speed) &&
    std::isfinite(config.max_angular_speed) &&
    std::isfinite(config.heading_slowdown_start) &&
    std::isfinite(config.heading_stop_angle) &&
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
  if (config.max_linear_accel <= 0.0 || config.max_linear_decel <= 0.0 ||
    config.max_angular_accel <= 0.0)
  {
    return rejectWithReason("velocity rate limits must be positive", reason);
  }
  return true;
}

// 根据当前机器人坐标系目标点计算最小有效速度和转向降速后的名义速度。
FollowResult computeFollowTarget(
  double target_x,
  double target_y,
  const FollowConfig & config)
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
  const double signed_angle_error = std::copysign(
    std::max(0.0, absolute_heading - config.angle_deadband), result.heading);
  result.target_velocity.angular_z = clampValue(
    config.angular_kp * signed_angle_error,
    -config.max_angular_speed, config.max_angular_speed);

  if (absolute_heading > config.heading_slowdown_start) {
    const double ratio =
      (absolute_heading - config.heading_slowdown_start) /
      (config.heading_stop_angle - config.heading_slowdown_start);
    result.heading_scale = 1.0 - smoothStep(ratio);
    result.target_velocity.linear_x *= result.heading_scale;
  }

  result.blind_rotation = absolute_heading >= config.heading_stop_angle;
  if (result.blind_rotation) {
    result.target_velocity.linear_x = 0.0;
    result.target_velocity.angular_z = clampValue(
      result.target_velocity.angular_z,
      -config.blind_rotation_max_speed, config.blind_rotation_max_speed);
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

// 按线加速、线减速和角加速度限制一个控制周期内的速度变化。
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
  return output;
}

}  // namespace go2_uwb_local_follow
