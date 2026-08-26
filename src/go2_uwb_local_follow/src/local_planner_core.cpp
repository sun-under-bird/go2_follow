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

#include "go2_uwb_local_follow/local_planner_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <utility>

namespace go2_uwb_local_follow
{
namespace
{

// 在需要时写入参数校验失败原因。
bool rejectWithReason(const std::string & message, std::string * reason)
{
  if (reason != nullptr) {
    *reason = message;
  }
  return false;
}

// 将数值限制在给定闭区间内。
double clampValue(double value, double minimum, double maximum)
{
  return std::max(minimum, std::min(value, maximum));
}

// 按最大变化率让一个数值逼近目标。
double approachValue(double current, double target, double maximum_rate, double dt)
{
  const double maximum_step = std::max(0.0, maximum_rate) * std::max(0.0, dt);
  return current + clampValue(target - current, -maximum_step, maximum_step);
}

// 返回标量平方，保持代价公式可读。
double squareValue(double value)
{
  return value * value;
}

// 使用步长首尾平均速度积分一次二维单轮车运动。
void integrateVelocityStep(
  PlannerPose2D & pose,
  const PlannerVelocity2D & start_velocity,
  const PlannerVelocity2D & end_velocity,
  double dt)
{
  const double average_linear = 0.5 * (start_velocity.linear_x + end_velocity.linear_x);
  const double average_angular = 0.5 * (start_velocity.angular_z + end_velocity.angular_z);
  const double middle_yaw = pose.yaw + 0.5 * average_angular * dt;
  pose.x += average_linear * std::cos(middle_yaw) * dt;
  pose.y += average_linear * std::sin(middle_yaw) * dt;
  pose.yaw += average_angular * dt;
}

// 根据目标速度分别使用线加速、线减速和角加速度计算下一时刻速度。
PlannerVelocity2D approachVelocity(
  const PlannerVelocity2D & current,
  const PlannerVelocity2D & target,
  const MotionLimits & limits,
  double dt)
{
  const double linear_rate = target.linear_x >= current.linear_x ?
    limits.max_linear_accel : limits.max_linear_decel;
  return PlannerVelocity2D{
    approachValue(current.linear_x, target.linear_x, linear_rate, dt),
    approachValue(current.angular_z, target.angular_z, limits.max_angular_accel, dt)};
}

// 仅裁剪速度范围但不跨越执行死区，用于保存真实测量值或历史指令。
PlannerVelocity2D clampVelocityToLimits(
  const PlannerVelocity2D & velocity,
  const MotionLimits & limits)
{
  return PlannerVelocity2D{
    clampValue(velocity.linear_x, 0.0, limits.max_linear_speed),
    clampValue(velocity.angular_z, -limits.max_angular_speed, limits.max_angular_speed)};
}

// 向候选列表追加速度，并消除浮点采样产生的重复项。
void appendUniqueVelocity(
  std::vector<PlannerVelocity2D> & candidates,
  const PlannerVelocity2D & velocity)
{
  constexpr double tolerance = 1e-9;
  const auto duplicate = std::find_if(
    candidates.begin(), candidates.end(),
    [&velocity](const PlannerVelocity2D & candidate) {
      return std::abs(candidate.linear_x - velocity.linear_x) <= tolerance &&
      std::abs(candidate.angular_z - velocity.angular_z) <= tolerance;
    });
  if (duplicate == candidates.end()) {
    candidates.push_back(velocity);
  }
}

// 把障碍点转换到给定预测姿态的机器人局部坐标系。
ObstaclePoint2D transformObstacleToPose(
  const PlannerPose2D & pose,
  const ObstaclePoint2D & obstacle)
{
  const double dx = obstacle.x - pose.x;
  const double dy = obstacle.y - pose.y;
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  return ObstaclePoint2D{
    cosine * dx + sine * dy,
    -sine * dx + cosine * dy};
}

}  // namespace

// 校验轨迹预测的时域和积分步长。
bool validateTrajectoryConfig(const TrajectoryConfig & config, std::string * reason)
{
  if (!std::isfinite(config.prediction_time) || !std::isfinite(config.simulation_dt)) {
    return rejectWithReason("trajectory config contains non-finite values", reason);
  }
  if (config.prediction_time <= 0.0 || config.simulation_dt <= 0.0) {
    return rejectWithReason("prediction time and simulation dt must be positive", reason);
  }
  if (config.simulation_dt > config.prediction_time) {
    return rejectWithReason("simulation dt must not exceed prediction time", reason);
  }
  return true;
}

// 校验矩形机器人尺寸和安全膨胀参数。
bool validateFootprintConfig(const FootprintConfig & config, std::string * reason)
{
  if (!std::isfinite(config.robot_length) || !std::isfinite(config.robot_width) ||
    !std::isfinite(config.safety_margin))
  {
    return rejectWithReason("footprint config contains non-finite values", reason);
  }
  if (config.robot_length <= 0.0 || config.robot_width <= 0.0 ||
    config.safety_margin < 0.0)
  {
    return rejectWithReason("robot dimensions must be positive and margin non-negative", reason);
  }
  return true;
}

// 校验速度死区、运动学限制和加减速度参数。
bool validateMotionLimits(const MotionLimits & limits, std::string * reason)
{
  const bool finite =
    std::isfinite(limits.min_linear_speed) && std::isfinite(limits.max_linear_speed) &&
    std::isfinite(limits.min_angular_speed) && std::isfinite(limits.max_angular_speed) &&
    std::isfinite(limits.max_linear_accel) && std::isfinite(limits.max_linear_decel) &&
    std::isfinite(limits.max_angular_accel);
  if (!finite) {
    return rejectWithReason("motion limits contain non-finite values", reason);
  }
  if (limits.min_linear_speed < 0.0 ||
    limits.min_linear_speed > limits.max_linear_speed || limits.max_linear_speed < 0.0)
  {
    return rejectWithReason("linear speed limits are invalid", reason);
  }
  if (limits.min_angular_speed < 0.0 ||
    limits.min_angular_speed > limits.max_angular_speed || limits.max_angular_speed < 0.0)
  {
    return rejectWithReason("angular speed limits are invalid", reason);
  }
  if (limits.max_linear_accel <= 0.0 || limits.max_linear_decel <= 0.0 ||
    limits.max_angular_accel <= 0.0)
  {
    return rejectWithReason("acceleration limits must be positive", reason);
  }
  return true;
}

// 校验角速度反馈参数，避免无穷大、负阻尼或负换向阈值进入控制链路。
bool validateAngularStabilizationConfig(
  const AngularStabilizationConfig & config,
  std::string * reason)
{
  if (!std::isfinite(config.velocity_damping_gain) ||
    !std::isfinite(config.command_deadband) ||
    !std::isfinite(config.reverse_speed_threshold))
  {
    return rejectWithReason("angular stabilization config contains non-finite values", reason);
  }
  if (config.velocity_damping_gain < 0.0 || config.command_deadband < 0.0 ||
    config.reverse_speed_threshold < 0.0)
  {
    return rejectWithReason("angular stabilization parameters must be non-negative", reason);
  }
  return true;
}

// 校验速度采样数量、障碍影响距离和各项代价权重。
bool validateVelocitySamplingConfig(
  const VelocitySamplingConfig & config,
  std::string * reason)
{
  if (config.linear_samples < 2 || config.angular_samples < 3) {
    return rejectWithReason("velocity sample counts are too small", reason);
  }
  const bool finite = std::isfinite(config.min_avoidance_angular_speed) &&
    std::isfinite(config.obstacle_influence_distance) &&
    std::isfinite(config.weight_follow_linear) &&
    std::isfinite(config.weight_follow_angular) &&
    std::isfinite(config.weight_smooth_linear) &&
    std::isfinite(config.weight_smooth_angular) &&
    std::isfinite(config.weight_obstacle) && std::isfinite(config.weight_progress);
  if (!finite) {
    return rejectWithReason("velocity sampling config contains non-finite values", reason);
  }
  if (config.min_avoidance_angular_speed < 0.0 ||
    config.obstacle_influence_distance <= 0.0 || config.weight_follow_linear < 0.0 ||
    config.weight_follow_angular < 0.0 || config.weight_smooth_linear < 0.0 ||
    config.weight_smooth_angular < 0.0 || config.weight_obstacle < 0.0 ||
    config.weight_progress < 0.0)
  {
    return rejectWithReason("sampling distances and weights must be non-negative", reason);
  }
  if (config.linear_speed_priority_scales.empty()) {
    return rejectWithReason("linear speed priority scales must not be empty", reason);
  }
  constexpr double tolerance = 1e-9;
  double previous_scale = std::numeric_limits<double>::infinity();
  for (const double scale : config.linear_speed_priority_scales) {
    if (!std::isfinite(scale) || scale < 0.0 || scale > 1.0 ||
      scale >= previous_scale - tolerance)
    {
      return rejectWithReason(
        "linear speed priority scales must strictly descend within [0, 1]", reason);
    }
    previous_scale = scale;
  }
  if (std::abs(config.linear_speed_priority_scales.front() - 1.0) > tolerance ||
    std::abs(config.linear_speed_priority_scales.back()) > tolerance)
  {
    return rejectWithReason("linear speed priority scales must start at 1 and end at 0", reason);
  }
  return true;
}

// 从当前原点使用单轮车模型预测局部轨迹，返回值包含初始姿态。
std::vector<PlannerPose2D> predictTrajectory(
  const PlannerVelocity2D & velocity,
  const TrajectoryConfig & config)
{
  const std::size_t step_count = static_cast<std::size_t>(
    std::ceil(config.prediction_time / config.simulation_dt));
  std::vector<PlannerPose2D> poses;
  poses.reserve(step_count + 1U);
  poses.push_back(PlannerPose2D{});

  PlannerPose2D pose;
  double elapsed = 0.0;
  for (std::size_t step = 0U; step < step_count; ++step) {
    const double dt = std::min(config.simulation_dt, config.prediction_time - elapsed);
    if (dt <= 0.0) {
      break;
    }
    // 使用步长起点的朝向积分，确保 w>0 时轨迹向机器人左侧弯曲。
    pose.x += velocity.linear_x * std::cos(pose.yaw) * dt;
    pose.y += velocity.linear_x * std::sin(pose.yaw) * dt;
    pose.yaw += velocity.angular_z * dt;
    poses.push_back(pose);
    elapsed += dt;
  }
  return poses;
}

// 从实测初始速度向候选速度加减速展开轨迹，并在时域末尾追加完整制动尾段。
std::vector<PlannerPose2D> predictAcceleratingTrajectory(
  const PlannerVelocity2D & initial_velocity,
  const PlannerVelocity2D & target_velocity,
  const TrajectoryConfig & config,
  const MotionLimits & limits,
  bool append_braking_tail)
{
  std::string reason;
  if (!validateTrajectoryConfig(config, &reason) || !validateMotionLimits(limits, &reason)) {
    return {};
  }

  PlannerVelocity2D current{
    clampValue(initial_velocity.linear_x, 0.0, limits.max_linear_speed),
    clampValue(
      initial_velocity.angular_z, -limits.max_angular_speed, limits.max_angular_speed)};
  const PlannerVelocity2D target = makeEffectiveVelocity(target_velocity, limits);
  PlannerPose2D pose;
  std::vector<PlannerPose2D> poses{pose};
  const std::size_t nominal_steps = static_cast<std::size_t>(
    std::ceil(config.prediction_time / config.simulation_dt));
  const double braking_time =
    limits.max_linear_decel > 0.0 ? limits.max_linear_speed / limits.max_linear_decel : 0.0;
  const double angular_braking_time = limits.max_angular_accel > 0.0 ?
    limits.max_angular_speed / limits.max_angular_accel : 0.0;
  const std::size_t braking_steps = append_braking_tail ?
    static_cast<std::size_t>(
    std::ceil(std::max(braking_time, angular_braking_time) / config.simulation_dt)) + 1U :
    0U;
  poses.reserve(nominal_steps + braking_steps + 1U);

  double elapsed = 0.0;
  while (elapsed < config.prediction_time) {
    const double dt = std::min(config.simulation_dt, config.prediction_time - elapsed);
    const PlannerVelocity2D next = approachVelocity(current, target, limits, dt);
    integrateVelocityStep(pose, current, next, dt);
    poses.push_back(pose);
    current = next;
    elapsed += dt;
  }

  if (append_braking_tail) {
    constexpr double stopped_tolerance = 1e-6;
    std::size_t step = 0U;
    while ((std::abs(current.linear_x) > stopped_tolerance ||
      std::abs(current.angular_z) > stopped_tolerance) && step < braking_steps)
    {
      const PlannerVelocity2D next = approachVelocity(
        current, PlannerVelocity2D{}, limits, config.simulation_dt);
      integrateVelocityStep(pose, current, next, config.simulation_dt);
      poses.push_back(pose);
      current = next;
      ++step;
    }
  }
  return poses;
}

// 判断障碍点是否落入指定姿态下旋转后的矩形足迹。
bool pointInsideFootprint(
  const PlannerPose2D & pose,
  const ObstaclePoint2D & obstacle,
  const FootprintConfig & footprint)
{
  const ObstaclePoint2D local = transformObstacleToPose(pose, obstacle);
  const double half_length = footprint.robot_length * 0.5 + footprint.safety_margin;
  const double half_width = footprint.robot_width * 0.5 + footprint.safety_margin;
  return std::abs(local.x) <= half_length && std::abs(local.y) <= half_width;
}

// 计算障碍点到指定姿态矩形足迹边界的二维最短距离。
double pointToFootprintClearance(
  const PlannerPose2D & pose,
  const ObstaclePoint2D & obstacle,
  const FootprintConfig & footprint)
{
  const ObstaclePoint2D local = transformObstacleToPose(pose, obstacle);
  const double half_length = footprint.robot_length * 0.5 + footprint.safety_margin;
  const double half_width = footprint.robot_width * 0.5 + footprint.safety_margin;
  const double outside_x = std::max(0.0, std::abs(local.x) - half_length);
  const double outside_y = std::max(0.0, std::abs(local.y) - half_width);
  return std::hypot(outside_x, outside_y);
}

// 沿整条预测轨迹检查旋转矩形碰撞并统计最小净空。
CollisionResult checkTrajectoryCollision(
  const std::vector<PlannerPose2D> & poses,
  const std::vector<ObstaclePoint2D> & obstacles,
  const FootprintConfig & footprint)
{
  CollisionResult result;
  for (std::size_t pose_index = 0U; pose_index < poses.size(); ++pose_index) {
    for (const auto & obstacle : obstacles) {
      const double clearance = pointToFootprintClearance(poses[pose_index], obstacle, footprint);
      result.min_clearance = std::min(result.min_clearance, clearance);
      if (clearance <= 0.0) {
        result.collision = true;
        result.collision_pose_index = pose_index;
        return result;
      }
    }
  }
  return result;
}

// 检查障碍是否进入机器人正前方的紧急停车矩形区域。
bool hasEmergencyFrontObstacle(
  const std::vector<ObstaclePoint2D> & obstacles,
  const FootprintConfig & footprint,
  double emergency_front_distance,
  double emergency_half_width)
{
  const double robot_front = footprint.robot_length * 0.5 + footprint.safety_margin;
  const double front_limit = robot_front + std::max(0.0, emergency_front_distance);
  const double half_width = std::max(0.0, emergency_half_width);
  return std::any_of(
    obstacles.begin(), obstacles.end(),
    [robot_front, front_limit, half_width](const ObstaclePoint2D & obstacle) {
      return obstacle.x >= robot_front && obstacle.x <= front_limit &&
      std::abs(obstacle.y) <= half_width;
    });
}

// 将候选速度限制到运动范围，并跨过 Go2 无法执行的最小非零速度区间。
PlannerVelocity2D makeEffectiveVelocity(
  const PlannerVelocity2D & velocity,
  const MotionLimits & limits)
{
  PlannerVelocity2D effective;
  effective.linear_x = clampValue(velocity.linear_x, 0.0, limits.max_linear_speed);
  if (effective.linear_x > 0.0 && effective.linear_x < limits.min_linear_speed) {
    effective.linear_x = limits.min_linear_speed;
  }
  effective.angular_z = clampValue(
    velocity.angular_z, -limits.max_angular_speed, limits.max_angular_speed);
  if (std::abs(effective.angular_z) > 0.0 &&
    std::abs(effective.angular_z) < limits.min_angular_speed)
  {
    effective.angular_z = std::copysign(limits.min_angular_speed, effective.angular_z);
  }
  return effective;
}

// 根据真实角速度提前制动，并禁止机器人尚未停稳时立即发布相反方向转向。
PlannerVelocity2D stabilizeNominalAngularVelocity(
  const PlannerVelocity2D & nominal_velocity,
  const PlannerVelocity2D & measured_velocity,
  const AngularStabilizationConfig & config)
{
  PlannerVelocity2D stabilized = nominal_velocity;
  const double desired = nominal_velocity.angular_z;
  const double measured = measured_velocity.angular_z;
  if (!std::isfinite(desired) || !std::isfinite(measured) ||
    !validateAngularStabilizationConfig(config))
  {
    stabilized.angular_z = 0.0;
    return stabilized;
  }

  if (std::abs(desired) <= config.command_deadband) {
    stabilized.angular_z = 0.0;
    return stabilized;
  }

  const bool reversing = desired * measured < 0.0;
  if (reversing && std::abs(measured) > config.reverse_speed_threshold) {
    // 先发零角速度让底盘刹住，防止 +w 直接跳到 -w 形成左右极限环。
    stabilized.angular_z = 0.0;
    return stabilized;
  }

  if (desired * measured > 0.0) {
    const double damped_magnitude = std::max(
      0.0, std::abs(desired) - config.velocity_damping_gain * std::abs(measured));
    stabilized.angular_z = damped_magnitude > config.command_deadband ?
      std::copysign(damped_magnitude, desired) : 0.0;
  }
  return stabilized;
}

// 生成覆盖全局范围、名义速度邻域和关键停车/原地转向速度的候选集合。
std::vector<PlannerVelocity2D> sampleCandidateVelocities(
  const PlannerVelocity2D & nominal_velocity,
  const PlannerVelocity2D & previous_command,
  const MotionLimits & limits,
  const VelocitySamplingConfig & config,
  bool force_linear_stop)
{
  std::vector<double> linear_values;
  std::vector<double> angular_values;
  for (int index = 0; index < config.linear_samples; ++index) {
    const double ratio = static_cast<double>(index) /
      static_cast<double>(config.linear_samples - 1);
    const double raw = force_linear_stop ? 0.0 : ratio * limits.max_linear_speed;
    const double effective = makeEffectiveVelocity({raw, 0.0}, limits).linear_x;
    if (std::find(linear_values.begin(), linear_values.end(), effective) == linear_values.end()) {
      linear_values.push_back(effective);
    }
    if (force_linear_stop) {
      break;
    }
  }
  for (int index = 0; index < config.angular_samples; ++index) {
    const double ratio = static_cast<double>(index) /
      static_cast<double>(config.angular_samples - 1);
    const double raw = -limits.max_angular_speed + 2.0 * limits.max_angular_speed * ratio;
    const double effective = makeEffectiveVelocity({0.0, raw}, limits).angular_z;
    if (std::find(angular_values.begin(), angular_values.end(), effective) ==
      angular_values.end())
    {
      angular_values.push_back(effective);
    }
  }

  std::vector<PlannerVelocity2D> candidates;
  for (const double linear : linear_values) {
    for (const double angular : angular_values) {
      appendUniqueVelocity(candidates, {linear, angular});
    }
  }

  const PlannerVelocity2D effective_nominal = makeEffectiveVelocity(nominal_velocity, limits);
  const PlannerVelocity2D effective_previous = makeEffectiveVelocity(previous_command, limits);
  const auto applyLinearPolicy = [force_linear_stop](PlannerVelocity2D velocity) {
      if (force_linear_stop) {
        velocity.linear_x = 0.0;
      }
      return velocity;
    };
  appendUniqueVelocity(candidates, applyLinearPolicy(effective_nominal));
  appendUniqueVelocity(candidates, applyLinearPolicy(effective_previous));
  appendUniqueVelocity(candidates, PlannerVelocity2D{});
  appendUniqueVelocity(candidates, {0.0, limits.min_angular_speed});
  appendUniqueVelocity(candidates, {0.0, -limits.min_angular_speed});

  constexpr std::array<double, 5> linear_offsets{{0.0, -0.05, 0.05, -0.10, 0.10}};
  constexpr std::array<double, 7> angular_offsets{{0.0, -0.10, 0.10, -0.20, 0.20, -0.40,
    0.40}};
  for (const double linear_offset : linear_offsets) {
    for (const double angular_offset : angular_offsets) {
      PlannerVelocity2D nearby{
        effective_nominal.linear_x + linear_offset,
        effective_nominal.angular_z + angular_offset};
      nearby = makeEffectiveVelocity(nearby, limits);
      appendUniqueVelocity(candidates, applyLinearPolicy(nearby));
    }
  }
  return candidates;
}

// 计算安全候选的跟随、平滑、净空和前进代价。
PlannerCost scoreVelocityCandidate(
  const PlannerVelocity2D & candidate,
  const PlannerVelocity2D & effective_nominal,
  const PlannerVelocity2D & previous_command,
  double min_clearance,
  const MotionLimits & limits,
  const VelocitySamplingConfig & config)
{
  PlannerCost cost;
  cost.follow = config.weight_follow_linear *
    squareValue(candidate.linear_x - effective_nominal.linear_x) +
    config.weight_follow_angular *
    squareValue(candidate.angular_z - effective_nominal.angular_z);
  double angular_smooth_error = candidate.angular_z - previous_command.angular_z;
  constexpr double angular_tolerance = 1e-9;
  if (std::abs(effective_nominal.angular_z) <= angular_tolerance &&
    std::abs(candidate.angular_z) <= angular_tolerance)
  {
    // UWB 已要求直行时，不让历史转向平滑代价把规划器永久锁在绕障角速度上。
    angular_smooth_error = 0.0;
  }
  cost.smooth = config.weight_smooth_linear *
    squareValue(candidate.linear_x - previous_command.linear_x) +
    config.weight_smooth_angular *
    squareValue(angular_smooth_error);
  if (std::isfinite(min_clearance) && min_clearance < config.obstacle_influence_distance) {
    const double ratio =
      (config.obstacle_influence_distance - std::max(0.0, min_clearance)) /
      config.obstacle_influence_distance;
    cost.obstacle = config.weight_obstacle * squareValue(ratio);
  }
  // 只惩罚低于名义跟随速度，禁止“前进奖励”诱导候选超过 UWB 控制器给出的速度。
  const double bounded_candidate = clampValue(
    candidate.linear_x, 0.0, limits.max_linear_speed);
  const double bounded_nominal = clampValue(
    effective_nominal.linear_x, 0.0, limits.max_linear_speed);
  cost.progress = config.weight_progress *
    std::max(0.0, bounded_nominal - bounded_candidate);
  cost.total = cost.follow + cost.smooth + cost.obstacle + cost.progress;
  return cost;
}

namespace
{

// 向角速度列表追加唯一值，避免名义邻域和全局采样产生重复候选。
void appendUniqueAngular(std::vector<double> & values, double angular_z)
{
  constexpr double tolerance = 1e-9;
  const auto duplicate = std::find_if(
    values.begin(), values.end(),
    [angular_z](double existing) {
      return std::abs(existing - angular_z) <= tolerance;
    });
  if (duplicate == values.end()) {
    values.push_back(angular_z);
  }
}

// 对主动避障候选应用独立最小角速度，UWB 原始名义角速度由调用方单独保留。
double makeAvoidanceAngularVelocity(
  double angular_z,
  const MotionLimits & limits,
  const VelocitySamplingConfig & config)
{
  double effective = clampValue(
    angular_z, -limits.max_angular_speed, limits.max_angular_speed);
  const double threshold = std::max(
    limits.min_angular_speed, config.min_avoidance_angular_speed);
  if (std::abs(effective) > 0.0 && std::abs(effective) < threshold) {
    effective = std::copysign(threshold, effective);
  }
  return effective;
}

// 生成以 UWB 名义转向为中心的避障角速度集合，并保留零角速度安全候选。
std::vector<double> sampleAvoidanceAngularVelocities(
  const PlannerVelocity2D & effective_nominal,
  const PlannerVelocity2D & previous_command,
  const MotionLimits & limits,
  const VelocitySamplingConfig & config)
{
  std::vector<double> angular_values;
  // 进入避障后名义转向也必须跨过避障门槛；无障碍路径已在调用方提前直接返回。
  appendUniqueAngular(
    angular_values,
    makeAvoidanceAngularVelocity(effective_nominal.angular_z, limits, config));
  appendUniqueAngular(angular_values, 0.0);
  appendUniqueAngular(
    angular_values,
    makeAvoidanceAngularVelocity(previous_command.angular_z, limits, config));

  for (int index = 0; index < config.angular_samples; ++index) {
    const double ratio = static_cast<double>(index) /
      static_cast<double>(config.angular_samples - 1);
    const double raw = -limits.max_angular_speed +
      2.0 * limits.max_angular_speed * ratio;
    appendUniqueAngular(
      angular_values, makeAvoidanceAngularVelocity(raw, limits, config));
  }

  constexpr std::array<double, 9> angular_offsets{{
    -0.60, -0.40, -0.20, -0.10, 0.0, 0.10, 0.20, 0.40, 0.60}};
  for (const double offset : angular_offsets) {
    appendUniqueAngular(
      angular_values,
      makeAvoidanceAngularVelocity(effective_nominal.angular_z + offset, limits, config));
  }
  appendUniqueAngular(
    angular_values,
    makeAvoidanceAngularVelocity(config.min_avoidance_angular_speed, limits, config));
  appendUniqueAngular(
    angular_values,
    makeAvoidanceAngularVelocity(-config.min_avoidance_angular_speed, limits, config));
  return angular_values;
}

struct LinearSpeedLevel
{
  double velocity{0.0};
  double scale{0.0};
};

// 根据配置生成严格按优先级排列的线速度层，并合并最小线速度造成的重复层。
std::vector<LinearSpeedLevel> makeLinearSpeedLevels(
  double nominal_linear,
  const MotionLimits & limits,
  const VelocitySamplingConfig & config,
  bool force_linear_stop)
{
  if (force_linear_stop || nominal_linear <= 0.0) {
    return {{0.0, 0.0}};
  }

  std::vector<LinearSpeedLevel> levels;
  constexpr double tolerance = 1e-9;
  for (const double scale : config.linear_speed_priority_scales) {
    const double velocity = scale <= 0.0 ? 0.0 :
      makeEffectiveVelocity({nominal_linear * scale, 0.0}, limits).linear_x;
    const bool duplicate = std::any_of(
      levels.begin(), levels.end(),
      [velocity](const LinearSpeedLevel & level) {
        return std::abs(level.velocity - velocity) <= tolerance;
      });
    if (!duplicate) {
      levels.push_back({velocity, scale});
    }
  }
  return levels;
}

}  // namespace

// 优先保持 UWB 线速度，仅在当前速度层没有安全角速度时按比例降速。
LocalPlanResult planLocalVelocity(
  const PlannerVelocity2D & measured_velocity,
  const PlannerVelocity2D & previous_command,
  const PlannerVelocity2D & nominal_velocity,
  const std::vector<ObstaclePoint2D> & obstacles,
  const TrajectoryConfig & trajectory_config,
  const FootprintConfig & footprint_config,
  const MotionLimits & limits,
  const VelocitySamplingConfig & sampling_config,
  bool force_linear_stop)
{
  LocalPlanResult result;
  result.effective_nominal = makeEffectiveVelocity(nominal_velocity, limits);
  const PlannerVelocity2D clamped_previous = clampVelocityToLimits(previous_command, limits);

  // 先单独验证 UWB 名义轨迹；净空充足时直接交给最终变化率限制器平滑执行。
  auto nominal_trajectory = predictAcceleratingTrajectory(
    measured_velocity, result.effective_nominal, trajectory_config, limits, true);
  const CollisionResult nominal_collision = checkTrajectoryCollision(
    nominal_trajectory, obstacles, footprint_config);
  const bool nominal_has_clearance = !std::isfinite(nominal_collision.min_clearance) ||
    nominal_collision.min_clearance >= sampling_config.obstacle_influence_distance;
  result.avoidance_active = force_linear_stop || nominal_collision.collision ||
    !nominal_has_clearance;
  if (!result.avoidance_active) {
    result.valid = true;
    result.selected_velocity = result.effective_nominal;
    result.selected_trajectory = std::move(nominal_trajectory);
    result.cost = scoreVelocityCandidate(
      result.selected_velocity, result.effective_nominal, clamped_previous,
      nominal_collision.min_clearance, limits, sampling_config);
    result.min_clearance = nominal_collision.min_clearance;
    result.evaluated_count = 1U;
    result.selected_speed_scale = result.effective_nominal.linear_x > 0.0 ? 1.0 : 0.0;
    return result;
  }

  const auto angular_values = sampleAvoidanceAngularVelocities(
    result.effective_nominal, clamped_previous, limits, sampling_config);
  const auto speed_levels = makeLinearSpeedLevels(
    result.effective_nominal.linear_x, limits, sampling_config, force_linear_stop);

  // 每个速度层内部只比较角速度；本层存在安全轨迹后立即返回，禁止代价函数偷选更低速度。
  for (const auto & level : speed_levels) {
    bool level_valid = false;
    PlannerVelocity2D level_velocity;
    std::vector<PlannerPose2D> level_trajectory;
    PlannerCost level_cost;
    double level_clearance = std::numeric_limits<double>::infinity();

    for (const double angular_z : angular_values) {
      const PlannerVelocity2D candidate{level.velocity, angular_z};
      auto trajectory = predictAcceleratingTrajectory(
        measured_velocity, candidate, trajectory_config, limits, true);
      const CollisionResult collision = checkTrajectoryCollision(
        trajectory, obstacles, footprint_config);
      ++result.evaluated_count;
      if (collision.collision) {
        ++result.collision_count;
        continue;
      }

      const PlannerCost cost = scoreVelocityCandidate(
        candidate, result.effective_nominal, clamped_previous,
        collision.min_clearance, limits, sampling_config);
      if (!level_valid || cost.total < level_cost.total) {
        level_valid = true;
        level_velocity = candidate;
        level_trajectory = std::move(trajectory);
        level_cost = cost;
        level_clearance = collision.min_clearance;
      }
    }

    if (level_valid) {
      result.valid = true;
      result.selected_velocity = level_velocity;
      result.selected_trajectory = std::move(level_trajectory);
      result.cost = level_cost;
      result.min_clearance = level_clearance;
      result.selected_speed_scale = level.scale;
      return result;
    }
  }
  return result;
}

// 按控制周期限制最终指令变化，并对非零指令跨过底盘执行死区。
PlannerVelocity2D limitCommandVelocity(
  const PlannerVelocity2D & previous_command,
  const PlannerVelocity2D & target_velocity,
  const MotionLimits & limits,
  double dt)
{
  const PlannerVelocity2D previous = clampVelocityToLimits(previous_command, limits);
  const PlannerVelocity2D target = makeEffectiveVelocity(target_velocity, limits);
  PlannerVelocity2D output = approachVelocity(previous, target, limits, dt);

  if (target.linear_x <= 0.0) {
    if (output.linear_x < limits.min_linear_speed) {
      output.linear_x = 0.0;
    }
  } else if (output.linear_x < limits.min_linear_speed) {
    output.linear_x = limits.min_linear_speed;
  }

  if (std::abs(target.angular_z) <= 0.0) {
    if (std::abs(output.angular_z) < limits.min_angular_speed) {
      output.angular_z = 0.0;
    }
  } else if (std::abs(output.angular_z) < limits.min_angular_speed) {
    const bool reversing = previous.angular_z * target.angular_z < 0.0;
    output.angular_z = reversing ? 0.0 :
      std::copysign(limits.min_angular_speed, target.angular_z);
  }
  return makeEffectiveVelocity(output, limits);
}

}  // namespace go2_uwb_local_follow
