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

#include "go2_uwb_local_follow/mppi_planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "go2_uwb_local_follow/obstacle_index.hpp"

namespace go2_uwb_local_follow
{
namespace
{

struct SequenceEvaluation
{
  bool valid{false};
  bool full_stop{false};
  std::vector<PlannerPose2D> trajectory;
  PlannerCost cost;
  CollisionResult collision;
  double required_clearance{0.0};
  double clearance_ttc{std::numeric_limits<double>::infinity()};
};

// 在需要时写入 MPPI 参数校验失败原因。
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

// 返回标量平方，保持序列代价公式可读。
double squareValue(double value)
{
  return value * value;
}

// 按最大变化率让一个数值逼近目标。
double approachValue(double current, double target, double maximum_rate, double dt)
{
  const double maximum_step = std::max(0.0, maximum_rate) * std::max(0.0, dt);
  return current + clampValue(target - current, -maximum_step, maximum_step);
}

// 按真实加减速度限制生成下一时刻速度，禁止一个积分步内直接跨过零点反向。
PlannerVelocity2D approachVelocity(
  const PlannerVelocity2D & current,
  const PlannerVelocity2D & target,
  const MotionLimits & limits,
  double dt)
{
  const bool changing_linear_direction = current.linear_x * target.linear_x < 0.0;
  const double linear_target = changing_linear_direction ? 0.0 : target.linear_x;
  const double linear_rate = std::abs(linear_target) >= std::abs(current.linear_x) ?
    limits.max_linear_accel : limits.max_linear_decel;
  return PlannerVelocity2D{
    approachValue(current.linear_x, linear_target, linear_rate, dt),
    approachValue(current.angular_z, target.angular_z, limits.max_angular_accel, dt)};
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

// 把 MPPI 控制限制到普通前进范围，并在展开前显式建模 Go2 线速度死区。
PlannerVelocity2D applyControlPolicy(
  PlannerVelocity2D control,
  double maximum_forward_speed,
  const MotionLimits & limits,
  const VelocitySamplingConfig & cost_config,
  bool force_linear_stop)
{
  control.linear_x = force_linear_stop ? 0.0 :
    clampValue(control.linear_x, 0.0, std::min(maximum_forward_speed, limits.max_linear_speed));
  if (control.linear_x > 0.0 && control.linear_x < limits.min_linear_speed) {
    // 以死区中点区分停车与起步，非零起步必须和最终限幅一样直接跨到可执行速度。
    control.linear_x = control.linear_x < 0.5 * limits.min_linear_speed ?
      0.0 : limits.min_linear_speed;
  }
  const double angular_limit = std::min(
    limits.max_angular_speed, cost_config.max_avoidance_angular_speed);
  control.angular_z = clampValue(control.angular_z, -angular_limit, angular_limit);
  return control;
}

// 将一段时变目标速度从实测状态展开，并在末尾追加完整制动轨迹。
std::vector<PlannerPose2D> rolloutControlSequence(
  const PlannerVelocity2D & measured_velocity,
  const std::vector<PlannerVelocity2D> & controls,
  const TrajectoryConfig & trajectory_config,
  const MotionLimits & limits)
{
  PlannerVelocity2D current{
    clampValue(measured_velocity.linear_x, -limits.max_reverse_speed, limits.max_linear_speed),
    clampValue(measured_velocity.angular_z, -limits.max_angular_speed, limits.max_angular_speed)};
  PlannerPose2D pose;
  std::vector<PlannerPose2D> poses{pose};
  poses.reserve(controls.size() + 32U);

  double elapsed = 0.0;
  for (const auto & target : controls) {
    const double dt = std::min(
      trajectory_config.simulation_dt, trajectory_config.prediction_time - elapsed);
    if (dt <= 0.0) {
      break;
    }
    const PlannerVelocity2D next = approachVelocity(current, target, limits, dt);
    integrateVelocityStep(pose, current, next, dt);
    poses.push_back(pose);
    current = next;
    elapsed += dt;
  }

  const double maximum_braking_time = std::max(
    std::abs(current.linear_x) / limits.max_linear_decel,
    std::abs(current.angular_z) / limits.max_angular_accel);
  const std::size_t maximum_braking_steps = static_cast<std::size_t>(
    std::ceil(maximum_braking_time / trajectory_config.simulation_dt)) + 1U;
  constexpr double stopped_tolerance = 1e-6;
  for (std::size_t step = 0U; step < maximum_braking_steps; ++step) {
    if (std::abs(current.linear_x) <= stopped_tolerance &&
      std::abs(current.angular_z) <= stopped_tolerance)
    {
      break;
    }
    const PlannerVelocity2D next = approachVelocity(
      current, PlannerVelocity2D{}, limits, trajectory_config.simulation_dt);
    integrateVelocityStep(pose, current, next, trajectory_config.simulation_dt);
    poses.push_back(pose);
    current = next;
  }
  return poses;
}

// 判断整段目标控制是否均为零，用作所有运动序列失败后的停车兜底。
bool isFullStopSequence(const std::vector<PlannerVelocity2D> & controls)
{
  constexpr double tolerance = 1e-9;
  return std::all_of(
    controls.begin(), controls.end(),
    [](const PlannerVelocity2D & control) {
      return std::abs(control.linear_x) <= tolerance &&
      std::abs(control.angular_z) <= tolerance;
    });
}

// 计算控制序列的跟随、平滑、净空和前进代价，并执行硬碰撞与 TTC 准入。
SequenceEvaluation evaluateSequence(
  const PlannerVelocity2D & measured_velocity,
  const PlannerVelocity2D & previous_command,
  const PlannerVelocity2D & nominal_velocity,
  const std::vector<PlannerVelocity2D> & controls,
  const ObstacleIndex & obstacle_index,
  const TrajectoryConfig & trajectory_config,
  const FootprintConfig & footprint_config,
  const MotionLimits & limits,
  const VelocitySamplingConfig & cost_config)
{
  SequenceEvaluation evaluation;
  evaluation.full_stop = isFullStopSequence(controls);
  evaluation.trajectory = rolloutControlSequence(
    measured_velocity, controls, trajectory_config, limits);
  evaluation.collision = obstacle_index.check(evaluation.trajectory, footprint_config);
  if (evaluation.collision.collision) {
    return evaluation;
  }

  const double half_length = footprint_config.robot_length * 0.5 +
    footprint_config.safety_margin;
  const double half_width = footprint_config.robot_width * 0.5 +
    footprint_config.safety_margin;
  const double footprint_radius = std::hypot(half_length, half_width);
  double maximum_boundary_speed = 0.0;
  PlannerVelocity2D last = previous_command;
  for (const auto & control : controls) {
    maximum_boundary_speed = std::max(
      maximum_boundary_speed,
      std::abs(control.linear_x) + std::abs(control.angular_z) * footprint_radius);
    evaluation.cost.follow += cost_config.weight_follow_linear *
      squareValue(control.linear_x - nominal_velocity.linear_x) +
      cost_config.weight_follow_angular *
      squareValue(control.angular_z - nominal_velocity.angular_z);
    evaluation.cost.smooth += cost_config.weight_smooth_linear *
      squareValue(control.linear_x - last.linear_x) +
      cost_config.weight_smooth_angular *
      squareValue(control.angular_z - last.angular_z);
    evaluation.cost.progress += cost_config.weight_progress *
      std::max(0.0, nominal_velocity.linear_x - control.linear_x);
    last = control;
  }
  if (!controls.empty()) {
    const double inverse_size = 1.0 / static_cast<double>(controls.size());
    evaluation.cost.follow *= inverse_size;
    evaluation.cost.smooth *= inverse_size;
    evaluation.cost.progress *= inverse_size;
  }

  evaluation.required_clearance = std::max(
    cost_config.minimum_safe_clearance,
    cost_config.minimum_ttc * maximum_boundary_speed);
  if (std::isfinite(evaluation.collision.min_clearance) &&
    maximum_boundary_speed > 1e-9)
  {
    evaluation.clearance_ttc = evaluation.collision.min_clearance / maximum_boundary_speed;
  }
  if (std::isfinite(evaluation.collision.min_clearance) &&
    evaluation.collision.min_clearance < cost_config.obstacle_influence_distance)
  {
    const double ratio =
      (cost_config.obstacle_influence_distance -
      std::max(0.0, evaluation.collision.min_clearance)) /
      cost_config.obstacle_influence_distance;
    evaluation.cost.obstacle = cost_config.weight_obstacle * squareValue(ratio);
  }
  evaluation.cost.total = evaluation.cost.follow + evaluation.cost.smooth +
    evaluation.cost.obstacle + evaluation.cost.progress;

  const bool strictly_safe = !std::isfinite(evaluation.collision.min_clearance) ||
    evaluation.collision.min_clearance + 1e-9 >= evaluation.required_clearance;
  // 停车兜底只要求完整制动轨迹不发生硬碰撞，运动序列仍须满足软净空与 TTC。
  evaluation.valid = strictly_safe || evaluation.full_stop;
  return evaluation;
}

// 生成恒定控制序列，作为名义、停车和连贯转弯的确定性种子。
std::vector<PlannerVelocity2D> makeConstantSequence(
  std::size_t step_count,
  PlannerVelocity2D control,
  double maximum_forward_speed,
  const MotionLimits & limits,
  const VelocitySamplingConfig & cost_config,
  bool force_linear_stop)
{
  control = applyControlPolicy(
    control, maximum_forward_speed, limits, cost_config, force_linear_stop);
  return std::vector<PlannerVelocity2D>(step_count, control);
}

// 根据候选编号生成少量确定性种子，避免有限随机批量缺少连贯左右绕障轨迹。
bool makeSeedSequence(
  int candidate_index,
  std::size_t step_count,
  const PlannerVelocity2D & nominal_velocity,
  const MotionLimits & limits,
  const VelocitySamplingConfig & cost_config,
  bool force_linear_stop,
  std::vector<PlannerVelocity2D> * sequence)
{
  if (sequence == nullptr) {
    return false;
  }
  const double turn = std::min(
    cost_config.max_avoidance_angular_speed,
    std::max(cost_config.min_avoidance_angular_speed, 0.75));
  switch (candidate_index) {
    case 1:
      *sequence = makeConstantSequence(
        step_count, nominal_velocity, nominal_velocity.linear_x,
        limits, cost_config, force_linear_stop);
      return true;
    case 2:
      *sequence = makeConstantSequence(
        step_count, PlannerVelocity2D{}, nominal_velocity.linear_x,
        limits, cost_config, force_linear_stop);
      return true;
    case 3:
      *sequence = makeConstantSequence(
        step_count, {nominal_velocity.linear_x, turn}, nominal_velocity.linear_x,
        limits, cost_config, force_linear_stop);
      return true;
    case 4:
      *sequence = makeConstantSequence(
        step_count, {nominal_velocity.linear_x, -turn}, nominal_velocity.linear_x,
        limits, cost_config, force_linear_stop);
      return true;
    case 5:
      *sequence = makeConstantSequence(
        step_count, {0.5 * nominal_velocity.linear_x, turn}, nominal_velocity.linear_x,
        limits, cost_config, force_linear_stop);
      return true;
    case 6:
      *sequence = makeConstantSequence(
        step_count, {0.5 * nominal_velocity.linear_x, -turn}, nominal_velocity.linear_x,
        limits, cost_config, force_linear_stop);
      return true;
    case 7:
      *sequence = makeConstantSequence(
        step_count, {0.5 * nominal_velocity.linear_x, nominal_velocity.angular_z},
        nominal_velocity.linear_x,
        limits, cost_config, force_linear_stop);
      return true;
    default:
      return false;
  }
}

}  // namespace

// 校验 MPPI 批量、迭代次数、温度和噪声参数。
bool validateMppiConfig(const MppiConfig & config, std::string * reason)
{
  if (config.batch_size < 8 || config.iteration_count < 1) {
    return rejectWithReason("MPPI batch size or iteration count is too small", reason);
  }
  const bool finite = std::isfinite(config.temperature) &&
    std::isfinite(config.linear_noise_std) &&
    std::isfinite(config.angular_noise_std) &&
    std::isfinite(config.noise_correlation);
  if (!finite) {
    return rejectWithReason("MPPI config contains non-finite values", reason);
  }
  if (config.temperature <= 0.0 || config.linear_noise_std < 0.0 ||
    config.angular_noise_std < 0.0 || config.noise_correlation < 0.0 ||
    config.noise_correlation >= 1.0)
  {
    return rejectWithReason("MPPI temperature or noise parameters are invalid", reason);
  }
  return true;
}

// 创建带固定随机种子的优化器，保证回归测试和影子测试可复现。
MppiOptimizer::MppiOptimizer()
: random_engine_(active_seed_)
{
}

// 清除上一周期最优控制序列，下次规划重新以名义速度初始化。
void MppiOptimizer::reset()
{
  mean_sequence_.clear();
  initialized_ = false;
}

// 对整段时变速度序列执行 MPPI 扰动、前向展开、加权更新和最终安全校验。
LocalPlanResult MppiOptimizer::plan(
  const PlannerVelocity2D & measured_velocity,
  const PlannerVelocity2D & previous_command,
  const PlannerVelocity2D & nominal_velocity,
  const std::vector<ObstaclePoint2D> & obstacles,
  const TrajectoryConfig & trajectory_config,
  const FootprintConfig & footprint_config,
  const MotionLimits & limits,
  const VelocitySamplingConfig & cost_config,
  const MppiConfig & mppi_config,
  bool force_linear_stop)
{
  LocalPlanResult result;
  result.effective_nominal = makeEffectiveVelocity(nominal_velocity, limits);
  result.effective_nominal.linear_x = std::max(0.0, result.effective_nominal.linear_x);
  if (!validateTrajectoryConfig(trajectory_config) ||
    !validateFootprintConfig(footprint_config) || !validateMotionLimits(limits) ||
    !validateVelocitySamplingConfig(cost_config) || !validateMppiConfig(mppi_config))
  {
    return result;
  }
  // 障碍集合在本周期内不变，只建一次树供全部 MPPI 采样序列精确查询。
  const ObstacleIndex obstacle_index(obstacles);

  if (active_seed_ != mppi_config.random_seed) {
    active_seed_ = mppi_config.random_seed;
    random_engine_.seed(active_seed_);
    reset();
  }
  const std::size_t step_count = static_cast<std::size_t>(
    std::ceil(trajectory_config.prediction_time / trajectory_config.simulation_dt));
  const PlannerVelocity2D bounded_nominal = applyControlPolicy(
    result.effective_nominal, limits.max_linear_speed, limits, cost_config, force_linear_stop);
  if (!initialized_ || mean_sequence_.size() != step_count) {
    mean_sequence_ = std::vector<PlannerVelocity2D>(step_count, bounded_nominal);
    initialized_ = true;
  } else if (!mean_sequence_.empty()) {
    // 将上一周期最优序列前移一位，并用当前名义速度补齐预测末端。
    std::move(mean_sequence_.begin() + 1, mean_sequence_.end(), mean_sequence_.begin());
    mean_sequence_.back() = bounded_nominal;
  }
  if (force_linear_stop) {
    for (auto & control : mean_sequence_) {
      control.linear_x = 0.0;
    }
  }

  const auto nominal_sequence = makeConstantSequence(
    step_count, bounded_nominal, bounded_nominal.linear_x,
    limits, cost_config, force_linear_stop);
  const SequenceEvaluation nominal_evaluation = evaluateSequence(
    measured_velocity, previous_command, bounded_nominal, nominal_sequence, obstacle_index,
    trajectory_config, footprint_config, limits, cost_config);
  result.avoidance_active = force_linear_stop || nominal_evaluation.collision.collision ||
    (std::isfinite(nominal_evaluation.collision.min_clearance) &&
    nominal_evaluation.collision.min_clearance < cost_config.obstacle_influence_distance) ||
    !nominal_evaluation.valid;

  std::normal_distribution<double> standard_normal(0.0, 1.0);
  std::vector<PlannerVelocity2D> best_sequence;
  SequenceEvaluation best_evaluation;
  double best_cost = std::numeric_limits<double>::infinity();
  bool best_is_moving = false;

  for (int iteration = 0; iteration < mppi_config.iteration_count; ++iteration) {
    std::vector<std::vector<PlannerVelocity2D>> candidates;
    std::vector<SequenceEvaluation> evaluations;
    candidates.reserve(static_cast<std::size_t>(mppi_config.batch_size));
    evaluations.reserve(static_cast<std::size_t>(mppi_config.batch_size));
    double iteration_best_cost = std::numeric_limits<double>::infinity();

    for (int candidate_index = 0; candidate_index < mppi_config.batch_size; ++candidate_index) {
      std::vector<PlannerVelocity2D> sequence;
      if (candidate_index == 0) {
        sequence = mean_sequence_;
      } else {
        const bool seeded = makeSeedSequence(
          candidate_index, step_count, bounded_nominal, limits, cost_config,
          force_linear_stop, &sequence);
        if (!seeded) {
          sequence.reserve(step_count);
          double linear_noise = 0.0;
          double angular_noise = 0.0;
          const double innovation_scale = std::sqrt(
            1.0 - squareValue(mppi_config.noise_correlation));
          for (std::size_t step = 0U; step < step_count; ++step) {
            // 一阶相关噪声在整段时域内保持连续，同时仍允许速度逐时刻改变。
            linear_noise = mppi_config.noise_correlation * linear_noise +
              innovation_scale * standard_normal(random_engine_);
            angular_noise = mppi_config.noise_correlation * angular_noise +
              innovation_scale * standard_normal(random_engine_);
            sequence.push_back(
              applyControlPolicy(
                {mean_sequence_[step].linear_x +
                  mppi_config.linear_noise_std * linear_noise,
                  mean_sequence_[step].angular_z +
                  mppi_config.angular_noise_std * angular_noise},
                bounded_nominal.linear_x, limits, cost_config, force_linear_stop));
          }
        }
      }

      SequenceEvaluation evaluation = evaluateSequence(
        measured_velocity, previous_command, bounded_nominal, sequence, obstacle_index,
        trajectory_config, footprint_config, limits, cost_config);
      ++result.evaluated_count;
      if (evaluation.collision.collision) {
        ++result.collision_count;
      } else if (!evaluation.valid) {
        ++result.marginal_count;
      }
      candidates.push_back(std::move(sequence));
      evaluations.push_back(std::move(evaluation));
    }

    const bool iteration_has_moving = std::any_of(
      evaluations.begin(), evaluations.end(),
      [](const SequenceEvaluation & evaluation) {
        return evaluation.valid && !evaluation.full_stop;
      });
    for (std::size_t index = 0U; index < candidates.size(); ++index) {
      const auto & evaluation = evaluations[index];
      if (!evaluation.valid || (iteration_has_moving && evaluation.full_stop)) {
        continue;
      }
      iteration_best_cost = std::min(iteration_best_cost, evaluation.cost.total);
      const bool moving = !evaluation.full_stop;
      if ((moving && !best_is_moving) || (moving == best_is_moving &&
        evaluation.cost.total < best_cost))
      {
        // 只要存在安全运动序列就不选择停车，停车仅作为全部运动序列失败后的兜底。
        best_is_moving = moving;
        best_cost = evaluation.cost.total;
        best_sequence = candidates[index];
        best_evaluation = evaluation;
      }
    }
    if (!std::isfinite(iteration_best_cost)) {
      break;
    }
    std::vector<PlannerVelocity2D> updated_mean(step_count);
    double weight_sum = 0.0;
    for (std::size_t index = 0U; index < candidates.size(); ++index) {
      if (!evaluations[index].valid ||
        (iteration_has_moving && evaluations[index].full_stop))
      {
        continue;
      }
      const double weight = std::exp(
        -(evaluations[index].cost.total - iteration_best_cost) / mppi_config.temperature);
      weight_sum += weight;
      for (std::size_t step = 0U; step < step_count; ++step) {
        updated_mean[step].linear_x += weight * candidates[index][step].linear_x;
        updated_mean[step].angular_z += weight * candidates[index][step].angular_z;
      }
    }
    if (weight_sum <= std::numeric_limits<double>::epsilon()) {
      break;
    }
    for (std::size_t step = 0U; step < step_count; ++step) {
      updated_mean[step].linear_x /= weight_sum;
      updated_mean[step].angular_z /= weight_sum;
      updated_mean[step] = applyControlPolicy(
        updated_mean[step], bounded_nominal.linear_x,
        limits, cost_config, force_linear_stop);
    }
    mean_sequence_ = std::move(updated_mean);
  }

  SequenceEvaluation mean_evaluation = evaluateSequence(
    measured_velocity, previous_command, bounded_nominal, mean_sequence_, obstacle_index,
    trajectory_config, footprint_config, limits, cost_config);
  std::vector<PlannerVelocity2D> selected_sequence = mean_sequence_;
  if ((!mean_evaluation.valid || (best_is_moving && mean_evaluation.full_stop)) &&
    !best_sequence.empty())
  {
    // 加权平均后的序列不一定保持可行，失败时退回本周期代价最低的已验证序列。
    selected_sequence = best_sequence;
    mean_evaluation = best_evaluation;
    mean_sequence_ = best_sequence;
  }
  if (!mean_evaluation.valid || selected_sequence.empty()) {
    reset();
    return result;
  }

  result.valid = true;
  result.selected_velocity = selected_sequence.front();
  result.selected_controls = selected_sequence;
  result.selected_trajectory = std::move(mean_evaluation.trajectory);
  result.cost = mean_evaluation.cost;
  result.min_clearance = mean_evaluation.collision.min_clearance;
  result.required_clearance = mean_evaluation.required_clearance;
  result.clearance_ttc = mean_evaluation.clearance_ttc;
  result.selected_speed_scale = bounded_nominal.linear_x > 1e-9 ?
    result.selected_velocity.linear_x / bounded_nominal.linear_x : 0.0;
  return result;
}

}  // namespace go2_uwb_local_follow
