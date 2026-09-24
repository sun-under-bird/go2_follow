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

#include "go2_uwb_behavior/behavior_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace go2_uwb_behavior
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1e-9;

// 把角度归一化到 [-pi, pi]，避免跨 ±pi 时把小幅偏航误判成大幅偏航。
double normalizeAngle(double angle)
{
  return std::remainder(angle, 2.0 * kPi);
}

// 在需要时写入参数校验失败原因。
bool rejectWithReason(const std::string & message, std::string * reason)
{
  if (reason != nullptr) {
    *reason = message;
  }
  return false;
}

// 返回一组数值的中位数，偶数样本取中间两项均值。
double medianValue(std::vector<double> values)
{
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2U;
  if (values.size() % 2U == 1U) {
    return values[middle];
  }
  return 0.5 * (values[middle - 1U] + values[middle]);
}

}  // namespace

// 校验主人中心滤波参数，失败时返回具体原因。
bool validateOwnerFilterConfig(const OwnerFilterConfig & config, std::string * reason)
{
  if (config.median_window == 0U) {
    return rejectWithReason("median window must be positive", reason);
  }
  if (!std::isfinite(config.low_pass_alpha) || config.low_pass_alpha <= 0.0 ||
    config.low_pass_alpha > 1.0)
  {
    return rejectWithReason("low pass alpha must be within (0, 1]", reason);
  }
  if (!std::isfinite(config.center_deadband) || config.center_deadband < 0.0 ||
    !std::isfinite(config.center_move_confirm_sec) || config.center_move_confirm_sec < 0.0 ||
    !std::isfinite(config.center_max_speed) || config.center_max_speed <= 0.0)
  {
    return rejectWithReason("owner center hysteresis parameters are invalid", reason);
  }
  return true;
}

// 校验随机目标采样参数及各半径之间的约束。
bool validateRoamSamplingConfig(const RoamSamplingConfig & config, std::string * reason)
{
  const bool finite = std::isfinite(config.owner_keepout_radius) &&
    std::isfinite(config.random_goal_radius_max) &&
    std::isfinite(config.random_step_min) && std::isfinite(config.random_step_max) &&
    std::isfinite(config.goal_obstacle_clearance);
  if (!finite) {
    return rejectWithReason("roam sampling config contains non-finite values", reason);
  }
  if (config.owner_keepout_radius < 0.0 ||
    config.random_goal_radius_max <= config.owner_keepout_radius)
  {
    return rejectWithReason("owner goal radii are invalid", reason);
  }
  if (config.random_step_min <= 0.0 || config.random_step_max < config.random_step_min) {
    return rejectWithReason("random step bounds are invalid", reason);
  }
  if (config.goal_obstacle_clearance < 0.0 || config.max_sample_attempts == 0U) {
    return rejectWithReason("goal clearance and sample attempts are invalid", reason);
  }
  return true;
}

// 校验软返回、限制区和绝对围栏的顺序关系。
bool validateGeofenceConfig(const GeofenceConfig & config, std::string * reason)
{
  const bool finite = std::isfinite(config.return_trigger_radius) &&
    std::isfinite(config.return_release_radius) &&
    std::isfinite(config.restrictive_radius) && std::isfinite(config.absolute_radius) &&
    std::isfinite(config.prediction_sec) && std::isfinite(config.simulation_dt);
  if (!finite) {
    return rejectWithReason("geofence config contains non-finite values", reason);
  }
  if (config.return_release_radius <= 0.0 ||
    config.return_trigger_radius <= config.return_release_radius ||
    config.restrictive_radius <= config.return_trigger_radius ||
    config.absolute_radius <= config.restrictive_radius)
  {
    return rejectWithReason("geofence radii must be strictly ordered", reason);
  }
  if (config.prediction_sec <= 0.0 || config.simulation_dt <= 0.0 ||
    config.simulation_dt > config.prediction_sec)
  {
    return rejectWithReason("geofence prediction timing is invalid", reason);
  }
  return true;
}

// 计算两个二维点之间的欧氏距离。
double distanceBetween(const Point2D & first, const Point2D & second)
{
  return std::hypot(first.x - second.x, first.y - second.y);
}

// 将机身坐标点转换到连续 odom 坐标系。
Point2D transformBasePointToOdom(const Point2D & point, const Pose2D & robot_pose)
{
  const double cosine = std::cos(robot_pose.yaw);
  const double sine = std::sin(robot_pose.yaw);
  return Point2D{
    robot_pose.x + cosine * point.x - sine * point.y,
    robot_pose.y + sine * point.x + cosine * point.y};
}

// 将 odom 坐标点转换到当前机身坐标系。
Point2D transformOdomPointToBase(const Point2D & point, const Pose2D & robot_pose)
{
  const double dx = point.x - robot_pose.x;
  const double dy = point.y - robot_pose.y;
  const double cosine = std::cos(robot_pose.yaw);
  const double sine = std::sin(robot_pose.yaw);
  return Point2D{cosine * dx + sine * dy, -sine * dx + cosine * dy};
}

// 初始化中值、低通和迟滞中心滤波器。
OwnerCenterFilter::OwnerCenterFilter(OwnerFilterConfig config)
: config_(std::move(config))
{
  std::string reason;
  if (!validateOwnerFilterConfig(config_, &reason)) {
    throw std::invalid_argument(reason);
  }
}

// 清空历史样本和已确认的玩耍中心。
void OwnerCenterFilter::reset()
{
  samples_.clear();
  filtered_owner_ = Point2D{};
  play_center_ = Point2D{};
  pending_move_sec_ = 0.0;
  valid_ = false;
}

// 输入一次新的主人 odom 坐标，并按真实采样间隔更新迟滞中心。
bool OwnerCenterFilter::update(const Point2D & measurement, double dt)
{
  if (!std::isfinite(measurement.x) || !std::isfinite(measurement.y) ||
    !std::isfinite(dt) || dt < 0.0)
  {
    return false;
  }

  samples_.push_back(measurement);
  while (samples_.size() > config_.median_window) {
    samples_.pop_front();
  }

  std::vector<double> x_values;
  std::vector<double> y_values;
  x_values.reserve(samples_.size());
  y_values.reserve(samples_.size());
  for (const auto & sample : samples_) {
    x_values.push_back(sample.x);
    y_values.push_back(sample.y);
  }
  const Point2D median{medianValue(x_values), medianValue(y_values)};

  if (!valid_) {
    filtered_owner_ = median;
    play_center_ = median;
    valid_ = true;
    return true;
  }

  filtered_owner_.x += config_.low_pass_alpha * (median.x - filtered_owner_.x);
  filtered_owner_.y += config_.low_pass_alpha * (median.y - filtered_owner_.y);

  const double center_error = distanceBetween(play_center_, filtered_owner_);
  if (center_error <= config_.center_deadband + kTolerance) {
    pending_move_sec_ = 0.0;
    return true;
  }

  pending_move_sec_ += dt;
  if (pending_move_sec_ + kTolerance < config_.center_move_confirm_sec) {
    return true;
  }

  // 确认主人发生持续位移后限速更新中心，避免 UWB 阶跃直接拖动随机目标区域。
  const double maximum_step = config_.center_max_speed * dt;
  const double step = std::min(maximum_step, center_error);
  if (center_error > kTolerance) {
    play_center_.x += step * (filtered_owner_.x - play_center_.x) / center_error;
    play_center_.y += step * (filtered_owner_.y - play_center_.y) / center_error;
  }
  return true;
}

// 返回是否已经产生有效的平滑位置。
bool OwnerCenterFilter::valid() const
{
  return valid_;
}

// 返回当前滤波后的实时主人位置，用于半径安全判断。
const Point2D & OwnerCenterFilter::filteredOwner() const
{
  return filtered_owner_;
}

// 返回带大死区和持续确认的玩耍中心，用于随机目标采样。
const Point2D & OwnerCenterFilter::playCenter() const
{
  return play_center_;
}

// 返回滤波窗口已接收的有效样本数量。
std::size_t OwnerCenterFilter::sampleCount() const
{
  return samples_.size();
}

// 在主人圆环、单步距离及障碍净空的交集内采样一个目标。
std::optional<Point2D> sampleRandomGoal(
  const Point2D & owner_center,
  const Point2D & robot_position,
  const std::vector<Point2D> & obstacle_points,
  const RoamSamplingConfig & config,
  std::mt19937 & generator)
{
  std::string reason;
  if (!validateRoamSamplingConfig(config, &reason)) {
    return std::nullopt;
  }

  std::uniform_real_distribution<double> step_squared(
    config.random_step_min * config.random_step_min,
    config.random_step_max * config.random_step_max);
  std::uniform_real_distribution<double> angle(-kPi, kPi);

  for (std::size_t attempt = 0U; attempt < config.max_sample_attempts; ++attempt) {
    // 先在机器人单步圆环内均匀采样，再检查主人圆环，避免机器人靠近主人时大量空采样。
    const double step_distance = std::sqrt(step_squared(generator));
    const double theta = angle(generator);
    const Point2D candidate{
      robot_position.x + step_distance * std::cos(theta),
      robot_position.y + step_distance * std::sin(theta)};
    const double owner_distance = distanceBetween(candidate, owner_center);
    if (owner_distance + kTolerance < config.owner_keepout_radius ||
      owner_distance > config.random_goal_radius_max + kTolerance)
    {
      continue;
    }

    const bool near_obstacle = std::any_of(
      obstacle_points.begin(), obstacle_points.end(),
      [&](const Point2D & obstacle) {
        return distanceBetween(candidate, obstacle) + kTolerance <
        config.goal_obstacle_clearance;
      });
    if (!near_obstacle) {
      return candidate;
    }
  }
  return std::nullopt;
}

// 以 UWB 固定中心为圆心，在指定圆环内按面积均匀采样并检查障碍净空。
std::optional<Point2D> sampleRandomGoalInAnnulus(
  const Point2D & center,
  const std::vector<Point2D> & obstacle_points,
  double minimum_radius,
  double maximum_radius,
  double obstacle_clearance,
  std::size_t max_attempts,
  std::mt19937 & generator)
{
  const bool valid = std::isfinite(center.x) && std::isfinite(center.y) &&
    std::isfinite(minimum_radius) && std::isfinite(maximum_radius) &&
    std::isfinite(obstacle_clearance) && minimum_radius >= 0.0 &&
    maximum_radius > minimum_radius && obstacle_clearance >= 0.0 && max_attempts > 0U;
  if (!valid) {
    return std::nullopt;
  }

  // 对半径平方做均匀采样，避免随机点偏向圆环内侧。
  std::uniform_real_distribution<double> radius_squared(
    minimum_radius * minimum_radius, maximum_radius * maximum_radius);
  std::uniform_real_distribution<double> angle(-kPi, kPi);
  for (std::size_t attempt = 0U; attempt < max_attempts; ++attempt) {
    const double radius = std::sqrt(radius_squared(generator));
    const double theta = angle(generator);
    const Point2D candidate{
      center.x + radius * std::cos(theta), center.y + radius * std::sin(theta)};
    const bool near_obstacle = std::any_of(
      obstacle_points.begin(), obstacle_points.end(),
      [&](const Point2D & obstacle) {
        return distanceBetween(candidate, obstacle) + kTolerance < obstacle_clearance;
      });
    if (!near_obstacle) {
      return candidate;
    }
  }
  return std::nullopt;
}

// 预测恒定速度下的短时轨迹，并判定是否违反主人半径围栏。
GeofenceResult evaluateGeofenceCommand(
  const Pose2D & robot_pose,
  const Point2D & owner_position,
  const Velocity2D & command,
  const GeofenceConfig & config)
{
  GeofenceResult result;
  result.current_distance = distanceBetween(Point2D{robot_pose.x, robot_pose.y}, owner_position);
  result.final_distance = result.current_distance;
  result.maximum_distance = result.current_distance;

  Pose2D predicted = robot_pose;
  double elapsed = 0.0;
  while (elapsed + kTolerance < config.prediction_sec) {
    const double step = std::min(config.simulation_dt, config.prediction_sec - elapsed);
    const double middle_yaw = predicted.yaw + 0.5 * command.angular_z * step;
    predicted.x += command.linear_x * std::cos(middle_yaw) * step;
    predicted.y += command.linear_x * std::sin(middle_yaw) * step;
    predicted.yaw += command.angular_z * step;
    elapsed += step;
    result.final_distance = distanceBetween(Point2D{predicted.x, predicted.y}, owner_position);
    result.maximum_distance = std::max(result.maximum_distance, result.final_distance);
  }

  if (result.current_distance <= config.absolute_radius + kTolerance &&
    result.maximum_distance > config.absolute_radius + kTolerance)
  {
    result.allowed = false;
  }
  if (result.current_distance >= config.restrictive_radius - kTolerance &&
    result.maximum_distance > result.current_distance + kTolerance)
  {
    result.allowed = false;
  }
  if (result.current_distance > config.absolute_radius + kTolerance &&
    result.final_distance >= result.current_distance - kTolerance)
  {
    result.allowed = false;
  }
  return result;
}

// 判断里程计速度是否已经进入可确认停车的阈值。
bool isRobotStopped(
  const Velocity2D & measured,
  double linear_threshold,
  double angular_threshold)
{
  return std::isfinite(measured.linear_x) && std::isfinite(measured.angular_z) &&
         std::abs(measured.linear_x) <= linear_threshold + kTolerance &&
         std::abs(measured.angular_z) <= angular_threshold + kTolerance;
}

// 配置允许的平移和偏航变化量。
PoseStationarityMonitor::PoseStationarityMonitor(double position_epsilon, double yaw_epsilon)
: position_epsilon_(position_epsilon), yaw_epsilon_(yaw_epsilon)
{
  if (!std::isfinite(position_epsilon_) || position_epsilon_ <= 0.0 ||
    !std::isfinite(yaw_epsilon_) || yaw_epsilon_ <= 0.0)
  {
    throw std::invalid_argument("pose stationarity epsilon must be positive");
  }
}

// 位姿相对参考点仍在容差内返回 true；超出容差则刷新参考点并返回 false。
bool PoseStationarityMonitor::update(const Pose2D & pose)
{
  if (!reference_valid_) {
    reference_ = pose;
    reference_valid_ = true;
    return false;
  }
  const double delta_x = pose.x - reference_.x;
  const double delta_y = pose.y - reference_.y;
  const double delta_yaw = normalizeAngle(pose.yaw - reference_.yaw);
  // 位姿出现非有限值时同样刷新参考点：宁可回到"未确认停车"，也不能据此宣称停稳。
  if (!std::isfinite(delta_x) || !std::isfinite(delta_y) || !std::isfinite(delta_yaw) ||
    std::hypot(delta_x, delta_y) > position_epsilon_ || std::abs(delta_yaw) > yaw_epsilon_)
  {
    reference_ = pose;
    return false;
  }
  return true;
}

// 丢弃参考点，下一次更新重新建立基准。
void PoseStationarityMonitor::reset()
{
  reference_valid_ = false;
}

// 配置有效进展距离和允许无进展的最长时间。
ProgressMonitor::ProgressMonitor(double required_progress, double window_sec)
: required_progress_(required_progress), window_sec_(window_sec)
{
  if (!std::isfinite(required_progress_) || required_progress_ <= 0.0 ||
    !std::isfinite(window_sec_) || window_sec_ <= 0.0)
  {
    throw std::invalid_argument("progress monitor parameters must be positive");
  }
}

// 以新目标距离重置无进展计时窗口。
void ProgressMonitor::reset(double distance)
{
  reference_distance_ = distance;
  elapsed_sec_ = 0.0;
  initialized_ = std::isfinite(distance);
}

// 更新目标距离，达到无进展窗口时返回 true。
bool ProgressMonitor::update(double distance, double dt)
{
  if (!std::isfinite(distance) || !std::isfinite(dt) || dt < 0.0) {
    return false;
  }
  if (!initialized_) {
    reset(distance);
    return false;
  }
  if (reference_distance_ - distance >= required_progress_ - kTolerance) {
    reference_distance_ = distance;
    elapsed_sec_ = 0.0;
    return false;
  }
  elapsed_sec_ += dt;
  return elapsed_sec_ + kTolerance >= window_sec_;
}

}  // namespace go2_uwb_behavior
