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

#include "go2_uwb_local_follow/rolling_obstacle_map_core.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace go2_uwb_local_follow
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kNanosecondsPerSecond = 1.0e9;

// 在需要时写入滚动地图参数校验失败原因。
bool rejectWithReason(const std::string & message, std::string * reason)
{
  if (reason != nullptr) {
    *reason = message;
  }
  return false;
}

// 判断二维位姿的时间和数值字段是否全部有效。
bool finitePose(const TimedPose2D & pose)
{
  return pose.stamp_ns > 0 && std::isfinite(pose.x) && std::isfinite(pose.y) &&
         std::isfinite(pose.yaw);
}

// 判断三维障碍点是否全部为有限值。
bool finitePoint(const RollingObstaclePoint & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

// 在两个时间有序的二维位姿之间执行线性位置和最短角度插值。
TimedPose2D interpolatePose(
  const TimedPose2D & first,
  const TimedPose2D & second,
  std::int64_t stamp_ns)
{
  const std::int64_t duration_ns = second.stamp_ns - first.stamp_ns;
  if (duration_ns <= 0) {
    TimedPose2D result = first;
    result.stamp_ns = stamp_ns;
    return result;
  }
  const double ratio = static_cast<double>(stamp_ns - first.stamp_ns) /
    static_cast<double>(duration_ns);
  TimedPose2D result;
  result.stamp_ns = stamp_ns;
  result.x = first.x + ratio * (second.x - first.x);
  result.y = first.y + ratio * (second.y - first.y);
  result.yaw = normalizeRollingAngle(
    first.yaw + ratio * normalizeRollingAngle(second.yaw - first.yaw));
  return result;
}

}  // namespace

// 校验体素、衰减、局部范围、里程计缓存和跳变阈值参数。
bool validateRollingMapConfig(const RollingMapConfig & config, std::string * reason)
{
  const bool finite = std::isfinite(config.voxel_size) &&
    std::isfinite(config.obstacle_retention_sec) &&
    std::isfinite(config.rolling_radius) &&
    std::isfinite(config.odom_buffer_duration_sec) &&
    std::isfinite(config.max_pose_extrapolation_sec) &&
    std::isfinite(config.odom_jump_distance) &&
    std::isfinite(config.odom_jump_yaw) &&
    std::isfinite(config.odom_jump_check_interval_sec) &&
    std::isfinite(config.ray_clearing_max_range) &&
    std::isfinite(config.ray_clearing_endpoint_margin);
  if (!finite) {
    return rejectWithReason("rolling map config contains non-finite values", reason);
  }
  if (config.voxel_size <= 0.0 || config.obstacle_retention_sec <= 0.0 ||
    config.rolling_radius <= 0.0 || config.max_obstacle_points == 0U ||
    config.odom_buffer_duration_sec <= 0.0 || config.max_pose_extrapolation_sec < 0.0 ||
    config.odom_jump_distance <= 0.0 || config.odom_jump_yaw <= 0.0 ||
    config.odom_jump_check_interval_sec <= 0.0 || config.ray_clearing_max_range <= 0.0 ||
    config.ray_clearing_endpoint_margin < 0.0 ||
    config.ray_clearing_endpoint_margin >= config.ray_clearing_max_range ||
    config.ray_clearing_min_observations == 0U)
  {
    return rejectWithReason("rolling map distances, durations and limits are invalid", reason);
  }
  return true;
}

// 将角度归一化到 [-pi, pi]，用于跨越正负 pi 的姿态插值。
double normalizeRollingAngle(double angle)
{
  if (!std::isfinite(angle)) {
    return 0.0;
  }
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

// 将采集时刻机身坐标系的点转换到局部 odom 坐标系。
RollingObstaclePoint transformRollingPointToOdom(
  const RollingObstaclePoint & point,
  const TimedPose2D & pose)
{
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  return RollingObstaclePoint{
    pose.x + cosine * point.x - sine * point.y,
    pose.y + sine * point.x + cosine * point.y,
    point.z};
}

// 将局部 odom 地图点转换到指定时刻的机身坐标系。
RollingObstaclePoint transformRollingPointToBase(
  const RollingObstaclePoint & point,
  const TimedPose2D & pose)
{
  const double dx = point.x - pose.x;
  const double dy = point.y - pose.y;
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  return RollingObstaclePoint{
    cosine * dx + sine * dy,
    -sine * dx + cosine * dy,
    point.z};
}

// 使用给定配置创建有界里程计位姿缓存。
OdomPoseBuffer::OdomPoseBuffer(const RollingMapConfig & config)
: config_(config)
{
}

// 追加单调递增的里程计位姿，并在短时间大跳变时重置缓存。
PoseAppendResult OdomPoseBuffer::append(const TimedPose2D & pose)
{
  if (!finitePose(pose)) {
    return PoseAppendResult::kRejected;
  }
  if (!poses_.empty() && pose.stamp_ns == poses_.back().stamp_ns) {
    return PoseAppendResult::kRejected;
  }
  if (!poses_.empty() && pose.stamp_ns < poses_.back().stamp_ns) {
    // ROS 时间或里程计时间回退后旧缓存不再具有共同时间基准，必须重新开始。
    poses_.clear();
    poses_.push_back(pose);
    return PoseAppendResult::kResetDetected;
  }

  bool reset_detected = false;
  if (!poses_.empty()) {
    const TimedPose2D & previous = poses_.back();
    const double interval_sec = static_cast<double>(pose.stamp_ns - previous.stamp_ns) /
      kNanosecondsPerSecond;
    const double distance = std::hypot(pose.x - previous.x, pose.y - previous.y);
    const double yaw_delta = std::abs(normalizeRollingAngle(pose.yaw - previous.yaw));
    if (interval_sec <= config_.odom_jump_check_interval_sec &&
      (distance > config_.odom_jump_distance || yaw_delta > config_.odom_jump_yaw))
    {
      poses_.clear();
      reset_detected = true;
    }
  }

  poses_.push_back(pose);
  const std::int64_t buffer_duration_ns = static_cast<std::int64_t>(
    config_.odom_buffer_duration_sec * kNanosecondsPerSecond);
  while (poses_.size() > 1U && pose.stamp_ns - poses_.front().stamp_ns > buffer_duration_ns) {
    poses_.pop_front();
  }
  return reset_detected ? PoseAppendResult::kResetDetected : PoseAppendResult::kAccepted;
}

// 按时间戳插值位姿；仅在配置容差内允许首尾两端短时外推。
bool OdomPoseBuffer::lookup(std::int64_t stamp_ns, TimedPose2D * pose) const
{
  if (pose == nullptr || stamp_ns <= 0 || poses_.empty()) {
    return false;
  }
  const std::int64_t extrapolation_ns = static_cast<std::int64_t>(
    config_.max_pose_extrapolation_sec * kNanosecondsPerSecond);
  if (stamp_ns < poses_.front().stamp_ns) {
    if (poses_.front().stamp_ns - stamp_ns > extrapolation_ns) {
      return false;
    }
    if (poses_.size() < 2U) {
      *pose = poses_.front();
      pose->stamp_ns = stamp_ns;
      return true;
    }
    // 启动阶段点云可能先于首帧里程计到达，使用最早两帧恢复采集时刻位姿。
    *pose = interpolatePose(poses_[0U], poses_[1U], stamp_ns);
    return true;
  }
  if (stamp_ns > poses_.back().stamp_ns) {
    if (stamp_ns - poses_.back().stamp_ns > extrapolation_ns) {
      return false;
    }
    if (poses_.size() < 2U) {
      *pose = poses_.back();
      pose->stamp_ns = stamp_ns;
      return true;
    }
    *pose = interpolatePose(poses_[poses_.size() - 2U], poses_.back(), stamp_ns);
    return true;
  }

  const auto upper = std::lower_bound(
    poses_.begin(), poses_.end(), stamp_ns,
    [](const TimedPose2D & candidate, std::int64_t value) {
      return candidate.stamp_ns < value;
    });
  if (upper == poses_.end()) {
    *pose = poses_.back();
    return true;
  }
  if (upper->stamp_ns == stamp_ns || upper == poses_.begin()) {
    *pose = *upper;
    return true;
  }
  *pose = interpolatePose(*std::prev(upper), *upper, stamp_ns);
  return true;
}

// 清空全部历史位姿。
void OdomPoseBuffer::clear()
{
  poses_.clear();
}

// 返回当前缓存位姿数量。
std::size_t OdomPoseBuffer::size() const
{
  return poses_.size();
}

// 使用给定配置创建有时间衰减的二维滚动障碍地图。
RollingObstacleMap::RollingObstacleMap(const RollingMapConfig & config)
: config_(config)
{
  cells_.reserve(config_.max_obstacle_points);
}

// 判断两个二维体素索引是否相同。
bool RollingObstacleMap::CellKey::operator==(const CellKey & other) const
{
  return x == other.x && y == other.y;
}

// 组合二维整数体素索引的哈希值。
std::size_t RollingObstacleMap::CellKeyHash::operator()(const CellKey & key) const
{
  const auto first = std::hash<std::int64_t>{}(key.x);
  const auto second = std::hash<std::int64_t>{}(key.y);
  return first ^ (second + 0x9e3779b97f4a7c15ULL + (first << 6U) + (first >> 2U));
}

// 把当前帧机身点转换到 odom，刷新对应体素并执行衰减和范围裁剪。
void RollingObstacleMap::integrate(
  const std::vector<RollingObstaclePoint> & base_points,
  const TimedPose2D & pose)
{
  (void)integrateObservation(base_points, {}, RollingObstaclePoint{}, pose);
}

// 先用同帧有效深度射线清除历史体素，再写入当前障碍并执行时间和范围裁剪。
std::size_t RollingObstacleMap::integrateObservation(
  const std::vector<RollingObstaclePoint> & base_obstacles,
  const std::vector<RollingObstaclePoint> & base_ray_endpoints,
  const RollingObstaclePoint & base_sensor_origin,
  const TimedPose2D & pose)
{
  if (!finitePose(pose)) {
    return 0U;
  }

  std::vector<RollingObstaclePoint> odom_obstacles;
  odom_obstacles.reserve(base_obstacles.size());
  std::unordered_map<CellKey, bool, CellKeyHash> protected_cells;
  protected_cells.reserve(base_obstacles.size());
  for (const auto & base_point : base_obstacles) {
    if (!finitePoint(base_point)) {
      continue;
    }
    const RollingObstaclePoint odom_point = transformRollingPointToOdom(base_point, pose);
    const CellKey key{
      static_cast<std::int64_t>(std::floor(odom_point.x / config_.voxel_size)),
      static_cast<std::int64_t>(std::floor(odom_point.y / config_.voxel_size))};
    odom_obstacles.push_back(odom_point);
    protected_cells.emplace(key, true);
  }

  std::size_t cleared_cells = 0U;
  if (config_.enable_ray_clearing && finitePoint(base_sensor_origin)) {
    std::vector<RollingObstaclePoint> odom_ray_endpoints;
    odom_ray_endpoints.reserve(base_ray_endpoints.size());
    for (const auto & base_endpoint : base_ray_endpoints) {
      if (finitePoint(base_endpoint)) {
        odom_ray_endpoints.push_back(transformRollingPointToOdom(base_endpoint, pose));
      }
    }
    const RollingObstaclePoint odom_sensor_origin =
      transformRollingPointToOdom(base_sensor_origin, pose);
    cleared_cells = clearObservedFreeSpace(
      odom_ray_endpoints, odom_sensor_origin, protected_cells);
  }

  for (const auto & odom_point : odom_obstacles) {
    const CellKey key{
      static_cast<std::int64_t>(std::floor(odom_point.x / config_.voxel_size)),
      static_cast<std::int64_t>(std::floor(odom_point.y / config_.voxel_size))};
    cells_[key] = CellValue{odom_point, pose.stamp_ns};
  }
  prune(pose);
  return cleared_cells;
}

// 将当前保留的 odom 障碍转换到指定姿态的机身坐标系。
std::vector<RollingObstaclePoint> RollingObstacleMap::pointsInBase(
  const TimedPose2D & pose) const
{
  std::vector<RollingObstaclePoint> points;
  points.reserve(cells_.size());
  for (const auto & item : cells_) {
    points.push_back(transformRollingPointToBase(item.second.point, pose));
  }
  std::sort(
    points.begin(), points.end(),
    [](const RollingObstaclePoint & first, const RollingObstaclePoint & second) {
      if (first.x != second.x) {
        return first.x < second.x;
      }
      if (first.y != second.y) {
        return first.y < second.y;
      }
      return first.z < second.z;
    });
  return points;
}

// 清空全部障碍体素。
void RollingObstacleMap::clear()
{
  cells_.clear();
}

// 返回当前保留的障碍体素数量。
std::size_t RollingObstacleMap::size() const
{
  return cells_.size();
}

// 统计每个二维体素的同帧射线穿越次数，并清除达到确认阈值的历史障碍。
std::size_t RollingObstacleMap::clearObservedFreeSpace(
  const std::vector<RollingObstaclePoint> & odom_ray_endpoints,
  const RollingObstaclePoint & odom_sensor_origin,
  const std::unordered_map<CellKey, bool, CellKeyHash> & protected_cells)
{
  if (odom_ray_endpoints.empty() || cells_.empty() || !finitePoint(odom_sensor_origin)) {
    return 0U;
  }

  std::unordered_map<CellKey, std::size_t, CellKeyHash> clearing_votes;
  clearing_votes.reserve(std::min(cells_.size(), odom_ray_endpoints.size()));
  for (const auto & endpoint : odom_ray_endpoints) {
    if (!finitePoint(endpoint)) {
      continue;
    }
    const double delta_x = endpoint.x - odom_sensor_origin.x;
    const double delta_y = endpoint.y - odom_sensor_origin.y;
    const double full_range = std::hypot(delta_x, delta_y);
    const double clearing_range = std::min(
      full_range - config_.ray_clearing_endpoint_margin,
      config_.ray_clearing_max_range);
    if (!std::isfinite(full_range) || clearing_range <= 0.0) {
      continue;
    }

    const double scale = clearing_range / full_range;
    const double end_x = odom_sensor_origin.x + delta_x * scale;
    const double end_y = odom_sensor_origin.y + delta_y * scale;
    CellKey current{
      static_cast<std::int64_t>(std::floor(odom_sensor_origin.x / config_.voxel_size)),
      static_cast<std::int64_t>(std::floor(odom_sensor_origin.y / config_.voxel_size))};
    const CellKey end{
      static_cast<std::int64_t>(std::floor(end_x / config_.voxel_size)),
      static_cast<std::int64_t>(std::floor(end_y / config_.voxel_size))};
    const std::int64_t step_x = delta_x > 0.0 ? 1 : (delta_x < 0.0 ? -1 : 0);
    const std::int64_t step_y = delta_y > 0.0 ? 1 : (delta_y < 0.0 ? -1 : 0);
    const double infinity = std::numeric_limits<double>::infinity();
    const double t_delta_x = step_x == 0 ? infinity : config_.voxel_size / std::abs(delta_x);
    const double t_delta_y = step_y == 0 ? infinity : config_.voxel_size / std::abs(delta_y);
    const double next_boundary_x = step_x > 0 ?
      static_cast<double>(current.x + 1) * config_.voxel_size :
      static_cast<double>(current.x) * config_.voxel_size;
    const double next_boundary_y = step_y > 0 ?
      static_cast<double>(current.y + 1) * config_.voxel_size :
      static_cast<double>(current.y) * config_.voxel_size;
    double t_max_x = step_x == 0 ? infinity :
      (next_boundary_x - odom_sensor_origin.x) / delta_x;
    double t_max_y = step_y == 0 ? infinity :
      (next_boundary_y - odom_sensor_origin.y) / delta_y;

    while (!(current == end)) {
      if (t_max_x < t_max_y) {
        current.x += step_x;
        t_max_x += t_delta_x;
      } else if (t_max_y < t_max_x) {
        current.y += step_y;
        t_max_y += t_delta_y;
      } else {
        current.x += step_x;
        current.y += step_y;
        t_max_x += t_delta_x;
        t_max_y += t_delta_y;
      }

      // 当前帧确认的障碍是射线终止面，禁止清除它及其后方的历史障碍。
      if (protected_cells.count(current) != 0U) {
        break;
      }
      if (cells_.count(current) != 0U) {
        ++clearing_votes[current];
      }
    }
  }

  std::size_t cleared_cells = 0U;
  for (const auto & vote : clearing_votes) {
    if (vote.second >= config_.ray_clearing_min_observations) {
      cleared_cells += cells_.erase(vote.first);
    }
  }
  return cleared_cells;
}

// 删除超过保留时间或滚动半径的障碍，并按新鲜度和距离限制总点数。
void RollingObstacleMap::prune(const TimedPose2D & pose)
{
  const std::int64_t retention_ns = static_cast<std::int64_t>(
    config_.obstacle_retention_sec * kNanosecondsPerSecond);
  const double radius_squared = config_.rolling_radius * config_.rolling_radius;
  for (auto iterator = cells_.begin(); iterator != cells_.end(); ) {
    const double dx = iterator->second.point.x - pose.x;
    const double dy = iterator->second.point.y - pose.y;
    const bool expired = pose.stamp_ns - iterator->second.last_seen_ns > retention_ns;
    const bool outside = dx * dx + dy * dy > radius_squared;
    if (expired || outside) {
      iterator = cells_.erase(iterator);
    } else {
      ++iterator;
    }
  }
  if (cells_.size() <= config_.max_obstacle_points) {
    return;
  }

  struct RankedCell
  {
    CellKey key;
    std::int64_t last_seen_ns{0};
    double distance_squared{0.0};
  };
  std::vector<RankedCell> ranked;
  ranked.reserve(cells_.size());
  for (const auto & item : cells_) {
    const double dx = item.second.point.x - pose.x;
    const double dy = item.second.point.y - pose.y;
    ranked.push_back(RankedCell{item.first, item.second.last_seen_ns, dx * dx + dy * dy});
  }
  std::sort(
    ranked.begin(), ranked.end(),
    [](const RankedCell & first, const RankedCell & second) {
      if (first.last_seen_ns != second.last_seen_ns) {
        return first.last_seen_ns > second.last_seen_ns;
      }
      return first.distance_squared < second.distance_squared;
    });
  for (std::size_t index = config_.max_obstacle_points; index < ranked.size(); ++index) {
    cells_.erase(ranked[index].key);
  }
}

}  // namespace go2_uwb_local_follow
