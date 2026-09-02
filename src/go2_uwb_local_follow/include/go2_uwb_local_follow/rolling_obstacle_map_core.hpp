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

#ifndef GO2_UWB_LOCAL_FOLLOW__ROLLING_OBSTACLE_MAP_CORE_HPP_
#define GO2_UWB_LOCAL_FOLLOW__ROLLING_OBSTACLE_MAP_CORE_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <unordered_map>
#include <vector>

namespace go2_uwb_local_follow
{

struct TimedPose2D
{
  std::int64_t stamp_ns{0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct RollingObstaclePoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct RollingMapConfig
{
  double voxel_size{0.05};
  double obstacle_retention_sec{1.00};
  double rolling_radius{3.00};
  std::size_t max_obstacle_points{5000U};
  double odom_buffer_duration_sec{3.00};
  double max_pose_extrapolation_sec{0.05};
  double odom_jump_distance{1.00};
  double odom_jump_yaw{0.80};
  double odom_jump_check_interval_sec{0.50};
};

enum class PoseAppendResult
{
  kAccepted,
  kResetDetected,
  kRejected
};

// 校验体素、衰减、局部范围、里程计缓存和跳变阈值参数。
bool validateRollingMapConfig(
  const RollingMapConfig & config,
  std::string * reason = nullptr);

// 将角度归一化到 [-pi, pi]，用于跨越正负 pi 的姿态插值。
double normalizeRollingAngle(double angle);

// 将采集时刻机身坐标系的点转换到局部 odom 坐标系。
RollingObstaclePoint transformRollingPointToOdom(
  const RollingObstaclePoint & point,
  const TimedPose2D & pose);

// 将局部 odom 地图点转换到指定时刻的机身坐标系。
RollingObstaclePoint transformRollingPointToBase(
  const RollingObstaclePoint & point,
  const TimedPose2D & pose);

class OdomPoseBuffer
{
public:
  // 使用给定配置创建有界里程计位姿缓存。
  explicit OdomPoseBuffer(const RollingMapConfig & config);

  // 追加单调递增的里程计位姿，并在短时间大跳变时重置缓存。
  PoseAppendResult append(const TimedPose2D & pose);

  // 按时间戳插值位姿；仅在配置容差内允许首尾两端短时外推。
  bool lookup(std::int64_t stamp_ns, TimedPose2D * pose) const;

  // 清空全部历史位姿。
  void clear();

  // 返回当前缓存位姿数量。
  std::size_t size() const;

private:
  RollingMapConfig config_;
  std::deque<TimedPose2D> poses_;
};

class RollingObstacleMap
{
public:
  // 使用给定配置创建有时间衰减的二维滚动障碍地图。
  explicit RollingObstacleMap(const RollingMapConfig & config);

  // 把当前帧机身点转换到 odom，刷新对应体素并执行衰减和范围裁剪。
  void integrate(
    const std::vector<RollingObstaclePoint> & base_points,
    const TimedPose2D & pose);

  // 将当前保留的 odom 障碍转换到指定姿态的机身坐标系。
  std::vector<RollingObstaclePoint> pointsInBase(const TimedPose2D & pose) const;

  // 清空全部障碍体素。
  void clear();

  // 返回当前保留的障碍体素数量。
  std::size_t size() const;

private:
  struct CellKey
  {
    std::int64_t x{0};
    std::int64_t y{0};

    // 判断两个二维体素索引是否相同。
    bool operator==(const CellKey & other) const;
  };

  struct CellKeyHash
  {
    // 组合二维整数体素索引的哈希值。
    std::size_t operator()(const CellKey & key) const;
  };

  struct CellValue
  {
    RollingObstaclePoint point;
    std::int64_t last_seen_ns{0};
  };

  // 删除超过保留时间或滚动半径的障碍，并按新鲜度和距离限制总点数。
  void prune(const TimedPose2D & pose);

  RollingMapConfig config_;
  std::unordered_map<CellKey, CellValue, CellKeyHash> cells_;
};

}  // namespace go2_uwb_local_follow

#endif  // GO2_UWB_LOCAL_FOLLOW__ROLLING_OBSTACLE_MAP_CORE_HPP_
