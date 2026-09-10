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

#ifndef GO2_UWB_BEHAVIOR__BEHAVIOR_CORE_HPP_
#define GO2_UWB_BEHAVIOR__BEHAVIOR_CORE_HPP_

#include <cstddef>
#include <deque>
#include <optional>
#include <random>
#include <string>
#include <vector>

namespace go2_uwb_behavior
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Velocity2D
{
  double linear_x{0.0};
  double angular_z{0.0};
};

struct OwnerFilterConfig
{
  std::size_t median_window{5U};
  double low_pass_alpha{0.25};
  double center_deadband{0.40};
  double center_move_confirm_sec{1.0};
  double center_max_speed{0.30};
};

struct RoamSamplingConfig
{
  double owner_keepout_radius{1.50};
  double random_goal_radius_max{4.50};
  double random_step_min{0.80};
  double random_step_max{1.80};
  double goal_obstacle_clearance{0.70};
  std::size_t max_sample_attempts{50U};
};

struct GeofenceConfig
{
  double return_trigger_radius{5.00};
  double return_release_radius{4.50};
  double restrictive_radius{5.60};
  double absolute_radius{6.00};
  double prediction_sec{1.00};
  double simulation_dt{0.05};
};

struct GeofenceResult
{
  bool allowed{true};
  double current_distance{0.0};
  double final_distance{0.0};
  double maximum_distance{0.0};
};

// 校验主人中心滤波参数，失败时返回具体原因。
bool validateOwnerFilterConfig(
  const OwnerFilterConfig & config, std::string * reason = nullptr);

// 校验随机目标采样参数及各半径之间的约束。
bool validateRoamSamplingConfig(
  const RoamSamplingConfig & config, std::string * reason = nullptr);

// 校验软返回、限制区和绝对围栏的顺序关系。
bool validateGeofenceConfig(
  const GeofenceConfig & config, std::string * reason = nullptr);

// 计算两个二维点之间的欧氏距离。
double distanceBetween(const Point2D & first, const Point2D & second);

// 将机身坐标点转换到连续 odom 坐标系。
Point2D transformBasePointToOdom(const Point2D & point, const Pose2D & robot_pose);

// 将 odom 坐标点转换到当前机身坐标系。
Point2D transformOdomPointToBase(const Point2D & point, const Pose2D & robot_pose);

class OwnerCenterFilter
{
public:
  // 初始化中值、低通和迟滞中心滤波器。
  explicit OwnerCenterFilter(OwnerFilterConfig config = OwnerFilterConfig{});

  // 清空历史样本和已确认的玩耍中心。
  void reset();

  // 输入一次新的主人 odom 坐标，并按真实采样间隔更新迟滞中心。
  bool update(const Point2D & measurement, double dt);

  // 返回是否已经产生有效的平滑位置。
  bool valid() const;

  // 返回当前滤波后的实时主人位置，用于半径安全判断。
  const Point2D & filteredOwner() const;

  // 返回带大死区和持续确认的玩耍中心，用于随机目标采样。
  const Point2D & playCenter() const;

  // 返回滤波窗口已接收的有效样本数量。
  std::size_t sampleCount() const;

private:
  OwnerFilterConfig config_;
  std::deque<Point2D> samples_;
  Point2D filtered_owner_;
  Point2D play_center_;
  double pending_move_sec_{0.0};
  bool valid_{false};
};

// 在主人圆环、单步距离及障碍净空的交集内采样一个目标。
std::optional<Point2D> sampleRandomGoal(
  const Point2D & owner_center,
  const Point2D & robot_position,
  const std::vector<Point2D> & obstacle_points,
  const RoamSamplingConfig & config,
  std::mt19937 & generator);

// 预测恒定速度下的短时轨迹，并判定是否违反主人半径围栏。
GeofenceResult evaluateGeofenceCommand(
  const Pose2D & robot_pose,
  const Point2D & owner_position,
  const Velocity2D & command,
  const GeofenceConfig & config);

// 判断里程计速度是否已经进入可确认停车的阈值。
bool isRobotStopped(
  const Velocity2D & measured,
  double linear_threshold,
  double angular_threshold);

class ProgressMonitor
{
public:
  // 配置有效进展距离和允许无进展的最长时间。
  ProgressMonitor(double required_progress, double window_sec);

  // 以新目标距离重置无进展计时窗口。
  void reset(double distance);

  // 更新目标距离，达到无进展窗口时返回 true。
  bool update(double distance, double dt);

private:
  double required_progress_{0.15};
  double window_sec_{3.0};
  double reference_distance_{0.0};
  double elapsed_sec_{0.0};
  bool initialized_{false};
};

}  // namespace go2_uwb_behavior

#endif  // GO2_UWB_BEHAVIOR__BEHAVIOR_CORE_HPP_
