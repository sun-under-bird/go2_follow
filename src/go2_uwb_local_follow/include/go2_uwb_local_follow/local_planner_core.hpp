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

#ifndef GO2_UWB_LOCAL_FOLLOW__LOCAL_PLANNER_CORE_HPP_
#define GO2_UWB_LOCAL_FOLLOW__LOCAL_PLANNER_CORE_HPP_

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace go2_uwb_local_follow
{

struct PlannerVelocity2D
{
  double linear_x{0.0};
  double angular_z{0.0};
};

struct PlannerPose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct ObstaclePoint2D
{
  double x{0.0};
  double y{0.0};
};

struct TrajectoryConfig
{
  double prediction_time{1.20};
  double simulation_dt{0.05};
};

struct FootprintConfig
{
  double robot_length{0.70};
  double robot_width{0.40};
  double safety_margin{0.08};
};

struct CollisionResult
{
  bool collision{false};
  double min_clearance{std::numeric_limits<double>::infinity()};
  std::size_t collision_pose_index{0U};
};

struct MotionLimits
{
  double min_linear_speed{0.12};
  double max_linear_speed{0.80};
  double min_angular_speed{0.0};
  double max_angular_speed{1.20};
  double max_linear_accel{0.80};
  double max_linear_decel{0.80};
  double max_angular_accel{1.50};
};

struct AngularStabilizationConfig
{
  // 相当于角速度反馈的时间常数：名义角速度减去 gain * 实测角速度。
  double velocity_damping_gain{0.35};
  // 阻尼后的角速度低于该值时提前撤销转向，避免再被放大到执行死区。
  double command_deadband{0.08};
  // 请求反向时，先等待当前实测角速度降到该值以下。
  double reverse_speed_threshold{0.15};
};

struct VelocitySamplingConfig
{
  int linear_samples{9};
  int angular_samples{25};
  // 避障候选的最小非零角速度；UWB 名义角速度不受该门槛影响。
  double min_avoidance_angular_speed{0.25};
  // 按顺序尝试的 UWB 线速度比例，当前层存在安全轨迹时不再继续减速。
  std::vector<double> linear_speed_priority_scales{1.0, 0.85, 0.70, 0.50, 0.0};
  double obstacle_influence_distance{0.35};
  double weight_follow_linear{8.0};
  double weight_follow_angular{12.0};
  double weight_smooth_linear{4.0};
  double weight_smooth_angular{8.0};
  double weight_obstacle{6.0};
  double weight_progress{0.80};
};

struct PlannerCost
{
  double follow{0.0};
  double smooth{0.0};
  double obstacle{0.0};
  double progress{0.0};
  double total{std::numeric_limits<double>::infinity()};
};

struct LocalPlanResult
{
  bool valid{false};
  bool avoidance_active{false};
  PlannerVelocity2D effective_nominal;
  PlannerVelocity2D selected_velocity;
  std::vector<PlannerPose2D> selected_trajectory;
  PlannerCost cost;
  double min_clearance{std::numeric_limits<double>::infinity()};
  std::size_t evaluated_count{0U};
  std::size_t collision_count{0U};
  double selected_speed_scale{0.0};
};

// 校验轨迹预测的时域和积分步长。
bool validateTrajectoryConfig(
  const TrajectoryConfig & config,
  std::string * reason = nullptr);

// 校验矩形机器人尺寸和安全膨胀参数。
bool validateFootprintConfig(
  const FootprintConfig & config,
  std::string * reason = nullptr);

// 校验速度死区、运动学限制和加减速度参数。
bool validateMotionLimits(
  const MotionLimits & limits,
  std::string * reason = nullptr);

// 校验角速度反馈阻尼、提前制动死区和安全换向阈值。
bool validateAngularStabilizationConfig(
  const AngularStabilizationConfig & config,
  std::string * reason = nullptr);

// 校验速度采样数量、障碍影响距离和各项代价权重。
bool validateVelocitySamplingConfig(
  const VelocitySamplingConfig & config,
  std::string * reason = nullptr);

// 从当前原点使用单轮车模型预测局部轨迹，返回值包含初始姿态。
std::vector<PlannerPose2D> predictTrajectory(
  const PlannerVelocity2D & velocity,
  const TrajectoryConfig & config);

// 从实测初始速度向候选速度加减速展开轨迹，并在时域末尾追加完整制动尾段。
std::vector<PlannerPose2D> predictAcceleratingTrajectory(
  const PlannerVelocity2D & initial_velocity,
  const PlannerVelocity2D & target_velocity,
  const TrajectoryConfig & config,
  const MotionLimits & limits,
  bool append_braking_tail = true);

// 判断障碍点是否落入指定姿态下旋转后的矩形足迹。
bool pointInsideFootprint(
  const PlannerPose2D & pose,
  const ObstaclePoint2D & obstacle,
  const FootprintConfig & footprint);

// 计算障碍点到指定姿态矩形足迹边界的二维最短距离。
double pointToFootprintClearance(
  const PlannerPose2D & pose,
  const ObstaclePoint2D & obstacle,
  const FootprintConfig & footprint);

// 沿整条预测轨迹检查旋转矩形碰撞并统计最小净空。
CollisionResult checkTrajectoryCollision(
  const std::vector<PlannerPose2D> & poses,
  const std::vector<ObstaclePoint2D> & obstacles,
  const FootprintConfig & footprint);

// 检查障碍是否进入机器人正前方的紧急停车矩形区域。
bool hasEmergencyFrontObstacle(
  const std::vector<ObstaclePoint2D> & obstacles,
  const FootprintConfig & footprint,
  double emergency_front_distance,
  double emergency_half_width);

// 将候选速度限制到运动范围，并跨过 Go2 无法执行的最小非零速度区间。
PlannerVelocity2D makeEffectiveVelocity(
  const PlannerVelocity2D & velocity,
  const MotionLimits & limits);

// 使用实测角速度修正 UWB 名义转向；接近目标朝向时提前制动，换向前先经过低速。
PlannerVelocity2D stabilizeNominalAngularVelocity(
  const PlannerVelocity2D & nominal_velocity,
  const PlannerVelocity2D & measured_velocity,
  const AngularStabilizationConfig & config);

// 生成兼容调试使用的全局速度候选集合。
std::vector<PlannerVelocity2D> sampleCandidateVelocities(
  const PlannerVelocity2D & nominal_velocity,
  const PlannerVelocity2D & previous_command,
  const MotionLimits & limits,
  const VelocitySamplingConfig & config,
  bool force_linear_stop = false);

// 计算安全候选的跟随、平滑、净空和前进代价。
PlannerCost scoreVelocityCandidate(
  const PlannerVelocity2D & candidate,
  const PlannerVelocity2D & effective_nominal,
  const PlannerVelocity2D & previous_command,
  double min_clearance,
  const MotionLimits & limits,
  const VelocitySamplingConfig & config);

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
  bool force_linear_stop = false);

// 按控制周期限制最终指令变化，并对非零指令跨过底盘执行死区。
PlannerVelocity2D limitCommandVelocity(
  const PlannerVelocity2D & previous_command,
  const PlannerVelocity2D & target_velocity,
  const MotionLimits & limits,
  double dt);

}  // namespace go2_uwb_local_follow

#endif  // GO2_UWB_LOCAL_FOLLOW__LOCAL_PLANNER_CORE_HPP_
