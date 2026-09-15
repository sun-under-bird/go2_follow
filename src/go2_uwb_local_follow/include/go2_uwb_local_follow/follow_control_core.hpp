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

#ifndef GO2_UWB_LOCAL_FOLLOW__FOLLOW_CONTROL_CORE_HPP_
#define GO2_UWB_LOCAL_FOLLOW__FOLLOW_CONTROL_CORE_HPP_

#include <string>

namespace go2_uwb_local_follow
{

struct FollowConfig
{
  double follow_distance{1.0};
  double distance_deadband{0.08};
  double angle_deadband{0.20};
  double angle_reengage{0.45};
  double turn_response_delay{0.20};
  double angular_braking_accel{1.50};
  double angular_brake_release_speed{0.08};
  double angular_reverse_speed_threshold{0.15};
  double linear_kp{0.6};
  double angular_kp{1.0};
  double min_linear_speed{0.23};
  double max_linear_speed{0.80};
  double max_angular_speed{2.00};
  // 兼容旧配置保留；当前停止角以内不再按方位角平滑降低线速度。
  double heading_slowdown_start{1.39};
  double heading_stop_angle{1.40};
  // 对准后回落到更小偏角才恢复前进，避免角度门限附近反复停走。
  double heading_alignment_hysteresis{0.15};
  bool enable_avoidance_heading_relaxation{false};
  // 仅在规划器反馈安全前进绕障时放宽；始终小于 90 度，禁止向目标反方向前进。
  double avoidance_heading_stop_angle{1.48};
  double avoidance_heading_max_linear_speed{0.50};
  double blind_rotation_max_speed{2.00};
  double max_linear_accel{0.80};
  double max_linear_decel{0.80};
  double max_angular_accel{2.00};
};

struct Velocity2D
{
  double linear_x{0.0};
  double angular_z{0.0};
};

struct FollowResult
{
  Velocity2D target_velocity;
  double distance{0.0};
  double heading{0.0};
  double heading_scale{1.0};
  double actual_angular_z{0.0};
  double brake_angle{0.0};
  double dynamic_stop_angle{0.0};
  int turn_direction{0};
  bool within_follow_distance{false};
  bool blind_rotation{false};
  bool avoidance_heading_relaxed{false};
  bool angular_braking{false};
  bool angular_brake_latched{false};
};

struct DynamicAngularBrakeResult
{
  double angular_z{0.0};
  double brake_angle{0.0};
  double dynamic_stop_angle{0.0};
  int turn_direction{0};
  bool braking{false};
  bool brake_latched{false};
};

// 校验跟随控制和速度变化率参数之间的约束关系。
bool validateFollowConfig(const FollowConfig & config, std::string * reason = nullptr);

// 结合安全绕障反馈和对准滞回，计算距离及角度约束下的名义速度。
FollowResult computeFollowTarget(
  double target_x,
  double target_y,
  const FollowConfig & config,
  bool safe_forward_avoidance = false,
  bool previous_heading_alignment = false);

// 根据停止与重启角度门限更新带滞回的转向方向。
int updateTurnDirection(
  double heading,
  double stop_angle,
  double reengage_angle,
  int previous_direction);

// 根据实测角速度计算动态停止角，并在需要时优先撤销 UWB 名义转向。
DynamicAngularBrakeResult applyDynamicAngularBrake(
  double heading,
  double desired_angular_z,
  double actual_angular_z,
  const FollowConfig & config,
  int previous_direction,
  bool brake_latched = false);

// 按线加速、线减速和角加速度限制一个控制周期内的速度变化。
Velocity2D limitVelocityRate(
  const Velocity2D & previous,
  const Velocity2D & target,
  const FollowConfig & config,
  double dt);

}  // namespace go2_uwb_local_follow

#endif  // GO2_UWB_LOCAL_FOLLOW__FOLLOW_CONTROL_CORE_HPP_
