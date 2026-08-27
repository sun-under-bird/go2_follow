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
  double linear_kp{0.6};
  double angular_kp{1.0};
  double min_linear_speed{0.12};
  double max_linear_speed{0.80};
  double max_angular_speed{2.00};
  double heading_slowdown_start{0.50};
  double heading_stop_angle{1.05};
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
  int turn_direction{0};
  bool within_follow_distance{false};
  bool blind_rotation{false};
};

// 校验跟随控制和速度变化率参数之间的约束关系。
bool validateFollowConfig(const FollowConfig & config, std::string * reason = nullptr);

// 根据当前机器人坐标系目标点计算最小有效速度和转向降速后的名义速度。
FollowResult computeFollowTarget(
  double target_x,
  double target_y,
  const FollowConfig & config);

// 根据停止与重启角度门限更新带滞回的转向方向。
int updateTurnDirection(
  double heading,
  double stop_angle,
  double reengage_angle,
  int previous_direction);

// 按线加速、线减速和角加速度限制一个控制周期内的速度变化。
Velocity2D limitVelocityRate(
  const Velocity2D & previous,
  const Velocity2D & target,
  const FollowConfig & config,
  double dt);

}  // namespace go2_uwb_local_follow

#endif  // GO2_UWB_LOCAL_FOLLOW__FOLLOW_CONTROL_CORE_HPP_
