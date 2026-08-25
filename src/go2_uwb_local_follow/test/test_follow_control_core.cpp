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

#include <cmath>
#include <string>

#include "gtest/gtest.h"

#include "go2_uwb_local_follow/follow_control_core.hpp"

namespace follow = go2_uwb_local_follow;

// 验证正前方远目标产生非负前进速度且不产生角速度。
TEST(FollowControl, DrivesTowardStraightTarget)
{
  follow::FollowConfig config;
  const auto result = follow::computeFollowTarget(2.0, 0.0, config);

  EXPECT_GT(result.target_velocity.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(result.target_velocity.angular_z, 0.0);
  EXPECT_FALSE(result.within_follow_distance);
}

// 验证目标进入跟随距离后停止前进但仍保持横向朝向控制。
TEST(FollowControl, HoldsDistanceAndKeepsHeadingControl)
{
  follow::FollowConfig config;
  const auto result = follow::computeFollowTarget(0.8, 0.4, config);

  EXPECT_DOUBLE_EQ(result.target_velocity.linear_x, 0.0);
  EXPECT_GT(result.target_velocity.angular_z, 0.0);
  EXPECT_TRUE(result.within_follow_distance);
}

// 验证距离死区内停车，越过边界后使用最小有效跟随速度。
TEST(FollowControl, UsesMinimumEffectiveSpeedOutsideDistanceDeadband)
{
  follow::FollowConfig config;
  const auto boundary = follow::computeFollowTarget(1.08, 0.0, config);
  const auto outside = follow::computeFollowTarget(1.081, 0.0, config);

  EXPECT_NEAR(boundary.target_velocity.linear_x, 0.0, 1e-12);
  EXPECT_NEAR(outside.target_velocity.linear_x, config.min_linear_speed, 1e-12);
}

// 验证远目标产生的前进速度不超过最大跟随速度。
TEST(FollowControl, LimitsMaximumLinearSpeed)
{
  follow::FollowConfig config;
  const auto result = follow::computeFollowTarget(10.0, 0.0, config);

  EXPECT_NEAR(result.target_velocity.linear_x, config.max_linear_speed, 1e-12);
}

// 验证目标位于侧后方时禁止前进并限制为低速盲转。
TEST(FollowControl, LimitsBlindRotationWithoutReverse)
{
  follow::FollowConfig config;
  const auto result = follow::computeFollowTarget(-2.0, 0.2, config);

  EXPECT_TRUE(result.blind_rotation);
  EXPECT_DOUBLE_EQ(result.target_velocity.linear_x, 0.0);
  EXPECT_NE(result.target_velocity.angular_z, 0.0);
  EXPECT_NEAR(
    std::abs(result.target_velocity.angular_z), config.blind_rotation_max_speed, 1e-12);
  EXPECT_LE(std::abs(result.target_velocity.angular_z), config.blind_rotation_max_speed);
}

// 验证转向在小误差内停止，并且只有越过更大门限才反向重启。
TEST(TurnHysteresis, PreventsImmediateDirectionReversal)
{
  const follow::FollowConfig config;
  const double stop_angle = config.angle_deadband;
  const double reengage_angle = config.angle_reengage;

  int direction = follow::updateTurnDirection(0.50, stop_angle, reengage_angle, 0);
  EXPECT_EQ(direction, 1);
  direction = follow::updateTurnDirection(0.30, stop_angle, reengage_angle, direction);
  EXPECT_EQ(direction, 1);
  direction = follow::updateTurnDirection(0.15, stop_angle, reengage_angle, direction);
  EXPECT_EQ(direction, 0);
  direction = follow::updateTurnDirection(0.40, stop_angle, reengage_angle, direction);
  EXPECT_EQ(direction, 0);
  direction = follow::updateTurnDirection(-0.30, stop_angle, reengage_angle, direction);
  EXPECT_EQ(direction, 0);
  direction = follow::updateTurnDirection(-0.50, stop_angle, reengage_angle, direction);
  EXPECT_EQ(direction, -1);
}

// 验证目标在正后方附近跨越 atan2 分支时不会突然反转。
TEST(TurnHysteresis, KeepsDirectionAcrossRearAngleWrap)
{
  EXPECT_EQ(follow::updateTurnDirection(-3.13, 0.12, 0.30, 1), 1);
  EXPECT_EQ(follow::updateTurnDirection(3.13, 0.12, 0.30, -1), -1);
}

// 验证线加速和角加速度都受单周期变化率约束。
TEST(VelocityRate, LimitsAccelerationPerControlPeriod)
{
  follow::FollowConfig config;
  const follow::Velocity2D previous{0.0, 0.0};
  const follow::Velocity2D target{0.4, 1.0};
  const auto output = follow::limitVelocityRate(previous, target, config, 0.05);

  EXPECT_NEAR(output.linear_x, 0.040, 1e-12);
  EXPECT_NEAR(output.angular_z, 0.075, 1e-12);
}

// 验证减速使用独立的更高线减速度参数。
TEST(VelocityRate, UsesDedicatedDecelerationLimit)
{
  follow::FollowConfig config;
  const follow::Velocity2D previous{0.4, 0.0};
  const follow::Velocity2D target{0.0, 0.0};
  const auto output = follow::limitVelocityRate(previous, target, config, 0.05);

  EXPECT_NEAR(output.linear_x, 0.36, 1e-12);
}

// 验证颠倒的大角度降速边界会被参数校验拒绝。
TEST(FollowConfig, RejectsInvertedHeadingAngles)
{
  follow::FollowConfig config;
  config.heading_slowdown_start = 1.2;
  config.heading_stop_angle = 1.0;
  std::string reason;

  EXPECT_FALSE(follow::validateFollowConfig(config, &reason));
  EXPECT_FALSE(reason.empty());
}

// 验证最小线速度超过最大线速度时拒绝启动。
TEST(FollowConfig, RejectsMinimumLinearSpeedAboveMaximum)
{
  follow::FollowConfig config;
  config.min_linear_speed = config.max_linear_speed + 0.01;
  std::string reason;

  EXPECT_FALSE(follow::validateFollowConfig(config, &reason));
  EXPECT_FALSE(reason.empty());
}

// 验证转向重启门限小于停止门限时拒绝启动。
TEST(FollowConfig, RejectsInvertedAngularHysteresis)
{
  follow::FollowConfig config;
  config.angle_reengage = config.angle_deadband - 0.01;
  std::string reason;

  EXPECT_FALSE(follow::validateFollowConfig(config, &reason));
  EXPECT_FALSE(reason.empty());
}
