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

// 大角度只有在新鲜安全绕障许可下才能前进，速度受限且目标位于侧后方时仍停车。
TEST(FollowControl, RelaxesHeadingOnlyForSafeForwardAvoidance)
{
  follow::FollowConfig config;
  config.heading_stop_angle = 1.05;
  config.enable_avoidance_heading_relaxation = true;
  for (const double sign : {-1.0, 1.0}) {
    const double heading = sign * 1.22;
    const auto normal = follow::computeFollowTarget(
      3.0 * std::cos(heading), 3.0 * std::sin(heading), config);
    const auto avoiding = follow::computeFollowTarget(
      3.0 * std::cos(heading), 3.0 * std::sin(heading), config, true);
    EXPECT_TRUE(normal.blind_rotation);
    EXPECT_DOUBLE_EQ(normal.target_velocity.linear_x, 0.0);
    EXPECT_TRUE(avoiding.avoidance_heading_relaxed);
    EXPECT_GE(avoiding.target_velocity.linear_x, config.min_linear_speed);
    EXPECT_LE(avoiding.target_velocity.linear_x, config.avoidance_heading_max_linear_speed);
    const auto too_far_side = follow::computeFollowTarget(
      3.0 * std::cos(sign * 1.50), 3.0 * std::sin(sign * 1.50), config, true);
    EXPECT_TRUE(too_far_side.blind_rotation);
    EXPECT_DOUBLE_EQ(too_far_side.target_velocity.linear_x, 0.0);
    const auto near_owner = follow::computeFollowTarget(
      0.8 * std::cos(heading), 0.8 * std::sin(heading), config, true);
    EXPECT_TRUE(near_owner.within_follow_distance);
    EXPECT_DOUBLE_EQ(near_owner.target_velocity.linear_x, 0.0);
  }
  config.enable_avoidance_heading_relaxation = false;
  EXPECT_DOUBLE_EQ(
    follow::computeFollowTarget(1.0, 2.0, config, true).target_velocity.linear_x, 0.0);
}

// 进入对准后，小幅跨回停止角不能立即前进，必须回落越过释放角。
TEST(FollowControl, LatchesHeadingAlignmentAcrossThresholdJitter)
{
  follow::FollowConfig config;
  config.heading_stop_angle = 1.05;
  bool aligning = false;
  for (const double heading : {1.06, 1.04, 1.07, 0.95}) {
    const auto result = follow::computeFollowTarget(
      3.0 * std::cos(heading), 3.0 * std::sin(heading), config, false, aligning);
    aligning = result.blind_rotation;
    EXPECT_TRUE(aligning);
    EXPECT_DOUBLE_EQ(result.target_velocity.linear_x, 0.0);
  }
  const auto released = follow::computeFollowTarget(
    3.0 * std::cos(0.89), 3.0 * std::sin(0.89), config, false, aligning);
  EXPECT_FALSE(released.blind_rotation);
  EXPECT_GT(released.target_velocity.linear_x, 0.0);
  config.enable_avoidance_heading_relaxation = true;
  const auto delayed_permission = follow::computeFollowTarget(
    3.0 * std::cos(1.22), 3.0 * std::sin(1.22), config, true, true);
  EXPECT_TRUE(delayed_permission.blind_rotation);
  EXPECT_DOUBLE_EQ(delayed_permission.target_velocity.linear_x, 0.0);
}

// 拒绝放宽到目标后方或低于底盘执行死区的参数组合。
TEST(FollowControl, ValidatesAvoidanceHeadingLimits)
{
  follow::FollowConfig config;
  config.enable_avoidance_heading_relaxation = true;
  EXPECT_TRUE(follow::validateFollowConfig(config));
  config.avoidance_heading_stop_angle = 1.60;
  EXPECT_FALSE(follow::validateFollowConfig(config));
  config.avoidance_heading_stop_angle = 1.48;
  config.avoidance_heading_max_linear_speed = 0.10;
  EXPECT_FALSE(follow::validateFollowConfig(config));
}

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

// 验证同样距离下目标角度越大，线速度越低且角速度随角度误差增大。
TEST(FollowControl, AdjustsVelocityByDistanceAndHeading)
{
  follow::FollowConfig config;
  constexpr double distance = 4.0;
  const auto straight = follow::computeFollowTarget(distance, 0.0, config);
  const double heading = 0.80;
  const auto angled = follow::computeFollowTarget(
    distance * std::cos(heading), distance * std::sin(heading), config);

  EXPECT_DOUBLE_EQ(straight.target_velocity.linear_x, config.max_linear_speed);
  EXPECT_GT(angled.target_velocity.linear_x, 0.0);
  EXPECT_LT(angled.target_velocity.linear_x, straight.target_velocity.linear_x);
  EXPECT_GT(angled.target_velocity.angular_z, 0.0);
}

// 验证远距离大角度目标的线速度和角速度分别受新上限约束。
TEST(FollowControl, UsesExpandedSpeedLimits)
{
  follow::FollowConfig config;
  const auto straight = follow::computeFollowTarget(10.0, 0.0, config);
  const auto behind = follow::computeFollowTarget(-10.0, 0.0, config);

  EXPECT_DOUBLE_EQ(config.max_linear_speed, 0.80);
  EXPECT_DOUBLE_EQ(config.max_angular_speed, 2.00);
  EXPECT_DOUBLE_EQ(config.max_angular_accel, 2.00);
  EXPECT_DOUBLE_EQ(straight.target_velocity.linear_x, 0.80);
  EXPECT_DOUBLE_EQ(std::abs(behind.target_velocity.angular_z), 2.00);
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

// 验证动态停止角同时计入控制链路延迟和恒定角减速度制动距离。
TEST(DynamicAngularBrake, ComputesStopAngleFromMeasuredVelocity)
{
  follow::FollowConfig config;
  config.angle_deadband = 0.08;
  config.angle_reengage = 0.15;
  config.turn_response_delay = 0.10;
  config.angular_braking_accel = 1.50;

  const auto result = follow::applyDynamicAngularBrake(0.30, 0.22, 0.40, config, 1);

  EXPECT_NEAR(result.brake_angle, 0.09333333333333334, 1e-12);
  EXPECT_NEAR(result.dynamic_stop_angle, 0.17333333333333334, 1e-12);
  EXPECT_DOUBLE_EQ(result.angular_z, 0.22);
  EXPECT_FALSE(result.braking);
}

// 验证左转进入动态停止角后优先撤销名义角速度。
TEST(DynamicAngularBrake, BrakesLeftTurnBeforeKpFeedback)
{
  follow::FollowConfig config;
  config.angle_deadband = 0.08;
  config.angle_reengage = 0.15;

  const auto result = follow::applyDynamicAngularBrake(0.17, 0.20, 0.40, config, 1);

  EXPECT_EQ(result.turn_direction, 0);
  EXPECT_DOUBLE_EQ(result.angular_z, 0.0);
  EXPECT_TRUE(result.braking);
  EXPECT_GT(result.dynamic_stop_angle, 0.17);
}

// 验证右转使用与左转完全对称的动态提前刹车条件。
TEST(DynamicAngularBrake, BrakesRightTurnSymmetrically)
{
  follow::FollowConfig config;
  config.angle_deadband = 0.08;
  config.angle_reengage = 0.15;

  const auto result = follow::applyDynamicAngularBrake(-0.17, -0.20, -0.40, config, -1);

  EXPECT_EQ(result.turn_direction, 0);
  EXPECT_DOUBLE_EQ(result.angular_z, 0.0);
  EXPECT_TRUE(result.braking);
}

// 验证实测角速度未停稳时不允许直接下发相反方向的 UWB 名义速度。
TEST(DynamicAngularBrake, WaitsForMeasuredVelocityBeforeReversing)
{
  follow::FollowConfig config;
  config.angle_deadband = 0.08;
  config.angle_reengage = 0.15;

  const auto braking = follow::applyDynamicAngularBrake(-0.30, -0.22, 0.20, config, 0);
  const auto reversing = follow::applyDynamicAngularBrake(-0.30, -0.22, 0.10, config, 0);

  EXPECT_EQ(braking.turn_direction, 0);
  EXPECT_DOUBLE_EQ(braking.angular_z, 0.0);
  EXPECT_TRUE(braking.braking);
  EXPECT_EQ(reversing.turn_direction, -1);
  EXPECT_DOUBLE_EQ(reversing.angular_z, -0.22);
}

// 验证动态刹车触发后，即使停止角随速度下降也继续锁住零名义角速度。
TEST(DynamicAngularBrake, KeepsBrakeLatchedUntilMeasuredSpeedIsLow)
{
  follow::FollowConfig config;
  config.angle_deadband = 0.08;
  config.angle_reengage = 0.15;
  config.turn_response_delay = 0.20;
  config.angular_braking_accel = 0.50;
  config.angular_brake_release_speed = 0.06;

  const auto triggered = follow::applyDynamicAngularBrake(
    0.369, 0.289, 0.45, config, 1, false);
  const auto held = follow::applyDynamicAngularBrake(
    0.329, 0.249, 0.12, config, 0, triggered.brake_latched);
  const auto released = follow::applyDynamicAngularBrake(
    0.300, 0.220, 0.05, config, 0, held.brake_latched);

  EXPECT_TRUE(triggered.brake_latched);
  EXPECT_DOUBLE_EQ(triggered.angular_z, 0.0);
  EXPECT_TRUE(held.brake_latched);
  EXPECT_DOUBLE_EQ(held.angular_z, 0.0);
  EXPECT_FALSE(released.brake_latched);
  EXPECT_EQ(released.turn_direction, 1);
  EXPECT_DOUBLE_EQ(released.angular_z, 0.220);
}

// 验证实测角速度为零时动态停止角退化为原有基础角度死区。
TEST(DynamicAngularBrake, UsesBaseDeadbandAtRest)
{
  follow::FollowConfig config;
  config.angle_deadband = 0.08;
  config.angle_reengage = 0.15;

  const auto result = follow::applyDynamicAngularBrake(0.20, 0.12, 0.0, config, 0);

  EXPECT_DOUBLE_EQ(result.brake_angle, 0.0);
  EXPECT_DOUBLE_EQ(result.dynamic_stop_angle, config.angle_deadband);
  EXPECT_EQ(result.turn_direction, 1);
}

// 验证动态停止仍保留目标位于正后方时的 atan2 跨界转向方向。
TEST(DynamicAngularBrake, KeepsDirectionAcrossRearAngleWrap)
{
  follow::FollowConfig config;
  config.angle_deadband = 0.08;
  config.angle_reengage = 0.15;

  const auto left = follow::applyDynamicAngularBrake(-3.13, 0.50, 0.40, config, 1);
  const auto right = follow::applyDynamicAngularBrake(3.13, -0.50, -0.40, config, -1);

  EXPECT_EQ(left.turn_direction, 1);
  EXPECT_DOUBLE_EQ(left.angular_z, 0.50);
  EXPECT_EQ(right.turn_direction, -1);
  EXPECT_DOUBLE_EQ(right.angular_z, -0.50);
}

// 验证角速度受单周期变化率约束，非零线速度起步则直接跨过实机死区。
TEST(VelocityRate, LimitsAccelerationPerControlPeriod)
{
  follow::FollowConfig config;
  const follow::Velocity2D previous{0.0, 0.0};
  const follow::Velocity2D target{0.4, 1.0};
  const auto output = follow::limitVelocityRate(previous, target, config, 0.05);

  EXPECT_NEAR(output.linear_x, config.min_linear_speed, 1e-12);
  EXPECT_NEAR(output.angular_z, config.max_angular_accel * 0.05, 1e-12);
}

// 验证方位降速不会重新产生低于实机最小起步速度的非零 x 指令。
TEST(FollowControl, KeepsHeadingSlowedVelocityOutsideDeadzone)
{
  follow::FollowConfig config;
  const double heading = config.heading_stop_angle - 0.01;
  const auto result = follow::computeFollowTarget(
    2.0 * std::cos(heading), 2.0 * std::sin(heading), config);

  EXPECT_FALSE(result.blind_rotation);
  EXPECT_DOUBLE_EQ(result.target_velocity.linear_x, config.min_linear_speed);
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

// 验证非正角减速度会使动态刹车公式失效，因此必须拒绝该配置。
TEST(FollowConfig, RejectsInvalidAngularBrakingAcceleration)
{
  follow::FollowConfig config;
  config.angular_braking_accel = 0.0;
  std::string reason;

  EXPECT_FALSE(follow::validateFollowConfig(config, &reason));
  EXPECT_FALSE(reason.empty());
}

// 验证刹车释放速度不能高于反向保护速度，否则会提前解除停稳锁存。
TEST(FollowConfig, RejectsBrakeReleaseSpeedAboveReverseThreshold)
{
  follow::FollowConfig config;
  config.angular_brake_release_speed = config.angular_reverse_speed_threshold + 0.01;
  std::string reason;

  EXPECT_FALSE(follow::validateFollowConfig(config, &reason));
  EXPECT_FALSE(reason.empty());
}
