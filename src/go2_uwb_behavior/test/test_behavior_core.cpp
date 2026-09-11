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
#include <random>
#include <vector>

#include "gtest/gtest.h"

#include "go2_uwb_behavior/behavior_core.hpp"

namespace behavior = go2_uwb_behavior;

// 验证机身与 odom 二维坐标转换互为逆变换。
TEST(BehaviorTransform, RoundTripsPoint)
{
  const behavior::Pose2D robot{1.0, -2.0, 0.7};
  const behavior::Point2D point_base{2.0, -0.4};
  const auto point_odom = behavior::transformBasePointToOdom(point_base, robot);
  const auto recovered = behavior::transformOdomPointToBase(point_odom, robot);
  EXPECT_NEAR(recovered.x, point_base.x, 1e-12);
  EXPECT_NEAR(recovered.y, point_base.y, 1e-12);
}

// 验证大死区能够吸收主人位置的小幅 UWB 抖动。
TEST(OwnerCenterFilter, IgnoresJitterInsideDeadband)
{
  behavior::OwnerFilterConfig config;
  config.median_window = 3U;
  config.low_pass_alpha = 1.0;
  behavior::OwnerCenterFilter filter(config);
  ASSERT_TRUE(filter.update({1.0, 2.0}, 0.1));
  ASSERT_TRUE(filter.update({1.2, 1.9}, 0.1));
  ASSERT_TRUE(filter.update({0.8, 2.1}, 0.1));
  EXPECT_NEAR(filter.playCenter().x, 1.0, 1e-12);
  EXPECT_NEAR(filter.playCenter().y, 2.0, 1e-12);
}

// 验证主人持续移动超过确认时间后，玩耍中心按速度上限缓慢跟随。
TEST(OwnerCenterFilter, MovesCenterAfterPersistentOffset)
{
  behavior::OwnerFilterConfig config;
  config.median_window = 1U;
  config.low_pass_alpha = 1.0;
  config.center_deadband = 0.4;
  config.center_move_confirm_sec = 1.0;
  config.center_max_speed = 0.3;
  behavior::OwnerCenterFilter filter(config);
  ASSERT_TRUE(filter.update({0.0, 0.0}, 0.0));
  for (int index = 0; index < 4; ++index) {
    ASSERT_TRUE(filter.update({2.0, 0.0}, 0.25));
  }
  EXPECT_NEAR(filter.playCenter().x, 0.075, 1e-12);
  EXPECT_NEAR(filter.playCenter().y, 0.0, 1e-12);
}

// 验证固定随机种子产生可复现且满足圆环与单步范围的目标。
TEST(RandomGoalSampling, IsDeterministicAndWithinBounds)
{
  behavior::RoamSamplingConfig config;
  config.owner_keepout_radius = 0.8;
  config.random_goal_radius_max = 1.8;
  config.random_step_min = 0.8;
  config.random_step_max = 1.8;
  std::mt19937 first_generator(42U);
  std::mt19937 second_generator(42U);
  const behavior::Point2D owner{0.0, 0.0};
  const behavior::Point2D robot{0.0, 0.0};
  const auto first = behavior::sampleRandomGoal(
    owner, robot, {}, config, first_generator);
  const auto second = behavior::sampleRandomGoal(
    owner, robot, {}, config, second_generator);
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_DOUBLE_EQ(first->x, second->x);
  EXPECT_DOUBLE_EQ(first->y, second->y);
  EXPECT_GE(behavior::distanceBetween(*first, owner), config.owner_keepout_radius);
  EXPECT_LE(behavior::distanceBetween(*first, owner), config.random_goal_radius_max);
}

// 验证所有候选都进入障碍净空时返回无有效目标。
TEST(RandomGoalSampling, RejectsObstacleCoveredArea)
{
  behavior::RoamSamplingConfig config;
  config.owner_keepout_radius = 0.5;
  config.random_goal_radius_max = 1.0;
  config.random_step_min = 0.5;
  config.random_step_max = 1.0;
  config.goal_obstacle_clearance = 10.0;
  std::mt19937 generator(7U);
  const auto goal = behavior::sampleRandomGoal(
    {0.0, 0.0}, {0.0, 0.0}, {{0.0, 0.0}}, config, generator);
  EXPECT_FALSE(goal.has_value());
}

// 验证限制区内向外速度被拒绝，朝向主人运动仍可放行。
TEST(Geofence, RejectsOutwardAndAllowsInwardCommand)
{
  behavior::GeofenceConfig config;
  const behavior::Point2D owner{0.0, 0.0};
  const auto outward = behavior::evaluateGeofenceCommand(
    {5.7, 0.0, 0.0}, owner, {0.35, 0.0}, config);
  const auto inward = behavior::evaluateGeofenceCommand(
    {5.7, 0.0, 3.14159265358979323846}, owner, {0.35, 0.0}, config);
  EXPECT_FALSE(outward.allowed);
  EXPECT_TRUE(inward.allowed);
  EXPECT_LT(inward.final_distance, inward.current_distance);
}

// 验证绝对边界内预测穿越 6 米的速度被拒绝。
TEST(Geofence, RejectsPredictedAbsoluteBoundaryCrossing)
{
  behavior::GeofenceConfig config;
  const auto result = behavior::evaluateGeofenceCommand(
    {5.9, 0.0, 0.0}, {0.0, 0.0}, {0.35, 0.0}, config);
  EXPECT_FALSE(result.allowed);
  EXPECT_GT(result.maximum_distance, config.absolute_radius);
}

// 验证每次取得足够进展会重置计时，无进展达到窗口后触发受阻。
TEST(ProgressMonitor, DetectsOnlyPersistentLackOfProgress)
{
  behavior::ProgressMonitor monitor(0.15, 3.0);
  monitor.reset(2.0);
  EXPECT_FALSE(monitor.update(1.8, 2.0));
  EXPECT_FALSE(monitor.update(1.79, 2.9));
  EXPECT_TRUE(monitor.update(1.79, 0.1));
}

// 验证线速度和角速度必须同时低于阈值才认为机器人停稳。
TEST(StopDetection, RequiresBothVelocityComponents)
{
  EXPECT_TRUE(behavior::isRobotStopped({0.03, 0.07}, 0.04, 0.08));
  EXPECT_FALSE(behavior::isRobotStopped({0.05, 0.07}, 0.04, 0.08));
  EXPECT_FALSE(behavior::isRobotStopped({0.03, 0.09}, 0.04, 0.08));
}
