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
#include <limits>
#include <random>
#include <stdexcept>
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

// 验证新接口直接以冻结的 UWB 中心为圆心，并遵守调用方给定的半径范围。
TEST(RandomGoalSampling, SamplesCallerSpecifiedFrozenCenterAnnulus)
{
  std::mt19937 generator(2026U);
  const behavior::Point2D center{3.0, -2.0};
  for (int index = 0; index < 100; ++index) {
    const auto goal = behavior::sampleRandomGoalInAnnulus(
      center, {}, 1.5, 3.0, 0.7, 50U, generator);
    ASSERT_TRUE(goal.has_value());
    const double radius = behavior::distanceBetween(*goal, center);
    EXPECT_GE(radius, 1.5);
    EXPECT_LE(radius, 3.0);
  }
}

// 验证指定圆环被障碍净空完全覆盖时不会返回危险目标。
TEST(RandomGoalSampling, CallerAnnulusRejectsObstacleCoveredArea)
{
  std::mt19937 generator(8U);
  const auto goal = behavior::sampleRandomGoalInAnnulus(
    {0.0, 0.0}, {{0.0, 0.0}}, 1.5, 3.0, 10.0, 50U, generator);
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

// 用实机数据回归"静止但 twist 带偏置"这个故障：2026-09-22 实机漫游卡在等停稳期间，
// /leg_odom2 的 675 个样本里位姿 x/y 逐位不变，而 twist 恒为
// linear=(-0.06925739115715755, -0.06829377979399552)、angular.z=0.0007234040531329811
// （节点按 hypot 折算成 0.0973 m/s），同一批样本的 yaw 极差仅 6.6e-4 rad。
// 下面这段循环复刻 updateStopped 的两条证据组合，钉住"只认 twist 会永远卡住"这一前提。
TEST(StopDetection, StandingRobotWithTwistBiasConfirmsStopByPose)
{
  const double measured_linear_x = -0.06925739115715755;
  const double measured_linear_y = -0.06829377979399552;
  const double measured_angular_z = 0.0007234040531329811;
  const behavior::Velocity2D folded{
    std::hypot(measured_linear_x, measured_linear_y), measured_angular_z};
  const behavior::Pose2D frozen{28.40139764534411, -10.634735686318578, -1.326983241};

  behavior::PoseStationarityMonitor monitor(0.02, 0.03);
  const double dt = 0.05;  // control_frequency 20 Hz
  double velocity_only_sec = 0.0;
  double combined_sec = 0.0;
  int confirm_tick = -1;
  for (int tick = 0; tick < 600; ++tick) {  // 跑满默认 30 s 漫游总超时
    const bool pose_ok = monitor.update(frozen);
    const bool velocity_ok = behavior::isRobotStopped(folded, 0.04, 0.08);
    velocity_only_sec = velocity_ok ? velocity_only_sec + dt : 0.0;
    combined_sec = (velocity_ok || pose_ok) ? combined_sec + dt : 0.0;
    if (confirm_tick < 0 && combined_sec >= 0.30) {
      confirm_tick = tick;
    }
  }
  // 旧逻辑（只认 twist）整整 30 秒一次都没确认停车，与实机卡死表现一致。
  EXPECT_DOUBLE_EQ(velocity_only_sec, 0.0);
  // 新逻辑（位姿不变也算停车）在确认窗口内完成确认，不再拖到总超时。
  ASSERT_GE(confirm_tick, 0);
  EXPECT_LE(confirm_tick * dt, 0.35);
}

// 位姿判据的容差必须远大于实机静止时的真实抖动，否则会退化成"永远判为运动"。
// 实测静止 675 样本的 yaw 极差 6.21e-4 rad、x/y 逐位不变。
TEST(PoseStationarity, RealStandingDriftStaysWithinEpsilon)
{
  behavior::PoseStationarityMonitor monitor(0.02, 0.03);
  const behavior::Pose2D frozen{28.40139764534411, -10.634735686318578, -1.326983241};
  EXPECT_FALSE(monitor.update(frozen));
  for (int i = 0; i < 675; ++i) {
    behavior::Pose2D drifted = frozen;
    drifted.yaw += 6.21e-4 * static_cast<double>(i) / 675.0;
    EXPECT_TRUE(monitor.update(drifted));
  }
}

// 验证位姿判据第一帧只建立基准，不直接宣称停稳。
TEST(PoseStationarity, FirstSampleOnlySetsReference)
{
  behavior::PoseStationarityMonitor monitor(0.02, 0.03);
  EXPECT_FALSE(monitor.update({1.0, 2.0, 0.5}));
  EXPECT_TRUE(monitor.update({1.0, 2.0, 0.5}));
}

// 验证位姿冻结（含小数点后第 15 位都相同）始终判定为静止，这正是实机静止时的表现。
TEST(PoseStationarity, FrozenPoseCountsAsStationary)
{
  behavior::PoseStationarityMonitor monitor(0.02, 0.03);
  const behavior::Pose2D frozen{28.40139764534411, -10.634735686318578, 0.3};
  EXPECT_FALSE(monitor.update(frozen));
  for (int i = 0; i < 20; ++i) {
    EXPECT_TRUE(monitor.update(frozen));
  }
}

// 验证平移超过容差时判为运动，并刷新参考点，随后从新位置重新确认。
TEST(PoseStationarity, TranslationBeyondEpsilonRefreshesReference)
{
  behavior::PoseStationarityMonitor monitor(0.02, 0.03);
  EXPECT_FALSE(monitor.update({0.0, 0.0, 0.0}));
  EXPECT_FALSE(monitor.update({0.05, 0.0, 0.0}));
  EXPECT_TRUE(monitor.update({0.05, 0.0, 0.0}));
  // 容差以内的抖动不刷新参考点，仍有资格确认停车。
  EXPECT_TRUE(monitor.update({0.059, 0.0, 0.0}));
}

// 验证偏航超过容差时判为运动，原地转向不能算停稳。
TEST(PoseStationarity, YawBeyondEpsilonCountsAsMoving)
{
  behavior::PoseStationarityMonitor monitor(0.02, 0.03);
  EXPECT_FALSE(monitor.update({0.0, 0.0, 0.0}));
  EXPECT_TRUE(monitor.update({0.0, 0.0, 0.01}));
  EXPECT_FALSE(monitor.update({0.0, 0.0, 0.05}));
  EXPECT_TRUE(monitor.update({0.0, 0.0, 0.05}));
}

// 验证跨 ±pi 的小幅偏航按最短角差判定，不因换台数被误判成大幅转向。
TEST(PoseStationarity, WrapsAcrossPiBoundary)
{
  behavior::PoseStationarityMonitor monitor(0.02, 0.03);
  EXPECT_FALSE(monitor.update({0.0, 0.0, 3.13}));
  EXPECT_TRUE(monitor.update({0.0, 0.0, -3.13}));
  EXPECT_FALSE(monitor.update({0.0, 0.0, 0.0}));
}

// 验证 reset 后重新建立基准，避免用过期参考点宣称停稳。
TEST(PoseStationarity, ResetRebuildsReference)
{
  behavior::PoseStationarityMonitor monitor(0.02, 0.03);
  EXPECT_FALSE(monitor.update({0.0, 0.0, 0.0}));
  EXPECT_TRUE(monitor.update({0.0, 0.0, 0.0}));
  monitor.reset();
  EXPECT_FALSE(monitor.update({3.0, 4.0, 1.0}));
  EXPECT_TRUE(monitor.update({3.0, 4.0, 1.0}));
}

// 验证非有限位姿不会被当成静止，宁可回到"未确认停车"。
TEST(PoseStationarity, NonFinitePoseNeverConfirmsStandstill)
{
  behavior::PoseStationarityMonitor monitor(0.02, 0.03);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(monitor.update({0.0, 0.0, 0.0}));
  EXPECT_FALSE(monitor.update({nan, 0.0, 0.0}));
  EXPECT_FALSE(monitor.update({nan, 0.0, 0.0}));
}

// 验证容差必须为正，配置错误在构造期立即暴露。
TEST(PoseStationarity, RejectsNonPositiveEpsilon)
{
  EXPECT_THROW(behavior::PoseStationarityMonitor(0.0, 0.03), std::invalid_argument);
  EXPECT_THROW(behavior::PoseStationarityMonitor(0.02, -0.01), std::invalid_argument);
}
