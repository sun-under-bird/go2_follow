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
#include <vector>

#include "gtest/gtest.h"

#include "go2_uwb_local_follow/local_planner_core.hpp"

namespace planner = go2_uwb_local_follow;

// 验证直行预测轨迹保持 y 和 yaw 为零并达到预期距离。
TEST(TrajectoryPredictor, PredictsStraightMotion)
{
  const planner::TrajectoryConfig config{1.0, 0.1};
  const auto poses = planner::predictTrajectory({0.5, 0.0}, config);

  ASSERT_EQ(poses.size(), 11U);
  EXPECT_NEAR(poses.back().x, 0.5, 1e-12);
  EXPECT_NEAR(poses.back().y, 0.0, 1e-12);
  EXPECT_NEAR(poses.back().yaw, 0.0, 1e-12);
}

// 验证正角速度生成向左弯曲且 yaw 增大的轨迹。
TEST(TrajectoryPredictor, PredictsLeftTurn)
{
  const planner::TrajectoryConfig config{1.0, 0.05};
  const auto poses = planner::predictTrajectory({0.3, 0.8}, config);

  EXPECT_GT(poses.back().x, 0.0);
  EXPECT_GT(poses.back().y, 0.0);
  EXPECT_NEAR(poses.back().yaw, 0.8, 1e-12);
}

// 验证原地旋转只改变朝向，不产生平移漂移。
TEST(TrajectoryPredictor, PredictsPureRotation)
{
  const planner::TrajectoryConfig config{1.0, 0.05};
  const auto poses = planner::predictTrajectory({0.0, -0.9}, config);

  EXPECT_NEAR(poses.back().x, 0.0, 1e-12);
  EXPECT_NEAR(poses.back().y, 0.0, 1e-12);
  EXPECT_NEAR(poses.back().yaw, -0.9, 1e-12);
}

// 验证直行轨迹会与机器人正前方障碍发生碰撞。
TEST(CollisionChecker, DetectsStraightCollision)
{
  const planner::TrajectoryConfig trajectory_config{1.2, 0.05};
  const planner::FootprintConfig footprint{0.70, 0.40, 0.08};
  const auto poses = planner::predictTrajectory({0.5, 0.0}, trajectory_config);
  const std::vector<planner::ObstaclePoint2D> obstacles{{0.75, 0.0}};
  const auto result = planner::checkTrajectoryCollision(poses, obstacles, footprint);

  EXPECT_TRUE(result.collision);
  EXPECT_DOUBLE_EQ(result.min_clearance, 0.0);
  EXPECT_GT(result.collision_pose_index, 0U);
}

// 验证足迹外侧的障碍不会误报为直行碰撞。
TEST(CollisionChecker, KeepsLateralObstacleClear)
{
  const planner::TrajectoryConfig trajectory_config{1.0, 0.05};
  const planner::FootprintConfig footprint{0.70, 0.40, 0.08};
  const auto poses = planner::predictTrajectory({0.3, 0.0}, trajectory_config);
  const std::vector<planner::ObstaclePoint2D> obstacles{{0.5, 0.8}};
  const auto result = planner::checkTrajectoryCollision(poses, obstacles, footprint);

  EXPECT_FALSE(result.collision);
  EXPECT_GT(result.min_clearance, 0.0);
}

// 验证原地旋转时使用旋转矩形足迹检测侧向扫掠碰撞。
TEST(CollisionChecker, DetectsRotatingFootprintSweep)
{
  const planner::TrajectoryConfig trajectory_config{1.8, 0.02};
  const planner::FootprintConfig footprint{0.70, 0.30, 0.02};
  const auto poses = planner::predictTrajectory({0.0, 0.9}, trajectory_config);
  const std::vector<planner::ObstaclePoint2D> obstacles{{0.0, 0.32}};
  const auto result = planner::checkTrajectoryCollision(poses, obstacles, footprint);

  EXPECT_TRUE(result.collision);
  EXPECT_GT(result.collision_pose_index, 0U);
}

// 验证前方紧急停车区域从膨胀后的机器人前缘开始计算。
TEST(EmergencyRegion, DetectsOnlyFrontCorridor)
{
  const planner::FootprintConfig footprint{0.70, 0.40, 0.08};

  EXPECT_TRUE(planner::hasEmergencyFrontObstacle({{0.50, 0.10}}, footprint, 0.25, 0.30));
  EXPECT_FALSE(planner::hasEmergencyFrontObstacle({{0.50, 0.40}}, footprint, 0.25, 0.30));
  EXPECT_FALSE(planner::hasEmergencyFrontObstacle({{-0.50, 0.0}}, footprint, 0.25, 0.30));
}

// 验证前方急停障碍不会阻止直线远离，且恢复轨迹严格受最大倒退距离限制。
TEST(EmergencyReverse, AllowsLimitedReverseAwayFromFrontObstacle)
{
  const planner::TrajectoryConfig trajectory_config{1.50, 0.05};
  const planner::FootprintConfig footprint{0.70, 0.38, 0.02};
  const planner::MotionLimits limits;
  const planner::EmergencyReverseConfig reverse_config;
  const auto result = planner::planEmergencyReverse(
    {{0.40, 0.0}}, trajectory_config, footprint, limits, reverse_config,
    reverse_config.distance);

  ASSERT_TRUE(result.valid);
  ASSERT_FALSE(result.selected_trajectory.empty());
  EXPECT_DOUBLE_EQ(result.selected_velocity.linear_x, -reverse_config.speed);
  EXPECT_NEAR(result.selected_trajectory.back().x, -reverse_config.distance, 1e-9);
  EXPECT_NEAR(result.selected_trajectory.back().y, 0.0, 1e-12);
}

// 验证后方障碍进入带额外安全边界的扫掠足迹时禁止急停倒退。
TEST(EmergencyReverse, RejectsUnsafeRearSweep)
{
  const planner::TrajectoryConfig trajectory_config{1.50, 0.05};
  const planner::FootprintConfig footprint{0.70, 0.38, 0.02};
  const planner::MotionLimits limits;
  const planner::EmergencyReverseConfig reverse_config;
  const auto result = planner::planEmergencyReverse(
    {{0.40, 0.0}, {-0.55, 0.0}}, trajectory_config, footprint, limits,
    reverse_config, reverse_config.distance);

  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.collision.collision);
}

// 验证急停区域仍占用且未达到距离上限时继续后退。
TEST(EmergencyReverse, ContinuesWhileEmergencyZoneOccupied)
{
  const planner::EmergencyReverseConfig config;
  const auto progress = planner::evaluateEmergencyReverseProgress(true, config, 0.20);

  EXPECT_TRUE(progress.should_reverse);
  EXPECT_FALSE(progress.zone_cleared);
  EXPECT_FALSE(progress.distance_limit_reached);
  EXPECT_NEAR(progress.commanded_distance, config.speed * 0.20, 1e-12);
}

// 验证障碍退出急停区域后立即结束倒退，不必耗尽距离预算。
TEST(EmergencyReverse, StopsWhenEmergencyZoneClears)
{
  const planner::EmergencyReverseConfig config;
  const auto progress = planner::evaluateEmergencyReverseProgress(false, config, 0.20);

  EXPECT_FALSE(progress.should_reverse);
  EXPECT_TRUE(progress.zone_cleared);
  EXPECT_FALSE(progress.distance_limit_reached);
}

// 验证急停区域一直不清空时由最大命令距离强制停车。
TEST(EmergencyReverse, StopsAtDistanceSafetyLimit)
{
  const planner::EmergencyReverseConfig config;
  const double elapsed = config.distance / config.speed;
  const auto progress = planner::evaluateEmergencyReverseProgress(true, config, elapsed);

  EXPECT_FALSE(progress.should_reverse);
  EXPECT_FALSE(progress.zone_cleared);
  EXPECT_TRUE(progress.distance_limit_reached);
  EXPECT_DOUBLE_EQ(progress.remaining_distance, 0.0);
}

// 验证非法轨迹积分参数会被拒绝。
TEST(PlannerConfig, RejectsInvalidTrajectoryStep)
{
  planner::TrajectoryConfig config;
  config.simulation_dt = config.prediction_time + 0.1;
  std::string reason;

  EXPECT_FALSE(planner::validateTrajectoryConfig(config, &reason));
  EXPECT_FALSE(reason.empty());
}

// 验证非法机器人尺寸会被拒绝。
TEST(PlannerConfig, RejectsInvalidFootprint)
{
  planner::FootprintConfig config;
  config.robot_width = 0.0;
  std::string reason;

  EXPECT_FALSE(planner::validateFootprintConfig(config, &reason));
  EXPECT_FALSE(reason.empty());
}

// 验证显式配置角速度死区时，非零候选会跨过底盘执行门槛。
TEST(VelocitySampling, AppliesEffectiveSpeedThresholds)
{
  planner::MotionLimits limits;
  limits.min_angular_speed = 0.82;
  const auto effective = planner::makeEffectiveVelocity({0.04, -0.20}, limits);

  EXPECT_DOUBLE_EQ(effective.linear_x, limits.min_linear_speed);
  EXPECT_DOUBLE_EQ(effective.angular_z, -limits.min_angular_speed);
}

// 验证恢复专用负速度同样跨过执行死区，但不超过最大倒退速度。
TEST(VelocitySampling, AppliesReverseSpeedLimits)
{
  const planner::MotionLimits limits;
  const auto effective = planner::makeEffectiveVelocity({-0.04, 0.0}, limits);
  const auto clamped = planner::makeEffectiveVelocity({-0.50, 0.0}, limits);

  EXPECT_DOUBLE_EQ(effective.linear_x, -limits.min_linear_speed);
  EXPECT_DOUBLE_EQ(clamped.linear_x, -limits.max_reverse_speed);
}

// 验证默认不设最小角速度，小角速度不会被强制放大。
TEST(VelocitySampling, PreservesSmallAngularVelocityByDefault)
{
  const planner::MotionLimits limits;
  const auto effective = planner::makeEffectiveVelocity({0.20, 0.085}, limits);

  EXPECT_DOUBLE_EQ(effective.angular_z, 0.085);
}

// 验证实际角速度低于名义值时，P 反馈按角速度误差提高下发目标。
TEST(AngularVelocityTracking, BoostsCommandWhenMeasuredSpeedIsLow)
{
  planner::AngularStabilizationConfig config;
  config.velocity_tracking_kp = 1.5;
  const planner::MotionLimits limits;
  const auto corrected = planner::correctNominalAngularVelocity(
    {0.40, 0.20}, {0.30, 0.0}, config, limits);

  EXPECT_DOUBLE_EQ(corrected.linear_x, 0.40);
  EXPECT_NEAR(corrected.angular_z, 0.50, 1e-12);
}

// 验证实际角速度逐渐接近名义值时，P 补偿量自动减小并最终回到名义值。
TEST(AngularVelocityTracking, ReducesCorrectionAsMeasuredSpeedApproachesDesired)
{
  const planner::AngularStabilizationConfig config;
  const planner::MotionLimits limits;
  const auto slower = planner::correctNominalAngularVelocity(
    {0.40, 0.20}, {0.30, 0.10}, config, limits);
  const auto matched = planner::correctNominalAngularVelocity(
    {0.40, 0.20}, {0.30, 0.20}, config, limits);

  EXPECT_NEAR(slower.angular_z, 0.30, 1e-12);
  EXPECT_NEAR(matched.angular_z, 0.20, 1e-12);
}

// 验证名义角速度位于命令死区时直接输出零，不让实测误差触发持续纠偏。
TEST(AngularVelocityTracking, KeepsDesiredCommandDeadband)
{
  const planner::AngularStabilizationConfig config;
  const planner::MotionLimits limits;
  const auto corrected = planner::correctNominalAngularVelocity(
    {0.40, config.command_deadband}, {0.30, -0.50}, config, limits);

  EXPECT_DOUBLE_EQ(corrected.angular_z, 0.0);
}

// 验证实际旋转尚未停稳时不立即反向，降到阈值后才允许纠偏。
TEST(AngularVelocityTracking, WaitsForLowMeasuredSpeedBeforeReversing)
{
  const planner::AngularStabilizationConfig config;
  const planner::MotionLimits limits;
  const auto braking = planner::correctNominalAngularVelocity(
    {0.30, -0.50}, {0.30, 0.40}, config, limits);
  const auto reversing = planner::correctNominalAngularVelocity(
    {0.30, -0.50}, {0.30, 0.10}, config, limits);

  EXPECT_DOUBLE_EQ(braking.angular_z, 0.0);
  EXPECT_DOUBLE_EQ(reversing.angular_z, -1.10);
}

// 验证 P 修正可能跨越零点时，仍等待较大的同向实测角速度先停止。
TEST(AngularVelocityTracking, PreventsCorrectionFromImmediateReversal)
{
  const planner::AngularStabilizationConfig config;
  const planner::MotionLimits limits;
  const auto corrected = planner::correctNominalAngularVelocity(
    {0.30, 0.20}, {0.30, 0.50}, config, limits);

  EXPECT_DOUBLE_EQ(corrected.angular_z, 0.0);
}

// 验证 P 修正后的角速度不会超过系统最大允许角速度。
TEST(AngularVelocityTracking, ClampsCorrectedCommandToMaximumSpeed)
{
  const planner::AngularStabilizationConfig config;
  planner::MotionLimits limits;
  limits.max_angular_speed = 1.50;
  const auto corrected = planner::correctNominalAngularVelocity(
    {0.30, 1.20}, {0.30, 0.0}, config, limits);

  EXPECT_DOUBLE_EQ(corrected.angular_z, limits.max_angular_speed);
}

// 验证负 P 反馈增益在节点启动前被拒绝。
TEST(PlannerConfig, RejectsInvalidAngularStabilization)
{
  planner::AngularStabilizationConfig config;
  config.velocity_tracking_kp = -0.1;
  std::string reason;

  EXPECT_FALSE(planner::validateAngularStabilizationConfig(config, &reason));
  EXPECT_FALSE(reason.empty());
}

// 验证加速度轨迹从实测速度开始，停车指令仍包含真实制动距离。
TEST(AcceleratingTrajectory, UsesMeasuredInitialVelocityForStopping)
{
  const planner::TrajectoryConfig trajectory_config{0.50, 0.05};
  const planner::MotionLimits limits;
  const auto poses = planner::predictAcceleratingTrajectory(
    {0.60, 0.0}, {0.0, 0.0}, trajectory_config, limits, false);

  ASSERT_FALSE(poses.empty());
  EXPECT_GT(poses.back().x, 0.15);
  EXPECT_LT(poses.back().x, 0.30);
}

// 验证预测时域末尾追加制动尾段，避免把尚未刹停的候选误判为安全。
TEST(AcceleratingTrajectory, AppendsBrakingTail)
{
  const planner::TrajectoryConfig trajectory_config{0.10, 0.05};
  const planner::MotionLimits limits;
  const auto short_poses = planner::predictAcceleratingTrajectory(
    {0.60, 0.0}, {0.60, 0.0}, trajectory_config, limits, false);
  const auto braking_poses = planner::predictAcceleratingTrajectory(
    {0.60, 0.0}, {0.60, 0.0}, trajectory_config, limits, true);

  ASSERT_FALSE(short_poses.empty());
  ASSERT_FALSE(braking_poses.empty());
  EXPECT_NEAR(short_poses.back().x, 0.06, 1e-9);
  EXPECT_GT(braking_poses.back().x, 0.27);
}

// 验证候选集合显式包含停车和两侧最小有效原地转向。
TEST(VelocitySampling, IncludesCriticalCandidates)
{
  const planner::MotionLimits limits;
  const planner::VelocitySamplingConfig sampling;
  const auto candidates = planner::sampleCandidateVelocities(
    {0.30, 0.10}, {0.20, 0.0}, limits, sampling);
  const auto contains = [&candidates](double linear, double angular) {
      for (const auto & candidate : candidates) {
        if (std::abs(candidate.linear_x - linear) < 1e-9 &&
          std::abs(candidate.angular_z - angular) < 1e-9)
        {
          return true;
        }
      }
      return false;
    };

  EXPECT_TRUE(contains(0.0, 0.0));
  EXPECT_TRUE(contains(0.0, limits.min_angular_speed));
  EXPECT_TRUE(contains(0.0, -limits.min_angular_speed));
  for (const auto & candidate : candidates) {
    EXPECT_GE(candidate.linear_x, 0.0);
    EXPECT_LE(std::abs(candidate.angular_z), sampling.max_avoidance_angular_speed);
  }
}

// 验证空旷场景中最低代价候选保持在名义跟随速度附近。
TEST(LocalVelocityPlanner, KeepsNominalVelocityInOpenSpace)
{
  const planner::TrajectoryConfig trajectory_config{1.20, 0.05};
  const planner::FootprintConfig footprint;
  const planner::MotionLimits limits;
  const planner::VelocitySamplingConfig sampling;
  const auto result = planner::planLocalVelocity(
    {0.30, 0.0}, {0.30, 0.0}, {0.30, 0.0}, {}, trajectory_config,
    footprint, limits, sampling);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.selected_velocity.linear_x, 0.30, 1e-9);
  EXPECT_NEAR(result.selected_velocity.angular_z, 0.0, 1e-9);
}

// 验证障碍消失且 UWB 要求直行时，历史绕障角速度不会把规划器锁在持续转向状态。
TEST(LocalVelocityPlanner, ReturnsToStraightAfterAvoidanceTurn)
{
  const planner::TrajectoryConfig trajectory_config{1.20, 0.05};
  const planner::FootprintConfig footprint;
  planner::MotionLimits limits;
  limits.min_angular_speed = 0.50;
  limits.max_angular_speed = 0.84;
  planner::VelocitySamplingConfig sampling;
  sampling.weight_follow_angular = 5.0;
  sampling.weight_smooth_angular = 7.0;
  const auto result = planner::planLocalVelocity(
    {0.30, 0.50}, {0.30, 0.50}, {0.30, 0.0}, {}, trajectory_config,
    footprint, limits, sampling);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.selected_velocity.linear_x, 0.30, 1e-9);
  EXPECT_NEAR(result.selected_velocity.angular_z, 0.0, 1e-9);
}

// 验证直行碰撞但同速转弯安全时，规划器保持 UWB 线速度并只调整角速度。
TEST(LocalVelocityPlanner, KeepsNominalSpeedWhenTurningIsSafe)
{
  const planner::TrajectoryConfig trajectory_config{1.20, 0.05};
  const planner::FootprintConfig footprint;
  const planner::MotionLimits limits;
  planner::VelocitySamplingConfig sampling;
  // 本用例只验证硬碰撞条件下的同速转弯，软净空和 TTC 降层由独立用例覆盖。
  sampling.minimum_safe_clearance = 0.0;
  sampling.minimum_ttc = 0.0;
  const std::vector<planner::ObstaclePoint2D> obstacles{{0.75, 0.0}};
  const auto result = planner::planLocalVelocity(
    {0.0, 0.0}, {0.0, 0.0}, {0.30, 0.0}, obstacles, trajectory_config,
    footprint, limits, sampling);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.avoidance_active);
  EXPECT_DOUBLE_EQ(result.selected_speed_scale, 1.0);
  EXPECT_DOUBLE_EQ(result.selected_velocity.linear_x, result.effective_nominal.linear_x);
  EXPECT_GE(
    std::abs(result.selected_velocity.angular_z), sampling.min_avoidance_angular_speed);
  EXPECT_GT(result.collision_count, 0U);
}

// 验证无障碍跟随保留 2.0 上限，进入主动避障后改用独立的 1.5 上限。
TEST(LocalVelocityPlanner, SeparatesFollowAndAvoidanceAngularLimits)
{
  const planner::TrajectoryConfig trajectory_config{1.20, 0.05};
  const planner::FootprintConfig footprint;
  const planner::MotionLimits limits;
  const planner::VelocitySamplingConfig sampling;
  const auto following = planner::planLocalVelocity(
    {0.0, 0.0}, {0.0, 0.0}, {0.0, 1.80}, {}, trajectory_config,
    footprint, limits, sampling);
  const auto avoiding = planner::planLocalVelocity(
    {0.0, 0.0}, {0.0, 0.0}, {0.30, 0.0}, {{0.75, 0.0}}, trajectory_config,
    footprint, limits, sampling);

  ASSERT_TRUE(following.valid);
  EXPECT_FALSE(following.avoidance_active);
  EXPECT_DOUBLE_EQ(following.selected_velocity.angular_z, 1.80);
  ASSERT_TRUE(avoiding.valid);
  EXPECT_TRUE(avoiding.avoidance_active);
  EXPECT_LE(
    std::abs(avoiding.selected_velocity.angular_z), sampling.max_avoidance_angular_speed);
}

// 验证障碍进入影响区但同速弧线仍安全时，不允许代价函数提前选择低速轨迹。
TEST(LocalVelocityPlanner, KeepsNominalSpeedForNearbyObstacle)
{
  const planner::TrajectoryConfig trajectory_config{1.20, 0.05};
  const planner::FootprintConfig footprint;
  const planner::MotionLimits limits;
  const planner::VelocitySamplingConfig sampling;
  const std::vector<planner::ObstaclePoint2D> obstacles{{0.90, 0.0}};
  const auto result = planner::planLocalVelocity(
    {0.0, 0.0}, {0.0, 0.0}, {0.30, 0.0}, obstacles, trajectory_config,
    footprint, limits, sampling);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.avoidance_active);
  EXPECT_DOUBLE_EQ(result.selected_speed_scale, 1.0);
  EXPECT_DOUBLE_EQ(result.selected_velocity.linear_x, result.effective_nominal.linear_x);
  const auto collision = planner::checkTrajectoryCollision(
    result.selected_trajectory, obstacles, footprint);
  EXPECT_FALSE(collision.collision);
}

// 验证整层同速角速度都不安全时，规划器才进入下一线速度比例层。
TEST(LocalVelocityPlanner, ReducesSpeedOnlyAfterNominalTierIsBlocked)
{
  const planner::TrajectoryConfig trajectory_config{1.20, 0.05};
  const planner::FootprintConfig footprint;
  planner::MotionLimits limits;
  limits.max_angular_accel = 1.50;
  const planner::VelocitySamplingConfig sampling;
  std::vector<planner::ObstaclePoint2D> obstacles;
  for (int index = -15; index <= 15; ++index) {
    obstacles.push_back({0.78, 0.10 * static_cast<double>(index)});
  }
  const auto result = planner::planLocalVelocity(
    {0.0, 0.0}, {0.0, 0.0}, {0.30, 0.0}, obstacles, trajectory_config,
    footprint, limits, sampling);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.avoidance_active);
  EXPECT_LT(result.selected_speed_scale, 1.0);
  EXPECT_GT(result.selected_speed_scale, 0.0);
  EXPECT_LT(result.selected_velocity.linear_x, result.effective_nominal.linear_x);
}

// 验证高速层只有勉强净空时继续降层，并在首个满足 TTC 的非零速度层返回。
TEST(LocalVelocityPlanner, ReducesSpeedForMarginalClearanceTtc)
{
  const planner::TrajectoryConfig trajectory_config{1.20, 0.05};
  const planner::FootprintConfig footprint;
  const planner::MotionLimits limits;
  planner::VelocitySamplingConfig sampling;
  sampling.angular_samples = 3;
  sampling.min_avoidance_angular_speed = 0.0;
  sampling.max_avoidance_angular_speed = 0.0;
  sampling.linear_speed_step = 0.0;
  sampling.linear_speed_priority_scales = {1.0, 0.5, 0.0};
  sampling.minimum_safe_clearance = 0.05;
  sampling.minimum_ttc = 0.40;
  const std::vector<planner::ObstaclePoint2D> obstacles{{0.60, 0.38}};

  const auto result = planner::planLocalVelocity(
    {0.0, 0.0}, {0.0, 0.0}, {0.30, 0.0}, obstacles, trajectory_config,
    footprint, limits, sampling);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.avoidance_active);
  EXPECT_DOUBLE_EQ(result.selected_speed_scale, 0.5);
  EXPECT_DOUBLE_EQ(result.selected_velocity.linear_x, limits.min_linear_speed);
  EXPECT_GT(result.marginal_count, 0U);
  EXPECT_GE(result.min_clearance, result.required_clearance);
  EXPECT_GE(result.clearance_ttc, sampling.minimum_ttc);
}

// 验证较低的非零速度层一旦严格安全，即使停车代价更低也禁止选择零速层。
TEST(LocalVelocityPlanner, PrefersAnySafeMovingTierOverStopping)
{
  const planner::TrajectoryConfig trajectory_config{1.20, 0.05};
  const planner::FootprintConfig footprint;
  const planner::MotionLimits limits;
  planner::VelocitySamplingConfig sampling;
  sampling.angular_samples = 3;
  sampling.min_avoidance_angular_speed = 0.0;
  sampling.max_avoidance_angular_speed = 0.0;
  sampling.linear_speed_step = 0.0;
  sampling.linear_speed_priority_scales = {1.0, 0.5, 0.0};
  sampling.minimum_safe_clearance = 0.05;
  sampling.minimum_ttc = 0.40;
  sampling.weight_follow_linear = 0.0;
  sampling.weight_follow_angular = 0.0;
  sampling.weight_smooth_linear = 0.0;
  sampling.weight_smooth_angular = 0.0;
  sampling.weight_obstacle = 0.0;
  sampling.weight_progress = 0.0;
  const std::vector<planner::ObstaclePoint2D> obstacles{{0.60, 0.38}};

  const auto result = planner::planLocalVelocity(
    {0.0, 0.0}, {0.0, 0.0}, {0.30, 0.0}, obstacles, trajectory_config,
    footprint, limits, sampling);

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.selected_speed_scale, 0.5);
  EXPECT_GT(result.selected_velocity.linear_x, 0.0);
}

// 验证全部非零层都只有勉强轨迹时，最后允许选择无硬碰撞的停车兜底。
TEST(LocalVelocityPlanner, AllowsStopOnlyAfterEveryMovingTierIsMarginal)
{
  const planner::TrajectoryConfig trajectory_config{1.20, 0.05};
  const planner::FootprintConfig footprint;
  const planner::MotionLimits limits;
  planner::VelocitySamplingConfig sampling;
  sampling.angular_samples = 3;
  sampling.min_avoidance_angular_speed = 0.0;
  sampling.max_avoidance_angular_speed = 0.0;
  sampling.linear_speed_step = 0.0;
  sampling.linear_speed_priority_scales = {1.0, 0.5, 0.0};
  sampling.minimum_safe_clearance = 0.25;
  sampling.minimum_ttc = 2.0;
  const std::vector<planner::ObstaclePoint2D> obstacles{{0.60, 0.38}};

  const auto result = planner::planLocalVelocity(
    {0.0, 0.0}, {0.0, 0.0}, {0.30, 0.0}, obstacles, trajectory_config,
    footprint, limits, sampling);

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.selected_speed_scale, 0.0);
  EXPECT_DOUBLE_EQ(result.selected_velocity.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(result.selected_velocity.angular_z, 0.0);
  EXPECT_GT(result.marginal_count, 0U);
  EXPECT_LT(result.min_clearance, result.required_clearance);
}

// 用侧障净空限制可行速度，验证所有新增档位都能参与真实规划。
TEST(LocalVelocityPlanner, SelectsFixedStepTiersByClearance)
{
  const planner::TrajectoryConfig trajectory{1.20, 0.05};
  const planner::FootprintConfig footprint;
  const planner::MotionLimits limits;
  planner::VelocitySamplingConfig sampling;
  sampling.angular_samples = 3;
  sampling.min_avoidance_angular_speed = 0.0;
  sampling.max_avoidance_angular_speed = 0.0;
  sampling.minimum_ttc = 0.40;
  const double half_width = footprint.robot_width * 0.5 + footprint.safety_margin;

  for (const double expected : {0.80, 0.70, 0.60, 0.50, 0.40, 0.30, 0.23, 0.0}) {
    SCOPED_TRACE(expected);
    const double clearance = expected > 0.0 ? (expected + 0.02) * 0.40 : 0.06;
    const std::vector<planner::ObstaclePoint2D> obstacles{{0.45, half_width + clearance}};
    const auto result = planner::planLocalVelocity(
      {}, {}, {0.80, 0.0}, obstacles, trajectory, footprint, limits, sampling);
    ASSERT_TRUE(result.valid);
    EXPECT_TRUE(result.avoidance_active);
    EXPECT_NEAR(result.selected_velocity.linear_x, expected, 1e-9);
    EXPECT_NEAR(result.selected_speed_scale, expected / 0.80, 1e-9);
    EXPECT_GE(result.min_clearance, result.required_clearance);
  }
}

TEST(LocalVelocityPlanner, KeepsMinimumExecutableTierAtLowNominalSpeeds)
{
  const planner::MotionLimits limits;
  planner::VelocitySamplingConfig sampling;
  sampling.angular_samples = 3;
  sampling.min_avoidance_angular_speed = 0.0;
  sampling.max_avoidance_angular_speed = 0.0;
  sampling.minimum_ttc = 0.40;
  for (const double nominal : {0.30, 0.23, 0.10, 0.0}) {
    SCOPED_TRACE(nominal);
    const auto result = planner::planLocalVelocity(
      {}, {}, {nominal, 0.0}, {{0.60, 0.38}}, {1.20, 0.05}, {}, limits, sampling);
    ASSERT_TRUE(result.valid);
    EXPECT_DOUBLE_EQ(result.selected_velocity.linear_x, nominal > 0.0 ? 0.23 : 0.0);
  }
  const auto forced_stop = planner::planLocalVelocity(
    {}, {}, {0.80, 0.0}, {}, {1.20, 0.05}, {}, limits, sampling, true);
  ASSERT_TRUE(forced_stop.valid);
  EXPECT_DOUBLE_EQ(forced_stop.selected_velocity.linear_x, 0.0);
}

TEST(PlannerConfig, ValidatesFixedSpeedStepAndLegacyMode)
{
  planner::VelocitySamplingConfig sampling;
  for (const double step : {-0.1, 0.001, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()})
  {
    sampling.linear_speed_step = step;
    EXPECT_FALSE(planner::validateVelocitySamplingConfig(sampling));
  }
  for (const double step : {0.0, 0.01, 0.10}) {
    sampling.linear_speed_step = step;
    EXPECT_TRUE(planner::validateVelocitySamplingConfig(sampling));
  }
  sampling.linear_speed_priority_scales.clear();
  EXPECT_TRUE(planner::validateVelocitySamplingConfig(sampling));
  sampling.linear_speed_step = 0.0;
  EXPECT_FALSE(planner::validateVelocitySamplingConfig(sampling));
}

// 验证障碍已经进入当前膨胀足迹时所有候选都会被安全淘汰。
TEST(LocalVelocityPlanner, RejectsEveryCandidateForOccupiedFootprint)
{
  const planner::TrajectoryConfig trajectory_config{1.20, 0.05};
  const planner::FootprintConfig footprint;
  const planner::MotionLimits limits;
  const planner::VelocitySamplingConfig sampling;
  const std::vector<planner::ObstaclePoint2D> obstacles{{0.40, 0.0}};
  const auto result = planner::planLocalVelocity(
    {0.0, 0.0}, {0.0, 0.0}, {0.30, 0.0}, obstacles, trajectory_config,
    footprint, limits, sampling);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.evaluated_count, result.collision_count);
}

// 验证最终速度限幅跨过执行死区，并在角速度换向时先经过零指令。
TEST(CommandLimiter, AppliesDeadzoneAndSafeAngularReversal)
{
  planner::MotionLimits limits;
  limits.min_angular_speed = 1.15;
  const auto accelerating = planner::limitCommandVelocity(
    {0.0, 0.0}, {0.60, 0.90}, limits, 0.05);
  const auto reversing = planner::limitCommandVelocity(
    {0.90, 0.90}, {0.20, -0.90}, limits, 0.05);

  EXPECT_DOUBLE_EQ(accelerating.linear_x, limits.min_linear_speed);
  EXPECT_DOUBLE_EQ(accelerating.angular_z, limits.min_angular_speed);
  EXPECT_DOUBLE_EQ(reversing.angular_z, 0.0);
}

// 验证线速度由前进切换到倒退时必须先发布零速，停稳后才允许负速度指令。
TEST(CommandLimiter, StopsBeforeEmergencyReverse)
{
  const planner::MotionLimits limits;
  const auto braking = planner::limitCommandVelocity(
    {0.13, 0.0}, {-limits.max_reverse_speed, 0.0}, limits, 0.05);
  const auto reversing = planner::limitCommandVelocity(
    {0.0, 0.0}, {-limits.max_reverse_speed, 0.0}, limits, 0.05);

  EXPECT_DOUBLE_EQ(braking.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(reversing.linear_x, -limits.min_linear_speed);
}

// 验证非法采样数量和运动学死区参数会被拒绝。
TEST(PlannerConfig, RejectsInvalidSamplingAndMotionLimits)
{
  planner::VelocitySamplingConfig sampling;
  sampling.linear_samples = 1;
  planner::VelocitySamplingConfig avoidance_limits;
  avoidance_limits.min_avoidance_angular_speed = 1.0;
  avoidance_limits.max_avoidance_angular_speed = 0.9;
  planner::VelocitySamplingConfig clearance_limits;
  clearance_limits.minimum_safe_clearance = -0.01;
  planner::VelocitySamplingConfig ttc_limits;
  ttc_limits.minimum_ttc = -0.01;
  planner::MotionLimits limits;
  limits.min_angular_speed = limits.max_angular_speed + 0.1;
  planner::MotionLimits reverse_limits;
  reverse_limits.max_reverse_speed = reverse_limits.min_linear_speed * 0.5;
  planner::EmergencyReverseConfig reverse_config;
  reverse_config.speed = 0.50;
  std::string reason;

  EXPECT_FALSE(planner::validateVelocitySamplingConfig(sampling, &reason));
  EXPECT_FALSE(planner::validateVelocitySamplingConfig(avoidance_limits, &reason));
  EXPECT_FALSE(planner::validateVelocitySamplingConfig(clearance_limits, &reason));
  EXPECT_FALSE(planner::validateVelocitySamplingConfig(ttc_limits, &reason));
  EXPECT_FALSE(planner::validateMotionLimits(limits, &reason));
  EXPECT_FALSE(planner::validateMotionLimits(reverse_limits, &reason));
  EXPECT_FALSE(
    planner::validateEmergencyReverseConfig(reverse_config, planner::MotionLimits{}, &reason));
}

TEST(CommandPrediction, IncludesHistoricalTurnAndPublishesCheckedCommand)
{
  planner::MotionLimits limits;
  limits.max_angular_speed = 1.5;
  limits.max_angular_accel = 1.5;
  limits.max_linear_accel = 0.5;
  const planner::PlannerVelocity2D previous{0.4, 1.5};
  const planner::CommandPredictionConfig prediction;
  const auto result = planner::planLocalVelocity(
    {0.4, 0.0}, previous, {0.8, 0.0}, {}, {1.5, 0.08}, {}, limits, {});
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.first_command.linear_x, 0.425, 1e-9);
  EXPECT_NEAR(result.first_command.angular_z, 1.425, 1e-9);
  EXPECT_GT(result.selected_trajectory.back().yaw, 0.1);
  const auto expected = planner::predictCommandTrajectory(
    {0.4, 0.0}, previous, result.selected_velocity, {1.5, 0.08}, limits,
    prediction, limits.max_angular_speed, false);
  ASSERT_EQ(expected.size(), result.selected_trajectory.size());
  EXPECT_DOUBLE_EQ(expected.back().x, result.selected_trajectory.back().x);
  EXPECT_DOUBLE_EQ(expected.back().yaw, result.selected_trajectory.back().yaw);
}

TEST(CommandPrediction, HistoricalTurnCollisionIsNotTreatedAsStraightClearance)
{
  const planner::MotionLimits limits;
  const planner::FootprintConfig footprint{0.70, 0.38, 0.02};
  const planner::TrajectoryConfig trajectory{1.5, 0.05};
  const auto straight = planner::predictAcceleratingTrajectory(
    {0.4, 0.0}, {0.8, 0.0}, trajectory, limits);
  const auto transition = planner::predictCommandTrajectory(
    {0.4, 0.0}, {0.4, 1.5}, {0.8, 0.0}, trajectory, limits, {}, 2.0, false);
  ASSERT_FALSE(transition.empty());
  // Pick a point on the turning path well outside the old straight swept footprint.
  planner::ObstaclePoint2D obstacle;
  bool found = false;
  for (const auto & pose : transition) {
    if (pose.y > 0.35) {
      obstacle = {pose.x, pose.y};
      found = true;
      break;
    }
  }
  ASSERT_TRUE(found);
  EXPECT_FALSE(planner::checkTrajectoryCollision(straight, {obstacle}, footprint).collision);
  EXPECT_TRUE(planner::checkTrajectoryCollision(transition, {obstacle}, footprint).collision);
  const auto result = planner::planLocalVelocity(
    {0.4, 0.0}, {0.4, 1.5}, {0.8, 0.0}, {obstacle}, trajectory, footprint, limits, {});
  EXPECT_TRUE(result.avoidance_active);
  if (result.valid) {
    EXPECT_FALSE(
      planner::checkTrajectoryCollision(
        result.selected_trajectory, {obstacle}, footprint).collision);
  }
}

TEST(CommandPrediction, DelayExtendsStoppingTravelAndStopOverridesArePredicted)
{
  const planner::MotionLimits limits;
  planner::CommandPredictionConfig prediction;
  prediction.response_delay = 0.0;
  const auto immediate = planner::predictCommandTrajectory(
    {0.8, 0.0}, {0.8, 0.0}, {}, {1.5, 0.05}, limits, prediction, 2.0, true);
  prediction.response_delay = 0.3;
  planner::PlannerVelocity2D first;
  const auto delayed = planner::predictCommandTrajectory(
    {0.8, 0.0}, {0.8, 0.0}, {}, {1.5, 0.05}, limits, prediction, 2.0, true, &first);
  ASSERT_FALSE(immediate.empty());
  ASSERT_FALSE(delayed.empty());
  EXPECT_DOUBLE_EQ(first.linear_x, 0.0);
  EXPECT_GT(delayed.back().x, immediate.back().x + 0.2);
  EXPECT_TRUE(planner::checkTrajectoryCollision({}, {}, {}).collision);
}

TEST(FollowRecovery, ResidualTurnCannotAccelerateOrReleaseWithoutNewObservations)
{
  planner::FollowRecoveryState state;
  state.phase = planner::FollowRecoveryPhase::AVOIDING;
  state.speed_cap = 0.30;
  planner::FollowRecoveryInput input{0.0, true, true};
  auto result = planner::planRecoveringVelocity(
    {0.3, 0.7}, {0.3, 0.7}, {0.8, 0.0}, {}, {}, {}, {}, {}, {}, {}, input, state);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(state.phase, planner::FollowRecoveryPhase::RECOVERING);
  EXPECT_LE(result.first_command.linear_x, 0.30);
  EXPECT_GT(result.first_command.angular_z, 0.0);
  EXPECT_EQ(state.clear_count, 0);
  // Zero measured rotation and zero nominal rotation do not imply heading alignment.
  input.heading = 0.4;
  result = planner::planRecoveringVelocity(
    {0.3, 0.0}, {0.3, 0.0}, {0.8, 0.0}, {}, {}, {}, {}, {}, {}, {}, input, state);
  EXPECT_EQ(state.clear_count, 0);
  EXPECT_LE(result.first_command.linear_x, 0.30);
  input.heading = 0.0;
  for (int index = 0; index < 2; ++index) {
    result = planner::planRecoveringVelocity(
      {0.3, 0.0}, {0.3, 0.0}, {0.8, 0.0}, {}, {}, {}, {}, {}, {}, {}, input, state);
    EXPECT_EQ(state.phase, planner::FollowRecoveryPhase::RECOVERING);
  }
  input.new_observation = false;
  for (int index = 0; index < 10; ++index) {
    result = planner::planRecoveringVelocity(
      {0.3, 0.0}, {0.3, 0.0}, {0.8, 0.0}, {}, {}, {}, {}, {}, {}, {}, input, state);
    EXPECT_LE(result.first_command.linear_x, 0.30);
  }
  EXPECT_EQ(state.clear_count, 2);
  input.new_observation = true;
  result = planner::planRecoveringVelocity(
    {0.3, 0.0}, {0.3, 0.0}, {0.8, 0.0}, {}, {}, {}, {}, {}, {}, {}, input, state);
  EXPECT_EQ(state.phase, planner::FollowRecoveryPhase::FOLLOWING);
  EXPECT_GT(result.first_command.linear_x, 0.30);
  EXPECT_LT(result.first_command.linear_x, 0.8);
}

TEST(FollowRecovery, SafeReverseCorrectionKeepsMovingThroughZeroTurn)
{
  planner::FollowRecoveryState state;
  planner::PlannerVelocity2D previous{0.4, 1.0};
  bool crossed_zero = false;
  for (int index = 0; index < 30; ++index) {
    const auto result = planner::planRecoveringVelocity(
      previous, previous, {0.8, -0.5}, {}, {}, {}, {}, {}, {}, {},
      {-0.5, true, true}, state);
    ASSERT_TRUE(result.valid);
    EXPECT_EQ(state.phase, planner::FollowRecoveryPhase::RECOVERING);
    EXPECT_FALSE(state.turn_unestablished);
    EXPECT_DOUBLE_EQ(result.first_command.linear_x, 0.4);
    EXPECT_LE(std::abs(result.first_command.angular_z - previous.angular_z), 0.100000001);
    if (index == 0) {
      EXPECT_TRUE(state.reversing);
      EXPECT_GT(result.first_command.angular_z, 0.0);
    }
    crossed_zero = crossed_zero || result.first_command.angular_z < 0.0;
    previous = result.first_command;
  }
  EXPECT_TRUE(crossed_zero);
}

TEST(FollowRecovery, SustainedWeakTurnStopsButTransientLagDoesNot)
{
  planner::FollowRecoveryState state;
  planner::PlannerVelocity2D previous{0.3, 0.4};
  for (int index = 0; index < 10; ++index) {
    const auto result = planner::planRecoveringVelocity(
      {0.3, 0.03}, previous, {0.8, 0.5}, {}, {}, {}, {}, {}, {}, {},
      {0.5, true, true}, state);
    ASSERT_TRUE(result.valid);
    if (index < 6) {
      EXPECT_FALSE(state.turn_unestablished);
      EXPECT_GT(result.first_command.linear_x, 0.0);
      EXPECT_LE(result.first_command.linear_x, previous.linear_x);
    } else {
      EXPECT_TRUE(state.turn_unestablished);
      EXPECT_DOUBLE_EQ(result.first_command.linear_x, 0.0);
    }
    EXPECT_GT(result.first_command.angular_z, 0.4);
    previous = result.first_command;
  }
}

TEST(FollowRecovery, UnsafeAccelerationProbePreventsRelease)
{
  planner::FollowRecoveryState state;
  state.phase = planner::FollowRecoveryPhase::RECOVERING;
  state.speed_cap = 0.30;
  for (int index = 0; index < 5; ++index) {
    const auto result = planner::planRecoveringVelocity(
      {}, {}, {0.8, 0.0}, {{0.7, 0.0}}, {}, {}, {}, {}, {}, {},
      {0.0, true, true}, state);
    EXPECT_NE(state.phase, planner::FollowRecoveryPhase::FOLLOWING);
    EXPECT_EQ(state.clear_count, 0);
    if (result.valid) {
      EXPECT_FALSE(
        planner::checkTrajectoryCollision(
          result.selected_trajectory, {{0.7, 0.0}}, {}).collision);
    }
  }
}

TEST(FollowRecovery, InvalidHeadingAndSubDeadzoneCapCannotStartWalking)
{
  planner::FollowRecoveryState state;
  auto result = planner::planRecoveringVelocity(
    {}, {}, {0.8, 0.0}, {}, {}, {}, {}, {}, {}, {}, {}, state);
  EXPECT_FALSE(result.valid);
  state.speed_cap = 0.20;
  const planner::FollowRecoveryInput input{0.4, true, true};
  result = planner::planRecoveringVelocity(
    {}, {}, {0.8, 0.0}, {}, {}, {}, {}, {}, {}, {}, input, state);
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.first_command.linear_x, 0.0);
}

TEST(FollowRecovery, StandingAvoidanceCanRestartOnlyAtMinimumSpeed)
{
  planner::FollowRecoveryState state;
  state.phase = planner::FollowRecoveryPhase::AVOIDING;
  state.speed_cap = 0.30;
  const auto result = planner::planRecoveringVelocity(
    {0.0, 0.5}, {0.0, 0.5}, {0.8, 0.0}, {{0.9, -0.2}}, {}, {}, {}, {}, {}, {},
    {0.0, true, true}, state);
  ASSERT_TRUE(result.valid);
  EXPECT_NE(state.phase, planner::FollowRecoveryPhase::FOLLOWING);
  EXPECT_LE(result.first_command.linear_x, 0.23);
  if (result.first_command.linear_x > 0.0) {
    EXPECT_DOUBLE_EQ(result.first_command.linear_x, 0.23);
  }
}

TEST(FollowRecovery, SafetyLossResetsConsecutiveReleaseCount)
{
  planner::FollowRecoveryState state;
  state.phase = planner::FollowRecoveryPhase::RECOVERING;
  state.speed_cap = 0.3;
  state.clear_count = 2;
  const auto result = planner::planRecoveringVelocity(
    {0.3, 0.0}, {0.3, 0.0}, {0.8, 0.0}, {{0.7, 0.0}}, {}, {}, {}, {}, {}, {},
    {0.0, true, false}, state);
  EXPECT_NE(state.phase, planner::FollowRecoveryPhase::FOLLOWING);
  EXPECT_EQ(state.clear_count, 0);
  if (result.valid) {
    EXPECT_FALSE(
      planner::checkTrajectoryCollision(
        result.selected_trajectory, {{0.7, 0.0}}, {}).collision);
  }
}

TEST(CommandPrediction, ScoringPreferenceCannotBecomeExecutionHistory)
{
  const planner::PlannerVelocity2D scoring{0.3, 1.5};
  const auto result = planner::planLocalVelocity(
    {0.3, 0.0}, {0.3, 0.0}, {0.8, 0.0}, {}, {}, {}, {}, {}, false, {}, &scoring);
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.first_command.angular_z, 0.0);
  EXPECT_DOUBLE_EQ(result.selected_trajectory.back().yaw, 0.0);
}

TEST(PlannerConfig, RejectsInvalidPredictionAndRecoveryParameters)
{
  planner::CommandPredictionConfig prediction;
  prediction.control_period = 0.0;
  EXPECT_FALSE(planner::validateCommandPredictionConfig(prediction));
  prediction = {};
  prediction.response_delay = -0.1;
  EXPECT_FALSE(planner::validateCommandPredictionConfig(prediction));
  prediction = {};
  prediction.angular_response_gain = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(planner::validateCommandPredictionConfig(prediction));
  planner::FollowRecoveryConfig recovery;
  recovery.clear_observations = 0;
  EXPECT_FALSE(planner::validateFollowRecoveryConfig(recovery));
  recovery = {};
  recovery.turn_tracking_timeout = -0.1;
  EXPECT_FALSE(planner::validateFollowRecoveryConfig(recovery));
  recovery = {};
  recovery.turn_command_threshold = recovery.settled_command_angular;
  EXPECT_FALSE(planner::validateFollowRecoveryConfig(recovery));
}

TEST(FollowRecovery, SafeAvoidanceKeepsPlannerSpeedAbovePointThree)
{
  const planner::PlannerVelocity2D measured{0.6, 0.5};
  const planner::PlannerVelocity2D nominal{0.8, 0.5};
  const std::vector<planner::ObstaclePoint2D> obstacles{{0.0, -0.6}};
  const auto baseline = planner::planLocalVelocity(
    measured, measured, nominal, obstacles, {}, {}, {}, {});
  ASSERT_TRUE(baseline.valid);
  ASSERT_TRUE(baseline.avoidance_active);
  ASSERT_GT(baseline.selected_velocity.linear_x, 0.3);
  // Both initial avoidance and re-entry from recovery must ignore old recovery caps.
  for (const auto phase : {planner::FollowRecoveryPhase::FOLLOWING,
      planner::FollowRecoveryPhase::AVOIDING, planner::FollowRecoveryPhase::RECOVERING})
  {
    planner::FollowRecoveryState state;
    state.phase = phase;
    state.speed_cap = 0.23;
    const auto result = planner::planRecoveringVelocity(
      measured, measured, nominal, obstacles, {}, {}, {}, {}, {}, {},
      {0.5, true, true}, state);
    ASSERT_TRUE(result.valid);
    EXPECT_EQ(state.phase, planner::FollowRecoveryPhase::AVOIDING);
    EXPECT_DOUBLE_EQ(result.selected_velocity.linear_x, baseline.selected_velocity.linear_x);
    EXPECT_DOUBLE_EQ(result.first_command.linear_x, baseline.first_command.linear_x);
    EXPECT_DOUBLE_EQ(result.first_command.angular_z, baseline.first_command.angular_z);
    EXPECT_GT(result.first_command.linear_x, 0.6);
  }
}

TEST(FollowRecovery, ExitCapturesLastOutputAndRatchetsOnlyWithActualDeceleration)
{
  planner::FollowRecoveryState state;
  state.phase = planner::FollowRecoveryPhase::AVOIDING;
  state.speed_cap = 0.23;  // Must not carry a stale cap into a new recovery episode.
  auto result = planner::planRecoveringVelocity(
    {0.6, 0.5}, {0.6, 0.5}, {0.8, 0.0}, {}, {}, {}, {}, {}, {}, {},
    {0.4, true, true}, state);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(state.phase, planner::FollowRecoveryPhase::RECOVERING);
  EXPECT_DOUBLE_EQ(state.speed_cap, 0.6);
  EXPECT_DOUBLE_EQ(result.first_command.linear_x, 0.6);
  result = planner::planRecoveringVelocity(
    {0.6, 0.5}, result.first_command, {0.3, 0.0}, {}, {}, {}, {}, {}, {}, {},
    {0.4, true, true}, state);
  ASSERT_TRUE(result.valid);
  const double reduced = result.first_command.linear_x;
  EXPECT_LT(reduced, 0.6);
  EXPECT_GT(reduced, 0.3);
  EXPECT_DOUBLE_EQ(state.speed_cap, reduced);
  result = planner::planRecoveringVelocity(
    {reduced, 0.5}, result.first_command, {0.8, 0.0}, {}, {}, {}, {}, {}, {}, {},
    {0.4, true, true}, state);
  ASSERT_TRUE(result.valid);
  EXPECT_LE(result.first_command.linear_x, reduced);
}


TEST(FollowRecovery, OppositeTargetDoesNotStopSafeAvoidance)
{
  planner::FollowRecoveryState state;
  const auto baseline = planner::planLocalVelocity(
    {0.4, 0.8}, {0.4, 0.8}, {0.8, -0.3}, {{0.6, -0.4}}, {}, {}, {}, {});
  ASSERT_TRUE(baseline.valid);
  ASSERT_TRUE(baseline.avoidance_active);
  ASSERT_GT(baseline.selected_velocity.angular_z, 0.0);
  const auto result = planner::planRecoveringVelocity(
    {0.4, 0.8}, {0.4, 0.8}, {0.8, -0.3}, {{0.6, -0.4}},
    {}, {}, {}, {}, {}, {}, {-0.5, true, true}, state);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(state.phase, planner::FollowRecoveryPhase::AVOIDING);
  EXPECT_FALSE(state.reversing);
  EXPECT_FALSE(state.turn_unestablished);
  EXPECT_DOUBLE_EQ(result.first_command.linear_x, baseline.first_command.linear_x);
  EXPECT_DOUBLE_EQ(result.first_command.angular_z, baseline.first_command.angular_z);
}

TEST(FollowRecovery, EstablishedTurnClearsTransientMismatch)
{
  planner::FollowRecoveryState state;
  state.turn_mismatch_sec = 0.25;
  const auto result = planner::planRecoveringVelocity(
    {0.4, 0.5}, {0.4, 0.5}, {0.8, 0.5}, {}, {}, {}, {}, {}, {}, {},
    {0.5, true, true}, state);
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(state.turn_unestablished);
  EXPECT_DOUBLE_EQ(state.turn_mismatch_sec, 0.0);
  EXPECT_GT(result.first_command.linear_x, 0.4);
}

TEST(FollowRecovery, LargeHeadingKeepsRecoveryMovingWithoutAcceleration)
{
  planner::FollowRecoveryState state;
  state.phase = planner::FollowRecoveryPhase::AVOIDING;
  const auto result = planner::planRecoveringVelocity(
    {0.6, 0.5}, {0.6, 0.5}, {0.8, 0.0}, {}, {}, {}, {}, {}, {}, {},
    {-0.8, true, true}, state);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(state.phase, planner::FollowRecoveryPhase::RECOVERING);
  EXPECT_FALSE(state.reversing);
  EXPECT_DOUBLE_EQ(result.first_command.linear_x, 0.6);
  EXPECT_GT(result.first_command.angular_z, 0.0);
  EXPECT_LT(result.first_command.angular_z, 0.5);
  EXPECT_EQ(state.clear_count, 0);
}

TEST(CollisionChecker, OptimizedScanMatchesPointwiseReference)
{
  const planner::FootprintConfig footprint{0.70, 0.38, 0.02};
  for (int scenario = 0; scenario < 20; ++scenario) {
    std::vector<planner::PlannerPose2D> poses;
    std::vector<planner::ObstaclePoint2D> obstacles;
    for (int i = 0; i < 30; ++i) {
      poses.push_back({0.01 * i, -0.02 * i, 0.04 * i - 0.5});
    }
    for (int i = 0; i < 100; ++i) {
      obstacles.push_back({0.1 * scenario + std::cos(i * 0.2), 1.0 + std::sin(i * 0.2)});
    }
    planner::CollisionResult reference;
    bool hit = false;
    for (std::size_t i = 0; i < poses.size() && !hit; ++i) {
      for (const auto & obstacle : obstacles) {
        const double clearance = planner::pointToFootprintClearance(poses[i], obstacle, footprint);
        reference.min_clearance = std::min(reference.min_clearance, clearance);
        if (clearance <= 0.0) {
          reference.collision = hit = true;
          reference.collision_pose_index = i;
          break;
        }
      }
    }
    const auto actual = planner::checkTrajectoryCollision(poses, obstacles, footprint);
    EXPECT_EQ(actual.collision, reference.collision);
    EXPECT_EQ(actual.collision_pose_index, reference.collision_pose_index);
    EXPECT_NEAR(actual.min_clearance, reference.min_clearance, 1e-12);
  }
}
