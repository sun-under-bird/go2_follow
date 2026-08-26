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

// 验证默认不设最小角速度，小角速度不会被强制放大。
TEST(VelocitySampling, PreservesSmallAngularVelocityByDefault)
{
  const planner::MotionLimits limits;
  const auto effective = planner::makeEffectiveVelocity({0.20, 0.085}, limits);

  EXPECT_DOUBLE_EQ(effective.angular_z, 0.085);
}

// 验证同向实测角速度会削弱名义转向，在接近目标方向时提前撤销角速度。
TEST(AngularStabilization, DampsAndStopsSameDirectionTurn)
{
  const planner::AngularStabilizationConfig config;
  const auto damped = planner::stabilizeNominalAngularVelocity(
    {0.40, 0.60}, {0.30, 0.50}, config);
  const auto stopped = planner::stabilizeNominalAngularVelocity(
    {0.40, 0.20}, {0.30, 0.50}, config);

  EXPECT_DOUBLE_EQ(damped.linear_x, 0.40);
  EXPECT_NEAR(damped.angular_z, 0.425, 1e-12);
  EXPECT_DOUBLE_EQ(stopped.angular_z, 0.0);
}

// 验证实际旋转尚未停稳时不立即反向，降到阈值后才允许纠偏。
TEST(AngularStabilization, WaitsForLowMeasuredSpeedBeforeReversing)
{
  const planner::AngularStabilizationConfig config;
  const auto braking = planner::stabilizeNominalAngularVelocity(
    {0.30, -0.50}, {0.30, 0.40}, config);
  const auto reversing = planner::stabilizeNominalAngularVelocity(
    {0.30, -0.50}, {0.30, 0.10}, config);

  EXPECT_DOUBLE_EQ(braking.angular_z, 0.0);
  EXPECT_DOUBLE_EQ(reversing.angular_z, -0.50);
}

// 验证负阻尼参数在节点启动前被拒绝。
TEST(PlannerConfig, RejectsInvalidAngularStabilization)
{
  planner::AngularStabilizationConfig config;
  config.velocity_damping_gain = -0.1;
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
  const planner::VelocitySamplingConfig sampling;
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
  const planner::MotionLimits limits;
  const planner::VelocitySamplingConfig sampling;
  std::vector<planner::ObstaclePoint2D> obstacles;
  for (int index = -15; index <= 15; ++index) {
    obstacles.push_back({0.70, 0.10 * static_cast<double>(index)});
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

// 验证非法采样数量和运动学死区参数会被拒绝。
TEST(PlannerConfig, RejectsInvalidSamplingAndMotionLimits)
{
  planner::VelocitySamplingConfig sampling;
  sampling.linear_samples = 1;
  planner::MotionLimits limits;
  limits.min_angular_speed = limits.max_angular_speed + 0.1;
  std::string reason;

  EXPECT_FALSE(planner::validateVelocitySamplingConfig(sampling, &reason));
  EXPECT_FALSE(planner::validateMotionLimits(limits, &reason));
}
