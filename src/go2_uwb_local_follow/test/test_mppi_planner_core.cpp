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

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "go2_uwb_local_follow/mppi_planner_core.hpp"

namespace planner = go2_uwb_local_follow;

// 验证空旷场景输出完整控制序列，并保持普通规划不产生负线速度。
TEST(MppiPlanner, ProducesForwardControlSequenceInOpenSpace)
{
  planner::MppiOptimizer optimizer;
  const planner::TrajectoryConfig trajectory{0.60, 0.05};
  const planner::FootprintConfig footprint;
  const planner::MotionLimits limits;
  const planner::VelocitySamplingConfig costs;
  planner::MppiConfig config;
  config.batch_size = 64;
  config.iteration_count = 2;

  const auto result = optimizer.plan(
    {0.30, 0.0}, {0.30, 0.0}, {0.30, 0.0}, {}, trajectory, footprint,
    limits, costs, config);

  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.selected_controls.size(), 12U);
  EXPECT_NEAR(result.selected_velocity.linear_x, 0.30, 0.01);
  EXPECT_NEAR(result.selected_velocity.angular_z, 0.0, 0.01);
  for (const auto & control : result.selected_controls) {
    EXPECT_GE(control.linear_x, 0.0);
    EXPECT_LE(control.linear_x, 0.30);
  }
}

// 验证正前方障碍会触发整段序列优化，最终轨迹仍通过硬碰撞检查。
TEST(MppiPlanner, OptimizesSafeSequenceAroundFrontObstacle)
{
  planner::MppiOptimizer optimizer;
  const planner::TrajectoryConfig trajectory{1.20, 0.05};
  const planner::FootprintConfig footprint;
  const planner::MotionLimits limits;
  planner::VelocitySamplingConfig costs;
  costs.minimum_safe_clearance = 0.0;
  costs.minimum_ttc = 0.0;
  planner::MppiConfig config;
  config.batch_size = 128;
  config.iteration_count = 2;
  const std::vector<planner::ObstaclePoint2D> obstacles{{0.75, 0.0}};

  const auto result = optimizer.plan(
    {0.0, 0.0}, {0.0, 0.0}, {0.30, 0.0}, obstacles, trajectory, footprint,
    limits, costs, config);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.avoidance_active);
  EXPECT_FALSE(result.selected_controls.empty());
  const bool time_varying = std::any_of(
    result.selected_controls.begin() + 1, result.selected_controls.end(),
    [&result](const planner::PlannerVelocity2D & control) {
      return std::abs(control.linear_x - result.selected_controls.front().linear_x) > 1e-6 ||
      std::abs(control.angular_z - result.selected_controls.front().angular_z) > 1e-6;
    });
  EXPECT_TRUE(time_varying);
  const auto collision = planner::checkTrajectoryCollision(
    result.selected_trajectory, obstacles, footprint);
  EXPECT_FALSE(collision.collision);
  EXPECT_GT(result.evaluated_count, 0U);
}

// 验证障碍占据当前膨胀足迹时，MPPI 不会用加权平均绕过起点硬碰撞。
TEST(MppiPlanner, RejectsOccupiedCurrentFootprint)
{
  planner::MppiOptimizer optimizer;
  const planner::TrajectoryConfig trajectory{0.60, 0.05};
  const planner::FootprintConfig footprint;
  const planner::MotionLimits limits;
  const planner::VelocitySamplingConfig costs;
  planner::MppiConfig config;
  config.batch_size = 32;
  config.iteration_count = 1;

  const auto result = optimizer.plan(
    {0.0, 0.0}, {0.0, 0.0}, {0.30, 0.0}, {{0.30, 0.0}}, trajectory,
    footprint, limits, costs, config);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.evaluated_count, result.collision_count);
}

// 验证非法批量、温度和噪声相关系数会在启动前被拒绝。
TEST(MppiPlanner, RejectsInvalidConfiguration)
{
  planner::MppiConfig small_batch;
  small_batch.batch_size = 7;
  planner::MppiConfig temperature;
  temperature.temperature = 0.0;
  planner::MppiConfig correlation;
  correlation.noise_correlation = 1.0;
  std::string reason;

  EXPECT_FALSE(planner::validateMppiConfig(small_batch, &reason));
  EXPECT_FALSE(planner::validateMppiConfig(temperature, &reason));
  EXPECT_FALSE(planner::validateMppiConfig(correlation, &reason));
}
