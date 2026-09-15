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
#include <random>
#include <vector>
#include "gtest/gtest.h"
#include "go2_uwb_local_follow/obstacle_index.hpp"
namespace planner = go2_uwb_local_follow;

// 与逐点精确计算对照随机旋转足迹，确保空间剪枝不会漏掉碰撞或改变最小净空。
TEST(ObstacleIndex, MatchesBruteForceAcrossRotatedFootprints)
{
  std::mt19937 generator(20260915);
  std::uniform_real_distribution<double> coordinate(-3.0, 3.0);
  for (int scenario = 0; scenario < 80; ++scenario) {
    std::vector<planner::ObstaclePoint2D> points;
    for (int i = 0; i < 300; ++i) {
      points.push_back({coordinate(generator), coordinate(generator)});
    }
    const planner::ObstacleIndex index(points);
    const planner::FootprintConfig footprint{0.7, 0.38, 0.05};
    const planner::PlannerPose2D pose{
      coordinate(generator), coordinate(generator), coordinate(generator)};
    double expected = std::numeric_limits<double>::infinity();
    for (const auto & point : points) {
      expected = std::min(expected, planner::pointToFootprintClearance(pose, point, footprint));
    }
    const auto result = index.check({pose}, footprint);
    EXPECT_NEAR(result.min_clearance, expected, 1e-12);
    EXPECT_EQ(result.collision, expected == 0.0);
  }
}

// 空点云、边界接触和首次碰撞序号保持原有碰撞检查语义。
TEST(ObstacleIndex, PreservesEmptyAndFirstCollisionSemantics)
{
  EXPECT_FALSE(planner::ObstacleIndex({}).check({{}}, {}).collision);
  const planner::ObstacleIndex index({{1.0, 0.0}});
  const auto result = index.check({{}, {0.8, 0.0, 0.0}, {1.0, 0.0, 0.0}}, {});
  ASSERT_TRUE(result.collision);
  EXPECT_EQ(result.collision_pose_index, 1U);
}
