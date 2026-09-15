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
#include <chrono>
#include <iostream>
#include <random>
#include <vector>

#include "go2_uwb_local_follow/local_planner_core.hpp"

// 用固定种子的侧墙场景测量不同点数下的规划时间，不向 ROS 或底盘发布任何消息。
int main()
{
  namespace planner = go2_uwb_local_follow;
  const planner::TrajectoryConfig trajectory{1.5, 0.05};
  const planner::FootprintConfig footprint{0.7, 0.38, 0.05};
  planner::MotionLimits limits;
  limits.max_reverse_speed = 0.3;
  limits.max_angular_speed = 1.5;
  limits.max_linear_accel = 0.5;
  limits.max_linear_decel = 0.7;
  limits.max_angular_accel = 1.5;
  planner::VelocitySamplingConfig sampling;
  sampling.min_avoidance_angular_speed = 0.5;
  sampling.obstacle_influence_distance = 0.5;
  sampling.weight_follow_linear = 4.0;
  sampling.weight_follow_angular = 4.0;
  sampling.weight_smooth_linear = 6.0;
  sampling.weight_smooth_angular = 5.0;
  sampling.weight_obstacle = 2.0;
  sampling.weight_progress = 1.0;
  for (const int count : {200, 2000, 5000}) {
    std::mt19937 generator(20260915);
    std::uniform_real_distribution<double> x_distribution(-0.7, 3.0);
    std::uniform_real_distribution<double> y_distribution(0.65, 1.25);
    std::vector<planner::ObstaclePoint2D> obstacles;
    for (int i = 0; i < count; ++i) {
      obstacles.push_back({x_distribution(generator), y_distribution(generator)});
    }
    std::vector<double> timings;
    std::size_t candidates = 0U;
    for (int repeat = 0; repeat < 101; ++repeat) {
      const auto start = std::chrono::steady_clock::now();
      const auto result = planner::planLocalVelocity(
        {0.4, 0.0}, {0.4, 0.0}, {0.8, 0.0}, obstacles,
        trajectory, footprint, limits, sampling, false, 0.05);
      const double elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
      if (repeat > 0) {
        timings.push_back(elapsed);
      }
      candidates = result.evaluated_count;
    }
    std::sort(timings.begin(), timings.end());
    std::cout << "障碍点=" << count << " 候选=" << candidates
              << " 中位耗时_ms=" << timings[49] << " P95_ms=" << timings[94]
              << " 最大耗时_ms=" << timings.back() << '\n';
  }
  return 0;
}
