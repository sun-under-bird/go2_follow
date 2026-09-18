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

#ifndef GO2_UWB_LOCAL_FOLLOW__MPPI_PLANNER_CORE_HPP_
#define GO2_UWB_LOCAL_FOLLOW__MPPI_PLANNER_CORE_HPP_

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "go2_uwb_local_follow/local_planner_core.hpp"

namespace go2_uwb_local_follow
{

struct MppiConfig
{
  int batch_size{48};
  int iteration_count{2};
  double temperature{0.05};
  double linear_noise_std{0.18};
  double angular_noise_std{0.45};
  // 相邻时刻噪声相关系数，让有限批量也能形成连续的转弯控制序列。
  double noise_correlation{0.85};
  std::uint32_t random_seed{7U};
};

// 校验 MPPI 批量、迭代次数、温度和噪声参数。
bool validateMppiConfig(
  const MppiConfig & config,
  std::string * reason = nullptr);

class MppiOptimizer
{
public:
  // 创建带固定随机种子的优化器，保证回归测试和影子测试可复现。
  MppiOptimizer();

  // 清除上一周期最优控制序列，下次规划重新以名义速度初始化。
  void reset();

  // 对整段时变速度序列执行 MPPI 扰动、前向展开、加权更新和最终安全校验。
  LocalPlanResult plan(
    const PlannerVelocity2D & measured_velocity,
    const PlannerVelocity2D & previous_command,
    const PlannerVelocity2D & nominal_velocity,
    const std::vector<ObstaclePoint2D> & obstacles,
    const TrajectoryConfig & trajectory_config,
    const FootprintConfig & footprint_config,
    const MotionLimits & limits,
    const VelocitySamplingConfig & cost_config,
    const MppiConfig & mppi_config,
    bool force_linear_stop = false);

private:
  std::vector<PlannerVelocity2D> mean_sequence_;
  std::mt19937 random_engine_;
  std::uint32_t active_seed_{7U};
  bool initialized_{false};
};

}  // namespace go2_uwb_local_follow

#endif  // GO2_UWB_LOCAL_FOLLOW__MPPI_PLANNER_CORE_HPP_
