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

#ifndef GO2_UWB_LOCAL_FOLLOW__OBSTACLE_INDEX_HPP_
#define GO2_UWB_LOCAL_FOLLOW__OBSTACLE_INDEX_HPP_

#include <cstddef>
#include <vector>

#include "go2_uwb_local_follow/local_planner_core.hpp"

namespace go2_uwb_local_follow
{

// 每次规划只构建一次包围盒树，供所有候选重复执行精确矩形净空查询。
class ObstacleIndex
{
public:
  // 复制有限障碍点并按最长轴建立平衡空间索引。
  explicit ObstacleIndex(const std::vector<ObstaclePoint2D> & obstacles);

  // 返回整条轨迹的精确最小净空和首次碰撞位置。
  CollisionResult check(
    const std::vector<PlannerPose2D> & poses, const FootprintConfig & footprint) const;

private:
  struct Node
  {
    double min_x{0.0};
    double max_x{0.0};
    double min_y{0.0};
    double max_y{0.0};
    std::size_t begin{0U};
    std::size_t end{0U};
    int left{-1};
    int right{-1};
  };

  struct Query
  {
    PlannerPose2D pose;
    double cosine{1.0};
    double sine{0.0};
    double half_length{0.0};
    double half_width{0.0};
    double extent_x{0.0};
    double extent_y{0.0};
  };

  // 递归分割点集合；叶子内保留少量点以减少树遍历开销。
  int build(std::size_t begin, std::size_t end);

  // 用旋转足迹的外接矩形求保守距离下界，保证不会跳过更近的障碍。
  double lowerBoundSquared(const Node & node, const Query & query) const;

  // 优先访问更近的子树，叶子使用精确旋转矩形距离更新最优值。
  void search(int index, const Query & query, double * best_squared) const;

  std::vector<ObstaclePoint2D> points_;
  std::vector<Node> nodes_;
};

}  // namespace go2_uwb_local_follow

#endif  // GO2_UWB_LOCAL_FOLLOW__OBSTACLE_INDEX_HPP_
