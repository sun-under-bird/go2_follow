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

#include "go2_uwb_local_follow/obstacle_index.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace go2_uwb_local_follow
{

// 复制有限点，预留节点内存并构建一次空间索引。
ObstacleIndex::ObstacleIndex(const std::vector<ObstaclePoint2D> & obstacles)
{
  points_.reserve(obstacles.size());
  for (const auto & point : obstacles) {
    if (std::isfinite(point.x) && std::isfinite(point.y)) {
      points_.push_back(point);
    }
  }
  nodes_.reserve(points_.size());
  if (!points_.empty()) {
    build(0U, points_.size());
  }
}

// 按包围盒最长轴二分，避免长走廊场景产生失衡索引。
int ObstacleIndex::build(std::size_t begin, std::size_t end)
{
  Node node;
  node.begin = begin;
  node.end = end;
  node.min_x = node.max_x = points_[begin].x;
  node.min_y = node.max_y = points_[begin].y;
  for (std::size_t i = begin + 1U; i < end; ++i) {
    node.min_x = std::min(node.min_x, points_[i].x);
    node.max_x = std::max(node.max_x, points_[i].x);
    node.min_y = std::min(node.min_y, points_[i].y);
    node.max_y = std::max(node.max_y, points_[i].y);
  }
  const int index = static_cast<int>(nodes_.size());
  nodes_.push_back(node);
  if (end - begin > 12U) {
    const bool split_x = node.max_x - node.min_x >= node.max_y - node.min_y;
    const std::size_t middle = begin + (end - begin) / 2U;
    std::nth_element(
      points_.begin() + begin, points_.begin() + middle, points_.begin() + end,
      [split_x](const ObstaclePoint2D & first, const ObstaclePoint2D & second) {
        return split_x ? first.x < second.x : first.y < second.y;
      });
    const int left = build(begin, middle);
    const int right = build(middle, end);
    nodes_[index].left = left;
    nodes_[index].right = right;
  }
  return index;
}

// 外接矩形距离只用于剪枝，真正的安全距离始终使用旋转后的机器人足迹。
double ObstacleIndex::lowerBoundSquared(const Node & node, const Query & query) const
{
  const double dx = std::max(
    {
      0.0, node.min_x - query.pose.x - query.extent_x,
      query.pose.x - query.extent_x - node.max_x});
  const double dy = std::max(
    {
      0.0, node.min_y - query.pose.y - query.extent_y,
      query.pose.y - query.extent_y - node.max_y});
  return dx * dx + dy * dy;
}

// 复用当前姿态的三角函数并以平方距离剪枝，省去逐点 hypot 和重复旋转准备。
void ObstacleIndex::search(int index, const Query & query, double * best_squared) const
{
  const Node & node = nodes_[index];
  if (lowerBoundSquared(node, query) > *best_squared + 1.0e-12) {
    return;
  }
  if (node.left < 0) {
    for (std::size_t i = node.begin; i < node.end; ++i) {
      const double dx = points_[i].x - query.pose.x;
      const double dy = points_[i].y - query.pose.y;
      const double x = std::max(
        0.0, std::abs(query.cosine * dx + query.sine * dy) - query.half_length);
      const double y = std::max(
        0.0, std::abs(-query.sine * dx + query.cosine * dy) - query.half_width);
      *best_squared = std::min(*best_squared, x * x + y * y);
      if (*best_squared == 0.0) {
        return;
      }
    }
    return;
  }
  const bool left_first = lowerBoundSquared(nodes_[node.left], query) <=
    lowerBoundSquared(nodes_[node.right], query);
  search(left_first ? node.left : node.right, query, best_squared);
  if (*best_squared > 0.0) {
    search(left_first ? node.right : node.left, query, best_squared);
  }
}

// 精确检查每个预测姿态，遇到首个碰撞即返回，保持原接口语义。
CollisionResult ObstacleIndex::check(
  const std::vector<PlannerPose2D> & poses, const FootprintConfig & footprint) const
{
  CollisionResult result;
  if (nodes_.empty()) {
    return result;
  }
  double best_squared = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0U; i < poses.size(); ++i) {
    Query query;
    query.pose = poses[i];
    query.cosine = std::cos(query.pose.yaw);
    query.sine = std::sin(query.pose.yaw);
    query.half_length = footprint.robot_length * 0.5 + footprint.safety_margin;
    query.half_width = footprint.robot_width * 0.5 + footprint.safety_margin;
    query.extent_x = std::abs(query.cosine) * query.half_length +
      std::abs(query.sine) * query.half_width;
    query.extent_y = std::abs(query.sine) * query.half_length +
      std::abs(query.cosine) * query.half_width;
    search(0, query, &best_squared);
    if (best_squared == 0.0) {
      result.collision = true;
      result.collision_pose_index = i;
      break;
    }
  }
  result.min_clearance = std::sqrt(best_squared);
  return result;
}

}  // namespace go2_uwb_local_follow
