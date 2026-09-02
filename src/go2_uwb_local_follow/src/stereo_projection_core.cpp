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

#include "go2_uwb_local_follow/stereo_projection_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace go2_uwb_local_follow
{
namespace
{

struct CellKey2D
{
  std::int64_t x{0};
  std::int64_t y{0};

  // 比较两个二维体素索引是否相同。
  bool operator==(const CellKey2D & other) const
  {
    return x == other.x && y == other.y;
  }
};

struct CellKey2DHash
{
  // 组合二维整数索引的哈希值。
  std::size_t operator()(const CellKey2D & key) const
  {
    const auto first = std::hash<std::int64_t>{}(key.x);
    const auto second = std::hash<std::int64_t>{}(key.y);
    return first ^ (second + 0x9e3779b97f4a7c15ULL + (first << 6U) + (first >> 2U));
  }
};

struct CellKey3D
{
  std::int64_t x{0};
  std::int64_t y{0};
  std::int64_t z{0};

  // 比较两个三维体素索引是否相同。
  bool operator==(const CellKey3D & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct CellKey3DHash
{
  // 组合三维整数索引的哈希值。
  std::size_t operator()(const CellKey3D & key) const
  {
    const auto first = std::hash<std::int64_t>{}(key.x);
    const auto second = std::hash<std::int64_t>{}(key.y);
    const auto third = std::hash<std::int64_t>{}(key.z);
    const auto planar =
      first ^ (second + 0x9e3779b97f4a7c15ULL + (first << 6U) + (first >> 2U));
    return planar ^ (third + 0x9e3779b97f4a7c15ULL + (planar << 6U) + (planar >> 2U));
  }
};

struct CellValue
{
  Point3D nearest;
  std::size_t point_count{0U};
};

// 判断三维点的全部坐标是否为有限值。
bool finitePoint(const Point3D & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

// 在需要时写入参数校验失败原因。
bool rejectWithReason(const std::string & message, std::string * reason)
{
  if (reason != nullptr) {
    *reason = message;
  }
  return false;
}

}  // namespace

// 校验相机投影参数是否可用于视差反投影。
bool validateCalibration(const CameraCalibration & calibration, std::string * reason)
{
  if (!std::isfinite(calibration.fx) || !std::isfinite(calibration.cx) ||
    !std::isfinite(calibration.cy))
  {
    return rejectWithReason("camera calibration contains non-finite values", reason);
  }
  if (calibration.fx <= 0.0) {
    return rejectWithReason("camera focal length must be positive", reason);
  }
  if (calibration.width == 0U || calibration.height == 0U) {
    return rejectWithReason("camera image dimensions must be non-zero", reason);
  }
  return true;
}

// 校验深度、空间裁剪和体素参数之间的约束关系。
bool validateProjectionConfig(const ProjectionConfig & config, std::string * reason)
{
  const bool finite =
    std::isfinite(config.depth_min) && std::isfinite(config.depth_max) &&
    std::isfinite(config.obstacle_x_min) && std::isfinite(config.obstacle_x_max) &&
    std::isfinite(config.obstacle_y_abs_max) && std::isfinite(config.obstacle_z_min) &&
    std::isfinite(config.obstacle_z_max) && std::isfinite(config.ray_endpoint_z_min) &&
    std::isfinite(config.ray_endpoint_z_max) && std::isfinite(config.voxel_size);
  if (!finite) {
    return rejectWithReason("projection config contains non-finite values", reason);
  }
  if (config.depth_min <= 0.0 || config.depth_max <= config.depth_min) {
    return rejectWithReason("depth range must satisfy 0 < min < max", reason);
  }
  if (config.obstacle_x_max <= config.obstacle_x_min) {
    return rejectWithReason("obstacle x range must satisfy min < max", reason);
  }
  if (config.obstacle_y_abs_max <= 0.0) {
    return rejectWithReason("obstacle lateral range must be positive", reason);
  }
  if (config.obstacle_z_max < config.obstacle_z_min) {
    return rejectWithReason("obstacle z range must satisfy min <= max", reason);
  }
  if (config.ray_endpoint_z_max < config.ray_endpoint_z_min) {
    return rejectWithReason("ray endpoint z range must satisfy min <= max", reason);
  }
  if (config.voxel_size <= 0.0) {
    return rejectWithReason("voxel size must be positive", reason);
  }
  return true;
}

// 将一个有效视差像素反投影到左目光学坐标系。
std::optional<Point3D> projectDisparityPixel(
  std::size_t u,
  std::size_t v,
  double disparity,
  double baseline,
  const CameraCalibration & calibration,
  const ProjectionConfig & config)
{
  if (!std::isfinite(disparity) || !std::isfinite(baseline) || disparity <= 0.0 ||
    baseline <= 0.0 || u >= calibration.width || v >= calibration.height)
  {
    return std::nullopt;
  }

  // 视差消息已经补偿左右主点差，深度可直接使用 Z=fT/d 计算。
  const double depth = calibration.fx * baseline / disparity;
  if (!std::isfinite(depth) || depth < config.depth_min || depth > config.depth_max) {
    return std::nullopt;
  }

  Point3D point;
  point.x = (static_cast<double>(u) - calibration.cx) * depth / calibration.fx;
  point.y = (static_cast<double>(v) - calibration.cy) * depth / calibration.fx;
  point.z = depth;
  return finitePoint(point) ? std::optional<Point3D>(point) : std::nullopt;
}

// 判断机身坐标系三维点是否位于障碍物保留范围内。
bool keepBasePoint(const Point3D & point, const ProjectionConfig & config)
{
  if (!finitePoint(point)) {
    return false;
  }
  return point.x >= config.obstacle_x_min && point.x <= config.obstacle_x_max &&
         std::abs(point.y) <= config.obstacle_y_abs_max &&
         point.z >= config.obstacle_z_min && point.z <= config.obstacle_z_max;
}

// 判断有效深度终点是否位于可用于二维自由空间清除的高度和范围内。
bool keepRayEndpoint(const Point3D & point, const ProjectionConfig & config)
{
  if (!finitePoint(point)) {
    return false;
  }
  return point.x >= config.obstacle_x_min && point.x <= config.obstacle_x_max &&
         std::abs(point.y) <= config.obstacle_y_abs_max &&
         point.z >= config.ray_endpoint_z_min && point.z <= config.ray_endpoint_z_max;
}

// 先按二维网格统计点支持数，再用三维 26 邻域聚簇删除不连续的小伪影。
std::vector<Point3D> voxelDownsampleNearest(
  const std::vector<Point3D> & points,
  double voxel_size,
  std::size_t min_points_per_voxel,
  std::size_t min_voxels_per_cluster)
{
  if (!std::isfinite(voxel_size) || voxel_size <= 0.0 || min_points_per_voxel == 0U ||
    min_voxels_per_cluster == 0U)
  {
    return {};
  }

  std::unordered_map<CellKey2D, CellValue, CellKey2DHash> cells;
  cells.reserve(points.size());
  for (const auto & point : points) {
    if (!finitePoint(point)) {
      continue;
    }
    const CellKey2D key{
      static_cast<std::int64_t>(std::floor(point.x / voxel_size)),
      static_cast<std::int64_t>(std::floor(point.y / voxel_size))};
    const auto found = cells.find(key);
    const double range_squared = point.x * point.x + point.y * point.y;
    if (found == cells.end()) {
      cells.emplace(key, CellValue{point, 1U});
      continue;
    }
    ++found->second.point_count;
    const double stored_range_squared =
      found->second.nearest.x * found->second.nearest.x +
      found->second.nearest.y * found->second.nearest.y;
    if (range_squared < stored_range_squared) {
      found->second.nearest = point;
    }
  }

  std::unordered_map<CellKey3D, CellKey2D, CellKey3DHash> supported_cells;
  supported_cells.reserve(cells.size());
  for (const auto & item : cells) {
    if (item.second.point_count < min_points_per_voxel) {
      continue;
    }
    const CellKey3D spatial_key{
      item.first.x,
      item.first.y,
      static_cast<std::int64_t>(std::floor(item.second.nearest.z / voxel_size))};
    supported_cells.emplace(spatial_key, item.first);
  }

  std::unordered_set<CellKey3D, CellKey3DHash> visited;
  visited.reserve(supported_cells.size());
  std::vector<Point3D> output;
  output.reserve(supported_cells.size());
  for (const auto & item : supported_cells) {
    if (visited.count(item.first) != 0U) {
      continue;
    }

    std::vector<CellKey3D> component;
    std::vector<CellKey3D> pending{item.first};
    visited.insert(item.first);
    while (!pending.empty()) {
      const CellKey3D current = pending.back();
      pending.pop_back();
      component.push_back(current);
      // 必须同时在 x、y、z 上连续，避免不同高度的伪点在俯视平面被误连接。
      for (std::int64_t offset_x = -1; offset_x <= 1; ++offset_x) {
        for (std::int64_t offset_y = -1; offset_y <= 1; ++offset_y) {
          for (std::int64_t offset_z = -1; offset_z <= 1; ++offset_z) {
            if (offset_x == 0 && offset_y == 0 && offset_z == 0) {
              continue;
            }
            const CellKey3D neighbor{
              current.x + offset_x,
              current.y + offset_y,
              current.z + offset_z};
            if (supported_cells.count(neighbor) == 0U || visited.count(neighbor) != 0U) {
              continue;
            }
            visited.insert(neighbor);
            pending.push_back(neighbor);
          }
        }
      }
    }
    if (component.size() < min_voxels_per_cluster) {
      continue;
    }
    for (const auto & key : component) {
      output.push_back(cells.at(supported_cells.at(key)).nearest);
    }
  }
  std::sort(
    output.begin(), output.end(),
    [](const Point3D & first, const Point3D & second) {
      if (first.x != second.x) {
        return first.x < second.x;
      }
      if (first.y != second.y) {
        return first.y < second.y;
      }
      return first.z < second.z;
    });
  return output;
}

}  // namespace go2_uwb_local_follow
