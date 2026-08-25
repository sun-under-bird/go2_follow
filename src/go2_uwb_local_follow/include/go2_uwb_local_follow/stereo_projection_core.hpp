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

#ifndef GO2_UWB_LOCAL_FOLLOW__STEREO_PROJECTION_CORE_HPP_
#define GO2_UWB_LOCAL_FOLLOW__STEREO_PROJECTION_CORE_HPP_

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace go2_uwb_local_follow
{

struct CameraCalibration
{
  double fx{0.0};
  double cx{0.0};
  double cy{0.0};
  std::size_t width{0};
  std::size_t height{0};
};

struct Point3D
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct ProjectionConfig
{
  double depth_min{0.20};
  double depth_max{3.00};
  double obstacle_x_min{-0.50};
  double obstacle_x_max{3.00};
  double obstacle_y_abs_max{2.00};
  double obstacle_z_min{0.10};
  double obstacle_z_max{0.50};
  double voxel_size{0.05};
};

// 校验相机投影参数是否可用于视差反投影。
bool validateCalibration(const CameraCalibration & calibration, std::string * reason = nullptr);

// 校验深度、空间裁剪和体素参数之间的约束关系。
bool validateProjectionConfig(const ProjectionConfig & config, std::string * reason = nullptr);

// 将一个有效视差像素反投影到左目光学坐标系。
std::optional<Point3D> projectDisparityPixel(
  std::size_t u,
  std::size_t v,
  double disparity,
  double baseline,
  const CameraCalibration & calibration,
  const ProjectionConfig & config);

// 判断机身坐标系三维点是否位于障碍物保留范围内。
bool keepBasePoint(const Point3D & point, const ProjectionConfig & config);

// 先按二维网格统计点支持数，再用三维连通簇过滤不连续伪影。
std::vector<Point3D> voxelDownsampleNearest(
  const std::vector<Point3D> & points,
  double voxel_size,
  std::size_t min_points_per_voxel = 1U,
  std::size_t min_voxels_per_cluster = 1U);

}  // namespace go2_uwb_local_follow

#endif  // GO2_UWB_LOCAL_FOLLOW__STEREO_PROJECTION_CORE_HPP_
