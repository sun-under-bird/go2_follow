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
#include <limits>
#include <vector>

#include "gtest/gtest.h"

#include "go2_uwb_local_follow/stereo_projection_core.hpp"

namespace stereo = go2_uwb_local_follow;

// 验证主点像素按 Z=fT/d 投影到光学轴中心。
TEST(StereoProjection, ProjectsPrincipalPoint)
{
  stereo::CameraCalibration calibration{400.0, 320.0, 240.0, 640U, 480U};
  stereo::ProjectionConfig config;
  const auto point = stereo::projectDisparityPixel(320U, 240U, 20.0, 0.05, calibration, config);

  ASSERT_TRUE(point.has_value());
  EXPECT_NEAR(point->x, 0.0, 1e-12);
  EXPECT_NEAR(point->y, 0.0, 1e-12);
  EXPECT_NEAR(point->z, 1.0, 1e-12);
}

// 验证超出深度范围或非有限视差不会生成三维点。
TEST(StereoProjection, RejectsInvalidAndOutOfRangeDisparity)
{
  stereo::CameraCalibration calibration{400.0, 320.0, 240.0, 640U, 480U};
  stereo::ProjectionConfig config;

  EXPECT_FALSE(stereo::projectDisparityPixel(0U, 0U, 0.0, 0.05, calibration, config));
  EXPECT_FALSE(
    stereo::projectDisparityPixel(
      0U, 0U, std::numeric_limits<double>::quiet_NaN(), 0.05, calibration, config));
  EXPECT_FALSE(stereo::projectDisparityPixel(0U, 0U, 2.0, 0.05, calibration, config));
}

// 验证机身高度过滤保留 0.10 m 和 0.50 m 两个边界。
TEST(BasePointFilter, KeepsInclusiveHeightBoundaries)
{
  stereo::ProjectionConfig config;

  EXPECT_TRUE(stereo::keepBasePoint({1.0, 0.0, 0.10}, config));
  EXPECT_TRUE(stereo::keepBasePoint({1.0, 0.0, 0.50}, config));
  EXPECT_FALSE(stereo::keepBasePoint({1.0, 0.0, 0.099}, config));
  EXPECT_FALSE(stereo::keepBasePoint({1.0, 0.0, 0.501}, config));
}

// 验证有效地面深度只作为清除射线终点，而高处点不会参与二维清除。
TEST(BasePointFilter, KeepsGroundForRayClearingOnly)
{
  stereo::ProjectionConfig config;

  EXPECT_FALSE(stereo::keepBasePoint({1.0, 0.0, 0.0}, config));
  EXPECT_TRUE(stereo::keepRayEndpoint({1.0, 0.0, 0.0}, config));
  EXPECT_TRUE(stereo::keepRayEndpoint({1.0, 0.0, 0.50}, config));
  EXPECT_FALSE(stereo::keepRayEndpoint({1.0, 0.0, 0.60}, config));
}

// 验证同一二维体素只保留平面距离机器人最近的点。
TEST(VoxelDownsample, KeepsNearestPointInEachCell)
{
  const std::vector<stereo::Point3D> points{
    {1.04, 0.01, 0.20},
    {1.01, 0.02, 0.30},
    {1.11, 0.01, 0.40}};

  const auto output = stereo::voxelDownsampleNearest(points, 0.05);

  ASSERT_EQ(output.size(), 2U);
  EXPECT_NEAR(output[0].x, 1.01, 1e-12);
  EXPECT_NEAR(output[1].x, 1.11, 1e-12);
}

// 验证支持点不足的孤立体素会被删除，而真实密集体素仍保留最近点。
TEST(VoxelDownsample, RemovesVoxelsWithoutEnoughPointSupport)
{
  const std::vector<stereo::Point3D> points{
    {1.01, 0.01, 0.20},
    {1.02, 0.02, 0.30},
    {1.03, 0.03, 0.40},
    {1.11, 0.01, 0.20}};

  const auto output = stereo::voxelDownsampleNearest(points, 0.05, 3U);

  ASSERT_EQ(output.size(), 1U);
  EXPECT_NEAR(output.front().x, 1.01, 1e-12);
}

// 验证双体素小伪影被删除，而三个相邻体素组成的障碍簇仍完整保留。
TEST(VoxelDownsample, RemovesClustersWithTooFewVoxels)
{
  std::vector<stereo::Point3D> points;
  const auto append_supported_cell = [&points](double x) {
      points.push_back({x, 0.01, 0.20});
      points.push_back({x + 0.005, 0.015, 0.30});
      points.push_back({x + 0.010, 0.020, 0.40});
    };
  append_supported_cell(1.01);
  append_supported_cell(1.06);
  append_supported_cell(2.01);
  append_supported_cell(2.06);
  append_supported_cell(2.11);

  const auto output = stereo::voxelDownsampleNearest(points, 0.05, 3U, 3U);

  ASSERT_EQ(output.size(), 3U);
  EXPECT_GT(output.front().x, 2.0);
}

// 验证俯视平面相邻但高度不连续的伪点不会被合并成障碍簇。
TEST(VoxelDownsample, SeparatesPlanarNeighborsAtDifferentHeights)
{
  std::vector<stereo::Point3D> points;
  const auto append_supported_cell = [&points](double x, double z) {
      points.push_back({x, 0.01, z});
      points.push_back({x + 0.005, 0.015, z + 0.001});
      points.push_back({x + 0.010, 0.020, z + 0.002});
    };
  append_supported_cell(1.01, 0.10);
  append_supported_cell(1.06, 0.30);
  append_supported_cell(1.11, 0.50);

  const auto output = stereo::voxelDownsampleNearest(points, 0.05, 3U, 3U);

  EXPECT_TRUE(output.empty());
}

// 验证非法过滤参数在节点启动前即可被识别。
TEST(ProjectionConfig, RejectsInvertedHeightRange)
{
  stereo::ProjectionConfig config;
  config.obstacle_z_min = 0.60;
  config.obstacle_z_max = 0.10;
  std::string reason;

  EXPECT_FALSE(stereo::validateProjectionConfig(config, &reason));
  EXPECT_FALSE(reason.empty());
}

// 验证非法射线终点高度范围会在节点启动前被拒绝。
TEST(ProjectionConfig, RejectsInvertedRayEndpointHeightRange)
{
  stereo::ProjectionConfig config;
  config.ray_endpoint_z_min = 0.50;
  config.ray_endpoint_z_max = -0.10;
  std::string reason;

  EXPECT_FALSE(stereo::validateProjectionConfig(config, &reason));
  EXPECT_FALSE(reason.empty());
}
