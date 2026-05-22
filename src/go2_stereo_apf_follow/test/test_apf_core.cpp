#include <algorithm>
#include <cmath>
#include <vector>

#include "gtest/gtest.h"

#include "go2_stereo_apf_follow/apf_core.hpp"

namespace apf = go2_stereo_apf_follow;

TEST(UwbTargetParsing, PrefersXyWhenAvailable)
{
  apf::UwbTargetConfig config;
  config.prefer_xy = true;

  auto target = apf::parse_uwb_target(1.2, -0.4, 5.0, 0.0, config);

  ASSERT_TRUE(target.has_value());
  EXPECT_DOUBLE_EQ(target->x, 1.2);
  EXPECT_DOUBLE_EQ(target->y, -0.4);
}

TEST(UwbTargetParsing, FallsBackToRangeAngle)
{
  apf::UwbTargetConfig config;
  config.prefer_xy = false;
  config.angle_in_degrees = true;

  auto target = apf::parse_uwb_target(0.0, 0.0, 2.0, 90.0, config);

  ASSERT_TRUE(target.has_value());
  EXPECT_NEAR(target->x, 0.0, 1e-9);
  EXPECT_NEAR(target->y, 2.0, 1e-9);
}

TEST(PointFiltering, RemovesGroundFarAndRobotFramePoints)
{
  apf::PointFilterConfig config;
  config.x_max = 4.0;
  config.y_abs = 1.5;
  config.z_min = 0.05;
  config.z_max = 1.2;
  config.robot_frame_front = 0.35;
  config.robot_frame_back = 0.35;
  config.robot_frame_left = 0.20;
  config.robot_frame_right = 0.20;

  std::vector<apf::Point3D> points{
    {1.0, 0.0, 0.3},
    {0.2, 0.0, 0.3},
    {1.0, 2.0, 0.3},
    {1.0, 0.0, 0.0},
    {5.0, 0.0, 0.3},
  };

  auto filtered = apf::filter_points(points, config);

  ASSERT_EQ(filtered.size(), 1u);
  EXPECT_DOUBLE_EQ(filtered[0].x, 1.0);
}

TEST(TargetTracking, ComputesCentroidAroundCurrentTarget)
{
  apf::TargetTrackingConfig config;
  config.target_radius = 0.3;
  config.min_points_in_target = 2;

  std::vector<apf::Point3D> points{
    {2.0, 0.1, 0.4},
    {2.1, -0.1, 0.4},
    {3.0, 0.0, 0.4},
  };

  auto centroid = apf::compute_target_centroid(points, apf::Point2D{2.0, 0.0}, config);

  ASSERT_TRUE(centroid.has_value());
  EXPECT_NEAR(centroid->x, 2.05, 1e-9);
  EXPECT_NEAR(centroid->y, 0.0, 1e-9);
}

TEST(TargetTracking, RejectsSparseTargetCluster)
{
  apf::TargetTrackingConfig config;
  config.target_radius = 0.3;
  config.min_points_in_target = 2;

  std::vector<apf::Point3D> points{{2.0, 0.1, 0.4}};

  auto centroid = apf::compute_target_centroid(points, apf::Point2D{2.0, 0.0}, config);

  EXPECT_FALSE(centroid.has_value());
}

TEST(LocalObstacleMap, MaintainsFourByFourGridAndPublishesOccupiedCells)
{
  apf::LocalMapConfig config;
  config.width_m = 4.0;
  config.height_m = 4.0;
  config.resolution = 1.0;
  config.origin_x = 0.0;
  config.origin_y = -2.0;
  config.obstacle_hold_sec = 0.6;
  config.min_points_per_cell = 2;
  apf::LocalObstacleMap map(config);

  EXPECT_EQ(map.width_cells(), 4);
  EXPECT_EQ(map.height_cells(), 4);

  map.update(
    std::vector<apf::Point3D>{
      {1.1, 0.1, 0.3},
      {1.2, 0.2, 0.3},
      {3.9, 1.9, 0.3},
    },
    10.0);

  const auto occupied = map.occupied_points(10.1);
  ASSERT_EQ(occupied.size(), 1u);
  EXPECT_NEAR(occupied[0].x, 1.5, 1e-9);
  EXPECT_NEAR(occupied[0].y, 0.5, 1e-9);

  const auto data = map.occupancy_data(10.1);
  EXPECT_EQ(data.size(), 16u);
  EXPECT_EQ(std::count(data.begin(), data.end(), 100), 1);
}

TEST(LocalObstacleMap, DropsCellsAfterHoldTimeout)
{
  apf::LocalMapConfig config;
  config.resolution = 1.0;
  config.obstacle_hold_sec = 0.5;
  apf::LocalObstacleMap map(config);

  map.update(std::vector<apf::Point3D>{{1.0, 0.0, 0.3}}, 2.0);

  EXPECT_EQ(map.occupied_points(2.4).size(), 1u);
  map.prune(2.6);
  EXPECT_TRUE(map.occupied_points(2.6).empty());
}

TEST(ApfControl, StopsInsideEmergencyDistance)
{
  apf::ApfConfig apf_config;
  apf_config.emergency_dist = 0.45;
  apf::FollowControlConfig control_config;
  control_config.follow_distance = 2.0;
  control_config.max_vx = 0.3;

  auto summary = apf::summarize_obstacles(
    std::vector<apf::Point3D>{{0.3, 0.0, 0.3}},
    apf::Point2D{3.0, 0.0},
    apf_config);
  auto cmd = apf::compute_follow_command(apf::Point2D{3.0, 0.0}, summary, apf_config, control_config);

  EXPECT_DOUBLE_EQ(cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(cmd.wz, 0.0);
}

TEST(ApfControl, AppliesLateralCorridorCorrection)
{
  apf::ApfConfig apf_config;
  apf_config.corridor_width = 0.7;
  apf_config.emergency_dist = 0.2;
  apf::FollowControlConfig control_config;
  control_config.follow_distance = 2.0;
  control_config.max_vx = 0.3;
  control_config.max_vy = 0.3;

  auto summary = apf::summarize_obstacles(
    std::vector<apf::Point3D>{{1.0, 0.2, 0.3}},
    apf::Point2D{3.0, 0.0},
    apf_config);
  auto cmd = apf::compute_follow_command(apf::Point2D{3.0, 0.0}, summary, apf_config, control_config);

  EXPECT_LT(cmd.vy, 0.0);
  EXPECT_GT(cmd.vx, 0.0);
}

TEST(SafetyLimits, ClipsAndRampsCommands)
{
  apf::SafetyLimitConfig limits;
  limits.max_vx = 0.3;
  limits.max_vy = 0.2;
  limits.max_wz = 0.8;
  limits.max_reverse_vx = 0.0;
  limits.max_accel_vx = 0.5;

  auto clipped = apf::clip_command(apf::TwistCommand{1.0, -1.0, 2.0}, limits);
  EXPECT_DOUBLE_EQ(clipped.vx, 0.3);
  EXPECT_DOUBLE_EQ(clipped.vy, -0.2);
  EXPECT_DOUBLE_EQ(clipped.wz, 0.8);

  auto ramped = apf::ramp_command(apf::TwistCommand{0.0, 0.0, 0.0}, clipped, 0.1, limits);
  EXPECT_NEAR(ramped.vx, 0.05, 1e-9);
}
