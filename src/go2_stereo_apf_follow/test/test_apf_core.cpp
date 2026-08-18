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
  config.robot_frame_front = 0.15;
  config.robot_frame_back = 0.35;
  config.robot_frame_left = 0.15;
  config.robot_frame_right = 0.15;

  std::vector<apf::Point3D> points{
    {1.0, 0.0, 0.3},
    {0.1, 0.0, 0.3},
    {1.0, 2.0, 0.3},
    {1.0, 0.0, 0.0},
    {5.0, 0.0, 0.3},
  };

  auto filtered = apf::filter_points(points, config);

  ASSERT_EQ(filtered.size(), 1u);
  EXPECT_DOUBLE_EQ(filtered[0].x, 1.0);
}

TEST(ApfControl, StopsInsideEmergencyDistance)
{
  apf::ApfConfig apf_config;
  apf_config.emergency_dist = 0.2;
  apf::FollowControlConfig control_config;
  control_config.follow_distance = 0.4;
  control_config.max_vx = 1.0;

  auto summary = apf::summarize_obstacles(
    std::vector<apf::Point3D>{{0.1, 0.0, 0.3}},
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
  apf_config.corridor_width = 0.35;
  apf_config.emergency_dist = 0.2;
  apf::FollowControlConfig control_config;
  control_config.follow_distance = 0.4;
  control_config.max_vx = 1.0;
  control_config.max_vy = 1.0;

  auto summary = apf::summarize_obstacles(
    std::vector<apf::Point3D>{{1.0, 0.1, 0.3}},
    apf::Point2D{3.0, 0.0},
    apf_config);
  auto cmd = apf::compute_follow_command(apf::Point2D{3.0, 0.0}, summary, apf_config, control_config);

  EXPECT_LT(cmd.vy, 0.0);
  EXPECT_GT(cmd.vx, 0.0);
}
