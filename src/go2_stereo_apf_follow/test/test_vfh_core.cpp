#include <cmath>
#include <vector>

#include "gtest/gtest.h"

#include "go2_stereo_apf_follow/vfh_core.hpp"

namespace vfh = go2_stereo_apf_follow;

// 构造测试用的 VFH 参数，并关闭速度平滑对断言的影响。
vfh::VfhConfig make_test_config()
{
  vfh::VfhConfig config;
  config.command_filter_alpha = 1.0;
  config.max_delta_vx_per_sec = 100.0;
  config.max_delta_vy_per_sec = 100.0;
  config.max_delta_wz_per_sec = 100.0;
  config.side_switch_hold_sec = 1.2;
  config.corridor_clear_hold_sec = 0.8;
  return config;
}

TEST(VfhControl, DrivesTowardClearUwbTarget)
{
  auto config = make_test_config();
  vfh::VfhState state;

  auto result = vfh::compute_vfh_command(
    {},
    vfh::Point2D{2.0, 0.0},
    0.4,
    0.1,
    config,
    state);

  EXPECT_EQ(result.mode, vfh::VfhMode::FOLLOW);
  EXPECT_FALSE(result.corridor_blocked);
  EXPECT_GT(result.command.vx, 0.0);
  EXPECT_NEAR(result.command.vy, 0.0, 1e-6);
}

TEST(VfhControl, SelectsBypassSideWhenPersonBlocksCorridor)
{
  auto config = make_test_config();
  vfh::VfhState state;
  const std::vector<vfh::Point3D> person_points{
    {1.0, -0.15, 0.4},
    {1.0, 0.0, 0.5},
    {1.0, 0.15, 0.6},
  };

  auto result = vfh::compute_vfh_command(
    person_points,
    vfh::Point2D{3.0, 0.0},
    0.4,
    0.1,
    config,
    state);

  EXPECT_TRUE(result.corridor_blocked);
  EXPECT_NE(result.mode, vfh::VfhMode::FOLLOW);
  EXPECT_GT(std::abs(result.selected_heading), 0.2);
  EXPECT_LT(std::abs(result.selected_heading), 1.5);
  EXPECT_NE(result.command.vy, 0.0);
}

TEST(VfhControl, KeepsLockedBypassSideDuringPointJitter)
{
  auto config = make_test_config();
  vfh::VfhState state;

  auto first = vfh::compute_vfh_command(
    std::vector<vfh::Point3D>{{1.0, -0.10, 0.4}, {1.0, 0.05, 0.5}},
    vfh::Point2D{3.0, 0.0},
    0.4,
    0.1,
    config,
    state);
  const auto locked_side = first.locked_side;

  auto second = vfh::compute_vfh_command(
    std::vector<vfh::Point3D>{{1.0, -0.05, 0.4}, {1.0, 0.12, 0.5}},
    vfh::Point2D{3.0, 0.0},
    0.4,
    0.5,
    config,
    state);

  EXPECT_NE(locked_side, vfh::BypassSide::NONE);
  EXPECT_EQ(second.locked_side, locked_side);
  EXPECT_EQ(second.mode, first.mode);
}

TEST(VfhControl, StopsInsideHardStopDistance)
{
  auto config = make_test_config();
  vfh::VfhState state;

  auto result = vfh::compute_vfh_command(
    std::vector<vfh::Point3D>{{0.20, 0.0, 0.4}},
    vfh::Point2D{3.0, 0.0},
    0.4,
    0.1,
    config,
    state);

  EXPECT_TRUE(result.hard_stop);
  EXPECT_DOUBLE_EQ(result.command.vx, 0.0);
  EXPECT_DOUBLE_EQ(result.command.vy, 0.0);
  EXPECT_DOUBLE_EQ(result.command.wz, 0.0);
}

TEST(VfhControl, ReturnsToFollowAfterCorridorStaysClear)
{
  auto config = make_test_config();
  vfh::VfhState state;

  auto blocked = vfh::compute_vfh_command(
    std::vector<vfh::Point3D>{{1.0, 0.0, 0.4}},
    vfh::Point2D{3.0, 0.0},
    0.4,
    0.1,
    config,
    state);
  ASSERT_NE(blocked.mode, vfh::VfhMode::FOLLOW);

  auto clearing = vfh::compute_vfh_command(
    {},
    vfh::Point2D{3.0, 0.0},
    0.4,
    0.5,
    config,
    state);
  EXPECT_NE(clearing.mode, vfh::VfhMode::FOLLOW);

  auto clear = vfh::compute_vfh_command(
    {},
    vfh::Point2D{3.0, 0.0},
    0.4,
    1.4,
    config,
    state);
  EXPECT_EQ(clear.mode, vfh::VfhMode::FOLLOW);
}
