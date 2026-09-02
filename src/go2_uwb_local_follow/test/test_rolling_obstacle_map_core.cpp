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
#include <cstdint>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "go2_uwb_local_follow/rolling_obstacle_map_core.hpp"

namespace rolling = go2_uwb_local_follow;

namespace
{

constexpr std::int64_t kSecond = 1000000000LL;
constexpr double kPi = 3.14159265358979323846;

}  // namespace

// 验证机身点经过 odom 变换和逆变换后保持原坐标。
TEST(RollingTransform, RoundTripsBasePoint)
{
  const rolling::TimedPose2D pose{kSecond, 1.2, -0.4, 0.7};
  const rolling::RollingObstaclePoint base_point{0.8, -0.2, 0.3};

  const auto odom_point = rolling::transformRollingPointToOdom(base_point, pose);
  const auto restored = rolling::transformRollingPointToBase(odom_point, pose);

  EXPECT_NEAR(restored.x, base_point.x, 1e-12);
  EXPECT_NEAR(restored.y, base_point.y, 1e-12);
  EXPECT_NEAR(restored.z, base_point.z, 1e-12);
}

// 验证位姿缓存按最短角度跨越正负 pi 插值。
TEST(OdomPoseBuffer, InterpolatesPositionAndWrappedYaw)
{
  rolling::RollingMapConfig config;
  rolling::OdomPoseBuffer buffer(config);
  ASSERT_EQ(
    buffer.append({kSecond, 0.0, 0.0, kPi - 0.1}),
    rolling::PoseAppendResult::kAccepted);
  ASSERT_EQ(
    buffer.append({2 * kSecond, 1.0, 2.0, -kPi + 0.1}),
    rolling::PoseAppendResult::kAccepted);

  rolling::TimedPose2D pose;
  ASSERT_TRUE(buffer.lookup(1500000000LL, &pose));
  EXPECT_NEAR(pose.x, 0.5, 1e-12);
  EXPECT_NEAR(pose.y, 1.0, 1e-12);
  EXPECT_NEAR(std::abs(pose.yaw), kPi, 1e-12);
}

// 验证点云时间稍晚于最新里程计时使用最近两帧进行有界外推。
TEST(OdomPoseBuffer, ExtrapolatesWithinConfiguredTolerance)
{
  rolling::RollingMapConfig config;
  config.max_pose_extrapolation_sec = 0.10;
  rolling::OdomPoseBuffer buffer(config);
  ASSERT_EQ(buffer.append({kSecond, 0.0, 0.0, 0.0}), rolling::PoseAppendResult::kAccepted);
  ASSERT_EQ(
    buffer.append({1100000000LL, 0.1, 0.0, 0.1}), rolling::PoseAppendResult::kAccepted);

  rolling::TimedPose2D pose;
  ASSERT_TRUE(buffer.lookup(1150000000LL, &pose));
  EXPECT_NEAR(pose.x, 0.15, 1e-12);
  EXPECT_NEAR(pose.yaw, 0.15, 1e-12);
  EXPECT_FALSE(buffer.lookup(1300000000LL, &pose));
}

// 验证启动阶段点云略早于首帧里程计时使用最早两帧进行有界反向外推。
TEST(OdomPoseBuffer, BackwardExtrapolatesWithinConfiguredTolerance)
{
  rolling::RollingMapConfig config;
  config.max_pose_extrapolation_sec = 0.10;
  rolling::OdomPoseBuffer buffer(config);
  ASSERT_EQ(buffer.append({kSecond, 0.0, 0.0, 0.0}), rolling::PoseAppendResult::kAccepted);
  ASSERT_EQ(
    buffer.append({1100000000LL, 0.1, 0.0, 0.1}), rolling::PoseAppendResult::kAccepted);

  rolling::TimedPose2D pose;
  ASSERT_TRUE(buffer.lookup(950000000LL, &pose));
  EXPECT_NEAR(pose.x, -0.05, 1e-12);
  EXPECT_NEAR(pose.yaw, -0.05, 1e-12);
  EXPECT_FALSE(buffer.lookup(800000000LL, &pose));
}

// 验证短时间里程计大跳变会重置历史缓存。
TEST(OdomPoseBuffer, ResetsOnPoseJump)
{
  rolling::RollingMapConfig config;
  config.odom_jump_distance = 0.5;
  rolling::OdomPoseBuffer buffer(config);
  ASSERT_EQ(buffer.append({kSecond, 0.0, 0.0, 0.0}), rolling::PoseAppendResult::kAccepted);

  EXPECT_EQ(
    buffer.append({1100000000LL, 2.0, 0.0, 0.0}),
    rolling::PoseAppendResult::kResetDetected);
  EXPECT_EQ(buffer.size(), 1U);
}

// 验证机器人移动后历史障碍经过里程计补偿出现在新的当前机身坐标中。
TEST(RollingObstacleMap, CompensatesRobotMotion)
{
  rolling::RollingMapConfig config;
  rolling::RollingObstacleMap map(config);
  const rolling::TimedPose2D first_pose{kSecond, 0.0, 0.0, 0.0};
  const rolling::TimedPose2D second_pose{1500000000LL, 0.4, 0.0, 0.0};
  map.integrate({{1.0, 0.2, 0.3}}, first_pose);
  map.integrate({}, second_pose);

  const auto points = map.pointsInBase(second_pose);
  ASSERT_EQ(points.size(), 1U);
  EXPECT_NEAR(points.front().x, 0.6, 1e-12);
  EXPECT_NEAR(points.front().y, 0.2, 1e-12);
}

// 验证超过保留时间且没有被新帧刷新的障碍会自动衰减删除。
TEST(RollingObstacleMap, ExpiresUnobservedObstacles)
{
  rolling::RollingMapConfig config;
  config.obstacle_retention_sec = 0.50;
  rolling::RollingObstacleMap map(config);
  map.integrate({{1.0, 0.0, 0.2}}, {kSecond, 0.0, 0.0, 0.0});
  ASSERT_EQ(map.size(), 1U);

  map.integrate({}, {1600000000LL, 0.0, 0.0, 0.0});
  EXPECT_EQ(map.size(), 0U);
}

// 验证同一 odom 体素的新观测只刷新时间和位置，不会重复累积点。
TEST(RollingObstacleMap, RefreshesExistingVoxel)
{
  rolling::RollingMapConfig config;
  config.voxel_size = 0.10;
  rolling::RollingObstacleMap map(config);
  map.integrate({{1.01, 0.01, 0.2}}, {kSecond, 0.0, 0.0, 0.0});
  map.integrate({{0.96, 0.01, 0.25}}, {1200000000LL, 0.05, 0.0, 0.0});

  EXPECT_EQ(map.size(), 1U);
  const auto points = map.pointsInBase({1200000000LL, 0.05, 0.0, 0.0});
  ASSERT_EQ(points.size(), 1U);
  EXPECT_NEAR(points.front().x, 0.96, 1e-12);
  EXPECT_NEAR(points.front().z, 0.25, 1e-12);
}

// 验证非法时间衰减参数会在节点启动前被拒绝。
TEST(RollingMapConfig, RejectsInvalidRetention)
{
  rolling::RollingMapConfig config;
  config.obstacle_retention_sec = 0.0;
  std::string reason;

  EXPECT_FALSE(rolling::validateRollingMapConfig(config, &reason));
  EXPECT_FALSE(reason.empty());
}
