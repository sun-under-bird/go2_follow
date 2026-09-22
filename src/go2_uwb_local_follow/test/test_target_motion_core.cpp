// Copyright 2026 OpenAI
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <limits>
#include "gtest/gtest.h"
#include "go2_uwb_local_follow/target_motion_core.hpp"
#include "go2_uwb_local_follow/rolling_obstacle_map_core.hpp"

namespace f = go2_uwb_local_follow;
namespace
{
int64_t ns(double t) {return static_cast<int64_t>(std::llround((100.0 + t) * 1e9));}
}

TEST(TargetMotion, EstimatesWorldVelocityDespiteRobotTranslationRotationAndNoise)
{
  f::TargetMotionEstimator tracker;
  for (int i = 0; i <= 80; ++i) {
    const double t = i * .05;
    f::TimedPose2D pose{ns(t), .3 * t, -.1 * t, .7 * t};
    const f::RollingObstaclePoint human{2.0 + .5 * t, 1.0 + .2 * t, 0.0, ns(t)};
    auto local = f::transformRollingPointToBase(human, pose);
    local.x += .005 * std::sin(i * 2.0);
    auto world = f::transformRollingPointToOdom(local, pose);
    tracker.observe(world.x, world.y, ns(t));
  }
  const auto e = tracker.estimate();
  ASSERT_TRUE(e.valid);
  EXPECT_NEAR(e.x, 4.0, .015);
  EXPECT_NEAR(e.y, 1.8, .015);
  EXPECT_NEAR(e.vx, .5, .025);
  EXPECT_NEAR(e.vy, .2, .025);
  EXPECT_TRUE(e.direction_valid);
  EXPECT_NEAR(e.heading, std::atan2(.2, .5), .06);
  EXPECT_EQ(e.motion, f::HumanMotion::FORWARD);
}

TEST(TargetMotion, JumpDuplicateAndReorderedPacketsDoNotRefreshEstimate)
{
  f::TargetMotionEstimator tracker;
  for (int i = 0; i <= 20; ++i) {
    tracker.observe(2 + .025 * i, 0, ns(.05 * i));
  }
  const auto before = tracker.estimate();
  EXPECT_EQ(tracker.observe(-5, 7, ns(1.05)), f::TargetUpdate::OUTLIER);
  EXPECT_EQ(tracker.observe(-5, 7, ns(1.05)), f::TargetUpdate::DUPLICATE);
  EXPECT_EQ(tracker.observe(1, 0, ns(.8)), f::TargetUpdate::DUPLICATE);
  EXPECT_EQ(tracker.estimate().stamp_ns, before.stamp_ns);
  EXPECT_EQ(tracker.estimate().motion, before.motion);
  EXPECT_EQ(tracker.observe(2.55, 0, ns(1.1)), f::TargetUpdate::ACCEPTED);
  EXPECT_NEAR(tracker.estimate().vx, .5, 1e-8);
}

TEST(TargetMotion, GapAndResetRequireFreshWarmup)
{
  f::TargetMotionEstimator tracker;
  for (int i = 0; i < 10; ++i) {
    tracker.observe(2, 0, ns(.05 * i));
  }
  ASSERT_TRUE(tracker.estimate().valid);
  EXPECT_EQ(tracker.observe(5, 0, ns(2)), f::TargetUpdate::WARMING);
  EXPECT_FALSE(tracker.estimate().valid);
  for (int i = 1; i <= 5; ++i) {
    tracker.observe(5, 0, ns(2 + .05 * i));
  }
  EXPECT_TRUE(tracker.estimate().valid);
  tracker.reset();
  EXPECT_EQ(tracker.observe(2, 0, ns(0)), f::TargetUpdate::WARMING);
  EXPECT_FALSE(tracker.estimate().direction_valid);
}

TEST(TargetMotion, DistinguishesStopTurnAndReturnWithMultipleObservations)
{
  for (int scenario = 0; scenario < 3; ++scenario) {
    f::TargetMotionEstimator tracker;
    for (int i = 0; i <= 20; ++i) {
      tracker.observe(2 + .025 * i, 0, ns(.05 * i));
    }
    bool seen = false;
    const auto expected = scenario == 0 ? f::HumanMotion::STOPPED :
      scenario == 1 ? f::HumanMotion::TURNING : f::HumanMotion::RETURNING;
    for (int i = 1; i <= 20; ++i) {
      double x = 2.5, y = 0.0;
      if (scenario == 1) {y = .025 * i;}
      if (scenario == 2) {x -= .025 * i;}
      tracker.observe(x, y, ns(1 + .05 * i));
      if (i == 1) {EXPECT_NE(tracker.estimate().motion, expected);}
      seen = seen || tracker.estimate().motion == expected;
    }
    EXPECT_TRUE(seen) << scenario;
    if (scenario == 0) {EXPECT_DOUBLE_EQ(tracker.estimate().vx, 0);}
    if (scenario == 2) {EXPECT_LT(tracker.estimate().vx, -.45);}
  }
}

TEST(TargetMotion, RecognizesDeceleration)
{
  f::TargetMotionEstimator tracker;
  for (int i = 0; i <= 20; ++i) {
    tracker.observe(2 + .05 * i, 0, ns(.05 * i));
  }
  bool slowing = false;
  for (int i = 1; i <= 20; ++i) {
    const double t = .05 * i;
    tracker.observe(3 + t - .4 * t * t, 0, ns(1 + t));
    slowing |= tracker.estimate().motion == f::HumanMotion::SLOWING;
  }
  EXPECT_TRUE(slowing);
}

TEST(TargetMotion, RejectsInvalidDataAndConfiguration)
{
  f::TargetMotionConfig bad;
  bad.minimum_samples = 1;
  EXPECT_THROW(f::TargetMotionEstimator{bad}, std::invalid_argument);
  f::TargetMotionEstimator tracker;
  EXPECT_EQ(tracker.observe(NAN, 0, ns(1)), f::TargetUpdate::INVALID);
  EXPECT_EQ(tracker.observe(2, 0, 0), f::TargetUpdate::INVALID);
  EXPECT_FALSE(tracker.estimate().valid);
}

TEST(PredictiveFollow, StartsAtDesiredDistanceUsingFeedforward)
{
  f::WalkState state;
  f::PredictiveFollowResult r;
  for (int i = 0; i < 5; ++i) {
    r = f::computePredictiveFollow(1, 0, .5, 0, ns(.05 * i), ns(.05 * i), {}, {}, state);
  }
  EXPECT_TRUE(state.walking);
  EXPECT_NEAR(r.follow.target_velocity.linear_x, .5, 1e-8);
  EXPECT_DOUBLE_EQ(f::computeFollowTarget(1, 0, {}).target_velocity.linear_x, 0);
}

TEST(PredictiveFollow, RepeatedControlCyclesCannotConfirmOneObservation)
{
  f::WalkState state;
  for (int i = 0; i < 20; ++i) {
    f::computePredictiveFollow(2, 0, .5, 0, ns(0), ns(.05 * i), {}, {}, state);
  }
  EXPECT_FALSE(state.walking);
}

TEST(PredictiveFollow, StopsWithoutChatterAndNeverCommandsIneffectiveSpeed)
{
  f::WalkState state;
  state.walking = true;
  int transitions = 0;
  bool previous = true;
  for (int i = 0; i < 300; ++i) {
    const auto r = f::computePredictiveFollow(
      1.02 + .025 * std::sin(i), 0,
      .015 * std::cos(i), 0, ns(.05 * i), ns(.05 * i), {}, {}, state);
    if (state.walking != previous) {++transitions; previous = state.walking;}
    const double v = r.follow.target_velocity.linear_x;
    EXPECT_TRUE(v == 0.0 || v >= .23);
  }
  EXPECT_FALSE(state.walking);
  EXPECT_EQ(transitions, 1);
}

TEST(PredictiveFollow, ReturningPersonDoesNotCauseForwardChase)
{
  f::WalkState state;
  state.walking = true;
  auto r = f::computePredictiveFollow(3, 0, -.5, 0, ns(0), ns(0), {}, {}, state);
  EXPECT_TRUE(r.approaching);
  EXPECT_FALSE(state.walking);
  EXPECT_DOUBLE_EQ(r.follow.target_velocity.linear_x, 0.0);
  EXPECT_EQ(r.reason, "TARGET_APPROACHING");
  r = f::computePredictiveFollow(-2, 0, .5, 0, ns(.1), ns(.1), {}, {}, state);
  EXPECT_TRUE(r.follow.blind_rotation);
  EXPECT_DOUBLE_EQ(r.follow.target_velocity.linear_x, 0);
}

TEST(PredictiveFollow, ClosedLoopUniformWalkReducesLagAndStopsOnce)
{
  double errors[2]{};
  for (int mode = 0; mode < 2; ++mode) {
    f::TargetMotionEstimator tracker;
    f::WalkState state;
    f::Velocity2D output;
    double person = 1.0, robot = 0.0;
    int late_starts = 0;
    for (int i = 0; i < 600; ++i) {
      const double t = .05 * i;
      if (t < 15) {person += .5 * .05;}
      tracker.observe(person, 0, ns(t));
      f::FollowResult desired;
      if (mode == 0) {
        desired = f::computeFollowTarget(person - robot, 0, {});
      } else if (tracker.estimate().valid) {
        desired = f::computePredictiveFollow(
          tracker.estimate().x - robot, 0,
          tracker.estimate().vx, 0, ns(t), ns(t), {}, {}, state, output.linear_x).follow;
      }
      const auto next = f::limitVelocityRate(output, desired.target_velocity, {}, .05);
      if (t > 18 && output.linear_x == 0 && next.linear_x > 0) {++late_starts;}
      output = next;
      robot += output.linear_x * .05;
      if (t >= 10 && t < 15) {errors[mode] += std::abs(person - robot - 1.0) / 100;}
    }
    if (mode == 1) {
      EXPECT_EQ(late_starts, 0);
      // Includes estimator latency and the existing acceleration-limited stop.
      EXPECT_NEAR(person - robot, 1.0, .20);
    }
  }
  EXPECT_GT(errors[0], .6);
  EXPECT_LT(errors[1], .08);
  RecordProperty("legacy_mean_distance_error_m", std::to_string(errors[0]));
  RecordProperty("stage2_mean_distance_error_m", std::to_string(errors[1]));
}

TEST(PredictiveFollow, SlowWalkerUsesDistanceBandAndZeroOrEffectiveSpeed)
{
  f::WalkState state;
  double distance = 1.0;
  int starts = 0;
  bool previous = false;
  for (int i = 0; i < 600; ++i) {
    const auto r = f::computePredictiveFollow(
      distance, 0, .1, 0,
      ns(.05 * i), ns(.05 * i), {}, {}, state);
    if (state.walking && !previous) {++starts;}
    previous = state.walking;
    const double v = r.follow.target_velocity.linear_x;
    EXPECT_TRUE(v == 0 || v >= .23);
    distance += (.1 - v) * .05;
    EXPECT_GT(distance, .85);
    EXPECT_LT(distance, 1.25);
  }
  EXPECT_GT(starts, 0);
  EXPECT_LT(starts, 15);
}

TEST(PredictiveFollow, MeasuredClosingSpeedReservesBrakingSpace)
{
  f::WalkState state;
  state.walking = true;
  const auto result = f::computePredictiveFollow(
    1.1, 0, 0, 0,
    ns(0), ns(0), {}, {}, state, .6);
  EXPECT_FALSE(state.walking);
  EXPECT_EQ(result.reason, "TOO_CLOSE");
  EXPECT_DOUBLE_EQ(result.follow.target_velocity.linear_x, 0);
}

TEST(TargetMotion, OneModerateNoisyFrameDoesNotConfirmReturn)
{
  for (double spike : {-.3, .3}) {
    f::TargetMotionEstimator tracker;
    for (int i = 0; i <= 40; ++i) {
      tracker.observe(2 + .025 * i + (i == 21 ? spike : 0), 0, ns(.05 * i));
      EXPECT_NE(tracker.estimate().motion, f::HumanMotion::RETURNING);
    }
  }
}
