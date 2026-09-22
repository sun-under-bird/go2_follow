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

#ifndef GO2_UWB_LOCAL_FOLLOW__TARGET_MOTION_CORE_HPP_
#define GO2_UWB_LOCAL_FOLLOW__TARGET_MOTION_CORE_HPP_

#include <cstdint>
#include <deque>
#include <string>

#include "go2_uwb_local_follow/follow_control_core.hpp"

namespace go2_uwb_local_follow
{

enum class HumanMotion {UNKNOWN, FORWARD, SLOWING, STOPPED, TURNING, RETURNING};
const char * humanMotionName(HumanMotion motion);

struct TargetMotionConfig
{
  double window_sec{0.40};
  double reset_gap_sec{0.60};
  double max_human_speed{3.0};
  double jump_margin{0.20};
  double innovation_limit{0.45};
  double velocity_filter_sec{0.12};
  double minimum_span_sec{0.18};
  int minimum_samples{3};
  double stop_speed{0.08};
  double moving_speed{0.16};
  double turn_angle{0.45};
  double return_angle{2.40};
  double direction_history_sec{1.50};
  double direction_consistency_angle{0.40};
  int confirmation_samples{3};
};

struct TargetMotionEstimate
{
  int64_t stamp_ns{0};
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  double heading{0.0};
  bool valid{false};
  bool direction_valid{false};
  HumanMotion motion{HumanMotion::UNKNOWN};
};

enum class TargetUpdate {ACCEPTED, WARMING, OUTLIER, INVALID, DUPLICATE};
const char * targetUpdateName(TargetUpdate update);
bool validateTargetMotionConfig(const TargetMotionConfig & config);

// All samples are in the same continuous odom frame and use source timestamps.
class TargetMotionEstimator
{
public:
  explicit TargetMotionEstimator(const TargetMotionConfig & config = TargetMotionConfig{});
  TargetUpdate observe(double x, double y, int64_t stamp_ns);
  const TargetMotionEstimate & estimate() const;
  void reset();

private:
  struct Sample {int64_t stamp_ns; double x; double y;};
  struct Direction {int64_t stamp_ns; double heading;};
  TargetMotionConfig config_;
  TargetMotionEstimate estimate_;
  std::deque<Sample> samples_;
  std::deque<Direction> directions_;
  int64_t last_input_stamp_{0};
  double filtered_vx_{0.0};
  double filtered_vy_{0.0};
  double pending_heading_{0.0};
  int heading_count_{0};
  HumanMotion pending_motion_{HumanMotion::UNKNOWN};
  int motion_count_{0};
};

struct WalkHysteresisConfig
{
  double start_distance_error{0.18};
  double stop_distance_error{0.06};
  double close_distance_margin{0.05};
  double stop_command_speed{0.15};
  double approach_speed{0.15};
  double confirmation_sec{0.15};
  double minimum_hold_sec{0.30};
};

struct WalkState
{
  bool walking{false};
  int64_t last_source_stamp{0};
  int64_t pending_since_ns{0};
  int pending_samples{0};
  int64_t hold_until_ns{0};
};

struct PredictiveFollowResult
{
  FollowResult follow;
  double feedforward{0.0};
  double desired_speed{0.0};
  bool approaching{false};
  std::string reason{"HOLD"};
};

bool validateWalkHysteresisConfig(const WalkHysteresisConfig & config);
PredictiveFollowResult computePredictiveFollow(
  double target_x, double target_y, double human_vx_base, double human_vy_base,
  int64_t source_stamp_ns, int64_t control_stamp_ns,
  const FollowConfig & follow_config, const WalkHysteresisConfig & walk_config, WalkState & state,
  double measured_linear_speed = 0.0);

}  // namespace go2_uwb_local_follow
#endif  // GO2_UWB_LOCAL_FOLLOW__TARGET_MOTION_CORE_HPP_
