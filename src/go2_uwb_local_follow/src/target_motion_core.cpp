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

#include "go2_uwb_local_follow/target_motion_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace go2_uwb_local_follow
{
namespace
{
double angleDifference(double first, double second)
{
  return std::atan2(std::sin(first - second), std::cos(first - second));
}
}  // namespace

const char * humanMotionName(HumanMotion motion)
{
  switch (motion) {
    case HumanMotion::FORWARD: return "FORWARD";
    case HumanMotion::SLOWING: return "SLOWING";
    case HumanMotion::STOPPED: return "STOPPED";
    case HumanMotion::TURNING: return "TURNING";
    case HumanMotion::RETURNING: return "RETURNING";
    default: return "UNKNOWN";
  }
}

const char * targetUpdateName(TargetUpdate update)
{
  switch (update) {
    case TargetUpdate::ACCEPTED: return "ACCEPTED";
    case TargetUpdate::WARMING: return "WARMING";
    case TargetUpdate::OUTLIER: return "OUTLIER";
    case TargetUpdate::DUPLICATE: return "DUPLICATE";
    default: return "INVALID";
  }
}

bool validateTargetMotionConfig(const TargetMotionConfig & c)
{
  const double values[] = {c.window_sec, c.reset_gap_sec, c.max_human_speed,
    c.jump_margin, c.innovation_limit, c.velocity_filter_sec, c.minimum_span_sec,
    c.stop_speed, c.moving_speed, c.turn_angle, c.return_angle,
    c.direction_history_sec, c.direction_consistency_angle};
  for (double value : values) {
    if (!std::isfinite(value) || value <= 0.0) {
      return false;
    }
  }
  return c.minimum_samples >= 3 && c.minimum_samples <= 100 && c.confirmation_samples >= 2 &&
         c.confirmation_samples <= 100 && c.minimum_span_sec < c.window_sec &&
         c.window_sec < c.reset_gap_sec && c.stop_speed < c.moving_speed &&
         c.moving_speed < c.max_human_speed && c.turn_angle < c.return_angle &&
         c.return_angle <= 3.141592653589793 && c.direction_consistency_angle < c.return_angle &&
         c.direction_history_sec >= c.window_sec;
}

TargetMotionEstimator::TargetMotionEstimator(const TargetMotionConfig & config)
: config_(config)
{
  if (!validateTargetMotionConfig(config_)) {
    throw std::invalid_argument("invalid target estimator config");
  }
}

void TargetMotionEstimator::reset()
{
  estimate_ = TargetMotionEstimate{};
  samples_.clear();
  directions_.clear();
  last_input_stamp_ = 0;
  filtered_vx_ = filtered_vy_ = 0.0;
  heading_count_ = motion_count_ = 0;
  pending_motion_ = HumanMotion::UNKNOWN;
}

const TargetMotionEstimate & TargetMotionEstimator::estimate() const {return estimate_;}

TargetUpdate TargetMotionEstimator::observe(double x, double y, int64_t stamp)
{
  if (stamp <= 0 || !std::isfinite(x) || !std::isfinite(y)) {return TargetUpdate::INVALID;}
  if (stamp <= last_input_stamp_) {return TargetUpdate::DUPLICATE;}
  if (!samples_.empty() && (stamp - samples_.back().stamp_ns) * 1e-9 > config_.reset_gap_sec) {
    reset();
  }
  last_input_stamp_ = stamp;
  if (!samples_.empty()) {
    const auto & last = samples_.back();
    const double dt = (stamp - last.stamp_ns) * 1e-9;
    const bool impossible_step = std::hypot(x - last.x, y - last.y) >
      config_.jump_margin + config_.max_human_speed * dt;
    const double prediction_dt = (stamp - estimate_.stamp_ns) * 1e-9;
    const bool innovation = estimate_.valid &&
      std::hypot(
      x - estimate_.x - filtered_vx_ * prediction_dt,
      y - estimate_.y - filtered_vy_ * prediction_dt) > config_.innovation_limit;
    if (impossible_step || innovation) {
      // Bad initialization cannot lock onto a lone jump; reacquisition still needs a full window.
      if (!estimate_.valid) {samples_.clear();} else {return TargetUpdate::OUTLIER;}
    }
  }
  samples_.push_back({stamp, x, y});
  while (samples_.size() > 1 &&
    ((stamp - samples_.front().stamp_ns) * 1e-9 > config_.window_sec + 1e-8 ||
    samples_.size() > 100))
  {samples_.pop_front();}
  if (samples_.size() < static_cast<std::size_t>(config_.minimum_samples) ||
    (stamp - samples_.front().stamp_ns) * 1e-9 < config_.minimum_span_sec)
  {return TargetUpdate::WARMING;}

  double mean_t = 0.0, mean_x = 0.0, mean_y = 0.0;
  for (const auto & sample : samples_) {
    mean_t += (sample.stamp_ns - stamp) * 1e-9;
    mean_x += sample.x;
    mean_y += sample.y;
  }
  mean_t /= samples_.size(); mean_x /= samples_.size(); mean_y /= samples_.size();
  double denominator = 0.0, numerator_x = 0.0, numerator_y = 0.0;
  for (const auto & sample : samples_) {
    const double t = (sample.stamp_ns - stamp) * 1e-9 - mean_t;
    denominator += t * t;
    numerator_x += t * (sample.x - mean_x);
    numerator_y += t * (sample.y - mean_y);
  }
  if (!std::isfinite(mean_x) || !std::isfinite(mean_y) ||
    !std::isfinite(numerator_x) || !std::isfinite(numerator_y))
  {return TargetUpdate::INVALID;}
  if (denominator <= 1e-12) {return TargetUpdate::WARMING;}
  double vx = numerator_x / denominator, vy = numerator_y / denominator;
  const double raw_speed = std::hypot(vx, vy);
  if (!std::isfinite(raw_speed)) {return TargetUpdate::INVALID;}
  if (raw_speed > config_.max_human_speed) {
    vx *= config_.max_human_speed / raw_speed;
    vy *= config_.max_human_speed / raw_speed;
  }
  const double dt = estimate_.valid ? (stamp - estimate_.stamp_ns) * 1e-9 : 0.0;
  const double alpha = estimate_.valid ? 1.0 - std::exp(-dt / config_.velocity_filter_sec) : 1.0;
  const double previous_speed = std::hypot(filtered_vx_, filtered_vy_);
  filtered_vx_ += alpha * (vx - filtered_vx_);
  filtered_vy_ += alpha * (vy - filtered_vy_);
  const double speed = std::hypot(filtered_vx_, filtered_vy_);
  const double heading = std::atan2(filtered_vy_, filtered_vx_);
  estimate_.x = mean_x - vx * mean_t;
  estimate_.y = mean_y - vy * mean_t;
  estimate_.stamp_ns = stamp;
  estimate_.valid = true;

  while (!directions_.empty() &&
    (stamp - directions_.front().stamp_ns) * 1e-9 > config_.direction_history_sec)
  {directions_.pop_front();}
  HumanMotion candidate = HumanMotion::FORWARD;
  if (speed <= config_.stop_speed ||
    (estimate_.motion == HumanMotion::STOPPED && speed < config_.moving_speed))
  {
    candidate = HumanMotion::STOPPED;
    heading_count_ = 0;
  } else if (speed >= config_.moving_speed) {
    if (heading_count_ == 0 ||
      std::abs(angleDifference(heading, pending_heading_)) > config_.direction_consistency_angle)
    {
      pending_heading_ = heading;
      heading_count_ = 1;
    } else {++heading_count_;}
    if (heading_count_ >= config_.confirmation_samples) {
      estimate_.heading = heading;
      estimate_.direction_valid = true;
      heading_count_ = config_.confirmation_samples;
      pending_heading_ = heading;
    }
    // An unconfirmed heading must not become the reference for a later turn/return.
    if (heading_count_ >= config_.confirmation_samples) {
      if (!directions_.empty()) {
        const double change = std::abs(angleDifference(heading, directions_.front().heading));
        if (change >= config_.return_angle) {candidate = HumanMotion::RETURNING;} else if (
          change >= config_.turn_angle) {candidate = HumanMotion::TURNING;}
      }
      directions_.push_back({stamp, heading});
      if (directions_.size() > 200) {directions_.pop_front();}
    } else {candidate = HumanMotion::UNKNOWN;}
  }
  if (candidate == HumanMotion::FORWARD && dt > 0.0 &&
    (speed - previous_speed) / dt < -0.25) {candidate = HumanMotion::SLOWING;}
  if (candidate != pending_motion_) {pending_motion_ = candidate; motion_count_ = 1;} else {
    ++motion_count_;
  }
  if (motion_count_ >= config_.confirmation_samples) {
    estimate_.motion = candidate;
    motion_count_ = config_.confirmation_samples;
  }
  estimate_.vx = estimate_.motion == HumanMotion::STOPPED ? 0.0 : filtered_vx_;
  estimate_.vy = estimate_.motion == HumanMotion::STOPPED ? 0.0 : filtered_vy_;
  return TargetUpdate::ACCEPTED;
}

bool validateWalkHysteresisConfig(const WalkHysteresisConfig & c)
{
  const double values[] = {c.start_distance_error, c.stop_distance_error, c.close_distance_margin,
    c.stop_command_speed, c.approach_speed, c.confirmation_sec, c.minimum_hold_sec};
  for (double value : values) {
    if (!std::isfinite(value) || value <= 0.0) {
      return false;
    }
  }
  return c.start_distance_error > c.stop_distance_error;
}

PredictiveFollowResult computePredictiveFollow(
  double x, double y, double vx, double vy, int64_t source_stamp, int64_t now_stamp,
  const FollowConfig & follow, const WalkHysteresisConfig & config, WalkState & state,
  double measured_linear_speed)
{
  PredictiveFollowResult result;
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(vx) || !std::isfinite(vy) ||
    source_stamp <= 0 || now_stamp <= 0 || !std::isfinite(measured_linear_speed) ||
    !validateFollowConfig(follow) || !validateWalkHysteresisConfig(config))
  {
    state = WalkState{};
    result.reason = "INVALID_TARGET_STATE";
    return result;
  }
  result.follow = computeFollowTarget(x, y, follow);
  const double distance = result.follow.distance;
  if (!std::isfinite(distance)) {
    state = WalkState{};
    result = PredictiveFollowResult{};
    result.reason = "INVALID_TARGET_STATE";
    return result;
  }
  result.feedforward = distance > 1e-6 ? vx * (x / distance) + vy * (y / distance) : 0.0;
  result.desired_speed = (result.feedforward + follow.linear_kp *
    (distance - follow.follow_distance)) * result.follow.heading_scale;
  result.approaching = result.feedforward <= -config.approach_speed;
  // Reserve braking distance only for closing motion; equal-speed walking is unaffected.
  const double closing_speed = distance > 1e-6 ?
    std::max(0.0, measured_linear_speed * x / distance - result.feedforward) : 0.0;
  const double braking_distance = closing_speed * closing_speed /
    (2.0 * std::max(1e-6, follow.max_linear_decel));
  if (!std::isfinite(result.feedforward) || !std::isfinite(result.desired_speed) ||
    !std::isfinite(braking_distance))
  {
    state = WalkState{};
    result = PredictiveFollowResult{};
    result.reason = "INVALID_TARGET_STATE";
    return result;
  }
  const bool close = distance <= std::max(
    0.0,
    follow.follow_distance - config.close_distance_margin + braking_distance);
  const bool new_sample = source_stamp > state.last_source_stamp;
  if (new_sample) {state.last_source_stamp = source_stamp;}
  if (close || result.approaching || result.follow.blind_rotation) {
    state.walking = false;
    state.pending_samples = 0;
    state.hold_until_ns = now_stamp + static_cast<int64_t>(config.minimum_hold_sec * 1e9);
    result.reason =
      close ? "TOO_CLOSE" : result.approaching ? "TARGET_APPROACHING" : "BLIND_ROTATE";
  } else {
    const bool start = now_stamp >= state.hold_until_ns &&
      (distance >= follow.follow_distance + config.start_distance_error ||
      (distance >= follow.follow_distance - config.stop_distance_error &&
      result.desired_speed >= follow.min_linear_speed));
    const bool stop = result.desired_speed <= config.stop_command_speed &&
      distance <= follow.follow_distance + config.stop_distance_error;
    const bool change = state.walking ? stop : start;
    if (!change) {state.pending_samples = 0;} else if (new_sample) {
      if (state.pending_samples == 0) {state.pending_since_ns = source_stamp;}
      ++state.pending_samples;
      if (state.pending_samples >= 2 &&
        (source_stamp - state.pending_since_ns) * 1e-9 + 1e-8 >= config.confirmation_sec)
      {
        state.walking = !state.walking;
        state.pending_samples = 0;
        if (!state.walking) {
          state.hold_until_ns = now_stamp + static_cast<int64_t>(config.minimum_hold_sec * 1e9);
        }
      }
    }
    result.reason = state.walking ? "WALKING" : "HOLD_HYSTERESIS";
  }
  result.follow.target_velocity.linear_x = state.walking ?
    std::clamp(result.desired_speed, follow.min_linear_speed, follow.max_linear_speed) : 0.0;
  result.follow.within_follow_distance = !state.walking;
  return result;
}

}  // namespace go2_uwb_local_follow
