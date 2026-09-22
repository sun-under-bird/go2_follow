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

#ifndef GO2_UWB_LOCAL_FOLLOW__OBSERVATION_TIME_HPP_
#define GO2_UWB_LOCAL_FOLLOW__OBSERVATION_TIME_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace go2_uwb_local_follow
{

// ROS source stamps and ROS now share a clock. Receipt watchdogs use steady_clock separately.
inline double sourceAge(std::int64_t stamp, std::int64_t now, double future_tolerance = 0.05)
{
  if (stamp <= 0 || now <= 0 || !std::isfinite(future_tolerance) || future_tolerance < 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  const double age = static_cast<double>(now - stamp) * 1e-9;
  return age < -future_tolerance ? std::numeric_limits<double>::infinity() : std::max(0.0, age);
}

// A duplicate/reordered source frame must never refresh receipt time or confirmation counts.
inline bool newerObservation(std::int64_t stamp, std::int64_t previous)
{
  return stamp > 0 && stamp > previous;
}

}  // namespace go2_uwb_local_follow
#endif  // GO2_UWB_LOCAL_FOLLOW__OBSERVATION_TIME_HPP_
