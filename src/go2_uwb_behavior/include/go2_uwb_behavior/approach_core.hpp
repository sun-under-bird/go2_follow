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

#ifndef GO2_UWB_BEHAVIOR__APPROACH_CORE_HPP_
#define GO2_UWB_BEHAVIOR__APPROACH_CORE_HPP_

#include <cmath>
#include <cstdint>

namespace go2_uwb_behavior
{

struct ArrivalConfig
{
  double stop_distance{0.5};
  double minimum_distance{0.5};
  double tolerance{0.1};
  double stable_sec{0.5};
  double maximum_sample_gap_sec{0.5};
  bool require_heading{false};
  double heading_tolerance{0.2};
};

// 只统计指定目标的新采集样本；控制循环反复读同一帧不能延长到达时间。
class ArrivalMonitor
{
public:
  // 创建独立任务的到达监视器，不对位置做任何滤波。
  explicit ArrivalMonitor(const ArrivalConfig & config = ArrivalConfig())
  : config_(config) {}

  // 检查距离和可选朝向；几何安全间距始终优先于到达容差。
  bool inRange(double distance, double heading) const
  {
    return std::isfinite(distance) && std::isfinite(heading) &&
           distance >= config_.minimum_distance &&
           distance <= config_.stop_distance + config_.tolerance &&
           (!config_.require_heading || std::abs(heading) <= config_.heading_tolerance);
  }

  // 用新采集时间累计连续稳定区间；不达标、时间倒退或采集间隔过大均重新计时。
  bool update(double distance, double heading, std::int64_t stamp_ns)
  {
    if (!inRange(distance, heading) || stamp_ns <= 0 ||
      (last_stamp_ns_ != 0 && stamp_ns < last_stamp_ns_))
    {
      reset();
      return false;
    }
    if (last_stamp_ns_ == stamp_ns) {
      return stable_;
    }
    if (last_stamp_ns_ == 0 ||
      static_cast<double>(stamp_ns - last_stamp_ns_) * 1e-9 >
      config_.maximum_sample_gap_sec)
    {
      first_stamp_ns_ = stamp_ns;
    }
    last_stamp_ns_ = stamp_ns;
    stable_ = static_cast<double>(stamp_ns - first_stamp_ns_) * 1e-9 >= config_.stable_sec;
    return stable_;
  }

  // 在目标切换、输入失效和重新接近时撤销全部到达证据。
  void reset()
  {
    first_stamp_ns_ = 0;
    last_stamp_ns_ = 0;
    stable_ = false;
  }

private:
  ArrivalConfig config_;
  std::int64_t first_stamp_ns_{0};
  std::int64_t last_stamp_ns_{0};
  bool stable_{false};
};

}  // namespace go2_uwb_behavior

#endif  // GO2_UWB_BEHAVIOR__APPROACH_CORE_HPP_
