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

#ifndef GO2_UWB_LOCAL_FOLLOW__INPUT_TIMING_HPP_
#define GO2_UWB_LOCAL_FOLLOW__INPUT_TIMING_HPP_

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <limits>

namespace go2_uwb_local_follow
{

// 将 ROS 时间字段转换为整数纳秒，避免在无 ROS 的核心测试中引入消息依赖。
template<typename StampT>
std::int64_t sourceStampNanoseconds(const StampT & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

// 同时拒绝零时间戳和明显来自未来的样本；时钟回退期间旧快照自动失效。
inline double sourceAgeSeconds(std::int64_t stamp_ns, std::int64_t now_ns)
{
  if (stamp_ns <= 0 || now_ns <= 0 || stamp_ns - now_ns > 50000000LL) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max(0.0, static_cast<double>(now_ns - stamp_ns) * 1.0e-9);
}

class SourceStampTracker
{
public:
  // 只接受新鲜且递增的源时间戳；本机 ROS 时钟回退后允许新时间轴重新开始。
  bool accept(std::int64_t stamp_ns, std::int64_t now_ns, double timeout_sec)
  {
    if (now_ns < last_clock_ns_) {
      last_stamp_ns_ = 0;
    }
    last_clock_ns_ = now_ns;
    if (!std::isfinite(timeout_sec) || timeout_sec <= 0.0 ||
      sourceAgeSeconds(stamp_ns, now_ns) > timeout_sec || stamp_ns <= last_stamp_ns_)
    {
      return false;
    }
    last_stamp_ns_ = stamp_ns;
    return true;
  }

private:
  std::int64_t last_stamp_ns_{0};
  std::int64_t last_clock_ns_{0};
};

// 安全前进绕障反馈只在短窗口内有效；重放旧反馈不能持续放宽跟随角度。
class AvoidanceFeedbackTracker
{
public:
  using SteadyTime = std::chrono::steady_clock::time_point;

  // 保存规划器每周期的显式反馈，零速、受阻或无避障时立即取消许可。
  void update(
    std::int64_t stamp_ns, std::int64_t now_ns, bool safe_forward_avoidance,
    SteadyTime receipt = std::chrono::steady_clock::now())
  {
    if (stamp_tracker_.accept(stamp_ns, now_ns, kTimeoutSec)) {
      stamp_ns_ = stamp_ns;
      receipt_ = receipt;
      active_ = safe_forward_avoidance;
    }
  }

  // 同时检查采集与单调接收年龄，断开反馈后最多 0.20 秒回到普通角度门限。
  bool active(
    std::int64_t now_ns, SteadyTime current = std::chrono::steady_clock::now()) const
  {
    return active_ && sourceAgeSeconds(stamp_ns_, now_ns) <= kTimeoutSec &&
           std::chrono::duration<double>(current - receipt_).count() <= kTimeoutSec;
  }

  // 输入中断或模式切换后撤销旧许可，必须收到更新的规划反馈。
  void reset()
  {
    active_ = false;
  }

private:
  static constexpr double kTimeoutSec = 0.20;
  SourceStampTracker stamp_tracker_;
  std::int64_t stamp_ns_{0};
  SteadyTime receipt_{};
  bool active_{false};
};

}  // namespace go2_uwb_local_follow

#endif  // GO2_UWB_LOCAL_FOLLOW__INPUT_TIMING_HPP_
