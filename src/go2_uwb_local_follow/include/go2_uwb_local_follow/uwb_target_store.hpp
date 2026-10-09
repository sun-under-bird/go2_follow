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

#ifndef GO2_UWB_LOCAL_FOLLOW__UWB_TARGET_STORE_HPP_
#define GO2_UWB_LOCAL_FOLLOW__UWB_TARGET_STORE_HPP_

#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>

#include "go2_uwb_local_follow/input_timing.hpp"

namespace go2_uwb_local_follow
{
// 单个标签的原样坐标及时间状态，不包含位置滤波。
struct UwbTargetRecord
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  std::uint8_t confidence{0U};
  std::int8_t state{0};
  bool valid{false};
  std::int64_t stamp_ns{0};
  std::chrono::steady_clock::time_point receipt{};
  std::uint64_t version{0U};
  double frequency_hz{0.0};
  SourceStampTracker stamp_tracker;
};

// 适配器和行为控制器共用的标签隔离规则。
class UwbTargetStore
{
public:
  // 限制未知 ID 占用的缓存数量，已有 ID 始终可以更新。
  explicit UwbTargetStore(std::size_t capacity = 64U)
  : capacity_(capacity) {}

  // 时间戳只与同 ID 比较；无效新样本立即撤销有效性，不延用旧位置许可。
  bool update(
    std::uint32_t id, std::int64_t stamp_ns, std::int64_t now_ns, double timeout_sec,
    std::chrono::steady_clock::time_point receipt,
    double x, double y, double z, std::uint8_t confidence, std::int8_t state, bool valid)
  {
    auto iterator = records_.find(id);
    if (iterator == records_.end()) {
      if (records_.size() >= capacity_ ||
        sourceAgeSeconds(stamp_ns, now_ns) > timeout_sec)
      {
        return false;
      }
      iterator = records_.emplace(id, UwbTargetRecord{}).first;
    }
    auto & record = iterator->second;
    if (!record.stamp_tracker.accept(stamp_ns, now_ns, timeout_sec)) {
      return false;
    }
    if (record.version > 0U) {
      const double interval = std::chrono::duration<double>(receipt - record.receipt).count();
      if (interval > 0.0) {
        record.frequency_hz = 1.0 / interval;
      }
    }
    record.x = x;
    record.y = y;
    record.z = z;
    record.confidence = confidence;
    record.state = state;
    record.valid = valid && std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    record.stamp_ns = stamp_ns;
    record.receipt = receipt;
    ++record.version;
    return true;
  }

  // 查询指定 ID，其他标签的数据不会替代它。
  const UwbTargetRecord * find(std::uint32_t id) const
  {
    const auto iterator = records_.find(id);
    return iterator == records_.end() ? nullptr : &iterator->second;
  }

  // 同时校验源时间和单调接收时间，另一标签更新不能掩盖本标签断流。
  bool fresh(
    std::uint32_t id, std::int64_t now_ns, std::chrono::steady_clock::time_point current,
    double timeout_sec) const
  {
    const auto * record = find(id);
    return record && record->valid &&
           sourceAgeSeconds(record->stamp_ns, now_ns) <= timeout_sec &&
           std::chrono::duration<double>(current - record->receipt).count() <= timeout_sec;
  }

  // 为定时诊断提供全部标签的独立状态。
  const std::map<std::uint32_t, UwbTargetRecord> & records() const
  {
    return records_;
  }

  // 坐标系跳变后丢弃所有旧目标。
  void clear()
  {
    records_.clear();
  }

private:
  std::size_t capacity_;
  std::map<std::uint32_t, UwbTargetRecord> records_;
};
}  // namespace go2_uwb_local_follow
#endif  // GO2_UWB_LOCAL_FOLLOW__UWB_TARGET_STORE_HPP_
