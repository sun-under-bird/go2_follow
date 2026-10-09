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

#include <chrono>
#include <limits>

#include "gtest/gtest.h"
#include "go2_uwb_local_follow/uwb_target_store.hpp"

namespace
{
using Store = go2_uwb_local_follow::UwbTargetStore;
using Time = std::chrono::steady_clock::time_point;

// 固定两种时钟以确定性重现各 ID 的时间戳和过期行为。
bool put(Store & store, std::uint32_t id, double stamp, double now, double receipt, double x)
{
  return store.update(
    id, static_cast<std::int64_t>(stamp * 1e9), static_cast<std::int64_t>(now * 1e9), 0.5,
    Time{} + std::chrono::duration_cast<Time::duration>(std::chrono::duration<double>(receipt)),
    x, 0.0, 0.0, 100, 0, true);
}
}  // namespace

// 交替 ID、相同采集时间以及跨 ID 的较早时间戳均应独立处理。
TEST(UwbTargetStore, SeparatesIdsAndTheirSourceClocks)
{
  Store store;
  ASSERT_TRUE(put(store, 1, 10.0, 10.1, 0.0, 1.0));
  ASSERT_TRUE(put(store, 2, 9.9, 10.1, 0.0, 2.0));
  ASSERT_TRUE(put(store, 3, 10.0, 10.1, 0.0, 3.0));
  EXPECT_DOUBLE_EQ(store.find(1)->x, 1.0);
  EXPECT_DOUBLE_EQ(store.find(2)->x, 2.0);
  EXPECT_DOUBLE_EQ(store.find(3)->x, 3.0);
  EXPECT_FALSE(put(store, 1, 10.0, 10.1, 0.1, 9.0));
}

// ID2 更新不得刷新 ID1 的位置、版本或有效期。
TEST(UwbTargetStore, OtherIdCannotHideDropout)
{
  Store store;
  ASSERT_TRUE(put(store, 1, 10.0, 10.0, 0.0, 1.0));
  ASSERT_TRUE(put(store, 2, 10.6, 10.6, 0.6, 2.0));
  const auto current = Time{} + std::chrono::milliseconds(600);
  EXPECT_FALSE(store.fresh(1, 10600000000LL, current, 0.5));
  EXPECT_TRUE(store.fresh(2, 10600000000LL, current, 0.5));
  EXPECT_EQ(store.find(1)->version, 1U);
}

// 新无效坐标必须立即撤销运动许可，旧帧不能覆盖它。
TEST(UwbTargetStore, InvalidSampleRevokesValidity)
{
  Store store;
  ASSERT_TRUE(put(store, 1, 10.0, 10.0, 0.0, 1.0));
  ASSERT_TRUE(put(store, 1, 10.1, 10.1, 0.1, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(store.find(1)->valid);
  EXPECT_FALSE(put(store, 1, 9.0, 10.1, 0.1, 1.0));
}

// 缓存有界且允许已知 ID 更新，重置后没有旧快照。
TEST(UwbTargetStore, BoundsStorageAndResets)
{
  Store store(1);
  EXPECT_TRUE(put(store, 1, 10.0, 10.0, 0.0, 1.0));
  EXPECT_FALSE(put(store, 2, 10.0, 10.0, 0.0, 2.0));
  EXPECT_TRUE(put(store, 1, 10.1, 10.1, 0.1, 1.1));
  store.clear();
  EXPECT_EQ(store.find(1), nullptr);
}
