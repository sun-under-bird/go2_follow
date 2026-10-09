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

#include <gtest/gtest.h>

#include "go2_uwb_behavior/approach_core.hpp"

using go2_uwb_behavior::ArrivalConfig;
using go2_uwb_behavior::ArrivalMonitor;

TEST(ArrivalMonitor, RequiresNewContinuousSamples)
{
  ArrivalMonitor monitor;
  EXPECT_FALSE(monitor.update(1.0, 0.0, 1000000000));
  EXPECT_FALSE(monitor.update(0.55, 0.0, 1100000000));
  // 多次控制周期读取同一帧不会计时，第一帧达标也不代表成功。
  for (int i = 0; i < 100; ++i) {
    EXPECT_FALSE(monitor.update(0.55, 0.0, 1100000000));
  }
  EXPECT_FALSE(monitor.update(0.55, 0.0, 1400000000));
  EXPECT_TRUE(monitor.update(0.55, 0.0, 1600000000));
}

TEST(ArrivalMonitor, UnsafeOrOutOfRangeRevokesArrival)
{
  ArrivalMonitor monitor;
  EXPECT_FALSE(monitor.update(0.55, 0.0, 1000000000));
  EXPECT_TRUE(monitor.update(0.55, 0.0, 1500000000));
  EXPECT_FALSE(monitor.update(0.49, 0.0, 1600000000));
  EXPECT_FALSE(monitor.update(0.55, 0.0, 1700000000));
  EXPECT_FALSE(monitor.update(0.61, 0.0, 1800000000));
  EXPECT_FALSE(monitor.update(0.55, 0.0, 1900000000));
}

TEST(ArrivalMonitor, DropoutAndGoalSwitchResetHistory)
{
  ArrivalMonitor monitor;
  EXPECT_FALSE(monitor.update(0.55, 0.0, 1000000000));
  EXPECT_FALSE(monitor.update(0.55, 0.0, 1600000000));
  EXPECT_TRUE(monitor.update(0.55, 0.0, 2100000000));
  monitor.reset();
  EXPECT_FALSE(monitor.update(0.55, 0.0, 2200000000));
  ArrivalConfig next;
  next.stop_distance = 1.0;
  monitor = ArrivalMonitor(next);
  EXPECT_FALSE(monitor.update(1.05, 0.0, 2300000000));
  EXPECT_TRUE(monitor.update(1.05, 0.0, 2800000000));
}

TEST(ArrivalMonitor, HeadingIsOptional)
{
  ArrivalMonitor monitor;
  EXPECT_FALSE(monitor.update(0.55, 1.0, 1000000000));
  EXPECT_TRUE(monitor.update(0.55, 1.0, 1500000000));
  ArrivalConfig config;
  config.require_heading = true;
  monitor = ArrivalMonitor(config);
  EXPECT_FALSE(monitor.update(0.55, 1.0, 2000000000));
  EXPECT_FALSE(monitor.update(0.55, 0.1, 2100000000));
  EXPECT_TRUE(monitor.update(0.55, 0.1, 2600000000));
}
