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

#include "gtest/gtest.h"
#include "go2_uwb_local_follow/input_timing.hpp"
namespace follow = go2_uwb_local_follow;

// 旧反馈不能续期，源时钟暂停时单调时钟仍会撤销角度放宽许可。
TEST(AvoidanceFeedback, ExpiresRepeatedSamplesAndHonorsExplicitStop)
{
  follow::AvoidanceFeedbackTracker feedback;
  const auto start = follow::AvoidanceFeedbackTracker::SteadyTime{};
  feedback.update(1000000000LL, 1000000000LL, true, start);
  EXPECT_TRUE(feedback.active(1100000000LL, start + std::chrono::milliseconds(100)));
  feedback.update(1000000000LL, 1100000000LL, true, start + std::chrono::milliseconds(100));
  EXPECT_FALSE(feedback.active(1100000000LL, start + std::chrono::milliseconds(210)));
  feedback.update(1300000000LL, 1300000000LL, true, start + std::chrono::milliseconds(300));
  EXPECT_TRUE(feedback.active(1300000000LL, start + std::chrono::milliseconds(300)));
  feedback.update(1400000000LL, 1400000000LL, false, start + std::chrono::milliseconds(400));
  EXPECT_FALSE(feedback.active(1400000000LL, start + std::chrono::milliseconds(400)));
}
// 模式切换清除许可，同一旧反馈不能重新激活，更新反馈可以恢复。
TEST(AvoidanceFeedback, RequiresNewSampleAfterReset)
{
  follow::AvoidanceFeedbackTracker feedback;
  const auto start = follow::AvoidanceFeedbackTracker::SteadyTime{};
  feedback.update(1000000000LL, 1000000000LL, true, start);
  feedback.reset();
  feedback.update(1000000000LL, 1100000000LL, true, start);
  EXPECT_FALSE(feedback.active(1100000000LL, start));
  feedback.update(1100000000LL, 1100000000LL, true, start);
  EXPECT_TRUE(feedback.active(1100000000LL, start));
}

// 重复或积压消息不能延长有效期，后续新鲜帧不需要重启即可恢复。
TEST(InputTiming, RejectsStaleDuplicatesAndRecovers)
{
  follow::SourceStampTracker tracker;
  EXPECT_TRUE(tracker.accept(10000000000LL, 10000000000LL, 0.5));
  EXPECT_FALSE(tracker.accept(10000000000LL, 10100000000LL, 0.5));
  EXPECT_FALSE(tracker.accept(11000000000LL, 16000000000LL, 0.5));
  EXPECT_TRUE(tracker.accept(16000000000LL, 16010000000LL, 0.5));
}

// 明显未来消息不污染递增状态，ROS 时间回退后重新接受新时间轴。
TEST(InputTiming, HandlesFutureSamplesAndClockReset)
{
  follow::SourceStampTracker tracker;
  EXPECT_FALSE(tracker.accept(20000000000LL, 10000000000LL, 0.5));
  EXPECT_TRUE(tracker.accept(10000000000LL, 10000000000LL, 0.5));
  EXPECT_TRUE(tracker.accept(1000000000LL, 1000000000LL, 0.5));
  EXPECT_GT(follow::sourceAgeSeconds(10000000000LL, 1000000000LL), 0.5);
  EXPECT_GT(follow::sourceAgeSeconds(0, 1000000000LL), 0.5);
}
