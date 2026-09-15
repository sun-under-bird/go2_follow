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

#include <cstdint>
#include <vector>
#include "gtest/gtest.h"
extern "C"
{
#include "uart_stack.h"
}

namespace
{
// 根据公开协议独立打包测试帧，避免测试直接调用被测 CRC 实现。
std::vector<std::uint8_t> makeFrame(std::size_t payload_size = sizeof(uwb_aoa_fob_pkg_t))
{
  std::vector<std::uint8_t> frame{0x55, 0xAA, 1,
    static_cast<std::uint8_t>(payload_size + 2U), 0, 0xC5,
    static_cast<std::uint8_t>(payload_size)};
  frame.resize(7U + payload_size, 0);
  std::uint16_t crc = 0;
  for (std::size_t i = 5; i < frame.size(); ++i) {
    crc ^= static_cast<std::uint16_t>(frame[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = crc & 0x8000U ? (crc << 1) ^ 0x1021U : crc << 1;
    }
  }
  frame.push_back(crc >> 8);
  frame.push_back(crc & 0xFFU);
  return frame;
}

// 按固定时刻送入一串字节，统计上层能够拿到的合法 C5 数据包。
int feed(const std::vector<std::uint8_t> & bytes, std::uint64_t now_ms)
{
  int count = 0;
  for (const auto byte : bytes) {
    if (uart_receive_byte_at(byte, now_ms) == 1) {
      void * packet = nullptr;
      if (static_cast<std::uint8_t>(uart_protocol_packet_process(&packet)) == 0xC5U) {
        EXPECT_NE(packet, nullptr);
        ++count;
      }
    }
  }
  return count;
}
}  // namespace

// 零长度及过长帧头之后无需重启即可重新接收完整帧。
TEST(UartRecovery, RecoversAfterInvalidLengths)
{
  for (const auto length : {0, 1, 99, 255}) {
    uart_reset_receiver();
    EXPECT_EQ(feed({0x55, 0xAA, 0, static_cast<std::uint8_t>(length), 0}, 100), 0);
    EXPECT_EQ(feed(makeFrame(), 101), 1);
  }
}

// 半帧中断超过 200 ms 后自动丢弃，重连复位也不能拼接旧帧。
TEST(UartRecovery, RecoversAfterTruncatedFrameAndReconnect)
{
  uart_reset_receiver();
  auto frame = makeFrame();
  EXPECT_EQ(feed({frame.begin(), frame.begin() + 12}, 100), 0);
  EXPECT_EQ(feed(frame, 301), 1);
  EXPECT_EQ(feed({frame.begin(), frame.begin() + 9}, 302), 0);
  uart_reset_receiver();
  EXPECT_EQ(feed(frame, 303), 1);
}

// CRC 错误和 CRC 正确的短载荷均不得产生伪造坐标，下一帧必须能够恢复。
TEST(UartRecovery, RejectsBadCrcAndShortPayload)
{
  uart_reset_receiver();
  auto bad_crc = makeFrame();
  bad_crc.back() ^= 0xFFU;
  EXPECT_EQ(feed(bad_crc, 100), 0);
  EXPECT_EQ(feed(makeFrame(1), 101), 0);
  EXPECT_EQ(feed(makeFrame(), 102), 1);
}

// 连续头字节及背靠背完整帧均能保留正确协议边界。
TEST(UartRecovery, HandlesOverlappingHeadersAndBackToBackFrames)
{
  uart_reset_receiver();
  EXPECT_EQ(feed({0x55}, 100), 0);
  EXPECT_EQ(feed(makeFrame(), 100), 1);
  EXPECT_EQ(feed(makeFrame(), 100), 1);
}
