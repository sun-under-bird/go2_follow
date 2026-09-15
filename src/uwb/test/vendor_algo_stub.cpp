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

extern "C"
{
#include "uwb_robot_algo.h"

// 测试替身没有历史滤波状态，重连清理为空操作。
void algo_uwb_aoa_clean(void)
{
}

// 测试替身只透传距离，用于在非 ARM64 主机上验证真实串口线程的断线重连。
unsigned char algo_uwb_aoa_merge(struct input_data * input, struct output * output)
{
  output->r = input->Distance;
  output->rad = 0.0F;
  output->x = input->Distance;
  output->y = 0.0F;
  output->state = 1;
  return 0;
}
}
