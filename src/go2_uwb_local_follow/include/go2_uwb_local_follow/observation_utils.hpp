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

#ifndef GO2_UWB_LOCAL_FOLLOW__OBSERVATION_UTILS_HPP_
#define GO2_UWB_LOCAL_FOLLOW__OBSERVATION_UTILS_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "go2_uwb_local_follow/input_timing.hpp"
#include "go2_uwb_local_follow/rolling_obstacle_map_core.hpp"

namespace go2_uwb_local_follow
{

// 检查坐标系、位置和四元数，再提取带源时间戳的二维里程计位姿。
inline bool extractOdomPose(
  const nav_msgs::msg::Odometry & message, const std::string & odom_frame,
  const std::string & base_frame, TimedPose2D * pose)
{
  const auto & q = message.pose.pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (pose == nullptr || message.header.frame_id != odom_frame ||
    message.child_frame_id != base_frame || !std::isfinite(norm) ||
    norm <= std::numeric_limits<double>::epsilon())
  {
    return false;
  }
  pose->stamp_ns = sourceStampNanoseconds(message.header.stamp);
  pose->x = message.pose.pose.position.x;
  pose->y = message.pose.pose.position.y;
  pose->yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y) / norm,
    1.0 - 2.0 * (q.y * q.y + q.z * q.z) / norm);
  return pose->stamp_ns > 0 && std::isfinite(pose->x) && std::isfinite(pose->y) &&
         std::isfinite(pose->yaw);
}

// 预检迭代器要求的连续 FLOAT32 布局，拒绝截断、类型错误和异端序消息。
inline bool validFloatCloud(
  const sensor_msgs::msg::PointCloud2 & cloud,
  std::initializer_list<const char *> fields)
{
  const std::uint16_t endian_probe = 1;
  const bool host_bigendian = *reinterpret_cast<const std::uint8_t *>(&endian_probe) == 0;
  const std::size_t row_size = static_cast<std::size_t>(cloud.width) * cloud.point_step;
  if ((cloud.is_bigendian != 0U) != host_bigendian || cloud.height == 0U ||
    cloud.point_step == 0U || cloud.row_step != row_size ||
    cloud.data.size() != row_size * cloud.height)
  {
    return false;
  }
  for (const char * name : fields) {
    const auto found = std::find_if(
      cloud.fields.begin(), cloud.fields.end(),
      [name](const sensor_msgs::msg::PointField & field) {return field.name == name;});
    if (found == cloud.fields.end() || found->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      found->count != 1U || cloud.point_step < sizeof(float) ||
      found->offset > cloud.point_step - sizeof(float) || found->offset % alignof(float) != 0U)
    {
      return false;
    }
  }
  return cloud.point_step % alignof(float) == 0U;
}

}  // namespace go2_uwb_local_follow

#endif  // GO2_UWB_LOCAL_FOLLOW__OBSERVATION_UTILS_HPP_
