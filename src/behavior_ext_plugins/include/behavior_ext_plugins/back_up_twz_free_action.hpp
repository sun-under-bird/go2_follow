// Copyright (c) 2022 Joshua Wallace
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

#ifndef BEHAVIOR_EXT_PLUGINS__BACK_UP_TWZ_FREE_ACTION_HPP_
#define BEHAVIOR_EXT_PLUGINS__BACK_UP_TWZ_FREE_ACTION_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_behaviors/plugins/drive_on_heading.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using BackUpAction = nav2_msgs::action::BackUp;

namespace nav2_behaviors
{
class BackUpTwzFree : public DriveOnHeading<nav2_msgs::action::BackUp>
{
public:
  // 功能：收到 BackUp action 后，从配置的 costmap 服务中寻找机器人周围的自由空间，
  // 并把本次脱困的线速度方向设置为朝向自由空间的方向。
  Status onRun(const std::shared_ptr<const BackUpAction::Goal> command) override;

  // 功能：周期性发布本次脱困速度，直到达到目标距离、超时或碰撞检测失败。
  Status onCycleUpdate() override;

protected:
  // 功能：读取插件参数，并创建 costmap 服务客户端与 RViz 可视化发布器。
  void onConfigure() override;

private:
  // 用于读取 local_costmap/get_costmap 或 global_costmap/get_costmap。
  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::string service_name_;

  double twist_x_{0.0};
  double twist_y_{0.0};
  double max_radius_{1.0};
  double robot_radius_{0.1};
  int free_threshold_{5};
  double cost_threshold_{0.0};
  bool visualization_{false};
  bool enable_strafe_{true};
};
}  // namespace nav2_behaviors

#endif  // BEHAVIOR_EXT_PLUGINS__BACK_UP_TWZ_FREE_ACTION_HPP_
