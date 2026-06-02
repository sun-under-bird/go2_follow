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

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <utility>
#include <vector>

#include "behavior_ext_plugins/back_up_twz_free_action.hpp"
#include "tf2/utils.h"

namespace nav2_behaviors
{
  void BackUpTwzFree::onConfigure()
  {
    auto node = this->node_.lock();
    if (!node)
    {
      throw std::runtime_error{"Failed to lock node"};
    }

    nav2_util::declare_parameter_if_not_declared(
      node,
      "robot_radius", rclcpp::ParameterValue(0.1));
    node->get_parameter("robot_radius", robot_radius_);

    nav2_util::declare_parameter_if_not_declared(
      node,
      "max_radius", rclcpp::ParameterValue(1.0));
    node->get_parameter("max_radius", max_radius_);

    if (max_radius_ < robot_radius_)
    {
      RCLCPP_WARN(
        node->get_logger(),
        "max_radius is smaller than robot_radius. Setting max_radius to robot_radius");
      max_radius_ = robot_radius_;
    }

    nav2_util::declare_parameter_if_not_declared(
      node,
      "service_name", rclcpp::ParameterValue(std::string("local_costmap/get_costmap")));
    node->get_parameter("service_name", service_name_);

    nav2_util::declare_parameter_if_not_declared(
      node,
      "free_threshold", rclcpp::ParameterValue(5));
    node->get_parameter("free_threshold", free_threshold_);

    nav2_util::declare_parameter_if_not_declared(
      node,
      "cost_threshold", rclcpp::ParameterValue(0.1));
    node->get_parameter("cost_threshold", cost_threshold_);

    nav2_util::declare_parameter_if_not_declared(
      node,
      "visualization", rclcpp::ParameterValue(false));
    node->get_parameter("visualization", visualization_);

    nav2_util::declare_parameter_if_not_declared(
      node,
      "enable_strafe", rclcpp::ParameterValue(true));
    node->get_parameter("enable_strafe", enable_strafe_);

    costmap_client_ = node->create_client<nav2_msgs::srv::GetCostmap>(service_name_);
    marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "back_up_twz_free_markers", 1);

    RCLCPP_DEBUG(node->get_logger(), "back_up_twz_free_action plugin initialized.");
  }

  Status BackUpTwzFree::onRun(const std::shared_ptr<const BackUpAction::Goal> command)
  {
    // 每次进入脱困行为时重新计算速度，避免复用上一次行为的方向。
    twist_x_ = 0.0;
    twist_y_ = 0.0;

    auto node = this->node_.lock();
    if (!node)
    {
      throw std::runtime_error{"Failed to lock node"};
    }

    // 从 costmap 服务获取当前代价地图；推荐对双目相机使用 local_costmap/get_costmap。
    if (!costmap_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(node->get_logger(), "Costmap service is not available.");
      return Status::FAILED;
    }

    auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
    auto result = costmap_client_->async_send_request(request);
    if (result.wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
    {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return Status::FAILED;
    }

    RCLCPP_DEBUG(node->get_logger(), "Got costmap");

    auto costmap = result.get()->map;
    if (costmap.data.empty() || costmap.metadata.size_x == 0 || costmap.metadata.size_y == 0)
    {
      RCLCPP_ERROR(node->get_logger(), "Costmap is empty.");
      return Status::FAILED;
    }

    if (!nav2_util::getCurrentPose(
            initial_pose_, *tf_, global_frame_, robot_base_frame_,
            transform_tolerance_))
    {
      RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
      return Status::FAILED;
    }

    // 读取当前位姿，把 costmap 中的自由栅格转换为 map 坐标后再计算脱困方向。
    auto pose_x = initial_pose_.pose.position.x;
    auto pose_y = initial_pose_.pose.position.y;
    auto yaw = tf2::getYaw(initial_pose_.pose.orientation);

    auto free_space_found = false;
    std::vector<geometry_msgs::msg::Point> free_points;

    // 从机器人外接圆之外开始扩半径搜索，避免把机器人自身 footprint 下的清空区域误认为脱困空间。
    auto radius = robot_radius_;
    const auto radius_step = std::max(static_cast<double>(costmap.metadata.resolution), 0.05);
    while (!free_space_found && radius <= max_radius_)
    {
      std::vector<geometry_msgs::msg::Point> candidate_points;

      for (unsigned int i = 0; i < costmap.metadata.size_x; i++)
      {
        for (unsigned int j = 0; j < costmap.metadata.size_y; j++)
        {
          auto costmap_index = i + j * costmap.metadata.size_x;
          auto x = (static_cast<double>(i) + 0.5) * costmap.metadata.resolution +
            costmap.metadata.origin.position.x;
          auto y = (static_cast<double>(j) + 0.5) * costmap.metadata.resolution +
            costmap.metadata.origin.position.y;
          auto distance_to_center = std::hypot(x - pose_x, y - pose_y);

          // 只统计机器人外侧到当前搜索半径之间的自由栅格；未知区和障碍区不会通过阈值。
          if (distance_to_center > robot_radius_ && distance_to_center <= radius)
          {
            if (static_cast<double>(costmap.data[costmap_index]) <= cost_threshold_)
            {
              candidate_points.push_back(geometry_msgs::msg::Point());
              candidate_points.back().x = x;
              candidate_points.back().y = y;
            }
          }
        }
      }

      if (candidate_points.size() >= static_cast<std::size_t>(free_threshold_))
      {
        free_space_found = true;
        free_points = std::move(candidate_points);
        RCLCPP_WARN(node->get_logger(), "free space found at radius: %f", radius);
        break;
      }
      else
      {
        RCLCPP_WARN(node->get_logger(), "free space not found at radius: %f", radius);
        radius += radius_step;
      }
    }

    if (!free_space_found || free_points.empty())
    {
      RCLCPP_WARN(
        node->get_logger(),
        "No free space found within %.3f meters. BackUpTwzFree failed.", max_radius_);
      return Status::FAILED;
    }

    // 用自由栅格质心作为脱困方向。 local costmap 把未知区保持为 unknown，
    // 否则未观测的侧后方可能会被误判为空闲区域。
    auto avg_x = 0.0;
    auto avg_y = 0.0;
    for (const auto & free_point : free_points)
    {
      avg_x += free_point.x;
      avg_y += free_point.y;
    }
    avg_x /= free_points.size();
    avg_y /= free_points.size();
    RCLCPP_WARN(node->get_logger(), "avg_x: %f, avg_y: %f", avg_x, avg_y);

    // RViz 可视化：红色点为本轮选中的自由栅格，绿色点为自由栅格质心。
    if (visualization_)
    {
      visualization_msgs::msg::MarkerArray markers;
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = global_frame_;
      marker.header.stamp = node->now();
      marker.ns = "free_space";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::POINTS;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = costmap.metadata.resolution;
      marker.scale.y = costmap.metadata.resolution;
      marker.color.r = 1.0;
      marker.color.a = 1.0;
      for (const auto & free_point : free_points)
      {
        marker.points.push_back(free_point);
      }
      markers.markers.push_back(marker);
      visualization_msgs::msg::Marker destination_marker;
      destination_marker.header.frame_id = global_frame_;
      destination_marker.header.stamp = node->now();
      destination_marker.ns = "destination";
      destination_marker.id = 0;
      destination_marker.type = visualization_msgs::msg::Marker::POINTS;
      destination_marker.action = visualization_msgs::msg::Marker::ADD;
      destination_marker.pose.orientation.w = 1.0;
      destination_marker.scale.x = costmap.metadata.resolution;
      destination_marker.scale.y = costmap.metadata.resolution;
      destination_marker.color.g = 1.0;
      destination_marker.color.a = 1.0;
      destination_marker.points.push_back(geometry_msgs::msg::Point());
      destination_marker.points.back().x = avg_x;
      destination_marker.points.back().y = avg_y;
      markers.markers.push_back(destination_marker);
      marker_pub_->publish(markers);
    }

    // 计算机器人坐标系下的自由空间方向，并据此生成线速度。
    auto angle_to_free_space = std::atan2(avg_y - pose_y, avg_x - pose_x);
    auto angle_diff = angle_to_free_space - yaw;
    if (angle_diff > M_PI)
    {
      angle_diff -= 2 * M_PI;
    }
    else if (angle_diff < -M_PI)
    {
      angle_diff += 2 * M_PI;
    }
    RCLCPP_WARN(node->get_logger(), "angle_diff: %f deg", angle_diff*180/M_PI);

    const auto free_direction_length = std::hypot(avg_x - pose_x, avg_y - pose_y);
    if (free_direction_length < costmap.metadata.resolution)
    {
      // 自由空间近似对称时，质心会落在机器人附近；此时退回到普通 BackUp 的前/后方向。
      const auto backup_direction = command->target.x >= 0.0 ? 1.0 : -1.0;
      twist_x_ = backup_direction * std::fabs(command->speed);
      twist_y_ = 0.0;
    }
    else if (enable_strafe_)
    {
      // 全向底盘：直接朝自由空间方向移动，可包含 y 方向速度。
      twist_x_ = std::cos(angle_diff) * command->speed;
      twist_y_ = std::sin(angle_diff) * command->speed;
    }
    else
    {
      // 非全向底盘：只保留 x 方向，选择更接近自由空间方向的前进或后退。
      twist_x_ = std::cos(angle_diff) >= 0.0 ?
        std::fabs(command->speed) : -std::fabs(command->speed);
      twist_y_ = 0.0;
    }

    command_x_ = command->target.x;
    command_time_allowance_ = command->time_allowance;

    end_time_ = this->clock_->now() + command_time_allowance_;

    RCLCPP_WARN(
      this->logger_, "backing up %f meters towards free space at angle %f",
      command_x_, angle_diff);

    return Status::SUCCEEDED;
  }

  Status BackUpTwzFree::onCycleUpdate()
  {
    // 检查 action 的最大允许执行时间，避免恢复行为长时间占用 behavior_server。
    rclcpp::Duration time_remaining = end_time_ - this->clock_->now();
    if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0)
    {
      this->stopRobot();
      RCLCPP_WARN(
        this->logger_,
        "Exceeded time allowance before reaching DriveOnHeading goal. Exiting.");
      return Status::FAILED;
    }

    geometry_msgs::msg::PoseStamped current_pose;
    if (!nav2_util::getCurrentPose(
            current_pose, *this->tf_, this->global_frame_, this->robot_base_frame_,
            this->transform_tolerance_))
    {
      RCLCPP_ERROR(this->logger_, "Current robot pose is not available.");
      return Status::FAILED;
    }

    double diff_x = initial_pose_.pose.position.x - current_pose.pose.position.x;
    double diff_y = initial_pose_.pose.position.y - current_pose.pose.position.y;
    double distance = hypot(diff_x, diff_y);

    feedback_->distance_traveled = distance;
    this->action_server_->publish_feedback(feedback_);

    // 只看实际移动距离是否达到 BackUp action 给定的目标距离。
    if (distance >= std::fabs(command_x_))
    {
      this->stopRobot();
      return Status::SUCCEEDED;
    }

    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_vel->linear.x = twist_x_;
    cmd_vel->linear.y = twist_y_;
    cmd_vel->angular.z = 0.0;

    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x = current_pose.pose.position.x;
    pose2d.y = current_pose.pose.position.y;
    pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

    // 复用 DriveOnHeading 的碰撞前瞻检查，防止恢复行为继续撞向障碍物。
    if (!isCollisionFree(distance, cmd_vel.get(), pose2d))
    {
      this->stopRobot();
      RCLCPP_WARN(this->logger_, "Collision Ahead - Exiting DriveOnHeading");
      return Status::FAILED;
    }

    this->vel_pub_->publish(std::move(cmd_vel));

    return Status::RUNNING;
  }

} // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::BackUpTwzFree, nav2_core::Behavior)
