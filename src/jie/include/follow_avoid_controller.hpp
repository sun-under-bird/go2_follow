// Copyright 2026 ZhangWanjie
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef FOLLOW_AVOID_CONTROLLER_HPP_
#define FOLLOW_AVOID_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "common_types.hpp"

class FollowAvoidController
{
public:
  using VelocityCallback = std::function<void (const geometry_msgs::msg::Twist &)>;

  // 构造跟随避障控制器，并绑定共享目标状态。
  FollowAvoidController(SharedState & state, FollowConfig config)
  : state_(state), config_(config) {}

  // 设置速度发布回调，控制器本身不直接依赖 ROS publisher。
  void setVelocityCallback(VelocityCallback cb)
  {
    velocity_callback_ = std::move(cb);
  }

  // 使用当前 LaserScan 直接计算跟随避障速度。
  void processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    if (!scan_msg || scan_msg->ranges.empty()) {
      publishZero();
      return;
    }

    processObstaclePoints(scanToObstaclePoints(scan_msg));
  }

  // 使用障碍点集合计算最终 /cmd_vel。
  void processObstaclePoints(const std::vector<ObstaclePoint2D> & obstacle_points)
  {
    if (!config_.active) {
      publishZero();
      return;
    }

    double target_x = 0.0;
    double target_y = 0.0;
    const bool has_target = state_.getTarget(target_x, target_y);
    if (!has_target) {
      publishZero();
      return;
    }

    const auto obstacle = extractObstacle(obstacle_points, target_x, target_y);
    geometry_msgs::msg::Twist cmd;
    calculateVelocity(cmd, target_x, target_y, obstacle);
    publishVelocity(cmd);
  }

private:
  struct ObstacleInfo
  {
    double min_dist = std::numeric_limits<double>::infinity();
    double repulse_x = 0.0;
    double repulse_y = 0.0;
    double left_y_min = 0.0;
    double right_y_min = 0.0;
  };

  SharedState & state_;
  FollowConfig config_;
  VelocityCallback velocity_callback_;

  // 将 LaserScan 转为机器人平面坐标点。
  std::vector<ObstaclePoint2D> scanToObstaclePoints(
    const sensor_msgs::msg::LaserScan::SharedPtr & scan_msg) const
  {
    std::vector<ObstaclePoint2D> points;
    points.reserve(scan_msg->ranges.size());

    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
      const float range = scan_msg->ranges[i];
      if (!std::isfinite(range) || range < scan_msg->range_min || range > scan_msg->range_max) {
        continue;
      }

      const double angle = scan_msg->angle_min + static_cast<double>(i) * scan_msg->angle_increment;
      points.push_back({range * std::cos(angle), range * std::sin(angle)});
    }

    return points;
  }

  // 从障碍点提取最近距离、APF 斥力和目标走廊左右占用。
  ObstacleInfo extractObstacle(
    const std::vector<ObstaclePoint2D> & obstacle_points,
    double target_x,
    double target_y) const
  {
    ObstacleInfo obstacle;
    obstacle.left_y_min = -config_.rectangle_width / 2.0;
    obstacle.right_y_min = config_.rectangle_width / 2.0;

    const double target_vec_len = std::hypot(target_x, target_y);

    for (const auto & point : obstacle_points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        continue;
      }
      if (isInRobotFrame(point.x, point.y) || isTargetPoint(point.x, point.y, target_x, target_y)) {
        continue;
      }

      const double dist = std::hypot(point.x, point.y);
      obstacle.min_dist = std::min(obstacle.min_dist, dist);

      if (dist < config_.apf_influence_dist && dist > 1e-6) {
        const double force = config_.apf_repulse_gain *
          (1.0 / dist - 1.0 / config_.apf_influence_dist) / (dist * dist);
        obstacle.repulse_x -= force * point.x / dist;
        obstacle.repulse_y -= force * point.y / dist;
      }

      // 模仿上游 lidar_tracker：把障碍投影到机器人到目标的矩形走廊，判断左右哪侧更拥挤。
      if (target_vec_len > 1e-6) {
        const double proj_x = (point.x * target_x + point.y * target_y) / target_vec_len;
        const double proj_y = (-point.x * target_y + point.y * target_x) / target_vec_len;

        if (proj_x >= 0.0 &&
          proj_x <= target_vec_len &&
          std::abs(proj_y) <= config_.rectangle_width / 2.0)
        {
          if (proj_y > 0.0 && proj_y > obstacle.left_y_min) {
            obstacle.left_y_min = proj_y;
          } else if (proj_y <= 0.0 && proj_y < obstacle.right_y_min) {
            obstacle.right_y_min = proj_y;
          }
        }
      }
    }

    normalizeRepulse(obstacle);
    return obstacle;
  }

  // 限制斥力大小，避免近距离噪声点让速度突变。
  void normalizeRepulse(ObstacleInfo & obstacle) const
  {
    const double repulse_mag = std::hypot(obstacle.repulse_x, obstacle.repulse_y);
    if (repulse_mag > 1.0) {
      obstacle.repulse_x /= repulse_mag;
      obstacle.repulse_y /= repulse_mag;
    }

    obstacle.repulse_x = std::clamp(obstacle.repulse_x, -config_.max_linear_speed, 0.0);
    obstacle.repulse_y = std::clamp(
      obstacle.repulse_y,
      -config_.max_lateral_speed,
      config_.max_lateral_speed);
  }

  // 判断障碍点是否落在机器人自身包围框内。
  bool isInRobotFrame(double x, double y) const
  {
    return x > -config_.robot_frame_back &&
           x < config_.robot_frame_front &&
           y > -config_.robot_frame_right &&
           y < config_.robot_frame_left;
  }

  // 排除 UWB 目标附近点，避免把跟随对象自己当作障碍。
  bool isTargetPoint(double x, double y, double target_x, double target_y) const
  {
    return std::hypot(x - target_x, y - target_y) < config_.target_exclusion_radius;
  }

  // 根据目标误差、走廊横向误差和 APF 斥力生成最终速度。
  void calculateVelocity(
    geometry_msgs::msg::Twist & cmd,
    double target_x,
    double target_y,
    const ObstacleInfo & obstacle) const
  {
    if (obstacle.min_dist < config_.apf_emergency_dist) {
      return;
    }

    const double target_dist = std::hypot(target_x, target_y);
    const double angle_error = std::atan2(target_y, target_x);
    if (std::abs(angle_error) >= config_.angle_deadband) {
      cmd.angular.z = std::clamp(
        angle_error * config_.angular_scale_factor,
        -config_.max_angular_speed,
        config_.max_angular_speed);
    }

    if (std::abs(angle_error) < config_.rotate_only_angle) {
      const double dist_error = target_dist - config_.follow_dist;
      if (dist_error > config_.distance_deadband) {
        cmd.linear.x = dist_error * config_.linear_scale_factor;
        if (cmd.linear.x < config_.min_forward_speed) {
          cmd.linear.x = config_.min_forward_speed;
        }
      }
    }

    cmd.linear.x += obstacle.repulse_x;

    double lateral_error = -(obstacle.left_y_min + obstacle.right_y_min);
    if (std::abs(lateral_error) > 1.0) {
      lateral_error = 0.0;
    }
    if (std::abs(lateral_error) >= config_.lateral_deadband) {
      cmd.linear.y = lateral_error * config_.linear_y_scale_factor;
    }

    cmd.linear.y += obstacle.repulse_y;

    if (obstacle.min_dist < config_.apf_slowdown_dist &&
      config_.apf_slowdown_dist > config_.apf_emergency_dist)
    {
      const double slowdown = (obstacle.min_dist - config_.apf_emergency_dist) /
        (config_.apf_slowdown_dist - config_.apf_emergency_dist);
      const double slowdown_factor = std::clamp(slowdown, 0.1, 1.0);
      cmd.linear.x *= slowdown_factor;
    }

    cmd.linear.x = std::clamp(cmd.linear.x, 0.0, config_.max_linear_speed);
    cmd.linear.y = std::clamp(cmd.linear.y, -config_.max_lateral_speed, config_.max_lateral_speed);
    cmd.angular.z =
      std::clamp(cmd.angular.z, -config_.max_angular_speed, config_.max_angular_speed);
  }

  // 发布零速度。
  void publishZero()
  {
    publishVelocity(geometry_msgs::msg::Twist{});
  }

  // 缓存并发布速度指令。
  void publishVelocity(const geometry_msgs::msg::Twist & cmd)
  {
    state_.setVelocity(cmd.linear.x, cmd.linear.y, cmd.angular.z);
    if (velocity_callback_) {
      velocity_callback_(cmd);
    }
  }
};

#endif  // FOLLOW_AVOID_CONTROLLER_HPP_
