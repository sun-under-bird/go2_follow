#ifndef FOLLOW_AVOID_CONTROLLER_HPP
#define FOLLOW_AVOID_CONTROLLER_HPP

#include "common_types.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class FollowAvoidController {
public:
    using VelocityCallback = std::function<void(const geometry_msgs::msg::Twist&)>;

    FollowAvoidController(SharedState& state, FollowConfig config)
        : state_(state), config_(config) {}

    // Set the publisher callback used by this pure controller.
    void setVelocityCallback(VelocityCallback cb) {
        velocity_callback_ = std::move(cb);
    }

    // Convert one LaserScan frame plus the latest UWB target into a velocity command.
    void processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        if (!config_.active || !scan_msg || scan_msg->ranges.empty()) {
            publishZero();
            return;
        }

        double target_x = 0.0;
        double target_y = 0.0;
        double target_age = 0.0;
        const bool has_target = state_.getTarget(target_x, target_y, target_age);
        if (!has_target || target_age > config_.target_timeout_sec) {
            publishZero();
            return;
        }

        const auto obstacle = extractObstacle(scan_msg, target_x, target_y);
        geometry_msgs::msg::Twist cmd;
        calculateVelocity(cmd, target_x, target_y, obstacle);
        publishVelocity(cmd);
    }

private:
    struct ObstacleInfo {
        double min_dist = std::numeric_limits<double>::infinity();
        double repulse_x = 0.0;
        double repulse_y = 0.0;
        double left_y_min = 0.0;
        double right_y_min = 0.0;
    };

    SharedState& state_;
    FollowConfig config_;
    VelocityCallback velocity_callback_;

    ObstacleInfo extractObstacle(
        const sensor_msgs::msg::LaserScan::SharedPtr& scan_msg,
        double target_x,
        double target_y) const {
        ObstacleInfo obstacle;
        obstacle.left_y_min = -config_.rectangle_width / 2.0;
        obstacle.right_y_min = config_.rectangle_width / 2.0;

        const double target_vec_len = std::hypot(target_x, target_y);

        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            const float range = scan_msg->ranges[i];
            if (!std::isfinite(range)) {
                continue;
            }
            if (range < scan_msg->range_min || range > scan_msg->range_max) {
                continue;
            }

            const double angle = scan_msg->angle_min + static_cast<double>(i) * scan_msg->angle_increment;
            const double point_x = range * std::cos(angle);
            const double point_y = range * std::sin(angle);
            if (point_x < 0.0) {
                continue;
            }

            if (isInRobotFrame(point_x, point_y) || isTargetPoint(point_x, point_y, target_x, target_y)) {
                continue;
            }

            const double dist = std::hypot(point_x, point_y);
            obstacle.min_dist = std::min(obstacle.min_dist, dist);

            if (dist < config_.apf_influence_dist && dist > 1e-6) {
                const double force = config_.apf_repulse_gain *
                    (1.0 / dist - 1.0 / config_.apf_influence_dist) / (dist * dist);
                obstacle.repulse_x -= force * point_x / dist;
                obstacle.repulse_y -= force * point_y / dist;
            }

            // Same corridor idea as the original lidar tracker: project points into
            // the rectangle between robot and target, then find the occupied side.
            if (target_vec_len > 1e-6) {
                const double proj_x = (point_x * target_x + point_y * target_y) / target_vec_len;
                // base_link uses y-left, so this lateral axis is right-positive.
                // That keeps the original lateral_error formula pushing away from the occupied side.
                const double proj_y = (point_x * target_y - point_y * target_x) / target_vec_len;

                if (proj_x >= 0.0 &&
                    proj_x <= target_vec_len &&
                    std::abs(proj_y) <= config_.rectangle_width / 2.0) {
                    if (proj_y > 0.0 && proj_y > obstacle.left_y_min) {
                        obstacle.left_y_min = proj_y;
                    } else if (proj_y <= 0.0 && proj_y < obstacle.right_y_min) {
                        obstacle.right_y_min = proj_y;
                    }
                }
            }
        }

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
        return obstacle;
    }

    bool isInRobotFrame(double x, double y) const {
        return x > -config_.robot_frame_back &&
               x < config_.robot_frame_front &&
               y > -config_.robot_frame_right &&
               y < config_.robot_frame_left;
    }

    // Exclude points near the tracked UWB target before obstacle force is calculated.
    bool isTargetPoint(double x, double y, double target_x, double target_y) const {
        return std::hypot(x - target_x, y - target_y) < config_.target_exclusion_radius;
    }

    // Calculate the final follow command using target error, corridor offset, and APF.
    void calculateVelocity(
        geometry_msgs::msg::Twist& cmd,
        double target_x,
        double target_y,
        const ObstacleInfo& obstacle) const {
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
            config_.apf_slowdown_dist > config_.apf_emergency_dist) {
            const double slowdown = (obstacle.min_dist - config_.apf_emergency_dist) /
                (config_.apf_slowdown_dist - config_.apf_emergency_dist);
            const double slowdown_factor = std::clamp(slowdown, 0.1, 1.0);
            cmd.linear.x *= slowdown_factor;
        }

        cmd.linear.x = std::clamp(cmd.linear.x, 0.0, config_.max_linear_speed);
        cmd.linear.y = std::clamp(cmd.linear.y, -config_.max_lateral_speed, config_.max_lateral_speed);
        cmd.angular.z = std::clamp(cmd.angular.z, -config_.max_angular_speed, config_.max_angular_speed);
    }

    void publishZero() {
        publishVelocity(geometry_msgs::msg::Twist{});
    }

    void publishVelocity(const geometry_msgs::msg::Twist& cmd) {
        state_.setVelocity(cmd.linear.x, cmd.linear.y, cmd.angular.z);
        if (velocity_callback_) {
            velocity_callback_(cmd);
        }
    }
};

#endif
