#ifndef COMMON_TYPES_HPP
#define COMMON_TYPES_HPP

#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>

struct FollowConfig {
    bool active = true;

    double follow_dist = 1.0;
    double target_timeout_sec = 0.5;
    double scan_timeout_sec = 0.5;
    double target_exclusion_radius = 0.35;

    double apf_influence_dist = 0.6;
    double apf_slowdown_dist = 0.6;
    double apf_emergency_dist = 0.3;
    double apf_repulse_gain = 0.01;

    double max_linear_speed = 0.5;
    double max_lateral_speed = 0.12;
    double max_angular_speed = 1.0;
    double linear_scale_factor = 0.5;
    double linear_y_scale_factor = 1.0;
    double angular_scale_factor = 1.0;

    double distance_deadband = 0.05;
    double lateral_deadband = 0.03;
    double angle_deadband = 0.08;
    double rotate_only_angle = 0.45;
    double min_forward_speed = 0.06;

    double rectangle_width = 0.4;

    double robot_frame_front = 0.25;
    double robot_frame_back = 0.25;
    double robot_frame_left = 0.16;
    double robot_frame_right = 0.16;

    bool local_map_enabled = true;
    double local_map_size_x = 1.0;
    double local_map_size_y = 1.0;
    double local_map_resolution = 0.05;
    double local_map_lifetime_sec = 2.0;
    int local_map_max_points = 1600;
    bool local_map_ray_clear_enabled = true;
    double local_map_ray_clear_radius = 0.06;
    double local_map_ray_clear_hit_margin = 0.08;
};

struct ObstaclePoint2D {
    double x = 0.0;
    double y = 0.0;
};

struct VelocityCmd {
    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;
};

struct SharedState {
    std::mutex target_mutex;
    double target_x = 1.0;
    double target_y = 0.0;
    bool has_target = false;
    std::chrono::steady_clock::time_point target_stamp{};

    std::mutex velocity_mutex;
    VelocityCmd last_velocity;

    // 更新 UWB 目标位置和目标时间戳。
    void setTarget(double x, double y) {
        std::lock_guard<std::mutex> lock(target_mutex);
        target_x = x;
        target_y = y;
        target_stamp = std::chrono::steady_clock::now();
        has_target = true;
    }

    // 读取 UWB 目标位置，并返回目标数据年龄。
    bool getTarget(double& x, double& y, double& age_sec) {
        std::lock_guard<std::mutex> lock(target_mutex);
        if (!has_target) {
            x = target_x;
            y = target_y;
            age_sec = std::numeric_limits<double>::infinity();
            return false;
        }

        x = target_x;
        y = target_y;
        age_sec = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - target_stamp).count();
        return true;
    }

    // 缓存最近一次发布的速度，方便其他模块读取。
    void setVelocity(double vx, double vy, double wz) {
        std::lock_guard<std::mutex> lock(velocity_mutex);
        last_velocity.vx = vx;
        last_velocity.vy = vy;
        last_velocity.wz = wz;
    }
};

#endif
