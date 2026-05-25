#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace go2_stereo_apf_follow
{

constexpr double kPi = 3.14159265358979323846;

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct Point3D
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct TwistCommand
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct UwbTargetConfig
{
  bool prefer_xy{true};
  bool angle_in_degrees{false};
  bool invert_y{false};
  double angle_offset_rad{0.0};
  double anchor_x_offset{0.0};
  double anchor_y_offset{0.0};
  double min_distance_m{0.15};
  double max_distance_m{8.0};
};

struct PointFilterConfig
{
  double x_min{-4.0};
  double x_max{4.0};
  double y_abs{4.0};
  double z_min{0.05};
  double z_max{1.2};
  double robot_frame_front{0.15};
  double robot_frame_back{0.35};
  double robot_frame_left{0.15};
  double robot_frame_right{0.15};
};

struct TargetTrackingConfig
{
  double target_radius{0.30};
  int min_points_in_target{1};
  double smoothing_alpha{0.35};
};

struct ApfConfig
{
  double influence_dist{0.25};
  double repulse_gain{0.01};
  double max_repulse{1.0};
  double emergency_dist{0.2};
  double slowdown_dist{0.25};
  double corridor_width{0.35};
};

struct FollowControlConfig
{
  double follow_distance{0.4};
  double distance_deadband{0.05};
  double lateral_deadband{0.03};
  double yaw_deadband{0.10};
  double linear_scale{0.50};
  double lateral_scale{1.0};
  double angular_scale{1.0};
  double max_vx{1.0};
  double max_vy{1.0};
  double max_wz{1.0};
  double max_reverse_vx{1.0};
  double reverse_scale{0.8};
  double min_vx_abs{0.06};
  bool allow_reverse{true};
};

struct ObstacleSummary
{
  double repulse_x{0.0};
  double repulse_y{0.0};
  double nearest_dist{0.0};
  bool has_nearest{false};
  double left_y_min{0.0};
  double right_y_min{0.0};
};

inline double clamp(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

inline double distance2d(const Point2D & point)
{
  return std::hypot(point.x, point.y);
}

inline double distance3d_xy(const Point3D & point)
{
  return std::hypot(point.x, point.y);
}

inline bool is_finite(double value)
{
  return std::isfinite(value);
}

inline bool is_finite(const Point2D & point)
{
  return is_finite(point.x) && is_finite(point.y);
}

inline bool is_finite(const Point3D & point)
{
  return is_finite(point.x) && is_finite(point.y) && is_finite(point.z);
}

inline bool in_robot_frame(const Point3D & point, const PointFilterConfig & config)
{
  return point.x > -config.robot_frame_back && point.x < config.robot_frame_front &&
         point.y > -config.robot_frame_right && point.y < config.robot_frame_left;
}

inline bool point_passes_filter(const Point3D & point, const PointFilterConfig & config)
{
  if (!is_finite(point)) {
    return false;
  }
  if (point.x < config.x_min || point.x > config.x_max) {
    return false;
  }
  if (std::abs(point.y) > config.y_abs) {
    return false;
  }
  if (point.z < config.z_min || point.z > config.z_max) {
    return false;
  }
  return !in_robot_frame(point, config);
}

inline std::vector<Point3D> filter_points(
  const std::vector<Point3D> & points,
  const PointFilterConfig & config)
{
  std::vector<Point3D> filtered;
  filtered.reserve(points.size());
  for (const auto & point : points) {
    if (point_passes_filter(point, config)) {
      filtered.push_back(point);
    }
  }
  return filtered;
}

inline std::optional<Point2D> parse_uwb_target(
  double x,
  double y,
  double range,
  double angle,
  const UwbTargetConfig & config)
{
  Point2D target;
  const bool xy_available = is_finite(x) && is_finite(y) && std::hypot(x, y) > 1e-6;
  if (config.prefer_xy && xy_available) {
    target.x = x + config.anchor_x_offset;
    target.y = y + config.anchor_y_offset;
  } else {
    if (!is_finite(range) || !is_finite(angle)) {
      return std::nullopt;
    }
    double angle_rad = config.angle_in_degrees ? angle * kPi / 180.0 : angle;
    angle_rad += config.angle_offset_rad;
    target.x = range * std::cos(angle_rad);
    target.y = range * std::sin(angle_rad);
  }

  if (config.invert_y) {
    target.y = -target.y;
  }

  const double distance = distance2d(target);
  if (!is_finite(target) || distance < config.min_distance_m || distance > config.max_distance_m) {
    return std::nullopt;
  }
  return target;
}

inline std::optional<Point2D> compute_target_centroid(
  const std::vector<Point3D> & points,
  const Point2D & current_target,
  const TargetTrackingConfig & config)
{
  double sum_x = 0.0;
  double sum_y = 0.0;
  int count = 0;
  for (const auto & point : points) {
    if (std::hypot(point.x - current_target.x, point.y - current_target.y) <= config.target_radius) {
      sum_x += point.x;
      sum_y += point.y;
      ++count;
    }
  }

  if (count < std::max(1, config.min_points_in_target)) {
    return std::nullopt;
  }
  return Point2D{sum_x / static_cast<double>(count), sum_y / static_cast<double>(count)};
}

inline Point2D smooth_target(
  const Point2D & current,
  const Point2D & measurement,
  double alpha)
{
  const double clipped_alpha = clamp(alpha, 0.0, 1.0);
  return Point2D{
    current.x + clipped_alpha * (measurement.x - current.x),
    current.y + clipped_alpha * (measurement.y - current.y)};
}

inline ObstacleSummary summarize_obstacles(
  const std::vector<Point3D> & points,
  const Point2D & target,
  const ApfConfig & config)
{
  ObstacleSummary summary;
  summary.left_y_min = -config.corridor_width * 0.5;
  summary.right_y_min = config.corridor_width * 0.5;

  const double target_len = std::hypot(target.x, target.y);
  for (const auto & point : points) {
    const double dist = distance3d_xy(point);
    if (dist <= 1e-6) {
      continue;
    }

    if (!summary.has_nearest || dist < summary.nearest_dist) {
      summary.nearest_dist = dist;
      summary.has_nearest = true;
    }

    if (dist < config.influence_dist && point.x > -0.1) {
      const double force =
        config.repulse_gain * (1.0 / dist - 1.0 / config.influence_dist) / (dist * dist);
      summary.repulse_x -= force * point.x / dist;
      summary.repulse_y -= force * point.y / dist;
    }

    if (target_len > 1e-6) {
      const double proj_x = (point.x * target.x + point.y * target.y) / target_len;
      const double proj_y = (point.x * -target.y + point.y * target.x) / target_len;
      if (proj_x >= 0.0 && proj_x <= target_len && std::abs(proj_y) <= config.corridor_width * 0.5) {
        if (proj_y > 0.0 && proj_y > summary.left_y_min) {
          summary.left_y_min = proj_y;
        } else if (proj_y <= 0.0 && proj_y < summary.right_y_min) {
          summary.right_y_min = proj_y;
        }
      }
    }
  }

  const double repulse_mag = std::hypot(summary.repulse_x, summary.repulse_y);
  if (repulse_mag > config.max_repulse && repulse_mag > 1e-6) {
    const double scale = config.max_repulse / repulse_mag;
    summary.repulse_x *= scale;
    summary.repulse_y *= scale;
  }

  return summary;
}

inline TwistCommand compute_follow_command(
  const Point2D & target,
  const ObstacleSummary & obstacles,
  const ApfConfig & apf_config,
  const FollowControlConfig & control_config)
{
  TwistCommand cmd;
  if (obstacles.has_nearest && obstacles.nearest_dist < apf_config.emergency_dist) {
    return cmd;
  }

  const double distance_error = target.x - control_config.follow_distance;
  if (std::abs(distance_error) >= control_config.distance_deadband) {
    cmd.vx = distance_error * control_config.linear_scale;
    if (cmd.vx < 0.0) {
      cmd.vx *= control_config.reverse_scale;
    }
    if (!control_config.allow_reverse) {
      cmd.vx = std::max(0.0, cmd.vx);
    } else if (control_config.min_vx_abs > 0.0 && std::abs(cmd.vx) < control_config.min_vx_abs) {
      cmd.vx = cmd.vx > 0.0 ? control_config.min_vx_abs : -control_config.min_vx_abs;
    }
  }

  const double bearing = std::atan2(target.y, target.x);
  if (std::abs(bearing) >= control_config.yaw_deadband) {
    cmd.wz = bearing * control_config.angular_scale;
  }

  double lateral_error = -(obstacles.left_y_min + obstacles.right_y_min);
  if (std::abs(lateral_error) > 1.0) {
    lateral_error = 0.0;
  }
  if (std::abs(lateral_error) >= control_config.lateral_deadband) {
    cmd.vy = lateral_error * control_config.lateral_scale;
  }

  cmd.vx += obstacles.repulse_x;
  cmd.vy += obstacles.repulse_y;

  if (
    obstacles.has_nearest && obstacles.nearest_dist < apf_config.slowdown_dist &&
    std::abs(cmd.vx) > 0.0)
  {
    const double scale =
      (obstacles.nearest_dist - apf_config.emergency_dist) /
      std::max(0.01, apf_config.slowdown_dist - apf_config.emergency_dist);
    cmd.vx *= clamp(scale, 0.1, 1.0);
  }

  const double min_vx = -std::max(0.0, control_config.max_reverse_vx);
  cmd.vx = clamp(cmd.vx, min_vx, control_config.max_vx);
  cmd.vy = clamp(cmd.vy, -control_config.max_vy, control_config.max_vy);
  cmd.wz = clamp(cmd.wz, -control_config.max_wz, control_config.max_wz);
  return cmd;
}

}  // namespace go2_stereo_apf_follow
