#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "go2_stereo_apf_follow/apf_core.hpp"

namespace go2_stereo_apf_follow
{

enum class VfhMode
{
  FOLLOW,
  BYPASS_LEFT,
  BYPASS_RIGHT
};

enum class BypassSide
{
  NONE,
  LEFT,
  RIGHT
};

struct VfhConfig
{
  double sector_angle_deg{5.0};
  double range_min{0.10};
  double range_max{4.0};
  double robot_radius{0.35};
  double safety_margin{0.20};
  double hard_stop_distance{0.35};
  double slowdown_distance{0.80};
  double corridor_width{0.75};
  double target_mask_radius{0.35};
  double side_switch_hold_sec{1.2};
  double corridor_clear_hold_sec{0.8};
  double heading_limit_deg{90.0};
  double target_heading_weight{1.0};
  double last_heading_weight{0.25};
  double wrong_side_penalty{3.0};
  double valley_width_weight{0.04};
  double max_vx{0.45};
  double max_vy{0.35};
  double max_wz{0.8};
  double angular_scale{1.0};
  double bypass_heading_blend{0.55};
  double command_filter_alpha{0.45};
  double max_delta_vx_per_sec{0.55};
  double max_delta_vy_per_sec{0.55};
  double max_delta_wz_per_sec{1.0};
  double min_move_speed{0.08};
  double linear_scale{0.45};
};

struct VfhHistogram
{
  std::vector<double> weights;
  std::vector<bool> blocked;
  std::vector<int> free_run;
  double sector_angle_rad{0.0};
};

struct VfhState
{
  VfhMode mode{VfhMode::FOLLOW};
  BypassSide locked_side{BypassSide::NONE};
  double last_heading{0.0};
  TwistCommand last_command;
  double corridor_clear_since{-1.0};
  double side_lock_time{-1.0};
  double last_update_time{-1.0};
};

struct VfhChoice
{
  double heading{0.0};
  double score{0.0};
  int sector{0};
  int free_run{0};
};

struct VfhResult
{
  TwistCommand command;
  Point2D follow_goal;
  VfhHistogram histogram;
  double target_heading{0.0};
  double selected_heading{0.0};
  double nearest_dist{0.0};
  bool has_nearest{false};
  bool hard_stop{false};
  bool corridor_blocked{false};
  bool target_sector_blocked{false};
  VfhMode mode{VfhMode::FOLLOW};
  BypassSide locked_side{BypassSide::NONE};
};

// 将角度从度转换为弧度。
inline double deg_to_rad(double degrees)
{
  return degrees * kPi / 180.0;
}

// 把角度归一化到 [-pi, pi)。
inline double normalize_angle(double angle)
{
  while (angle >= kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

// 计算两个角度之间的最小绝对差。
inline double abs_angle_diff(double a, double b)
{
  return std::abs(normalize_angle(a - b));
}

// 根据配置得到单个 VFH 扇区的弧度宽度。
inline double vfh_sector_angle_rad(const VfhConfig & config)
{
  return deg_to_rad(clamp(config.sector_angle_deg, 1.0, 45.0));
}

// 根据配置计算整圈直方图的扇区数量。
inline int vfh_sector_count(const VfhConfig & config)
{
  return std::max(8, static_cast<int>(std::ceil(2.0 * kPi / vfh_sector_angle_rad(config))));
}

// 判断某个 heading 是否在前向可行搜索范围内。
inline bool heading_allowed(double heading, const VfhConfig & config)
{
  return std::abs(normalize_angle(heading)) <= deg_to_rad(clamp(config.heading_limit_deg, 30.0, 180.0));
}

// 创建指定尺寸的空 VFH 直方图。
inline VfhHistogram make_empty_histogram(const VfhConfig & config)
{
  const int count = vfh_sector_count(config);
  VfhHistogram histogram;
  histogram.weights.assign(count, 0.0);
  histogram.blocked.assign(count, false);
  histogram.free_run.assign(count, 0);
  histogram.sector_angle_rad = 2.0 * kPi / static_cast<double>(count);
  return histogram;
}

// 获取扇区中心对应的 heading。
inline double sector_center_heading(const VfhHistogram & histogram, int sector)
{
  const int count = static_cast<int>(histogram.blocked.size());
  if (count <= 0) {
    return 0.0;
  }
  const int safe_sector = ((sector % count) + count) % count;
  return normalize_angle(-kPi + (static_cast<double>(safe_sector) + 0.5) * histogram.sector_angle_rad);
}

// 将 heading 映射到直方图扇区下标。
inline int sector_index_for_heading(const VfhHistogram & histogram, double heading)
{
  const int count = static_cast<int>(histogram.blocked.size());
  if (count <= 0) {
    return 0;
  }
  const double shifted = normalize_angle(heading) + kPi;
  int index = static_cast<int>(std::floor(shifted / histogram.sector_angle_rad));
  return std::clamp(index, 0, count - 1);
}

// 判断障碍点是否属于 UWB 标签附近的人体点。
inline bool point_near_target_body(
  const Point3D & point,
  const Point2D & target,
  const VfhConfig & config)
{
  if (config.target_mask_radius <= 0.0) {
    return false;
  }
  return std::hypot(point.x - target.x, point.y - target.y) <= config.target_mask_radius;
}

// 计算扣除跟随距离后的局部跟随目标点。
inline Point2D compute_follow_goal(const Point2D & target, double follow_distance)
{
  const double distance = distance2d(target);
  if (distance <= std::max(0.0, follow_distance) || distance <= 1e-6) {
    return Point2D{};
  }
  const double scale = (distance - std::max(0.0, follow_distance)) / distance;
  return Point2D{target.x * scale, target.y * scale};
}

// 把一个障碍的角度膨胀区间写入直方图。
inline void mark_blocked_interval(
  VfhHistogram & histogram,
  double center_heading,
  double half_width,
  double weight)
{
  // 扇区数量很少，直接全扇区扫描可以避免跨 -pi/pi 时的边界错误。
  const int count = static_cast<int>(histogram.blocked.size());
  for (int i = 0; i < count; ++i) {
    const double heading = sector_center_heading(histogram, i);
    if (abs_angle_diff(heading, center_heading) <= half_width) {
      histogram.blocked[i] = true;
      histogram.weights[i] = std::max(histogram.weights[i], weight);
    }
  }
}

// 统计每个空闲扇区左右连续可通行宽度。
inline void populate_free_runs(VfhHistogram & histogram)
{
  const int count = static_cast<int>(histogram.blocked.size());
  for (int i = 0; i < count; ++i) {
    if (histogram.blocked[i]) {
      histogram.free_run[i] = 0;
      continue;
    }
    int run = 1;
    for (int step = 1; step < count && run < count; ++step) {
      const int left = (i - step + count) % count;
      if (histogram.blocked[left]) {
        break;
      }
      ++run;
    }
    for (int step = 1; step < count && run < count; ++step) {
      const int right = (i + step) % count;
      if (histogram.blocked[right]) {
        break;
      }
      ++run;
    }
    histogram.free_run[i] = run;
  }
}

// 构建 VFH 极坐标直方图，并同步统计最近障碍距离。
inline VfhHistogram build_vfh_histogram(
  const std::vector<Point3D> & points,
  const Point2D & target,
  const VfhConfig & config,
  bool & has_nearest,
  double & nearest_dist)
{
  auto histogram = make_empty_histogram(config);
  has_nearest = false;
  nearest_dist = 0.0;

  const double inflated_radius = std::max(0.0, config.robot_radius + config.safety_margin);
  const double range_max = std::max(config.range_min, config.range_max);
  for (const auto & point : points) {
    if (!is_finite(point) || point_near_target_body(point, target, config)) {
      continue;
    }

    const double dist = distance3d_xy(point);
    if (dist <= 1e-6) {
      continue;
    }
    if (!has_nearest || dist < nearest_dist) {
      has_nearest = true;
      nearest_dist = dist;
    }
    if (dist < config.range_min || dist > range_max) {
      continue;
    }

    const double heading = std::atan2(point.y, point.x);
    const double ratio = clamp(inflated_radius / std::max(dist, 1e-6), 0.0, 1.0);
    const double half_width = std::asin(ratio) + 0.5 * histogram.sector_angle_rad;
    const double weight = 1.0 + (range_max - dist) / std::max(0.01, range_max);
    mark_blocked_interval(histogram, heading, half_width, weight);
  }

  populate_free_runs(histogram);
  return histogram;
}

// 判断障碍点是否落在机器人到 UWB 标签之间的走廊内。
inline bool corridor_has_obstacle(
  const std::vector<Point3D> & points,
  const Point2D & target,
  const VfhConfig & config)
{
  const double target_len = distance2d(target);
  if (target_len <= 1e-6) {
    return false;
  }

  const double corridor_half_width = std::max(0.0, config.corridor_width * 0.5);
  const double end_margin = std::max(0.0, config.target_mask_radius);
  for (const auto & point : points) {
    if (!is_finite(point) || point_near_target_body(point, target, config)) {
      continue;
    }
    const double proj_x = (point.x * target.x + point.y * target.y) / target_len;
    const double proj_y = (point.x * -target.y + point.y * target.x) / target_len;
    if (proj_x > 0.0 && proj_x < target_len - end_margin && std::abs(proj_y) <= corridor_half_width) {
      return true;
    }
  }
  return false;
}

// 判断目标方向对应扇区是否不可通行。
inline bool target_sector_blocked(
  const VfhHistogram & histogram,
  double target_heading,
  const VfhConfig & config)
{
  if (!heading_allowed(target_heading, config)) {
    return true;
  }
  const int index = sector_index_for_heading(histogram, target_heading);
  return histogram.blocked.empty() || histogram.blocked[index];
}

// 计算某一侧或任意侧的最佳可通行 heading。
inline std::optional<VfhChoice> choose_heading(
  const VfhHistogram & histogram,
  double target_heading,
  double last_heading,
  BypassSide side,
  const VfhConfig & config)
{
  std::optional<VfhChoice> best;
  const int count = static_cast<int>(histogram.blocked.size());
  for (int i = 0; i < count; ++i) {
    if (histogram.blocked[i]) {
      continue;
    }
    const double heading = sector_center_heading(histogram, i);
    if (!heading_allowed(heading, config)) {
      continue;
    }

    double score =
      config.target_heading_weight * abs_angle_diff(heading, target_heading) +
      config.last_heading_weight * abs_angle_diff(heading, last_heading) -
      config.valley_width_weight * static_cast<double>(histogram.free_run[i]);

    // 绕障状态下给锁定侧强偏置，防止正前方障碍边缘抖动导致左右来回切换。
    if (side == BypassSide::LEFT && heading < 0.0) {
      score += config.wrong_side_penalty;
    } else if (side == BypassSide::RIGHT && heading > 0.0) {
      score += config.wrong_side_penalty;
    }

    if (!best.has_value() || score < best->score) {
      best = VfhChoice{heading, score, i, histogram.free_run[i]};
    }
  }
  return best;
}

// 根据左右可行扇区评分选择首次绕障方向。
inline BypassSide choose_bypass_side(
  const VfhHistogram & histogram,
  double target_heading,
  double last_heading,
  const Point2D & target,
  const VfhConfig & config)
{
  const auto left = choose_heading(histogram, target_heading, last_heading, BypassSide::LEFT, config);
  const auto right = choose_heading(histogram, target_heading, last_heading, BypassSide::RIGHT, config);
  if (left.has_value() && !right.has_value()) {
    return BypassSide::LEFT;
  }
  if (right.has_value() && !left.has_value()) {
    return BypassSide::RIGHT;
  }
  if (left.has_value() && right.has_value()) {
    if (std::abs(left->score - right->score) < 1e-6) {
      return target.y >= 0.0 ? BypassSide::LEFT : BypassSide::RIGHT;
    }
    return left->score <= right->score ? BypassSide::LEFT : BypassSide::RIGHT;
  }
  return BypassSide::NONE;
}

// 将绕障侧转换为控制模式。
inline VfhMode mode_from_side(BypassSide side)
{
  if (side == BypassSide::LEFT) {
    return VfhMode::BYPASS_LEFT;
  }
  if (side == BypassSide::RIGHT) {
    return VfhMode::BYPASS_RIGHT;
  }
  return VfhMode::FOLLOW;
}

// 将控制模式转换为锁定绕障侧。
inline BypassSide side_from_mode(VfhMode mode)
{
  if (mode == VfhMode::BYPASS_LEFT) {
    return BypassSide::LEFT;
  }
  if (mode == VfhMode::BYPASS_RIGHT) {
    return BypassSide::RIGHT;
  }
  return BypassSide::NONE;
}

// 更新 FOLLOW/BYPASS 状态机。
inline void update_vfh_state(
  const VfhHistogram & histogram,
  bool corridor_blocked,
  bool target_blocked,
  double target_heading,
  const Point2D & target,
  double now_sec,
  const VfhConfig & config,
  VfhState & state)
{
  if (state.mode == VfhMode::FOLLOW) {
    state.corridor_clear_since = -1.0;
    state.locked_side = BypassSide::NONE;
    if (corridor_blocked || target_blocked) {
      const auto side = choose_bypass_side(histogram, target_heading, state.last_heading, target, config);
      state.locked_side = side;
      state.mode = mode_from_side(side);
      state.side_lock_time = now_sec;
    }
    return;
  }

  if (!corridor_blocked && !target_blocked) {
    if (state.corridor_clear_since < 0.0) {
      state.corridor_clear_since = now_sec;
    }
    if (now_sec - state.corridor_clear_since >= config.corridor_clear_hold_sec) {
      state.mode = VfhMode::FOLLOW;
      state.locked_side = BypassSide::NONE;
      state.corridor_clear_since = -1.0;
      return;
    }
  } else {
    state.corridor_clear_since = -1.0;
  }

  const auto locked_side = side_from_mode(state.mode);
  const auto locked_choice =
    choose_heading(histogram, target_heading, state.last_heading, locked_side, config);
  if (locked_choice.has_value() || now_sec - state.side_lock_time < config.side_switch_hold_sec) {
    state.locked_side = locked_side;
    return;
  }

  // 锁定侧完全无路且保持时间已过时，才允许切到另一侧，避免卡死。
  const auto new_side = locked_side == BypassSide::LEFT ? BypassSide::RIGHT : BypassSide::LEFT;
  const auto new_choice = choose_heading(histogram, target_heading, state.last_heading, new_side, config);
  if (new_choice.has_value()) {
    state.locked_side = new_side;
    state.mode = mode_from_side(new_side);
    state.side_lock_time = now_sec;
  }
}

// 对两个角度做圆周插值。
inline double blend_heading(double from, double to, double to_weight)
{
  const double weight = clamp(to_weight, 0.0, 1.0);
  return normalize_angle(from + normalize_angle(to - from) * weight);
}

// 根据 heading 和距离生成未滤波速度。
inline TwistCommand make_raw_vfh_command(
  double selected_heading,
  double target_heading,
  double goal_distance,
  bool has_nearest,
  double nearest_dist,
  VfhMode mode,
  const VfhConfig & config)
{
  TwistCommand command;
  if (goal_distance <= 0.03) {
    return command;
  }

  const double cos_h = std::cos(selected_heading);
  const double sin_h = std::sin(selected_heading);
  double speed_limit = std::max(0.01, config.max_vx);
  if (std::abs(sin_h) > 1e-4) {
    speed_limit = std::min(speed_limit, std::max(0.01, config.max_vy) / std::abs(sin_h));
  }
  if (std::abs(cos_h) > 1e-4) {
    speed_limit = std::min(speed_limit, std::max(0.01, config.max_vx) / std::abs(cos_h));
  }

  double speed = std::min(speed_limit, goal_distance * std::max(0.01, config.linear_scale));
  speed = std::max(std::min(speed, speed_limit), std::min(config.min_move_speed, speed_limit));
  if (has_nearest && nearest_dist < config.slowdown_distance) {
    const double denom = std::max(0.01, config.slowdown_distance - config.hard_stop_distance);
    const double scale = clamp((nearest_dist - config.hard_stop_distance) / denom, 0.15, 1.0);
    speed *= scale;
  }

  command.vx = speed * cos_h;
  command.vy = speed * sin_h;
  const double yaw_heading = mode == VfhMode::FOLLOW ?
    target_heading :
    blend_heading(target_heading, selected_heading, config.bypass_heading_blend);
  command.wz = clamp(yaw_heading * config.angular_scale, -config.max_wz, config.max_wz);
  command.vx = clamp(command.vx, 0.0, config.max_vx);
  command.vy = clamp(command.vy, -config.max_vy, config.max_vy);
  return command;
}

// 限制单个速度轴的变化量。
inline double limit_delta(double previous, double current, double max_delta)
{
  return previous + clamp(current - previous, -max_delta, max_delta);
}

// 对速度做低通和加速度限制。
inline TwistCommand smooth_vfh_command(
  const TwistCommand & raw,
  double now_sec,
  const VfhConfig & config,
  VfhState & state)
{
  if (state.last_update_time < 0.0) {
    state.last_update_time = now_sec;
    state.last_command = raw;
    return raw;
  }

  const double dt = clamp(now_sec - state.last_update_time, 0.02, 0.20);
  const double alpha = clamp(config.command_filter_alpha, 0.0, 1.0);
  TwistCommand filtered;
  filtered.vx = state.last_command.vx + alpha * (raw.vx - state.last_command.vx);
  filtered.vy = state.last_command.vy + alpha * (raw.vy - state.last_command.vy);
  filtered.wz = state.last_command.wz + alpha * (raw.wz - state.last_command.wz);

  filtered.vx = limit_delta(state.last_command.vx, filtered.vx, config.max_delta_vx_per_sec * dt);
  filtered.vy = limit_delta(state.last_command.vy, filtered.vy, config.max_delta_vy_per_sec * dt);
  filtered.wz = limit_delta(state.last_command.wz, filtered.wz, config.max_delta_wz_per_sec * dt);

  state.last_update_time = now_sec;
  state.last_command = filtered;
  return filtered;
}

// 计算完整 VFH 控制结果，并更新内部绕障状态。
inline VfhResult compute_vfh_command(
  const std::vector<Point3D> & points,
  const Point2D & target,
  double follow_distance,
  double now_sec,
  const VfhConfig & config,
  VfhState & state)
{
  VfhResult result;
  result.follow_goal = compute_follow_goal(target, follow_distance);
  result.target_heading = distance2d(result.follow_goal) > 1e-6 ?
    std::atan2(result.follow_goal.y, result.follow_goal.x) :
    std::atan2(target.y, target.x);
  result.histogram = build_vfh_histogram(
    points,
    target,
    config,
    result.has_nearest,
    result.nearest_dist);
  result.hard_stop = result.has_nearest && result.nearest_dist < config.hard_stop_distance;
  result.target_sector_blocked = target_sector_blocked(result.histogram, result.target_heading, config);
  result.corridor_blocked = corridor_has_obstacle(points, target, config) || result.target_sector_blocked;

  if (result.hard_stop) {
    // 急停必须绕过低通滤波，保证近距离障碍出现时立即输出零速度。
    state.last_command = TwistCommand{};
    state.last_update_time = now_sec;
    result.mode = state.mode;
    result.locked_side = state.locked_side;
    result.selected_heading = state.last_heading;
    return result;
  }

  update_vfh_state(
    result.histogram,
    result.corridor_blocked,
    result.target_sector_blocked,
    result.target_heading,
    target,
    now_sec,
    config,
    state);

  std::optional<VfhChoice> choice;
  if (state.mode == VfhMode::FOLLOW && !result.target_sector_blocked) {
    choice = VfhChoice{
      result.target_heading,
      0.0,
      sector_index_for_heading(result.histogram, result.target_heading),
      0};
  } else {
    choice = choose_heading(
      result.histogram,
      result.target_heading,
      state.last_heading,
      side_from_mode(state.mode),
      config);
  }

  if (!choice.has_value()) {
    state.last_command = TwistCommand{};
    state.last_update_time = now_sec;
    result.mode = state.mode;
    result.locked_side = state.locked_side;
    result.selected_heading = state.last_heading;
    return result;
  }

  result.selected_heading = choice->heading;
  state.last_heading = result.selected_heading;
  const double goal_distance = distance2d(result.follow_goal);
  const auto raw = make_raw_vfh_command(
    result.selected_heading,
    result.target_heading,
    goal_distance,
    result.has_nearest,
    result.nearest_dist,
    state.mode,
    config);
  result.command = smooth_vfh_command(raw, now_sec, config, state);
  result.mode = state.mode;
  result.locked_side = state.locked_side;
  return result;
}

}  // namespace go2_stereo_apf_follow
