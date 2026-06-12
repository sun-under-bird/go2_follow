#ifndef LOCAL_OBSTACLE_MAP_HPP
#define LOCAL_OBSTACLE_MAP_HPP

#include "common_types.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class LocalObstacleMap {
public:
    // 构造局部障碍地图，并保存地图尺寸、分辨率和生命周期配置。
    explicit LocalObstacleMap(FollowConfig config)
        : config_(config) {}

    // 使用一帧相机转出的 LaserScan 更新按参数配置的滚动局部障碍记忆。
    void updateFromScan(
        const sensor_msgs::msg::LaserScan::SharedPtr& scan_msg,
        const geometry_msgs::msg::TransformStamped& scan_to_map,
        const geometry_msgs::msg::TransformStamped& map_to_robot) {
        const auto now = std::chrono::steady_clock::now();
        pruneStale(now);

        if (!scan_msg || scan_msg->ranges.empty()) {
            pruneOutsideWindow(map_to_robot);
            return;
        }

        clearVisibleRays(*scan_msg, scan_to_map);

        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            const float range = scan_msg->ranges[i];
            if (!isValidRange(*scan_msg, range)) {
                continue;
            }

            const double angle = scan_msg->angle_min + static_cast<double>(i) * scan_msg->angle_increment;
            const double scan_x = range * std::cos(angle);
            const double scan_y = range * std::sin(angle);
            const auto map_point = transformPoint(scan_x, scan_y, scan_to_map);

            // 障碍点写入固定地图坐标系，机器人移动后仍能短时间记住侧向障碍。
            insertOrRefreshPoint(map_point.x, map_point.y, now);
        }

        pruneOutsideWindow(map_to_robot);
        limitPointCount();
    }

    // 将当前局部地图投影回机器人坐标系，供控制器计算侧向避障。
    std::vector<ObstaclePoint2D> getObstaclePoints(
        const geometry_msgs::msg::TransformStamped& map_to_robot) const {
        std::vector<ObstaclePoint2D> points;
        points.reserve(cells_.size());

        for (const auto& cell : cells_) {
            const auto robot_point = transformPoint(cell.x, cell.y, map_to_robot);
            if (isInsideLocalWindow(robot_point.x, robot_point.y)) {
                points.push_back(robot_point);
            }
        }

        return points;
    }

    // TF 长时间不可用时清空地图，避免过期障碍一直影响控制。
    void clear() {
        cells_.clear();
    }

private:
    struct MapCell {
        double x = 0.0;
        double y = 0.0;
        std::chrono::steady_clock::time_point stamp{};
    };

    FollowConfig config_;
    std::vector<MapCell> cells_;

    // 检查 scan 距离值是否在传感器有效范围内。
    bool isValidRange(const sensor_msgs::msg::LaserScan& scan_msg, float range) const {
        return std::isfinite(range) &&
               range >= scan_msg.range_min &&
               range <= scan_msg.range_max;
    }

    // 用当前相机视野中的每条光束清除自由空间内的旧障碍点。
    void clearVisibleRays(
        const sensor_msgs::msg::LaserScan& scan_msg,
        const geometry_msgs::msg::TransformStamped& scan_to_map) {
        if (!config_.local_map_ray_clear_enabled || cells_.empty()) {
            return;
        }

        const auto origin = transformPoint(0.0, 0.0, scan_to_map);
        for (size_t i = 0; i < scan_msg.ranges.size(); ++i) {
            double clear_range = 0.0;
            if (!getRayClearRange(scan_msg, scan_msg.ranges[i], clear_range)) {
                continue;
            }

            const double angle = scan_msg.angle_min + static_cast<double>(i) * scan_msg.angle_increment;
            const auto end = transformPoint(
                clear_range * std::cos(angle),
                clear_range * std::sin(angle),
                scan_to_map);
            clearCellsAlongRay(origin, end);
        }
    }

    // 计算一条 scan 光束可以清除到的距离。
    bool getRayClearRange(
        const sensor_msgs::msg::LaserScan& scan_msg,
        float range,
        double& clear_range) const {
        const double hit_margin = std::max(config_.local_map_ray_clear_hit_margin, config_.local_map_resolution);
        if (std::isfinite(range)) {
            if (range < scan_msg.range_min) {
                return false;
            }
            if (range >= scan_msg.range_max) {
                clear_range = scan_msg.range_max;
                return clear_range >= scan_msg.range_min;
            }

            // 有命中点时只清到命中点之前，给真实障碍留出末端保护距离。
            clear_range = static_cast<double>(range) - hit_margin;
        } else if (std::isinf(range)) {
            // 无穷远表示这条光束在 range_max 内没有障碍，整条自由空间都可清除。
            clear_range = scan_msg.range_max;
        } else {
            return false;
        }

        clear_range = std::clamp(clear_range, 0.0, static_cast<double>(scan_msg.range_max));
        return clear_range >= scan_msg.range_min;
    }

    // 删除落在一条自由空间光束附近的历史障碍点。
    void clearCellsAlongRay(const ObstaclePoint2D& origin, const ObstaclePoint2D& end) {
        const double radius = std::max(config_.local_map_ray_clear_radius, config_.local_map_resolution);
        const double radius_sq = radius * radius;

        cells_.erase(
            std::remove_if(
                cells_.begin(),
                cells_.end(),
                [this, &origin, &end, radius_sq](const MapCell& cell) {
                    // 用点到线段距离判断该历史点是否落在当前光束自由空间内。
                    return pointToSegmentDistanceSq({cell.x, cell.y}, origin, end) <= radius_sq;
                }),
            cells_.end());
    }

    // 计算二维点到线段的最短距离平方。
    double pointToSegmentDistanceSq(
        const ObstaclePoint2D& point,
        const ObstaclePoint2D& start,
        const ObstaclePoint2D& end) const {
        const double vx = end.x - start.x;
        const double vy = end.y - start.y;
        const double wx = point.x - start.x;
        const double wy = point.y - start.y;
        const double len_sq = vx * vx + vy * vy;
        if (len_sq <= 1e-9) {
            const double dx = point.x - start.x;
            const double dy = point.y - start.y;
            return dx * dx + dy * dy;
        }

        const double t = std::clamp((wx * vx + wy * vy) / len_sq, 0.0, 1.0);
        const double proj_x = start.x + t * vx;
        const double proj_y = start.y + t * vy;
        const double dx = point.x - proj_x;
        const double dy = point.y - proj_y;
        return dx * dx + dy * dy;
    }

    // 使用 TF 把二维点转换到目标坐标系。
    ObstaclePoint2D transformPoint(
        double x,
        double y,
        const geometry_msgs::msg::TransformStamped& transform) const {
        geometry_msgs::msg::PointStamped input;
        geometry_msgs::msg::PointStamped output;
        input.point.x = x;
        input.point.y = y;
        input.point.z = 0.0;
        tf2::doTransform(input, output, transform);
        return {output.point.x, output.point.y};
    }

    // 写入障碍点；落在同一分辨率邻域内的点只刷新，不重复膨胀点数。
    void insertOrRefreshPoint(double x, double y, std::chrono::steady_clock::time_point now) {
        const double resolution = std::max(config_.local_map_resolution, 0.01);
        const double merge_dist_sq = resolution * resolution;

        for (auto& cell : cells_) {
            const double dx = cell.x - x;
            const double dy = cell.y - y;
            if (dx * dx + dy * dy <= merge_dist_sq) {
                // 对同一格点做轻微平滑，降低相机噪声造成的栅格抖动。
                cell.x = cell.x * 0.7 + x * 0.3;
                cell.y = cell.y * 0.7 + y * 0.3;
                cell.stamp = now;
                return;
            }
        }

        cells_.push_back({x, y, now});
    }

    // 删除超过生命周期的障碍记忆。
    void pruneStale(std::chrono::steady_clock::time_point now) {
        const double lifetime_sec = std::max(config_.local_map_lifetime_sec, 0.1);
        cells_.erase(
            std::remove_if(
                cells_.begin(),
                cells_.end(),
                [now, lifetime_sec](const MapCell& cell) {
                    return std::chrono::duration<double>(now - cell.stamp).count() > lifetime_sec;
                }),
            cells_.end());
    }

    // 只保留当前机器人周围局部窗口内的障碍点。
    void pruneOutsideWindow(const geometry_msgs::msg::TransformStamped& map_to_robot) {
        cells_.erase(
            std::remove_if(
                cells_.begin(),
                cells_.end(),
                [this, &map_to_robot](const MapCell& cell) {
                    const auto robot_point = transformPoint(cell.x, cell.y, map_to_robot);
                    return !isInsideLocalWindow(robot_point.x, robot_point.y);
                }),
            cells_.end());
    }

    // 判断点是否位于以机器人为中心的局部地图窗口内。
    bool isInsideLocalWindow(double x, double y) const {
        const double half_x = std::max(config_.local_map_size_x, 0.1) / 2.0;
        const double half_y = std::max(config_.local_map_size_y, 0.1) / 2.0;
        return x >= -half_x && x <= half_x && y >= -half_y && y <= half_y;
    }

    // 点数超限时保留最近观测，避免历史噪声拖慢控制循环。
    void limitPointCount() {
        const size_t max_points = static_cast<size_t>(std::max(config_.local_map_max_points, 1));
        if (cells_.size() <= max_points) {
            return;
        }

        std::sort(
            cells_.begin(),
            cells_.end(),
            [](const MapCell& lhs, const MapCell& rhs) {
                return lhs.stamp > rhs.stamp;
            });
        cells_.resize(max_points);
    }
};

#endif
