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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "go2_uwb_local_follow/rolling_obstacle_map_core.hpp"

namespace go2_uwb_local_follow
{
namespace
{

constexpr double kNanosecondsPerSecond = 1.0e9;

// 判断 PointCloud2 是否包含指定字段。
bool cloudHasField(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  return std::any_of(
    cloud.fields.begin(), cloud.fields.end(),
    [&name](const sensor_msgs::msg::PointField & field) {
      return field.name == name;
    });
}

// 将有限浮点数格式化为诊断话题使用的短字符串。
std::string formatDouble(double value, int precision = 3)
{
  if (!std::isfinite(value)) {
    return "inf";
  }
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

// 将 ROS 内置时间戳转换为不带时钟类型的纳秒整数。
std::int64_t stampNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * static_cast<std::int64_t>(kNanosecondsPerSecond) +
         static_cast<std::int64_t>(stamp.nanosec);
}

// 从经过归一化校验的四元数提取平面 yaw。
bool quaternionYaw(
  const geometry_msgs::msg::Quaternion & quaternion,
  double * yaw)
{
  if (yaw == nullptr) {
    return false;
  }
  const double norm_squared = quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  if (!std::isfinite(norm_squared) || norm_squared <= std::numeric_limits<double>::epsilon()) {
    return false;
  }
  const double inverse_norm = 1.0 / std::sqrt(norm_squared);
  const double x = quaternion.x * inverse_norm;
  const double y = quaternion.y * inverse_norm;
  const double z = quaternion.z * inverse_norm;
  const double w = quaternion.w * inverse_norm;
  *yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  return std::isfinite(*yaw);
}

}  // namespace

class RollingObstacleMapNode : public rclcpp::Node
{
public:
  // 初始化里程计缓存、滚动障碍地图、点云接口和故障诊断。
  explicit RollingObstacleMapNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("rolling_obstacle_map_node", options),
    pose_buffer_(declareMapConfig()),
    obstacle_map_(map_config_)
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    odom_child_frame_ = declare_parameter<std::string>("odom_child_frame", "base_footprint");
    input_observation_topic_ = declare_parameter<std::string>(
      "input_observation_topic", "/local_depth_observation");
    output_obstacle_topic_ = declare_parameter<std::string>(
      "output_obstacle_topic", "/local_rolling_obstacle");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom_leg");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/go2_uwb_local_follow/rolling_map_diagnostics");
    input_timeout_sec_ = declare_parameter<double>("input_timeout_sec", 0.60);
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 0.15);
    diagnostic_frequency_ = declare_parameter<double>("diagnostic_frequency", 2.0);
    validateParameters();

    auto cloud_qos = rclcpp::SensorDataQoS();
    cloud_qos.keep_last(1);
    auto odom_qos = rclcpp::SensorDataQoS();
    odom_qos.keep_last(200);
    obstacle_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_observation_topic_, cloud_qos,
      std::bind(&RollingObstacleMapNode::observationCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, odom_qos,
      std::bind(&RollingObstacleMapNode::odomCallback, this, std::placeholders::_1));
    obstacle_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_obstacle_topic_, cloud_qos);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, 10);

    diagnostic_period_ = std::chrono::duration<double>(1.0 / diagnostic_frequency_);
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&RollingObstacleMapNode::watchdogTick, this));

    RCLCPP_INFO(
      get_logger(),
      "Rolling obstacle map started: input=%s odom=%s output=%s retention=%.2fs radius=%.2fm",
      input_observation_topic_.c_str(), odom_topic_.c_str(), output_obstacle_topic_.c_str(),
      map_config_.obstacle_retention_sec, map_config_.rolling_radius);
  }

private:
  struct StatusSnapshot
  {
    std::string state{"WAIT_ODOM"};
    std::string reason;
    double input_age{std::numeric_limits<double>::infinity()};
    double odom_age{std::numeric_limits<double>::infinity()};
    std::size_t input_points{0U};
    std::size_t obstacle_points{0U};
    std::size_t ray_endpoints{0U};
    std::size_t ray_cleared_cells{0U};
    std::size_t map_points{0U};
    std::size_t odom_buffer_size{0U};
    std::size_t odom_reset_count{0U};
  };

  // 声明核心地图参数，并在成员初始化阶段保存同一份配置。
  RollingMapConfig declareMapConfig()
  {
    map_config_.voxel_size = declare_parameter<double>("voxel_size", 0.05);
    map_config_.obstacle_retention_sec = declare_parameter<double>(
      "obstacle_retention_sec", 1.00);
    map_config_.rolling_radius = declare_parameter<double>("rolling_radius", 3.00);
    map_config_.max_obstacle_points = static_cast<std::size_t>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("max_obstacle_points", 5000)));
    map_config_.odom_buffer_duration_sec = declare_parameter<double>(
      "odom_buffer_duration_sec", 3.00);
    map_config_.max_pose_extrapolation_sec = declare_parameter<double>(
      "max_pose_extrapolation_sec", 0.05);
    map_config_.odom_jump_distance = declare_parameter<double>("odom_jump_distance", 1.00);
    map_config_.odom_jump_yaw = declare_parameter<double>("odom_jump_yaw", 0.80);
    map_config_.odom_jump_check_interval_sec = declare_parameter<double>(
      "odom_jump_check_interval_sec", 0.50);
    map_config_.enable_ray_clearing = declare_parameter<bool>("enable_ray_clearing", true);
    map_config_.ray_clearing_max_range = declare_parameter<double>(
      "ray_clearing_max_range", 3.00);
    map_config_.ray_clearing_endpoint_margin = declare_parameter<double>(
      "ray_clearing_endpoint_margin", 0.10);
    map_config_.ray_clearing_min_observations = static_cast<std::size_t>(
      std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("ray_clearing_min_observations", 2)));
    return map_config_;
  }

  // 校验坐标系、话题、地图范围、超时和诊断频率参数。
  void validateParameters()
  {
    std::string reason;
    if (!validateRollingMapConfig(map_config_, &reason)) {
      throw std::invalid_argument(reason);
    }
    if (base_frame_.empty() || odom_frame_.empty() || odom_child_frame_.empty() ||
      input_observation_topic_.empty() || output_obstacle_topic_.empty() || odom_topic_.empty() ||
      diagnostics_topic_.empty())
    {
      throw std::invalid_argument("rolling map frame and topic names must not be empty");
    }
    if (!std::isfinite(input_timeout_sec_) || input_timeout_sec_ <= 0.0 ||
      !std::isfinite(odom_timeout_sec_) || odom_timeout_sec_ <= 0.0 ||
      !std::isfinite(diagnostic_frequency_) || diagnostic_frequency_ <= 0.0)
    {
      throw std::invalid_argument("rolling map timeouts and diagnostic frequency must be positive");
    }
  }

  // 校验并缓存 /odom_leg 的二维位置和朝向，检测跳变后清空旧障碍地图。
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    last_odom_receipt_ = std::chrono::steady_clock::now();
    have_odom_receipt_ = true;
    if (message->header.frame_id != odom_frame_ || message->child_frame_id != odom_child_frame_) {
      have_valid_odom_ = false;
      setStatus("ODOM_FRAME_INVALID", "odom frame_id or child_frame_id differs from configuration");
      return;
    }

    TimedPose2D pose;
    pose.stamp_ns = stampNanoseconds(message->header.stamp);
    pose.x = message->pose.pose.position.x;
    pose.y = message->pose.pose.position.y;
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
      !quaternionYaw(message->pose.pose.orientation, &pose.yaw))
    {
      have_valid_odom_ = false;
      setStatus("ODOM_INVALID", "odom pose or orientation is invalid");
      return;
    }

    const PoseAppendResult result = pose_buffer_.append(pose);
    if (result == PoseAppendResult::kRejected) {
      setStatus("ODOM_INVALID", "odom timestamp is zero or duplicated");
      return;
    }
    if (result == PoseAppendResult::kResetDetected) {
      obstacle_map_.clear();
      ++odom_reset_count_;
      last_cloud_stamp_ns_ = 0;
      RCLCPP_WARN(get_logger(), "Odom jump or time reset detected; cleared rolling obstacle map");
    }
    have_valid_odom_ = true;
    if (!have_input_receipt_) {
      setStatus("WAIT_OBSERVATION", "waiting for depth observation cloud");
    }
  }

  // 解析原子化深度观测；兼容没有射线元数据的旧障碍点云输入。
  bool parseObservationCloud(
    const sensor_msgs::msg::PointCloud2 & message,
    std::vector<RollingObstaclePoint> * obstacles,
    std::vector<RollingObstaclePoint> * ray_endpoints,
    RollingObstaclePoint * sensor_origin,
    std::string * reason) const
  {
    if (obstacles == nullptr || ray_endpoints == nullptr || sensor_origin == nullptr) {
      return false;
    }
    const bool has_intensity = cloudHasField(message, "intensity");
    const bool has_viewpoint_x = cloudHasField(message, "vp_x");
    const bool has_viewpoint_y = cloudHasField(message, "vp_y");
    const bool has_viewpoint_z = cloudHasField(message, "vp_z");
    const bool has_any_metadata =
      has_intensity || has_viewpoint_x || has_viewpoint_y || has_viewpoint_z;
    const bool has_all_metadata =
      has_intensity && has_viewpoint_x && has_viewpoint_y && has_viewpoint_z;
    if (has_any_metadata && !has_all_metadata) {
      if (reason != nullptr) {
        *reason = "depth observation metadata fields are incomplete";
      }
      return false;
    }

    const std::size_t point_count =
      static_cast<std::size_t>(message.width) * static_cast<std::size_t>(message.height);
    obstacles->reserve(point_count);
    if (!has_all_metadata) {
      sensor_msgs::PointCloud2ConstIterator<float> x_iterator(message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y_iterator(message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z_iterator(message, "z");
      for (; x_iterator != x_iterator.end(); ++x_iterator, ++y_iterator, ++z_iterator) {
        const RollingObstaclePoint point{
          static_cast<double>(*x_iterator),
          static_cast<double>(*y_iterator),
          static_cast<double>(*z_iterator)};
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
          obstacles->push_back(point);
        }
      }
      return true;
    }

    ray_endpoints->reserve(point_count);
    sensor_msgs::PointCloud2ConstIterator<float> x_iterator(message, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y_iterator(message, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z_iterator(message, "z");
    sensor_msgs::PointCloud2ConstIterator<float> intensity_iterator(message, "intensity");
    sensor_msgs::PointCloud2ConstIterator<float> viewpoint_x_iterator(message, "vp_x");
    sensor_msgs::PointCloud2ConstIterator<float> viewpoint_y_iterator(message, "vp_y");
    sensor_msgs::PointCloud2ConstIterator<float> viewpoint_z_iterator(message, "vp_z");
    bool have_sensor_origin = false;
    for (; x_iterator != x_iterator.end();
      ++x_iterator, ++y_iterator, ++z_iterator, ++intensity_iterator,
      ++viewpoint_x_iterator, ++viewpoint_y_iterator, ++viewpoint_z_iterator)
    {
      const RollingObstaclePoint point{
        static_cast<double>(*x_iterator),
        static_cast<double>(*y_iterator),
        static_cast<double>(*z_iterator)};
      const RollingObstaclePoint point_origin{
        static_cast<double>(*viewpoint_x_iterator),
        static_cast<double>(*viewpoint_y_iterator),
        static_cast<double>(*viewpoint_z_iterator)};
      const double intensity = static_cast<double>(*intensity_iterator);
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
        !std::isfinite(point_origin.x) || !std::isfinite(point_origin.y) ||
        !std::isfinite(point_origin.z) || !std::isfinite(intensity))
      {
        continue;
      }
      if (!have_sensor_origin) {
        *sensor_origin = point_origin;
        have_sensor_origin = true;
      } else {
        if (
          std::abs(sensor_origin->x - point_origin.x) > 1.0e-4 ||
          std::abs(sensor_origin->y - point_origin.y) > 1.0e-4 ||
          std::abs(sensor_origin->z - point_origin.z) > 1.0e-4)
        {
          if (reason != nullptr) {
            *reason = "depth observation contains inconsistent sensor origins";
          }
          return false;
        }
      }
      if (intensity >= 0.5) {
        obstacles->push_back(point);
      } else {
        ray_endpoints->push_back(point);
      }
    }
    return true;
  }

  // 使用观测时间查询里程计位姿，以同帧射线清除历史体素并发布补偿障碍。
  void observationCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    last_input_receipt_ = std::chrono::steady_clock::now();
    have_input_receipt_ = true;
    if (message->header.frame_id != base_frame_) {
      setStatus("CLOUD_FRAME_INVALID", "depth observation frame differs from base_frame");
      return;
    }
    const std::int64_t cloud_stamp_ns = stampNanoseconds(message->header.stamp);
    if (cloud_stamp_ns <= 0 || cloud_stamp_ns <= last_cloud_stamp_ns_) {
      setStatus("CLOUD_TIME_INVALID", "depth observation timestamp is zero or not increasing");
      return;
    }
    if (!have_valid_odom_) {
      setStatus("WAIT_ODOM", "no valid odom pose is available");
      return;
    }

    TimedPose2D cloud_pose;
    if (!pose_buffer_.lookup(cloud_stamp_ns, &cloud_pose)) {
      setStatus("ODOM_LOOKUP_FAILED", "cloud timestamp is outside odom pose buffer");
      return;
    }

    std::vector<RollingObstaclePoint> obstacle_points;
    std::vector<RollingObstaclePoint> ray_endpoints;
    RollingObstaclePoint sensor_origin;
    std::string reason;
    try {
      if (!parseObservationCloud(
          *message, &obstacle_points, &ray_endpoints, &sensor_origin, &reason))
      {
        setStatus("CLOUD_INVALID", reason);
        return;
      }
    } catch (const std::runtime_error & exception) {
      setStatus("CLOUD_INVALID", exception.what());
      return;
    }

    const std::size_t cleared_cells = obstacle_map_.integrateObservation(
      obstacle_points, ray_endpoints, sensor_origin, cloud_pose);
    const auto output_points = obstacle_map_.pointsInBase(cloud_pose);
    last_cloud_stamp_ns_ = cloud_stamp_ns;
    publishObstacleCloud(message->header.stamp, output_points);
    status_.input_points = obstacle_points.size() + ray_endpoints.size();
    status_.obstacle_points = obstacle_points.size();
    status_.ray_endpoints = ray_endpoints.size();
    status_.ray_cleared_cells = cleared_cells;
    status_.map_points = output_points.size();
    status_.odom_buffer_size = pose_buffer_.size();
    status_.odom_reset_count = odom_reset_count_;
    setStatus("MAP_VALID", output_points.empty() ? "valid empty rolling map" : "rolling map valid");
    publishDiagnosticIfDue(false);
  }

  // 发布补偿到当前点云时刻 base_footprint 的滚动障碍点云。
  void publishObstacleCloud(
    const builtin_interfaces::msg::Time & stamp,
    const std::vector<RollingObstaclePoint> & points)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = base_frame_;
    cloud.height = 1U;
    cloud.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x_iterator(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y_iterator(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z_iterator(cloud, "z");
    for (const auto & point : points) {
      *x_iterator = static_cast<float>(point.x);
      *y_iterator = static_cast<float>(point.y);
      *z_iterator = static_cast<float>(point.z);
      ++x_iterator;
      ++y_iterator;
      ++z_iterator;
    }
    obstacle_pub_->publish(cloud);
  }

  // 保存最新诊断状态和可选原因，实际发布由限频函数统一处理。
  void setStatus(const std::string & state, const std::string & reason)
  {
    status_.state = state;
    status_.reason = reason;
  }

  // 周期检查真实点云和里程计接收是否超时，避免滚动地图掩盖传感器断流。
  void watchdogTick()
  {
    const auto current = std::chrono::steady_clock::now();
    status_.input_age = have_input_receipt_ ?
      std::chrono::duration<double>(current - last_input_receipt_).count() :
      std::numeric_limits<double>::infinity();
    status_.odom_age = have_odom_receipt_ ?
      std::chrono::duration<double>(current - last_odom_receipt_).count() :
      std::numeric_limits<double>::infinity();
    status_.map_points = obstacle_map_.size();
    status_.odom_buffer_size = pose_buffer_.size();
    status_.odom_reset_count = odom_reset_count_;

    if (!have_odom_receipt_) {
      setStatus("WAIT_ODOM", "waiting for odom input");
    } else if (status_.odom_age > odom_timeout_sec_) {
      setStatus("ODOM_TIMEOUT", "odom input timeout");
    } else if (!have_input_receipt_) {
      setStatus("WAIT_OBSERVATION", "waiting for depth observation cloud");
    } else if (status_.input_age > input_timeout_sec_) {
      setStatus("SENSOR_TIMEOUT", "depth observation cloud timeout");
    }
    publishDiagnosticIfDue(false);
  }

  // 按限制频率发布输入时效、地图点数、缓存规模和重置次数。
  void publishDiagnosticIfDue(bool force)
  {
    const auto current = std::chrono::steady_clock::now();
    if (!force && have_diagnostic_time_ && current - last_diagnostic_time_ < diagnostic_period_) {
      return;
    }
    have_diagnostic_time_ = true;
    last_diagnostic_time_ = current;

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus diagnostic;
    diagnostic.level = status_.state == "MAP_VALID" ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diagnostic.name = get_fully_qualified_name() + std::string(": rolling obstacle map");
    diagnostic.hardware_id = "odom_compensated_local_map";
    diagnostic.message = status_.state;
    const std::pair<std::string, std::string> entries[] = {
      {"reason", status_.reason},
      {"input_age_sec", formatDouble(status_.input_age)},
      {"odom_age_sec", formatDouble(status_.odom_age)},
      {"input_points", std::to_string(status_.input_points)},
      {"obstacle_points", std::to_string(status_.obstacle_points)},
      {"ray_endpoints", std::to_string(status_.ray_endpoints)},
      {"ray_cleared_cells", std::to_string(status_.ray_cleared_cells)},
      {"map_points", std::to_string(status_.map_points)},
      {"odom_buffer_size", std::to_string(status_.odom_buffer_size)},
      {"odom_reset_count", std::to_string(status_.odom_reset_count)}};
    for (const auto & entry : entries) {
      diagnostic_msgs::msg::KeyValue value;
      value.key = entry.first;
      value.value = entry.second;
      diagnostic.values.push_back(std::move(value));
    }
    array.status.push_back(std::move(diagnostic));
    diagnostics_pub_->publish(array);
  }

  RollingMapConfig map_config_;
  OdomPoseBuffer pose_buffer_;
  RollingObstacleMap obstacle_map_;

  std::string base_frame_;
  std::string odom_frame_;
  std::string odom_child_frame_;
  std::string input_observation_topic_;
  std::string output_obstacle_topic_;
  std::string odom_topic_;
  std::string diagnostics_topic_;
  double input_timeout_sec_{0.60};
  double odom_timeout_sec_{0.15};
  double diagnostic_frequency_{2.0};

  StatusSnapshot status_;
  bool have_odom_receipt_{false};
  bool have_valid_odom_{false};
  bool have_input_receipt_{false};
  std::int64_t last_cloud_stamp_ns_{0};
  std::size_t odom_reset_count_{0U};
  std::chrono::steady_clock::time_point last_odom_receipt_{};
  std::chrono::steady_clock::time_point last_input_receipt_{};
  std::chrono::duration<double> diagnostic_period_{0.5};
  bool have_diagnostic_time_{false};
  std::chrono::steady_clock::time_point last_diagnostic_time_{};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace go2_uwb_local_follow

// 启动里程计补偿滚动障碍地图节点并进入 ROS 事件循环。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<go2_uwb_local_follow::RollingObstacleMapNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("rolling_obstacle_map_node"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
