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
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "go2_uwb_local_follow/stereo_projection_core.hpp"

namespace go2_uwb_local_follow
{
namespace
{

using DiagnosticValues = std::vector<std::pair<std::string, std::string>>;

struct PixelWindow
{
  std::size_t x_begin{0};
  std::size_t x_end{0};
  std::size_t y_begin{0};
  std::size_t y_end{0};
};

// 判断当前主机是否采用大端字节序。
bool hostIsBigEndian()
{
  const std::uint16_t marker = 0x0102U;
  const auto * bytes = reinterpret_cast<const std::uint8_t *>(&marker);
  return bytes[0] == 0x01U;
}

// 对一个 32 位无符号整数执行字节序翻转。
std::uint32_t swapUint32(std::uint32_t value)
{
  return ((value & 0x000000FFU) << 24U) |
         ((value & 0x0000FF00U) << 8U) |
         ((value & 0x00FF0000U) >> 8U) |
         ((value & 0xFF000000U) >> 24U);
}

// 从可能具有不同字节序的 32FC1 图像中读取一个像素。
bool readFloatPixel(
  const sensor_msgs::msg::Image & image,
  std::size_t u,
  std::size_t v,
  float * value)
{
  if (value == nullptr || u >= image.width || v >= image.height) {
    return false;
  }
  constexpr std::size_t kFloatBytes = sizeof(float);
  const std::size_t offset = v * static_cast<std::size_t>(image.step) + u * kFloatBytes;
  if (offset > image.data.size() || image.data.size() - offset < kFloatBytes) {
    return false;
  }

  std::uint32_t raw = 0U;
  std::memcpy(&raw, image.data.data() + offset, kFloatBytes);
  if ((image.is_bigendian != 0U) != hostIsBigEndian()) {
    raw = swapUint32(raw);
  }
  std::memcpy(value, &raw, kFloatBytes);
  return true;
}

// 将本机字节序浮点值写入 32FC1 图像缓冲区。
void writeFloatPixel(sensor_msgs::msg::Image & image, std::size_t u, std::size_t v, float value)
{
  const std::size_t offset =
    v * static_cast<std::size_t>(image.step) + u * sizeof(float);
  if (offset <= image.data.size() && image.data.size() - offset >= sizeof(float)) {
    std::memcpy(image.data.data() + offset, &value, sizeof(float));
  }
}

// 把视差消息的有效窗口裁剪到实际图像边界。
PixelWindow clippedValidWindow(const stereo_msgs::msg::DisparityImage & message)
{
  const std::size_t image_width = message.image.width;
  const std::size_t image_height = message.image.height;
  if (message.valid_window.width == 0U || message.valid_window.height == 0U) {
    return PixelWindow{0U, image_width, 0U, image_height};
  }

  const std::size_t x_begin = std::min<std::size_t>(
    message.valid_window.x_offset, image_width);
  const std::size_t y_begin = std::min<std::size_t>(
    message.valid_window.y_offset, image_height);
  const std::size_t x_end = std::min<std::size_t>(
    image_width, x_begin + static_cast<std::size_t>(message.valid_window.width));
  const std::size_t y_end = std::min<std::size_t>(
    image_height, y_begin + static_cast<std::size_t>(message.valid_window.height));
  return PixelWindow{x_begin, x_end, y_begin, y_end};
}

// 判断视差值是否满足消息声明的有效范围和正深度要求。
bool validDisparity(float disparity, const stereo_msgs::msg::DisparityImage & message)
{
  return std::isfinite(disparity) && disparity > 0.0F &&
         disparity >= message.min_disparity && disparity <= message.max_disparity;
}

// 把浮点数格式化为诊断话题使用的短字符串。
std::string formatDouble(double value, int precision = 3)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

// 计算平面距离平方，供输出点数限制作最近点排序。
double planarRangeSquared(const Point3D & point)
{
  return point.x * point.x + point.y * point.y;
}

}  // namespace

class StereoObstacleProjectorNode : public rclcpp::Node
{
public:
  // 初始化视差、相机内参、TF、过滤参数及障碍点云输出接口。
  explicit StereoObstacleProjectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("stereo_obstacle_projector_node", options),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    disparity_topic_ = declare_parameter<std::string>("disparity_topic", "/stereo/disparity");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/camera/camera/infra1/camera_info");
    obstacle_cloud_topic_ = declare_parameter<std::string>(
      "obstacle_cloud_topic", "/local_grid_obstacle");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/stereo/obstacle_diagnostics");
    depth_debug_topic_ = declare_parameter<std::string>(
      "depth_debug_topic", "/stereo/depth_debug");

    publish_debug_depth_ = declare_parameter<bool>("publish_debug_depth", false);
    pixel_stride_ = static_cast<std::size_t>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("pixel_stride", 2)));
    min_valid_disparity_samples_ = static_cast<std::size_t>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("min_valid_disparity_samples", 200)));
    max_output_points_ = static_cast<std::size_t>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("max_output_points", 20000)));
    min_points_per_voxel_ = static_cast<std::size_t>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("min_points_per_voxel", 3)));
    min_voxels_per_cluster_ = static_cast<std::size_t>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("min_voxels_per_cluster", 3)));
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.10);
    input_timeout_sec_ = declare_parameter<double>("input_timeout_sec", 0.60);

    config_.depth_min = declare_parameter<double>("depth_min", 0.20);
    config_.depth_max = declare_parameter<double>("depth_max", 3.00);
    config_.obstacle_x_min = declare_parameter<double>("obstacle_x_min", -0.50);
    config_.obstacle_x_max = declare_parameter<double>("obstacle_x_max", 3.00);
    config_.obstacle_y_abs_max = declare_parameter<double>("obstacle_y_abs_max", 2.00);
    config_.obstacle_z_min = declare_parameter<double>("obstacle_z_min", 0.10);
    config_.obstacle_z_max = declare_parameter<double>("obstacle_z_max", 0.50);
    config_.voxel_size = declare_parameter<double>("voxel_size", 0.05);

    validateParameters();

    auto sensor_qos = rclcpp::SensorDataQoS();
    sensor_qos.keep_last(1);
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, sensor_qos,
      std::bind(&StereoObstacleProjectorNode::cameraInfoCallback, this, std::placeholders::_1));
    disparity_sub_ = create_subscription<stereo_msgs::msg::DisparityImage>(
      disparity_topic_, sensor_qos,
      std::bind(&StereoObstacleProjectorNode::disparityCallback, this, std::placeholders::_1));

    obstacle_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      obstacle_cloud_topic_, sensor_qos);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, 10);
    if (publish_debug_depth_) {
      depth_debug_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_debug_topic_, 1);
    }

    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&StereoObstacleProjectorNode::watchdogTick, this));

    RCLCPP_INFO(
      get_logger(),
      "Stereo obstacle projector started: disparity=%s camera_info=%s output=%s frame=%s",
      disparity_topic_.c_str(), camera_info_topic_.c_str(), obstacle_cloud_topic_.c_str(),
      base_frame_.c_str());
  }

private:
  // 检查启动参数，避免以不一致的过滤边界运行。
  void validateParameters()
  {
    std::string reason;
    if (base_frame_.empty()) {
      throw std::invalid_argument("base_frame must not be empty");
    }
    if (!validateProjectionConfig(config_, &reason)) {
      throw std::invalid_argument(reason);
    }
    if (!std::isfinite(transform_timeout_sec_) || transform_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("transform_timeout_sec must be positive");
    }
    if (!std::isfinite(input_timeout_sec_) || input_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("input_timeout_sec must be positive");
    }
  }

  // 接收并缓存左目校正投影参数，不对有效内参做时间滤波。
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr message)
  {
    CameraCalibration calibration;
    calibration.fx = message->p[0];
    calibration.cx = message->p[2];
    calibration.cy = message->p[6];
    calibration.width = message->width;
    calibration.height = message->height;

    std::string reason;
    if (!validateCalibration(calibration, &reason)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Reject invalid camera info: %s", reason.c_str());
      publishDiagnostic(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "invalid camera calibration", {{"reason", reason}});
      return;
    }

    std::lock_guard<std::mutex> lock(calibration_mutex_);
    calibration_ = calibration;
    calibration_frame_ = message->header.frame_id;
    have_calibration_ = true;
  }

  // 返回最近一次经过校验的相机内参快照。
  bool calibrationSnapshot(CameraCalibration * calibration, std::string * frame)
  {
    if (calibration == nullptr || frame == nullptr) {
      return false;
    }
    std::lock_guard<std::mutex> lock(calibration_mutex_);
    if (!have_calibration_) {
      return false;
    }
    *calibration = calibration_;
    *frame = calibration_frame_;
    return true;
  }

  // 校验视差图结构、标定尺寸和双目几何参数。
  bool validateDisparityMessage(
    const stereo_msgs::msg::DisparityImage & message,
    const CameraCalibration & calibration,
    const std::string & calibration_frame,
    std::string * reason) const
  {
    const auto reject = [reason](const std::string & text) {
        if (reason != nullptr) {
          *reason = text;
        }
        return false;
      };

    if (message.header.frame_id.empty()) {
      return reject("disparity frame_id is empty");
    }
    if (!calibration_frame.empty() && calibration_frame != message.header.frame_id) {
      return reject("camera_info frame differs from disparity frame");
    }
    if (message.image.encoding != sensor_msgs::image_encodings::TYPE_32FC1) {
      return reject("disparity image encoding must be 32FC1");
    }
    if (message.image.width == 0U || message.image.height == 0U ||
      message.image.step < message.image.width * sizeof(float))
    {
      return reject("disparity image dimensions or row step are invalid");
    }
    const std::size_t required_size =
      static_cast<std::size_t>(message.image.step) * message.image.height;
    if (message.image.data.size() < required_size) {
      return reject("disparity image data is truncated");
    }
    if (message.image.width != calibration.width || message.image.height != calibration.height) {
      return reject("camera_info and disparity dimensions differ");
    }
    if (!std::isfinite(message.f) || !std::isfinite(message.t) || message.f <= 0.0F ||
      message.t <= 0.0F)
    {
      return reject("disparity focal length and baseline must be positive");
    }
    if (!std::isfinite(message.min_disparity) || !std::isfinite(message.max_disparity) ||
      message.max_disparity <= message.min_disparity)
    {
      return reject("disparity range is invalid");
    }
    const double focal_relative_error =
      std::abs(static_cast<double>(message.f) - calibration.fx) / calibration.fx;
    if (focal_relative_error > 0.02) {
      return reject("camera_info and disparity focal lengths differ by more than 2 percent");
    }
    return true;
  }

  // 按视差时间戳查询光学坐标系到机身坐标系的变换。
  bool lookupBaseTransform(
    const stereo_msgs::msg::DisparityImage & message,
    tf2::Matrix3x3 * rotation,
    tf2::Vector3 * translation,
    std::string * reason)
  {
    if (rotation == nullptr || translation == nullptr) {
      return false;
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(
        base_frame_, message.header.frame_id, rclcpp::Time(message.header.stamp),
        rclcpp::Duration::from_seconds(transform_timeout_sec_));
      const auto & rotation_message = transform.transform.rotation;
      tf2::Quaternion quaternion(
        rotation_message.x, rotation_message.y, rotation_message.z, rotation_message.w);
      if (quaternion.length2() <= std::numeric_limits<double>::epsilon()) {
        if (reason != nullptr) {
          *reason = "TF rotation quaternion has zero length";
        }
        return false;
      }
      quaternion.normalize();
      *rotation = tf2::Matrix3x3(quaternion);
      *translation = tf2::Vector3(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z);
      return true;
    } catch (const std::exception & exception) {
      if (reason != nullptr) {
        *reason = exception.what();
      }
      return false;
    }
  }

  // 接收一帧视差，完成稀疏反投影、TF、高度过滤和当前帧点云发布。
  void disparityCallback(const stereo_msgs::msg::DisparityImage::SharedPtr message)
  {
    const auto processing_start = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(receipt_mutex_);
      have_disparity_message_ = true;
      last_disparity_receipt_ = processing_start;
    }

    CameraCalibration calibration;
    std::string calibration_frame;
    if (!calibrationSnapshot(&calibration, &calibration_frame)) {
      publishDiagnostic(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, "waiting for left camera_info");
      return;
    }

    std::string reason;
    if (!validateDisparityMessage(*message, calibration, calibration_frame, &reason)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Reject disparity frame: %s", reason.c_str());
      publishDiagnostic(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "invalid disparity frame", {{"reason", reason}});
      return;
    }

    tf2::Matrix3x3 rotation;
    tf2::Vector3 translation;
    if (!lookupBaseTransform(*message, &rotation, &translation, &reason)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF failed from %s to %s: %s", message->header.frame_id.c_str(),
        base_frame_.c_str(), reason.c_str());
      publishDiagnostic(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "TF unavailable", {{"reason", reason}});
      return;
    }

    CameraCalibration frame_calibration = calibration;
    frame_calibration.fx = message->f;
    const auto window = clippedValidWindow(*message);
    std::vector<Point3D> base_points;
    const std::size_t sampled_width =
      window.x_end > window.x_begin ? (window.x_end - window.x_begin) / pixel_stride_ + 1U : 0U;
    const std::size_t sampled_height =
      window.y_end > window.y_begin ? (window.y_end - window.y_begin) / pixel_stride_ + 1U : 0U;
    base_points.reserve(sampled_width * sampled_height);

    std::size_t sampled_count = 0U;
    std::size_t valid_disparity_count = 0U;
    std::size_t depth_count = 0U;
    for (std::size_t v = window.y_begin; v < window.y_end; v += pixel_stride_) {
      for (std::size_t u = window.x_begin; u < window.x_end; u += pixel_stride_) {
        ++sampled_count;
        float disparity = 0.0F;
        if (!readFloatPixel(message->image, u, v, &disparity) ||
          !validDisparity(disparity, *message))
        {
          continue;
        }
        ++valid_disparity_count;
        const auto optical_point = projectDisparityPixel(
          u, v, disparity, message->t, frame_calibration, config_);
        if (!optical_point.has_value()) {
          continue;
        }
        ++depth_count;

        // 光学坐标为右、下、前，统一交给 TF 变换，禁止手工交换轴向。
        const tf2::Vector3 optical(
          optical_point->x, optical_point->y, optical_point->z);
        const tf2::Vector3 base = rotation * optical + translation;
        const Point3D base_point{base.x(), base.y(), base.z()};
        if (keepBasePoint(base_point, config_)) {
          base_points.push_back(base_point);
        }
      }
    }

    if (publish_debug_depth_) {
      publishDebugDepth(*message);
    }

    if (valid_disparity_count < min_valid_disparity_samples_) {
      publishDiagnostic(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "insufficient valid disparity samples",
        {{"sampled", std::to_string(sampled_count)},
          {"valid_disparity", std::to_string(valid_disparity_count)},
          {"required", std::to_string(min_valid_disparity_samples_)}});
      return;
    }

    auto obstacle_points = voxelDownsampleNearest(
      base_points, config_.voxel_size, min_points_per_voxel_, min_voxels_per_cluster_);
    if (obstacle_points.size() > max_output_points_) {
      // 点数超限时只保留最近障碍，避免远处点挤占局部规划预算。
      std::nth_element(
        obstacle_points.begin(), obstacle_points.begin() + max_output_points_,
        obstacle_points.end(),
        [](const Point3D & first, const Point3D & second) {
          return planarRangeSquared(first) < planarRangeSquared(second);
        });
      obstacle_points.resize(max_output_points_);
    }

    publishObstacleCloud(*message, obstacle_points);
    const auto processing_end = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(receipt_mutex_);
      have_valid_cloud_ = true;
      last_valid_cloud_receipt_ = processing_end;
    }
    const double processing_ms =
      std::chrono::duration<double, std::milli>(processing_end - processing_start).count();
    publishDiagnostic(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      obstacle_points.empty() ? "valid empty obstacle cloud" : "obstacle cloud valid",
      {{"sampled", std::to_string(sampled_count)},
        {"valid_disparity", std::to_string(valid_disparity_count)},
        {"within_depth", std::to_string(depth_count)},
        {"within_base_filter", std::to_string(base_points.size())},
        {"min_points_per_voxel", std::to_string(min_points_per_voxel_)},
        {"min_voxels_per_cluster", std::to_string(min_voxels_per_cluster_)},
        {"output_points", std::to_string(obstacle_points.size())},
        {"processing_ms", formatDouble(processing_ms)}});
  }

  // 发布只含 XYZ 字段且位于 base_footprint 的过滤点云。
  void publishObstacleCloud(
    const stereo_msgs::msg::DisparityImage & disparity,
    const std::vector<Point3D> & points)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = disparity.header.stamp;
    cloud.header.frame_id = base_frame_;
    cloud.height = 1U;
    cloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    for (const auto & point : points) {
      *iter_x = static_cast<float>(point.x);
      *iter_y = static_cast<float>(point.y);
      *iter_z = static_cast<float>(point.z);
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
    obstacle_cloud_pub_->publish(cloud);
  }

  // 可选发布以米为单位的完整 32FC1 深度图，默认关闭以节省带宽。
  void publishDebugDepth(const stereo_msgs::msg::DisparityImage & disparity)
  {
    if (!depth_debug_pub_) {
      return;
    }
    sensor_msgs::msg::Image depth;
    depth.header = disparity.header;
    depth.height = disparity.image.height;
    depth.width = disparity.image.width;
    depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    depth.is_bigendian = hostIsBigEndian() ? 1U : 0U;
    depth.step = depth.width * sizeof(float);
    depth.data.resize(static_cast<std::size_t>(depth.step) * depth.height);

    const float invalid = std::numeric_limits<float>::quiet_NaN();
    for (std::size_t v = 0U; v < depth.height; ++v) {
      for (std::size_t u = 0U; u < depth.width; ++u) {
        float disparity_value = 0.0F;
        float depth_value = invalid;
        if (readFloatPixel(disparity.image, u, v, &disparity_value) &&
          validDisparity(disparity_value, disparity))
        {
          const double depth_meters =
            static_cast<double>(disparity.f) * disparity.t / disparity_value;
          if (std::isfinite(depth_meters) && depth_meters > 0.0) {
            depth_value = static_cast<float>(depth_meters);
          }
        }
        writeFloatPixel(depth, u, v, depth_value);
      }
    }
    depth_debug_pub_->publish(depth);
  }

  // 发布结构化诊断状态及本帧处理统计。
  void publishDiagnostic(
    std::uint8_t level,
    const std::string & message,
    const DiagnosticValues & values = {})
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = level;
    status.name = get_fully_qualified_name() + std::string(": stereo obstacle projection");
    status.hardware_id = "stereo_camera";
    status.message = message;
    status.values.reserve(values.size());
    for (const auto & value : values) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = value.first;
      item.value = value.second;
      status.values.push_back(std::move(item));
    }
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(array);
  }

  // 周期检查视差输入和有效障碍帧是否已经超时。
  void watchdogTick()
  {
    const auto current = std::chrono::steady_clock::now();
    bool have_disparity = false;
    bool have_valid_cloud = false;
    std::chrono::steady_clock::time_point last_disparity;
    std::chrono::steady_clock::time_point last_valid_cloud;
    {
      std::lock_guard<std::mutex> lock(receipt_mutex_);
      have_disparity = have_disparity_message_;
      have_valid_cloud = have_valid_cloud_;
      last_disparity = last_disparity_receipt_;
      last_valid_cloud = last_valid_cloud_receipt_;
    }

    if (!have_disparity) {
      publishDiagnostic(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, "waiting for disparity input");
      return;
    }
    const double disparity_age = std::chrono::duration<double>(current - last_disparity).count();
    if (disparity_age > input_timeout_sec_) {
      publishDiagnostic(
        diagnostic_msgs::msg::DiagnosticStatus::STALE,
        "disparity input timeout", {{"age_sec", formatDouble(disparity_age)}});
      return;
    }
    if (!have_valid_cloud) {
      publishDiagnostic(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "no valid obstacle cloud has been produced");
      return;
    }
    const double valid_age = std::chrono::duration<double>(current - last_valid_cloud).count();
    if (valid_age > input_timeout_sec_) {
      publishDiagnostic(
        diagnostic_msgs::msg::DiagnosticStatus::STALE,
        "valid obstacle cloud timeout", {{"age_sec", formatDouble(valid_age)}});
    }
  }

  std::string base_frame_;
  std::string disparity_topic_;
  std::string camera_info_topic_;
  std::string obstacle_cloud_topic_;
  std::string diagnostics_topic_;
  std::string depth_debug_topic_;

  bool publish_debug_depth_{false};
  std::size_t pixel_stride_{2U};
  std::size_t min_valid_disparity_samples_{200U};
  std::size_t max_output_points_{20000U};
  std::size_t min_points_per_voxel_{3U};
  std::size_t min_voxels_per_cluster_{3U};
  double transform_timeout_sec_{0.10};
  double input_timeout_sec_{0.60};
  ProjectionConfig config_;

  std::mutex calibration_mutex_;
  CameraCalibration calibration_;
  std::string calibration_frame_;
  bool have_calibration_{false};

  std::mutex receipt_mutex_;
  bool have_disparity_message_{false};
  bool have_valid_cloud_{false};
  std::chrono::steady_clock::time_point last_disparity_receipt_{};
  std::chrono::steady_clock::time_point last_valid_cloud_receipt_{};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr disparity_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_cloud_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_debug_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace go2_uwb_local_follow

// 启动双目障碍点云投影节点并进入 ROS 事件循环。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<go2_uwb_local_follow::StereoObstacleProjectorNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("stereo_obstacle_projector_node"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
