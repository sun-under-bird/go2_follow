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

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "uwb_aoa_pkg/msg/lib_aoa_robot_msg.hpp"

namespace go2_uwb_local_follow
{
namespace
{

// 把浮点数格式化为诊断话题使用的短字符串。
std::string formatDouble(double value, int precision = 3)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

}  // namespace

class UwbTargetAdapterNode : public rclcpp::Node
{
public:
  // 初始化厂家 UWB 输入、二维安装外参和带时间戳目标点输出。
  explicit UwbTargetAdapterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("uwb_target_adapter_node", options)
  {
    raw_topic_ = declare_parameter<std::string>(
      "raw_topic", "/libAoa_robot_publisher");
    target_topic_ = declare_parameter<std::string>("target_topic", "/uwb/target_point");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_footprint");
    sensor_offset_x_ = declare_parameter<double>("sensor_offset_x", 0.0);
    sensor_offset_y_ = declare_parameter<double>("sensor_offset_y", 0.0);
    sensor_yaw_ = declare_parameter<double>("sensor_yaw", 0.0);
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/uwb/target_adapter_diagnostics");
    diagnostic_frequency_ = declare_parameter<double>("diagnostic_frequency", 2.0);
    validateParameters();

    raw_sub_ = create_subscription<uwb_aoa_pkg::msg::LibAoaRobotMsg>(
      raw_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      std::bind(&UwbTargetAdapterNode::rawTargetCallback, this, std::placeholders::_1));
    target_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      target_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, 10);

    diagnostic_period_ = std::chrono::duration<double>(1.0 / diagnostic_frequency_);
    RCLCPP_INFO(
      get_logger(), "UWB target adapter started: raw=%s target=%s frame=%s",
      raw_topic_.c_str(), target_topic_.c_str(), target_frame_.c_str());
  }

private:
  // 检查目标坐标系、安装外参和诊断频率参数。
  void validateParameters()
  {
    if (target_frame_.empty()) {
      throw std::invalid_argument("target_frame must not be empty");
    }
    if (!std::isfinite(sensor_offset_x_) || !std::isfinite(sensor_offset_y_) ||
      !std::isfinite(sensor_yaw_))
    {
      throw std::invalid_argument("UWB sensor extrinsics must be finite");
    }
    if (!std::isfinite(diagnostic_frequency_) || diagnostic_frequency_ <= 0.0) {
      throw std::invalid_argument("diagnostic_frequency must be positive");
    }
  }

  // 接收厂家 x/y，每帧立即赋接收时间戳并转换到配置的机身二维坐标。
  void rawTargetCallback(const uwb_aoa_pkg::msg::LibAoaRobotMsg::SharedPtr message)
  {
    if (!std::isfinite(message->x) || !std::isfinite(message->y)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Reject non-finite UWB x/y sample");
      publishDiagnostic(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "non-finite UWB x/y", *message, 0.0, 0.0, true);
      return;
    }

    const double cosine = std::cos(sensor_yaw_);
    const double sine = std::sin(sensor_yaw_);
    const double target_x = sensor_offset_x_ + cosine * message->x - sine * message->y;
    const double target_y = sensor_offset_y_ + sine * message->x + cosine * message->y;

    geometry_msgs::msg::PointStamped target;
    // 统一使用本节点接收时刻，避免驱动时钟与控制节点时钟不一致造成目标误判过期。
    target.header.stamp = now();
    target.header.frame_id = target_frame_;
    target.point.x = target_x;
    target.point.y = target_y;
    target.point.z = 0.0;
    target_pub_->publish(target);
    publishDiagnostic(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "UWB target forwarded", *message, target_x, target_y, false);
  }

  // 按限制频率发布厂家状态和适配后的目标坐标，不参与目标有效性门控。
  void publishDiagnostic(
    std::uint8_t level,
    const std::string & status_message,
    const uwb_aoa_pkg::msg::LibAoaRobotMsg & raw,
    double target_x,
    double target_y,
    bool force)
  {
    const auto current = std::chrono::steady_clock::now();
    if (!force && have_diagnostic_time_ &&
      current - last_diagnostic_time_ < diagnostic_period_)
    {
      return;
    }
    have_diagnostic_time_ = true;
    last_diagnostic_time_ = current;

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = level;
    status.name = get_fully_qualified_name() + std::string(": UWB target adapter");
    status.hardware_id = "uwb_aoa";
    status.message = status_message;
    const std::pair<std::string, std::string> entries[] = {
      {"raw_x", formatDouble(raw.x)},
      {"raw_y", formatDouble(raw.y)},
      {"target_x", formatDouble(target_x)},
      {"target_y", formatDouble(target_y)},
      {"state", std::to_string(raw.state)},
      {"pos_confidence", std::to_string(raw.pos_confidence)}};
    for (const auto & entry : entries) {
      diagnostic_msgs::msg::KeyValue value;
      value.key = entry.first;
      value.value = entry.second;
      status.values.push_back(std::move(value));
    }
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(array);
  }

  std::string raw_topic_;
  std::string target_topic_;
  std::string target_frame_;
  std::string diagnostics_topic_;
  double sensor_offset_x_{0.0};
  double sensor_offset_y_{0.0};
  double sensor_yaw_{0.0};
  double diagnostic_frequency_{2.0};
  std::chrono::duration<double> diagnostic_period_{0.5};
  bool have_diagnostic_time_{false};
  std::chrono::steady_clock::time_point last_diagnostic_time_{};

  rclcpp::Subscription<uwb_aoa_pkg::msg::LibAoaRobotMsg>::SharedPtr raw_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
};

}  // namespace go2_uwb_local_follow

// 启动厂家 UWB 目标适配节点并进入 ROS 事件循环。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<go2_uwb_local_follow::UwbTargetAdapterNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("uwb_target_adapter_node"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
