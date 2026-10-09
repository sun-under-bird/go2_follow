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
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "uwb_aoa_pkg/msg/lib_aoa_robot_msg.hpp"
#include "uwb_aoa_pkg/msg/uwb_target.hpp"
#include "go2_uwb_local_follow/uwb_target_store.hpp"

namespace go2_uwb_local_follow
{
// 分发原样厂家定位结果；多目标链路与旧人员目标话题共用安装外参。
class UwbTargetAdapterNode : public rclcpp::Node
{
public:
  // 初始化每 ID 独立状态、两种目标输出和持续运行的过期诊断。
  explicit UwbTargetAdapterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("uwb_target_adapter_node", options)
  {
    raw_topic_ = declare_parameter<std::string>("raw_topic", "/libAoa_robot_publisher");
    target_topic_ = declare_parameter<std::string>("target_topic", "/uwb/target_point");
    targets_topic_ = declare_parameter<std::string>("targets_topic", "/uwb/targets");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_footprint");
    person_target_id_ = declare_parameter<std::int64_t>("person_target_id", 1);
    valid_target_ids_ = declare_parameter<std::vector<std::int64_t>>(
      "valid_target_ids", std::vector<std::int64_t>{});
    sensor_offset_x_ = declare_parameter<double>("sensor_offset_x", 0.0);
    sensor_offset_y_ = declare_parameter<double>("sensor_offset_y", 0.0);
    sensor_yaw_ = declare_parameter<double>("sensor_yaw", 0.0);
    target_timeout_sec_ = declare_parameter<double>("target_timeout_sec", 0.50);
    minimum_confidence_ = declare_parameter<int>("minimum_confidence", 0);
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/uwb/target_adapter_diagnostics");
    diagnostic_frequency_ = declare_parameter<double>("diagnostic_frequency", 2.0);
    validateParameters();
    raw_sub_ = create_subscription<uwb_aoa_pkg::msg::LibAoaRobotMsg>(
      raw_topic_, rclcpp::QoS(64).reliable(),
      std::bind(&UwbTargetAdapterNode::rawTargetCallback, this, std::placeholders::_1));
    target_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(target_topic_, 10);
    targets_pub_ = create_publisher<uwb_aoa_pkg::msg::UwbTarget>(
      targets_topic_, rclcpp::QoS(64).reliable());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, 10);
    // 诊断不依赖输入回调，否则标签全部断流时就无法报告过期。
    diagnostic_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / diagnostic_frequency_)),
      std::bind(&UwbTargetAdapterNode::publishDiagnostics, this));
  }

private:
  // 检查外参、有效期及 uint32 标签编号，避免配置值截断成另一个 ID。
  void validateParameters()
  {
    const auto valid_id = [](std::int64_t id) {
        return id >= 0 && id <= std::numeric_limits<std::uint32_t>::max();
      };
    if (target_frame_.empty() || !std::isfinite(sensor_offset_x_) ||
      !std::isfinite(sensor_offset_y_) || !std::isfinite(sensor_yaw_) ||
      !std::isfinite(diagnostic_frequency_) || diagnostic_frequency_ <= 0.0 ||
      !std::isfinite(target_timeout_sec_) || target_timeout_sec_ <= 0.0 ||
      !valid_id(person_target_id_) || minimum_confidence_ < 0 || minimum_confidence_ > 255 ||
      !std::all_of(valid_target_ids_.begin(), valid_target_ids_.end(), valid_id))
    {
      throw std::invalid_argument("invalid UWB adapter parameters");
    }
    if (!allowed(static_cast<std::uint32_t>(person_target_id_))) {
      throw std::invalid_argument("person_target_id must be allowed by valid_target_ids");
    }
  }

  // 空列表接受所有标签，否则仅接受显式允许的厂家 ID。
  bool allowed(std::uint32_t id) const
  {
    return valid_target_ids_.empty() ||
           std::find(valid_target_ids_.begin(), valid_target_ids_.end(), id) !=
           valid_target_ids_.end();
  }

  // 按 ID 校验采集时间并转换二维安装外参；新无效样本也通知多目标控制器。
  void rawTargetCallback(const uwb_aoa_pkg::msg::LibAoaRobotMsg::SharedPtr message)
  {
    if (!allowed(message->fob_id)) {
      return;
    }
    const double cosine = std::cos(sensor_yaw_);
    const double sine = std::sin(sensor_yaw_);
    const double x = sensor_offset_x_ + cosine * message->x - sine * message->y;
    const double y = sensor_offset_y_ + sine * message->x + cosine * message->y;
    const bool valid = message->state >= 0 &&
      message->pos_confidence >= minimum_confidence_ && std::isfinite(x) && std::isfinite(y);
    if (!targets_.update(
        message->fob_id, sourceStampNanoseconds(message->header.stamp), now().nanoseconds(),
        target_timeout_sec_, std::chrono::steady_clock::now(),
        x, y, 0.0, message->pos_confidence, message->state, valid))
    {
      return;
    }
    uwb_aoa_pkg::msg::UwbTarget target;
    target.header = message->header;
    target.header.frame_id = target_frame_;
    target.target_id = message->fob_id;
    target.confidence = message->pos_confidence;
    target.state = message->state;
    target.valid = valid;
    // 无效消息只撤销许可，避免向下游 TF 运算传播 NaN。
    target.position.x = valid ? x : 0.0;
    target.position.y = valid ? y : 0.0;
    targets_pub_->publish(target);
    if (valid && message->fob_id == static_cast<std::uint32_t>(person_target_id_)) {
      geometry_msgs::msg::PointStamped person;
      person.header = target.header;
      person.point = target.position;
      target_pub_->publish(person);
    }
  }

  // 定时逐 ID 报告最后采集时间、采集与接收年龄、有效性及更新频率。
  void publishDiagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    const auto current = std::chrono::steady_clock::now();
    for (const auto & entry : targets_.records()) {
      const auto & record = entry.second;
      const bool fresh = targets_.fresh(
        entry.first, now().nanoseconds(), current, target_timeout_sec_);
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = get_fully_qualified_name() + std::string(": ID ") +
        std::to_string(entry.first);
      status.hardware_id = "uwb_aoa";
      status.level = fresh ? diagnostic_msgs::msg::DiagnosticStatus::OK :
        diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = fresh ? "UWB_TARGET_VALID" : "UWB_TARGET_INVALID_OR_STALE";
      const std::pair<std::string, std::string> entries[] = {
        {"target_id", std::to_string(entry.first)}, {"source_stamp_ns", std::to_string(
            record.stamp_ns)},
        {"source_age_sec", std::to_string(sourceAgeSeconds(record.stamp_ns, now().nanoseconds()))},
        {"receipt_age_sec", std::to_string(
            std::chrono::duration<double>(current - record.receipt).count())},
        {"valid", fresh ? "true" : "false"}, {"frequency_hz", std::to_string(record.frequency_hz)},
        {"state", std::to_string(record.state)},
        {"pos_confidence", std::to_string(record.confidence)},
        {"target_x", std::to_string(record.x)}, {"target_y", std::to_string(record.y)}};
      for (const auto & value : entries) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = value.first;
        item.value = value.second;
        status.values.push_back(item);
      }
      array.status.push_back(status);
    }
    diagnostics_pub_->publish(array);
  }

  UwbTargetStore targets_;
  std::string raw_topic_, target_topic_, targets_topic_, target_frame_, diagnostics_topic_;
  std::int64_t person_target_id_{1};
  std::vector<std::int64_t> valid_target_ids_;
  double sensor_offset_x_{0.0}, sensor_offset_y_{0.0}, sensor_yaw_{0.0};
  double diagnostic_frequency_{2.0}, target_timeout_sec_{0.50};
  int minimum_confidence_{0};
  rclcpp::Subscription<uwb_aoa_pkg::msg::LibAoaRobotMsg>::SharedPtr raw_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<uwb_aoa_pkg::msg::UwbTarget>::SharedPtr targets_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};
}  // namespace go2_uwb_local_follow

// 启动 UWB 目标适配器，配置错误以非零状态退出。
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
