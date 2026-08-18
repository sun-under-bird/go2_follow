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

#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "common_types.hpp"
#include "follow_avoid_controller.hpp"
#include "local_obstacle_map.hpp"

class RobotNexusNode : public rclcpp::Node
{
public:
  // 初始化原始 UWB 跟随、相机 scan 输入、局部地图和直接速度发布。
  RobotNexusNode()
  : Node("robot_nexus")
  {
    const auto config = loadConfig();
    local_map_enabled_ = config.local_map_enabled;

    const std::string scan_topic = this->get_parameter("scan_topic").as_string();
    const std::string uwb_target_topic = this->get_parameter("uwb_target_topic").as_string();
    const std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    uwb_input_frame_ = this->get_parameter("uwb_input_frame").as_string();
    local_map_frame_ = this->get_parameter("local_map_frame").as_string();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    local_map_ = std::make_unique<LocalObstacleMap>(config);
    controller_ = std::make_unique<FollowAvoidController>(shared_state_, config);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 1);
    controller_->setVelocityCallback(
      [this](const geometry_msgs::msg::Twist & cmd) {
        cmd_vel_pub_->publish(cmd);
      });

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&RobotNexusNode::scanCallback, this, std::placeholders::_1));

    uwb_target_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      uwb_target_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&RobotNexusNode::uwbTargetCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "robot_nexus started: scan=%s, uwb_target=%s, cmd_vel=%s, "
      "follow_dist=%.2f, target_frame=%s, local_map=%s/%s",
      scan_topic.c_str(),
      uwb_target_topic.c_str(),
      cmd_vel_topic.c_str(),
      config.follow_dist,
      target_frame_.c_str(),
      local_map_enabled_ ? "on" : "off",
      local_map_frame_.c_str());
  }

private:
  SharedState shared_state_;
  std::unique_ptr<FollowAvoidController> controller_;
  std::unique_ptr<LocalObstacleMap> local_map_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr uwb_target_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool local_map_enabled_ = true;
  std::string target_frame_ = "base_footprint";
  std::string uwb_input_frame_ = "base_footprint";
  std::string local_map_frame_ = "odom";

  // 声明 ROS 参数，并同步到控制器和局部地图配置。
  FollowConfig loadConfig()
  {
    this->declare_parameter<bool>("active", true);
    this->declare_parameter<std::string>("scan_topic", "/scan");
    this->declare_parameter<std::string>("uwb_target_topic", "/uwb/target_point");
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<std::string>("target_frame", "base_footprint");
    this->declare_parameter<std::string>("uwb_input_frame", "base_footprint");
    this->declare_parameter<std::string>("local_map_frame", "odom");
    this->declare_parameter<double>("follow_dist", 1.0);
    this->declare_parameter<double>("target_exclusion_radius", 0.35);
    this->declare_parameter<double>("apf_influence_dist", 0.6);
    this->declare_parameter<double>("apf_slowdown_dist", 0.6);
    this->declare_parameter<double>("apf_emergency_dist", 0.3);
    this->declare_parameter<double>("apf_repulse_gain", 0.01);
    this->declare_parameter<double>("max_linear_speed", 0.5);
    this->declare_parameter<double>("max_lateral_speed", 0.12);
    this->declare_parameter<double>("max_angular_speed", 1.0);
    this->declare_parameter<double>("linear_scale_factor", 0.5);
    this->declare_parameter<double>("linear_y_scale_factor", 1.0);
    this->declare_parameter<double>("angular_scale_factor", 1.0);
    this->declare_parameter<double>("distance_deadband", 0.05);
    this->declare_parameter<double>("lateral_deadband", 0.03);
    this->declare_parameter<double>("angle_deadband", 0.08);
    this->declare_parameter<double>("rotate_only_angle", 0.45);
    this->declare_parameter<double>("min_forward_speed", 0.06);
    this->declare_parameter<double>("rectangle_width", 0.4);
    this->declare_parameter<double>("robot_frame_front", 0.25);
    this->declare_parameter<double>("robot_frame_back", 0.25);
    this->declare_parameter<double>("robot_frame_left", 0.16);
    this->declare_parameter<double>("robot_frame_right", 0.16);
    this->declare_parameter<bool>("local_map_enabled", true);
    this->declare_parameter<double>("local_map_size_x", 1.0);
    this->declare_parameter<double>("local_map_size_y", 1.0);
    this->declare_parameter<double>("local_map_resolution", 0.05);
    this->declare_parameter<double>("local_map_lifetime_sec", 2.0);
    this->declare_parameter<int>("local_map_max_points", 1600);
    this->declare_parameter<bool>("local_map_ray_clear_enabled", true);
    this->declare_parameter<double>("local_map_ray_clear_radius", 0.06);
    this->declare_parameter<double>("local_map_ray_clear_hit_margin", 0.08);

    FollowConfig config;
    config.active = this->get_parameter("active").as_bool();
    config.follow_dist = this->get_parameter("follow_dist").as_double();
    config.target_exclusion_radius = this->get_parameter("target_exclusion_radius").as_double();
    config.apf_influence_dist = this->get_parameter("apf_influence_dist").as_double();
    config.apf_slowdown_dist = this->get_parameter("apf_slowdown_dist").as_double();
    config.apf_emergency_dist = this->get_parameter("apf_emergency_dist").as_double();
    config.apf_repulse_gain = this->get_parameter("apf_repulse_gain").as_double();
    config.max_linear_speed = this->get_parameter("max_linear_speed").as_double();
    config.max_lateral_speed = this->get_parameter("max_lateral_speed").as_double();
    config.max_angular_speed = this->get_parameter("max_angular_speed").as_double();
    config.linear_scale_factor = this->get_parameter("linear_scale_factor").as_double();
    config.linear_y_scale_factor = this->get_parameter("linear_y_scale_factor").as_double();
    config.angular_scale_factor = this->get_parameter("angular_scale_factor").as_double();
    config.distance_deadband = this->get_parameter("distance_deadband").as_double();
    config.lateral_deadband = this->get_parameter("lateral_deadband").as_double();
    config.angle_deadband = this->get_parameter("angle_deadband").as_double();
    config.rotate_only_angle = this->get_parameter("rotate_only_angle").as_double();
    config.min_forward_speed = this->get_parameter("min_forward_speed").as_double();
    config.rectangle_width = this->get_parameter("rectangle_width").as_double();
    config.robot_frame_front = this->get_parameter("robot_frame_front").as_double();
    config.robot_frame_back = this->get_parameter("robot_frame_back").as_double();
    config.robot_frame_left = this->get_parameter("robot_frame_left").as_double();
    config.robot_frame_right = this->get_parameter("robot_frame_right").as_double();
    config.local_map_enabled = this->get_parameter("local_map_enabled").as_bool();
    config.local_map_size_x = this->get_parameter("local_map_size_x").as_double();
    config.local_map_size_y = this->get_parameter("local_map_size_y").as_double();
    config.local_map_resolution = this->get_parameter("local_map_resolution").as_double();
    config.local_map_lifetime_sec = this->get_parameter("local_map_lifetime_sec").as_double();
    config.local_map_max_points =
      static_cast<int>(this->get_parameter("local_map_max_points").as_int());
    config.local_map_ray_clear_enabled =
      this->get_parameter("local_map_ray_clear_enabled").as_bool();
    config.local_map_ray_clear_radius =
      this->get_parameter("local_map_ray_clear_radius").as_double();
    config.local_map_ray_clear_hit_margin =
      this->get_parameter("local_map_ray_clear_hit_margin").as_double();

    return config;
  }

  // 处理相机转出的 scan，优先写入局部地图后再计算速度。
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    if (!local_map_enabled_ || !scan_msg) {
      controller_->processScan(scan_msg);
      return;
    }

    try {
      const std::string scan_frame = resolveScanFrame(scan_msg);
      const auto scan_to_map = tf_buffer_->lookupTransform(
        local_map_frame_,
        scan_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(0.05));
      const auto map_to_target = tf_buffer_->lookupTransform(
        target_frame_,
        local_map_frame_,
        tf2::TimePointZero,
        tf2::durationFromSec(0.05));

      local_map_->updateFromScan(scan_msg, scan_to_map, map_to_target);
      controller_->processObstaclePoints(local_map_->getObstaclePoints(map_to_target));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Local map TF failed, fallback to current scan only: %s",
        ex.what());
      controller_->processScan(scan_msg);
    }
  }

  // 解析 scan 的坐标系，缺省时使用控制坐标系兜底。
  std::string resolveScanFrame(const sensor_msgs::msg::LaserScan::SharedPtr & scan_msg) const
  {
    if (scan_msg && !scan_msg->header.frame_id.empty()) {
      return scan_msg->header.frame_id;
    }
    return target_frame_;
  }

  // 将 UWB 自定义消息转换到配置的平面跟随坐标系。
  void uwbTargetCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (!msg || !std::isfinite(msg->point.x) || !std::isfinite(msg->point.y)) {
      RCLCPP_WARN(this->get_logger(), "Ignored invalid UWB target");
      return;
    }
    geometry_msgs::msg::PointStamped uwb_target;
    uwb_target.header = msg->header;
    uwb_target.point.x = msg->point.x;
    uwb_target.point.y = msg->point.y;
    uwb_target.point.z = 0.0;
    if (uwb_target.header.frame_id.empty()) {
      uwb_target.header.frame_id = uwb_input_frame_;
    }

    try {
      const auto base_target = tf_buffer_->transform(
        uwb_target,
        target_frame_,
        tf2::durationFromSec(0.05));
      shared_state_.setTarget(base_target.point.x, base_target.point.y);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Failed to transform UWB target from '%s' to '%s': %s",
        uwb_target.header.frame_id.c_str(),
        target_frame_.c_str(),
        ex.what());
    }
  }
};

// 启动 ROS2 节点主循环。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotNexusNode>());
  rclcpp::shutdown();
  return 0;
}
