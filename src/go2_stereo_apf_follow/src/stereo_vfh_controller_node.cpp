#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "go2_stereo_apf_follow/vfh_core.hpp"

namespace
{

// 将 yaw 转成 ROS 四元数。
geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

// 使用给定 TF 变换三维点到目标坐标系。
go2_stereo_apf_follow::Point3D transform_point(
  const go2_stereo_apf_follow::Point3D & point,
  const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & rotation = transform.transform.rotation;
  tf2::Quaternion q(rotation.x, rotation.y, rotation.z, rotation.w);
  tf2::Matrix3x3 matrix(q);
  tf2::Vector3 v(point.x, point.y, point.z);
  tf2::Vector3 out = matrix * v;
  out += tf2::Vector3(
    transform.transform.translation.x,
    transform.transform.translation.y,
    transform.transform.translation.z);
  return go2_stereo_apf_follow::Point3D{out.x(), out.y(), out.z()};
}

// 使用给定 TF 变换二维点到目标坐标系。
go2_stereo_apf_follow::Point2D transform_point(
  const go2_stereo_apf_follow::Point2D & point,
  const geometry_msgs::msg::TransformStamped & transform)
{
  const auto out = transform_point(go2_stereo_apf_follow::Point3D{point.x, point.y, 0.0}, transform);
  return go2_stereo_apf_follow::Point2D{out.x, out.y};
}

// 将内部速度结构转换成 ROS Twist。
geometry_msgs::msg::Twist to_twist(const go2_stereo_apf_follow::TwistCommand & cmd)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = cmd.vx;
  msg.linear.y = cmd.vy;
  msg.angular.z = cmd.wz;
  return msg;
}

// 构造 Marker 使用的 RGBA 颜色。
std_msgs::msg::ColorRGBA make_color(double r, double g, double b, double a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = static_cast<float>(r);
  color.g = static_cast<float>(g);
  color.b = static_cast<float>(b);
  color.a = static_cast<float>(a);
  return color;
}

// 创建一个二维点 Marker 坐标。
geometry_msgs::msg::Point make_marker_point(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

// 给 LINE_LIST Marker 增加一条带颜色的线段。
void add_colored_line(
  visualization_msgs::msg::Marker & marker,
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & end,
  const std_msgs::msg::ColorRGBA & color)
{
  marker.points.push_back(start);
  marker.points.push_back(end);
  marker.colors.push_back(color);
  marker.colors.push_back(color);
}

// 根据 VFH 模式生成状态字符串前缀。
std::string mode_prefix(go2_stereo_apf_follow::VfhMode mode)
{
  if (mode == go2_stereo_apf_follow::VfhMode::BYPASS_LEFT) {
    return "bypass_left";
  }
  if (mode == go2_stereo_apf_follow::VfhMode::BYPASS_RIGHT) {
    return "bypass_right";
  }
  return "follow";
}

}  // namespace

class StereoVfhControllerNode : public rclcpp::Node
{
public:
  // 初始化 VFH 控制节点的参数、订阅和发布接口。
  StereoVfhControllerNode()
  : Node("stereo_vfh_controller_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic", "/local_grid_obstacle");
    seed_target_topic_ = declare_parameter<std::string>("seed_target_topic", "/stereo_vfh/seed_target");
    seed_valid_topic_ = declare_parameter<std::string>("seed_valid_topic", "/stereo_vfh/seed_valid");
    manual_target_topic_ = declare_parameter<std::string>("manual_target_topic", "/stereo_vfh/manual_target");
    enabled_topic_ = declare_parameter<std::string>("enabled_topic", "/stereo_vfh/enabled");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    target_topic_ = declare_parameter<std::string>("target_topic", "/stereo_vfh/target");
    follow_goal_topic_ = declare_parameter<std::string>("follow_goal_topic", "/stereo_vfh/follow_goal");
    status_topic_ = declare_parameter<std::string>("status_topic", "/stereo_vfh/status");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/stereo_vfh/markers");
    enabled_ = declare_parameter<bool>("enabled_default", true);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
    pointcloud_timeout_sec_ = declare_parameter<double>("pointcloud_timeout_sec", 0.5);
    seed_timeout_sec_ = declare_parameter<double>("seed_timeout_sec", 3.0);
    target_hold_sec_ = declare_parameter<double>("target_hold_sec", 0.8);
    seed_reset_distance_ = declare_parameter<double>("seed_reset_distance", 1.0);
    max_points_per_cloud_ = declare_parameter<int>("max_points_per_cloud", 60000);

    filter_config_.x_min = declare_parameter<double>("obstacle_x_min", -4.0);
    filter_config_.x_max = declare_parameter<double>("obstacle_x_max", 4.0);
    filter_config_.y_abs = declare_parameter<double>("obstacle_y_abs", 4.0);
    filter_config_.z_min = declare_parameter<double>("obstacle_z_min", 0.05);
    filter_config_.z_max = declare_parameter<double>("obstacle_z_max", 1.2);
    filter_config_.robot_frame_front = declare_parameter<double>("robot_frame_front", 0.15);
    filter_config_.robot_frame_back = declare_parameter<double>("robot_frame_back", 0.35);
    filter_config_.robot_frame_left = declare_parameter<double>("robot_frame_left", 0.15);
    filter_config_.robot_frame_right = declare_parameter<double>("robot_frame_right", 0.15);

    tracking_config_.target_radius = declare_parameter<double>("target_radius", 0.30);
    tracking_config_.min_points_in_target = declare_parameter<int>("min_points_in_target", 1);
    tracking_config_.smoothing_alpha = declare_parameter<double>("smoothing_alpha", 0.35);

    follow_distance_ = declare_parameter<double>("follow_distance", 0.40);
    vfh_config_.sector_angle_deg = declare_parameter<double>("sector_angle_deg", 5.0);
    vfh_config_.range_min = declare_parameter<double>("vfh_range_min", 0.10);
    vfh_config_.range_max = declare_parameter<double>("vfh_range_max", 4.0);
    vfh_config_.robot_radius = declare_parameter<double>("robot_radius", 0.35);
    vfh_config_.safety_margin = declare_parameter<double>("safety_margin", 0.20);
    vfh_config_.hard_stop_distance = declare_parameter<double>("hard_stop_distance", 0.35);
    vfh_config_.slowdown_distance = declare_parameter<double>("slowdown_distance", 0.80);
    vfh_config_.corridor_width = declare_parameter<double>("corridor_width", 0.75);
    vfh_config_.target_mask_radius = declare_parameter<double>("target_mask_radius", 0.35);
    vfh_config_.side_switch_hold_sec = declare_parameter<double>("side_switch_hold_sec", 1.2);
    vfh_config_.corridor_clear_hold_sec = declare_parameter<double>("corridor_clear_hold_sec", 0.8);
    vfh_config_.heading_limit_deg = declare_parameter<double>("heading_limit_deg", 90.0);
    vfh_config_.target_heading_weight = declare_parameter<double>("target_heading_weight", 1.0);
    vfh_config_.last_heading_weight = declare_parameter<double>("last_heading_weight", 0.25);
    vfh_config_.wrong_side_penalty = declare_parameter<double>("wrong_side_penalty", 3.0);
    vfh_config_.valley_width_weight = declare_parameter<double>("valley_width_weight", 0.04);
    vfh_config_.max_vx = declare_parameter<double>("max_vx", 0.45);
    vfh_config_.max_vy = declare_parameter<double>("max_vy", 0.35);
    vfh_config_.max_wz = declare_parameter<double>("max_vyaw", 0.8);
    vfh_config_.angular_scale = declare_parameter<double>("angular_scale", 1.0);
    vfh_config_.bypass_heading_blend = declare_parameter<double>("bypass_heading_blend", 0.55);
    vfh_config_.command_filter_alpha = declare_parameter<double>("command_filter_alpha", 0.45);
    vfh_config_.max_delta_vx_per_sec = declare_parameter<double>("max_delta_vx_per_sec", 0.55);
    vfh_config_.max_delta_vy_per_sec = declare_parameter<double>("max_delta_vy_per_sec", 0.55);
    vfh_config_.max_delta_wz_per_sec = declare_parameter<double>("max_delta_wz_per_sec", 1.0);
    vfh_config_.min_move_speed = declare_parameter<double>("min_move_speed", 0.08);
    vfh_config_.linear_scale = declare_parameter<double>("linear_scale", 0.45);

    publish_markers_ = declare_parameter<bool>("publish_markers", true);
    marker_radius_ = declare_parameter<double>("marker_radius", 1.2);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_,
      5,
      std::bind(&StereoVfhControllerNode::cloud_callback, this, std::placeholders::_1));
    seed_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      seed_target_topic_,
      10,
      std::bind(&StereoVfhControllerNode::seed_callback, this, std::placeholders::_1));
    seed_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      seed_valid_topic_,
      10,
      std::bind(&StereoVfhControllerNode::seed_valid_callback, this, std::placeholders::_1));
    manual_target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      manual_target_topic_,
      10,
      std::bind(&StereoVfhControllerNode::manual_target_callback, this, std::placeholders::_1));
    enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
      enabled_topic_,
      10,
      std::bind(&StereoVfhControllerNode::enabled_callback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(target_topic_, 10);
    follow_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(follow_goal_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 2);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&StereoVfhControllerNode::tick, this));

    RCLCPP_INFO(get_logger(), "stereo_vfh_controller_node started");
  }

private:
  // 接收点云，转换到 base_frame 并做基础 ROI 过滤。
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped tf;
    bool have_tf = false;
    if (!msg->header.frame_id.empty() && msg->header.frame_id != base_frame_) {
      try {
        tf = tf_buffer_.lookupTransform(base_frame_, msg->header.frame_id, tf2::TimePointZero);
        have_tf = true;
      } catch (const std::exception & exc) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "PointCloud TF failed from %s to %s: %s",
          msg->header.frame_id.c_str(),
          base_frame_.c_str(),
          exc.what());
        return;
      }
    }

    std::vector<go2_stereo_apf_follow::Point3D> points;
    const auto cloud_size =
      static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height);
    const auto max_points = static_cast<std::size_t>(std::max(0, max_points_per_cloud_));
    points.reserve(std::min(cloud_size, max_points));
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    int seen = 0;
    for (; iter_x != iter_x.end() && seen < max_points_per_cloud_; ++iter_x, ++iter_y, ++iter_z, ++seen) {
      go2_stereo_apf_follow::Point3D point{
        static_cast<double>(*iter_x),
        static_cast<double>(*iter_y),
        static_cast<double>(*iter_z)};
      if (have_tf) {
        point = transform_point(point, tf);
      }
      if (go2_stereo_apf_follow::point_passes_filter(point, filter_config_)) {
        points.push_back(point);
      }
    }

    latest_points_ = std::move(points);
    latest_cloud_time_ = now();
    have_cloud_ = true;
  }

  // 接收 UWB seed 目标。
  void seed_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    auto target = pose_to_base(*msg);
    if (!target.has_value()) {
      return;
    }
    seed_target_ = target.value();
    seed_time_ = now();
    have_seed_ = true;
  }

  // 接收 UWB seed 有效状态。
  void seed_valid_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    seed_valid_ = msg->data;
    seed_valid_time_ = now();
  }

  // 接收手动目标，便于离线调试绕障。
  void manual_target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    auto target = pose_to_base(*msg);
    if (!target.has_value()) {
      return;
    }
    target_ = target.value();
    target_time_ = now();
    have_target_ = true;
    target_source_ = "manual";
    publish_status("follow: manual target");
  }

  // 接收外部使能开关。
  void enabled_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    enabled_ = msg->data;
  }

  // 将 PoseStamped 目标转换到 base_frame。
  std::optional<go2_stereo_apf_follow::Point2D> pose_to_base(
    const geometry_msgs::msg::PoseStamped & pose)
  {
    go2_stereo_apf_follow::Point2D point{pose.pose.position.x, pose.pose.position.y};
    const std::string frame = pose.header.frame_id.empty() ? base_frame_ : pose.header.frame_id;
    if (frame == base_frame_) {
      return point;
    }
    try {
      const auto tf = tf_buffer_.lookupTransform(base_frame_, frame, tf2::TimePointZero);
      return transform_point(point, tf);
    } catch (const std::exception & exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Target TF failed from %s to %s: %s",
        frame.c_str(),
        base_frame_.c_str(),
        exc.what());
      return std::nullopt;
    }
  }

  // 判断某个时间戳是否仍在有效期内。
  bool age_ok(const rclcpp::Time & stamp, double timeout_sec) const
  {
    if (stamp.nanoseconds() == 0) {
      return false;
    }
    return (now() - stamp).seconds() <= timeout_sec;
  }

  // 判断 UWB seed 是否新鲜且有效。
  bool seed_is_fresh() const
  {
    return have_seed_ && seed_valid_ && age_ok(seed_time_, seed_timeout_sec_) &&
           age_ok(seed_valid_time_, seed_timeout_sec_);
  }

  // 按需用 UWB seed 刷新当前目标。
  bool refresh_target_from_seed(bool force)
  {
    if (!seed_is_fresh()) {
      return false;
    }
    if (!force && have_target_) {
      const double delta = std::hypot(seed_target_.x - target_.x, seed_target_.y - target_.y);
      if (delta <= seed_reset_distance_) {
        return false;
      }
    }
    target_ = seed_target_;
    target_time_ = now();
    have_target_ = true;
    target_source_ = "seed";
    return true;
  }

  // 主控制循环，负责安全检查、目标更新和 VFH 控制输出。
  void tick()
  {
    if (!enabled_) {
      publish_marker_delete();
      publish_zero("stop: disabled");
      return;
    }
    if (!have_cloud_ || !age_ok(latest_cloud_time_, pointcloud_timeout_sec_)) {
      publish_marker_delete();
      publish_zero("stop: obstacle cloud stale");
      return;
    }

    if (!have_target_) {
      if (!refresh_target_from_seed(true)) {
        publish_marker_delete();
        publish_zero("stop: target unavailable");
        return;
      }
    } else {
      refresh_target_from_seed(false);
    }

    if (!update_target_from_cloud_or_seed()) {
      return;
    }

    const auto result = go2_stereo_apf_follow::compute_vfh_command(
      latest_points_,
      target_,
      follow_distance_,
      now().seconds(),
      vfh_config_,
      vfh_state_);
    cmd_pub_->publish(to_twist(result.command));
    publish_target();
    publish_follow_goal(result.follow_goal, result.selected_heading);
    publish_markers(result);
    publish_status(status_from_result(result));
  }

  // 根据目标附近点云质心或 UWB seed 维持当前目标。
  bool update_target_from_cloud_or_seed()
  {
    auto centroid = go2_stereo_apf_follow::compute_target_centroid(
      latest_points_,
      target_,
      tracking_config_);
    if (centroid.has_value()) {
      target_ = go2_stereo_apf_follow::smooth_target(
        target_,
        centroid.value(),
        tracking_config_.smoothing_alpha);
      target_time_ = now();
      target_source_ = "cloud";
      return true;
    }

    if ((now() - target_time_).seconds() > target_hold_sec_) {
      if (!refresh_target_from_seed(true)) {
        have_target_ = false;
        publish_marker_delete();
        publish_zero("stop: target lost");
        return false;
      }
    }
    return true;
  }

  // 发布当前 UWB 跟随目标。
  void publish_target()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = base_frame_;
    pose.pose.position.x = target_.x;
    pose.pose.position.y = target_.y;
    pose.pose.orientation = yaw_to_quaternion(std::atan2(target_.y, target_.x));
    target_pub_->publish(pose);
  }

  // 发布扣除跟随距离后的 VFH 实际跟随目标。
  void publish_follow_goal(const go2_stereo_apf_follow::Point2D & follow_goal, double heading)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = base_frame_;
    pose.pose.position.x = follow_goal.x;
    pose.pose.position.y = follow_goal.y;
    pose.pose.orientation = yaw_to_quaternion(heading);
    follow_goal_pub_->publish(pose);
  }

  // 删除 RViz 中的旧 VFH Marker。
  void publish_marker_delete()
  {
    if (!publish_markers_) {
      return;
    }
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = base_frame_;
    marker.ns = "stereo_vfh";
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(marker);
    marker_pub_->publish(markers);
  }

  // 发布 VFH 扇区、目标方向和当前选择方向。
  void publish_markers(const go2_stereo_apf_follow::VfhResult & result)
  {
    if (!publish_markers_) {
      return;
    }

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.stamp = now();
    clear_marker.header.frame_id = base_frame_;
    clear_marker.ns = "stereo_vfh";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);

    markers.markers.push_back(make_sector_marker(result));
    markers.markers.push_back(make_heading_marker(1, "target_heading", result.target_heading, 0.9, make_color(0.0, 0.45, 1.0, 0.9)));
    markers.markers.push_back(make_heading_marker(2, "selected_heading", result.selected_heading, 1.1, make_color(0.0, 1.0, 0.25, 0.95)));
    marker_pub_->publish(markers);
  }

  // 生成所有 VFH 扇区的可视化线段。
  visualization_msgs::msg::Marker make_sector_marker(
    const go2_stereo_apf_follow::VfhResult & result)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = base_frame_;
    marker.ns = "stereo_vfh_sectors";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.018;
    marker.color.a = 1.0;

    const auto origin = make_marker_point(0.0, 0.0, 0.08);
    const int count = static_cast<int>(result.histogram.blocked.size());
    for (int i = 0; i < count; ++i) {
      const double heading = go2_stereo_apf_follow::sector_center_heading(result.histogram, i);
      const double radius = std::max(0.2, marker_radius_);
      const auto end = make_marker_point(radius * std::cos(heading), radius * std::sin(heading), 0.08);
      const auto color = result.histogram.blocked[i] ?
        make_color(1.0, 0.1, 0.05, 0.75) :
        make_color(0.3, 0.8, 0.3, 0.35);
      add_colored_line(marker, origin, end, color);
    }
    return marker;
  }

  // 生成一个 heading 箭头 Marker。
  visualization_msgs::msg::Marker make_heading_marker(
    int id,
    const std::string & ns,
    double heading,
    double length,
    const std_msgs::msg::ColorRGBA & color)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = base_frame_;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.points.push_back(make_marker_point(0.0, 0.0, 0.14));
    marker.points.push_back(make_marker_point(length * std::cos(heading), length * std::sin(heading), 0.14));
    marker.scale.x = 0.035;
    marker.scale.y = 0.075;
    marker.scale.z = 0.075;
    marker.color = color;
    return marker;
  }

  // 根据 VFH 结果生成外部可读状态。
  std::string status_from_result(const go2_stereo_apf_follow::VfhResult & result) const
  {
    if (result.hard_stop) {
      return "stop: obstacle emergency";
    }
    const std::string prefix = mode_prefix(result.mode);
    if (result.has_nearest && result.nearest_dist < vfh_config_.slowdown_distance) {
      return "slow: " + prefix + ", obstacle=" + std::to_string(result.nearest_dist) + "m";
    }
    return prefix + ": target=" + target_source_;
  }

  // 发布零速度并记录状态。
  void publish_zero(const std::string & status)
  {
    vfh_state_.last_command = go2_stereo_apf_follow::TwistCommand{};
    vfh_state_.last_update_time = now().seconds();
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    publish_status(status);
  }

  // 发布去重后的状态文本。
  void publish_status(const std::string & status)
  {
    if (status == last_status_) {
      return;
    }
    last_status_ = status;
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "%s", status.c_str());
  }

  std::string base_frame_;
  std::string pointcloud_topic_;
  std::string seed_target_topic_;
  std::string seed_valid_topic_;
  std::string manual_target_topic_;
  std::string enabled_topic_;
  std::string cmd_vel_topic_;
  std::string target_topic_;
  std::string follow_goal_topic_;
  std::string status_topic_;
  std::string marker_topic_;
  bool enabled_{true};
  double publish_rate_hz_{30.0};
  double pointcloud_timeout_sec_{0.5};
  double seed_timeout_sec_{3.0};
  double target_hold_sec_{0.8};
  double seed_reset_distance_{1.0};
  int max_points_per_cloud_{60000};
  double follow_distance_{0.40};
  bool publish_markers_{true};
  double marker_radius_{1.2};

  go2_stereo_apf_follow::PointFilterConfig filter_config_;
  go2_stereo_apf_follow::TargetTrackingConfig tracking_config_;
  go2_stereo_apf_follow::VfhConfig vfh_config_;
  go2_stereo_apf_follow::VfhState vfh_state_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr seed_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr seed_valid_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr manual_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enabled_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr follow_goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<go2_stereo_apf_follow::Point3D> latest_points_;
  rclcpp::Time latest_cloud_time_{0, 0, RCL_ROS_TIME};
  bool have_cloud_{false};
  go2_stereo_apf_follow::Point2D seed_target_;
  rclcpp::Time seed_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time seed_valid_time_{0, 0, RCL_ROS_TIME};
  bool have_seed_{false};
  bool seed_valid_{false};
  go2_stereo_apf_follow::Point2D target_;
  rclcpp::Time target_time_{0, 0, RCL_ROS_TIME};
  bool have_target_{false};
  std::string target_source_{"none"};
  std::string last_status_;
};

// 启动 ROS2 VFH 控制节点。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StereoVfhControllerNode>());
  rclcpp::shutdown();
  return 0;
}
