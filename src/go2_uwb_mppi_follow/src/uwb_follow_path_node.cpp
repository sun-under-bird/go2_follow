#include "go2_uwb_mppi_follow/uwb_follow_path_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <string>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace go2_uwb_mppi_follow
{
namespace
{

// 判断浮点数是否为有效有限值。
bool finiteValue(const double value)
{
  return std::isfinite(value);
}

// 判断三维点的各坐标是否为有效有限值。
bool finitePoint(const geometry_msgs::msg::Point & point)
{
  return finiteValue(point.x) && finiteValue(point.y) && finiteValue(point.z);
}

// 根据二维坐标构造 z 为 0 的 ROS 点。
geometry_msgs::msg::Point makePoint(const double x, const double y)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = 0.0;
  return point;
}

// 将平面 yaw 角转换为四元数姿态。
geometry_msgs::msg::Quaternion yawToQuaternion(const double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quaternion);
}

}  // namespace

// 构造 UWB 路径生成节点，并初始化 TF、话题和定时器。
UwbFollowPathNode::UwbFollowPathNode()
: Node("uwb_follow_path_node"),
  latest_target_stamp_(0, 0, get_clock()->get_clock_type())
{
  declareParameters();
  loadParameters();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  uwb_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
    config_.uwb_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&UwbFollowPathNode::uwbCallback, this, std::placeholders::_1));

  follow_path_pub_ = create_publisher<nav_msgs::msg::Path>(config_.follow_path_topic, 10);
  target_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(config_.target_topic, 10);
  follow_goal_pub_ =
    create_publisher<geometry_msgs::msg::PointStamped>(config_.follow_goal_topic, 10);
  target_valid_pub_ = create_publisher<std_msgs::msg::Bool>(config_.target_valid_topic, 10);
  status_pub_ = create_publisher<std_msgs::msg::String>(config_.status_topic, 10);
  if (config_.rotate_to_target_within_follow_distance && !config_.cmd_vel_topic.empty()) {
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(config_.cmd_vel_topic, 10);
  }

  const double publish_rate_hz = std::max(1.0, config_.publish_rate_hz);
  const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_hz));
  timer_ = create_wall_timer(timer_period, std::bind(&UwbFollowPathNode::timerCallback, this));

  RCLCPP_WARN(get_logger(), "uwb_follow_path_node started");
}

// 声明 UWB 目标、规划器、MPPI 跟踪和停车相关参数。
void UwbFollowPathNode::declareParameters()
{
  declare_parameter<std::string>("uwb_topic", "/uwb/target_point");
  declare_parameter<std::string>("odom_frame", "odom");
  declare_parameter<std::string>("base_frame", "base_footprint");
  declare_parameter<std::string>("follow_path_topic", "/uwb_follow/path");
  declare_parameter<std::string>("target_topic", "/uwb_follow/target_raw");
  declare_parameter<std::string>("follow_goal_topic", "/uwb_follow/follow_goal");
  declare_parameter<std::string>("target_valid_topic", "/follow/target_valid");
  declare_parameter<std::string>("status_topic", "/follow/uwb_path_status");
  declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  declare_parameter<double>("follow_distance_m", 1.5);
  declare_parameter<double>("target_timeout_sec", 1.0);
  declare_parameter<double>("publish_rate_hz", 5.0);
  declare_parameter<double>("transform_timeout_sec", 0.2);
  declare_parameter<double>("direct_path_resolution_m", 0.20);
  declare_parameter<double>("hold_rotate_yaw_gain", 1.0);
  declare_parameter<double>("hold_rotate_max_angular_vel", 0.8);
  declare_parameter<double>("hold_rotate_yaw_deadband_rad", 0.05);
  declare_parameter<int>("min_path_poses", 2);
  declare_parameter<bool>("use_latest_tf", true);
  declare_parameter<bool>("rotate_to_target_within_follow_distance", true);
}

// 读取 ROS 参数并写入本节点的运行配置。
void UwbFollowPathNode::loadParameters()
{
  config_.uwb_topic = get_parameter("uwb_topic").as_string();
  config_.odom_frame = get_parameter("odom_frame").as_string();
  config_.base_frame = get_parameter("base_frame").as_string();
  config_.follow_path_topic = get_parameter("follow_path_topic").as_string();
  config_.target_topic = get_parameter("target_topic").as_string();
  config_.follow_goal_topic = get_parameter("follow_goal_topic").as_string();
  config_.target_valid_topic = get_parameter("target_valid_topic").as_string();
  config_.status_topic = get_parameter("status_topic").as_string();
  config_.cmd_vel_topic = get_parameter("cmd_vel_topic").as_string();
  config_.follow_distance_m = get_parameter("follow_distance_m").as_double();
  config_.target_timeout_sec = get_parameter("target_timeout_sec").as_double();
  config_.publish_rate_hz = get_parameter("publish_rate_hz").as_double();
  config_.transform_timeout_sec = get_parameter("transform_timeout_sec").as_double();
  config_.direct_path_resolution_m =
    std::max(0.05, get_parameter("direct_path_resolution_m").as_double());
  config_.hold_rotate_yaw_gain = get_parameter("hold_rotate_yaw_gain").as_double();
  config_.hold_rotate_max_angular_vel =
    std::max(0.0, get_parameter("hold_rotate_max_angular_vel").as_double());
  config_.hold_rotate_yaw_deadband_rad =
    std::max(0.0, get_parameter("hold_rotate_yaw_deadband_rad").as_double());
  config_.min_path_poses = static_cast<int>(get_parameter("min_path_poses").as_int());
  config_.use_latest_tf = get_parameter("use_latest_tf").as_bool();
  config_.rotate_to_target_within_follow_distance =
    get_parameter("rotate_to_target_within_follow_distance").as_bool();
}

// 接收 UWB 目标点，转换到机器人坐标系后直接保存和发布原始目标。
void UwbFollowPathNode::uwbCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  const rclcpp::Time stamp = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ?
    now() : rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());

  const auto target_base = targetToBase(*msg);
  if (!target_base) {
    publishStatus("reject: UWB target TF failed");
    return;
  }

  latest_target_base_ = target_base->point;
  latest_target_stamp_ = stamp;
  have_target_ = true;
  publishTargetRaw(latest_target_base_, stamp);

  const double range = std::hypot(latest_target_base_.x, latest_target_base_.y);
  if (range < config_.follow_distance_m) {
    holdAndRotateToTarget(latest_target_base_, stamp);
    return;
  }

  publishStatus("tracking: raw UWB target");
}

// 将输入目标点转换到 base_frame 坐标系。
std::optional<geometry_msgs::msg::PointStamped> UwbFollowPathNode::targetToBase(
  const geometry_msgs::msg::PointStamped & target)
{
  if (!finitePoint(target.point)) {
    return std::nullopt;
  }

  const std::string source_frame =
    target.header.frame_id.empty() ? config_.base_frame : target.header.frame_id;
  if (source_frame == config_.base_frame) {
    geometry_msgs::msg::PointStamped target_base = target;
    target_base.header.frame_id = config_.base_frame;
    target_base.point.z = 0.0;
    return target_base;
  }

  geometry_msgs::msg::PointStamped source = target;
  source.header.frame_id = source_frame;
  if (config_.use_latest_tf) {
    source.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  try {
    auto transformed = tf_buffer_->transform(
      source,
      config_.base_frame,
      tf2::durationFromSec(config_.transform_timeout_sec));
    transformed.point.z = 0.0;
    return transformed;
  } catch (const tf2::TransformException & exc) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "UWB target transform to base failed: %s",
      exc.what());
    return std::nullopt;
  }
}

// 把真实 UWB 人员点向机器人方向回退 follow_distance，生成 MPPI 追踪站位点。
geometry_msgs::msg::Point UwbFollowPathNode::makeFollowGoalPoint(
  const geometry_msgs::msg::Point & target) const
{
  const double range = std::hypot(target.x, target.y);
  if (range <= 1e-6) {
    return makePoint(0.0, 0.0);
  }

  const double travel = std::max(0.0, range - config_.follow_distance_m);
  return makePoint(travel * target.x / range, travel * target.y / range);
}

// 周期性使用最近一次原始目标，近距离原地转向，否则生成局部路径交给 MPPI。
void UwbFollowPathNode::timerCallback()
{
  const rclcpp::Time now_stamp = now();
  const bool target_valid = have_target_ &&
    (now_stamp - latest_target_stamp_).seconds() <= config_.target_timeout_sec;
  publishTargetValid(target_valid);

  if (!target_valid) {
    const bool timed_out = have_target_;
    have_target_ = false;
    stopFollowing(
      timed_out ? "stop: UWB target timeout" : "stop: waiting for UWB target", now_stamp);
    return;
  }

  const double range = std::hypot(latest_target_base_.x, latest_target_base_.y);
  if (range < config_.follow_distance_m) {
    holdAndRotateToTarget(latest_target_base_, now_stamp);
    return;
  }

  const double target_yaw = std::atan2(
    latest_target_base_.y,
    latest_target_base_.x);
  const geometry_msgs::msg::Point goal_base = makeFollowGoalPoint(latest_target_base_);
  publishFollowGoal(goal_base, now_stamp);

  const auto path = buildDirectFollowPath(goal_base, target_yaw, now_stamp);
  if (!path) {
    stopFollowing("stop: direct follow path TF failed", now_stamp);
    return;
  }

  // 持续发布用于保活；行为树执行器只在终点变化超过阈值时更新 FollowPath goal。
  follow_path_pub_->publish(*path);
  publishStatus("tracking: direct MPPI path published to recovery tree");
}

// 生成从机器人当前位置到 UWB 目标点的局部直线路径，供 MPPI 自行避障跟踪。
std::optional<nav_msgs::msg::Path> UwbFollowPathNode::buildDirectFollowPath(
  const geometry_msgs::msg::Point & goal_base,
  const double goal_yaw_base,
  const rclcpp::Time & stamp)
{
  const auto start_pose = transformPoseToOdom(
    makeBasePose(0.0, 0.0, goal_yaw_base, config_.base_frame, stamp));
  const auto goal_pose = transformPoseToOdom(
    makeBasePose(goal_base.x, goal_base.y, goal_yaw_base, config_.base_frame, stamp));

  if (!start_pose || !goal_pose) {
    return std::nullopt;
  }

  const double dx = goal_pose->pose.position.x - start_pose->pose.position.x;
  const double dy = goal_pose->pose.position.y - start_pose->pose.position.y;
  const double distance = std::hypot(dx, dy);
  const int segment_count = std::max(
    config_.min_path_poses - 1,
    static_cast<int>(std::ceil(distance / config_.direct_path_resolution_m)));
  if (segment_count < 1) {
    return std::nullopt;
  }

  nav_msgs::msg::Path path;
  path.header.frame_id = config_.odom_frame;
  path.header.stamp = stamp;

  const double path_yaw = std::atan2(dy, dx);
  const auto orientation = yawToQuaternion(path_yaw);
  for (int i = 0; i <= segment_count; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(segment_count);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = start_pose->pose.position.x + dx * ratio;
    pose.pose.position.y = start_pose->pose.position.y + dy * ratio;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = orientation;
    path.poses.push_back(pose);
  }

  return path;
}

// 将 base_frame 下的目标姿态转换到 odom，供局部直线路径使用。
std::optional<geometry_msgs::msg::PoseStamped> UwbFollowPathNode::transformPoseToOdom(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (pose.header.frame_id == config_.odom_frame) {
    return pose;
  }

  geometry_msgs::msg::PoseStamped source = pose;
  if (config_.use_latest_tf) {
    source.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  try {
    auto transformed = tf_buffer_->transform(
      source,
      config_.odom_frame,
      tf2::durationFromSec(config_.transform_timeout_sec));
    transformed.header.stamp = pose.header.stamp;
    return transformed;
  } catch (const tf2::TransformException & exc) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "follow path transform to odom failed: %s",
      exc.what());
    return std::nullopt;
  }
}

// 停止跟随：只发布空路径让行为树撤销动作，避免与控制器并发发布零速度。
void UwbFollowPathNode::stopFollowing(const std::string & status, const rclcpp::Time & stamp)
{
  publishEmptyPath(stamp);
  if (cmd_vel_pub_) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
  }
  publishStatus(status);
}

// 近距离保持：不使用 MPPI，只发布原地转向速度让机身朝向 UWB 目标。
void UwbFollowPathNode::holdAndRotateToTarget(
  const geometry_msgs::msg::Point & target,
  const rclcpp::Time & stamp)
{
  publishFollowGoal(makeFollowGoalPoint(target), stamp);
  publishEmptyPath(stamp);

  if (config_.rotate_to_target_within_follow_distance) {
    publishHoldRotateVelocity(target);
  }

  publishStatus("hold: rotate to UWB target");
}

// 发布当前用于规划和人员点云清除的原始 UWB 目标点。
void UwbFollowPathNode::publishTargetRaw(
  const geometry_msgs::msg::Point & target,
  const rclcpp::Time & stamp)
{
  geometry_msgs::msg::PointStamped msg;
  msg.header.frame_id = config_.base_frame;
  msg.header.stamp = stamp;
  msg.point = target;
  target_pub_->publish(msg);
}

// 发布保持跟随距离后的 MPPI 站位目标，便于 RViz 和调试确认。
void UwbFollowPathNode::publishFollowGoal(
  const geometry_msgs::msg::Point & goal,
  const rclcpp::Time & stamp)
{
  geometry_msgs::msg::PointStamped msg;
  msg.header.frame_id = config_.base_frame;
  msg.header.stamp = stamp;
  msg.point = goal;
  follow_goal_pub_->publish(msg);
}

// 发布当前 UWB 目标是否有效。
void UwbFollowPathNode::publishTargetValid(const bool valid)
{
  std_msgs::msg::Bool msg;
  msg.data = valid;
  target_valid_pub_->publish(msg);
}

// 发布空路径，用于通知下游当前没有可跟踪路径。
void UwbFollowPathNode::publishEmptyPath(const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = config_.odom_frame;
  path.header.stamp = stamp;
  follow_path_pub_->publish(path);
}

// 发布近距离原地转向速度，线速度固定为 0。
void UwbFollowPathNode::publishHoldRotateVelocity(const geometry_msgs::msg::Point & target)
{
  if (!cmd_vel_pub_) {
    return;
  }

  geometry_msgs::msg::Twist twist;
  if (!finitePoint(target)) {
    cmd_vel_pub_->publish(twist);
    return;
  }

  const double yaw_error = std::atan2(target.y, target.x);
  if (
    std::abs(yaw_error) > config_.hold_rotate_yaw_deadband_rad &&
    config_.hold_rotate_max_angular_vel > 0.0)
  {
    // 角速度只负责把 base_footprint 的 x 轴转向 UWB 目标方向，不产生前进速度。
    const double angular_z = config_.hold_rotate_yaw_gain * yaw_error;
    twist.angular.z = std::clamp(
      angular_z,
      -config_.hold_rotate_max_angular_vel,
      config_.hold_rotate_max_angular_vel);
  }

  cmd_vel_pub_->publish(twist);
}

// 发布状态文本，并只将异常状态输出为 warn 日志。
void UwbFollowPathNode::publishStatus(const std::string & status)
{
  if (status == last_status_) {
    return;
  }

  last_status_ = status;
  std_msgs::msg::String msg;
  msg.data = status;
  status_pub_->publish(msg);
  if (
    status.rfind("tracking:", 0) == 0 ||
    status.rfind("planning:", 0) == 0 ||
    status.rfind("hold:", 0) == 0)
  {
    RCLCPP_DEBUG(get_logger(), "%s", status.c_str());
    return;
  }
  RCLCPP_WARN(get_logger(), "%s", status.c_str());
}

// 构造指定坐标系下的二维目标姿态。
geometry_msgs::msg::PoseStamped UwbFollowPathNode::makeBasePose(
  const double x,
  const double y,
  const double yaw,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.header.stamp = stamp;
  pose.pose.position = makePoint(x, y);
  pose.pose.orientation = yawToQuaternion(yaw);
  return pose;
}

// 计算两个 ROS 点之间的平面距离。
double UwbFollowPathNode::distance2d(
  const geometry_msgs::msg::Point & lhs,
  const geometry_msgs::msg::Point & rhs)
{
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

}  // namespace go2_uwb_mppi_follow

// 启动 UWB 直线路径与 MPPI 跟踪节点。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2_uwb_mppi_follow::UwbFollowPathNode>());
  rclcpp::shutdown();
  return 0;
}
