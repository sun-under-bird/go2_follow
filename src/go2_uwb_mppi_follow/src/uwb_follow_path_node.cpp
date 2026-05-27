#include "go2_uwb_mppi_follow/uwb_follow_path_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <functional>
#include <limits>
#include <string>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace go2_uwb_mppi_follow
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

bool finiteValue(const double value)
{
  return std::isfinite(value);
}

bool finitePoint(const geometry_msgs::msg::Point & point)
{
  return finiteValue(point.x) && finiteValue(point.y) && finiteValue(point.z);
}

geometry_msgs::msg::Point makePoint(const double x, const double y)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = 0.0;
  return point;
}

geometry_msgs::msg::Quaternion yawToQuaternion(const double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quaternion);
}

}  // namespace

UwbFollowPathNode::UwbFollowPathNode()
: Node("uwb_follow_path_node"),
  latest_target_stamp_(0, 0, get_clock()->get_clock_type())
{
  declareParameters();
  loadParameters();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  follow_path_client_ = rclcpp_action::create_client<FollowPath>(this, config_.follow_path_action);

  uwb_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
    config_.uwb_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&UwbFollowPathNode::uwbCallback, this, std::placeholders::_1));

  follow_path_pub_ = create_publisher<nav_msgs::msg::Path>(config_.follow_path_topic, 10);
  target_filtered_pub_ =
    create_publisher<geometry_msgs::msg::PointStamped>(config_.target_filtered_topic, 10);
  target_valid_pub_ = create_publisher<std_msgs::msg::Bool>(config_.target_valid_topic, 10);
  status_pub_ = create_publisher<std_msgs::msg::String>(config_.status_topic, 10);
  if (config_.publish_zero_velocity_on_stop && !config_.stop_cmd_vel_topic.empty()) {
    stop_cmd_vel_pub_ =
      create_publisher<geometry_msgs::msg::Twist>(config_.stop_cmd_vel_topic, 10);
  }

  const double publish_rate_hz = std::max(1.0, config_.publish_rate_hz);
  const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_hz));
  timer_ = create_wall_timer(timer_period, std::bind(&UwbFollowPathNode::timerCallback, this));

  RCLCPP_WARN(get_logger(), "uwb_follow_path_node started");
}

void UwbFollowPathNode::declareParameters()
{
  declare_parameter<std::string>("uwb_topic", "/uwb/target_point");
  declare_parameter<std::string>("odom_frame", "odom");
  declare_parameter<std::string>("base_frame", "base_link");
  declare_parameter<std::string>("follow_path_topic", "/uwb_follow/path");
  declare_parameter<std::string>("target_filtered_topic", "/uwb_follow/target_filtered");
  declare_parameter<std::string>("target_valid_topic", "/follow/target_valid");
  declare_parameter<std::string>("status_topic", "/follow/uwb_path_status");
  declare_parameter<std::string>("follow_path_action", "/follow_path");
  declare_parameter<std::string>("controller_id", "FollowPath");
  declare_parameter<std::string>("goal_checker_id", "general_goal_checker");
  declare_parameter<std::string>("stop_cmd_vel_topic", "/cmd_vel_nav");
  declare_parameter<double>("follow_distance_m", 1.0);
  declare_parameter<double>("distance_deadband_m", 0.15);
  declare_parameter<double>("min_goal_distance_m", 0.25);
  declare_parameter<double>("max_goal_distance_m", 2.0);
  declare_parameter<double>("path_resolution_m", 0.05);
  declare_parameter<double>("target_timeout_sec", 0.4);
  declare_parameter<double>("min_target_distance_m", 0.05);
  declare_parameter<double>("max_target_distance_m", 8.0);
  declare_parameter<double>("max_target_jump_m", 0.7);
  declare_parameter<double>("max_target_speed_mps", 3.0);
  declare_parameter<double>("smoothing_alpha", 0.35);
  declare_parameter<double>("publish_rate_hz", 5.0);
  declare_parameter<double>("transform_timeout_sec", 0.2);
  declare_parameter<double>("goal_update_distance_m", 0.10);
  declare_parameter<double>("goal_update_angle_rad", 0.0872664626);
  declare_parameter<double>("slow_turn_angle_rad", 1.0471975512);
  declare_parameter<double>("slow_turn_goal_scale", 0.5);
  declare_parameter<int>("min_path_poses", 2);
  declare_parameter<bool>("use_latest_tf", true);
  declare_parameter<bool>("publish_zero_velocity_on_stop", true);
}

void UwbFollowPathNode::loadParameters()
{
  config_.uwb_topic = get_parameter("uwb_topic").as_string();
  config_.odom_frame = get_parameter("odom_frame").as_string();
  config_.base_frame = get_parameter("base_frame").as_string();
  config_.follow_path_topic = get_parameter("follow_path_topic").as_string();
  config_.target_filtered_topic = get_parameter("target_filtered_topic").as_string();
  config_.target_valid_topic = get_parameter("target_valid_topic").as_string();
  config_.status_topic = get_parameter("status_topic").as_string();
  config_.follow_path_action = get_parameter("follow_path_action").as_string();
  config_.controller_id = get_parameter("controller_id").as_string();
  config_.goal_checker_id = get_parameter("goal_checker_id").as_string();
  config_.stop_cmd_vel_topic = get_parameter("stop_cmd_vel_topic").as_string();
  config_.follow_distance_m = get_parameter("follow_distance_m").as_double();
  config_.distance_deadband_m = get_parameter("distance_deadband_m").as_double();
  config_.min_goal_distance_m = get_parameter("min_goal_distance_m").as_double();
  config_.max_goal_distance_m = get_parameter("max_goal_distance_m").as_double();
  config_.path_resolution_m = get_parameter("path_resolution_m").as_double();
  config_.target_timeout_sec = get_parameter("target_timeout_sec").as_double();
  config_.min_target_distance_m = get_parameter("min_target_distance_m").as_double();
  config_.max_target_distance_m = get_parameter("max_target_distance_m").as_double();
  config_.max_target_jump_m = get_parameter("max_target_jump_m").as_double();
  config_.max_target_speed_mps = get_parameter("max_target_speed_mps").as_double();
  config_.smoothing_alpha = std::clamp(get_parameter("smoothing_alpha").as_double(), 0.0, 1.0);
  config_.publish_rate_hz = get_parameter("publish_rate_hz").as_double();
  config_.transform_timeout_sec = get_parameter("transform_timeout_sec").as_double();
  config_.goal_update_distance_m = get_parameter("goal_update_distance_m").as_double();
  config_.goal_update_angle_rad = get_parameter("goal_update_angle_rad").as_double();
  config_.slow_turn_angle_rad = get_parameter("slow_turn_angle_rad").as_double();
  config_.slow_turn_goal_scale =
    std::clamp(get_parameter("slow_turn_goal_scale").as_double(), 0.0, 1.0);
  config_.min_path_poses = get_parameter("min_path_poses").as_int();
  config_.use_latest_tf = get_parameter("use_latest_tf").as_bool();
  config_.publish_zero_velocity_on_stop =
    get_parameter("publish_zero_velocity_on_stop").as_bool();
}

void UwbFollowPathNode::uwbCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  const rclcpp::Time stamp = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ?
    now() : rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());

  const auto target_base = targetToBase(*msg);
  if (!target_base) {
    publishStatus("reject: UWB target TF failed");
    return;
  }

  if (!acceptTarget(target_base->point, stamp)) {
    return;
  }

  latest_filtered_target_base_ = filterTarget(target_base->point);
  latest_target_stamp_ = stamp;
  have_target_ = true;

  publishTargetFiltered(latest_filtered_target_base_, stamp);
  publishStatus("tracking: UWB target accepted");
}

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

bool UwbFollowPathNode::acceptTarget(
  const geometry_msgs::msg::Point & target,
  const rclcpp::Time & stamp)
{
  if (!finitePoint(target)) {
    publishStatus("reject: UWB target is not finite");
    return false;
  }

  const double target_distance = std::hypot(target.x, target.y);
  if (target_distance < config_.min_target_distance_m) {
    publishStatus("reject: UWB target too close");
    return false;
  }
  if (target_distance > config_.max_target_distance_m) {
    publishStatus("reject: UWB target too far");
    return false;
  }

  if (!have_target_) {
    return true;
  }

  const double jump_distance = distance2d(latest_filtered_target_base_, target);
  if (jump_distance > config_.max_target_jump_m) {
    publishStatus("reject: UWB target jump too large");
    return false;
  }

  const double dt = (stamp - latest_target_stamp_).seconds();
  if (dt > 1e-3 && jump_distance / dt > config_.max_target_speed_mps) {
    publishStatus("reject: UWB target speed too high");
    return false;
  }

  return true;
}

geometry_msgs::msg::Point UwbFollowPathNode::filterTarget(
  const geometry_msgs::msg::Point & target) const
{
  if (!have_target_ || config_.smoothing_alpha >= 1.0) {
    return makePoint(target.x, target.y);
  }

  geometry_msgs::msg::Point filtered;
  filtered.x =
    config_.smoothing_alpha * target.x +
    (1.0 - config_.smoothing_alpha) * latest_filtered_target_base_.x;
  filtered.y =
    config_.smoothing_alpha * target.y +
    (1.0 - config_.smoothing_alpha) * latest_filtered_target_base_.y;
  filtered.z = 0.0;
  return filtered;
}

void UwbFollowPathNode::timerCallback()
{
  const rclcpp::Time now_stamp = now();
  const bool target_valid = have_target_ &&
    (now_stamp - latest_target_stamp_).seconds() <= config_.target_timeout_sec;
  publishTargetValid(target_valid);

  if (!target_valid) {
    stopFollowing("stop: UWB target timeout", now_stamp);
    return;
  }

  const double range = std::hypot(
    latest_filtered_target_base_.x,
    latest_filtered_target_base_.y);
  if (range <= config_.follow_distance_m + config_.distance_deadband_m) {
    stopFollowing("hold: target within follow distance", now_stamp);
    return;
  }

  const auto path = buildFollowPath(latest_filtered_target_base_, now_stamp);
  if (!path) {
    stopFollowing("stop: follow path unavailable", now_stamp);
    return;
  }

  follow_path_pub_->publish(*path);

  const double target_yaw = std::atan2(
    latest_filtered_target_base_.y,
    latest_filtered_target_base_.x);
  double goal_distance = std::clamp(
    range - config_.follow_distance_m,
    config_.min_goal_distance_m,
    config_.max_goal_distance_m);
  if (std::abs(target_yaw) > config_.slow_turn_angle_rad) {
    goal_distance =
      std::max(config_.min_goal_distance_m, goal_distance * config_.slow_turn_goal_scale);
  }
  const geometry_msgs::msg::Point goal_base = makePoint(
    goal_distance * std::cos(target_yaw),
    goal_distance * std::sin(target_yaw));

  if (shouldSendActionGoal(goal_base, target_yaw)) {
    sendFollowPathGoal(*path, goal_base, target_yaw);
  }
}

std::optional<nav_msgs::msg::Path> UwbFollowPathNode::buildFollowPath(
  const geometry_msgs::msg::Point & target,
  const rclcpp::Time & stamp)
{
  const double range = std::hypot(target.x, target.y);
  if (range <= std::numeric_limits<double>::epsilon()) {
    return std::nullopt;
  }

  const double yaw = std::atan2(target.y, target.x);
  double goal_distance = std::clamp(
    range - config_.follow_distance_m,
    config_.min_goal_distance_m,
    config_.max_goal_distance_m);
  if (std::abs(yaw) > config_.slow_turn_angle_rad) {
    goal_distance =
      std::max(config_.min_goal_distance_m, goal_distance * config_.slow_turn_goal_scale);
  }

  const double resolution = std::max(0.01, config_.path_resolution_m);
  const int steps = std::max(config_.min_path_poses - 1,
    static_cast<int>(std::ceil(goal_distance / resolution)));

  nav_msgs::msg::Path path;
  path.header.frame_id = config_.odom_frame;
  path.header.stamp = stamp;
  path.poses.reserve(static_cast<std::size_t>(steps + 1));

  for (int i = 0; i <= steps; ++i) {
    const double distance = std::min(goal_distance, resolution * static_cast<double>(i));
    const auto base_pose = makeBasePose(
      distance * std::cos(yaw),
      distance * std::sin(yaw),
      yaw,
      config_.base_frame,
      stamp);
    const auto odom_pose = transformPoseToOdom(base_pose);
    if (!odom_pose) {
      return std::nullopt;
    }
    path.poses.push_back(*odom_pose);
  }

  return path;
}

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

void UwbFollowPathNode::sendFollowPathGoal(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Point & goal_base,
  const double goal_yaw_base)
{
  if (static_cast<int>(path.poses.size()) < config_.min_path_poses) {
    stopFollowing("stop: path too short", path.header.stamp);
    return;
  }

  if (!follow_path_client_->action_server_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "FollowPath action server is not ready");
    return;
  }

  FollowPath::Goal goal;
  goal.path = path;
  goal.controller_id = config_.controller_id;
  goal.goal_checker_id = config_.goal_checker_id;

  rclcpp_action::Client<FollowPath>::SendGoalOptions options;
  options.goal_response_callback =
    [this](const GoalHandleFollowPath::SharedPtr & goal_handle) {
      if (!goal_handle) {
        active_goal_handle_.reset();
        last_sent_goal_base_.reset();
        publishStatus("stop: FollowPath goal rejected");
        return;
      }
      active_goal_handle_ = goal_handle;
      publishStatus("tracking: FollowPath goal active");
    };
  options.result_callback =
    [this](const GoalHandleFollowPath::WrappedResult & result) {
      active_goal_handle_.reset();
      last_sent_goal_base_.reset();
      RCLCPP_DEBUG(get_logger(), "FollowPath result code: %d", static_cast<int>(result.code));
    };

  follow_path_client_->async_send_goal(goal, options);
  last_sent_goal_base_ = goal_base;
  last_sent_goal_yaw_base_ = goal_yaw_base;
}

bool UwbFollowPathNode::shouldSendActionGoal(
  const geometry_msgs::msg::Point & goal_base,
  const double goal_yaw_base) const
{
  if (!last_sent_goal_base_) {
    return true;
  }

  const double goal_delta = distance2d(*last_sent_goal_base_, goal_base);
  const double angle_delta = std::abs(normalizeAngle(goal_yaw_base - last_sent_goal_yaw_base_));
  return goal_delta >= config_.goal_update_distance_m ||
    angle_delta >= config_.goal_update_angle_rad;
}

void UwbFollowPathNode::cancelActiveGoal()
{
  if (!active_goal_handle_) {
    return;
  }

  try {
    follow_path_client_->async_cancel_goal(active_goal_handle_);
  } catch (const std::exception & exc) {
    RCLCPP_DEBUG(get_logger(), "FollowPath cancel failed: %s", exc.what());
  }
  active_goal_handle_.reset();
}

void UwbFollowPathNode::stopFollowing(const std::string & status, const rclcpp::Time & stamp)
{
  cancelActiveGoal();
  last_sent_goal_base_.reset();
  publishEmptyPath(stamp);
  publishZeroVelocity();
  publishStatus(status);
}

void UwbFollowPathNode::publishTargetFiltered(
  const geometry_msgs::msg::Point & target,
  const rclcpp::Time & stamp)
{
  geometry_msgs::msg::PointStamped msg;
  msg.header.frame_id = config_.base_frame;
  msg.header.stamp = stamp;
  msg.point = target;
  target_filtered_pub_->publish(msg);
}

void UwbFollowPathNode::publishTargetValid(const bool valid)
{
  std_msgs::msg::Bool msg;
  msg.data = valid;
  target_valid_pub_->publish(msg);
}

void UwbFollowPathNode::publishEmptyPath(const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = config_.odom_frame;
  path.header.stamp = stamp;
  follow_path_pub_->publish(path);
}

void UwbFollowPathNode::publishZeroVelocity()
{
  if (!stop_cmd_vel_pub_) {
    return;
  }

  geometry_msgs::msg::Twist twist;
  stop_cmd_vel_pub_->publish(twist);
}

void UwbFollowPathNode::publishStatus(const std::string & status)
{
  if (status == last_status_) {
    return;
  }

  last_status_ = status;
  std_msgs::msg::String msg;
  msg.data = status;
  status_pub_->publish(msg);
  if (status.rfind("tracking:", 0) == 0 || status.rfind("hold:", 0) == 0) {
    RCLCPP_DEBUG(get_logger(), "%s", status.c_str());
    return;
  }
  RCLCPP_WARN(get_logger(), "%s", status.c_str());
}

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

double UwbFollowPathNode::distance2d(
  const geometry_msgs::msg::Point & lhs,
  const geometry_msgs::msg::Point & rhs)
{
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

double UwbFollowPathNode::normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

}  // namespace go2_uwb_mppi_follow

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2_uwb_mppi_follow::UwbFollowPathNode>());
  rclcpp::shutdown();
  return 0;
}
