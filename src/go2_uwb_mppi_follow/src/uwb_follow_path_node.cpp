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

constexpr double kPi = 3.14159265358979323846;

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

// 构造 UWB 路径跟踪节点，并初始化 TF、action、话题和定时器。
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
  follow_goal_pub_ =
    create_publisher<geometry_msgs::msg::PointStamped>(config_.follow_goal_topic, 10);
  target_valid_pub_ = create_publisher<std_msgs::msg::Bool>(config_.target_valid_topic, 10);
  status_pub_ = create_publisher<std_msgs::msg::String>(config_.status_topic, 10);
  if (
    (config_.publish_zero_velocity_on_stop ||
    config_.rotate_to_target_within_follow_distance) &&
    !config_.stop_cmd_vel_topic.empty())
  {
    stop_cmd_vel_pub_ =
      create_publisher<geometry_msgs::msg::Twist>(config_.stop_cmd_vel_topic, 10);
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
  declare_parameter<std::string>("target_filtered_topic", "/uwb_follow/target_filtered");
  declare_parameter<std::string>("follow_goal_topic", "/uwb_follow/follow_goal");
  declare_parameter<std::string>("target_valid_topic", "/follow/target_valid");
  declare_parameter<std::string>("status_topic", "/follow/uwb_path_status");
  declare_parameter<std::string>("follow_path_action", "/follow_path");
  declare_parameter<std::string>("controller_id", "FollowPath");
  declare_parameter<std::string>("goal_checker_id", "general_goal_checker");
  declare_parameter<std::string>("stop_cmd_vel_topic", "/cmd_vel_nav");
  declare_parameter<double>("follow_distance_m", 1.5);
  declare_parameter<double>("target_timeout_sec", 1.0);
  declare_parameter<double>("min_target_distance_m", 0.0);
  declare_parameter<double>("max_target_distance_m", 8.0);
  declare_parameter<double>("smoothing_alpha", 1.0);
  declare_parameter<double>("target_hold_sec", 0.5);
  declare_parameter<double>("max_target_jump_m", 1.0);
  declare_parameter<double>("max_target_speed_mps", 2.5);
  declare_parameter<double>("publish_rate_hz", 5.0);
  declare_parameter<double>("transform_timeout_sec", 0.2);
  declare_parameter<double>("goal_update_distance_m", 0.10);
  declare_parameter<double>("goal_update_angle_rad", 0.0872664626);
  declare_parameter<double>("direct_path_resolution_m", 0.20);
  declare_parameter<double>("hold_rotate_yaw_gain", 1.0);
  declare_parameter<double>("hold_rotate_max_angular_vel", 0.8);
  declare_parameter<double>("hold_rotate_yaw_deadband_rad", 0.05);
  declare_parameter<int>("min_valid_samples", 1);
  declare_parameter<int>("max_invalid_samples", 4);
  declare_parameter<int>("min_path_poses", 2);
  declare_parameter<bool>("use_latest_tf", true);
  declare_parameter<bool>("publish_zero_velocity_on_stop", true);
  declare_parameter<bool>("rotate_to_target_within_follow_distance", true);
}

// 读取 ROS 参数并写入本节点的运行配置。
void UwbFollowPathNode::loadParameters()
{
  config_.uwb_topic = get_parameter("uwb_topic").as_string();
  config_.odom_frame = get_parameter("odom_frame").as_string();
  config_.base_frame = get_parameter("base_frame").as_string();
  config_.follow_path_topic = get_parameter("follow_path_topic").as_string();
  config_.target_filtered_topic = get_parameter("target_filtered_topic").as_string();
  config_.follow_goal_topic = get_parameter("follow_goal_topic").as_string();
  config_.target_valid_topic = get_parameter("target_valid_topic").as_string();
  config_.status_topic = get_parameter("status_topic").as_string();
  config_.follow_path_action = get_parameter("follow_path_action").as_string();
  config_.controller_id = get_parameter("controller_id").as_string();
  config_.goal_checker_id = get_parameter("goal_checker_id").as_string();
  config_.stop_cmd_vel_topic = get_parameter("stop_cmd_vel_topic").as_string();
  config_.follow_distance_m = get_parameter("follow_distance_m").as_double();
  config_.target_timeout_sec = get_parameter("target_timeout_sec").as_double();
  config_.min_target_distance_m = get_parameter("min_target_distance_m").as_double();
  config_.max_target_distance_m = get_parameter("max_target_distance_m").as_double();
  config_.smoothing_alpha = std::clamp(get_parameter("smoothing_alpha").as_double(), 0.0, 1.0);
  config_.target_hold_sec = std::max(0.0, get_parameter("target_hold_sec").as_double());
  config_.max_target_jump_m = std::max(0.0, get_parameter("max_target_jump_m").as_double());
  config_.max_target_speed_mps =
    std::max(0.0, get_parameter("max_target_speed_mps").as_double());
  config_.publish_rate_hz = get_parameter("publish_rate_hz").as_double();
  config_.transform_timeout_sec = get_parameter("transform_timeout_sec").as_double();
  config_.goal_update_distance_m = get_parameter("goal_update_distance_m").as_double();
  config_.goal_update_angle_rad = get_parameter("goal_update_angle_rad").as_double();
  config_.direct_path_resolution_m =
    std::max(0.05, get_parameter("direct_path_resolution_m").as_double());
  config_.hold_rotate_yaw_gain = get_parameter("hold_rotate_yaw_gain").as_double();
  config_.hold_rotate_max_angular_vel =
    std::max(0.0, get_parameter("hold_rotate_max_angular_vel").as_double());
  config_.hold_rotate_yaw_deadband_rad =
    std::max(0.0, get_parameter("hold_rotate_yaw_deadband_rad").as_double());
  config_.min_valid_samples =
    std::max(1, static_cast<int>(get_parameter("min_valid_samples").as_int()));
  config_.max_invalid_samples =
    std::max(0, static_cast<int>(get_parameter("max_invalid_samples").as_int()));
  config_.min_path_poses = static_cast<int>(get_parameter("min_path_poses").as_int());
  config_.use_latest_tf = get_parameter("use_latest_tf").as_bool();
  config_.publish_zero_velocity_on_stop =
    get_parameter("publish_zero_velocity_on_stop").as_bool();
  config_.rotate_to_target_within_follow_distance =
    get_parameter("rotate_to_target_within_follow_distance").as_bool();
}

// 接收 UWB 目标点，转换到机器人坐标系并发布当前有效目标。
void UwbFollowPathNode::uwbCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  const rclcpp::Time stamp = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ?
    now() : rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());

  const auto target_base = targetToBase(*msg);
  if (!target_base) {
    handleRejectedTarget("reject: UWB target TF failed", stamp);
    return;
  }

  std::string reject_reason;
  if (!acceptTarget(target_base->point, reject_reason)) {
    handleRejectedTarget(reject_reason, stamp);
    return;
  }

  if (rejectUnstableTarget(target_base->point, stamp, reject_reason)) {
    handleRejectedTarget(reject_reason, stamp);
    return;
  }

  latest_filtered_target_base_ = filterTarget(target_base->point);
  have_filtered_target_ = true;
  latest_target_stamp_ = stamp;
  ++valid_sample_count_;
  invalid_sample_count_ = 0;

  publishTargetFiltered(latest_filtered_target_base_, stamp);

  if (valid_sample_count_ < config_.min_valid_samples) {
    publishTargetValid(false);
    publishStatus("acquiring: UWB target samples");
    return;
  }

  have_target_ = true;
  const double range = std::hypot(
    latest_filtered_target_base_.x,
    latest_filtered_target_base_.y);
  if (range < config_.follow_distance_m) {
    holdAndRotateToTarget(latest_filtered_target_base_, stamp);
    return;
  }

  publishStatus("tracking: UWB target accepted");
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

// 检查目标点是否满足有限值和距离范围要求。
bool UwbFollowPathNode::acceptTarget(
  const geometry_msgs::msg::Point & target,
  std::string & reject_reason) const
{
  if (!finitePoint(target)) {
    reject_reason = "reject: UWB target is not finite";
    return false;
  }

  const double target_distance = std::hypot(target.x, target.y);
  if (target_distance < config_.min_target_distance_m) {
    reject_reason = "reject: UWB target too close";
    return false;
  }
  if (target_distance > config_.max_target_distance_m) {
    reject_reason = "reject: UWB target too far";
    return false;
  }

  return true;
}

// 检查 UWB 是否相对上一帧出现不合理跳变或速度尖峰。
bool UwbFollowPathNode::rejectUnstableTarget(
  const geometry_msgs::msg::Point & target,
  const rclcpp::Time & stamp,
  std::string & reject_reason) const
{
  if (!have_filtered_target_) {
    return false;
  }

  const double jump = distance2d(latest_filtered_target_base_, target);
  if (config_.max_target_jump_m > 0.0 && jump > config_.max_target_jump_m) {
    reject_reason = "hold: UWB target jump";
    return true;
  }

  const double dt = (stamp - latest_target_stamp_).seconds();
  if (dt > 1e-3 && config_.max_target_speed_mps > 0.0) {
    const double speed = jump / dt;
    if (speed > config_.max_target_speed_mps) {
      reject_reason = "hold: UWB target speed spike";
      return true;
    }
  }

  return false;
}

// 对目标点做一阶平滑；默认 alpha 为 1 时不改变 UWB 原始点。
geometry_msgs::msg::Point UwbFollowPathNode::filterTarget(
  const geometry_msgs::msg::Point & target) const
{
  if (!have_filtered_target_ || config_.smoothing_alpha >= 1.0) {
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

// 处理无效 UWB 样本：短时保留上一目标，连续异常或超时后停车。
void UwbFollowPathNode::handleRejectedTarget(
  const std::string & status,
  const rclcpp::Time & stamp)
{
  ++invalid_sample_count_;

  const bool can_hold_previous = have_target_ &&
    invalid_sample_count_ <= config_.max_invalid_samples &&
    (stamp - latest_target_stamp_).seconds() <= config_.target_hold_sec;
  if (can_hold_previous) {
    publishStatus(status);
    return;
  }

  have_target_ = false;
  have_filtered_target_ = false;
  valid_sample_count_ = 0;
  stopFollowing(status, stamp);
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

// 周期性检查目标状态，近距离原地转向，否则生成局部路径交给 MPPI。
void UwbFollowPathNode::timerCallback()
{
  const rclcpp::Time now_stamp = now();
  const bool target_valid = have_target_ &&
    (now_stamp - latest_target_stamp_).seconds() <= config_.target_timeout_sec;
  publishTargetValid(target_valid);

  if (!target_valid) {
    const bool acquiring = !have_target_ &&
      valid_sample_count_ > 0 &&
      (now_stamp - latest_target_stamp_).seconds() <= config_.target_timeout_sec;
    if (acquiring) {
      publishEmptyPath(now_stamp);
      publishZeroVelocity();
      publishStatus("acquiring: UWB target samples");
      return;
    }
    have_target_ = false;
    have_filtered_target_ = false;
    valid_sample_count_ = 0;
    invalid_sample_count_ = 0;
    stopFollowing("stop: UWB target timeout", now_stamp);
    return;
  }

  const double range = std::hypot(
    latest_filtered_target_base_.x,
    latest_filtered_target_base_.y);
  if (range < config_.follow_distance_m) {
    holdAndRotateToTarget(latest_filtered_target_base_, now_stamp);
    return;
  }

  const double target_yaw = std::atan2(
    latest_filtered_target_base_.y,
    latest_filtered_target_base_.x);
  const geometry_msgs::msg::Point goal_base = makeFollowGoalPoint(latest_filtered_target_base_);
  publishFollowGoal(goal_base, now_stamp);

  if (shouldSendActionGoal(goal_base, target_yaw)) {
    const auto path = buildDirectFollowPath(goal_base, target_yaw, now_stamp);
    if (!path) {
      stopFollowing("stop: direct follow path TF failed", now_stamp);
      return;
    }

    follow_path_pub_->publish(*path);
    sendFollowPathGoal(*path, goal_base, target_yaw);
    publishStatus("tracking: direct MPPI path requested");
  }
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

// 将局部直线路径发送给 Nav2 FollowPath，由 MPPI 进行避障跟踪。
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
    stopFollowing("stop: FollowPath action server is not ready", path.header.stamp);
    return;
  }

  FollowPath::Goal goal;
  goal.path = path;
  goal.controller_id = config_.controller_id;
  goal.goal_checker_id = config_.goal_checker_id;

  const std::uint64_t request_id = ++follow_goal_request_id_;
  rclcpp_action::Client<FollowPath>::SendGoalOptions options;
  options.goal_response_callback =
    [this, request_id](const GoalHandleFollowPath::SharedPtr & goal_handle) {
      if (request_id != follow_goal_request_id_) {
        if (goal_handle) {
          try {
            // 停车或重规划后才返回的旧 goal 不能继续执行，拿到句柄后立即取消。
            follow_path_client_->async_cancel_goal(goal_handle);
          } catch (const std::exception & exc) {
            RCLCPP_DEBUG(get_logger(), "Stale FollowPath cancel failed: %s", exc.what());
          }
        }
        return;
      }
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
    [this, request_id](const GoalHandleFollowPath::WrappedResult & result) {
      if (request_id != follow_goal_request_id_) {
        return;
      }
      active_goal_handle_.reset();
      last_sent_goal_base_.reset();
      RCLCPP_DEBUG(get_logger(), "FollowPath result code: %d", static_cast<int>(result.code));
    };

  follow_path_client_->async_send_goal(goal, options);
  last_sent_goal_base_ = goal_base;
  last_sent_goal_yaw_base_ = goal_yaw_base;
}

// 判断当前 UWB 目标相对上次发送目标是否变化到需要重新规划。
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

// 取消正在进行的 FollowPath 跟踪目标。
void UwbFollowPathNode::cancelActiveGoal()
{
  ++follow_goal_request_id_;

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

// 停止跟随：取消跟踪、发布空路径、发布零速度并更新状态。
void UwbFollowPathNode::stopFollowing(const std::string & status, const rclcpp::Time & stamp)
{
  cancelActiveGoal();
  last_sent_goal_base_.reset();
  publishEmptyPath(stamp);
  publishZeroVelocity();
  publishStatus(status);
}

// 近距离保持：不使用 MPPI，只发布原地转向速度让机身朝向 UWB 目标。
void UwbFollowPathNode::holdAndRotateToTarget(
  const geometry_msgs::msg::Point & target,
  const rclcpp::Time & stamp)
{
  cancelActiveGoal();
  last_sent_goal_base_.reset();
  publishFollowGoal(makeFollowGoalPoint(target), stamp);
  publishEmptyPath(stamp);

  if (config_.rotate_to_target_within_follow_distance) {
    publishHoldRotateVelocity(target);
  } else {
    publishZeroVelocity();
  }

  publishStatus("hold: rotate to UWB target");
}

// 发布当前用于规划和过滤的 UWB 目标点。
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

// 向 Nav2 原始速度话题发布零速度。
void UwbFollowPathNode::publishZeroVelocity()
{
  if (!config_.publish_zero_velocity_on_stop || !stop_cmd_vel_pub_) {
    return;
  }

  geometry_msgs::msg::Twist twist;
  stop_cmd_vel_pub_->publish(twist);
}

// 发布近距离原地转向速度，线速度固定为 0。
void UwbFollowPathNode::publishHoldRotateVelocity(const geometry_msgs::msg::Point & target)
{
  if (!stop_cmd_vel_pub_) {
    return;
  }

  geometry_msgs::msg::Twist twist;
  if (!finitePoint(target)) {
    stop_cmd_vel_pub_->publish(twist);
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

  stop_cmd_vel_pub_->publish(twist);
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

// 将角度归一化到 [-pi, pi] 区间。
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

// 启动 UWB 直线路径与 MPPI 跟踪节点。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2_uwb_mppi_follow::UwbFollowPathNode>());
  rclcpp::shutdown();
  return 0;
}
