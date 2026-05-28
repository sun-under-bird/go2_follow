#include "go2_uwb_teb_follow/uwb_teb_follow_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace go2_uwb_teb_follow
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

// 判断浮点数是否是可用于几何计算的有限值。
bool finiteValue(const double value)
{
  return std::isfinite(value);
}

// 将平面 yaw 角转换为 ROS 四元数姿态。
geometry_msgs::msg::Quaternion yawToQuaternion(const double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quaternion);
}

}  // namespace

// 构造 UWB 到 Smac Hybrid 再到 TEB 的跟随节点，并初始化 action、话题和 TF。
UwbTebFollowNode::UwbTebFollowNode()
: Node("uwb_teb_follow_node"),
  latest_target_stamp_(0, 0, get_clock()->get_clock_type()),
  planner_request_stamp_(0, 0, get_clock()->get_clock_type())
{
  declareParameters();
  loadParameters();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  planner_client_ =
    rclcpp_action::create_client<ComputePathToPose>(this, config_.planner_action);
  follow_path_client_ = rclcpp_action::create_client<FollowPath>(this, config_.follow_path_action);

  uwb_sub_ = create_subscription<LibAoaRobotMsg>(
    config_.uwb_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&UwbTebFollowNode::uwbCallback, this, std::placeholders::_1));

  target_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(config_.target_topic, 10);
  path_pub_ = create_publisher<nav_msgs::msg::Path>(config_.path_topic, 10);
  target_valid_pub_ = create_publisher<std_msgs::msg::Bool>(config_.target_valid_topic, 10);
  status_pub_ = create_publisher<std_msgs::msg::String>(config_.status_topic, 10);
  if (config_.publish_zero_velocity_on_stop && !config_.stop_cmd_vel_topic.empty()) {
    stop_cmd_vel_pub_ =
      create_publisher<geometry_msgs::msg::Twist>(config_.stop_cmd_vel_topic, 10);
  }

  const double publish_rate_hz = std::max(1.0, config_.publish_rate_hz);
  const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_hz));
  timer_ = create_wall_timer(timer_period, std::bind(&UwbTebFollowNode::timerCallback, this));

  RCLCPP_WARN(get_logger(), "uwb_teb_follow_node started");
}

// 声明 UWB 输入、Nav2 action、跟随距离和停车输出等参数。
void UwbTebFollowNode::declareParameters()
{
  declare_parameter<std::string>("uwb_topic", "/libAoa_robot_publisher");
  declare_parameter<std::string>("odom_frame", "odom");
  declare_parameter<std::string>("base_frame", "base_link");
  declare_parameter<std::string>("uwb_frame", "base_link");
  declare_parameter<std::string>("target_topic", "/uwb_teb/target");
  declare_parameter<std::string>("path_topic", "/uwb_teb/path");
  declare_parameter<std::string>("target_valid_topic", "/follow/target_valid");
  declare_parameter<std::string>("status_topic", "/follow/uwb_teb_status");
  declare_parameter<std::string>("planner_action", "/compute_path_to_pose");
  declare_parameter<std::string>("planner_id", "GridBased");
  declare_parameter<std::string>("follow_path_action", "/follow_path");
  declare_parameter<std::string>("controller_id", "FollowPath");
  declare_parameter<std::string>("goal_checker_id", "general_goal_checker");
  declare_parameter<std::string>("stop_cmd_vel_topic", "/cmd_vel_nav");
  declare_parameter<double>("follow_distance_m", 1.0);
  declare_parameter<double>("target_timeout_sec", 1.0);
  declare_parameter<double>("transform_timeout_sec", 0.2);
  declare_parameter<double>("planner_timeout_sec", 1.0);
  declare_parameter<double>("goal_update_distance_m", 0.10);
  declare_parameter<double>("goal_update_angle_rad", 0.0872664626);
  declare_parameter<double>("publish_rate_hz", 5.0);
  declare_parameter<int>("min_path_poses", 2);
  declare_parameter<bool>("use_latest_tf", true);
  declare_parameter<bool>("publish_zero_velocity_on_stop", true);
}

// 读取 ROS 参数并缓存到配置结构，避免回调中反复查参数服务器。
void UwbTebFollowNode::loadParameters()
{
  config_.uwb_topic = get_parameter("uwb_topic").as_string();
  config_.odom_frame = get_parameter("odom_frame").as_string();
  config_.base_frame = get_parameter("base_frame").as_string();
  config_.uwb_frame = get_parameter("uwb_frame").as_string();
  config_.target_topic = get_parameter("target_topic").as_string();
  config_.path_topic = get_parameter("path_topic").as_string();
  config_.target_valid_topic = get_parameter("target_valid_topic").as_string();
  config_.status_topic = get_parameter("status_topic").as_string();
  config_.planner_action = get_parameter("planner_action").as_string();
  config_.planner_id = get_parameter("planner_id").as_string();
  config_.follow_path_action = get_parameter("follow_path_action").as_string();
  config_.controller_id = get_parameter("controller_id").as_string();
  config_.goal_checker_id = get_parameter("goal_checker_id").as_string();
  config_.stop_cmd_vel_topic = get_parameter("stop_cmd_vel_topic").as_string();
  config_.follow_distance_m = get_parameter("follow_distance_m").as_double();
  config_.target_timeout_sec = get_parameter("target_timeout_sec").as_double();
  config_.transform_timeout_sec = get_parameter("transform_timeout_sec").as_double();
  config_.planner_timeout_sec = get_parameter("planner_timeout_sec").as_double();
  config_.goal_update_distance_m = get_parameter("goal_update_distance_m").as_double();
  config_.goal_update_angle_rad = get_parameter("goal_update_angle_rad").as_double();
  config_.publish_rate_hz = get_parameter("publish_rate_hz").as_double();
  config_.min_path_poses = get_parameter("min_path_poses").as_int();
  config_.use_latest_tf = get_parameter("use_latest_tf").as_bool();
  config_.publish_zero_velocity_on_stop =
    get_parameter("publish_zero_velocity_on_stop").as_bool();
}

// 接收 UWB 原始消息，只根据 state 判断有效性，并更新当前可规划目标。
void UwbTebFollowNode::uwbCallback(const LibAoaRobotMsg::SharedPtr msg)
{
  const rclcpp::Time stamp = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ?
    now() : rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());

  if (msg->state < 0) {
    clearTargetAndStop("stop: invalid UWB state", stamp);
    return;
  }

  const auto parsed_target = parseUwbTarget(*msg);
  if (!parsed_target) {
    clearTargetAndStop("stop: UWB target parse failed", stamp);
    return;
  }

  const std::string source_frame = msg->header.frame_id.empty() ?
    config_.uwb_frame : msg->header.frame_id;
  geometry_msgs::msg::PointStamped target_source;
  target_source.header.frame_id = source_frame;
  target_source.header.stamp = stamp;
  target_source.point = *parsed_target;

  const auto target_base = transformPoint(target_source, config_.base_frame);
  if (!target_base) {
    clearTargetAndStop("stop: UWB target TF to base failed", stamp);
    return;
  }

  const auto target_odom = transformPoint(*target_base, config_.odom_frame);
  if (!target_odom) {
    clearTargetAndStop("stop: UWB target TF to odom failed", stamp);
    return;
  }

  latest_target_base_ = target_base->point;
  latest_target_odom_ = target_odom->point;
  latest_target_stamp_ = stamp;
  have_target_ = true;

  publishTarget(*target_base);
  publishTargetValid(true);

  // 近距离目标不再送给 Smac/TEB，直接停车并发布空路径。
  if (std::hypot(latest_target_base_.x, latest_target_base_.y) < config_.follow_distance_m) {
    stopFollowing("hold: target within follow distance", stamp);
    return;
  }

  publishStatus("tracking: UWB target accepted");
}

// 从 LibAoaRobotMsg 中解析目标点：优先使用非零 x/y，失败后回退到 r/a。
std::optional<geometry_msgs::msg::Point> UwbTebFollowNode::parseUwbTarget(
  const LibAoaRobotMsg & msg) const
{
  const bool xy_valid = finiteValue(msg.x) && finiteValue(msg.y);
  const bool xy_non_zero = std::hypot(msg.x, msg.y) > 1e-6;
  if (xy_valid && xy_non_zero) {
    return makePoint(msg.x, msg.y);
  }

  if (finiteValue(msg.r) && finiteValue(msg.a)) {
    return makePoint(msg.r * std::cos(msg.a), msg.r * std::sin(msg.a));
  }

  if (xy_valid) {
    return makePoint(msg.x, msg.y);
  }

  return std::nullopt;
}

// 将点转换到指定坐标系；坐标系相同则只规范 frame_id 和 z 值。
std::optional<geometry_msgs::msg::PointStamped> UwbTebFollowNode::transformPoint(
  const geometry_msgs::msg::PointStamped & point,
  const std::string & target_frame)
{
  if (!finitePoint(point.point)) {
    return std::nullopt;
  }

  if (point.header.frame_id == target_frame) {
    geometry_msgs::msg::PointStamped transformed = point;
    transformed.header.frame_id = target_frame;
    transformed.point.z = 0.0;
    return transformed;
  }

  geometry_msgs::msg::PointStamped source = point;
  if (config_.use_latest_tf) {
    source.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  try {
    auto transformed = tf_buffer_->transform(
      source,
      target_frame,
      tf2::durationFromSec(config_.transform_timeout_sec));
    transformed.header.stamp = point.header.stamp;
    transformed.point.z = 0.0;
    return transformed;
  } catch (const tf2::TransformException & exc) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "target transform from %s to %s failed: %s",
      point.header.frame_id.c_str(),
      target_frame.c_str(),
      exc.what());
    return std::nullopt;
  }
}

// 周期性检查目标状态，必要时请求 Smac Hybrid 重新规划全局路径。
void UwbTebFollowNode::timerCallback()
{
  const rclcpp::Time now_stamp = now();
  const bool target_valid = have_target_ &&
    (now_stamp - latest_target_stamp_).seconds() <= config_.target_timeout_sec;
  publishTargetValid(target_valid);

  if (!target_valid) {
    stopFollowing("stop: UWB target timeout", now_stamp);
    return;
  }

  const double range = std::hypot(latest_target_base_.x, latest_target_base_.y);
  if (range < config_.follow_distance_m) {
    stopFollowing("hold: target within follow distance", now_stamp);
    return;
  }

  const double target_yaw_base = std::atan2(latest_target_base_.y, latest_target_base_.x);
  if (planner_request_in_flight_) {
    if (plannerRequestTimedOut(now_stamp)) {
      stopFollowing("stop: planner timeout", now_stamp);
      return;
    }
    publishStatus("planning: global path pending");
    return;
  }

  if (shouldRequestPath(latest_target_base_, target_yaw_base)) {
    requestGlobalPath(latest_target_base_, target_yaw_base, now_stamp);
  }
}

// 请求 planner_server 使用 Smac Hybrid 计算到 UWB 点的全局路径。
void UwbTebFollowNode::requestGlobalPath(
  const geometry_msgs::msg::Point & goal_base,
  const double goal_yaw_base,
  const rclcpp::Time & stamp)
{
  if (!planner_client_->action_server_is_ready()) {
    stopFollowing("stop: ComputePathToPose action server is not ready", stamp);
    return;
  }

  const auto goal_pose = transformPoseToOdom(
    makeBasePose(goal_base.x, goal_base.y, goal_yaw_base, config_.base_frame, stamp));
  if (!goal_pose) {
    stopFollowing("stop: UWB goal pose TF to odom failed", stamp);
    return;
  }

  ComputePathToPose::Goal goal;
  goal.goal = *goal_pose;
  goal.planner_id = config_.planner_id;
  goal.use_start = false;

  const std::uint64_t request_id = ++planner_request_id_;
  rclcpp_action::Client<ComputePathToPose>::SendGoalOptions options;
  options.goal_response_callback =
    [this, request_id](const GoalHandleComputePathToPose::SharedPtr & goal_handle) {
      if (request_id != planner_request_id_) {
        return;
      }
      if (!goal_handle) {
        planner_request_in_flight_ = false;
        active_planner_goal_handle_.reset();
        publishStatus("stop: ComputePathToPose goal rejected");
        return;
      }
      active_planner_goal_handle_ = goal_handle;
      publishStatus("planning: global path active");
    };
  options.result_callback =
    [this, request_id, goal_base, goal_yaw_base](
      const GoalHandleComputePathToPose::WrappedResult & result) {
      if (request_id != planner_request_id_) {
        return;
      }
      planner_request_in_flight_ = false;
      active_planner_goal_handle_.reset();

      const rclcpp::Time now_stamp = now();
      const bool target_valid = have_target_ &&
        (now_stamp - latest_target_stamp_).seconds() <= config_.target_timeout_sec;
      if (!target_valid) {
        stopFollowing("stop: UWB target timeout", now_stamp);
        return;
      }

      const double range = std::hypot(latest_target_base_.x, latest_target_base_.y);
      if (range < config_.follow_distance_m) {
        stopFollowing("hold: target within follow distance", now_stamp);
        return;
      }

      // 规划结果返回时目标已经明显移动，则丢弃旧路径，等待下一次重规划。
      if (distance2d(latest_target_base_, goal_base) > config_.goal_update_distance_m) {
        publishStatus("skip: stale global path result");
        return;
      }

      if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result) {
        stopFollowing("stop: global planner failed", now_stamp);
        return;
      }

      auto path = result.result->path;
      path.header.stamp = now_stamp;
      if (static_cast<int>(path.poses.size()) < config_.min_path_poses) {
        stopFollowing("stop: global path too short", now_stamp);
        return;
      }

      path_pub_->publish(path);
      sendFollowPathGoal(path, goal_base, goal_yaw_base);
    };

  planner_request_in_flight_ = true;
  planner_request_stamp_ = stamp;
  planner_client_->async_send_goal(goal, options);
  publishStatus("planning: global path requested");
}

// 将 base_frame 下的目标姿态转换到 odom，供 ComputePathToPose 使用。
std::optional<geometry_msgs::msg::PoseStamped> UwbTebFollowNode::transformPoseToOdom(
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
      "goal pose transform to odom failed: %s",
      exc.what());
    return std::nullopt;
  }
}

// 将 Smac Hybrid 返回的全局路径发送给 controller_server 的 TEB FollowPath。
void UwbTebFollowNode::sendFollowPathGoal(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Point & goal_base,
  const double goal_yaw_base)
{
  if (static_cast<int>(path.poses.size()) < config_.min_path_poses) {
    stopFollowing("stop: path too short", path.header.stamp);
    return;
  }

  if (!follow_path_client_->action_server_is_ready()) {
    stopFollowing("stop: FollowPath action server is not ready", path.header.stamp);
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
        active_follow_goal_handle_.reset();
        last_sent_goal_base_.reset();
        publishStatus("stop: FollowPath goal rejected");
        return;
      }
      active_follow_goal_handle_ = goal_handle;
      publishStatus("tracking: FollowPath goal active");
    };
  options.result_callback =
    [this](const GoalHandleFollowPath::WrappedResult & result) {
      active_follow_goal_handle_.reset();
      last_sent_goal_base_.reset();
      RCLCPP_DEBUG(get_logger(), "FollowPath result code: %d", static_cast<int>(result.code));
    };

  follow_path_client_->async_send_goal(goal, options);
  last_sent_goal_base_ = goal_base;
  last_sent_goal_yaw_base_ = goal_yaw_base;
}

// 判断目标移动或朝向变化是否达到重新规划阈值。
bool UwbTebFollowNode::shouldRequestPath(
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

// 判断当前 ComputePathToPose 请求是否超过允许等待时间。
bool UwbTebFollowNode::plannerRequestTimedOut(const rclcpp::Time & stamp) const
{
  if (!planner_request_in_flight_) {
    return false;
  }

  return (stamp - planner_request_stamp_).seconds() > config_.planner_timeout_sec;
}

// 取消正在进行的全局规划请求，并让旧回调因 request_id 变化而失效。
void UwbTebFollowNode::cancelActivePlannerGoal()
{
  ++planner_request_id_;
  planner_request_in_flight_ = false;

  if (!active_planner_goal_handle_) {
    return;
  }

  try {
    planner_client_->async_cancel_goal(active_planner_goal_handle_);
  } catch (const std::exception & exc) {
    RCLCPP_DEBUG(get_logger(), "ComputePathToPose cancel failed: %s", exc.what());
  }
  active_planner_goal_handle_.reset();
}

// 取消正在执行的 TEB FollowPath 跟踪目标。
void UwbTebFollowNode::cancelActiveFollowGoal()
{
  if (!active_follow_goal_handle_) {
    return;
  }

  try {
    follow_path_client_->async_cancel_goal(active_follow_goal_handle_);
  } catch (const std::exception & exc) {
    RCLCPP_DEBUG(get_logger(), "FollowPath cancel failed: %s", exc.what());
  }
  active_follow_goal_handle_.reset();
}

// 停止跟随：取消规划和控制，发布空路径、零速度和状态。
void UwbTebFollowNode::stopFollowing(const std::string & status, const rclcpp::Time & stamp)
{
  cancelActivePlannerGoal();
  cancelActiveFollowGoal();
  last_sent_goal_base_.reset();
  publishEmptyPath(stamp);
  publishZeroVelocity();
  publishStatus(status);
}

// 清除当前目标后停车，用于 state 无效、解析失败或 TF 失败等硬失效场景。
void UwbTebFollowNode::clearTargetAndStop(
  const std::string & status,
  const rclcpp::Time & stamp)
{
  have_target_ = false;
  publishTargetValid(false);
  stopFollowing(status, stamp);
}

// 发布当前 UWB 目标点，默认 frame 为 base_frame，便于在 RViz 中观察距离。
void UwbTebFollowNode::publishTarget(const geometry_msgs::msg::PointStamped & target)
{
  target_pub_->publish(target);
}

// 发布当前 UWB 目标是否有效。
void UwbTebFollowNode::publishTargetValid(const bool valid)
{
  std_msgs::msg::Bool msg;
  msg.data = valid;
  target_valid_pub_->publish(msg);
}

// 发布空路径，通知调试端和下游当前没有可跟踪路径。
void UwbTebFollowNode::publishEmptyPath(const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = config_.odom_frame;
  path.header.stamp = stamp;
  path_pub_->publish(path);
}

// 向 Nav2 原始速度话题发布零速度，用于近距离和异常停车。
void UwbTebFollowNode::publishZeroVelocity()
{
  if (!stop_cmd_vel_pub_) {
    return;
  }

  geometry_msgs::msg::Twist twist;
  stop_cmd_vel_pub_->publish(twist);
}

// 发布状态文本，并只将非正常跟踪状态输出为 warn 日志。
void UwbTebFollowNode::publishStatus(const std::string & status)
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

// 根据二维坐标构造 z 为 0 的 ROS 点。
geometry_msgs::msg::Point UwbTebFollowNode::makePoint(const double x, const double y)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = 0.0;
  return point;
}

// 构造指定坐标系下的二维目标姿态。
geometry_msgs::msg::PoseStamped UwbTebFollowNode::makeBasePose(
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
double UwbTebFollowNode::distance2d(
  const geometry_msgs::msg::Point & lhs,
  const geometry_msgs::msg::Point & rhs)
{
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

// 将角度归一化到 [-pi, pi]，用于判断目标朝向变化。
double UwbTebFollowNode::normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

// 判断三维点的所有坐标是否都是有限值。
bool UwbTebFollowNode::finitePoint(const geometry_msgs::msg::Point & point)
{
  return finiteValue(point.x) && finiteValue(point.y) && finiteValue(point.z);
}

}  // namespace go2_uwb_teb_follow

// 启动 UWB + Smac Hybrid + TEB 跟随节点。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2_uwb_teb_follow::UwbTebFollowNode>());
  rclcpp::shutdown();
  return 0;
}
