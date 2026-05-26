#include "go2_uwb_mppi_follow/uwb_path_tracker_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <iterator>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace go2_uwb_mppi_follow
{
namespace
{

// 判断浮点数是否是可用于几何计算的有限值。
bool isFinite(const double value)
{
  return std::isfinite(value);
}

// 判断几何点的三个坐标是否都是有限值。
bool isFinitePoint(const geometry_msgs::msg::Point & point)
{
  return isFinite(point.x) && isFinite(point.y) && isFinite(point.z);
}

// 根据 x、y 坐标创建 z 为 0 的平面目标点。
geometry_msgs::msg::Point makePoint(const double x, const double y)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = 0.0;
  return point;
}

}  // 命名空间

// 构造 UWB 路径跟随节点，并初始化参数、话题、TF 和定时器。
UwbPathTrackerNode::UwbPathTrackerNode()
: Node("uwb_path_tracker_node")
{
  declareParameters();
  loadParameters();

  last_goal_send_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  last_valid_target_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  uwb_sub_ = create_subscription<LibAoaRobotMsg>(
    config_.uwb_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&UwbPathTrackerNode::uwbCallback, this, std::placeholders::_1));

  target_path_pub_ = create_publisher<nav_msgs::msg::Path>(config_.target_path_topic, 10);
  follow_path_pub_ = create_publisher<nav_msgs::msg::Path>(config_.follow_path_topic, 10);
  target_valid_pub_ = create_publisher<std_msgs::msg::Bool>(config_.target_valid_topic, 10);
  status_pub_ = create_publisher<std_msgs::msg::String>(config_.status_topic, 10);

  follow_path_client_ = rclcpp_action::create_client<FollowPath>(this, config_.follow_path_action);

  const double publish_rate_hz = std::max(1.0, config_.publish_rate_hz);
  const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_hz));
  timer_ = create_wall_timer(
    timer_period,
    std::bind(&UwbPathTrackerNode::timerCallback, this));

  RCLCPP_WARN(get_logger(), "uwb_path_tracker_node started");
}

// 声明节点使用的所有可调 ROS 参数。
void UwbPathTrackerNode::declareParameters()
{
  declare_parameter<std::string>("uwb_topic", "/libAoa_robot_publisher");
  declare_parameter<std::string>("odom_frame", "odom");
  declare_parameter<std::string>("base_frame", "base_link");
  declare_parameter<std::string>("uwb_frame", "uwb_link");
  declare_parameter<std::string>("target_path_topic", "/uwb_target_history");
  declare_parameter<std::string>("follow_path_topic", "/follow_path");
  declare_parameter<std::string>("target_valid_topic", "/follow/target_valid");
  declare_parameter<std::string>("status_topic", "/follow/uwb_path_status");
  declare_parameter<std::string>("follow_path_action", "follow_path");
  declare_parameter<std::string>("controller_id", "FollowPath");
  declare_parameter<std::string>("goal_checker_id", "");
  declare_parameter<double>("follow_distance_m", 1.2);
  declare_parameter<double>("history_length_m", 6.0);
  declare_parameter<double>("min_sample_spacing_m", 0.10);
  declare_parameter<double>("max_target_jump_m", 1.5);
  declare_parameter<double>("max_target_speed_mps", 3.0);
  declare_parameter<double>("min_target_distance_m", 0.05);
  declare_parameter<double>("max_target_distance_m", 8.0);
  declare_parameter<double>("target_timeout_sec", 2.0);
  declare_parameter<double>("publish_rate_hz", 10.0);
  declare_parameter<double>("action_resend_period_sec", 0.5);
  declare_parameter<double>("transform_timeout_sec", 0.2);
  declare_parameter<int>("min_path_poses", 2);
  declare_parameter<bool>("use_tf_for_uwb", true);
  declare_parameter<bool>("use_latest_tf", true);
  declare_parameter<bool>("prefer_range_angle", true);
}

// 读取 ROS 参数并缓存到跟随配置中。
void UwbPathTrackerNode::loadParameters()
{
  config_.uwb_topic = get_parameter("uwb_topic").as_string();
  config_.odom_frame = get_parameter("odom_frame").as_string();
  config_.base_frame = get_parameter("base_frame").as_string();
  config_.uwb_frame = get_parameter("uwb_frame").as_string();
  config_.target_path_topic = get_parameter("target_path_topic").as_string();
  config_.follow_path_topic = get_parameter("follow_path_topic").as_string();
  config_.target_valid_topic = get_parameter("target_valid_topic").as_string();
  config_.status_topic = get_parameter("status_topic").as_string();
  config_.follow_path_action = get_parameter("follow_path_action").as_string();
  config_.controller_id = get_parameter("controller_id").as_string();
  config_.goal_checker_id = get_parameter("goal_checker_id").as_string();
  config_.follow_distance_m = get_parameter("follow_distance_m").as_double();
  config_.history_length_m = get_parameter("history_length_m").as_double();
  config_.min_sample_spacing_m = get_parameter("min_sample_spacing_m").as_double();
  config_.max_target_jump_m = get_parameter("max_target_jump_m").as_double();
  config_.max_target_speed_mps = get_parameter("max_target_speed_mps").as_double();
  config_.min_target_distance_m = get_parameter("min_target_distance_m").as_double();
  config_.max_target_distance_m = get_parameter("max_target_distance_m").as_double();
  config_.target_timeout_sec = get_parameter("target_timeout_sec").as_double();
  config_.publish_rate_hz = get_parameter("publish_rate_hz").as_double();
  config_.action_resend_period_sec = get_parameter("action_resend_period_sec").as_double();
  config_.transform_timeout_sec = get_parameter("transform_timeout_sec").as_double();
  config_.min_path_poses = get_parameter("min_path_poses").as_int();
  config_.use_tf_for_uwb = get_parameter("use_tf_for_uwb").as_bool();
  config_.use_latest_tf = get_parameter("use_latest_tf").as_bool();
  config_.prefer_range_angle = get_parameter("prefer_range_angle").as_bool();
}

// 接收原始 UWB 数据，转换到 odom 后写入短时目标历史路径。
void UwbPathTrackerNode::uwbCallback(const LibAoaRobotMsg::SharedPtr msg)
{
  const rclcpp::Time stamp = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ?
    now() : rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());

  const auto target = parseUwbTarget(*msg);
  if (!target) {
    publishStatus("stop: UWB target parse failed");
    return;
  }

  const std::string source_frame = config_.use_tf_for_uwb ?
    (msg->header.frame_id.empty() ? config_.uwb_frame : msg->header.frame_id) : config_.base_frame;
  const auto target_odom = targetToOdom(*target, stamp, source_frame);
  if (!target_odom) {
    publishStatus("stop: UWB target TF failed");
    return;
  }

  if (!isTargetSampleValid(*target_odom, stamp)) {
    return;
  }

  appendTargetSample(*target_odom, stamp);
  publishStatus("tracking: UWB target accepted");
}

// 定时发布目标历史路径、生成 MPPI 参考路径，并刷新 FollowPath 目标。
void UwbPathTrackerNode::timerCallback()
{
  const rclcpp::Time now_stamp = now();
  pruneTargetHistory(now_stamp);
  target_path_pub_->publish(buildTargetHistoryPath(now_stamp));

  const bool valid = !target_history_.empty() &&
    (now_stamp - target_history_.back().stamp).seconds() <= config_.target_timeout_sec;
  publishTargetValid(valid);

  if (!valid) {
    cancelActiveGoal();
    publishStatus("stop: UWB target timeout");
    return;
  }

  const auto follow_path = buildFollowPath(now_stamp);
  if (!follow_path) {
    cancelActiveGoal();
    publishStatus("stop: UWB history path too short");
    return;
  }

  follow_path_pub_->publish(*follow_path);
  publishStatus("tracking: follow path ready");

  if (shouldResendActionGoal(now_stamp)) {
    sendFollowPathGoal(*follow_path);
  }
}

// 将 UWB 消息解析为 UWB 坐标系下的目标点。
std::optional<geometry_msgs::msg::Point> UwbPathTrackerNode::parseUwbTarget(
  const LibAoaRobotMsg & msg) const
{
  if (msg.state < 0) {
    return std::nullopt;
  }

  const bool xy_valid = isFinite(msg.x) && isFinite(msg.y);
  const bool range_angle_valid = isFinite(msg.r) && isFinite(msg.a) && msg.r > 1e-6;

  if (config_.prefer_range_angle && range_angle_valid) {
    return makePoint(msg.r * std::cos(msg.a), msg.r * std::sin(msg.a));
  }

  if (xy_valid && std::hypot(msg.x, msg.y) > 1e-6) {
    return makePoint(msg.x, msg.y);
  }

  if (range_angle_valid) {
    return makePoint(msg.r * std::cos(msg.a), msg.r * std::sin(msg.a));
  }

  if (xy_valid) {
    return makePoint(msg.x, msg.y);
  }

  return std::nullopt;
}

// 将目标点从来源坐标系转换到 odom 坐标系。
std::optional<geometry_msgs::msg::Point> UwbPathTrackerNode::targetToOdom(
  const geometry_msgs::msg::Point & target,
  const rclcpp::Time & stamp,
  const std::string & source_frame)
{
  if (!isFinitePoint(target)) {
    return std::nullopt;
  }

  if (source_frame == config_.odom_frame) {
    return target;
  }

  geometry_msgs::msg::PointStamped source;
  source.header.frame_id = source_frame;
  source.header.stamp = config_.use_latest_tf ? rclcpp::Time(0, 0, get_clock()->get_clock_type()) : stamp;
  source.point = target;

  try {
    const auto transformed = tf_buffer_->transform(
      source,
      config_.odom_frame,
      tf2::durationFromSec(config_.transform_timeout_sec));
    return transformed.point;
  } catch (const tf2::TransformException & exc) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "UWB target transform failed: %s",
      exc.what());
    return std::nullopt;
  }
}

// 获取机器人 base 坐标原点在 odom 坐标系下的位置。
std::optional<geometry_msgs::msg::Point> UwbPathTrackerNode::robotPointInOdom(
  const rclcpp::Time & stamp)
{
  if (config_.base_frame == config_.odom_frame) {
    return makePoint(0.0, 0.0);
  }

  geometry_msgs::msg::PointStamped source;
  source.header.frame_id = config_.base_frame;
  source.header.stamp = config_.use_latest_tf ? rclcpp::Time(0, 0, get_clock()->get_clock_type()) : stamp;
  source.point = makePoint(0.0, 0.0);

  try {
    const auto transformed = tf_buffer_->transform(
      source,
      config_.odom_frame,
      tf2::durationFromSec(config_.transform_timeout_sec));
    return transformed.point;
  } catch (const tf2::TransformException & exc) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "base to odom transform failed: %s",
      exc.what());
    return std::nullopt;
  }
}

// 检查目标点的距离、速度和跳变是否满足接收条件。
bool UwbPathTrackerNode::isTargetSampleValid(
  const geometry_msgs::msg::Point & point,
  const rclcpp::Time & stamp)
{
  if (!isFinitePoint(point)) {
    publishStatus("reject: UWB target is not finite");
    return false;
  }

  const auto robot_point = robotPointInOdom(stamp);
  if (!robot_point) {
    publishStatus("reject: robot pose unavailable");
    return false;
  }

  const double target_distance = distance2d(*robot_point, point);
  if (target_distance < config_.min_target_distance_m) {
    publishStatus("reject: UWB target too close");
    return false;
  }
  if (target_distance > config_.max_target_distance_m) {
    publishStatus("reject: UWB target too far");
    return false;
  }

  if (target_history_.empty()) {
    return true;
  }

  const auto & last_sample = target_history_.back();
  const double sample_spacing = distance2d(last_sample.point, point);
  if (sample_spacing < config_.min_sample_spacing_m) {
    publishStatus("skip: UWB target spacing too small");
    return false;
  }
  if (sample_spacing > config_.max_target_jump_m) {
    publishStatus("reject: UWB target jump too large");
    return false;
  }

  const double dt = (stamp - last_sample.stamp).seconds();
  if (dt > 1e-3) {
    const double target_speed = sample_spacing / dt;
    if (target_speed > config_.max_target_speed_mps) {
      publishStatus("reject: UWB target speed too high");
      return false;
    }
  }

  return true;
}

// 追加有效目标点，并保持历史路径长度在短时范围内。
void UwbPathTrackerNode::appendTargetSample(
  const geometry_msgs::msg::Point & point,
  const rclcpp::Time & stamp)
{
  target_history_.push_back(TargetSample{stamp, point});
  last_valid_target_time_ = stamp;
  pruneTargetHistory(stamp);
}

// 删除超时或超出历史长度窗口的目标历史点。
void UwbPathTrackerNode::pruneTargetHistory(const rclcpp::Time & now_stamp)
{
  while (!target_history_.empty() &&
    (now_stamp - target_history_.front().stamp).seconds() > config_.target_timeout_sec * 5.0)
  {
    target_history_.pop_front();
  }

  double kept_length = 0.0;
  for (auto it = target_history_.rbegin(); it != target_history_.rend();) {
    const auto next_it = std::next(it);
    if (next_it == target_history_.rend()) {
      break;
    }

    kept_length += distance2d(it->point, next_it->point);
    if (kept_length <= config_.history_length_m) {
      ++it;
      continue;
    }

    const auto erase_end = next_it.base();
    target_history_.erase(target_history_.begin(), erase_end);
    break;
  }
}

// 生成用于 RViz 等工具查看的 UWB 目标历史路径。
nav_msgs::msg::Path UwbPathTrackerNode::buildTargetHistoryPath(const rclcpp::Time & now_stamp) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = config_.odom_frame;
  path.header.stamp = now_stamp;

  for (std::size_t i = 0; i < target_history_.size(); ++i) {
    const auto next_point = i + 1 < target_history_.size() ?
      std::optional<geometry_msgs::msg::Point>(target_history_[i + 1].point) : std::nullopt;
    path.poses.push_back(makePose(target_history_[i].point, next_point, now_stamp));
  }

  return path;
}

// 生成 MPPI 需要跟踪的短时滞后参考路径。
std::optional<nav_msgs::msg::Path> UwbPathTrackerNode::buildFollowPath(
  const rclcpp::Time & now_stamp)
{
  if (target_history_.size() < 2) {
    return std::nullopt;
  }

  const auto robot_point = robotPointInOdom(now_stamp);
  if (!robot_point) {
    return std::nullopt;
  }

  geometry_msgs::msg::Point follow_point = target_history_.front().point;
  std::size_t follow_prev_index = 0;
  double distance_from_target = 0.0;
  bool found_follow_point = false;

  for (std::size_t i = target_history_.size() - 1; i > 0; --i) {
    const auto & current = target_history_[i].point;
    const auto & previous = target_history_[i - 1].point;
    const double segment_length = distance2d(current, previous);
    if (segment_length < 1e-6) {
      continue;
    }

    if (distance_from_target + segment_length >= config_.follow_distance_m) {
      const double needed = config_.follow_distance_m - distance_from_target;
      const double ratio = needed / segment_length;
      follow_point.x = current.x + (previous.x - current.x) * ratio;
      follow_point.y = current.y + (previous.y - current.y) * ratio;
      follow_point.z = 0.0;
      follow_prev_index = i - 1;
      found_follow_point = true;
      break;
    }

    distance_from_target += segment_length;
  }

  if (!found_follow_point) {
    return std::nullopt;
  }

  std::size_t nearest_index = 0;
  double nearest_distance = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i <= follow_prev_index; ++i) {
    const double candidate_distance = distance2d(*robot_point, target_history_[i].point);
    if (candidate_distance < nearest_distance) {
      nearest_index = i;
      nearest_distance = candidate_distance;
    }
  }

  std::vector<geometry_msgs::msg::Point> reference_points;
  reference_points.push_back(*robot_point);
  for (std::size_t i = nearest_index; i <= follow_prev_index; ++i) {
    if (distance2d(reference_points.back(), target_history_[i].point) > 1e-3) {
      reference_points.push_back(target_history_[i].point);
    }
  }
  if (distance2d(reference_points.back(), follow_point) > 1e-3) {
    reference_points.push_back(follow_point);
  }

  if (static_cast<int>(reference_points.size()) < config_.min_path_poses) {
    return std::nullopt;
  }

  nav_msgs::msg::Path path;
  path.header.frame_id = config_.odom_frame;
  path.header.stamp = now_stamp;
  for (std::size_t i = 0; i < reference_points.size(); ++i) {
    const auto next_point = i + 1 < reference_points.size() ?
      std::optional<geometry_msgs::msg::Point>(reference_points[i + 1]) : std::nullopt;
    path.poses.push_back(makePose(reference_points[i], next_point, now_stamp));
  }

  return path;
}

// 发布当前 UWB 目标历史路径是否可用。
void UwbPathTrackerNode::publishTargetValid(const bool valid)
{
  std_msgs::msg::Bool msg;
  msg.data = valid;
  target_valid_pub_->publish(msg);
}

// 在节点状态变化时发布简短状态文本。
void UwbPathTrackerNode::publishStatus(const std::string & status)
{
  if (status == last_status_) {
    return;
  }

  last_status_ = status;
  std_msgs::msg::String msg;
  msg.data = status;
  status_pub_->publish(msg);
  if (status.rfind("tracking:", 0) == 0 || status.rfind("skip:", 0) == 0) {
    RCLCPP_DEBUG(get_logger(), "%s", status.c_str());
    return;
  }
  RCLCPP_WARN(get_logger(), "%s", status.c_str());
}

// 目标跟踪无效时取消当前 FollowPath 目标。
void UwbPathTrackerNode::cancelActiveGoal()
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

// 判断是否已经到达重新发送 FollowPath 目标的时间。
bool UwbPathTrackerNode::shouldResendActionGoal(const rclcpp::Time & now_stamp) const
{
  if (last_goal_send_time_.nanoseconds() == 0) {
    return true;
  }

  return (now_stamp - last_goal_send_time_).seconds() >= config_.action_resend_period_sec;
}

// 将最新参考路径发送给 Nav2 FollowPath action 服务。
void UwbPathTrackerNode::sendFollowPathGoal(const nav_msgs::msg::Path & path)
{
  if (static_cast<int>(path.poses.size()) < config_.min_path_poses) {
    cancelActiveGoal();
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
        publishStatus("stop: FollowPath goal rejected");
        return;
      }
      active_goal_handle_ = goal_handle;
    };
  options.result_callback =
    [this](const GoalHandleFollowPath::WrappedResult & result) {
      active_goal_handle_.reset();
      RCLCPP_DEBUG(get_logger(), "FollowPath result code: %d", static_cast<int>(result.code));
    };

  follow_path_client_->async_send_goal(goal, options);
  last_goal_send_time_ = now();
}

// 根据当前点和下一个点生成带朝向的路径位姿。
geometry_msgs::msg::PoseStamped UwbPathTrackerNode::makePose(
  const geometry_msgs::msg::Point & point,
  const std::optional<geometry_msgs::msg::Point> & next_point,
  const rclcpp::Time & stamp) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = config_.odom_frame;
  pose.header.stamp = stamp;
  pose.pose.position = point;

  double yaw = 0.0;
  if (next_point) {
    yaw = std::atan2(next_point->y - point.y, next_point->x - point.x);
  }

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation = tf2::toMsg(quaternion);
  return pose;
}

// 计算两个几何点之间的平面距离。
double UwbPathTrackerNode::distance2d(
  const geometry_msgs::msg::Point & lhs,
  const geometry_msgs::msg::Point & rhs)
{
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

}  // 命名空间 go2_uwb_mppi_follow

// 启动 UWB 路径跟随节点。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2_uwb_mppi_follow::UwbPathTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
