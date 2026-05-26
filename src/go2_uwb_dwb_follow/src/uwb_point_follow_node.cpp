#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <limits>
#include <optional>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "uwb_aoa_pkg/msg/lib_aoa_robot_msg.hpp"

namespace go2_uwb_dwb_follow
{
namespace
{

using FollowPath = nav2_msgs::action::FollowPath;
using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;
using LibAoaRobotMsg = uwb_aoa_pkg::msg::LibAoaRobotMsg;

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

double distance2d(const geometry_msgs::msg::Point & lhs, const geometry_msgs::msg::Point & rhs)
{
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

geometry_msgs::msg::Quaternion yawToQuaternion(const double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quaternion);
}

}  // namespace

class UwbPointFollowNode : public rclcpp::Node
{
public:
  UwbPointFollowNode()
  : Node("uwb_point_follow_node"),
    latest_target_stamp_(0, 0, get_clock()->get_clock_type()),
    last_goal_send_time_(0, 0, get_clock()->get_clock_type())
  {
    declareParameters();
    loadParameters();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    follow_path_client_ = rclcpp_action::create_client<FollowPath>(this, follow_path_action_);

    uwb_sub_ = create_subscription<LibAoaRobotMsg>(
      uwb_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&UwbPointFollowNode::uwbCallback, this, std::placeholders::_1));

    target_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(target_topic_, 10);
    adapter_path_pub_ = create_publisher<nav_msgs::msg::Path>(adapter_path_topic_, 10);
    target_valid_pub_ = create_publisher<std_msgs::msg::Bool>(target_valid_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

    const double publish_rate_hz = std::max(1.0, publish_rate_hz_);
    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz));
    timer_ = create_wall_timer(timer_period, std::bind(&UwbPointFollowNode::timerCallback, this));

    RCLCPP_WARN(get_logger(), "uwb_point_follow_node started");
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("uwb_topic", "/libAoa_robot_publisher");
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("uwb_frame", "uwb_link");
    declare_parameter<bool>("use_tf_for_uwb", true);
    declare_parameter<bool>("use_latest_tf", true);
    declare_parameter<bool>("prefer_range_angle", true);
    declare_parameter<double>("transform_timeout_sec", 0.2);

    declare_parameter<std::string>("target_topic", "/uwb_follow_target");
    declare_parameter<std::string>("adapter_path_topic", "/follow_path");
    declare_parameter<std::string>("target_valid_topic", "/follow/target_valid");
    declare_parameter<std::string>("status_topic", "/follow/uwb_point_status");

    declare_parameter<std::string>("follow_path_action", "follow_path");
    declare_parameter<std::string>("controller_id", "FollowPath");
    declare_parameter<std::string>("goal_checker_id", "");

    declare_parameter<double>("follow_distance_m", 1.2);
    declare_parameter<double>("goal_tolerance_m", 0.15);
    declare_parameter<double>("target_timeout_sec", 2.0);
    declare_parameter<double>("min_target_distance_m", 0.05);
    declare_parameter<double>("max_target_distance_m", 8.0);
    declare_parameter<double>("max_target_jump_m", 1.5);
    declare_parameter<double>("max_target_speed_mps", 3.0);
    declare_parameter<double>("smoothing_alpha", 0.35);
    declare_parameter<bool>("allow_reverse_goal", false);

    declare_parameter<double>("publish_rate_hz", 10.0);
    declare_parameter<double>("action_resend_period_sec", 0.4);
  }

  void loadParameters()
  {
    uwb_topic_ = get_parameter("uwb_topic").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    uwb_frame_ = get_parameter("uwb_frame").as_string();
    use_tf_for_uwb_ = get_parameter("use_tf_for_uwb").as_bool();
    use_latest_tf_ = get_parameter("use_latest_tf").as_bool();
    prefer_range_angle_ = get_parameter("prefer_range_angle").as_bool();
    transform_timeout_sec_ = get_parameter("transform_timeout_sec").as_double();

    target_topic_ = get_parameter("target_topic").as_string();
    adapter_path_topic_ = get_parameter("adapter_path_topic").as_string();
    target_valid_topic_ = get_parameter("target_valid_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();

    follow_path_action_ = get_parameter("follow_path_action").as_string();
    controller_id_ = get_parameter("controller_id").as_string();
    goal_checker_id_ = get_parameter("goal_checker_id").as_string();

    follow_distance_m_ = get_parameter("follow_distance_m").as_double();
    goal_tolerance_m_ = get_parameter("goal_tolerance_m").as_double();
    target_timeout_sec_ = get_parameter("target_timeout_sec").as_double();
    min_target_distance_m_ = get_parameter("min_target_distance_m").as_double();
    max_target_distance_m_ = get_parameter("max_target_distance_m").as_double();
    max_target_jump_m_ = get_parameter("max_target_jump_m").as_double();
    max_target_speed_mps_ = get_parameter("max_target_speed_mps").as_double();
    smoothing_alpha_ = std::clamp(get_parameter("smoothing_alpha").as_double(), 0.0, 1.0);
    allow_reverse_goal_ = get_parameter("allow_reverse_goal").as_bool();

    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    action_resend_period_sec_ = get_parameter("action_resend_period_sec").as_double();
  }

  void uwbCallback(const LibAoaRobotMsg::SharedPtr msg)
  {
    const rclcpp::Time stamp = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ?
      now() : rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());

    const auto target = parseUwbTarget(*msg);
    if (!target) {
      publishStatus("stop: UWB target parse failed");
      return;
    }

    const std::string source_frame = use_tf_for_uwb_ ?
      (msg->header.frame_id.empty() ? uwb_frame_ : msg->header.frame_id) : base_frame_;
    const auto target_odom = targetToOdom(*target, stamp, source_frame);
    if (!target_odom) {
      publishStatus("stop: UWB target TF failed");
      return;
    }

    if (!acceptTarget(*target_odom, stamp)) {
      return;
    }

    latest_target_odom_ = smoothTarget(*target_odom);
    latest_target_stamp_ = stamp;
    have_target_ = true;
    publishTargetPoint(latest_target_odom_, stamp);
    publishStatus("tracking: UWB point accepted");
  }

  std::optional<geometry_msgs::msg::Point> parseUwbTarget(const LibAoaRobotMsg & msg) const
  {
    if (msg.state < 0) {
      return std::nullopt;
    }

    const bool xy_valid = finiteValue(msg.x) && finiteValue(msg.y);
    const bool range_angle_valid = finiteValue(msg.r) && finiteValue(msg.a) && msg.r > 1e-6;

    if (prefer_range_angle_ && range_angle_valid) {
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

  std::optional<geometry_msgs::msg::Point> targetToOdom(
    const geometry_msgs::msg::Point & target,
    const rclcpp::Time & stamp,
    const std::string & source_frame)
  {
    if (!finitePoint(target)) {
      return std::nullopt;
    }
    if (source_frame == odom_frame_) {
      return target;
    }

    geometry_msgs::msg::PointStamped source;
    source.header.frame_id = source_frame;
    source.header.stamp = use_latest_tf_ ? rclcpp::Time(0, 0, get_clock()->get_clock_type()) : stamp;
    source.point = target;

    try {
      const auto transformed = tf_buffer_->transform(
        source,
        odom_frame_,
        tf2::durationFromSec(transform_timeout_sec_));
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

  std::optional<geometry_msgs::msg::Point> robotPointInOdom(const rclcpp::Time & stamp)
  {
    if (base_frame_ == odom_frame_) {
      return makePoint(0.0, 0.0);
    }

    geometry_msgs::msg::PointStamped source;
    source.header.frame_id = base_frame_;
    source.header.stamp = use_latest_tf_ ? rclcpp::Time(0, 0, get_clock()->get_clock_type()) : stamp;
    source.point = makePoint(0.0, 0.0);

    try {
      const auto transformed = tf_buffer_->transform(
        source,
        odom_frame_,
        tf2::durationFromSec(transform_timeout_sec_));
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

  bool acceptTarget(const geometry_msgs::msg::Point & target, const rclcpp::Time & stamp)
  {
    const auto robot = robotPointInOdom(stamp);
    if (!robot) {
      publishStatus("reject: robot pose unavailable");
      return false;
    }

    const double target_distance = distance2d(*robot, target);
    if (target_distance < min_target_distance_m_) {
      publishStatus("reject: UWB target too close");
      return false;
    }
    if (target_distance > max_target_distance_m_) {
      publishStatus("reject: UWB target too far");
      return false;
    }

    if (!have_target_) {
      return true;
    }

    const double spacing = distance2d(latest_target_odom_, target);
    if (spacing > max_target_jump_m_) {
      publishStatus("reject: UWB target jump too large");
      return false;
    }

    const double dt = (stamp - latest_target_stamp_).seconds();
    if (dt > 1e-3 && spacing / dt > max_target_speed_mps_) {
      publishStatus("reject: UWB target speed too high");
      return false;
    }

    return true;
  }

  geometry_msgs::msg::Point smoothTarget(const geometry_msgs::msg::Point & target) const
  {
    if (!have_target_ || smoothing_alpha_ >= 1.0) {
      return target;
    }

    geometry_msgs::msg::Point smoothed;
    smoothed.x = smoothing_alpha_ * target.x + (1.0 - smoothing_alpha_) * latest_target_odom_.x;
    smoothed.y = smoothing_alpha_ * target.y + (1.0 - smoothing_alpha_) * latest_target_odom_.y;
    smoothed.z = 0.0;
    return smoothed;
  }

  void timerCallback()
  {
    const rclcpp::Time now_stamp = now();
    const bool target_valid = have_target_ &&
      (now_stamp - latest_target_stamp_).seconds() <= target_timeout_sec_;
    publishTargetValid(target_valid);

    if (!target_valid) {
      cancelActiveGoal();
      publishEmptyAdapterPath(now_stamp);
      publishStatus("stop: UWB target timeout");
      return;
    }

    const auto robot = robotPointInOdom(now_stamp);
    if (!robot) {
      cancelActiveGoal();
      publishStatus("stop: robot pose unavailable");
      return;
    }

    const double target_distance = distance2d(*robot, latest_target_odom_);
    if (target_distance <= follow_distance_m_ + goal_tolerance_m_ && !allow_reverse_goal_) {
      cancelActiveGoal();
      publishEmptyAdapterPath(now_stamp);
      publishStatus("hold: target within follow distance");
      return;
    }

    const auto follow_goal = buildFollowGoal(*robot, latest_target_odom_, target_distance);
    if (!follow_goal) {
      cancelActiveGoal();
      publishEmptyAdapterPath(now_stamp);
      publishStatus("stop: follow goal unavailable");
      return;
    }

    if (distance2d(*robot, *follow_goal) <= goal_tolerance_m_) {
      cancelActiveGoal();
      publishEmptyAdapterPath(now_stamp);
      publishStatus("hold: follow goal reached");
      return;
    }

    const auto adapter_path = buildAdapterPath(*robot, *follow_goal, now_stamp);
    adapter_path_pub_->publish(adapter_path);

    if (shouldResendActionGoal(now_stamp)) {
      sendFollowPathGoal(adapter_path);
    }
  }

  std::optional<geometry_msgs::msg::Point> buildFollowGoal(
    const geometry_msgs::msg::Point & robot,
    const geometry_msgs::msg::Point & target,
    const double target_distance) const
  {
    if (target_distance < 1e-6) {
      return std::nullopt;
    }

    const double ux = (target.x - robot.x) / target_distance;
    const double uy = (target.y - robot.y) / target_distance;

    geometry_msgs::msg::Point goal;
    goal.x = target.x - follow_distance_m_ * ux;
    goal.y = target.y - follow_distance_m_ * uy;
    goal.z = 0.0;
    return goal;
  }

  nav_msgs::msg::Path buildAdapterPath(
    const geometry_msgs::msg::Point & robot,
    const geometry_msgs::msg::Point & goal,
    const rclcpp::Time & stamp) const
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = odom_frame_;
    path.header.stamp = stamp;
    path.poses.push_back(makePose(robot, goal, stamp));
    path.poses.push_back(makePose(goal, goal, stamp));
    return path;
  }

  geometry_msgs::msg::PoseStamped makePose(
    const geometry_msgs::msg::Point & point,
    const geometry_msgs::msg::Point & next,
    const rclcpp::Time & stamp) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = odom_frame_;
    pose.header.stamp = stamp;
    pose.pose.position = point;
    pose.pose.orientation = yawToQuaternion(std::atan2(next.y - point.y, next.x - point.x));
    return pose;
  }

  void sendFollowPathGoal(const nav_msgs::msg::Path & path)
  {
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
    goal.controller_id = controller_id_;
    goal.goal_checker_id = goal_checker_id_;

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

  void cancelActiveGoal()
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

  bool shouldResendActionGoal(const rclcpp::Time & stamp) const
  {
    if (last_goal_send_time_.nanoseconds() == 0) {
      return true;
    }
    return (stamp - last_goal_send_time_).seconds() >= action_resend_period_sec_;
  }

  void publishTargetPoint(
    const geometry_msgs::msg::Point & target,
    const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PointStamped msg;
    msg.header.frame_id = odom_frame_;
    msg.header.stamp = stamp;
    msg.point = target;
    target_pub_->publish(msg);
  }

  void publishTargetValid(const bool valid)
  {
    std_msgs::msg::Bool msg;
    msg.data = valid;
    target_valid_pub_->publish(msg);
  }

  void publishEmptyAdapterPath(const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = odom_frame_;
    path.header.stamp = stamp;
    adapter_path_pub_->publish(path);
  }

  void publishStatus(const std::string & status)
  {
    if (status == last_status_) {
      return;
    }

    last_status_ = status;
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
    if (status.rfind("tracking:", 0) == 0) {
      RCLCPP_DEBUG(get_logger(), "%s", status.c_str());
      return;
    }
    RCLCPP_WARN(get_logger(), "%s", status.c_str());
  }

  std::string uwb_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string uwb_frame_;
  bool use_tf_for_uwb_;
  bool use_latest_tf_;
  bool prefer_range_angle_;
  double transform_timeout_sec_;

  std::string target_topic_;
  std::string adapter_path_topic_;
  std::string target_valid_topic_;
  std::string status_topic_;

  std::string follow_path_action_;
  std::string controller_id_;
  std::string goal_checker_id_;

  double follow_distance_m_;
  double goal_tolerance_m_;
  double target_timeout_sec_;
  double min_target_distance_m_;
  double max_target_distance_m_;
  double max_target_jump_m_;
  double max_target_speed_mps_;
  double smoothing_alpha_;
  bool allow_reverse_goal_;

  double publish_rate_hz_;
  double action_resend_period_sec_;

  bool have_target_{false};
  geometry_msgs::msg::Point latest_target_odom_;
  rclcpp::Time latest_target_stamp_;
  rclcpp::Time last_goal_send_time_;
  std::string last_status_;

  rclcpp::Subscription<LibAoaRobotMsg>::SharedPtr uwb_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr adapter_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;
  GoalHandleFollowPath::SharedPtr active_goal_handle_;
};

}  // namespace go2_uwb_dwb_follow

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2_uwb_dwb_follow::UwbPointFollowNode>());
  rclcpp::shutdown();
  return 0;
}
