#pragma once

#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace go2_uwb_mppi_follow
{

struct UwbFollowPathConfig
{
  std::string uwb_topic;
  std::string odom_frame;
  std::string base_frame;
  std::string follow_path_topic;
  std::string target_topic;
  std::string follow_goal_topic;
  std::string target_valid_topic;
  std::string status_topic;
  std::string cmd_vel_topic;
  double follow_distance_m;
  double target_timeout_sec;
  double publish_rate_hz;
  double transform_timeout_sec;
  double direct_path_resolution_m;
  double hold_rotate_yaw_gain;
  double hold_rotate_max_angular_vel;
  double hold_rotate_yaw_deadband_rad;
  int min_path_poses;
  bool use_latest_tf;
  bool rotate_to_target_within_follow_distance;
};

class UwbFollowPathNode : public rclcpp::Node
{
public:
  UwbFollowPathNode();

private:
  void declareParameters();
  void loadParameters();

  void uwbCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void timerCallback();

  std::optional<geometry_msgs::msg::PointStamped> targetToBase(
    const geometry_msgs::msg::PointStamped & target);

  // 根据真实 UWB 人员点计算保持跟随距离后的 MPPI 站位目标。
  geometry_msgs::msg::Point makeFollowGoalPoint(
    const geometry_msgs::msg::Point & target) const;

  // 根据当前机器人位置和 UWB 目标点生成给 MPPI 跟踪的局部直线路径。
  std::optional<nav_msgs::msg::Path> buildDirectFollowPath(
    const geometry_msgs::msg::Point & goal_base,
    double goal_yaw_base,
    const rclcpp::Time & stamp);
  std::optional<geometry_msgs::msg::PoseStamped> transformPoseToOdom(
    const geometry_msgs::msg::PoseStamped & pose);

  void stopFollowing(const std::string & status, const rclcpp::Time & stamp);

  // 在近距离保持模式下取消 MPPI，并只发布原地转向速度。
  void holdAndRotateToTarget(
    const geometry_msgs::msg::Point & target,
    const rclcpp::Time & stamp);

  void publishTargetRaw(const geometry_msgs::msg::Point & target, const rclcpp::Time & stamp);
  void publishFollowGoal(const geometry_msgs::msg::Point & goal, const rclcpp::Time & stamp);
  void publishTargetValid(bool valid);
  void publishEmptyPath(const rclcpp::Time & stamp);
  // 根据近距离 UWB 方位角发布线速度为 0 的角速度指令。
  void publishHoldRotateVelocity(const geometry_msgs::msg::Point & target);
  void publishStatus(const std::string & status);

  static geometry_msgs::msg::PoseStamped makeBasePose(
    double x,
    double y,
    double yaw,
    const std::string & frame_id,
    const rclcpp::Time & stamp);
  static double distance2d(
    const geometry_msgs::msg::Point & lhs,
    const geometry_msgs::msg::Point & rhs);

  UwbFollowPathConfig config_;
  bool have_target_{false};
  geometry_msgs::msg::Point latest_target_base_;
  rclcpp::Time latest_target_stamp_;
  std::string last_status_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr uwb_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr follow_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr follow_goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace go2_uwb_mppi_follow
