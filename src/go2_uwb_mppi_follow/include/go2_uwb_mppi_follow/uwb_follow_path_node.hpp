#pragma once

#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
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
  std::string target_filtered_topic;
  std::string target_valid_topic;
  std::string status_topic;
  std::string follow_path_action;
  std::string controller_id;
  std::string goal_checker_id;
  std::string stop_cmd_vel_topic;
  double follow_distance_m;
  double distance_deadband_m;
  double min_goal_distance_m;
  double max_goal_distance_m;
  double path_resolution_m;
  double target_timeout_sec;
  double min_target_distance_m;
  double max_target_distance_m;
  double max_target_jump_m;
  double max_target_speed_mps;
  double smoothing_alpha;
  double publish_rate_hz;
  double transform_timeout_sec;
  double goal_update_distance_m;
  double goal_update_angle_rad;
  double slow_turn_angle_rad;
  double slow_turn_goal_scale;
  int min_path_poses;
  bool use_latest_tf;
  bool publish_zero_velocity_on_stop;
};

class UwbFollowPathNode : public rclcpp::Node
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

  UwbFollowPathNode();

private:
  void declareParameters();
  void loadParameters();

  void uwbCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void timerCallback();

  std::optional<geometry_msgs::msg::PointStamped> targetToBase(
    const geometry_msgs::msg::PointStamped & target);
  bool acceptTarget(const geometry_msgs::msg::Point & target, const rclcpp::Time & stamp);
  geometry_msgs::msg::Point filterTarget(const geometry_msgs::msg::Point & target) const;

  std::optional<nav_msgs::msg::Path> buildFollowPath(
    const geometry_msgs::msg::Point & target,
    const rclcpp::Time & stamp);
  std::optional<geometry_msgs::msg::PoseStamped> transformPoseToOdom(
    const geometry_msgs::msg::PoseStamped & pose);

  void sendFollowPathGoal(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::Point & goal_base,
    double goal_yaw_base);
  bool shouldSendActionGoal(
    const geometry_msgs::msg::Point & goal_base,
    double goal_yaw_base) const;
  void cancelActiveGoal();
  void stopFollowing(const std::string & status, const rclcpp::Time & stamp);

  void publishTargetFiltered(const geometry_msgs::msg::Point & target, const rclcpp::Time & stamp);
  void publishTargetValid(bool valid);
  void publishEmptyPath(const rclcpp::Time & stamp);
  void publishZeroVelocity();
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
  static double normalizeAngle(double angle);

  UwbFollowPathConfig config_;
  bool have_target_{false};
  geometry_msgs::msg::Point latest_filtered_target_base_;
  rclcpp::Time latest_target_stamp_;
  std::optional<geometry_msgs::msg::Point> last_sent_goal_base_;
  double last_sent_goal_yaw_base_{0.0};
  std::string last_status_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr uwb_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr follow_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_filtered_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr stop_cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;
  GoalHandleFollowPath::SharedPtr active_goal_handle_;
};

}  // namespace go2_uwb_mppi_follow
