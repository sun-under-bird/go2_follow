#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "uwb_aoa_pkg/msg/lib_aoa_robot_msg.hpp"

namespace go2_uwb_teb_follow
{

struct UwbTebFollowConfig
{
  std::string uwb_topic;
  std::string odom_frame;
  std::string base_frame;
  std::string uwb_frame;
  std::string target_topic;
  std::string path_topic;
  std::string target_valid_topic;
  std::string status_topic;
  std::string planner_action;
  std::string planner_id;
  std::string follow_path_action;
  std::string controller_id;
  std::string goal_checker_id;
  std::string stop_cmd_vel_topic;
  double follow_distance_m;
  double target_timeout_sec;
  double transform_timeout_sec;
  double planner_timeout_sec;
  double goal_update_distance_m;
  double goal_update_angle_rad;
  double publish_rate_hz;
  int min_path_poses;
  bool use_latest_tf;
  bool publish_zero_velocity_on_stop;
};

class UwbTebFollowNode : public rclcpp::Node
{
public:
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using GoalHandleComputePathToPose =
    rclcpp_action::ClientGoalHandle<ComputePathToPose>;
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;
  using LibAoaRobotMsg = uwb_aoa_pkg::msg::LibAoaRobotMsg;

  UwbTebFollowNode();

private:
  void declareParameters();
  void loadParameters();

  void uwbCallback(const LibAoaRobotMsg::SharedPtr msg);
  void timerCallback();

  std::optional<geometry_msgs::msg::Point> parseUwbTarget(const LibAoaRobotMsg & msg) const;
  std::optional<geometry_msgs::msg::PointStamped> transformPoint(
    const geometry_msgs::msg::PointStamped & point,
    const std::string & target_frame);

  void requestGlobalPath(
    const geometry_msgs::msg::Point & goal_base,
    double goal_yaw_base,
    const rclcpp::Time & stamp);
  std::optional<geometry_msgs::msg::PoseStamped> transformPoseToOdom(
    const geometry_msgs::msg::PoseStamped & pose);

  void sendFollowPathGoal(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::Point & goal_base,
    double goal_yaw_base);
  bool shouldRequestPath(
    const geometry_msgs::msg::Point & goal_base,
    double goal_yaw_base) const;
  bool plannerRequestTimedOut(const rclcpp::Time & stamp) const;
  void cancelActivePlannerGoal();
  void cancelActiveFollowGoal();
  void stopFollowing(const std::string & status, const rclcpp::Time & stamp);
  void clearTargetAndStop(const std::string & status, const rclcpp::Time & stamp);

  void publishTarget(const geometry_msgs::msg::PointStamped & target);
  void publishTargetValid(bool valid);
  void publishEmptyPath(const rclcpp::Time & stamp);
  void publishZeroVelocity();
  void publishStatus(const std::string & status);

  static geometry_msgs::msg::Point makePoint(double x, double y);
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
  static bool finitePoint(const geometry_msgs::msg::Point & point);

  UwbTebFollowConfig config_;
  bool have_target_{false};
  bool planner_request_in_flight_{false};
  geometry_msgs::msg::Point latest_target_base_;
  geometry_msgs::msg::Point latest_target_odom_;
  rclcpp::Time latest_target_stamp_;
  rclcpp::Time planner_request_stamp_;
  std::uint64_t planner_request_id_{0};
  std::optional<geometry_msgs::msg::Point> last_sent_goal_base_;
  double last_sent_goal_yaw_base_{0.0};
  std::string last_status_;

  rclcpp::Subscription<LibAoaRobotMsg>::SharedPtr uwb_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr stop_cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr planner_client_;
  GoalHandleComputePathToPose::SharedPtr active_planner_goal_handle_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;
  GoalHandleFollowPath::SharedPtr active_follow_goal_handle_;
};

}  // namespace go2_uwb_teb_follow
