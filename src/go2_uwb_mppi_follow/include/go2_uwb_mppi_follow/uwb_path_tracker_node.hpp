#pragma once

#include <deque>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "uwb_aoa_pkg/msg/lib_aoa_robot_msg.hpp"

namespace go2_uwb_mppi_follow
{

struct TargetSample
{
  rclcpp::Time stamp;
  geometry_msgs::msg::Point point;
};

struct TrackerConfig
{
  std::string uwb_topic;
  std::string odom_frame;
  std::string base_frame;
  std::string uwb_frame;
  std::string target_path_topic;
  std::string follow_path_topic;
  std::string target_valid_topic;
  std::string status_topic;
  std::string follow_path_action;
  std::string controller_id;
  std::string goal_checker_id;
  double follow_distance_m;
  double history_length_m;
  double min_sample_spacing_m;
  double max_target_jump_m;
  double max_target_speed_mps;
  double min_target_distance_m;
  double max_target_distance_m;
  double target_timeout_sec;
  double publish_rate_hz;
  double action_resend_period_sec;
  int min_path_poses;
  bool use_tf_for_uwb;
};

class UwbPathTrackerNode : public rclcpp::Node
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;
  using LibAoaRobotMsg = uwb_aoa_pkg::msg::LibAoaRobotMsg;

  // 构造 UWB 路径跟随节点，并初始化 ROS 通信接口。
  UwbPathTrackerNode();

private:
  // 声明节点使用的所有可调 ROS 参数。
  void declareParameters();

  // 读取 ROS 参数并缓存到跟随配置中。
  void loadParameters();

  // 接收原始 UWB 数据，并在有效时加入目标历史路径。
  void uwbCallback(const LibAoaRobotMsg::SharedPtr msg);

  // 定时发布路径，并刷新 Nav2 FollowPath 目标。
  void timerCallback();

  // 将 UWB 消息解析为 UWB 坐标系下的目标点。
  std::optional<geometry_msgs::msg::Point> parseUwbTarget(const LibAoaRobotMsg & msg) const;

  // 将目标点从来源坐标系转换到 odom 坐标系。
  std::optional<geometry_msgs::msg::Point> targetToOdom(
    const geometry_msgs::msg::Point & target,
    const rclcpp::Time & stamp,
    const std::string & source_frame);

  // 获取机器人 base 坐标原点在 odom 坐标系下的位置。
  std::optional<geometry_msgs::msg::Point> robotPointInOdom(const rclcpp::Time & stamp);

  // 检查目标点的距离、速度和跳变是否满足接收条件。
  bool isTargetSampleValid(const geometry_msgs::msg::Point & point, const rclcpp::Time & stamp);

  // 追加有效目标点，并保持历史路径长度在短时范围内。
  void appendTargetSample(const geometry_msgs::msg::Point & point, const rclcpp::Time & stamp);

  // 删除过旧或距离当前目标过远的历史点。
  void pruneTargetHistory(const rclcpp::Time & now);

  // 生成 MPPI 需要跟踪的短时滞后参考路径。
  std::optional<nav_msgs::msg::Path> buildFollowPath(const rclcpp::Time & now);

  // 生成用于可视化的 UWB 目标历史路径。
  nav_msgs::msg::Path buildTargetHistoryPath(const rclcpp::Time & now) const;

  // 发布当前目标历史路径是否可用。
  void publishTargetValid(bool valid);

  // 在节点状态变化时发布简短状态文本。
  void publishStatus(const std::string & status);

  // 将最新参考路径发送给 Nav2 FollowPath action 服务。
  void sendFollowPathGoal(const nav_msgs::msg::Path & path);

  // 目标跟踪无效时取消当前 FollowPath 目标。
  void cancelActiveGoal();

  // 判断是否已经到达重新发送 FollowPath 目标的时间。
  bool shouldResendActionGoal(const rclcpp::Time & now) const;

  // 根据当前点和下一个点生成带朝向的路径位姿。
  geometry_msgs::msg::PoseStamped makePose(
    const geometry_msgs::msg::Point & point,
    const std::optional<geometry_msgs::msg::Point> & next_point,
    const rclcpp::Time & stamp) const;

  // 计算两个几何点之间的平面距离。
  static double distance2d(
    const geometry_msgs::msg::Point & lhs,
    const geometry_msgs::msg::Point & rhs);

  TrackerConfig config_;
  std::deque<TargetSample> target_history_;
  std::string last_status_;
  rclcpp::Time last_goal_send_time_;
  rclcpp::Time last_valid_target_time_;

  rclcpp::Subscription<LibAoaRobotMsg>::SharedPtr uwb_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr target_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr follow_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;
  GoalHandleFollowPath::SharedPtr active_goal_handle_;
};

}  // 命名空间 go2_uwb_mppi_follow
