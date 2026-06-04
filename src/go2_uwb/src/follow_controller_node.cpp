#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"


class FollowControllerNode : public rclcpp::Node
{
public:
  FollowControllerNode()
  : Node("follow_controller_node")
  {
    const auto target_topic = declare_parameter<std::string>("target_topic", "/uwb/target_point");
    const auto obstacle_distance_topic = declare_parameter<std::string>(
      "obstacle_distance_topic", "/obstacle/nearest_distance");
    const auto avoid_vector_topic = declare_parameter<std::string>(
      "avoid_vector_topic", "/obstacle/avoid_vector");
    const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto status_topic = declare_parameter<std::string>(
      "status_topic", "/go2_uwb/controller_status");

    control_rate_ = declare_parameter<double>("control_rate", 20.0);
    status_rate_ = declare_parameter<double>("status_rate", 2.0);
    target_distance_ = declare_parameter<double>("target_distance", 1.5);
    target_deadband_ = declare_parameter<double>("target_deadband", 0.12);
    angle_deadband_ = declare_parameter<double>("angle_deadband", 0.08);
    max_target_jump_ = declare_parameter<double>("max_target_jump", 0.7);
    avoid_distance_ = declare_parameter<double>("avoid_distance", 0.9);
    avoid_release_distance_ = declare_parameter<double>("avoid_release_distance", 1.05);
    front_stop_distance_ = declare_parameter<double>("front_stop_distance", 0.45);
    max_linear_ = declare_parameter<double>("max_linear", 0.5);
    max_angular_ = declare_parameter<double>("max_angular", 0.8);
    max_linear_accel_ = declare_parameter<double>("max_linear_accel", 0.4);
    max_angular_accel_ = declare_parameter<double>("max_angular_accel", 1.2);
    linear_k_ = declare_parameter<double>("linear_k", 0.4);
    angular_k_ = declare_parameter<double>("angular_k", 1.0);
    uwb_timeout_ = declare_parameter<double>("uwb_timeout", 1.0);
    obstacle_timeout_ = declare_parameter<double>("obstacle_timeout", 0.7);
    target_filter_alpha_ = declare_parameter<double>("target_filter_alpha", 0.35);
    avoid_angular_filter_alpha_ = declare_parameter<double>("avoid_angular_filter_alpha", 0.4);
    turn_slowdown_angle_ = declare_parameter<double>("turn_slowdown_angle", 0.8);
    min_turn_slowdown_ = declare_parameter<double>("min_turn_slowdown", 0.35);

    sanitizeParameters();

    cmd_publisher_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic, 10);
    target_subscription_ = create_subscription<geometry_msgs::msg::PointStamped>(
      target_topic,
      10,
      std::bind(&FollowControllerNode::targetCallback, this, std::placeholders::_1));
    distance_subscription_ = create_subscription<std_msgs::msg::Float32>(
      obstacle_distance_topic,
      10,
      std::bind(&FollowControllerNode::obstacleDistanceCallback, this, std::placeholders::_1));
    avoid_subscription_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      avoid_vector_topic,
      10,
      std::bind(&FollowControllerNode::avoidVectorCallback, this, std::placeholders::_1));

    const double safe_rate = std::max(1.0, control_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / safe_rate),
      std::bind(&FollowControllerNode::controlLoop, this));
    status_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(0.2, status_rate_)),
      std::bind(&FollowControllerNode::publishStatus, this));

    RCLCPP_INFO(
      get_logger(),
      "跟随控制已启动：UWB %s，输出 %s",
      target_topic.c_str(),
      cmd_vel_topic.c_str());
  }

private:
  // 统一修正异常参数，避免启动时因为配置错误产生危险速度。
  void sanitizeParameters()
  {
    control_rate_ = std::max(1.0, control_rate_);
    status_rate_ = std::max(0.2, status_rate_);
    target_distance_ = std::max(0.1, target_distance_);
    target_deadband_ = std::max(0.0, target_deadband_);
    angle_deadband_ = std::max(0.0, angle_deadband_);
    max_target_jump_ = std::max(0.0, max_target_jump_);
    front_stop_distance_ = std::max(0.05, front_stop_distance_);
    avoid_distance_ = std::max(front_stop_distance_ + 0.05, avoid_distance_);
    avoid_release_distance_ = std::max(avoid_distance_ + 0.05, avoid_release_distance_);
    max_linear_ = std::max(0.0, max_linear_);
    max_angular_ = std::max(0.0, max_angular_);
    max_linear_accel_ = std::max(0.01, max_linear_accel_);
    max_angular_accel_ = std::max(0.01, max_angular_accel_);
    linear_k_ = std::max(0.0, linear_k_);
    angular_k_ = std::max(0.0, angular_k_);
    uwb_timeout_ = std::max(0.1, uwb_timeout_);
    obstacle_timeout_ = std::max(0.1, obstacle_timeout_);
    target_filter_alpha_ = clamp(target_filter_alpha_, 0.0, 1.0);
    avoid_angular_filter_alpha_ = clamp(avoid_angular_filter_alpha_, 0.0, 1.0);
    turn_slowdown_angle_ = std::max(0.05, turn_slowdown_angle_);
    min_turn_slowdown_ = clamp(min_turn_slowdown_, 0.0, 1.0);
  }

  // 保存并低通滤波最近一次有效 UWB 主人坐标，降低 UWB 抖动对速度的影响。
  void targetCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (!std::isfinite(msg->point.x) || !std::isfinite(msg->point.y)) {
      // UWB 出现 NaN/inf 时忽略该帧，避免异常坐标进入控制器。
      ++invalid_target_count_;
      return;
    }

    const auto constrained_target = constrainTargetJump(*msg);

    if (!has_filtered_target_) {
      filtered_target_ = constrained_target;
      has_filtered_target_ = true;
    } else {
      filtered_target_.header = constrained_target.header;
      filtered_target_.point.x = lowPass(
        filtered_target_.point.x,
        constrained_target.point.x,
        target_filter_alpha_);
      filtered_target_.point.y = lowPass(
        filtered_target_.point.y,
        constrained_target.point.y,
        target_filter_alpha_);
      filtered_target_.point.z = 0.0;
    }

    has_target_ = true;
    last_target_time_ = now();
  }

  // 保存最近一次前方障碍距离。
  void obstacleDistanceCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    last_obstacle_distance_ = static_cast<double>(msg->data);
    has_obstacle_data_ = true;
    last_obstacle_time_ = now();
  }

  // 对避障角速度建议做低通滤波，减少左右绕行方向频繁跳变。
  void avoidVectorCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    const double raw_avoid = clamp(msg->vector.z, -max_angular_, max_angular_);
    last_avoid_angular_ = lowPass(
      last_avoid_angular_,
      raw_avoid,
      avoid_angular_filter_alpha_);
  }

  // 周期性计算跟随速度，并在目标或障碍信息超时时立即停车。
  void controlLoop()
  {
    const auto current_time = now();

    if (isTargetLost(current_time)) {
      controller_state_ = "target_lost";
      publishStop();
      return;
    }

    auto cmd = buildFollowCommand();
    const bool needs_follow_motion = shouldUseObstacleAvoidance();

    if (needs_follow_motion) {
      if (isObstacleDataLost(current_time)) {
        controller_state_ = "obstacle_timeout";
        publishStop();
        return;
      }
      cmd = applyObstaclePolicy(cmd);
    } else {
      // 跟随距离以内只按 UWB 方位调整朝向，不让障碍物触发绕行或停车。
      avoidance_active_ = false;
      controller_state_ = "hold_direction";
    }

    cmd = limitAcceleration(cmd, current_time);
    if (needs_follow_motion) {
      controller_state_ = avoidance_active_ ? "avoid" : "follow";
      if (last_obstacle_distance_ <= front_stop_distance_) {
        controller_state_ = "front_stop";
      }
    }

    last_command_ = cmd;
    last_command_time_ = current_time;
    has_last_command_ = true;
    cmd_publisher_->publish(cmd);
  }

  // 判断 UWB 有效目标是否已经超时。
  bool isTargetLost(const rclcpp::Time & current_time) const
  {
    if (!has_target_ || !has_filtered_target_) {
      return true;
    }
    return (current_time - last_target_time_).seconds() > uwb_timeout_;
  }

  // 判断点云避障结果是否已经超时。
  bool isObstacleDataLost(const rclcpp::Time & current_time) const
  {
    if (!has_obstacle_data_) {
      return true;
    }
    return (current_time - last_obstacle_time_).seconds() > obstacle_timeout_;
  }

  // 限制 UWB 单帧跳变距离，避免偶发跳点让速度突然变化。
  geometry_msgs::msg::PointStamped constrainTargetJump(
    const geometry_msgs::msg::PointStamped & raw_target)
  {
    if (!has_filtered_target_ || max_target_jump_ <= 0.0) {
      return raw_target;
    }

    const double dx = raw_target.point.x - filtered_target_.point.x;
    const double dy = raw_target.point.y - filtered_target_.point.y;
    const double jump = std::hypot(dx, dy);
    if (jump <= max_target_jump_) {
      return raw_target;
    }

    geometry_msgs::msg::PointStamped limited = raw_target;
    limited.point.x = filtered_target_.point.x + dx / jump * max_target_jump_;
    limited.point.y = filtered_target_.point.y + dy / jump * max_target_jump_;
    limited.point.z = 0.0;
    ++limited_target_jump_count_;
    return limited;
  }

  // 根据 UWB 主人坐标计算边走边转的基础跟随速度。
  geometry_msgs::msg::Twist buildFollowCommand() const
  {
    const double x = filtered_target_.point.x;
    const double y = filtered_target_.point.y;
    const double distance = std::hypot(x, y);
    const double angle = std::atan2(y, x);

    geometry_msgs::msg::Twist cmd;
    const double distance_error = std::max(0.0, distance - target_distance_);
    if (distance_error > target_deadband_) {
      cmd.linear.x = clamp(linear_k_ * distance_error, 0.0, max_linear_);
    }

    if (std::abs(angle) > angle_deadband_) {
      cmd.angular.z = clamp(angular_k_ * angle, -max_angular_, max_angular_);
    }

    // 转向角度较大时主动降低前进速度，避免边大角度转向边快速前冲。
    const double turn_slowdown = clamp(
      1.0 - std::abs(angle) / turn_slowdown_angle_,
      min_turn_slowdown_,
      1.0);
    cmd.linear.x *= turn_slowdown;
    return cmd;
  }

  // 只有主人距离超过跟随距离和死区后，才认为需要前进，并启用点云避障。
  bool shouldUseObstacleAvoidance() const
  {
    return targetDistance() > target_distance_ + target_deadband_;
  }

  // 获取滤波后的 UWB 目标距离。
  double targetDistance() const
  {
    return std::hypot(filtered_target_.point.x, filtered_target_.point.y);
  }

  // 根据障碍距离叠加减速、强停和绕行动作。
  geometry_msgs::msg::Twist applyObstaclePolicy(geometry_msgs::msg::Twist cmd)
  {
    updateAvoidanceState();

    if (!avoidance_active_) {
      return cmd;
    }

    if (last_obstacle_distance_ <= front_stop_distance_) {
      // 障碍过近时禁止继续前进，只保留受限转向以尝试寻找空隙。
      cmd.linear.x = 0.0;
      cmd.angular.z = clamp(last_avoid_angular_, -max_angular_, max_angular_);
      return cmd;
    }

    // 障碍越近，线速度越低，避障角速度越强；距离恢复后由迟滞逻辑退出避障。
    const double denominator = std::max(0.001, avoid_distance_ - front_stop_distance_);
    const double slow_ratio = clamp(
      (last_obstacle_distance_ - front_stop_distance_) / denominator,
      0.0,
      1.0);
    const double avoid_scale = 1.0 - slow_ratio;

    cmd.linear.x *= slow_ratio;
    cmd.angular.z = clamp(
      cmd.angular.z + last_avoid_angular_ * avoid_scale,
      -max_angular_,
      max_angular_);
    return cmd;
  }

  // 使用进入/退出距离迟滞，避免障碍距离在阈值附近抖动时反复切换状态。
  void updateAvoidanceState()
  {
    if (!std::isfinite(last_obstacle_distance_)) {
      avoidance_active_ = false;
      return;
    }
    if (last_obstacle_distance_ <= avoid_distance_) {
      avoidance_active_ = true;
      return;
    }
    if (last_obstacle_distance_ >= avoid_release_distance_) {
      avoidance_active_ = false;
    }
  }

  // 周期性发布控制器状态，方便现场判断停车或避障原因。
  void publishStatus()
  {
    std_msgs::msg::String msg;
    std::ostringstream stream;
    stream << "state=" << controller_state_
      << ", target_age=" << ageSeconds(last_target_time_, has_target_)
      << ", target_distance=" << (has_filtered_target_ ? targetDistance() : -1.0)
      << ", obstacle_age=" << ageSeconds(last_obstacle_time_, has_obstacle_data_)
      << ", obstacle_distance=" << last_obstacle_distance_
      << ", avoid_active=" << (avoidance_active_ ? 1 : 0)
      << ", invalid_target_count=" << invalid_target_count_
      << ", limited_target_jump_count=" << limited_target_jump_count_
      << ", cmd_linear=" << last_command_.linear.x
      << ", cmd_angular=" << last_command_.angular.z;
    msg.data = stream.str();
    status_publisher_->publish(msg);
  }

  // 计算某类输入距当前时刻的时间，未收到过数据时返回 -1。
  double ageSeconds(const rclcpp::Time & stamp, bool valid) const
  {
    if (!valid) {
      return -1.0;
    }
    return (now() - stamp).seconds();
  }

  // 限制速度变化率，让发给 go2_twist_bridge 的 /cmd_vel 不出现突变。
  geometry_msgs::msg::Twist limitAcceleration(
    const geometry_msgs::msg::Twist & target_cmd,
    const rclcpp::Time & current_time) const
  {
    if (!has_last_command_) {
      return target_cmd;
    }

    const double dt = clamp(
      (current_time - last_command_time_).seconds(),
      0.001,
      1.0 / std::max(1.0, control_rate_));

    geometry_msgs::msg::Twist limited = target_cmd;
    limited.linear.x = limitStep(
      last_command_.linear.x,
      target_cmd.linear.x,
      max_linear_accel_ * dt);
    limited.angular.z = limitStep(
      last_command_.angular.z,
      target_cmd.angular.z,
      max_angular_accel_ * dt);
    return limited;
  }

  // 发布零速度，并把斜率限制的历史速度也清零。
  void publishStop()
  {
    geometry_msgs::msg::Twist stop;
    last_command_ = stop;
    last_command_time_ = now();
    has_last_command_ = true;
    cmd_publisher_->publish(stop);
  }

  // 一阶低通滤波。
  static double lowPass(double previous, double current, double alpha)
  {
    return previous * (1.0 - alpha) + current * alpha;
  }

  // 单周期限幅，用于限制速度斜率。
  static double limitStep(double previous, double target, double max_delta)
  {
    return previous + clamp(target - previous, -max_delta, max_delta);
  }

  // 把数值限制在指定范围内。
  static double clamp(double value, double lower, double upper)
  {
    return std::max(lower, std::min(upper, value));
  }

  double control_rate_;
  double status_rate_;
  double target_distance_;
  double target_deadband_;
  double angle_deadband_;
  double max_target_jump_;
  double avoid_distance_;
  double avoid_release_distance_;
  double front_stop_distance_;
  double max_linear_;
  double max_angular_;
  double max_linear_accel_;
  double max_angular_accel_;
  double linear_k_;
  double angular_k_;
  double uwb_timeout_;
  double obstacle_timeout_;
  double target_filter_alpha_;
  double avoid_angular_filter_alpha_;
  double turn_slowdown_angle_;
  double min_turn_slowdown_;
  bool has_target_{false};
  bool has_filtered_target_{false};
  bool has_obstacle_data_{false};
  bool has_last_command_{false};
  bool avoidance_active_{false};
  std::string controller_state_{"waiting"};
  int invalid_target_count_{0};
  int limited_target_jump_count_{0};
  geometry_msgs::msg::PointStamped filtered_target_;
  rclcpp::Time last_target_time_;
  double last_obstacle_distance_{std::numeric_limits<double>::infinity()};
  rclcpp::Time last_obstacle_time_;
  double last_avoid_angular_{0.0};
  geometry_msgs::msg::Twist last_command_;
  rclcpp::Time last_command_time_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr avoid_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowControllerNode>());
  rclcpp::shutdown();
  return 0;
}
