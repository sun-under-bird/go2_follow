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
  // 初始化目标、障碍输入和直接发布到 /cmd_vel 的跟随控制循环。
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
    avoid_distance_ = declare_parameter<double>("avoid_distance", 0.9);
    avoid_release_distance_ = declare_parameter<double>("avoid_release_distance", 1.05);
    front_stop_distance_ = declare_parameter<double>("front_stop_distance", 0.45);
    min_avoid_angular_scale_ = declare_parameter<double>("min_avoid_angular_scale", 0.0);
    max_linear_ = declare_parameter<double>("max_linear", 0.5);
    max_angular_ = declare_parameter<double>("max_angular", 0.8);
    linear_k_ = declare_parameter<double>("linear_k", 0.4);
    angular_k_ = declare_parameter<double>("angular_k", 1.0);
    turn_slowdown_angle_ = declare_parameter<double>("turn_slowdown_angle", 0.45);
    rotate_in_place_angle_ = declare_parameter<double>("rotate_in_place_angle", 1.0);
    rotate_resume_angle_ = declare_parameter<double>("rotate_resume_angle", 0.55);
    min_turn_linear_scale_ = declare_parameter<double>("min_turn_linear_scale", 0.2);

    sanitizeParameters();

    cmd_publisher_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic, 10);
    target_subscription_ = create_subscription<geometry_msgs::msg::PointStamped>(
      target_topic,
      rclcpp::SensorDataQoS(),
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
  // 统一修正控制参数，保证几何计算和速度限幅有效。
  void sanitizeParameters()
  {
    control_rate_ = std::max(1.0, control_rate_);
    status_rate_ = std::max(0.2, status_rate_);
    target_distance_ = std::max(0.1, target_distance_);
    target_deadband_ = std::max(0.0, target_deadband_);
    angle_deadband_ = std::max(0.0, angle_deadband_);
    front_stop_distance_ = std::max(0.05, front_stop_distance_);
    avoid_distance_ = std::max(front_stop_distance_ + 0.05, avoid_distance_);
    avoid_release_distance_ = std::max(avoid_distance_ + 0.05, avoid_release_distance_);
    min_avoid_angular_scale_ = clamp(min_avoid_angular_scale_, 0.0, 1.0);
    max_linear_ = std::max(0.0, max_linear_);
    max_angular_ = std::max(0.0, max_angular_);
    linear_k_ = std::max(0.0, linear_k_);
    angular_k_ = std::max(0.0, angular_k_);
    turn_slowdown_angle_ = std::max(angle_deadband_ + 0.05, turn_slowdown_angle_);
    rotate_in_place_angle_ = std::max(turn_slowdown_angle_ + 0.05, rotate_in_place_angle_);
    rotate_resume_angle_ = clamp(
      rotate_resume_angle_, angle_deadband_, rotate_in_place_angle_ - 0.05);
    min_turn_linear_scale_ = clamp(min_turn_linear_scale_, 0.0, 1.0);
  }

  // 直接保存最近一次原始 UWB 主人坐标，不做平滑或跳变限制。
  void targetCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (!std::isfinite(msg->point.x) || !std::isfinite(msg->point.y)) {
      // UWB 出现 NaN/inf 时忽略该帧，避免异常坐标进入控制器。
      ++invalid_target_count_;
      return;
    }

    raw_target_ = *msg;
    raw_target_.point.z = 0.0;
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

  // 直接保存当前帧避障角速度建议。
  void avoidVectorCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    last_avoid_angular_ = clamp(msg->vector.z, -max_angular_, max_angular_);
  }

  // 周期性根据最近一次原始目标和当前障碍结果直接发布速度。
  void controlLoop()
  {
    if (!has_target_) {
      controller_state_ = "waiting_target";
      publishStop();
      return;
    }

    updateRotateInPlaceState();
    auto cmd = buildFollowCommand();
    const bool needs_follow_motion = shouldUseObstacleAvoidance();

    if (needs_follow_motion) {
      if (has_obstacle_data_) {
        cmd = applyObstaclePolicy(cmd);
      }
    } else {
      // 跟随距离以内只按 UWB 方位调整朝向，不让障碍物触发绕行或停车。
      avoidance_active_ = false;
      controller_state_ = "hold_direction";
    }

    if (needs_follow_motion) {
      controller_state_ = avoidance_active_ ? "avoid" : "follow";
      if (last_obstacle_distance_ <= front_stop_distance_) {
        controller_state_ = "front_stop";
      }
    }

    last_command_ = cmd;
    cmd_publisher_->publish(cmd);
  }

  // 根据 UWB 主人坐标计算边走边转的基础跟随速度。
  geometry_msgs::msg::Twist buildFollowCommand() const
  {
    const double x = raw_target_.point.x;
    const double y = raw_target_.point.y;
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

    if (rotate_in_place_active_) {
      cmd.linear.x = 0.0;
    } else if (std::abs(angle) > turn_slowdown_angle_) {
      const double turn_ratio = clamp(
        (rotate_in_place_angle_ - std::abs(angle)) /
        (rotate_in_place_angle_ - turn_slowdown_angle_),
        0.0,
        1.0);
      const double linear_scale = min_turn_linear_scale_ +
        (1.0 - min_turn_linear_scale_) * turn_ratio;
      cmd.linear.x *= linear_scale;
    }

    return cmd;
  }

  // 目标转到大角度时先原地转向，回到安全角度后再恢复前进。
  void updateRotateInPlaceState()
  {
    const double angle = std::abs(targetAngle());
    if (rotate_in_place_active_) {
      if (angle <= rotate_resume_angle_) {
        rotate_in_place_active_ = false;
      }
    } else if (angle >= rotate_in_place_angle_) {
      rotate_in_place_active_ = true;
    }
  }

  // 获取最近一次原始 UWB 目标方位角。
  double targetAngle() const
  {
    return std::atan2(raw_target_.point.y, raw_target_.point.x);
  }

  // 只有主人距离超过跟随距离和死区后，才认为需要前进，并启用点云避障。
  bool shouldUseObstacleAvoidance() const
  {
    return targetDistance() > target_distance_ + target_deadband_;
  }

  // 获取最近一次原始 UWB 目标距离。
  double targetDistance() const
  {
    return std::hypot(raw_target_.point.x, raw_target_.point.y);
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
    const double avoid_scale = std::max(min_avoid_angular_scale_, 1.0 - slow_ratio);

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
      << ", target_distance=" << (has_target_ ? targetDistance() : -1.0)
      << ", obstacle_age=" << ageSeconds(last_obstacle_time_, has_obstacle_data_)
      << ", obstacle_distance=" << last_obstacle_distance_
      << ", avoid_active=" << (avoidance_active_ ? 1 : 0)
      << ", rotate_in_place=" << (rotate_in_place_active_ ? 1 : 0)
      << ", invalid_target_count=" << invalid_target_count_
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

  // 发布零速度。
  void publishStop()
  {
    geometry_msgs::msg::Twist stop;
    last_command_ = stop;
    cmd_publisher_->publish(stop);
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
  double avoid_distance_;
  double avoid_release_distance_;
  double front_stop_distance_;
  double min_avoid_angular_scale_;
  double max_linear_;
  double max_angular_;
  double linear_k_;
  double angular_k_;
  double turn_slowdown_angle_;
  double rotate_in_place_angle_;
  double rotate_resume_angle_;
  double min_turn_linear_scale_;
  bool has_target_{false};
  bool has_obstacle_data_{false};
  bool avoidance_active_{false};
  bool rotate_in_place_active_{false};
  std::string controller_state_{"waiting"};
  int invalid_target_count_{0};
  geometry_msgs::msg::PointStamped raw_target_;
  rclcpp::Time last_target_time_;
  double last_obstacle_distance_{std::numeric_limits<double>::infinity()};
  rclcpp::Time last_obstacle_time_;
  double last_avoid_angular_{0.0};
  geometry_msgs::msg::Twist last_command_;
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
