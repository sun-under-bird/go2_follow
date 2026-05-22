#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "go2_stereo_apf_follow/apf_core.hpp"

namespace
{

go2_stereo_apf_follow::Point3D transform_point(
  const go2_stereo_apf_follow::Point3D & point,
  const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & rotation = transform.transform.rotation;
  tf2::Quaternion q(rotation.x, rotation.y, rotation.z, rotation.w);
  tf2::Matrix3x3 matrix(q);
  tf2::Vector3 v(point.x, point.y, point.z);
  tf2::Vector3 out = matrix * v;
  out += tf2::Vector3(
    transform.transform.translation.x,
    transform.transform.translation.y,
    transform.transform.translation.z);
  return go2_stereo_apf_follow::Point3D{out.x(), out.y(), out.z()};
}

go2_stereo_apf_follow::TwistCommand from_twist(const geometry_msgs::msg::Twist & msg)
{
  return go2_stereo_apf_follow::TwistCommand{
    msg.linear.x,
    msg.linear.y,
    msg.angular.z};
}

geometry_msgs::msg::Twist to_twist(const go2_stereo_apf_follow::TwistCommand & cmd)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = cmd.vx;
  msg.linear.y = cmd.vy;
  msg.angular.z = cmd.wz;
  return msg;
}

}  // namespace

class ApfSafetyMuxNode : public rclcpp::Node
{
public:
  ApfSafetyMuxNode()
  : Node("apf_safety_mux_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    cmd_vel_in_topic_ = declare_parameter<std::string>("cmd_vel_in", "/cmd_vel_apf");
    cmd_vel_out_topic_ = declare_parameter<std::string>("cmd_vel_out", "/cmd_vel_safe");
    pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic", "/local_grid_obstacle");
    target_topic_ = declare_parameter<std::string>("target_topic", "/stereo_apf/target");
    status_topic_ = declare_parameter<std::string>("status_topic", "/stereo_apf/safety_status");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
    cmd_timeout_sec_ = declare_parameter<double>("cmd_timeout_sec", 0.3);
    target_timeout_sec_ = declare_parameter<double>("target_timeout_sec", 0.5);
    pointcloud_timeout_sec_ = declare_parameter<double>("pointcloud_timeout_sec", 0.5);
    emergency_x_max_ = declare_parameter<double>("emergency_x_max", 0.45);
    obstacle_clear_x_max_ = declare_parameter<double>("obstacle_clear_x_max", 0.60);
    slow_x_max_ = declare_parameter<double>("slow_x_max", 1.0);
    front_y_abs_ = declare_parameter<double>("front_y_abs", 0.45);
    obstacle_z_min_ = declare_parameter<double>("obstacle_z_min", 0.05);
    obstacle_z_max_ = declare_parameter<double>("obstacle_z_max", 1.2);
    max_points_per_cloud_ = declare_parameter<int>("max_points_per_cloud", 60000);

    filter_config_.robot_frame_front = declare_parameter<double>("robot_frame_front", 0.35);
    filter_config_.robot_frame_back = declare_parameter<double>("robot_frame_back", 0.35);
    filter_config_.robot_frame_left = declare_parameter<double>("robot_frame_left", 0.20);
    filter_config_.robot_frame_right = declare_parameter<double>("robot_frame_right", 0.20);

    limits_.max_vx = declare_parameter<double>("max_vx", 0.30);
    limits_.max_vy = declare_parameter<double>("max_vy", 0.20);
    limits_.max_wz = declare_parameter<double>("max_vyaw", 0.80);
    limits_.max_reverse_vx = declare_parameter<double>("max_reverse_vx", 0.0);
    limits_.max_accel_vx = declare_parameter<double>("max_accel_vx", 0.6);
    limits_.max_accel_vy = declare_parameter<double>("max_accel_vy", 0.8);
    limits_.max_accel_wz = declare_parameter<double>("max_accel_vyaw", 1.5);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_in_topic_,
      10,
      std::bind(&ApfSafetyMuxNode::cmd_callback, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_,
      5,
      std::bind(&ApfSafetyMuxNode::cloud_callback, this, std::placeholders::_1));
    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      target_topic_,
      10,
      std::bind(&ApfSafetyMuxNode::target_callback, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_out_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

    last_tick_time_ = now();
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ApfSafetyMuxNode::tick, this));

    RCLCPP_INFO(get_logger(), "apf_safety_mux_node started");
  }

private:
  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_cmd_ = from_twist(*msg);
    latest_cmd_time_ = now();
    have_cmd_ = true;
  }

  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr)
  {
    latest_target_time_ = now();
    have_target_ = true;
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped tf;
    bool have_tf = false;
    if (!msg->header.frame_id.empty() && msg->header.frame_id != base_frame_) {
      try {
        tf = tf_buffer_.lookupTransform(base_frame_, msg->header.frame_id, tf2::TimePointZero);
        have_tf = true;
      } catch (const std::exception & exc) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Safety PointCloud TF failed from %s to %s: %s",
          msg->header.frame_id.c_str(),
          base_frame_.c_str(),
          exc.what());
        return;
      }
    }

    bool found = false;
    double nearest = 0.0;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    int seen = 0;
    for (; iter_x != iter_x.end() && seen < max_points_per_cloud_; ++iter_x, ++iter_y, ++iter_z, ++seen) {
      go2_stereo_apf_follow::Point3D point{
        static_cast<double>(*iter_x),
        static_cast<double>(*iter_y),
        static_cast<double>(*iter_z)};
      if (have_tf) {
        point = transform_point(point, tf);
      }
      if (!go2_stereo_apf_follow::is_finite(point) || go2_stereo_apf_follow::in_robot_frame(point, filter_config_)) {
        continue;
      }
      if (point.x < 0.0 || point.x > slow_x_max_) {
        continue;
      }
      if (std::abs(point.y) > front_y_abs_) {
        continue;
      }
      if (point.z < obstacle_z_min_ || point.z > obstacle_z_max_) {
        continue;
      }
      if (!found || point.x < nearest) {
        nearest = point.x;
        found = true;
      }
      if (nearest <= emergency_x_max_) {
        break;
      }
    }

    if (found && nearest <= emergency_x_max_) {
      obstacle_stop_latched_ = true;
    } else if (obstacle_stop_latched_ && (!found || nearest >= obstacle_clear_x_max_)) {
      obstacle_stop_latched_ = false;
    }

    nearest_front_obstacle_ = nearest;
    have_nearest_front_obstacle_ = found;
    latest_cloud_time_ = now();
    have_cloud_ = true;
  }

  bool age_ok(const rclcpp::Time & stamp, double timeout_sec) const
  {
    if (stamp.nanoseconds() == 0) {
      return false;
    }
    return (now() - stamp).seconds() <= timeout_sec;
  }

  void tick()
  {
    const auto current_time = now();
    const double dt = std::max(1e-3, (current_time - last_tick_time_).seconds());
    last_tick_time_ = current_time;

    go2_stereo_apf_follow::TwistCommand output;
    std::string status = "ok";

    if (!have_target_ || !age_ok(latest_target_time_, target_timeout_sec_)) {
      status = "stop: target stale";
      obstacle_stop_latched_ = false;
    } else if (!have_cloud_ || !age_ok(latest_cloud_time_, pointcloud_timeout_sec_)) {
      status = "stop: obstacle cloud stale";
      obstacle_stop_latched_ = false;
    } else if (!have_cmd_ || !age_ok(latest_cmd_time_, cmd_timeout_sec_)) {
      status = "stop: cmd stale";
    } else if (obstacle_stop_latched_) {
      if (have_nearest_front_obstacle_) {
        status = "stop: obstacle at " + std::to_string(nearest_front_obstacle_) + "m";
      } else {
        status = "stop: obstacle latch";
      }
    } else {
      auto target = go2_stereo_apf_follow::clip_command(latest_cmd_, limits_);
      if (
        have_nearest_front_obstacle_ && nearest_front_obstacle_ > emergency_x_max_ &&
        nearest_front_obstacle_ <= slow_x_max_ && target.vx > 0.0)
      {
        const double scale =
          (nearest_front_obstacle_ - emergency_x_max_) /
          std::max(0.01, slow_x_max_ - emergency_x_max_);
        target.vx *= go2_stereo_apf_follow::clamp(scale, 0.0, 1.0);
        status = "slow: obstacle at " + std::to_string(nearest_front_obstacle_) + "m";
      }
      output = go2_stereo_apf_follow::ramp_command(last_output_, target, dt, limits_);
    }

    if (status.rfind("stop:", 0) == 0) {
      output = go2_stereo_apf_follow::TwistCommand{};
    }
    last_output_ = output;
    cmd_pub_->publish(to_twist(output));
    publish_status(status);
  }

  void publish_status(const std::string & status)
  {
    if (status == last_status_) {
      return;
    }
    last_status_ = status;
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "%s", status.c_str());
  }

  std::string base_frame_;
  std::string cmd_vel_in_topic_;
  std::string cmd_vel_out_topic_;
  std::string pointcloud_topic_;
  std::string target_topic_;
  std::string status_topic_;
  double publish_rate_hz_{30.0};
  double cmd_timeout_sec_{0.3};
  double target_timeout_sec_{0.5};
  double pointcloud_timeout_sec_{0.5};
  double emergency_x_max_{0.45};
  double obstacle_clear_x_max_{0.60};
  double slow_x_max_{1.0};
  double front_y_abs_{0.45};
  double obstacle_z_min_{0.05};
  double obstacle_z_max_{1.2};
  int max_points_per_cloud_{60000};
  go2_stereo_apf_follow::PointFilterConfig filter_config_;
  go2_stereo_apf_follow::SafetyLimitConfig limits_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  go2_stereo_apf_follow::TwistCommand latest_cmd_;
  go2_stereo_apf_follow::TwistCommand last_output_;
  rclcpp::Time latest_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_cloud_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_target_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tick_time_{0, 0, RCL_ROS_TIME};
  bool have_cmd_{false};
  bool have_cloud_{false};
  bool have_target_{false};
  bool obstacle_stop_latched_{false};
  double nearest_front_obstacle_{0.0};
  bool have_nearest_front_obstacle_{false};
  std::string last_status_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApfSafetyMuxNode>());
  rclcpp::shutdown();
  return 0;
}
