#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "uwb_aoa_pkg/msg/lib_aoa_robot_msg.hpp"

#include "go2_stereo_apf_follow/apf_core.hpp"

namespace
{

geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

go2_stereo_apf_follow::Point2D transform_point(
  const go2_stereo_apf_follow::Point2D & point,
  const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & rotation = transform.transform.rotation;
  tf2::Quaternion q(rotation.x, rotation.y, rotation.z, rotation.w);
  tf2::Matrix3x3 matrix(q);
  tf2::Vector3 v(point.x, point.y, 0.0);
  tf2::Vector3 out = matrix * v;
  out += tf2::Vector3(
    transform.transform.translation.x,
    transform.transform.translation.y,
    transform.transform.translation.z);
  return go2_stereo_apf_follow::Point2D{out.x(), out.y()};
}

}  // namespace

class UwbTargetSeedNode : public rclcpp::Node
{
public:
  UwbTargetSeedNode()
  : Node("uwb_target_seed_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    one1000_topic_ = declare_parameter<std::string>("one1000_topic", "/libAoa_robot_publisher");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    default_one1000_frame_ = declare_parameter<std::string>("one1000_frame", "uwb_link");
    seed_target_topic_ = declare_parameter<std::string>("seed_target_topic", "/stereo_apf/seed_target");
    seed_valid_topic_ = declare_parameter<std::string>("seed_valid_topic", "/stereo_apf/seed_valid");
    use_tf_ = declare_parameter<bool>("use_tf", true);
    require_tf_ = declare_parameter<bool>("require_tf", true);
    disable_quality_gating_ = declare_parameter<bool>("disable_quality_gating", true);
    confidence_threshold_ = declare_parameter<double>("confidence_threshold", 0.0);
    target_timeout_sec_ = declare_parameter<double>("target_timeout_sec", 3.0);
    status_publish_period_sec_ = declare_parameter<double>("status_publish_period_sec", 0.5);

    uwb_config_.prefer_xy = declare_parameter<bool>("prefer_xy", true);
    uwb_config_.angle_in_degrees = declare_parameter<bool>("angle_in_degrees", false);
    uwb_config_.invert_y = declare_parameter<bool>("invert_y", false);
    uwb_config_.angle_offset_rad = declare_parameter<double>("angle_offset_rad", 0.0);
    uwb_config_.anchor_x_offset = declare_parameter<double>("anchor_x_offset", 0.0);
    uwb_config_.anchor_y_offset = declare_parameter<double>("anchor_y_offset", 0.0);
    uwb_config_.min_distance_m = declare_parameter<double>("min_target_distance", 0.15);
    uwb_config_.max_distance_m = declare_parameter<double>("max_target_distance", 8.0);

    seed_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(seed_target_topic_, 10);
    valid_pub_ = create_publisher<std_msgs::msg::Bool>(seed_valid_topic_, 10);

    sub_ = create_subscription<uwb_aoa_pkg::msg::LibAoaRobotMsg>(
      one1000_topic_,
      10,
      std::bind(&UwbTargetSeedNode::target_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&UwbTargetSeedNode::watchdog_tick, this));

    RCLCPP_INFO(get_logger(), "uwb_target_seed_node started");
  }

private:
  void target_callback(const uwb_aoa_pkg::msg::LibAoaRobotMsg::SharedPtr msg)
  {
    const bool quality_ok = disable_quality_gating_ ||
      (msg->state >= 0 && static_cast<double>(msg->pos_confidence) >= confidence_threshold_);
    if (!quality_ok) {
      publish_valid(false);
      return;
    }

    auto parsed = go2_stereo_apf_follow::parse_uwb_target(
      msg->x,
      msg->y,
      msg->r,
      msg->a,
      uwb_config_);
    if (!parsed.has_value()) {
      publish_valid(false);
      return;
    }

    auto target = parsed.value();
    const std::string source_frame =
      msg->header.frame_id.empty() ? default_one1000_frame_ : msg->header.frame_id;
    if (use_tf_ && source_frame != base_frame_) {
      try {
        const auto tf = tf_buffer_.lookupTransform(base_frame_, source_frame, tf2::TimePointZero);
        target = transform_point(target, tf);
      } catch (const std::exception & exc) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "UWB TF failed from %s to %s: %s",
          source_frame.c_str(),
          base_frame_.c_str(),
          exc.what());
        if (require_tf_) {
          publish_valid(false);
          return;
        }
      }
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = base_frame_;
    pose.pose.position.x = target.x;
    pose.pose.position.y = target.y;
    pose.pose.orientation = yaw_to_quaternion(std::atan2(target.y, target.x));
    seed_pub_->publish(pose);
    last_target_time_ = now();
    have_recent_target_ = true;
    publish_valid(true);
  }

  void watchdog_tick()
  {
    if (!have_recent_target_) {
      return;
    }
    const double age = (now() - last_target_time_).seconds();
    if (age > target_timeout_sec_) {
      have_recent_target_ = false;
      publish_valid(false);
    }
  }

  void publish_valid(bool valid)
  {
    const auto current = now();
    const bool should_repeat = (current - last_valid_publish_time_).seconds() >= status_publish_period_sec_;
    if (valid == last_valid_ && !should_repeat) {
      return;
    }
    std_msgs::msg::Bool msg;
    msg.data = valid;
    valid_pub_->publish(msg);
    last_valid_ = valid;
    last_valid_publish_time_ = current;
  }

  std::string one1000_topic_;
  std::string base_frame_;
  std::string default_one1000_frame_;
  std::string seed_target_topic_;
  std::string seed_valid_topic_;
  bool use_tf_{true};
  bool require_tf_{true};
  bool disable_quality_gating_{true};
  double confidence_threshold_{0.0};
  double target_timeout_sec_{3.0};
  double status_publish_period_sec_{0.5};
  go2_stereo_apf_follow::UwbTargetConfig uwb_config_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<uwb_aoa_pkg::msg::LibAoaRobotMsg>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr seed_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_target_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_valid_publish_time_{0, 0, RCL_ROS_TIME};
  bool have_recent_target_{false};
  bool last_valid_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UwbTargetSeedNode>());
  rclcpp::shutdown();
  return 0;
}
