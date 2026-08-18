#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
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
  // 初始化原始 UWB 坐标转换和目标发布。
  UwbTargetSeedNode()
  : Node("uwb_target_seed_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    one1000_topic_ = declare_parameter<std::string>("one1000_topic", "/uwb/target_point");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    default_one1000_frame_ = declare_parameter<std::string>("one1000_frame", "base_footprint");
    seed_target_topic_ = declare_parameter<std::string>("seed_target_topic", "/stereo_apf/seed_target");
    seed_valid_topic_ = declare_parameter<std::string>("seed_valid_topic", "/stereo_apf/seed_valid");
    use_tf_ = declare_parameter<bool>("use_tf", true);
    require_tf_ = declare_parameter<bool>("require_tf", true);

    uwb_config_.prefer_xy = declare_parameter<bool>("prefer_xy", true);
    uwb_config_.angle_in_degrees = declare_parameter<bool>("angle_in_degrees", false);
    uwb_config_.invert_y = declare_parameter<bool>("invert_y", false);
    uwb_config_.angle_offset_rad = declare_parameter<double>("angle_offset_rad", 0.0);
    uwb_config_.anchor_x_offset = declare_parameter<double>("anchor_x_offset", 0.0);
    uwb_config_.anchor_y_offset = declare_parameter<double>("anchor_y_offset", 0.0);

    seed_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(seed_target_topic_, 10);
    seed_valid_pub_ = create_publisher<std_msgs::msg::Bool>(seed_valid_topic_, 10);

    sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      one1000_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&UwbTargetSeedNode::target_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "uwb_target_seed_node started");
  }

private:
  void target_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    // 目标已由通用适配节点完成解析、符号校正与质量门控。
    // The shared adapter already parsed, sign-corrected and gated this target.
    go2_stereo_apf_follow::Point2D target;
    target.x = msg->point.x;
    target.y = msg->point.y;
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

    std_msgs::msg::Bool valid;
    valid.data = true;
    seed_valid_pub_->publish(valid);
  }

  std::string one1000_topic_;
  std::string base_frame_;
  std::string default_one1000_frame_;
  std::string seed_target_topic_;
  std::string seed_valid_topic_;
  bool use_tf_{true};
  bool require_tf_{true};
  go2_stereo_apf_follow::UwbTargetConfig uwb_config_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr seed_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr seed_valid_pub_;
};

// 初始化并运行 UWB 目标种子适配节点。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UwbTargetSeedNode>());
  rclcpp::shutdown();
  return 0;
}
