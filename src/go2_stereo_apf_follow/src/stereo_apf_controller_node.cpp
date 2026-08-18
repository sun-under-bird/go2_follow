#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

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

go2_stereo_apf_follow::Point2D transform_point(
  const go2_stereo_apf_follow::Point2D & point,
  const geometry_msgs::msg::TransformStamped & transform)
{
  const auto out = transform_point(go2_stereo_apf_follow::Point3D{point.x, point.y, 0.0}, transform);
  return go2_stereo_apf_follow::Point2D{out.x, out.y};
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

class StereoApfControllerNode : public rclcpp::Node
{
public:
  // 初始化 APF 控制节点，并使用适合前向双目感知的保守速度和停车参数。
  StereoApfControllerNode()
  : Node("stereo_apf_controller_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic", "/local_grid_obstacle");
    seed_target_topic_ = declare_parameter<std::string>("seed_target_topic", "/stereo_apf/seed_target");
    manual_target_topic_ = declare_parameter<std::string>("manual_target_topic", "/stereo_apf/manual_target");
    enabled_topic_ = declare_parameter<std::string>("enabled_topic", "/stereo_apf/enabled");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    target_topic_ = declare_parameter<std::string>("target_topic", "/stereo_apf/target");
    status_topic_ = declare_parameter<std::string>("status_topic", "/stereo_apf/status");
    potential_field_topic_ =
      declare_parameter<std::string>("potential_field_topic", "/stereo_apf/potential_field");
    enabled_ = declare_parameter<bool>("enabled_default", true);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
    max_points_per_cloud_ = declare_parameter<int>("max_points_per_cloud", 60000);

    filter_config_.x_min = declare_parameter<double>("obstacle_x_min", -4.0);
    filter_config_.x_max = declare_parameter<double>("obstacle_x_max", 4.0);
    filter_config_.y_abs = declare_parameter<double>("obstacle_y_abs", 4.0);
    filter_config_.z_min = declare_parameter<double>("obstacle_z_min", 0.05);
    filter_config_.z_max = declare_parameter<double>("obstacle_z_max", 1.2);
    filter_config_.robot_frame_front = declare_parameter<double>("robot_frame_front", 0.15);
    filter_config_.robot_frame_back = declare_parameter<double>("robot_frame_back", 0.35);
    filter_config_.robot_frame_left = declare_parameter<double>("robot_frame_left", 0.15);
    filter_config_.robot_frame_right = declare_parameter<double>("robot_frame_right", 0.15);

    apf_config_.influence_dist = declare_parameter<double>("apf_influence_dist", 0.80);
    apf_config_.repulse_gain = declare_parameter<double>("apf_repulse_gain", 0.01);
    apf_config_.max_repulse = declare_parameter<double>("apf_max_repulse", 1.0);
    apf_config_.emergency_dist = declare_parameter<double>("apf_emergency_dist", 0.35);
    apf_config_.slowdown_dist = declare_parameter<double>("apf_slowdown_dist", 0.80);
    apf_config_.corridor_width = declare_parameter<double>("corridor_width", 0.35);

    control_config_.follow_distance = declare_parameter<double>("follow_distance", 1.0);
    control_config_.distance_deadband = declare_parameter<double>("distance_deadband", 0.05);
    control_config_.lateral_deadband = declare_parameter<double>("lateral_deadband", 0.03);
    control_config_.yaw_deadband = declare_parameter<double>("yaw_deadband", 0.10);
    control_config_.linear_scale = declare_parameter<double>("linear_scale", 0.50);
    control_config_.lateral_scale = declare_parameter<double>("lateral_scale", 1.0);
    control_config_.angular_scale = declare_parameter<double>("angular_scale", 1.0);
    control_config_.max_vx = declare_parameter<double>("max_vx", 0.45);
    control_config_.max_vy = declare_parameter<double>("max_vy", 0.25);
    control_config_.max_wz = declare_parameter<double>("max_vyaw", 0.8);
    control_config_.max_reverse_vx = declare_parameter<double>("max_reverse_vx", 0.0);
    control_config_.reverse_scale = declare_parameter<double>("reverse_scale", 0.8);
    control_config_.min_vx_abs = declare_parameter<double>("min_vx_abs", 0.06);
    control_config_.allow_reverse = declare_parameter<bool>("allow_reverse", false);

    publish_potential_field_ = declare_parameter<bool>("publish_potential_field", false);
    potential_field_sample_step_ = declare_parameter<double>("potential_field_sample_step", 0.40);
    potential_field_arrow_length_ = declare_parameter<double>("potential_field_arrow_length", 0.18);
    potential_field_attraction_gain_ = declare_parameter<double>("potential_field_attraction_gain", 1.0);
    potential_field_min_vector_norm_ = declare_parameter<double>("potential_field_min_vector_norm", 0.02);
    potential_field_max_arrows_ = declare_parameter<int>("potential_field_max_arrows", 300);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&StereoApfControllerNode::cloud_callback, this, std::placeholders::_1));
    seed_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      seed_target_topic_,
      10,
      std::bind(&StereoApfControllerNode::seed_callback, this, std::placeholders::_1));
    manual_target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      manual_target_topic_,
      10,
      std::bind(&StereoApfControllerNode::manual_target_callback, this, std::placeholders::_1));
    enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
      enabled_topic_,
      10,
      std::bind(&StereoApfControllerNode::enabled_callback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(target_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
    potential_field_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(potential_field_topic_, 2);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&StereoApfControllerNode::tick, this));

    RCLCPP_INFO(get_logger(), "stereo_apf_controller_node started");
  }

private:
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
          "PointCloud TF failed from %s to %s: %s",
          msg->header.frame_id.c_str(),
          base_frame_.c_str(),
          exc.what());
        return;
      }
    }

    std::vector<go2_stereo_apf_follow::Point3D> points;
    const auto cloud_size =
      static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height);
    const auto max_points = static_cast<std::size_t>(std::max(0, max_points_per_cloud_));
    points.reserve(std::min(cloud_size, max_points));
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
      if (go2_stereo_apf_follow::point_passes_filter(point, filter_config_)) {
        points.push_back(point);
      }
    }

    latest_points_ = std::move(points);
    have_cloud_ = true;
  }

  void seed_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // UWB seed 已是原始目标，转换坐标后直接用于控制。
    auto target = pose_to_base(*msg);
    if (!target.has_value()) {
      return;
    }
    target_ = target.value();
    have_target_ = true;
    target_source_ = "raw_uwb";
  }

  void manual_target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    auto target = pose_to_base(*msg);
    if (!target.has_value()) {
      return;
    }
    target_ = target.value();
    have_target_ = true;
    target_source_ = "manual";
    publish_status("tracking: manual target");
  }

  void enabled_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    enabled_ = msg->data;
  }

  std::optional<go2_stereo_apf_follow::Point2D> pose_to_base(
    const geometry_msgs::msg::PoseStamped & pose)
  {
    go2_stereo_apf_follow::Point2D point{pose.pose.position.x, pose.pose.position.y};
    const std::string frame = pose.header.frame_id.empty() ? base_frame_ : pose.header.frame_id;
    if (frame == base_frame_) {
      return point;
    }
    try {
      const auto tf = tf_buffer_.lookupTransform(base_frame_, frame, tf2::TimePointZero);
      return transform_point(point, tf);
    } catch (const std::exception & exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Target TF failed from %s to %s: %s",
        frame.c_str(),
        base_frame_.c_str(),
        exc.what());
      return std::nullopt;
    }
  }

  void tick()
  {
    if (!enabled_) {
      publish_potential_field_delete();
      publish_zero("stop: disabled");
      return;
    }
    if (!have_cloud_) {
      publish_potential_field_delete();
      publish_zero("stop: waiting for obstacle cloud");
      return;
    }

    if (!have_target_) {
      publish_potential_field_delete();
      publish_zero("stop: target unavailable");
      return;
    }

    const auto summary = go2_stereo_apf_follow::summarize_obstacles(
      latest_points_,
      target_,
      apf_config_);
    const auto cmd = go2_stereo_apf_follow::compute_follow_command(
      target_,
      summary,
      apf_config_,
      control_config_);
    cmd_pub_->publish(to_twist(cmd));
    publish_target();
    publish_potential_field();

    if (summary.has_nearest && summary.nearest_dist < apf_config_.emergency_dist) {
      publish_status("stop: obstacle emergency");
    } else if (summary.has_nearest && summary.nearest_dist < apf_config_.slowdown_dist) {
      publish_status("slow: obstacle near, target=" + target_source_);
    } else {
      publish_status("ok: target=" + target_source_);
    }
  }

  void publish_target()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = base_frame_;
    pose.pose.position.x = target_.x;
    pose.pose.position.y = target_.y;
    pose.pose.orientation = yaw_to_quaternion(std::atan2(target_.y, target_.x));
    target_pub_->publish(pose);
  }

  go2_stereo_apf_follow::Point2D potential_vector_at(
    const go2_stereo_apf_follow::Point2D & sample) const
  {
    go2_stereo_apf_follow::Point2D vector{
      potential_field_attraction_gain_ * (target_.x - sample.x),
      potential_field_attraction_gain_ * (target_.y - sample.y)};

    for (const auto & obstacle : latest_points_) {
      const double dx = sample.x - obstacle.x;
      const double dy = sample.y - obstacle.y;
      const double dist = std::hypot(dx, dy);
      if (dist <= 1e-6 || dist >= apf_config_.influence_dist) {
        continue;
      }
      const double force =
        apf_config_.repulse_gain * (1.0 / dist - 1.0 / apf_config_.influence_dist) / (dist * dist);
      vector.x += force * dx / dist;
      vector.y += force * dy / dist;
    }
    return vector;
  }

  bool sample_has_obstacle(
    const go2_stereo_apf_follow::Point2D & sample,
    double radius) const
  {
    for (const auto & obstacle : latest_points_) {
      if (std::hypot(sample.x - obstacle.x, sample.y - obstacle.y) <= radius) {
        return true;
      }
    }
    return false;
  }

  void publish_potential_field_delete()
  {
    if (!publish_potential_field_) {
      return;
    }
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = base_frame_;
    marker.ns = "stereo_apf_potential_field";
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(marker);
    potential_field_pub_->publish(markers);
  }

  void publish_potential_field()
  {
    if (!publish_potential_field_) {
      return;
    }
    const double step = std::max(0.05, potential_field_sample_step_);
    const double arrow_length = std::max(0.01, potential_field_arrow_length_);
    const int max_arrows = std::max(1, potential_field_max_arrows_);

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.stamp = now();
    clear_marker.header.frame_id = base_frame_;
    clear_marker.ns = "stereo_apf_potential_field";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);

    int id = 0;
    const double min_x = std::max(-2.0, filter_config_.x_min);
    const double max_x = std::min(4.0, filter_config_.x_max);
    const double min_y = -std::min(2.0, filter_config_.y_abs);
    const double max_y = std::min(2.0, filter_config_.y_abs);

    for (double x = min_x + 0.5 * step; x <= max_x && id < max_arrows; x += step) {
      for (double y = min_y + 0.5 * step; y <= max_y && id < max_arrows; y += step) {
        const go2_stereo_apf_follow::Point2D sample{x, y};
        const auto vector = potential_vector_at(sample);
        const double norm = std::hypot(vector.x, vector.y);
        if (norm < potential_field_min_vector_norm_) {
          continue;
        }

        geometry_msgs::msg::Point start;
        start.x = sample.x;
        start.y = sample.y;
        start.z = 0.05;
        geometry_msgs::msg::Point end;
        end.x = sample.x + arrow_length * vector.x / norm;
        end.y = sample.y + arrow_length * vector.y / norm;
        end.z = 0.05;

        visualization_msgs::msg::Marker marker;
        marker.header.stamp = now();
        marker.header.frame_id = base_frame_;
        marker.ns = "stereo_apf_potential_field";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.points.push_back(start);
        marker.points.push_back(end);
        marker.scale.x = 0.025;
        marker.scale.y = 0.055;
        marker.scale.z = 0.055;
        marker.color.a = 0.85;

        const bool occupied = sample_has_obstacle(sample, step * 0.5);
        if (occupied) {
          marker.color.r = 1.0;
          marker.color.g = 0.1;
          marker.color.b = 0.05;
        } else {
          const double obstacle_intensity = go2_stereo_apf_follow::clamp(norm / 2.0, 0.0, 1.0);
          marker.color.r = obstacle_intensity;
          marker.color.g = 0.35;
          marker.color.b = 1.0 - 0.6 * obstacle_intensity;
        }

        markers.markers.push_back(marker);
      }
    }

    potential_field_pub_->publish(markers);
  }

  void publish_zero(const std::string & status)
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
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
  std::string pointcloud_topic_;
  std::string seed_target_topic_;
  std::string manual_target_topic_;
  std::string enabled_topic_;
  std::string cmd_vel_topic_;
  std::string target_topic_;
  std::string status_topic_;
  std::string potential_field_topic_;
  bool enabled_{true};
  double publish_rate_hz_{30.0};
  int max_points_per_cloud_{60000};

  go2_stereo_apf_follow::PointFilterConfig filter_config_;
  go2_stereo_apf_follow::ApfConfig apf_config_;
  go2_stereo_apf_follow::FollowControlConfig control_config_;
  bool publish_potential_field_{false};
  double potential_field_sample_step_{0.40};
  double potential_field_arrow_length_{0.18};
  double potential_field_attraction_gain_{1.0};
  double potential_field_min_vector_norm_{0.02};
  int potential_field_max_arrows_{300};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr seed_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr manual_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enabled_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr potential_field_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<go2_stereo_apf_follow::Point3D> latest_points_;
  bool have_cloud_{false};
  go2_stereo_apf_follow::Point2D target_;
  bool have_target_{false};
  std::string target_source_{"none"};
  std::string last_status_;
};

// 初始化并运行双目 APF 跟随控制节点。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StereoApfControllerNode>());
  rclcpp::shutdown();
  return 0;
}
