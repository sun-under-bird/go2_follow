#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
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
  StereoApfControllerNode()
  : Node("stereo_apf_controller_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic", "/local_grid_obstacle");
    seed_target_topic_ = declare_parameter<std::string>("seed_target_topic", "/stereo_apf/seed_target");
    seed_valid_topic_ = declare_parameter<std::string>("seed_valid_topic", "/stereo_apf/seed_valid");
    manual_target_topic_ = declare_parameter<std::string>("manual_target_topic", "/stereo_apf/manual_target");
    enabled_topic_ = declare_parameter<std::string>("enabled_topic", "/stereo_apf/enabled");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_apf");
    target_topic_ = declare_parameter<std::string>("target_topic", "/stereo_apf/target");
    status_topic_ = declare_parameter<std::string>("status_topic", "/stereo_apf/status");
    local_map_topic_ = declare_parameter<std::string>("local_map_topic", "/stereo_apf/local_map");
    enabled_ = declare_parameter<bool>("enabled_default", true);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
    pointcloud_timeout_sec_ = declare_parameter<double>("pointcloud_timeout_sec", 0.5);
    seed_timeout_sec_ = declare_parameter<double>("seed_timeout_sec", 3.0);
    target_hold_sec_ = declare_parameter<double>("target_hold_sec", 0.8);
    seed_reset_distance_ = declare_parameter<double>("seed_reset_distance", 1.0);
    max_points_per_cloud_ = declare_parameter<int>("max_points_per_cloud", 60000);

    filter_config_.x_min = declare_parameter<double>("obstacle_x_min", 0.05);
    filter_config_.x_max = declare_parameter<double>("obstacle_x_max", 4.0);
    filter_config_.y_abs = declare_parameter<double>("obstacle_y_abs", 1.5);
    filter_config_.z_min = declare_parameter<double>("obstacle_z_min", 0.05);
    filter_config_.z_max = declare_parameter<double>("obstacle_z_max", 1.2);
    filter_config_.robot_frame_front = declare_parameter<double>("robot_frame_front", 0.35);
    filter_config_.robot_frame_back = declare_parameter<double>("robot_frame_back", 0.35);
    filter_config_.robot_frame_left = declare_parameter<double>("robot_frame_left", 0.20);
    filter_config_.robot_frame_right = declare_parameter<double>("robot_frame_right", 0.20);

    local_map_config_.width_m = declare_parameter<double>("local_map_width_m", 4.0);
    local_map_config_.height_m = declare_parameter<double>("local_map_height_m", 4.0);
    local_map_config_.resolution = declare_parameter<double>("local_map_resolution", 0.05);
    local_map_config_.origin_x = declare_parameter<double>("local_map_origin_x", 0.0);
    local_map_config_.origin_y = declare_parameter<double>("local_map_origin_y", -2.0);
    local_map_config_.obstacle_hold_sec = declare_parameter<double>("obstacle_hold_sec", 0.6);
    local_map_config_.min_points_per_cell = declare_parameter<int>("local_map_min_points_per_cell", 1);
    local_map_.configure(local_map_config_);

    tracking_config_.target_radius = declare_parameter<double>("target_radius", 0.30);
    tracking_config_.min_points_in_target = declare_parameter<int>("min_points_in_target", 3);
    tracking_config_.smoothing_alpha = declare_parameter<double>("smoothing_alpha", 0.35);

    apf_config_.influence_dist = declare_parameter<double>("apf_influence_dist", 1.2);
    apf_config_.repulse_gain = declare_parameter<double>("apf_repulse_gain", 0.08);
    apf_config_.max_repulse = declare_parameter<double>("apf_max_repulse", 0.30);
    apf_config_.emergency_dist = declare_parameter<double>("apf_emergency_dist", 0.45);
    apf_config_.slowdown_dist = declare_parameter<double>("apf_slowdown_dist", 1.0);
    apf_config_.corridor_width = declare_parameter<double>("corridor_width", 0.70);

    control_config_.follow_distance = declare_parameter<double>("follow_distance", 2.0);
    control_config_.distance_deadband = declare_parameter<double>("distance_deadband", 0.10);
    control_config_.lateral_deadband = declare_parameter<double>("lateral_deadband", 0.03);
    control_config_.yaw_deadband = declare_parameter<double>("yaw_deadband", 0.08);
    control_config_.linear_scale = declare_parameter<double>("linear_scale", 0.40);
    control_config_.lateral_scale = declare_parameter<double>("lateral_scale", 0.60);
    control_config_.angular_scale = declare_parameter<double>("angular_scale", 1.0);
    control_config_.max_vx = declare_parameter<double>("max_vx", 0.30);
    control_config_.max_vy = declare_parameter<double>("max_vy", 0.20);
    control_config_.max_wz = declare_parameter<double>("max_vyaw", 0.80);
    control_config_.max_reverse_vx = declare_parameter<double>("max_reverse_vx", 0.0);
    control_config_.allow_reverse = declare_parameter<bool>("allow_reverse", false);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_,
      5,
      std::bind(&StereoApfControllerNode::cloud_callback, this, std::placeholders::_1));
    seed_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      seed_target_topic_,
      10,
      std::bind(&StereoApfControllerNode::seed_callback, this, std::placeholders::_1));
    seed_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      seed_valid_topic_,
      10,
      std::bind(&StereoApfControllerNode::seed_valid_callback, this, std::placeholders::_1));
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
    local_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(local_map_topic_, 2);

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

    const auto current_time = now();
    local_map_.update(points, current_time.seconds());
    latest_points_ = local_map_.occupied_points(current_time.seconds());
    latest_cloud_time_ = current_time;
    have_cloud_ = true;
  }

  void seed_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    auto target = pose_to_base(*msg);
    if (!target.has_value()) {
      return;
    }
    seed_target_ = target.value();
    seed_time_ = now();
    have_seed_ = true;
  }

  void seed_valid_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    seed_valid_ = msg->data;
    seed_valid_time_ = now();
  }

  void manual_target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    auto target = pose_to_base(*msg);
    if (!target.has_value()) {
      return;
    }
    target_ = target.value();
    target_time_ = now();
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

  bool age_ok(const rclcpp::Time & stamp, double timeout_sec) const
  {
    if (stamp.nanoseconds() == 0) {
      return false;
    }
    return (now() - stamp).seconds() <= timeout_sec;
  }

  bool seed_is_fresh() const
  {
    return have_seed_ && seed_valid_ && age_ok(seed_time_, seed_timeout_sec_) &&
           age_ok(seed_valid_time_, seed_timeout_sec_);
  }

  bool refresh_target_from_seed(bool force)
  {
    if (!seed_is_fresh()) {
      return false;
    }
    if (!force && have_target_) {
      const double delta = std::hypot(seed_target_.x - target_.x, seed_target_.y - target_.y);
      if (delta <= seed_reset_distance_) {
        return false;
      }
    }
    target_ = seed_target_;
    target_time_ = now();
    have_target_ = true;
    target_source_ = "seed";
    return true;
  }

  void tick()
  {
    const auto current_time = now();
    local_map_.prune(current_time.seconds());
    latest_points_ = local_map_.occupied_points(current_time.seconds());
    publish_local_map(current_time.seconds());

    if (!enabled_) {
      publish_zero("stop: disabled");
      return;
    }
    if (!have_cloud_ || !age_ok(latest_cloud_time_, pointcloud_timeout_sec_)) {
      publish_zero("stop: obstacle cloud stale");
      return;
    }

    if (!have_target_) {
      if (!refresh_target_from_seed(true)) {
        publish_zero("stop: target unavailable");
        return;
      }
    } else {
      refresh_target_from_seed(false);
    }

    auto centroid = go2_stereo_apf_follow::compute_target_centroid(
      latest_points_,
      target_,
      tracking_config_);
    if (centroid.has_value()) {
      target_ = go2_stereo_apf_follow::smooth_target(
        target_,
        centroid.value(),
        tracking_config_.smoothing_alpha);
      target_time_ = now();
      target_source_ = "cloud";
    } else if ((now() - target_time_).seconds() > target_hold_sec_) {
      if (!refresh_target_from_seed(true)) {
        have_target_ = false;
        publish_zero("stop: target lost");
        return;
      }
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

  void publish_local_map(double now_sec)
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = now();
    grid.header.frame_id = base_frame_;
    grid.info.resolution = local_map_config_.resolution;
    grid.info.width = static_cast<unsigned int>(local_map_.width_cells());
    grid.info.height = static_cast<unsigned int>(local_map_.height_cells());
    grid.info.origin.position.x = local_map_config_.origin_x;
    grid.info.origin.position.y = local_map_config_.origin_y;
    grid.info.origin.orientation.w = 1.0;
    grid.data = local_map_.occupancy_data(now_sec);
    local_map_pub_->publish(grid);
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
  std::string seed_valid_topic_;
  std::string manual_target_topic_;
  std::string enabled_topic_;
  std::string cmd_vel_topic_;
  std::string target_topic_;
  std::string status_topic_;
  std::string local_map_topic_;
  bool enabled_{true};
  double publish_rate_hz_{30.0};
  double pointcloud_timeout_sec_{0.5};
  double seed_timeout_sec_{3.0};
  double target_hold_sec_{0.8};
  double seed_reset_distance_{1.0};
  int max_points_per_cloud_{60000};

  go2_stereo_apf_follow::PointFilterConfig filter_config_;
  go2_stereo_apf_follow::LocalMapConfig local_map_config_;
  go2_stereo_apf_follow::LocalObstacleMap local_map_;
  go2_stereo_apf_follow::TargetTrackingConfig tracking_config_;
  go2_stereo_apf_follow::ApfConfig apf_config_;
  go2_stereo_apf_follow::FollowControlConfig control_config_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr seed_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr seed_valid_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr manual_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enabled_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<go2_stereo_apf_follow::Point3D> latest_points_;
  rclcpp::Time latest_cloud_time_{0, 0, RCL_ROS_TIME};
  bool have_cloud_{false};
  go2_stereo_apf_follow::Point2D seed_target_;
  rclcpp::Time seed_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time seed_valid_time_{0, 0, RCL_ROS_TIME};
  bool have_seed_{false};
  bool seed_valid_{false};
  go2_stereo_apf_follow::Point2D target_;
  rclcpp::Time target_time_{0, 0, RCL_ROS_TIME};
  bool have_target_{false};
  std::string target_source_{"none"};
  std::string last_status_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StereoApfControllerNode>());
  rclcpp::shutdown();
  return 0;
}
