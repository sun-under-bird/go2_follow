#include "go2_uwb_mppi_follow/target_obstacle_filter_node.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace go2_uwb_mppi_follow
{

// 构造目标障碍过滤节点，并初始化参数、话题和 TF。
TargetObstacleFilterNode::TargetObstacleFilterNode()
: Node("target_obstacle_filter_node")
{
  declareParameters();
  loadParameters();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  target_path_sub_ = create_subscription<nav_msgs::msg::Path>(
    config_.target_path_topic,
    10,
    std::bind(&TargetObstacleFilterNode::targetPathCallback, this, std::placeholders::_1));

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    config_.input_cloud_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&TargetObstacleFilterNode::cloudCallback, this, std::placeholders::_1));

  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(config_.output_cloud_topic, 10);

  RCLCPP_WARN(get_logger(), "target_obstacle_filter_node started");
}

// 声明节点使用的所有可调 ROS 参数。
void TargetObstacleFilterNode::declareParameters()
{
  declare_parameter<std::string>("input_cloud_topic", "/local_grid_obstacle");
  declare_parameter<std::string>("output_cloud_topic", "/local_grid_obstacle_filtered");
  declare_parameter<std::string>("target_path_topic", "/uwb_target_history");
  declare_parameter<std::string>("filter_frame", "base_link");
  declare_parameter<double>("target_timeout_sec", 0.8);
  declare_parameter<double>("transform_timeout_sec", 0.2);
  declare_parameter<double>("clear_radius_m", 0.45);
  declare_parameter<double>("clear_z_min_m", 0.05);
  declare_parameter<double>("clear_z_max_m", 2.0);
  declare_parameter<bool>("pass_through_without_target", true);
  declare_parameter<bool>("use_latest_tf", true);
}

// 读取 ROS 参数并缓存到过滤配置中。
void TargetObstacleFilterNode::loadParameters()
{
  config_.input_cloud_topic = get_parameter("input_cloud_topic").as_string();
  config_.output_cloud_topic = get_parameter("output_cloud_topic").as_string();
  config_.target_path_topic = get_parameter("target_path_topic").as_string();
  config_.filter_frame = get_parameter("filter_frame").as_string();
  config_.target_timeout_sec = get_parameter("target_timeout_sec").as_double();
  config_.transform_timeout_sec = get_parameter("transform_timeout_sec").as_double();
  config_.clear_radius_m = get_parameter("clear_radius_m").as_double();
  config_.clear_z_min_m = get_parameter("clear_z_min_m").as_double();
  config_.clear_z_max_m = get_parameter("clear_z_max_m").as_double();
  config_.pass_through_without_target = get_parameter("pass_through_without_target").as_bool();
  config_.use_latest_tf = get_parameter("use_latest_tf").as_bool();
}

// 接收 UWB 目标历史路径，并取最后一个点作为当前被跟随目标。
void TargetObstacleFilterNode::targetPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (msg->poses.empty()) {
    latest_target_.reset();
    return;
  }

  const auto & pose = msg->poses.back();
  const rclcpp::Time stamp = pose.header.stamp.sec == 0 && pose.header.stamp.nanosec == 0 ?
    now() : rclcpp::Time(pose.header.stamp, get_clock()->get_clock_type());

  TargetPoint target;
  target.stamp = stamp;
  target.frame_id = pose.header.frame_id.empty() ? msg->header.frame_id : pose.header.frame_id;
  target.point = pose.pose.position;
  latest_target_ = target;
}

// 接收障碍点云，删除目标人附近的小圆柱区域后发布过滤点云。
void TargetObstacleFilterNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const rclcpp::Time stamp = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ?
    now() : rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());

  if (!hasFreshTarget(stamp)) {
    if (config_.pass_through_without_target) {
      publishPassThrough(*msg);
    }
    return;
  }

  const auto target = targetInFilterFrame(stamp);
  if (!target) {
    publishPassThrough(*msg);
    return;
  }

  geometry_msgs::msg::TransformStamped cloud_to_filter;
  try {
    const rclcpp::Time tf_time = config_.use_latest_tf ?
      rclcpp::Time(0, 0, get_clock()->get_clock_type()) : stamp;
    cloud_to_filter = tf_buffer_->lookupTransform(
      config_.filter_frame,
      msg->header.frame_id,
      tf_time,
      tf2::durationFromSec(config_.transform_timeout_sec));
  } catch (const tf2::TransformException & exc) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Obstacle cloud transform failed: %s",
      exc.what());
    publishPassThrough(*msg);
    return;
  }

  sensor_msgs::msg::PointCloud2 filtered = *msg;
  filtered.height = 1;
  filtered.width = 0;
  filtered.row_step = 0;
  filtered.data.clear();
  filtered.data.reserve(msg->data.size());
  filtered.is_dense = false;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  for (std::uint32_t row = 0; row < msg->height; ++row) {
    for (std::uint32_t col = 0; col < msg->width; ++col, ++iter_x, ++iter_y, ++iter_z) {
      geometry_msgs::msg::PointStamped source;
      source.header = msg->header;
      source.point.x = *iter_x;
      source.point.y = *iter_y;
      source.point.z = *iter_z;

      geometry_msgs::msg::PointStamped point_in_filter;
      tf2::doTransform(source, point_in_filter, cloud_to_filter);

      if (pointInsideTargetCylinder(point_in_filter.point, *target)) {
        continue;
      }

      const std::size_t offset =
        static_cast<std::size_t>(row) * msg->row_step +
        static_cast<std::size_t>(col) * msg->point_step;
      const auto * begin = msg->data.data() + offset;
      filtered.data.insert(filtered.data.end(), begin, begin + msg->point_step);
      ++filtered.width;
    }
  }

  filtered.row_step = filtered.point_step * filtered.width;
  cloud_pub_->publish(filtered);
}

// 判断当前是否有未超时的目标点。
bool TargetObstacleFilterNode::hasFreshTarget(const rclcpp::Time & stamp) const
{
  if (!latest_target_) {
    return false;
  }

  return (stamp - latest_target_->stamp).seconds() <= config_.target_timeout_sec;
}

// 将缓存的目标点转换到过滤坐标系。
std::optional<geometry_msgs::msg::Point> TargetObstacleFilterNode::targetInFilterFrame(
  const rclcpp::Time & stamp)
{
  if (!latest_target_) {
    return std::nullopt;
  }

  if (latest_target_->frame_id == config_.filter_frame) {
    return latest_target_->point;
  }

  geometry_msgs::msg::PointStamped source;
  source.header.frame_id = latest_target_->frame_id;
  source.header.stamp = config_.use_latest_tf ?
    rclcpp::Time(0, 0, get_clock()->get_clock_type()) : stamp;
  source.point = latest_target_->point;

  try {
    const auto transformed = tf_buffer_->transform(
      source,
      config_.filter_frame,
      tf2::durationFromSec(config_.transform_timeout_sec));
    return transformed.point;
  } catch (const tf2::TransformException & exc) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Target transform for obstacle filtering failed: %s",
      exc.what());
    return std::nullopt;
  }
}

// 判断过滤坐标系下的点是否位于目标清除圆柱内。
bool TargetObstacleFilterNode::pointInsideTargetCylinder(
  const geometry_msgs::msg::Point & point,
  const geometry_msgs::msg::Point & target) const
{
  const double dx = point.x - target.x;
  const double dy = point.y - target.y;
  const double dz = point.z - target.z;
  const double radius = std::hypot(dx, dy);
  return radius <= config_.clear_radius_m &&
    dz >= config_.clear_z_min_m &&
    dz <= config_.clear_z_max_m;
}

// 直接转发原始点云。
void TargetObstacleFilterNode::publishPassThrough(const sensor_msgs::msg::PointCloud2 & msg)
{
  cloud_pub_->publish(msg);
}

}  // 命名空间 go2_uwb_mppi_follow

// 启动目标障碍过滤节点。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2_uwb_mppi_follow::TargetObstacleFilterNode>());
  rclcpp::shutdown();
  return 0;
}
