#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "pcl/filters/filter.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"


class ObstacleDetectorNode : public rclcpp::Node
{
public:
  ObstacleDetectorNode()
  : Node("obstacle_detector_node"),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/stereo/points2");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    distance_topic_ = declare_parameter<std::string>(
      "distance_topic", "/obstacle/nearest_distance");
    avoid_vector_topic_ = declare_parameter<std::string>(
      "avoid_vector_topic", "/obstacle/avoid_vector");
    debug_cloud_topic_ = declare_parameter<std::string>(
      "debug_cloud_topic", "/obstacle/used_points");
    publish_debug_cloud_ = declare_parameter<bool>("publish_debug_cloud", true);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.05);
    enable_passthrough_filter_ = declare_parameter<bool>("enable_passthrough_filter", true);
    enable_radius_outlier_filter_ = declare_parameter<bool>(
      "enable_radius_outlier_filter", true);
    radius_search_ = declare_parameter<double>("radius_search", 0.12);
    min_neighbors_in_radius_ = declare_parameter<int>("min_neighbors_in_radius", 3);
    min_x_ = declare_parameter<double>("min_x", 0.2);
    max_x_ = declare_parameter<double>("max_x", 2.0);
    max_abs_y_ = declare_parameter<double>("max_abs_y", 0.8);
    min_z_ = declare_parameter<double>("min_z", 0.08);
    max_z_ = declare_parameter<double>("max_z", 1.0);
    max_avoid_angular_ = declare_parameter<double>("max_avoid_angular", 0.8);
    min_obstacle_points_ = declare_parameter<int>("min_obstacle_points", 8);
    side_count_deadband_ = declare_parameter<int>("side_count_deadband", 3);

    sanitizeParameters();

    distance_publisher_ = create_publisher<std_msgs::msg::Float32>(distance_topic_, 10);
    vector_publisher_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      avoid_vector_topic_,
      10);
    if (publish_debug_cloud_) {
      debug_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        debug_cloud_topic_,
        10);
    }
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ObstacleDetectorNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "点云避障检测已启动：订阅 %s，TF 目标坐标系 %s，发布 %s 和 %s",
      cloud_topic_.c_str(),
      target_frame_.c_str(),
      distance_topic_.c_str(),
      avoid_vector_topic_.c_str());
  }

private:
  using PclCloud = pcl::PointCloud<pcl::PointXYZ>;

  // 修正异常参数，避免滤波阈值或检测区域被配置成无效值。
  void sanitizeParameters()
  {
    if (target_frame_.empty()) {
      target_frame_ = "base_link";
    }
    tf_timeout_sec_ = std::max(0.0, tf_timeout_sec_);
    radius_search_ = std::max(0.01, radius_search_);
    min_neighbors_in_radius_ = std::max(1, min_neighbors_in_radius_);
    min_x_ = std::max(0.0, min_x_);
    max_x_ = std::max(min_x_ + 0.05, max_x_);
    max_abs_y_ = std::max(0.05, max_abs_y_);
    max_z_ = std::max(min_z_ + 0.05, max_z_);
    max_avoid_angular_ = std::max(0.0, max_avoid_angular_);
    min_obstacle_points_ = std::max(1, min_obstacle_points_);
    side_count_deadband_ = std::max(0, side_count_deadband_);
  }

  // 接收双目原始点云，转换坐标系并用 PCL 做轻量滤波后发布避障结果。
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 cloud_in_base;
    if (!transformCloudToTargetFrame(*msg, cloud_in_base)) {
      // TF 失败时不发布新结果，让控制器通过障碍数据超时逻辑停车。
      return;
    }

    PclCloud::Ptr pcl_cloud(new PclCloud);
    try {
      pcl::fromROSMsg(cloud_in_base, *pcl_cloud);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "点云无法转换为 PCL XYZ 点云，跳过本帧：%s",
        error.what());
      return;
    }

    PclCloud::Ptr filtered_cloud = filterCloud(pcl_cloud);
    detectObstaclePoints(filtered_cloud, cloud_in_base.header);
  }

  // 将 stereo_image_proc 点云转换到 base_link，确保 x/y/z 的物理含义可用于避障。
  bool transformCloudToTargetFrame(
    const sensor_msgs::msg::PointCloud2 & cloud_msg,
    sensor_msgs::msg::PointCloud2 & transformed_cloud)
  {
    if (cloud_msg.header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "点云 header.frame_id 为空，无法查询 TF 转换到 %s",
        target_frame_.c_str());
      return false;
    }

    if (cloud_msg.header.frame_id == target_frame_) {
      transformed_cloud = cloud_msg;
      transformed_cloud.header.frame_id = target_frame_;
      return true;
    }

    try {
      const auto transform = tf_buffer_->lookupTransform(
        target_frame_,
        cloud_msg.header.frame_id,
        tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_sec_));
      tf2::doTransform(cloud_msg, transformed_cloud, transform);
      transformed_cloud.header.frame_id = target_frame_;
      return true;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "无法将点云从 %s 转换到 %s，跳过本帧：%s",
        cloud_msg.header.frame_id.c_str(),
        target_frame_.c_str(),
        error.what());
      return false;
    }
  }

  // 对点云执行 NaN 清理、z 高度过滤和半径离群点过滤。
  PclCloud::Ptr filterCloud(const PclCloud::Ptr & input_cloud)
  {
    PclCloud::Ptr finite_cloud(new PclCloud);
    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*input_cloud, *finite_cloud, valid_indices);

    PclCloud::Ptr filtered_cloud = finite_cloud;
    if (enable_passthrough_filter_) {
      filtered_cloud = applyPassThroughFilter(filtered_cloud);
    }
    if (enable_radius_outlier_filter_ && !filtered_cloud->empty()) {
      filtered_cloud = applyRadiusOutlierFilter(filtered_cloud);
    }
    return filtered_cloud;
  }

  // 使用 PCL PassThrough 先过滤 z 高度，减少地面点和过高点进入避障计算。
  PclCloud::Ptr applyPassThroughFilter(const PclCloud::Ptr & input_cloud) const
  {
    PclCloud::Ptr output_cloud(new PclCloud);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(input_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(static_cast<float>(min_z_), static_cast<float>(max_z_));
    pass.filter(*output_cloud);
    return output_cloud;
  }

  // 使用 PCL RadiusOutlierRemoval 去掉周围邻居太少的孤立深度噪点。
  PclCloud::Ptr applyRadiusOutlierFilter(const PclCloud::Ptr & input_cloud) const
  {
    PclCloud::Ptr output_cloud(new PclCloud);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter;
    radius_filter.setInputCloud(input_cloud);
    radius_filter.setRadiusSearch(radius_search_);
    radius_filter.setMinNeighborsInRadius(min_neighbors_in_radius_);
    radius_filter.filter(*output_cloud);
    return output_cloud;
  }

  // 在滤波后的 base_link 点云中统计前方 ROI 障碍点，计算最近距离和绕行方向。
  void detectObstaclePoints(
    const PclCloud::Ptr & cloud,
    const std_msgs::msg::Header & header)
  {
    double nearest_distance = std::numeric_limits<double>::infinity();
    std::size_t left_count = 0;
    std::size_t right_count = 0;
    std::size_t obstacle_count = 0;
    PclCloud::Ptr used_cloud(new PclCloud);

    for (const auto & point : cloud->points) {
      const double x = static_cast<double>(point.x);
      const double y = static_cast<double>(point.y);
      const double z = static_cast<double>(point.z);

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      if (!isInsideFrontRoi(x, y, z)) {
        continue;
      }

      // 这个点会真正参与最近距离和左右绕行方向统计，同时发布到调试点云。
      used_cloud->points.push_back(point);

      // 点云已经转换到 base_link：x 是前方距离，y 是左右位置。
      const double distance = std::hypot(x, y);
      nearest_distance = std::min(nearest_distance, distance);
      ++obstacle_count;

      if (y >= 0.0) {
        ++left_count;
      } else {
        ++right_count;
      }
    }

    publishDebugCloud(used_cloud, header);

    if (obstacle_count < static_cast<std::size_t>(min_obstacle_points_)) {
      // 少量点通常是深度噪声，发布无障碍结果。
      publishDistance(std::numeric_limits<double>::infinity());
      publishAvoidVector(header, std::numeric_limits<double>::infinity(), 0, 0);
      return;
    }

    publishDistance(nearest_distance);
    publishAvoidVector(header, nearest_distance, left_count, right_count);
  }

  // 判断点是否落在前方矩形避障检测区域内。
  bool isInsideFrontRoi(double x, double y, double z) const
  {
    return min_x_ <= x && x <= max_x_ &&
      std::abs(y) <= max_abs_y_ &&
      min_z_ <= z && z <= max_z_;
  }

  // 发布前方最近障碍距离；没有障碍时发布 inf。
  void publishDistance(double nearest_distance)
  {
    std_msgs::msg::Float32 distance_msg;
    distance_msg.data = static_cast<float>(nearest_distance);
    distance_publisher_->publish(distance_msg);
  }

  // 根据左右障碍点数量发布向空侧绕行的角速度建议。
  void publishAvoidVector(
    const std_msgs::msg::Header & header,
    double nearest_distance,
    std::size_t left_count,
    std::size_t right_count)
  {
    geometry_msgs::msg::Vector3Stamped vector_msg;
    vector_msg.header = header;
    vector_msg.header.frame_id = target_frame_;

    if (std::isinf(nearest_distance)) {
      // 无障碍时不建议转向，并重置为默认左绕方向。
      vector_msg.vector.z = 0.0;
      last_avoid_direction_ = 1;
      vector_publisher_->publish(vector_msg);
      return;
    }

    const int direction = chooseAvoidDirection(left_count, right_count);
    vector_msg.vector.z = static_cast<double>(direction) * max_avoid_angular_;
    vector_publisher_->publish(vector_msg);
  }

  // 发布真正参与避障判断的点云，便于在 RViz 中检查 ROI 和滤波结果。
  void publishDebugCloud(
    const PclCloud::Ptr & used_cloud,
    const std_msgs::msg::Header & header)
  {
    if (!publish_debug_cloud_ || !debug_cloud_publisher_) {
      return;
    }

    used_cloud->width = static_cast<std::uint32_t>(used_cloud->points.size());
    used_cloud->height = 1;
    used_cloud->is_dense = true;

    sensor_msgs::msg::PointCloud2 debug_msg;
    pcl::toROSMsg(*used_cloud, debug_msg);
    debug_msg.header = header;
    debug_msg.header.frame_id = target_frame_;
    debug_cloud_publisher_->publish(debug_msg);
  }

  // 根据左右点数选择绕行方向；差异不明显时沿用上一次方向，避免左右来回跳。
  int chooseAvoidDirection(std::size_t left_count, std::size_t right_count)
  {
    const int diff = static_cast<int>(left_count) - static_cast<int>(right_count);
    if (std::abs(diff) <= side_count_deadband_) {
      return last_avoid_direction_;
    }

    if (diff > 0) {
      // 左侧障碍更多，建议向右转；ROS 中 z 角速度负值表示右转。
      last_avoid_direction_ = -1;
    } else {
      // 右侧障碍更多，建议向左转；ROS 中 z 角速度正值表示左转。
      last_avoid_direction_ = 1;
    }
    return last_avoid_direction_;
  }

  std::string cloud_topic_;
  std::string target_frame_;
  std::string distance_topic_;
  std::string avoid_vector_topic_;
  std::string debug_cloud_topic_;
  bool publish_debug_cloud_;
  double tf_timeout_sec_;
  bool enable_passthrough_filter_;
  bool enable_radius_outlier_filter_;
  double radius_search_;
  int min_neighbors_in_radius_;
  double min_x_;
  double max_x_;
  double max_abs_y_;
  double min_z_;
  double max_z_;
  double max_avoid_angular_;
  int min_obstacle_points_;
  int side_count_deadband_;
  int last_avoid_direction_{1};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr vector_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_cloud_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
