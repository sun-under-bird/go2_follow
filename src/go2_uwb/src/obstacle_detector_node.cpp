#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"


class ObstacleDetectorNode : public rclcpp::Node
{
public:
  // 初始化避障节点参数、发布器、订阅器和 TF 监听器。
  ObstacleDetectorNode()
  : Node("obstacle_detector_node"),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    // D435i 已矫正双目图由 RTAB-Map 生成局部障碍云，本节点不再启动图像处理链。
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/local_grid_obstacle");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_footprint");
    distance_topic_ = declare_parameter<std::string>(
      "distance_topic", "/obstacle/nearest_distance");
    avoid_vector_topic_ = declare_parameter<std::string>(
      "avoid_vector_topic", "/obstacle/avoid_vector");
    debug_cloud_topic_ = declare_parameter<std::string>(
      "debug_cloud_topic", "/obstacle/used_points");
    publish_debug_cloud_ = declare_parameter<bool>("publish_debug_cloud", true);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.05);
    enable_passthrough_filter_ = declare_parameter<bool>("enable_passthrough_filter", true);
    enable_voxel_filter_ = declare_parameter<bool>("enable_voxel_filter", true);
    voxel_leaf_size_ = declare_parameter<double>("voxel_leaf_size", 0.05);
    enable_radius_outlier_filter_ = declare_parameter<bool>(
      "enable_radius_outlier_filter", false);
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

  // 记录点云各处理阶段的点数，便于节流日志定位耗时或过滤问题。
  struct FilterStats
  {
    std::size_t finite_count{0};
    std::size_t roi_count{0};
    std::size_t voxel_count{0};
    std::size_t output_count{0};
  };

  // 保存单帧 TF 的旋转矩阵和平移，避免每个点重复展开四元数。
  struct PointTransform
  {
    double r00{1.0};
    double r01{0.0};
    double r02{0.0};
    double r10{0.0};
    double r11{1.0};
    double r12{0.0};
    double r20{0.0};
    double r21{0.0};
    double r22{1.0};
    double tx{0.0};
    double ty{0.0};
    double tz{0.0};
  };

  // 修正异常参数，避免滤波阈值或检测区域被配置成无效值。
  void sanitizeParameters()
  {
    if (target_frame_.empty()) {
      target_frame_ = "base_footprint";
    }
    tf_timeout_sec_ = std::max(0.0, tf_timeout_sec_);
    voxel_leaf_size_ = std::max(0.01, voxel_leaf_size_);
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
    const auto start_time = std::chrono::steady_clock::now();
    refreshRuntimeParameters();

    FilterStats filter_stats;
    PclCloud::Ptr roi_cloud(new PclCloud);
    std_msgs::msg::Header target_header;
    if (!convertCloudToTargetRoi(*msg, roi_cloud, target_header, filter_stats)) {
      // 点云字段或 TF 无效时不发布新结果，让控制器通过障碍数据超时逻辑停车。
      return;
    }

    PclCloud::Ptr filtered_cloud = filterCloud(roi_cloud, filter_stats);
    detectObstaclePoints(filtered_cloud, target_header);

    const auto end_time = std::chrono::steady_clock::now();
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      end_time - start_time).count();
    publishProcessingStats(
      static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height),
      filter_stats,
      elapsed_ms);
  }

  // 将 PointCloud2 逐点转换到目标坐标系，并只保留 ROI 内的有限点。
  bool convertCloudToTargetRoi(
    const sensor_msgs::msg::PointCloud2 & cloud_msg,
    const PclCloud::Ptr & roi_cloud,
    std_msgs::msg::Header & target_header,
    FilterStats & stats)
  {
    geometry_msgs::msg::TransformStamped transform;
    if (!lookupTransformToTargetFrame(cloud_msg, transform)) {
      return false;
    }

    target_header = cloud_msg.header;
    target_header.frame_id = target_frame_;
    const PointTransform point_transform = makePointTransform(transform);
    roi_cloud->points.reserve(
      std::min<std::size_t>(
        static_cast<std::size_t>(cloud_msg.width) * static_cast<std::size_t>(cloud_msg.height),
        50000));

    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_msg, "z");
      const auto iter_end = iter_x.end();

      for (; iter_x != iter_end; ++iter_x, ++iter_y, ++iter_z) {
        const double raw_x = static_cast<double>(*iter_x);
        const double raw_y = static_cast<double>(*iter_y);
        const double raw_z = static_cast<double>(*iter_z);

        if (!std::isfinite(raw_x) || !std::isfinite(raw_y) || !std::isfinite(raw_z)) {
          continue;
        }

        ++stats.finite_count;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        transformPoint(raw_x, raw_y, raw_z, point_transform, x, y, z);

        // 如果关闭前置 ROI，仍转换有限点，后续 detectObstaclePoints 会做兜底 ROI 判断。
        if (enable_passthrough_filter_ && !isInsideFrontRoi(x, y, z)) {
          continue;
        }

        roi_cloud->points.emplace_back(
          static_cast<float>(x),
          static_cast<float>(y),
          static_cast<float>(z));
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "点云缺少 float32 x/y/z 字段或字段无法读取，跳过本帧：%s",
        error.what());
      return false;
    }

    finalizeCloudLayout(roi_cloud);
    stats.roi_count = roi_cloud->points.size();
    return true;
  }

  // 查询点云坐标系到目标坐标系的 TF；同坐标系时返回单位变换。
  bool lookupTransformToTargetFrame(
    const sensor_msgs::msg::PointCloud2 & cloud_msg,
    geometry_msgs::msg::TransformStamped & transform)
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
      transform.header = cloud_msg.header;
      transform.child_frame_id = cloud_msg.header.frame_id;
      transform.transform.rotation.w = 1.0;
      return true;
    }

    try {
      transform = tf_buffer_->lookupTransform(
        target_frame_,
        cloud_msg.header.frame_id,
        tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_sec_));
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

  // 将 TransformStamped 预计算为旋转矩阵和平移，降低逐点变换开销。
  PointTransform makePointTransform(
    const geometry_msgs::msg::TransformStamped & transform) const
  {
    const auto & rotation = transform.transform.rotation;
    const auto & translation = transform.transform.translation;
    PointTransform point_transform;

    double qx = rotation.x;
    double qy = rotation.y;
    double qz = rotation.z;
    double qw = rotation.w;
    const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (norm > 0.0) {
      qx /= norm;
      qy /= norm;
      qz /= norm;
      qw /= norm;
    } else {
      qx = 0.0;
      qy = 0.0;
      qz = 0.0;
      qw = 1.0;
    }

    const double xx = qx * qx;
    const double yy = qy * qy;
    const double zz = qz * qz;
    const double xy = qx * qy;
    const double xz = qx * qz;
    const double yz = qy * qz;
    const double wx = qw * qx;
    const double wy = qw * qy;
    const double wz = qw * qz;

    point_transform.r00 = 1.0 - 2.0 * (yy + zz);
    point_transform.r01 = 2.0 * (xy - wz);
    point_transform.r02 = 2.0 * (xz + wy);
    point_transform.r10 = 2.0 * (xy + wz);
    point_transform.r11 = 1.0 - 2.0 * (xx + zz);
    point_transform.r12 = 2.0 * (yz - wx);
    point_transform.r20 = 2.0 * (xz - wy);
    point_transform.r21 = 2.0 * (yz + wx);
    point_transform.r22 = 1.0 - 2.0 * (xx + yy);
    point_transform.tx = translation.x;
    point_transform.ty = translation.y;
    point_transform.tz = translation.z;
    return point_transform;
  }

  // 使用预计算矩阵对单个点做坐标变换，避免整帧点云 TF 转换的额外开销。
  void transformPoint(
    double input_x,
    double input_y,
    double input_z,
    const PointTransform & transform,
    double & output_x,
    double & output_y,
    double & output_z) const
  {
    output_x = transform.r00 * input_x + transform.r01 * input_y +
      transform.r02 * input_z + transform.tx;
    output_y = transform.r10 * input_x + transform.r11 * input_y +
      transform.r12 * input_z + transform.ty;
    output_z = transform.r20 * input_x + transform.r21 * input_y +
      transform.r22 * input_z + transform.tz;
  }

  // 刷新可现场调参的滤波和 ROI 参数，让 ros2 param set 能在下一帧生效。
  void refreshRuntimeParameters()
  {
    get_parameter("tf_timeout_sec", tf_timeout_sec_);
    get_parameter("enable_passthrough_filter", enable_passthrough_filter_);
    get_parameter("enable_voxel_filter", enable_voxel_filter_);
    get_parameter("voxel_leaf_size", voxel_leaf_size_);
    get_parameter("enable_radius_outlier_filter", enable_radius_outlier_filter_);
    get_parameter("radius_search", radius_search_);
    get_parameter("min_neighbors_in_radius", min_neighbors_in_radius_);
    get_parameter("min_x", min_x_);
    get_parameter("max_x", max_x_);
    get_parameter("max_abs_y", max_abs_y_);
    get_parameter("min_z", min_z_);
    get_parameter("max_z", max_z_);
    get_parameter("max_avoid_angular", max_avoid_angular_);
    get_parameter("min_obstacle_points", min_obstacle_points_);
    get_parameter("side_count_deadband", side_count_deadband_);
    sanitizeParameters();
  }

  // 对 ROI 点云执行体素降采样和可选半径离群点过滤。
  PclCloud::Ptr filterCloud(const PclCloud::Ptr & input_cloud, FilterStats & stats)
  {
    PclCloud::Ptr filtered_cloud = input_cloud;

    if (enable_voxel_filter_ && !filtered_cloud->empty()) {
      filtered_cloud = applyVoxelGridFilter(filtered_cloud);
    }
    stats.voxel_count = filtered_cloud->points.size();

    if (enable_radius_outlier_filter_ && !filtered_cloud->empty()) {
      filtered_cloud = applyRadiusOutlierFilter(filtered_cloud);
    }
    stats.output_count = filtered_cloud->points.size();
    return filtered_cloud;
  }

  // 使用 VoxelGrid 对 ROI 点云降采样，保证高分辨率双目点云也能稳定实时处理。
  PclCloud::Ptr applyVoxelGridFilter(const PclCloud::Ptr & input_cloud) const
  {
    PclCloud::Ptr output_cloud(new PclCloud);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(input_cloud);
    voxel_filter.setLeafSize(
      static_cast<float>(voxel_leaf_size_),
      static_cast<float>(voxel_leaf_size_),
      static_cast<float>(voxel_leaf_size_));
    voxel_filter.filter(*output_cloud);
    return output_cloud;
  }

  // 统一设置 PCL 点云布局字段，保证后续 PCL 滤波器按无序点云处理。
  void finalizeCloudLayout(const PclCloud::Ptr & cloud) const
  {
    cloud->width = static_cast<std::uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;
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

  // 在滤波后的 base_footprint 点云中统计前方 ROI 障碍点，计算最近距离和绕行方向。
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

      // 点云已经转换到 base_footprint：x 是前方距离，y 是左右位置。
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

  // 节流输出点云处理统计，便于现场判断是否卡在裁剪、降采样或半径滤波。
  void publishProcessingStats(
    std::size_t input_count,
    const FilterStats & stats,
    double elapsed_ms)
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "点云处理统计：输入 %zu，有限 %zu，ROI %zu，体素 %zu，最终 %zu，耗时 %.2f ms",
      input_count,
      stats.finite_count,
      stats.roi_count,
      stats.voxel_count,
      stats.output_count,
      elapsed_ms);
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
  bool enable_voxel_filter_;
  double voxel_leaf_size_;
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


// 启动 ROS2 节点并进入事件循环。
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
