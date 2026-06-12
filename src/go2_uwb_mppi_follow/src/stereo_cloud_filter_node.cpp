#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace go2_uwb_mppi_follow
{
namespace
{

// 保存已转换到目标坐标系的轻量点。
struct CloudPoint
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
};

// 体素索引，用于把密集双目点云压缩成稳定稀疏点云。
struct VoxelKey
{
  int x{0};
  int y{0};
  int z{0};

  // 判断两个体素索引是否指向同一个格子。
  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

// 为 VoxelKey 提供哈希函数，供 unordered_map 使用。
struct VoxelKeyHash
{
  // 混合三个整数索引，减少常见栅格排列下的哈希冲突。
  std::size_t operator()(const VoxelKey & key) const
  {
    std::size_t seed = static_cast<std::size_t>(key.x) * 73856093U;
    seed ^= static_cast<std::size_t>(key.y) * 19349663U;
    seed ^= static_cast<std::size_t>(key.z) * 83492791U;
    return seed;
  }
};

using VoxelMap = std::unordered_map<VoxelKey, CloudPoint, VoxelKeyHash>;

// 判断浮点值是否有限，避免 NaN 点污染 costmap。
bool finiteValue(const float value)
{
  return std::isfinite(static_cast<double>(value));
}

}  // namespace

class StereoCloudFilterNode : public rclcpp::Node
{
public:
  // 初始化双目点云过滤节点、参数、TF 和发布订阅。
  StereoCloudFilterNode()
  : Node("stereo_cloud_filter_node"),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    declareParameters();
    loadParameters();

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&StereoCloudFilterNode::cloudCallback, this, std::placeholders::_1));
    obstacle_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      obstacle_cloud_topic_,
      rclcpp::SensorDataQoS());
    ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      ground_cloud_topic_,
      rclcpp::SensorDataQoS());

    RCLCPP_WARN(
      get_logger(),
      "stereo_cloud_filter_node started: %s -> %s, %s",
      input_cloud_topic_.c_str(),
      obstacle_cloud_topic_.c_str(),
      ground_cloud_topic_.c_str());
  }

private:
  // 声明点云输入、输出、坐标系和过滤阈值参数。
  void declareParameters()
  {
    declare_parameter<std::string>("input_cloud_topic", "/stereo/points2");
    declare_parameter<std::string>("obstacle_cloud_topic", "/local_grid_obstacle");
    declare_parameter<std::string>("ground_cloud_topic", "/local_grid_ground");
    declare_parameter<std::string>("target_frame", "base_link");
    declare_parameter<bool>("use_latest_tf", true);
    declare_parameter<double>("transform_timeout_sec", 0.2);
    declare_parameter<double>("voxel_leaf_size", 0.05);
    declare_parameter<double>("min_x", 0.15);
    declare_parameter<double>("max_x", 3.0);
    declare_parameter<double>("max_abs_y", 1.5);
    declare_parameter<double>("min_z", 0.0);
    declare_parameter<double>("max_z", 1.5);
    declare_parameter<double>("ground_z_max", 0.12);
    declare_parameter<double>("obstacle_z_min", 0.10);
  }

  // 读取参数并做基础修正，避免无效阈值导致整帧点云被过滤。
  void loadParameters()
  {
    input_cloud_topic_ = get_parameter("input_cloud_topic").as_string();
    obstacle_cloud_topic_ = get_parameter("obstacle_cloud_topic").as_string();
    ground_cloud_topic_ = get_parameter("ground_cloud_topic").as_string();
    target_frame_ = get_parameter("target_frame").as_string();
    use_latest_tf_ = get_parameter("use_latest_tf").as_bool();
    transform_timeout_sec_ = std::max(0.0, get_parameter("transform_timeout_sec").as_double());
    voxel_leaf_size_ = std::max(0.01, get_parameter("voxel_leaf_size").as_double());
    min_x_ = get_parameter("min_x").as_double();
    max_x_ = get_parameter("max_x").as_double();
    max_abs_y_ = std::max(0.05, get_parameter("max_abs_y").as_double());
    min_z_ = get_parameter("min_z").as_double();
    max_z_ = get_parameter("max_z").as_double();
    ground_z_max_ = get_parameter("ground_z_max").as_double();
    obstacle_z_min_ = get_parameter("obstacle_z_min").as_double();

    if (target_frame_.empty()) {
      target_frame_ = "base_link";
    }
    if (max_x_ <= min_x_) {
      max_x_ = min_x_ + 0.5;
    }
    if (max_z_ <= min_z_) {
      max_z_ = min_z_ + 0.5;
    }
    ground_z_max_ = std::clamp(ground_z_max_, min_z_, max_z_);
    obstacle_z_min_ = std::clamp(obstacle_z_min_, min_z_, max_z_);
  }

  // 每帧刷新可运行时调整的过滤参数。
  void refreshRuntimeParameters()
  {
    get_parameter("use_latest_tf", use_latest_tf_);
    get_parameter("transform_timeout_sec", transform_timeout_sec_);
    get_parameter("voxel_leaf_size", voxel_leaf_size_);
    get_parameter("min_x", min_x_);
    get_parameter("max_x", max_x_);
    get_parameter("max_abs_y", max_abs_y_);
    get_parameter("min_z", min_z_);
    get_parameter("max_z", max_z_);
    get_parameter("ground_z_max", ground_z_max_);
    get_parameter("obstacle_z_min", obstacle_z_min_);
    loadParameters();
  }

  // 接收双目点云，转换坐标系后拆分成障碍点云和地面清除点云。
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    refreshRuntimeParameters();

    geometry_msgs::msg::TransformStamped transform;
    if (!lookupTransform(*msg, transform)) {
      publishEmptyCloud(msg->header.stamp);
      return;
    }

    VoxelMap obstacle_voxels;
    VoxelMap ground_voxels;
    std::size_t finite_count = 0;
    std::size_t roi_count = 0;

    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (!finiteValue(*iter_x) || !finiteValue(*iter_y) || !finiteValue(*iter_z)) {
          continue;
        }

        ++finite_count;
        const auto point = transformPoint(*iter_x, *iter_y, *iter_z, msg->header, transform);
        if (!point || !insideRoi(*point)) {
          continue;
        }

        ++roi_count;
        if (isGroundPoint(*point)) {
          addVoxel(ground_voxels, *point);
        }
        if (isObstaclePoint(*point)) {
          addVoxel(obstacle_voxels, *point);
        }
      }
    } catch (const std::exception & exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "stereo cloud missing float x/y/z fields: %s",
        exc.what());
      publishEmptyCloud(msg->header.stamp);
      return;
    }

    publishVoxelCloud(obstacle_voxels, obstacle_pub_, msg->header.stamp);
    publishVoxelCloud(ground_voxels, ground_pub_, msg->header.stamp);
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "stereo cloud filter: finite=%zu roi=%zu obstacle=%zu ground=%zu",
      finite_count,
      roi_count,
      obstacle_voxels.size(),
      ground_voxels.size());
  }

  // 查询点云坐标系到目标坐标系的 TF；同坐标系时返回单位变换。
  bool lookupTransform(
    const sensor_msgs::msg::PointCloud2 & cloud,
    geometry_msgs::msg::TransformStamped & transform)
  {
    if (cloud.header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "stereo cloud frame_id is empty");
      return false;
    }

    if (cloud.header.frame_id == target_frame_) {
      transform.header = cloud.header;
      transform.child_frame_id = cloud.header.frame_id;
      transform.transform.rotation.w = 1.0;
      return true;
    }

    try {
      const rclcpp::Time tf_time = use_latest_tf_ ?
        rclcpp::Time(0, 0, get_clock()->get_clock_type()) :
        rclcpp::Time(cloud.header.stamp, get_clock()->get_clock_type());
      transform = tf_buffer_->lookupTransform(
        target_frame_,
        cloud.header.frame_id,
        tf_time,
        tf2::durationFromSec(transform_timeout_sec_));
      return true;
    } catch (const tf2::TransformException & exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "stereo cloud TF failed from %s to %s: %s",
        cloud.header.frame_id.c_str(),
        target_frame_.c_str(),
        exc.what());
      return false;
    }
  }

  // 将单个点转换到目标坐标系，失败时返回空值跳过该点。
  std::optional<CloudPoint> transformPoint(
    const float x,
    const float y,
    const float z,
    const std_msgs::msg::Header & header,
    const geometry_msgs::msg::TransformStamped & transform) const
  {
    geometry_msgs::msg::PointStamped source;
    source.header = header;
    source.point.x = x;
    source.point.y = y;
    source.point.z = z;

    geometry_msgs::msg::PointStamped output;
    tf2::doTransform(source, output, transform);
    CloudPoint point;
    point.x = static_cast<float>(output.point.x);
    point.y = static_cast<float>(output.point.y);
    point.z = static_cast<float>(output.point.z);
    return point;
  }

  // 判断点是否落在前视局部代价地图关心的区域。
  bool insideRoi(const CloudPoint & point) const
  {
    return point.x >= min_x_ && point.x <= max_x_ &&
      std::abs(point.y) <= max_abs_y_ &&
      point.z >= min_z_ && point.z <= max_z_;
  }

  // 判断点是否属于地面清除层。
  bool isGroundPoint(const CloudPoint & point) const
  {
    return point.z <= ground_z_max_;
  }

  // 判断点是否属于障碍标记层。
  bool isObstaclePoint(const CloudPoint & point) const
  {
    return point.z >= obstacle_z_min_;
  }

  // 将点加入体素表，同一体素只保留第一枚点以降低点云密度。
  void addVoxel(VoxelMap & voxels, const CloudPoint & point) const
  {
    const VoxelKey key{
      static_cast<int>(std::floor(point.x / voxel_leaf_size_)),
      static_cast<int>(std::floor(point.y / voxel_leaf_size_)),
      static_cast<int>(std::floor(point.z / voxel_leaf_size_))};
    voxels.emplace(key, point);
  }

  // 发布体素点云，输出字段保持为标准 x/y/z float32。
  void publishVoxelCloud(
    const VoxelMap & voxels,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
    const builtin_interfaces::msg::Time & stamp) const
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = target_frame_;
    cloud.header.stamp = stamp;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(voxels.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    for (const auto & item : voxels) {
      *iter_x = item.second.x;
      *iter_y = item.second.y;
      *iter_z = item.second.z;
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }

    cloud.is_dense = false;
    publisher->publish(cloud);
  }

  // TF 或字段异常时发布空点云，避免 costmap 长时间使用旧输入。
  void publishEmptyCloud(const builtin_interfaces::msg::Time & stamp) const
  {
    VoxelMap empty;
    publishVoxelCloud(empty, obstacle_pub_, stamp);
    publishVoxelCloud(empty, ground_pub_, stamp);
  }

  std::string input_cloud_topic_;
  std::string obstacle_cloud_topic_;
  std::string ground_cloud_topic_;
  std::string target_frame_;
  bool use_latest_tf_{true};
  double transform_timeout_sec_{0.2};
  double voxel_leaf_size_{0.05};
  double min_x_{0.15};
  double max_x_{3.0};
  double max_abs_y_{1.5};
  double min_z_{0.0};
  double max_z_{1.5};
  double ground_z_max_{0.12};
  double obstacle_z_min_{0.10};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace go2_uwb_mppi_follow

// 启动双目点云过滤节点。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2_uwb_mppi_follow::StereoCloudFilterNode>());
  rclcpp::shutdown();
  return 0;
}
