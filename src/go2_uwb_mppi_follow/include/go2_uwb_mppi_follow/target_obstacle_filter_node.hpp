#pragma once

#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace go2_uwb_mppi_follow
{

struct TargetFilterConfig
{
  std::string input_cloud_topic;
  std::string output_cloud_topic;
  std::string target_topic;
  std::string filter_frame;
  double transform_timeout_sec;
  double clear_radius_m;
  double clear_z_min_m;
  double clear_z_max_m;
  bool pass_through_without_target;
  bool use_latest_tf;
};

struct TargetPoint
{
  rclcpp::Time stamp;
  std::string frame_id;
  geometry_msgs::msg::Point point;
};

class TargetObstacleFilterNode : public rclcpp::Node
{
public:
  // 构造目标障碍过滤节点，并初始化点云、目标点和 TF 接口。
  TargetObstacleFilterNode();

private:
  // 声明节点使用的所有可调 ROS 参数。
  void declareParameters();

  // 读取 ROS 参数并缓存到过滤配置中。
  void loadParameters();

  // 接收 UWB 目标点，并缓存为当前被跟随目标。
  void targetCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

  // 接收障碍点云，删除目标人附近的小圆柱点云后发布。
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // 将缓存的目标点转换到过滤坐标系。
  std::optional<geometry_msgs::msg::Point> targetInFilterFrame(const rclcpp::Time & stamp);

  // 判断过滤坐标系下的点是否位于目标清除圆柱内。
  bool pointInsideTargetCylinder(
    const geometry_msgs::msg::Point & point,
    const geometry_msgs::msg::Point & target) const;

  // 直接转发原始点云。
  void publishPassThrough(const sensor_msgs::msg::PointCloud2 & msg);

  TargetFilterConfig config_;
  std::optional<TargetPoint> latest_target_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // 命名空间 go2_uwb_mppi_follow
