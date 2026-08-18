#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "uwb_aoa_pkg/msg/lib_aoa_robot_msg.hpp"


class UwbFilterNode : public rclcpp::Node
{
public:
  // 初始化原始 UWB 坐标适配和到机器人基座坐标系的 TF 转换。
  UwbFilterNode()
  : Node("uwb_filter_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/libAoa_robot_publisher");
    output_topic_ = declare_parameter<std::string>("output_topic", "/uwb/target_point");
    input_frame_ = declare_parameter<std::string>("input_frame", "base_footprint");
    output_frame_ = declare_parameter<std::string>("output_frame", "base_footprint");
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.2);

    publisher_ = create_publisher<geometry_msgs::msg::PointStamped>(output_topic_, 10);
    subscription_ = create_subscription<uwb_aoa_pkg::msg::LibAoaRobotMsg>(
      input_topic_,
      10,
      std::bind(&UwbFilterNode::uwbCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "UWB 原始坐标适配已启动：%s -> %s",
      input_topic_.c_str(),
      output_topic_.c_str());
  }

private:
  // 直接使用原始坐标，仅在发布前转换到配置的基座坐标系。
  void uwbCallback(const uwb_aoa_pkg::msg::LibAoaRobotMsg::SharedPtr msg)
  {
    geometry_msgs::msg::PointStamped source;
    source.header = msg->header;
    if (source.header.frame_id.empty()) {
      source.header.frame_id = input_frame_;
    }
    if (source.header.stamp.sec == 0 && source.header.stamp.nanosec == 0) {
      source.header.stamp = now();
    }
    source.point.x = static_cast<double>(msg->x);
    source.point.y = static_cast<double>(msg->y);
    source.point.z = 0.0;

    try {
      const auto target = source.header.frame_id == output_frame_ ?
        source : tf_buffer_.transform(
        source,
        output_frame_,
        tf2::durationFromSec(transform_timeout_sec_));
      publisher_->publish(target);
    } catch (const tf2::TransformException & exc) {
      // TF 缺失时禁止只改 frame_id，避免错误坐标进入速度控制器。
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "UWB target TF failed from %s to %s: %s",
        source.header.frame_id.c_str(), output_frame_.c_str(), exc.what());
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string input_frame_;
  std::string output_frame_;
  double transform_timeout_sec_{0.2};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
  rclcpp::Subscription<uwb_aoa_pkg::msg::LibAoaRobotMsg>::SharedPtr subscription_;
};


// 初始化并运行 UWB 原始坐标适配节点。
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UwbFilterNode>());
  rclcpp::shutdown();
  return 0;
}
