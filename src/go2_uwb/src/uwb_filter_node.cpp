#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "uwb_aoa_pkg/msg/lib_aoa_robot_msg.hpp"


class UwbFilterNode : public rclcpp::Node
{
public:
  UwbFilterNode()
  : Node("uwb_filter_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/libAoa_robot_publisher");
    output_topic_ = declare_parameter<std::string>("output_topic", "/uwb/target_point");
    output_frame_ = declare_parameter<std::string>("output_frame", "uwb_link");

    publisher_ = create_publisher<geometry_msgs::msg::PointStamped>(output_topic_, 10);
    subscription_ = create_subscription<uwb_aoa_pkg::msg::LibAoaRobotMsg>(
      input_topic_,
      10,
      std::bind(&UwbFilterNode::uwbCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "UWB 有效坐标过滤已启动：%s -> %s",
      input_topic_.c_str(),
      output_topic_.c_str());
  }

private:
  // 收到 UWB 原始坐标后，只发布 state == 1 的主人位置。
  void uwbCallback(const uwb_aoa_pkg::msg::LibAoaRobotMsg::SharedPtr msg)
  {
    if (msg->state != 1) {
      // state 不等于 1 时认为主人坐标无效；不发布，让控制器用超时停车。
      return;
    }

    geometry_msgs::msg::PointStamped target;
    target.header.stamp = msg->header.stamp;
    target.header.frame_id = output_frame_;
    target.point.x = static_cast<double>(msg->x);
    target.point.y = static_cast<double>(msg->y);
    target.point.z = 0.0;

    publisher_->publish(target);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
  rclcpp::Subscription<uwb_aoa_pkg::msg::LibAoaRobotMsg>::SharedPtr subscription_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UwbFilterNode>());
  rclcpp::shutdown();
  return 0;
}
