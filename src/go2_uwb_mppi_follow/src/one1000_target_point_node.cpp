#include <cmath>
#include <functional>
#include <optional>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "uwb_aoa_pkg/msg/lib_aoa_robot_msg.hpp"

namespace
{

using LibAoaRobotMsg = uwb_aoa_pkg::msg::LibAoaRobotMsg;

// 判断浮点数是否为有效有限值。
bool finiteValue(const double value)
{
  return std::isfinite(value);
}

// 判断点的三个坐标是否都有效。
bool finitePoint(const geometry_msgs::msg::Point & point)
{
  return finiteValue(point.x) && finiteValue(point.y) && finiteValue(point.z);
}

// 根据二维坐标构造 z 为 0 的目标点。
geometry_msgs::msg::Point makePoint(const double x, const double y)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = 0.0;
  return point;
}

}  // namespace

namespace go2_uwb_mppi_follow
{

class One1000TargetPointNode : public rclcpp::Node
{
public:
  // 构造 One1000 目标转换节点，并初始化订阅、发布和 TF。
  One1000TargetPointNode()
  : Node("one1000_target_point_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declareParameters();
    loadParameters();

    sub_ = create_subscription<LibAoaRobotMsg>(
      one1000_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&One1000TargetPointNode::uwbCallback, this, std::placeholders::_1));
    pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      target_point_topic_,
      rclcpp::SensorDataQoS());

    RCLCPP_WARN(
      get_logger(),
      "one1000_target_point_node started: %s -> %s (%s)",
      one1000_topic_.c_str(),
      target_point_topic_.c_str(),
      target_frame_.c_str());
  }

private:
  // 声明 One1000 解析、状态判断和 TF 转换参数。
  void declareParameters()
  {
    declare_parameter<std::string>("one1000_topic", "/libAoa_robot_publisher");
    declare_parameter<std::string>("target_point_topic", "/uwb/target_point");
    declare_parameter<std::string>("target_frame", "base_footprint");
    declare_parameter<std::string>("one1000_frame", "base_link");
    declare_parameter<bool>("use_tf", true);
    declare_parameter<bool>("require_tf", true);
    declare_parameter<bool>("use_latest_tf", true);
    declare_parameter<double>("transform_timeout_sec", 0.2);

    declare_parameter<bool>("prefer_xy", true);
    declare_parameter<bool>("angle_in_degrees", false);
    declare_parameter<bool>("invert_y", false);
    declare_parameter<double>("angle_offset_rad", 0.0);
    declare_parameter<double>("anchor_x_offset", 0.0);
    declare_parameter<double>("anchor_y_offset", 0.0);
    declare_parameter<bool>("require_valid_state", true);
    declare_parameter<double>("min_target_distance_m", 0.0);
    declare_parameter<double>("max_target_distance_m", 8.0);
  }

  // 读取 ROS 参数并缓存到节点成员变量。
  void loadParameters()
  {
    one1000_topic_ = get_parameter("one1000_topic").as_string();
    target_point_topic_ = get_parameter("target_point_topic").as_string();
    target_frame_ = get_parameter("target_frame").as_string();
    one1000_frame_ = get_parameter("one1000_frame").as_string();
    use_tf_ = get_parameter("use_tf").as_bool();
    require_tf_ = get_parameter("require_tf").as_bool();
    use_latest_tf_ = get_parameter("use_latest_tf").as_bool();
    transform_timeout_sec_ = get_parameter("transform_timeout_sec").as_double();

    prefer_xy_ = get_parameter("prefer_xy").as_bool();
    angle_in_degrees_ = get_parameter("angle_in_degrees").as_bool();
    invert_y_ = get_parameter("invert_y").as_bool();
    angle_offset_rad_ = get_parameter("angle_offset_rad").as_double();
    anchor_x_offset_ = get_parameter("anchor_x_offset").as_double();
    anchor_y_offset_ = get_parameter("anchor_y_offset").as_double();
    require_valid_state_ = get_parameter("require_valid_state").as_bool();
    min_target_distance_m_ = get_parameter("min_target_distance_m").as_double();
    max_target_distance_m_ = get_parameter("max_target_distance_m").as_double();
  }

  // 接收 One1000 原始消息，只按 state 判断状态后发布目标点。
  void uwbCallback(const LibAoaRobotMsg::SharedPtr msg)
  {
    if (require_valid_state_ && msg->state < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "reject: invalid One1000 state");
      return;
    }

    const auto parsed = parseTarget(*msg);
    if (!parsed) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "reject: One1000 target parse failed");
      return;
    }

    const rclcpp::Time stamp = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ?
      now() : rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
    const std::string source_frame = msg->header.frame_id.empty() ?
      one1000_frame_ : msg->header.frame_id;

    geometry_msgs::msg::PointStamped point;
    point.header.stamp = stamp;
    point.header.frame_id = source_frame;
    point.point = *parsed;

    if (use_tf_ && source_frame != target_frame_) {
      const auto transformed = transformToTargetFrame(point);
      if (!transformed) {
        if (require_tf_) {
          return;
        }
        point.header.frame_id = target_frame_;
      } else {
        point = *transformed;
      }
    } else {
      point.header.frame_id = target_frame_;
    }

    point.header.stamp = stamp;
    point.point.z = 0.0;
    pub_->publish(point);
  }

  // 从 One1000 消息中解析 x/y 或 r/a 目标坐标。
  std::optional<geometry_msgs::msg::Point> parseTarget(const LibAoaRobotMsg & msg) const
  {
    const bool xy_valid = finiteValue(msg.x) && finiteValue(msg.y);
    const bool range_angle_valid = finiteValue(msg.r) && finiteValue(msg.a) && msg.r > 1e-6;

    if (prefer_xy_ && xy_valid && std::hypot(msg.x, msg.y) > 1e-6) {
      return validatePoint(makePoint(
        msg.x + anchor_x_offset_,
        (invert_y_ ? -msg.y : msg.y) + anchor_y_offset_));
    }

    if (range_angle_valid) {
      double angle = msg.a;
      if (angle_in_degrees_) {
        angle *= 3.14159265358979323846 / 180.0;
      }
      angle += angle_offset_rad_;
      const double y = msg.r * std::sin(angle);
      return validatePoint(makePoint(
        msg.r * std::cos(angle) + anchor_x_offset_,
        (invert_y_ ? -y : y) + anchor_y_offset_));
    }

    if (xy_valid) {
      return validatePoint(makePoint(
        msg.x + anchor_x_offset_,
        (invert_y_ ? -msg.y : msg.y) + anchor_y_offset_));
    }

    return std::nullopt;
  }

  // 校验目标点是否有限且距离在允许范围内。
  std::optional<geometry_msgs::msg::Point> validatePoint(
    const geometry_msgs::msg::Point & point) const
  {
    if (!finitePoint(point)) {
      return std::nullopt;
    }

    const double distance = std::hypot(point.x, point.y);
    if (distance < min_target_distance_m_ || distance > max_target_distance_m_) {
      return std::nullopt;
    }

    return point;
  }

  // 将目标点转换到配置的目标坐标系。
  std::optional<geometry_msgs::msg::PointStamped> transformToTargetFrame(
    const geometry_msgs::msg::PointStamped & point)
  {
    geometry_msgs::msg::PointStamped source = point;
    if (use_latest_tf_) {
      source.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    }

    try {
      auto transformed = tf_buffer_.transform(
        source,
        target_frame_,
        tf2::durationFromSec(transform_timeout_sec_));
      transformed.header.frame_id = target_frame_;
      return transformed;
    } catch (const tf2::TransformException & exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "One1000 target TF failed from %s to %s: %s",
        point.header.frame_id.c_str(),
        target_frame_.c_str(),
        exc.what());
      return std::nullopt;
    }
  }

  std::string one1000_topic_;
  std::string target_point_topic_;
  std::string target_frame_;
  std::string one1000_frame_;
  bool use_tf_{true};
  bool require_tf_{true};
  bool use_latest_tf_{true};
  double transform_timeout_sec_{0.2};

  bool prefer_xy_{true};
  bool angle_in_degrees_{false};
  bool invert_y_{false};
  double angle_offset_rad_{0.0};
  double anchor_x_offset_{0.0};
  double anchor_y_offset_{0.0};
  bool require_valid_state_{true};
  double min_target_distance_m_{0.0};
  double max_target_distance_m_{8.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<LibAoaRobotMsg>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;
};

}  // namespace go2_uwb_mppi_follow

// 启动 One1000 目标转换节点。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2_uwb_mppi_follow::One1000TargetPointNode>());
  rclcpp::shutdown();
  return 0;
}
