#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <uwb_aoa_pkg/msg/lib_aoa_robot_msg.hpp>

#include "common_types.hpp"
#include "follow_avoid_controller.hpp"

class RobotNexusNode : public rclcpp::Node {
public:
    RobotNexusNode() : Node("robot_nexus") {
        const auto config = loadConfig();
        active_ = config.active;
        target_timeout_sec_ = config.target_timeout_sec;
        scan_timeout_sec_ = config.scan_timeout_sec;

        const std::string scan_topic = this->get_parameter("scan_topic").as_string();
        const std::string uwb_target_topic = this->get_parameter("uwb_target_topic").as_string();
        const std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
        target_frame_ = this->get_parameter("target_frame").as_string();
        uwb_input_frame_ = this->get_parameter("uwb_input_frame").as_string();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        controller_ = std::make_unique<FollowAvoidController>(shared_state_, config);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 1);
        controller_->setVelocityCallback([this](const geometry_msgs::msg::Twist& cmd) {
            cmd_vel_pub_->publish(cmd);
        });

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&RobotNexusNode::scanCallback, this, std::placeholders::_1));

        uwb_target_sub_ = this->create_subscription<uwb_aoa_pkg::msg::LibAoaRobotMsg>(
            uwb_target_topic,
            10,
            std::bind(&RobotNexusNode::uwbTargetCallback, this, std::placeholders::_1));

        watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RobotNexusNode::watchdogCallback, this));

        RCLCPP_INFO(
            this->get_logger(),
            "robot_nexus started: scan=%s, uwb_target=%s, cmd_vel=%s, follow_dist=%.2f, target_frame=%s",
            scan_topic.c_str(),
            uwb_target_topic.c_str(),
            cmd_vel_topic.c_str(),
            config.follow_dist,
            target_frame_.c_str());
    }

private:
    SharedState shared_state_;
    std::unique_ptr<FollowAvoidController> controller_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<uwb_aoa_pkg::msg::LibAoaRobotMsg>::SharedPtr uwb_target_sub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    bool active_ = true;
    double target_timeout_sec_ = 0.5;
    double scan_timeout_sec_ = 0.5;
    bool has_scan_ = false;
    std::chrono::steady_clock::time_point last_scan_time_{};
    std::string target_frame_ = "base_footprint";
    std::string uwb_input_frame_ = "uwb_link";

    // Declare ROS parameters and copy them into the controller config.
    FollowConfig loadConfig() {
        this->declare_parameter<bool>("active", true);
        this->declare_parameter<std::string>("scan_topic", "/scan");
        this->declare_parameter<std::string>("uwb_target_topic", "/uwb_target");
        this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_safe");
        this->declare_parameter<std::string>("target_frame", "base_footprint");
        this->declare_parameter<std::string>("uwb_input_frame", "uwb_link");
        this->declare_parameter<double>("follow_dist", 1.0);
        this->declare_parameter<double>("target_timeout_sec", 0.5);
        this->declare_parameter<double>("scan_timeout_sec", 0.5);
        this->declare_parameter<double>("target_exclusion_radius", 0.35);
        this->declare_parameter<double>("apf_influence_dist", 0.3);
        this->declare_parameter<double>("apf_slowdown_dist", 0.3);
        this->declare_parameter<double>("apf_emergency_dist", 0.2);
        this->declare_parameter<double>("apf_repulse_gain", 0.01);
        this->declare_parameter<double>("max_linear_speed", 0.5);
        this->declare_parameter<double>("max_lateral_speed", 0.12);
        this->declare_parameter<double>("max_angular_speed", 1.0);
        this->declare_parameter<double>("linear_scale_factor", 0.5);
        this->declare_parameter<double>("linear_y_scale_factor", 1.0);
        this->declare_parameter<double>("angular_scale_factor", 1.0);
        this->declare_parameter<double>("distance_deadband", 0.05);
        this->declare_parameter<double>("lateral_deadband", 0.03);
        this->declare_parameter<double>("angle_deadband", 0.08);
        this->declare_parameter<double>("rotate_only_angle", 0.45);
        this->declare_parameter<double>("min_forward_speed", 0.06);
        this->declare_parameter<double>("rectangle_width", 0.35);

        FollowConfig config;
        config.active = this->get_parameter("active").as_bool();
        config.follow_dist = this->get_parameter("follow_dist").as_double();
        config.target_timeout_sec = this->get_parameter("target_timeout_sec").as_double();
        config.scan_timeout_sec = this->get_parameter("scan_timeout_sec").as_double();
        config.target_exclusion_radius = this->get_parameter("target_exclusion_radius").as_double();
        config.apf_influence_dist = this->get_parameter("apf_influence_dist").as_double();
        config.apf_slowdown_dist = this->get_parameter("apf_slowdown_dist").as_double();
        config.apf_emergency_dist = this->get_parameter("apf_emergency_dist").as_double();
        config.apf_repulse_gain = this->get_parameter("apf_repulse_gain").as_double();
        config.max_linear_speed = this->get_parameter("max_linear_speed").as_double();
        config.max_lateral_speed = this->get_parameter("max_lateral_speed").as_double();
        config.max_angular_speed = this->get_parameter("max_angular_speed").as_double();
        config.linear_scale_factor = this->get_parameter("linear_scale_factor").as_double();
        config.linear_y_scale_factor = this->get_parameter("linear_y_scale_factor").as_double();
        config.angular_scale_factor = this->get_parameter("angular_scale_factor").as_double();
        config.distance_deadband = this->get_parameter("distance_deadband").as_double();
        config.lateral_deadband = this->get_parameter("lateral_deadband").as_double();
        config.angle_deadband = this->get_parameter("angle_deadband").as_double();
        config.rotate_only_angle = this->get_parameter("rotate_only_angle").as_double();
        config.min_forward_speed = this->get_parameter("min_forward_speed").as_double();
        config.rectangle_width = this->get_parameter("rectangle_width").as_double();

        return config;
    }

    // Handle the converted stereo scan and let the controller compute /cmd_vel_safe.
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        last_scan_time_ = std::chrono::steady_clock::now();
        has_scan_ = true;
        controller_->processScan(scan_msg);
    }

    // Transform the custom UWB message into the configured planar follow frame.
    void uwbTargetCallback(const uwb_aoa_pkg::msg::LibAoaRobotMsg::SharedPtr msg) {
        if (!msg || !std::isfinite(msg->x) || !std::isfinite(msg->y)) {
            RCLCPP_WARN(this->get_logger(), "Ignored invalid UWB target");
            return;
        }
        if (msg->state != 1) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Ignored inactive UWB target: state=%d",
                msg->state);
            return;
        }

        geometry_msgs::msg::PointStamped uwb_target;
        uwb_target.header = msg->header;
        uwb_target.point.x = msg->x;
        uwb_target.point.y = msg->y;
        uwb_target.point.z = 0.0;
        if (uwb_target.header.frame_id.empty()) {
            uwb_target.header.frame_id = uwb_input_frame_;
        }

        try {
            const auto base_target = tf_buffer_->transform(
                uwb_target,
                target_frame_,
                tf2::durationFromSec(0.05));
            shared_state_.setTarget(base_target.point.x, base_target.point.y);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Failed to transform UWB target from '%s' to '%s': %s",
                uwb_target.header.frame_id.c_str(),
                target_frame_.c_str(),
                ex.what());
        }
    }

    // Stop the robot when target or scan data becomes stale.
    void watchdogCallback() {
        double target_x = 0.0;
        double target_y = 0.0;
        double target_age = 0.0;
        const bool target_ok = shared_state_.getTarget(target_x, target_y, target_age) &&
            target_age <= target_timeout_sec_;

        const bool scan_ok = has_scan_ &&
            std::chrono::duration<double>(std::chrono::steady_clock::now() - last_scan_time_).count() <=
                scan_timeout_sec_;

        if (!active_ || !target_ok || !scan_ok) {
            publishStop();
        }
    }

    // Publish and cache a zero velocity command.
    void publishStop() {
        geometry_msgs::msg::Twist stop;
        shared_state_.setVelocity(0.0, 0.0, 0.0);
        cmd_vel_pub_->publish(stop);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotNexusNode>());
    rclcpp::shutdown();
    return 0;
}
