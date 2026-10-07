#include <memory>
#include <mutex>
#include "braking_geometry.hpp"
#include "nav2_mppi_controller/critic_function.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace mppi::critics
{
// 将最终执行层的停车可执行性提前放入采样评分，避免反复产生将被末级拒收的高速转向。
class BrakingCritic : public CriticFunction
{
public:
  // 订阅相机证据地图和显式停车参数，不从仿真模型读取障碍。
  void initialize() override
  {
    auto get_param = parameters_handler_->getParamGetter(name_);
    get_param(weight_, "cost_weight", 1000.0f);
    get_param(length_, "robot_length", .7);
    get_param(width_, "robot_width", .32);
    get_param(continuation_, "continuation_distance", .6);
    auto node = parent_.lock();
    map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/follow_demo/local_geometry", rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
        const auto & info = message->info;
        if (message->header.frame_id != "odom" || info.width == 0 || info.height == 0 ||
          info.width > 1000 || info.height > 1000 ||
          !std::isfinite(info.origin.position.x) || !std::isfinite(info.origin.position.y) ||
          message->data.size() != info.width * info.height || message->data.size() > 1000000 ||
          !std::isfinite(info.resolution) || info.resolution <= 0 ||
          std::abs(info.origin.orientation.z) > 1e-9 || std::abs(info.origin.orientation.w - 1.) > 1e-9) {return;}
        std::vector<uint8_t> free(message->data.size());
        for (size_t i = 0; i < free.size(); ++i) {free[i] = message->data[i] == 0;}
        auto geometry = std::make_shared<go2_follow::Geometry>(free.data(), info.width, info.height,
          info.resolution, info.origin.position.x, info.origin.position.y, length_, width_);
        std::lock_guard<std::mutex> lock(mutex_);
        geometry_ = geometry;
        map_stamp_ = rclcpp::Time(message->header.stamp).seconds();
      });
    limits_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/follow_demo/braking_limits", rclcpp::QoS(1),
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr message) {
        const auto & d = message->data;
        if (d.size() != 4 || !std::all_of(d.begin(), d.end(), [](double v) {return std::isfinite(v);}) ||
          d[1] < .25 || d[1] > .750001 || d[2] <= .01 || d[3] <= .01) {return;}
        std::lock_guard<std::mutex> lock(mutex_);
        stamp_ = d[0]; hold_ = d[1]; deceleration_ = d[2]; angular_deceleration_ = d[3];
      });
  }

  // 候选前速、实测侧移及三种转速响应组合，与最终保护保持同一预测口径。
  void score(CriticData & data) override
  {
    if (!enabled_) {return;}
    std::shared_ptr<go2_follow::Geometry> geometry;
    double hold = .75, deceleration = .45, angular = 1.;
    const double now = parent_.lock()->now().seconds();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (now - map_stamp_ >= -.01 && now - map_stamp_ <= .6) {geometry = geometry_;}
      if (now - stamp_ >= -.01 && now - stamp_ <= .3) {
        hold = hold_; deceleration = deceleration_; angular = angular_deceleration_;
      }
    }
    const auto & q = data.state.pose.pose.orientation;
    const go2_follow::Pose pose{data.state.pose.pose.position.x, data.state.pose.pose.position.y,
      std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))};
    for (size_t row = 0; row < data.state.cvx.shape(0); ++row) {
      if (!geometry || data.state.pose.header.frame_id != "odom") {
        data.costs(row) += weight_; continue;
      }
      const double v = std::max(static_cast<double>(data.state.cvx(row, 0)),
        std::max(0., data.state.speed.linear.x));
      const double w = data.state.cwz(row, 0), measured = data.state.speed.angular.z;
      bool nominal = true;
      for (const double turn : {w, measured, .5 * (w + measured)}) {
        const auto result = geometry->brake(pose, v, turn, data.state.speed.linear.y,
          hold, deceleration, angular);
        if (!result.safe) {
          // 拒收越早惩罚越大，所有采样暂不可执行时仍保留减速/转向的优化梯度。
          data.costs(row) += weight_ * (3. + (result.duration - (result.rejected_step + 1) * .04) /
            std::max(.04, result.duration));
          break;
        }
        if (nominal && !geometry->seedable(result.stop, continuation_)) {
          // 当前机身不碰未知、能接入一两个姿态，仍可能困在没有续行出口的盲区口袋。
          // 优先选停车后尚有已知直行出口的候选；保持软评分，不扩大碰撞包络。
          data.costs(row) += weight_ * 2.;
        }
        nominal = false;
      }
    }
  }
private:
  float weight_{1000.};
  double length_{.7}, width_{.32}, stamp_{-1e9}, map_stamp_{-1e9};
  double continuation_{.6};
  double hold_{.75}, deceleration_{.45}, angular_deceleration_{1.};
  std::mutex mutex_;
  std::shared_ptr<go2_follow::Geometry> geometry_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr limits_sub_;
};
}
PLUGINLIB_EXPORT_CLASS(mppi::critics::BrakingCritic, mppi::critics::CriticFunction)
