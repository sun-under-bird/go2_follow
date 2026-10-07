#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_mppi_controller/critic_function.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mppi::critics
{
// 在原MPPI优化器中加入速度目标，保留地图、路径与碰撞评分的联合决策。
class FollowSpeedCritic : public CriticFunction
{
public:
  // 建立带时间戳的速度参考订阅，过期目标不会继续催促前进。
  void initialize() override
  {
    auto get_param = parameters_handler_->getParamGetter(name_);
    get_param(weight_, "cost_weight", 18.0f);
    get_param(timeout_, "reference_timeout", 0.30);
    get_param(maximum_, "maximum_speed", 0.8f);
    get_param(decay_, "reference_decay_seconds", 0.6f);
    std::string topic;
    get_param(topic, "reference_topic", std::string("/follow_demo/reference_speed"));
    subscription_ = parent_.lock()->create_subscription<geometry_msgs::msg::TwistStamped>(
      topic, rclcpp::QoS(1), [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr message) {
        const auto speed = message->twist.linear.x;
        if (message->header.frame_id != "base_footprint" || !std::isfinite(speed) ||
          speed < 0.0 || speed > maximum_ + 1e-6) {return;}
        std::lock_guard<std::mutex> lock(mutex_);
        reference_ = static_cast<float>(speed);
        stamp_ = rclcpp::Time(message->header.stamp).seconds();
        received_ = true;
      });
  }

  // 在同一批候选轨迹中联合选择速度与转向，不拼接两条轨迹的vx和wz。
  void score(CriticData & data) override
  {
    if (!enabled_) {return;}
    float desired = 0.0f;
    const double now = parent_.lock()->now().seconds();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const double age = now - stamp_;
      if (received_ && age >= -0.01 && age <= timeout_) {desired = reference_;}
    }
    const auto count = data.state.vx.shape(1);
    if (count < 2) {return;}
    std::vector<float> weights(count, 0.0f);
    float total = 0.0f;
    for (size_t column = 1; column < count; ++column) {
      // 参考是当前速度需求，不能强迫整段2.4s预测保持同速，阻止末端/转角减速。
      weights[column] = std::exp(-static_cast<float>(column - 1) * data.model_dt /
        std::max(0.05f, decay_));
      total += weights[column];
    }
    for (size_t row = 0; row < data.state.vx.shape(0); ++row) {
      float penalty = 0.0f;
      // 第一列是无法立即改变的实测速度，后续列才是优化变量。
      for (size_t column = 1; column < count; ++column) {
        penalty += weights[column] * std::abs(data.state.vx(row, column) - desired);
      }
      data.costs(row) += weight_ * penalty / total;
    }
  }

private:
  float weight_{18.0f}, maximum_{0.8f}, reference_{0.0f}, decay_{0.6f};
  double timeout_{0.30}, stamp_{0.0};
  bool received_{false};
  std::mutex mutex_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_;
};
}  // namespace mppi::critics
PLUGINLIB_EXPORT_CLASS(mppi::critics::FollowSpeedCritic, mppi::critics::CriticFunction)
