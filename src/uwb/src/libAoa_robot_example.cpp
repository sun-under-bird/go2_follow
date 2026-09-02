#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

extern "C"
{
#include "uart_stack.h"
#include "uwb_robot_algo.h"
}

#include "rclcpp/rclcpp.hpp"
#include "uwb_aoa_pkg/msg/lib_aoa_robot_msg.hpp"

namespace uwb_aoa_pkg
{

class LibAoaRobotPublisher : public rclcpp::Node
{
public:
  // 初始化厂家融合参数、串口和原始 UWB 消息发布线程。
  explicit LibAoaRobotPublisher(const std::string & device_name)
  : Node("libAoa_robot_publisher")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "uwb_link");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    aoa_frequency_hz_ = declare_parameter<int>("aoa_frequency_hz", 10);
    if (frame_id_.empty()) {
      throw std::invalid_argument("frame_id must not be empty");
    }
    if (publish_rate_hz_ <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be positive");
    }
    if (aoa_frequency_hz_ <= 0) {
      throw std::invalid_argument("aoa_frequency_hz must be positive");
    }
    publish_period_ = std::chrono::duration<double>(1.0 / publish_rate_hz_);

    publisher_ = create_publisher<msg::LibAoaRobotMsg>("/libAoa_robot_publisher", 10);
    openAndConfigureSerial(device_name);
    running_ = true;
    read_thread_ = std::thread(&LibAoaRobotPublisher::readSerialLoop, this);
    RCLCPP_INFO(get_logger(), "UWB serial driver started on %s", device_name.c_str());
  }

  // 停止读取线程并关闭串口，避免节点退出后继续占用设备。
  ~LibAoaRobotPublisher() override
  {
    running_ = false;
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
    if (serial_port_ >= 0) {
      close(serial_port_);
    }
  }

private:
  // 打开 115200-8N1 串口并切换到原始字节读取模式。
  void openAndConfigureSerial(const std::string & device_name)
  {
    serial_port_ = open(device_name.c_str(), O_RDWR);
    if (serial_port_ < 0) {
      throw std::runtime_error(
              "cannot open UWB serial port " + device_name + ": " + std::strerror(errno));
    }

    struct termios tty {};
    if (tcgetattr(serial_port_, &tty) != 0) {
      const std::string reason = std::strerror(errno);
      close(serial_port_);
      serial_port_ = -1;
      throw std::runtime_error("cannot read UWB serial attributes: " + reason);
    }

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;
    tty.c_cc[VTIME] = 10;
    tty.c_cc[VMIN] = 0;
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
      const std::string reason = std::strerror(errno);
      close(serial_port_);
      serial_port_ = -1;
      throw std::runtime_error("cannot configure UWB serial port: " + reason);
    }
  }

  // 连续解析 C5 数据包，经厂家算法融合后发布机器人平面目标坐标。
  void readSerialLoop()
  {
    msg::LibAoaRobotMsg message;
    struct input_data algorithm_input {};
    struct output algorithm_output {};
    const auto algorithm_start_time = std::chrono::steady_clock::now();
    algorithm_input.aoa_frequency = aoa_frequency_hz_;
    algorithm_input.clean_buffer_time = 2;
    algorithm_input.config_r = 0;
    algorithm_input.config_rad = 0;
    algorithm_input.direction = -1;
    algorithm_input.stopband_rate = 0.1;
    algorithm_input.State = 1;
    algorithm_input.win_fil = 5;
    algorithm_input.use_pitch_dis = 0;

    std::uint8_t input_byte = 0;
    while (rclcpp::ok() && running_) {
      // 使用短超时轮询，保证串口无数据时节点也能及时响应退出和重启。
      struct pollfd descriptor {serial_port_, POLLIN, 0};
      const int poll_result = poll(&descriptor, 1, 200);
      if (poll_result <= 0) {
        continue;
      }
      if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      if ((descriptor.revents & POLLIN) == 0) {
        continue;
      }
      const ssize_t bytes_read = read(serial_port_, &input_byte, 1);
      if (bytes_read <= 0 || uart_receive_byte(input_byte) != 1) {
        continue;
      }

      uwb_aoa_fob_pkg_t * packet = nullptr;
      const std::uint8_t packet_type = uart_protocol_packet_process(
        reinterpret_cast<void **>(&packet));
      if (packet_type != NOTIFY_DISTANCE_ANGLE_RSSI_FOBID || packet == nullptr) {
        continue;
      }

      algorithm_input.Distance = packet->distance;
      algorithm_input.Azimuth = packet->angle;
      algorithm_input.Pitch = packet->pitch;
      const auto steady_now = std::chrono::steady_clock::now();
      const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        steady_now - algorithm_start_time).count();
      algorithm_input.t = static_cast<std::uint32_t>(elapsed_ms);
      algo_uwb_aoa_merge(&algorithm_input, &algorithm_output);

      // 预留 10% 出包周期容差，避免轻微抖动导致发布频率意外减半。
      if (have_publish_time_ && steady_now - last_publish_time_ < publish_period_ * 0.9) {
        continue;
      }
      have_publish_time_ = true;
      last_publish_time_ = steady_now;

      message.header.stamp = now();
      message.header.frame_id = frame_id_;
      message.r = algorithm_output.r;
      message.a = algorithm_output.rad;
      message.x = algorithm_output.x;
      message.y = algorithm_output.y;
      message.state = algorithm_output.state;
      for (std::size_t index = 0; index < message.rssi.size(); ++index) {
        message.rssi[index] = packet->rssi[index];
      }
      message.pos_confidence = packet->pos_confidence;
      message.sync_cnt = packet->sync_cnt;
      message.fob_id = packet->fob_id;
      message.fob_type = packet->fob_type;
      message.raw_distance = packet->distance;
      message.raw_angle = packet->angle;
      message.raw_pitch = packet->pitch;
      message.rx_power = packet->rx_power;
      message.rssi_fpp = packet->rssi_fpp;
      message.rssi_np = packet->rssi_np;
      message.rssi_ble = packet->rssi_ble;
      publisher_->publish(message);
    }
  }

  rclcpp::Publisher<msg::LibAoaRobotMsg>::SharedPtr publisher_;
  std::thread read_thread_;
  std::atomic<bool> running_{false};
  int serial_port_{-1};
  std::string frame_id_;
  double publish_rate_hz_{10.0};
  int aoa_frequency_hz_{10};
  std::chrono::duration<double> publish_period_{0.1};
  bool have_publish_time_{false};
  std::chrono::steady_clock::time_point last_publish_time_{};
};

}  // namespace uwb_aoa_pkg

// 启动 UWB 串口驱动，并在初始化失败时以非零状态退出。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    const std::string device_name = argc > 1 ? argv[1] : "/dev/ttyUSB0";
    rclcpp::spin(std::make_shared<uwb_aoa_pkg::LibAoaRobotPublisher>(device_name));
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("libAoa_robot_publisher"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
