#include <atomic>
#include <chrono>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <memory>
#include <string>
#include <thread>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "uwb_aoa_pkg/msg/lib_aoa_robot_msg.hpp"

extern "C" {
#include "uart_stack.h"
#include "uwb_robot_algo.h"
}

using namespace std::chrono_literals;

class LibAoaRobotPublisher : public rclcpp::Node
{
public:
  explicit LibAoaRobotPublisher(const std::string & dev_name)
  : Node("libAoa_robot_publisher"), running_(false), serial_port_(-1)
  {
    frame_id_ = this->declare_parameter<std::string>("frame_id", "uwb_link");
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 10.0);
    if (publish_rate_hz_ <= 0.0) {
      RCLCPP_WARN(
        this->get_logger(),
        "publish_rate_hz must be positive, using 10.0 Hz instead");
      publish_rate_hz_ = 10.0;
    }
    publish_period_ = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    publisher_ = this->create_publisher<uwb_aoa_pkg::msg::LibAoaRobotMsg>(
      "/libAoa_robot_publisher", 10);

    serial_port_ = open(dev_name.c_str(), O_RDWR | O_NOCTTY);
    if (serial_port_ < 0) {
      RCLCPP_FATAL(
        this->get_logger(), "failed to open UWB serial port %s: %s",
        dev_name.c_str(), std::strerror(errno));
      return;
    }

    struct termios tty;
    std::memset(&tty, 0, sizeof tty);
    if (tcgetattr(serial_port_, &tty) != 0) {
      RCLCPP_FATAL(
        this->get_logger(), "tcgetattr failed on %s: %s",
        dev_name.c_str(), std::strerror(errno));
      close(serial_port_);
      serial_port_ = -1;
      return;
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
      RCLCPP_FATAL(
        this->get_logger(), "tcsetattr failed on %s: %s",
        dev_name.c_str(), std::strerror(errno));
      close(serial_port_);
      serial_port_ = -1;
      return;
    }

    running_ = true;
    read_thread_ = std::thread(&LibAoaRobotPublisher::readSerialLoop, this);
    RCLCPP_INFO(
      this->get_logger(),
      "UWB serial publisher started on %s at %.2f Hz",
      dev_name.c_str(), publish_rate_hz_);
  }

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
  void readSerialLoop()
  {
    uint8_t read_buf = 0;

    struct input_data my_input;
    struct output my_output;
    std::memset(&my_input, 0, sizeof my_input);
    std::memset(&my_output, 0, sizeof my_output);

    my_input.aoa_frequency = 20;
    my_input.clean_buffer_time = 2;
    my_input.config_r = 0;
    my_input.config_rad = 0;
    my_input.direction = -1;
    my_input.stopband_rate = 0.1;
    my_input.State = 1;
    my_input.win_fil = 5;
    my_input.use_pitch_dis = 2;

    while (rclcpp::ok() && running_) {
      const int num_bytes = read(serial_port_, &read_buf, 1);
      if (num_bytes <= 0) {
        continue;
      }

      const int ret = uart_receive_byte(read_buf);
      if (ret != 1) {
        continue;
      }

      uwb_aoa_fob_pkg_t * c5 = nullptr;
      const uint8_t parse_ret = uart_protocol_packet_process(reinterpret_cast<void **>(&c5));
      if (parse_ret != NOTIFY_DISTANCE_ANGLE_RSSI_FOBID || c5 == nullptr) {
        continue;
      }

      const std::time_t now = std::time(nullptr);
      my_input.Distance = c5->distance;
      my_input.Azimuth = c5->angle;
      my_input.Pitch = c5->pitch;
      my_input.t = now;
      algo_uwb_aoa_merge(&my_input, &my_output);

      const auto steady_now = std::chrono::steady_clock::now();
      if (
        last_publish_time_ != std::chrono::steady_clock::time_point::min() &&
        steady_now - last_publish_time_ < publish_period_)
      {
        continue;
      }
      last_publish_time_ = steady_now;

      auto message = uwb_aoa_pkg::msg::LibAoaRobotMsg();
      message.header.stamp = this->now();
      message.header.frame_id = frame_id_;
      message.x = my_output.x;
      message.y = my_output.y;
      message.r = my_output.r;
      message.a = my_output.rad;
      message.state = my_output.state;
      for (size_t i = 0; i < message.rssi.size() && i < sizeof(c5->rssi); ++i) {
        message.rssi[i] = c5->rssi[i];
      }
      message.pos_confidence = c5->pos_confidence;
      message.sync_cnt = c5->sync_cnt;
      message.fob_id = c5->fob_id;
      message.fob_type = c5->fob_type;
      message.raw_distance = c5->distance;
      message.raw_angle = c5->angle;
      message.raw_pitch = c5->pitch;
      message.rx_power = c5->rx_power;
      message.rssi_fpp = c5->rssi_fpp;
      message.rssi_np = c5->rssi_np;
      message.rssi_ble = c5->rssi_ble;

      publisher_->publish(message);
    }
  }

  rclcpp::Publisher<uwb_aoa_pkg::msg::LibAoaRobotMsg>::SharedPtr publisher_;
  std::thread read_thread_;
  std::atomic<bool> running_;
  int serial_port_;
  std::string frame_id_;
  double publish_rate_hz_;
  std::chrono::duration<double> publish_period_{1.0};
  std::chrono::steady_clock::time_point last_publish_time_{
    std::chrono::steady_clock::time_point::min()};
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const std::string dev_name = argc > 1 ? argv[1] : "/dev/ttyUSB0";
  rclcpp::spin(std::make_shared<LibAoaRobotPublisher>(dev_name));
  rclcpp::shutdown();
  return 0;
}
