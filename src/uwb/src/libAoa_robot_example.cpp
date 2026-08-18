#include <cstdio>
extern "C"{
  #include "uwb_robot_algo.h"
  #include "uart_stack.h"
}

#include <atomic>
#include <chrono>
#include <cstring>
#include <ctime>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

#include "rclcpp/rclcpp.hpp"
#include "uwb_aoa_pkg/msg/lib_aoa_robot_msg.hpp"

using namespace std::chrono_literals;

class libAoa_robot_publisher : public rclcpp::Node
{
  public:
    libAoa_robot_publisher(const std::string & dev_name)
    : Node("libAoa_robot_publisher"),running_(false),serial_port_(-1)
    {
      frame_id_ = this->declare_parameter<std::string>("frame_id", "uwb_link");
      publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 10.0);
      if (publish_rate_hz_ <= 0.0) {
        publish_rate_hz_ = 10.0;
      }
      publish_period_ = std::chrono::duration<double>(1.0 / publish_rate_hz_);

      publisher_ = this->create_publisher<uwb_aoa_pkg::msg::LibAoaRobotMsg>("/libAoa_robot_publisher", 10);

	  {

		serial_port_ = open(dev_name.c_str(), O_RDWR); // linux
		if (serial_port_ < 0)
		{
			RCLCPP_FATAL(this->get_logger(),"can't open port %s: %s", dev_name.c_str(), strerror(errno));
			return;
		}
		struct termios tty;
		if (tcgetattr(serial_port_, &tty) != 0)
		{
			printf("Error! Check uart connection/permission!\n");
			printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
			close(serial_port_);
			serial_port_ = -1;
			return;
		}

		tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
		tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
		tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
		tty.c_cflag |= CS8;            // 8 bits per byte (most common)
		tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
		tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

		tty.c_lflag &= ~ICANON;
		tty.c_lflag &= ~ECHO;                                                        // Disable echo
		tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
		tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
		tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
		tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
		tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

		tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
		tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

		tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
		tty.c_cc[VMIN] = 0;

		// Set in/out baud rate to be 115200
		cfsetispeed(&tty, B115200);
		cfsetospeed(&tty, B115200);

		// Save tty settings, also checking for error
		if (tcsetattr(serial_port_, TCSANOW, &tty) != 0)
		{
			printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
			close(serial_port_);
			serial_port_ = -1;
			return;
		}

		RCLCPP_INFO(this->get_logger(),"open port success: %d\n",serial_port_);
	  }

	  running_ = true;
	  read_thread_ = std::thread(&libAoa_robot_publisher::readSerialLoop,this);
	  RCLCPP_INFO(this->get_logger(),"start serial publish");
    }

	~libAoa_robot_publisher(){
		running_ = false;
		if (read_thread_.joinable()) read_thread_.join();
		if (serial_port_ >= 0) close(serial_port_);
	}

  private:

	void readSerialLoop(){

		auto message = uwb_aoa_pkg::msg::LibAoaRobotMsg();

		uint8_t read_buf[2];
		memset(&read_buf, '\0', sizeof(read_buf));

		struct input_data my_input;
		struct output my_output;

		my_input.aoa_frequency = 20;
		my_input.clean_buffer_time = 2;
		my_input.config_r = 0;
		my_input.config_rad = 0; // 角度偏移
		my_input.direction = -1;
		my_input.stopband_rate = 0.1;
		my_input.t = 0;
		my_input.Azimuth = 0;
		my_input.Distance = 0;
		my_input.Pitch = 0;
		my_input.State = 1;
		my_input.StationXOffset = 0;
		my_input.StationYOffset = 0;
		my_input.win_fil = 5;
		my_input.use_pitch_dis = 0;

		my_output.r = 0; my_output.rad = 0; my_output.state = 0;
		my_output.x = 0; my_output.y = 0;

		while (rclcpp::ok() && running_){
			int num_bytes = read(serial_port_, &read_buf, 1);
			if (num_bytes > 0)
			{
				int ret = uart_receive_byte(read_buf[0]);
				if (ret == 1) {
					uwb_aoa_fob_pkg_t *c5 = NULL;
					uint8_t parse_ret = uart_protocol_packet_process((void **)&c5);
					if (parse_ret == 0xc5 && c5 != NULL) {

						time_t start_time = time(NULL);

						my_input.Distance = c5->distance; // update dis
						my_input.Azimuth = c5->angle; //update azi
						my_input.Pitch = c5->pitch; // update pitch
						my_input.t = start_time; // update time(uint32)
						algo_uwb_aoa_merge(&my_input, &my_output);

						const auto steady_now = std::chrono::steady_clock::now();
						// 留 10% 容差，避免出包间隔略小于目标周期时每两包丢一包。
						// Allow 10% slack so a packet interval just under the period is not halved.
						if (last_publish_time_ != std::chrono::steady_clock::time_point::min() &&
							steady_now - last_publish_time_ < publish_period_ * 0.9)
						{
							continue;
						}
						last_publish_time_ = steady_now;

						message.header.stamp = this->now();
						message.header.frame_id = frame_id_;

						message.x = my_output.x;
						message.y = my_output.y;
						message.r = my_output.r;
						message.a = my_output.rad;
						message.state = my_output.state;

						message.rssi[0] = c5->rssi[0];
						message.rssi[1] = c5->rssi[1];
						message.rssi[2] = c5->rssi[2];
						message.rssi[3] = c5->rssi[3];
						message.rssi[4] = c5->rssi[4];
						message.rssi[5] = c5->rssi[5];

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
			}
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
  	rclcpp::spin(std::make_shared<libAoa_robot_publisher>(dev_name));
 	rclcpp::shutdown();

	return 0;
}
