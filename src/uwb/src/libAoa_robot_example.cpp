#include <cstdio>
extern "C"{
  #include "uwb_robot_algo.h"
  #include "uart_stack.h"
}

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

#include "rclcpp/rclcpp.hpp"
#include "uwb_aoa_pkg/lib_aoa_robot_msg.hpp"

using namespace std::chrono_literals;

class libAoa_robot_publisher : public rclcpp::Node
{
  public:
    libAoa_robot_publisher(const char* dev_name)
    : Node("libAoa_robot_publisher"),running_(false)
    {

      publisher_ = this->create_publisher<uwb_aoa_pkg::msg::LibAoaRobotMsg>("libAoa_robot_publisher", 10);
      
	  try 
	  {

		serial_port_ = open(dev_name, O_RDWR); // linux
		struct termios tty;
		if (tcgetattr(serial_port_, &tty) != 0)
		{
			printf("Error! Check uart connection/permission!\n");
			printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
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
		// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
		// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

		tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
		tty.c_cc[VMIN] = 0;

		// Set in/out baud rate to be 9600
		cfsetispeed(&tty, B115200);
		cfsetospeed(&tty, B115200);

		// Save tty settings, also checking for error
		if (tcsetattr(serial_port_, TCSANOW, &tty) != 0)
		{
			printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
			return;
		}

		RCLCPP_INFO(this->get_logger(),"open port success: %d\n",serial_port_);
	  } 
	  catch (const std::exception& e)
	  {
		RCLCPP_FATAL(this->get_logger(),"can't open port %s",e.what());
		return;
	  }

	  running_ = true;
	  read_thread_ = std::thread(&libAoa_robot_publisher::readSerialLoop,this);
	  RCLCPP_INFO(this->get_logger(),"start serial publish");
    }

	~libAoa_robot_publisher(){
		running_ = false;
		if (read_thread_.joinable()) read_thread_.join();
		close(serial_port_);
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
		my_input.use_pitch_dis = 2;

		my_output.r = 0; my_output.rad = 0; my_output.state = 0;
		my_output.x = 0; my_output.y = 0;

		int c5_len = sizeof(uwb_aoa_fob_pkg_t);
		int merge_len = sizeof(struct output);
		uint8_t *c5_buffer = (uint8_t *)malloc(c5_len + 2);
		uint8_t *merge_buffer = (uint8_t *)malloc(merge_len + 2);
		c5_buffer[0] = 0xc5;
		c5_buffer[1] = c5_len;
		merge_buffer[0] = 0xc6;
		merge_buffer[1] = merge_len;
		
		while (rclcpp::ok() && running_){
			int num_bytes = read(serial_port_, &read_buf, 1);
			if (num_bytes > 0)
			{
				printf("%02x", read_buf[0]);
				int ret = uart_receive_byte(read_buf[0]);
				if (ret == 1) {
					printf("\n");
					uwb_aoa_fob_pkg_t *c5 = NULL;
					uint8_t parse_ret = uart_protocol_packet_process((void **)&c5);
					if (parse_ret == 0xc5) {
						//printf("[UWB12]: dis: %.3f, agl: %.3f, pitch: %.3f\r\n", c5->distance, c5->angle, c5->pitch);

						time_t start_time = time(NULL);

						my_input.Distance = c5->distance; // update dis
						my_input.Azimuth = c5->angle; //update azi 
						my_input.Pitch = c5->pitch; // update pitch
						my_input.t = start_time; // update time(uint32)
						algo_uwb_aoa_merge(&my_input, &my_output);
						printf("output_result: %f,%f,%f,%f,%d\n\n", my_output.r, my_output.rad,my_output.x,my_output.y, my_output.state);


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

    // rclcpp::TimerBase::SharedPtr timer_;
    // size_t count_;
};



int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
	
	rclcpp::init(argc, argv);
  	rclcpp::spin(std::make_shared<libAoa_robot_publisher>(argv[1]));
 	rclcpp::shutdown();
	
	return 0;
}
