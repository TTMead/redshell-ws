#include "redshell_interface.h"

// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> 		// Contains file controls like O_RDWR
#include <errno.h> 		// Error integer and strerror() function
#include <termios.h> 	// Contains POSIX terminal control definitions
#include <unistd.h> 	// write(), read(), close()

// Messaging headers
#include "redshell/command.h"
#include "redshell/imu.h"
#include "redshell/encoder.h"

static constexpr char serial_port_location[] = "/dev/ttyUSB0";
static constexpr double cmdvel_timeout_s = 0.5; // The amount of time to wait for a cmd_vel msg before stopping motors
static constexpr double cmdvel_min_period_s = 0.1; // The amount of time to ignore cmd_vel msg after one has been received
static constexpr double motor_deadzone = 9.0; // The threshold of percentage PWM that causes the motor to start moving

RedshellInterface::RedshellInterface() : Node("redshell_interface")
{
	// Subscribe to /cmd_vel topic
	_cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
		"/cmd_vel", 10, 
		std::bind(&RedshellInterface::cmd_vel_callback, this, std::placeholders::_1)
	);

	// Initialise publishers
	_imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
	_encoder_pub = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("wheel_encoders", 10);

	this->declare_parameter("port", "/dev/ttyUSB0");
	std::string serial_port_location = this->get_parameter("port").as_string();

	// Open serial port connection to the motor interface board
	this->_serial_port = open(serial_port_location.c_str(), O_RDWR);
	if (this->_serial_port < 0) {
		RCLCPP_ERROR(this->get_logger(), "Error %i while attempting to open serial connection '%s': \"%s\"\n", errno, serial_port_location.c_str(), strerror(errno));
	}
	configure_serial_port();

	// Initialise command timeout timer
	_last_command_timestamp = rclcpp::Node::now();
	_command_timout_timer = this->create_wall_timer(
    	std::chrono::milliseconds(500), 
		std::bind(&RedshellInterface::timout_timer_callback, this)
	);

	// Initialise run loop timer for reading serial
	_run_thread = std::thread(std::bind(&RedshellInterface::run, this));
}

RedshellInterface::~RedshellInterface() {
	close(this->_serial_port);

	if (_run_thread.joinable())
	{
		_run_thread.join();
	}
}

void
RedshellInterface::run()
{
	while (rclcpp::ok())
	{
		char incoming_byte[1];
		read(this->_serial_port, incoming_byte, 1);

		// RCLCPP_INFO(this->get_logger(), "%c", incoming_byte[0]);

		if (incoming_byte[0] == REDSHELL_START_BYTE)
		{
			_command_index = 0;
			_reading_msg = true;
		}

		if (_reading_msg)
		{
			_command_buffer[_command_index] = incoming_byte[0];
			_command_index++;

			if (_command_index >= REDSHELL_MESSAGE_SIZE)
			{
				PacketInfo incoming_packet;
				deserialize(&incoming_packet, (uint8_t*)(_command_buffer));
				handle_message(incoming_packet);
				_reading_msg = false;
			}
		}
	}
}

void
RedshellInterface::handle_message(const PacketInfo& msg)
{
	switch (msg.id)
	{
		case (REDSHELL_MSG_ID_IMU): {
			sensor_msgs::msg::Imu imu_msg;
			imu_msg.header.stamp = this->get_clock()->now();
			imu_msg.header.frame_id = "imu";

			int16_t x, y, z;
			msg_imu_decode(msg, (uint16_t*)&x, (uint16_t*)&y, (uint16_t*)&z);

			// ADXL345 Data Sheet Rev. G
			static constexpr double adxl345_scaling_factor_mg_per_lsb = 31.2;
			static constexpr double g_to_ms2 = 9.81;
			static constexpr double imu_scaling = adxl345_scaling_factor_mg_per_lsb * 1e-3 * g_to_ms2;

			imu_msg.orientation_covariance[0] = -1;
			imu_msg.angular_velocity_covariance[0] = -1;
			imu_msg.linear_acceleration.x = imu_scaling * x;
			imu_msg.linear_acceleration.y = imu_scaling * y;
			imu_msg.linear_acceleration.z = imu_scaling * z;

			static constexpr double accel_std_dev = 0.5;
			imu_msg.linear_acceleration_covariance[0] = std::sqrt(accel_std_dev);
			imu_msg.linear_acceleration_covariance[4] = std::sqrt(accel_std_dev);
			imu_msg.linear_acceleration_covariance[8] = std::sqrt(accel_std_dev);

			_imu_pub->publish(imu_msg);
			break;
		}

		case (REDSHELL_MSG_ID_ENCODER): {
			geometry_msgs::msg::TwistWithCovarianceStamped encoder_msg;
			encoder_msg.header.stamp = this->get_clock()->now();
			encoder_msg.header.frame_id = "base_link";

			int32_t speed_motor_left_rpm, speed_motor_right_rpm;
			msg_encoder_decode(msg, &speed_motor_left_rpm, &speed_motor_right_rpm);

			// RCLCPP_INFO(this->get_logger(), "ENC (%d, %d)", speed_motor_left_rpm, speed_motor_right_rpm);

			static constexpr double wheel_radius_m = 0.05;
			static constexpr double rpm_to_ms = (1.0 / 60.0) * 2.0 * M_PI * wheel_radius_m;

			const double left_vel_ms = speed_motor_left_rpm * rpm_to_ms;
			const double right_vel_ms = speed_motor_right_rpm * rpm_to_ms;

			const double forward_vel_ms = (left_vel_ms + right_vel_ms) / 2.0;

			static constexpr double wheel_base_m = 0.23;
			const double angular_vel_ms = (right_vel_ms - left_vel_ms) / wheel_base_m;

			encoder_msg.twist.twist.linear.x = forward_vel_ms;
			encoder_msg.twist.twist.linear.y = 0.0;
			encoder_msg.twist.twist.linear.z = 0.0;
			encoder_msg.twist.twist.angular.x = 0.0;
			encoder_msg.twist.twist.angular.y = 0.0;
			encoder_msg.twist.twist.angular.z = angular_vel_ms;

			static constexpr double forward_vel_std_dev = 0.3;
			encoder_msg.twist.covariance[0] = std::sqrt(forward_vel_std_dev);

			static constexpr double angular_vel_std_dev = 0.2;
			encoder_msg.twist.covariance[35] = std::sqrt(angular_vel_std_dev);

			_encoder_pub->publish(encoder_msg);
			break;
		}
		default: {
			RCLCPP_ERROR_THROTTLE(this->get_logger(), *(this->get_clock()), 1000, "Unsupported packet with msg %u", msg.id);
		}
	}
}

void
RedshellInterface::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
	if (msg->linear.y != 0 ||
		msg->linear.z != 0 ||
		msg->angular.x != 0 ||
		msg->angular.y != 0) 
	{
		RCLCPP_WARN(this->get_logger(), "Motor interface module has received a /cmd_vel message that cannot be translated into motor commands.");
	}

	// If we received /cmd_vel at a rate too high, ignore the message (to prevent flooding the UART bus)
	rclcpp::Time current_time = rclcpp::Node::now();
	if ((current_time - _last_command_timestamp) < rclcpp::Duration::from_seconds(cmdvel_min_period_s)) {
		return;
	}

	write_motor_command(msg->linear.x, msg->angular.z);

	// Reset the motor command time
	_last_command_timestamp = rclcpp::Node::now();
}


void
RedshellInterface::timout_timer_callback() {
	rclcpp::Time current_time = rclcpp::Node::now();

	// Stop the motors if we havent received a command for over 1 second
	if ((current_time - _last_command_timestamp) > rclcpp::Duration::from_seconds(cmdvel_timeout_s)) {
		write_motor_command(0, 0);
	}
}

int32_t
RedshellInterface::scale_motor_command(int32_t command) const
{
	// If below deadzone, don't run
	if (command < motor_deadzone) {
		return 0;
	}

	// Else return scaled value
	float command_f = static_cast<float>(command);
	command_f = ((command_f * (99.0 - motor_deadzone)) / 99.0) + motor_deadzone;
	return static_cast<int32_t>(command_f);
}

void
RedshellInterface::write_motor_command(float throttle, float yaw_rate){
	// Speed values are within the range of [-1 to 1]
	const float left_speed = (throttle - yaw_rate)/2;    
	const float right_speed = (throttle + yaw_rate)/2;

	// Motor speed commands are within the range of [-100 to 100]
	int32_t left_cmd = std::round(left_speed*100.0);
	int32_t right_cmd = std::round(right_speed*100.0);

	uint8_t motor_command_msg[REDSHELL_MESSAGE_SIZE];
	serialize(msg_command_encode(left_cmd, right_cmd), motor_command_msg);

	write(this->_serial_port, motor_command_msg, sizeof(motor_command_msg));
}

void
RedshellInterface::configure_serial_port()
{
	/* Critical serial specifications:
	 * Baud: 19200
	 * Data Size: 8 bits
	 * Parity: None
	 * Stop Bits: 1 */
	
	struct termios tty;

	// Read in existing settings, and handle any error
	if(tcgetattr(this->_serial_port, &tty) != 0) {
		RCLCPP_ERROR(this->get_logger(), "Error %i from tcgetattr: %s\n", errno, strerror(errno));
		return;
	}

	// Control mode flags
	tty.c_cflag &= ~PARENB; 		// Clear parity bit, disabling parity
	tty.c_cflag &= ~CSTOPB; 		// Clear stop field, only one stop bit used in communication
	tty.c_cflag &= ~CSIZE; 			// Clear all bits that set the data size 
	tty.c_cflag |= CS8; 			// 8 bits per data segment
	tty.c_cflag &= ~CRTSCTS; 		// Disable RTS/CTS hardware flow control
	tty.c_cflag |= CREAD | CLOCAL; 	// Turn on READ & ignore ctrl lines (CLOCAL = 1)

	// Local mode flags
	tty.c_lflag &= ~ICANON;		// Disable canonical mode
	tty.c_lflag &= ~ECHO; 		// Disable echo
	tty.c_lflag &= ~ECHOE; 		// Disable erasure
	tty.c_lflag &= ~ECHONL;		// Disable new-line echo
	tty.c_lflag &= ~ISIG; 		// Disable interpretation of INTR, QUIT and SUSP

	// Input mode flags
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); 							// Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); 	// Disable any special handling of received bytes

	// Output mode flags
	tty.c_oflag &= ~OPOST; 		// Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; 		// Prevent conversion of newline to carriage return/line feed

	// Timeout configuration
	// Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
	tty.c_cc[VTIME] = 10;
	tty.c_cc[VMIN] = 0;

	// Baudrate configuration
	cfsetispeed(&tty, B19200);
	cfsetospeed(&tty, B19200);

	// Save tty settings
	if (tcsetattr(this->_serial_port, TCSANOW, &tty) != 0) {
		RCLCPP_ERROR(this->get_logger(), "Error %i from tcsetattr: %s\n", errno, strerror(errno));
		return;
	}
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RedshellInterface>());
	rclcpp::shutdown();
	return 0;
}