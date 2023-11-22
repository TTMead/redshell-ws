#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"

class VsControl : public rclcpp::Node
{
	public:
		VsControl();

	private:
	void error_callback(const std_msgs::msg::Float32::SharedPtr msg);

		rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _error_sub;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_publisher;
};