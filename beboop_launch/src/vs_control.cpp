#include "vs_control.hpp"

VsControl::VsControl() : Node("visual_servo_control_node")
{
	_cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    _error_sub = this->create_subscription<std_msgs::msg::Float32>(
		"/track_error", 10, 
		std::bind(&VsControl::error_callback, this, std::placeholders::_1)
	);

    this->declare_parameter("kp", 1.0);
    this->declare_parameter("speed", 1.0);
}

void
VsControl::error_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    float throttle{static_cast<float>(this->get_parameter("speed").as_double())};
    float yaw_rate{0};

    RCLCPP_INFO(this->get_logger(), "error: '%f'", msg->data);
    
    if (std::isnan(msg->data))
    {
        // Stop movement when NaN is received
        throttle = 0;
    }
    else
    {
        // Apply proportional controller
        yaw_rate = -this->get_parameter("kp").as_double()*msg->data;
    }

    geometry_msgs::msg::Twist cmd_msg;
    cmd_msg.linear.x = throttle;
    cmd_msg.angular.z = yaw_rate;
    _cmd_vel_publisher->publish(cmd_msg);
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VsControl>());
	rclcpp::shutdown();
	return 0;
}