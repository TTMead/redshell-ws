#include "pure_pursuit.h"

PurePursuitParams get_pure_pursuit_params(const std::shared_ptr<rclcpp::Node>& node)
{
    node->declare_parameter("path_topic", "path");
    node->declare_parameter("look_ahead_dist_m", 0.1);
    node->declare_parameter("yaw_gain", 0.1);
    node->declare_parameter("max_yaw_rate", 1.0);
    node->declare_parameter("forward_velocity", 0.3);
    return {
        node->get_parameter("path_topic").as_string(),
        node->get_parameter("look_ahead_dist_m").as_double(),
        node->get_parameter("yaw_gain").as_double(),
        node->get_parameter("max_yaw_rate").as_double(),
        node->get_parameter("forward_velocity").as_double()
    };
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("pure_pursuit");
    const auto params = get_pure_pursuit_params(node);
    PurePursuit pure_pursuit_controller(params, node);

    auto path_sub = node->create_subscription<nav_msgs::msg::Path>("path", 10, 
        std::bind(&PurePursuit::path_callback, &pure_pursuit_controller, std::placeholders::_1));

    auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    auto look_ahead_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/look_ahead", 10);
    auto control_loop_timer = node->create_wall_timer(std::chrono::milliseconds(100), [&](){
        geometry_msgs::msg::Twist cmd_msg{};
        geometry_msgs::msg::PoseStamped look_ahead_msg{};
        pure_pursuit_controller.run(cmd_msg, look_ahead_msg);
        cmd_vel_pub->publish(cmd_msg);
        look_ahead_pub->publish(look_ahead_msg);
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
