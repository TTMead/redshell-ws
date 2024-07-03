#include "path_planner.h"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("path_planner_node");

    PathPlanner path_planner(node);

    node->declare_parameter("occupancy_grid_topic", "potential_field_combined");
    node->declare_parameter("path_topic", "path");

    auto occupancy_grid_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        node->get_parameter("occupancy_grid_topic").as_string(), 10, 
        std::bind(&PathPlanner::occupancy_grid_callback, &path_planner, std::placeholders::_1)
    );

    auto path_pub = node->create_publisher<nav_msgs::msg::Path>(node->get_parameter("path_topic").as_string(), 10);
    auto distance_transform_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("distance_transform", 10);
    auto publish_timer = node->create_wall_timer(std::chrono::milliseconds(10), [&](){
        auto path_msg = path_planner.consume_path_msg();
        if (path_msg)
        {
            path_pub->publish(*path_msg);
        }
        
        auto distance_transform_msg = path_planner.consume_distance_transform_msg();
        if (distance_transform_msg)
        {
            distance_transform_pub->publish(*distance_transform_msg);
        }
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
