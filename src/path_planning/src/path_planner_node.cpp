#include "path_planner.h"

int main(int argc, char ** argv)
{
    // Initialize ROS 2 node
    rclcpp::init(argc, argv);

    // Create a node instance with a unique name
    auto node = std::make_shared<rclcpp::Node>("path_planner_node");

    // Instantiate PathPlanner class, passing the node instance
    PathPlanner path_planner(node);

    // Declare parameters for the node with default values
    node->declare_parameter("occupancy_grid_topic", "potential_field_combined");
    node->declare_parameter("path_topic", "path");

    // Subscribe to the occupancy grid topic defined in parameters
    auto occupancy_grid_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        node->get_parameter("occupancy_grid_topic").as_string(), 10, 
        std::bind(&PathPlanner::occupancy_grid_callback, &path_planner, std::placeholders::_1)
    );

    // Create publishers for path and distance transform messages
    auto path_pub = node->create_publisher<nav_msgs::msg::Path>(node->get_parameter("path_topic").as_string(), 10);
    auto distance_transform_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("distance_transform", 10);

    // Create a timer to periodically publish path and distance transform messages
    auto publish_timer = node->create_wall_timer(std::chrono::milliseconds(10), [&](){
        // Publish path message if available
        auto path_msg = path_planner.consume_path_msg();
        if (path_msg)
        {
            path_pub->publish(*path_msg);
        }
        
        // Publish distance transform message if available
        auto distance_transform_msg = path_planner.consume_distance_transform_msg();
        if (distance_transform_msg)
        {
            distance_transform_pub->publish(*distance_transform_msg);
        }
    });

    // Spin the node to process callbacks and timers
    rclcpp::spin(node);

    // Shutdown ROS 2 node
    rclcpp::shutdown();

    return 0;
}
