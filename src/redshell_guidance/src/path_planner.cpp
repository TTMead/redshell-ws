#include "path_planner.h"


PathPlanner::PathPlanner() : Node("path_planner_node")
{
    this->declare_parameter("occupancy_grid_topic", "potential_field_combined");
    this->declare_parameter("path_topic", "path");

    _tf_buffer.reset(new tf2_ros::Buffer(this->get_clock()));
    _tf_listener.reset(new tf2_ros::TransformListener(*_tf_buffer));

    _occupancy_grid_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        this->get_parameter("potential_field_combined").as_string(), 10, 
        std::bind(&PathPlanner::occupancy_grid_callback, this, std::placeholders::_1)
    );


    _path_pub = this->create_publisher<nav_msgs::msg::Path>(this->get_parameter("path_topic").as_string(), 10);
}

void
PathPlanner::occupancy_grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    
}