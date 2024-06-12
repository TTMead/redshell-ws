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
    nav_msgs::msg::OccupancyGrid grid = *msg;
    add_wave(grid, M_PI / 2.0);
}

void
PathPlanner::add_wave(nav_msgs::msg::OccupancyGrid& costmap, double bearing_rad)
{
    static constexpr double max_wave_value = 30;
    static constexpr double min_wave_value = -30;

    // For each cell
    for (int32_t col = 1; col < static_cast<int32_t>(costmap.info.width - 1); col++)
    {
        for (int32_t row = 1; row < static_cast<int32_t>(costmap.info.height - 1); row++)
        {
            const uint64_t cell_index = col + (row * costmap.info.width);

            // Add directional wave function
            const double wave_function_value = std::clamp(min_wave_value + (std::sin(bearing_rad) * col * (1.0 / costmap.info.width))
                                                                         + (std::cos(bearing_rad) * row * (1.0 / costmap.info.height)),
                                                          min_wave_value, max_wave_value);
            costmap.data[cell_index] += static_cast<int8_t>(std::round(wave_function_value));
        }
    }
}