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

    nav_msgs::msg::Path path_msg = generate_path(grid);
    _path_pub->publish(path_msg);
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

nav_msgs::msg::Path
PathPlanner::generate_path(nav_msgs::msg::OccupancyGrid& costmap)
{
    nav_msgs::msg::Path path;

    geometry_msgs::msg::TransformStamped map_to_robot;
    try
    {
        static constexpr char map_frame[] = "map";
        static constexpr char robot_frame[] = "base_link";
        map_to_robot = _tf_buffer->lookupTransform(map_frame, robot_frame, tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        return path;
    }

    // Initialise path with current robot position
    geometry_msgs::msg::PoseStamped next_path_location;
    next_path_location.pose.position.x = map_to_robot.transform.translation.x;
    next_path_location.pose.position.y = map_to_robot.transform.translation.y;
    next_path_location.pose.position.z = map_to_robot.transform.translation.z;
    next_path_location.pose.orientation = map_to_robot.transform.rotation;
    path.poses.push_back(next_path_location);

    const double costmap_resolution_m_per_cell = costmap.info.resolution;
    int32_t row = std::round(next_path_location.pose.position.x / costmap_resolution_m_per_cell);
    int32_t col = std::round(next_path_location.pose.position.y / costmap_resolution_m_per_cell);
    int8_t min_value = INT8_MAX;

    bool path_complete = false;
    while(!path_complete)
    {
        // Look at immediate neighbours
        int32_t col_next = col;
        int32_t row_next = row;

        for (int8_t x = -1; x < 2; x++)
        {
            for (int8_t y = -1; y < 2; y++)
            {
                const uint64_t kernel_index = (col + x) + ((row + y) * costmap.info.width);
                const int8_t cell_value = costmap.data[kernel_index];

                // Save cell if new minimum is found
                if (cell_value < min_value)
                {
                    min_value = cell_value;
                    col_next = col + x;
                    row_next = row + y;
                }
            }
        }

        // If no updates are made, end the path discovery
        if (col == col_next && row == row_next)
        {
            path_complete = true;
            continue;
        }

        // Update the current cell pointer
        col = col_next;
        row = row_next;

        // Add the new point to path
        next_path_location.pose.position.x = row * costmap_resolution_m_per_cell;
        next_path_location.pose.position.y = col * costmap_resolution_m_per_cell;
        path.poses.push_back(next_path_location);
    }

    return path;
}