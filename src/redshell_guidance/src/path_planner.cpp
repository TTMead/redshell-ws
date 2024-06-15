#include "path_planner.h"

PathPlanner::PathPlanner() : Node("path_planner_node")
{
    this->declare_parameter("occupancy_grid_topic", "potential_field_combined");
    this->declare_parameter("path_topic", "path");

    _tf_buffer.reset(new tf2_ros::Buffer(this->get_clock()));
    _tf_listener.reset(new tf2_ros::TransformListener(*_tf_buffer));

    _occupancy_grid_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        this->get_parameter("occupancy_grid_topic").as_string(), 10, 
        std::bind(&PathPlanner::occupancy_grid_callback, this, std::placeholders::_1)
    );


    _path_pub = this->create_publisher<nav_msgs::msg::Path>(this->get_parameter("path_topic").as_string(), 10);
    _distance_transform_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("distance_transform", 10);
}

void
PathPlanner::occupancy_grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    nav_msgs::msg::OccupancyGrid grid = *msg;
    add_wave(grid, 1.8);

    nav_msgs::msg::Path path_msg = generate_path(grid);
    _path_pub->publish(path_msg);
}

void
PathPlanner::add_wave(nav_msgs::msg::OccupancyGrid& costmap, double bearing_rad)
{
    static constexpr double min_wave_value = 0;
    static constexpr double max_wave_value = 50;

    // For each cell
    for (int32_t col = 1; col < static_cast<int32_t>(costmap.info.width - 1); col++)
    {
        for (int32_t row = 1; row < static_cast<int32_t>(costmap.info.height - 1); row++)
        {
            const int64_t cell_index = col + (row * costmap.info.width);

            // Add directional wave function
            const double wave_function_value = std::clamp(min_wave_value + (std::sin(bearing_rad) * (static_cast<double>(col) / static_cast<double>(costmap.info.width)) * (max_wave_value - min_wave_value))
                                                                         + (std::cos(bearing_rad) * (static_cast<double>(row) / static_cast<double>(costmap.info.height)) * (max_wave_value - min_wave_value)),
                                                          min_wave_value, max_wave_value);
            
            if (costmap.data[cell_index] + wave_function_value > INT8_MAX)
            {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Path planner attempted to overflow costmap cell");
            }
            
            costmap.data[cell_index] += static_cast<int8_t>(std::round(wave_function_value));

            // Clamp cell value to [0, 100]
            static constexpr int8_t min_cell_value = 0;
            static constexpr int8_t max_cell_value = 100;
            costmap.data[cell_index] = std::clamp(costmap.data[cell_index], min_cell_value, max_cell_value);
        }
    }

    _distance_transform_pub->publish(costmap);
}

double
PathPlanner::distance(int32_t from_row, int32_t from_col, int32_t to_row, int32_t to_col)
{
    return std::sqrt(std::pow(to_row - from_row, 2) + std::pow(to_col - from_col, 2));
}

nav_msgs::msg::Path
PathPlanner::generate_path(nav_msgs::msg::OccupancyGrid& costmap)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";

    geometry_msgs::msg::TransformStamped map_to_robot;
    try
    {
        static constexpr char map_frame[] = "map";
        static constexpr char robot_frame[] = "base_link";
        map_to_robot = _tf_buffer->lookupTransform(map_frame, robot_frame, tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_DEBUG(this->get_logger(), "Wating for state estimation: %s", ex.what());
        return path;
    }

    geometry_msgs::msg::PoseStamped next_path_location;
    next_path_location.pose.position.x = map_to_robot.transform.translation.x;
    next_path_location.pose.position.y = map_to_robot.transform.translation.y;
    next_path_location.pose.position.z = map_to_robot.transform.translation.z;
    next_path_location.pose.orientation = map_to_robot.transform.rotation;

    const double costmap_resolution_m_per_cell = costmap.info.resolution;
    int32_t robot_row = std::round((next_path_location.pose.position.x / costmap_resolution_m_per_cell) + (costmap.info.height/2.0));
    int32_t robot_col = std::round((next_path_location.pose.position.y / costmap_resolution_m_per_cell) + (costmap.info.width/2.0));

    if ((robot_row <= 0) || (robot_row >= static_cast<int32_t>(costmap.info.height))
     || (robot_col <= 0) || (robot_col >= static_cast<int32_t>(costmap.info.width)))
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Robot is outside of aggregate costmap");
        return path;
    }

    // Initialise path with current robot position
    path.poses.push_back(next_path_location);
    int32_t row = robot_row;
    int32_t col = robot_col;
    int8_t min_value = INT8_MAX;
    double current_distance_from_robot = 0.0;

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
                const int64_t kernel_index = (col + x) + ((row + y) * costmap.info.width);
                const int8_t cell_value = costmap.data[kernel_index];

                // Save cell if new cell is found that is further away
                if (cell_value <= min_value)
                {
                    double new_distance_from_robot = distance(robot_row, robot_col, row + y, col + x);
                    if (new_distance_from_robot > current_distance_from_robot)
                    {
                        min_value = cell_value;
                        col_next = col + x;
                        row_next = row + y;
                    }
                }
            }
        }

        // If no updates are made, end the path discovery
        if (col == col_next && row == row_next)
        {
            path_complete = true;
            continue;
        }

        // If reached edge of costmap, end the path discovery
        if ((col_next <= 0) || (col_next >= static_cast<int32_t>(costmap.info.width-1))
         || (row_next <= 0) || (row_next >= static_cast<int32_t>(costmap.info.height-1)))
        {
            path_complete = true;
            continue;
        }

        // Update the current cell pointer
        col = col_next;
        row = row_next;

        // Add the new point to path
        next_path_location.pose.position.x = (row  - (costmap.info.height/2.0)) * costmap_resolution_m_per_cell;
        next_path_location.pose.position.y = (col - (costmap.info.width/2.0)) * costmap_resolution_m_per_cell;
        path.poses.push_back(next_path_location);
    }

    return path;
}