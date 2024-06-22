#include "path_planner.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
        return;
    }

    // Extract heading from TF
    tf2::Quaternion tf_quat;
    tf2::fromMsg(map_to_robot.transform.rotation, tf_quat);
    tf2::Matrix3x3 m(tf_quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Extract pose from TF
    geometry_msgs::msg::Pose robot_pose;
    robot_pose.position.x = map_to_robot.transform.translation.x;
    robot_pose.position.y = map_to_robot.transform.translation.y;
    robot_pose.position.z = map_to_robot.transform.translation.z;
    robot_pose.orientation = map_to_robot.transform.rotation;

    nav_msgs::msg::OccupancyGrid grid = *msg;
    add_wave(grid, robot_pose, -yaw);

    nav_msgs::msg::Path path_msg = generate_path(grid, map_to_robot);
    _path_pub->publish(path_msg);
}

void
PathPlanner::add_wave(nav_msgs::msg::OccupancyGrid& costmap, geometry_msgs::msg::Pose& robot_pose, double bearing_rad)
{
    // Convert rover position to row/col
    const double costmap_resolution_m_per_cell = costmap.info.resolution;
    int32_t robot_col = std::round((robot_pose.position.x / costmap_resolution_m_per_cell) + (costmap.info.width/2.0));
    int32_t robot_row = std::round((robot_pose.position.y / costmap_resolution_m_per_cell) + (costmap.info.height/2.0));

    if ((robot_row <= 0) || (robot_row >= static_cast<int32_t>(costmap.info.height))
     || (robot_col <= 0) || (robot_col >= static_cast<int32_t>(costmap.info.width)))
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Robot is outside of aggregate costmap");
        return;
    }

    // For each cell in wave
    static constexpr int8_t wave_dist = 40;
    for (int8_t x = -wave_dist; x < wave_dist; x++)
    {
        for (int8_t y = -wave_dist; y < wave_dist; y++)
        {
            // Ignore cells that are out of bounds
            if ((robot_col + x <= 0) || (robot_col + x >= static_cast<int32_t>(costmap.info.width)) || (robot_row + y <= 0) || (robot_row + y >= static_cast<int32_t>(costmap.info.height)))
            {
                continue;
            }

            const int8_t value = -std::round((std::cos(bearing_rad) * x) + (-std::sin(bearing_rad) * y));
            const int64_t cell_index = (robot_col + x) + ((robot_row + y) * costmap.info.width);

            // If the new value will be too big, clamp it to the maximum int
            if (costmap.data[cell_index] > (INT8_MAX - (value + wave_dist)))
            {
                costmap.data[cell_index] = INT8_MAX;
                continue;
            }

            costmap.data[cell_index] += wave_dist + value;
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
PathPlanner::generate_path(nav_msgs::msg::OccupancyGrid& costmap, geometry_msgs::msg::TransformStamped& map_to_robot)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";

    geometry_msgs::msg::PoseStamped next_path_location;
    next_path_location.pose.position.x = map_to_robot.transform.translation.x;
    next_path_location.pose.position.y = map_to_robot.transform.translation.y;
    next_path_location.pose.position.z = map_to_robot.transform.translation.z;
    next_path_location.pose.orientation = map_to_robot.transform.rotation;

    const double costmap_resolution_m_per_cell = costmap.info.resolution;
    int32_t robot_row = std::round((next_path_location.pose.position.y / costmap_resolution_m_per_cell) + (costmap.info.height/2.0));
    int32_t robot_col = std::round((next_path_location.pose.position.x / costmap_resolution_m_per_cell) + (costmap.info.width/2.0));

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
                if (cell_value < min_value)
                {
                    double new_distance_from_robot = distance(robot_row, robot_col, row + y, col + x);
                    if (new_distance_from_robot >= current_distance_from_robot)
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
        next_path_location.pose.position.y = (row  - (costmap.info.height/2.0)) * costmap_resolution_m_per_cell;
        next_path_location.pose.position.x = (col - (costmap.info.width/2.0)) * costmap_resolution_m_per_cell;
        path.poses.push_back(next_path_location);
    }

    return path;
}