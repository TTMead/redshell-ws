#include "path_planner.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Constructor: Initialize PathPlanner with a node instance
PathPlanner::PathPlanner(const std::shared_ptr<rclcpp::Node>& node) : _node(node)
{
    // Initialize TF2 buffer and listener using node's clock
    _tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
}

// Callback function for occupancy grid subscription
void PathPlanner::occupancy_grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    // Look up transform from "map" to "base_link" frame
    geometry_msgs::msg::TransformStamped map_to_robot;
    try
    {
        static constexpr char map_frame[] = "map";
        static constexpr char robot_frame[] = "base_link";
        map_to_robot = _tf_buffer->lookupTransform(map_frame, robot_frame, tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
        // Handle transform lookup failure
        RCLCPP_INFO_THROTTLE(_node->get_logger(), *(_node->get_clock()), 1000, "Waiting for state estimation: %s", ex.what());
        return;
    }

    // Extract robot's yaw (orientation) from transform
    tf2::Quaternion tf_quat;
    tf2::fromMsg(map_to_robot.transform.rotation, tf_quat);
    tf2::Matrix3x3 m(tf_quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Extract robot's pose (position and orientation) from transform
    geometry_msgs::msg::Pose robot_pose;
    robot_pose.position.x = map_to_robot.transform.translation.x;
    robot_pose.position.y = map_to_robot.transform.translation.y;
    robot_pose.position.z = map_to_robot.transform.translation.z;
    robot_pose.orientation = map_to_robot.transform.rotation;

    // Copy the received occupancy grid message
    nav_msgs::msg::OccupancyGrid grid = *msg;

    // Modify the grid with wavefront algorithm using robot's pose and yaw
    add_wave(grid, robot_pose, -yaw);

    // Generate a path based on the modified grid and robot's transform
    _path = generate_path(grid, map_to_robot);
}

// Function to modify the occupancy grid using wavefront algorithm
void PathPlanner::add_wave(nav_msgs::msg::OccupancyGrid& costmap, const geometry_msgs::msg::Pose& robot_pose, double bearing_rad)
{
    // Convert robot's position to grid coordinates
    const double costmap_resolution_m_per_cell = costmap.info.resolution;
    int32_t robot_col = std::round((robot_pose.position.x / costmap_resolution_m_per_cell) + (costmap.info.width/2.0));
    int32_t robot_row = std::round((robot_pose.position.y / costmap_resolution_m_per_cell) + (costmap.info.height/2.0));

    // Check if robot is within the valid range of the costmap
    if ((robot_row <= 0) || (robot_row >= static_cast<int32_t>(costmap.info.height))
     || (robot_col <= 0) || (robot_col >= static_cast<int32_t>(costmap.info.width)))
    {
        // Log error if robot is outside the valid range
        RCLCPP_ERROR_THROTTLE(_node->get_logger(), *(_node->get_clock()), 1000, "Robot is outside of aggregate costmap");
        return;
    }

    // Define wavefront distance
    static constexpr int8_t wave_dist = 40;

    // Iterate through cells around the robot within wavefront distance
    for (int8_t x = -wave_dist; x < wave_dist; x++)
    {
        for (int8_t y = -wave_dist; y < wave_dist; y++)
        {
            // Skip cells that are out of bounds
            if ((robot_col + x <= 0) || (robot_col + x >= static_cast<int32_t>(costmap.info.width)) || 
                (robot_row + y <= 0) || (robot_row + y >= static_cast<int32_t>(costmap.info.height)))
            {
                continue;
            }

            // Calculate wavefront value for the cell
            const int8_t value = -std::round((std::cos(bearing_rad) * x) + (-std::sin(bearing_rad) * y));
            const int64_t cell_index = (robot_col + x) + ((robot_row + y) * costmap.info.width);

            // Clamp the value if it exceeds INT8_MAX
            if (costmap.data[cell_index] > (INT8_MAX - (value + wave_dist)))
            {
                costmap.data[cell_index] = INT8_MAX;
                continue;
            }

            // Add wavefront value to the cell
            costmap.data[cell_index] += wave_dist + value;
        }
    }

    //

    // Store the modified costmap (distance transform)
    _distance_transform = costmap;
}

// Function to generate a path from modified occupancy grid
nav_msgs::msg::Path PathPlanner::generate_path(const nav_msgs::msg::OccupancyGrid& costmap, const geometry_msgs::msg::TransformStamped& map_to_robot)
{
    static constexpr char path_frame_id[] = "map";

    // Initialize path message
    nav_msgs::msg::Path path;
    path.header.frame_id = path_frame_id;

    // Set initial position for path generation
    geometry_msgs::msg::PoseStamped next_path_location;
    next_path_location.header.frame_id = path_frame_id;
    next_path_location.header.stamp = _node->get_clock()->now();
    next_path_location.pose.position.x = map_to_robot.transform.translation.x;
    next_path_location.pose.position.y = map_to_robot.transform.translation.y;
    next_path_location.pose.position.z = map_to_robot.transform.translation.z;
    next_path_location.pose.orientation = map_to_robot.transform.rotation;

    // Convert robot's position to grid coordinates
    const double costmap_resolution_m_per_cell = costmap.info.resolution;
    int32_t robot_row = std::round((next_path_location.pose.position.y / costmap_resolution_m_per_cell) + (costmap.info.height/2.0));
    int32_t robot_col = std::round((next_path_location.pose.position.x / costmap_resolution_m_per_cell) + (costmap.info.width/2.0));

    // Check if robot is within the valid range of the costmap
    if ((robot_row <= 0) || (robot_row >= static_cast<int32_t>(costmap.info.height))
     || (robot_col <= 0) || (robot_col >= static_cast<int32_t>(costmap.info.width)))
    {
        // Log error if robot is outside the valid range
        RCLCPP_ERROR_THROTTLE(_node->get_logger(), *(_node->get_clock()), 1000, "Robot is outside of aggregate costmap");
        return path;
    }

    // Add initial robot position to the path
    path.poses.push_back(next_path_location);

    int32_t row = robot_row;
    int32_t col = robot_col;
    int8_t min_value = INT8_MAX;
    double current_distance_from_robot = 0.0;

    bool path_complete = false;
    while (!path_complete)
    {
        // Initialize next cell indices
        int32_t col_next = col;
        int32_t row_next = row;

        // Iterate through neighboring cells
        for (int8_t x = -1; x < 2; x++)
        {
            for (int8_t y = -1; y < 2; y++)
            {
                // Calculate index of the neighboring cell
                const int64_t kernel_index = (col + x) + ((row + y) * costmap.info.width);
                const int8_t cell_value = costmap.data[kernel_index];

                // Update minimum value and next path location if a better cell is found
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

        // Update current cell indices
        col = col_next;
        row = row_next;

        // Update next path location with new cell coordinates
        next_path_location.pose.position.y = (row - (costmap.info.height/2.0)) * costmap_resolution_m_per_cell;
        next_path_location.pose.position.x = (col - (costmap.info.width/2.0)) * costmap_resolution_m_per_cell;
        next_path_location.header.stamp = _node->get_clock()->now();

        // Add updated path location to the path
        path.poses.push_back(next_path_location);
    }

    // Set timestamp for the path message and return it
    path.header.stamp = _node->get_clock()->now();
    return path;
}
