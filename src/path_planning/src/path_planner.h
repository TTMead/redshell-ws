#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

// Class definition for PathPlanner
class PathPlanner
{
public:
    // Constructor taking a shared_ptr to rclcpp::Node
    PathPlanner(const std::shared_ptr<rclcpp::Node>& node);

    // Deleted copy constructor and assignment operator to prevent copying
    PathPlanner(const PathPlanner&) = delete;
    PathPlanner& operator=(const PathPlanner&) = delete;

    // Callback function for occupancy grid subscription
    void occupancy_grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    // Function to consume path messages and clear the stored path
    const std::optional<nav_msgs::msg::Path> consume_path_msg() { return consume<nav_msgs::msg::Path>(_path); }

    // Function to consume distance transform messages and clear the stored distance transform
    const std::optional<nav_msgs::msg::OccupancyGrid> consume_distance_transform_msg() { return consume<nav_msgs::msg::OccupancyGrid>(_distance_transform); }

private:
    /**
     * @brief Clears the contents of the optional-type data and returns its value
     * @tparam T Type of data to consume (nav_msgs::msg::Path or nav_msgs::msg::OccupancyGrid)
     */
    template<typename T>
    const std::optional<T> consume(std::optional<T>& data)
    {
        std::optional<T> out;
        data.swap(out); // Swap data with out to clear data and return the cleared value
        return out;
    }

    /**
     * @brief Adds a wavefront to the costmap object in a given direction provided by bearing_rad, centered around the robot_pose
     * @param costmap The occupancy grid to modify
     * @param robot_pose The pose of the robot
     * @param bearing_rad The direction in radians where the wavefront is added
     */
    void add_wave(nav_msgs::msg::OccupancyGrid& costmap, const geometry_msgs::msg::Pose& robot_pose, double bearing_rad);

    /**
     * @brief Generates a path message starting at the robot pose and traversing a given costmap
     * @param costmap The occupancy grid representing the environment
     * @param map_to_robot The transform from map frame to robot frame
     * @return A nav_msgs::msg::Path message representing the planned path
     */
    nav_msgs::msg::Path generate_path(const nav_msgs::msg::OccupancyGrid& costmap, const geometry_msgs::msg::TransformStamped& map_to_robot);

    /**
     * @brief Calculates the Euclidean distance between two grid cell locations
     * @param from_row Row index of the starting cell
     * @param from_col Column index of the starting cell
     * @param to_row Row index of the target cell
     * @param to_col Column index of the target cell
     * @return Euclidean distance in units of cells
     */
    static constexpr double distance(const int32_t from_row, const int32_t from_col, const int32_t to_row, const int32_t to_col)
    {
        return std::sqrt(std::pow(to_row - from_row, 2) + std::pow(to_col - from_col, 2));
    }

    // Member variables
    std::shared_ptr<tf2_ros::Buffer> _tf_buffer; // Buffer for TF2 transforms
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener; // Listener for TF2 transforms

    std::shared_ptr<rclcpp::Node> _node; // Pointer to the ROS 2 node

    std::optional<nav_msgs::msg::Path> _path; // Optional container for storing path messages
    std::optional<nav_msgs::msg::OccupancyGrid> _distance_transform; // Optional container for storing distance transform messages
};
