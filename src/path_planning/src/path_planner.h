#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class PathPlanner
{
public:
    PathPlanner(const std::shared_ptr<rclcpp::Node>& node);

    PathPlanner (const PathPlanner&) = delete;
    PathPlanner& operator= (const PathPlanner&) = delete;


    void occupancy_grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    const std::optional<nav_msgs::msg::Path> consume_path_msg() { return consume<nav_msgs::msg::Path>(_path); }
    const std::optional<nav_msgs::msg::OccupancyGrid> consume_distance_transform_msg() { return consume<nav_msgs::msg::OccupancyGrid>(_distance_transform); }
private:
    /**
     * @brief Clears the contents of the optional-type data and returns its value
     */
    template<typename T> const std::optional<T> consume(std::optional<T>& data)
    {
        std::optional<T> out;
        data.swap(out);
        return out;
    }

    /**
     * @brief Adds a linear wave to the costmap object in a given direction provided by bearing_rad, centred about the robot_pose
     */
    void add_wave(nav_msgs::msg::OccupancyGrid& costmap, const geometry_msgs::msg::Pose& robot_pose, double bearing_rad);

    /**
     * @brief Generates a nav Path message starting at the robot pose and traversing downwards a given costmap
     */
    nav_msgs::msg::Path generate_path(const nav_msgs::msg::OccupancyGrid& costmap, const geometry_msgs::msg::TransformStamped& map_to_robot);

    /**
     * @brief Returns the distance in units of cells between two cell locations
     */
    static constexpr double distance(const int32_t from_row, const int32_t from_col, const int32_t to_row, const int32_t to_col)
    {
        return std::sqrt(std::pow(to_row - from_row, 2) + std::pow(to_col - from_col, 2));
    }

    std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;

    std::shared_ptr<rclcpp::Node> _node;

    std::optional<nav_msgs::msg::Path> _path;
    std::optional<nav_msgs::msg::OccupancyGrid> _distance_transform;
};