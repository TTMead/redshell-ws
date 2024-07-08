#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

struct PurePursuitParams
{
    const std::string path_topic;
    const double look_ahead_dist_m;
    const double yaw_gain;
    const double max_yaw_rate;
    const double forward_velocity;
};

class PurePursuit
{
public:
    PurePursuit(const PurePursuitParams& params, const std::shared_ptr<rclcpp::Node>& node);

    PurePursuit (const PurePursuit&) = delete;
    PurePursuit& operator= (const PurePursuit&) = delete;

    /**
     * @brief Updates the path being followed.
     */
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);

    /**
     * @brief Runs the pure_pursuit control loop.
     */
    void run(geometry_msgs::msg::Twist& cmd_msg, geometry_msgs::msg::PoseStamped& look_ahead_msg);
private:

    /**
     * @brief Returns the latest robot_pose from the TF2 tree.
     * @return True if a valid pose was found. False otherwise.
     */
    bool get_robot_pose(geometry_msgs::msg::Pose &robot_pose) const;

    /**
     * @brief Implements a proportional control law on heading to move the robot towards the goal given the robot pose.
     */
    geometry_msgs::msg::Twist generate_control_command(const geometry_msgs::msg::Pose& robot_pose, const geometry_msgs::msg::Point& goal) const;

    static inline double heading_from_orientation(const geometry_msgs::msg::Quaternion& orientation)
    {
        tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
        tf2::Matrix3x3 m(q);
        double roll_rad, pitch_rad, yaw_rad;
        m.getRPY(roll_rad, pitch_rad, yaw_rad);
        return yaw_rad;
    }

    static constexpr double subtract_angles(const double target_rad, const double source_rad)
    {
        double result_rad = target_rad - source_rad;
        result_rad = std::fmod((result_rad + M_PI_2), M_PI) - M_PI_2;
        return result_rad;
    }

    static constexpr double distance_between_points(const geometry_msgs::msg::Point& A, const geometry_msgs::msg::Point& B)
    {
        return std::sqrt(std::pow(B.x - A.x, 2)
                        + std::pow(B.y - A.y, 2) 
                        + std::pow(B.z - A.z, 2));
    }

    static constexpr double bearing_between_points(const geometry_msgs::msg::Point& from, const geometry_msgs::msg::Point& to)
    {
        return std::atan2(from.y - to.y, from.x - to.x);
    }

    const PurePursuitParams _params;

    rclcpp::Node::SharedPtr _node;
    std::optional<nav_msgs::msg::Path> _path;
    std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
};