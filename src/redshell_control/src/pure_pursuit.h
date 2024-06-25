#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class PurePursuit : public rclcpp::Node
{
    public:
        PurePursuit();

    private:
        void path_callback(const nav_msgs::msg::Path::SharedPtr msg);

        /**
         * @brief Returns the latest robot_pose from the TF2 tree.
         * @return True if a valid pose was found. False otherwise.
         */
        bool get_robot_pose(geometry_msgs::msg::Pose &robot_pose);

        geometry_msgs::msg::Twist generate_control_command(geometry_msgs::msg::Pose &robot_pose, geometry_msgs::msg::Point &goal);

        void run();

        static double subtract_angles(double target_rad, double source_rad)
        {
            double result = target_rad - source_rad;
            result = std::fmod((result + M_PI_2), M_PI) - M_PI;
            return result;
        }

        static double heading_from_orientation(geometry_msgs::msg::Quaternion &orientation)
        {
            tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
            tf2::Matrix3x3 m(q);
            double roll_rad, pitch_rad, yaw_rad;
            m.getRPY(roll_rad, pitch_rad, yaw_rad);
            return yaw_rad;
        }

        static constexpr double distance_between_points(const geometry_msgs::msg::Point &A, const geometry_msgs::msg::Point &B)
        {
            return std::sqrt(std::pow(B.x - A.x, 2)
                           + std::pow(B.y - A.y, 2) 
                           + std::pow(B.z - A.z, 2));
        }

        static constexpr double bearing_between_points(const geometry_msgs::msg::Point &from, const geometry_msgs::msg::Point &to)
        {
            return std::atan2(from.y - to.y, from.x - to.x);
        }

        std::optional<nav_msgs::msg::Path> _path;

        std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> _tf_listener;

        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_sub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _look_ahead_pub;
        rclcpp::TimerBase::SharedPtr _run_timer;
};