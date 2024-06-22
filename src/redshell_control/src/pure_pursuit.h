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

        std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> _tf_listener;

        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_sub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_pub;
};