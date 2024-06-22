#include "pure_pursuit.h"

PurePursuit::PurePursuit() : Node("pure_pursuit_node")
{
    this->declare_parameter("path_topic", "path");

    _tf_buffer.reset(new tf2_ros::Buffer(this->get_clock()));
    _tf_listener.reset(new tf2_ros::TransformListener(*_tf_buffer));

    _path_sub = this->create_subscription<nav_msgs::msg::Path>(
        this->get_parameter("path_topic").as_string(), 10, 
        std::bind(&PurePursuit::path_callback, this, std::placeholders::_1)
    );


    _cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

void
PurePursuit::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    
}

bool
PurePursuit::get_robot_pose(geometry_msgs::msg::Pose &robot_pose)
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
        return false;
    }

    // Extract pose from TF
    robot_pose.position.x = map_to_robot.transform.translation.x;
    robot_pose.position.y = map_to_robot.transform.translation.y;
    robot_pose.position.z = map_to_robot.transform.translation.z;
    robot_pose.orientation = map_to_robot.transform.rotation;

    return true;
}
