#include "pure_pursuit.h"

PurePursuit::PurePursuit() : Node("pure_pursuit_node")
{
    this->declare_parameter("path_topic", "path");
    this->declare_parameter("look_ahead_dist_m", 0.3);
    this->declare_parameter("yaw_gain", 0.1);
    this->declare_parameter("max_yaw_rate", 1.0);
    this->declare_parameter("forward_velocity", 0.1);

    _tf_buffer.reset(new tf2_ros::Buffer(this->get_clock()));
    _tf_listener.reset(new tf2_ros::TransformListener(*_tf_buffer));

    _path_sub = this->create_subscription<nav_msgs::msg::Path>(
        this->get_parameter("path_topic").as_string(), 10, 
        std::bind(&PurePursuit::path_callback, this, std::placeholders::_1)
    );

    _cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    _look_ahead_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/look_ahead", 10);

    using namespace std::chrono_literals;
    _run_timer = this->create_wall_timer(100ms, std::bind(&PurePursuit::run, this));
}

void
PurePursuit::run()
{
    geometry_msgs::msg::Twist cmd_msg{};

    geometry_msgs::msg::Pose robot_pose;

    // If both a path and pose are available
    if (_path.has_value() && get_robot_pose(robot_pose))
    {
        for (geometry_msgs::msg::PoseStamped &path_pose_stamped : _path->poses)
        {
            geometry_msgs::msg::Pose path_pose = path_pose_stamped.pose;
            if (distance_between_points(path_pose.position, robot_pose.position) > this->get_parameter("look_ahead_dist_m").as_double())
            {
                // Publish look-ahead for debugging
                _look_ahead_pub->publish(path_pose_stamped);

                cmd_msg = generate_control_command(robot_pose, path_pose.position);
                break;
            }
        }
    }

    _cmd_vel_pub->publish(cmd_msg);
}

geometry_msgs::msg::Twist
PurePursuit::generate_control_command(geometry_msgs::msg::Pose &robot_pose, geometry_msgs::msg::Point &goal)
{
    const double desired_bearing_rad = bearing_between_points(robot_pose.position, goal);
    const double current_bearing_rad = heading_from_orientation(robot_pose.orientation);
    const double yaw_error_rad = subtract_angles(desired_bearing_rad, current_bearing_rad);

    double yaw_effort_rads = this->get_parameter("yaw_gain").as_double() * (yaw_error_rad);
    yaw_effort_rads = std::clamp(yaw_effort_rads, -this->get_parameter("max_yaw_rate").as_double(), this->get_parameter("max_yaw_rate").as_double());

    geometry_msgs::msg::Twist control_command{};
    control_command.linear.x = this->get_parameter("forward_velocity").as_double();
    control_command.angular.z = yaw_effort_rads;
    return control_command;
}

void
PurePursuit::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    _path = *msg;
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
