#include "pure_pursuit.h"

PurePursuit::PurePursuit(const PurePursuitParams& params, const std::shared_ptr<rclcpp::Node>& node) : _params(params), _node(node)
{
    _tf_buffer.reset(new tf2_ros::Buffer(node->get_clock()));
    _tf_listener.reset(new tf2_ros::TransformListener(*_tf_buffer));
}

void
PurePursuit::run(geometry_msgs::msg::Twist& cmd_msg, geometry_msgs::msg::PoseStamped& look_ahead_msg)
{
    geometry_msgs::msg::Pose robot_pose;

    // If both a path and pose are available
    if (_path.has_value() && get_robot_pose(robot_pose))
    {
        for (const geometry_msgs::msg::PoseStamped& path_pose_stamped : _path->poses)
        {
            geometry_msgs::msg::Pose path_pose = path_pose_stamped.pose;
            if (distance_between_points(path_pose.position, robot_pose.position) > _params.look_ahead_dist_m)
            {
                look_ahead_msg = path_pose_stamped; // Look-ahead is published for debugging
                cmd_msg = generate_control_command(robot_pose, path_pose.position);
                break;
            }
        }
    }
}

geometry_msgs::msg::Twist
PurePursuit::generate_control_command(const geometry_msgs::msg::Pose &robot_pose, const geometry_msgs::msg::Point &goal) const
{
    const double desired_bearing_rad = bearing_between_points(robot_pose.position, goal);
    const double current_bearing_rad = heading_from_orientation(robot_pose.orientation);
    const double yaw_error_rad = subtract_angles(desired_bearing_rad, current_bearing_rad);

    double yaw_effort_rads = _params.yaw_gain * (yaw_error_rad);
    yaw_effort_rads = std::clamp(yaw_effort_rads, -_params.max_yaw_rate, _params.max_yaw_rate);

    geometry_msgs::msg::Twist control_command{};
    control_command.linear.x = _params.forward_velocity;
    control_command.angular.z = yaw_effort_rads;
    return control_command;
}

void
PurePursuit::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    _path = *msg;
}

bool
PurePursuit::get_robot_pose(geometry_msgs::msg::Pose &robot_pose) const
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
        RCLCPP_INFO_THROTTLE(_node->get_logger(), *_node->get_clock(), 1000, "Wating for state estimation: %s", ex.what());
        return false;
    }
    robot_pose.position.x = map_to_robot.transform.translation.x;
    robot_pose.position.y = map_to_robot.transform.translation.y;
    robot_pose.position.z = map_to_robot.transform.translation.z;
    robot_pose.orientation = map_to_robot.transform.rotation;
    return true;
}
