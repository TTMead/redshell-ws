#include "ekf_supervisor.h"

using namespace std::chrono_literals;

EkfSupervisor::EkfSupervisor() : Node("ekf_supervisor")
{
    this->declare_parameter("reset_distance_m", 15.0);

    _tf_buffer.reset(new tf2_ros::Buffer(this->get_clock()));
    _tf_listener.reset(new tf2_ros::TransformListener(*_tf_buffer));

    _run_timer = this->create_wall_timer(100ms, std::bind(&EkfSupervisor::run, this));

    _set_pose_client = this->create_client<robot_localization::srv::SetPose>("set_pose");
    _reset_aggregate_grid_client = this->create_client<occupancy_grid_aggregator_srv::srv::ResetAggregateGrid>("reset_aggregate_grid");
}

void
EkfSupervisor::run()
{
    geometry_msgs::msg::Pose robot_pose;
    if (get_robot_pose(robot_pose))
    {
        const double reset_distance_m = this->get_parameter("reset_distance_m").as_double();
        const bool robot_out_of_bounds = (std::abs(robot_pose.position.x) >= reset_distance_m)
                                      || (std::abs(robot_pose.position.y) >= reset_distance_m);
        if (robot_out_of_bounds)
        {
            reset_ekf();
        }
    }
}

bool
EkfSupervisor::get_robot_pose(geometry_msgs::msg::Pose &robot_pose)
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
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Wating for state estimation: %s", ex.what());
        return false;
    }

    // Extract pose from TF
    robot_pose.position.x = map_to_robot.transform.translation.x;
    robot_pose.position.y = map_to_robot.transform.translation.y;
    robot_pose.position.z = map_to_robot.transform.translation.z;
    robot_pose.orientation = map_to_robot.transform.rotation;

    return true;
}

static geometry_msgs::msg::PoseWithCovarianceStamped create_empty_pose(rclcpp::Time now)
{
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = now;
    pose.pose.pose.position.x = 0;
    pose.pose.pose.position.y = 0;
    pose.pose.pose.position.z = 0;
    pose.pose.pose.orientation.x = 0;
    pose.pose.pose.orientation.y = 0;
    pose.pose.pose.orientation.z = 0;
    pose.pose.pose.orientation.w = 1;
    static constexpr double epsilon = 1e-3;
    pose.pose.covariance = {epsilon, 0, 0, 0, 0, 0,
                            0, epsilon, 0, 0, 0, 0,
                            0, 0, epsilon, 0, 0, 0,
                            0, 0, 0, epsilon, 0, 0,
                            0, 0, 0, 0, epsilon, 0,
                            0, 0, 0, 0, 0, epsilon};
    return pose;
}

void
EkfSupervisor::reset_ekf()
{
    auto ekf_setpose_request = std::make_shared<robot_localization::srv::SetPose::Request>();
    ekf_setpose_request->pose = create_empty_pose(this->get_clock()->now());
    while (!_set_pose_client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for EKF service. Exiting.");
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, cannot reset EKF.");
    }

    auto ekf_result_future = _set_pose_client->async_send_request(ekf_setpose_request,
        [this](rclcpp::Client<robot_localization::srv::SetPose>::SharedFuture future){
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready) {
                RCLCPP_INFO(this->get_logger(), "EKF has been reset");
            } else {
                RCLCPP_INFO(this->get_logger(), "Failed to reset EKF");
            }
        }
    );


    

    auto grid_reset_request = std::make_shared<occupancy_grid_aggregator_srv::srv::ResetAggregateGrid::Request>();
    while (!_reset_aggregate_grid_client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for occupancy grid service. Exiting.");
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, cannot reset EKF.");
    }

    auto grid_reset_result_future = _reset_aggregate_grid_client->async_send_request(grid_reset_request,
        [this](rclcpp::Client<occupancy_grid_aggregator_srv::srv::ResetAggregateGrid>::SharedFuture future){
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready) {
                RCLCPP_INFO(this->get_logger(), "Occupancy grid has been reset");
            } else {
                RCLCPP_INFO(this->get_logger(), "Failed to reset occupancy grid");
            }
        }
    );

    
}
