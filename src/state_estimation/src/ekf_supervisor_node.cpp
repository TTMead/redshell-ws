#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <robot_localization/srv/set_pose.hpp>
#include <occupancy_grid_aggregator_srv/srv/reset_aggregate_grid.hpp>

bool get_robot_pose(geometry_msgs::msg::Pose &robot_pose, const std::shared_ptr<rclcpp::Node>& node, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer)
{
    geometry_msgs::msg::TransformStamped map_to_robot;
    try
    {
        static constexpr char map_frame[] = "map";
        static constexpr char robot_frame[] = "base_link";
        map_to_robot = tf_buffer->lookupTransform(map_frame, robot_frame, tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_DEBUG_THROTTLE(node->get_logger(), *(node->get_clock()), 1000, "Wating for state estimation: %s", ex.what());
        return false;
    }
    robot_pose.position.x = map_to_robot.transform.translation.x;
    robot_pose.position.y = map_to_robot.transform.translation.y;
    robot_pose.position.z = map_to_robot.transform.translation.z;
    robot_pose.orientation = map_to_robot.transform.rotation;
    return true;
}

geometry_msgs::msg::PoseWithCovarianceStamped create_empty_pose(const rclcpp::Time& now)
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

void reset_ekf(const std::shared_ptr<rclcpp::Node>& node,
    const std::shared_ptr<rclcpp::Client<robot_localization::srv::SetPose>>& set_pose_client,
    const std::shared_ptr<rclcpp::Client<occupancy_grid_aggregator_srv::srv::ResetAggregateGrid>>& reset_aggregate_grid_client)
{
    auto ekf_setpose_request = std::make_shared<robot_localization::srv::SetPose::Request>();
    ekf_setpose_request->pose = create_empty_pose(node->get_clock()->now());
    while (!set_pose_client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for EKF service. Exiting.");
        }
        RCLCPP_INFO(node->get_logger(), "Service not available, cannot reset EKF.");
    }

    auto ekf_result_future = set_pose_client->async_send_request(ekf_setpose_request,
        [node](rclcpp::Client<robot_localization::srv::SetPose>::SharedFuture future){
            auto status = future.wait_for(std::chrono::seconds(1));
            if (status == std::future_status::ready) {
                RCLCPP_INFO(node->get_logger(), "EKF has been reset");
            } else {
                RCLCPP_INFO(node->get_logger(), "Failed to reset EKF");
            }
        }
    );

    auto grid_reset_request = std::make_shared<occupancy_grid_aggregator_srv::srv::ResetAggregateGrid::Request>();
    while (!reset_aggregate_grid_client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for occupancy grid service. Exiting.");
        }
        RCLCPP_INFO(node->get_logger(), "Service not available, cannot reset EKF.");
    }

    auto grid_reset_result_future = reset_aggregate_grid_client->async_send_request(grid_reset_request,
        [node](rclcpp::Client<occupancy_grid_aggregator_srv::srv::ResetAggregateGrid>::SharedFuture future){
            auto status = future.wait_for(std::chrono::seconds(1));
            if (status == std::future_status::ready) {
                RCLCPP_INFO(node->get_logger(), "Occupancy grid has been reset");
            } else {
                RCLCPP_INFO(node->get_logger(), "Failed to reset occupancy grid");
            }
        }
    );
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("ekf_supervisor");

    node->declare_parameter("reset_distance_m", 15.0);

    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Initialise the services to send when the robot goes out of bounds
    auto set_pose_client = node->create_client<robot_localization::srv::SetPose>("set_pose");
    auto reset_aggregate_grid_client = node->create_client<occupancy_grid_aggregator_srv::srv::ResetAggregateGrid>("reset_aggregate_grid");

    // Setup the run loop that checks if the robot is out of bounds
    auto run_timer = node->create_wall_timer(std::chrono::milliseconds(100), [&](){
        geometry_msgs::msg::Pose robot_pose;
        if (get_robot_pose(robot_pose, node, tf_buffer))
        {
            const double reset_distance_m = node->get_parameter("reset_distance_m").as_double();
            const bool robot_out_of_bounds = (std::abs(robot_pose.position.x) >= reset_distance_m)
                                          || (std::abs(robot_pose.position.y) >= reset_distance_m);
            if (robot_out_of_bounds)
            {
                reset_ekf(node, set_pose_client, reset_aggregate_grid_client);
            }
        }
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
