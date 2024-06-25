#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <robot_localization/srv/set_pose.hpp>
#include <occupancy_grid_aggregator_srv/srv/reset_aggregate_grid.hpp>

class EkfSupervisor : public rclcpp::Node
{
    public:
        EkfSupervisor();

        ~EkfSupervisor() {};
        EkfSupervisor (const EkfSupervisor&) = delete;
        EkfSupervisor& operator= (const EkfSupervisor&) = delete;

    private:
        void run();

        /**
         * @brief Returns the latest robot_pose from the TF2 tree.
         * @return True if a valid pose was found. False otherwise.
         */
        bool get_robot_pose(geometry_msgs::msg::Pose &robot_pose);

        void reset_ekf();

        std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> _tf_listener;

        rclcpp::TimerBase::SharedPtr _run_timer;
        rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr _set_pose_client;
        rclcpp::Client<occupancy_grid_aggregator_srv::srv::ResetAggregateGrid>::SharedPtr _reset_aggregate_grid_client;
};