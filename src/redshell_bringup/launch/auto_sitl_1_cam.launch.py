from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # ==== Unity Interfacing ====
        launch_ros.actions.Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            parameters=[{"ROS_IP": "127.0.0.1"}, {"ROS_TCP_PORT": 10000}]
        ),

        # ==== Camera Drivers ====
        launch_ros.actions.Node(
            package='redshell_vision',
            executable='potential_field_driver',
            name='front_camera_driver',
            parameters=[
                {"is_sitl": True}, 
                {"camera_topic": "/front_camera"}, 
                {"field_topic": "/front_field"}, 
                {"frame_id": "front_cam"}
            ]
        ),

        # ==== Occupancy Aggregator ====
        launch_ros.actions.Node(
            package='occupancy_grid_aggregator',
            executable='aggregator_node',
            name='aggregator_node',
            parameters=[
                {"field_topics": ["/front_field"]}, 
                {"aggregate_frame_id": "/map"}
            ]
        ),

        # ==== Path Planner ====
        launch_ros.actions.Node(
            package='redshell_guidance',
            executable='path_planner',
            name='path_planner',
            parameters=[
                {"occupancy_grid_topic": "/potential_field_combined"}, 
                {"path_topic": "/path"}
            ]
        ),

        # ==== Pure Pursuit Controller ====
        # launch_ros.actions.Node(
        #     package='redshell_control',
        #     executable='pure_pursuit',
        #     name='pure_pursuit',
        #     parameters=[
        #         {"path_topic": "/path"}
        #     ]
        # ),

        # ==== State Estimation ====
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{"use_sim_time": False}, os.path.join(get_package_share_directory("redshell_bringup"), 'config', 'state_estimation_params.yaml')]
        ),
        
        # ==== Static Broadcasters ====
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["0.164", "0", "0.428", "0", "0", "0", "base_link", "front_cam"]
        )
    ])