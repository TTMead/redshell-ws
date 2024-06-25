import os
import yaml
import pathlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        # ==== Unity Interfacing ====
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            parameters=[{"ROS_IP": "127.0.0.1"}, {"ROS_TCP_PORT": 10000}]
        ),

        # ==== Camera Drivers ====
        Node(
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
        Node(
            package='occupancy_grid_aggregator',
            executable='aggregator_node',
            name='aggregator_node',
            parameters=[
                {"field_topics": ["/front_field"]}, 
                {"aggregate_frame_id": "/map"}
            ]
        ),

        # ==== Path Planner ====
        Node(
            package='redshell_guidance',
            executable='path_planner',
            name='path_planner',
            parameters=[
                {"occupancy_grid_topic": "/potential_field_combined"}, 
                {"path_topic": "/path"}
            ]
        ),

        # ==== Pure Pursuit Controller ====
        Node(
            package='redshell_control',
            executable='pure_pursuit',
            name='pure_pursuit',
            parameters=[
                {"path_topic": "/path"}
            ]
        ),

        # ==== State Estimation ====
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{"use_sim_time": False}, os.path.join(get_package_share_directory("redshell_bringup"), 'config', 'state_estimation_params.yaml')]
        ),
        
        # ==== Static Broadcasters ====
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["0.164", "0", "0.428", "0", "0", "0", "base_link", "front_cam"]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu"]
        ),

        # ==== Diagnostics ====
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(get_package_share_directory('foxglove_bridge'),'launch/foxglove_bridge_launch.xml')
            )
        )
    ])