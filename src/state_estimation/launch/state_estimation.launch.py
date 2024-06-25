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
        # ==== EKF Node ====
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{"use_sim_time": False}, os.path.join(get_package_share_directory("state_estimation"), 'config', 'state_estimation_params.yaml')]
        ),

        # ==== EKF Supervisor ====
        Node(
            package='state_estimation',
            executable='ekf_supervisor',
            name='ekf_supervisor',
            output='screen'
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
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu"]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["0.164", "0", "0.428", "0", "0", "0", "base_link", "front_cam"]
        )
    ])