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
            parameters=[{"ROS_IP": "127.0.0.1"}, {"ROS_TCP_PORT": 10000}],
        ),

        # ==== Vision ====
        launch_ros.actions.Node(
            package='redshell_vision',
            executable='track_error_driver',
            parameters=[{"is_sitl": True}],
        ),

        # ==== State Estimation ====
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('state_estimation'), 'launch/state_estimation.launch.py')
            )
        )
    ])