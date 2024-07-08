import os
import yaml

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # ==== Motor IO ====
        Node(
            package='redshell_io',
            executable='redshell_interface',
        ),

        # ==== Vision Error Node ====
        Node(
            package='vision',
            executable='track_error_driver',
            parameters=[
                {"is_sitl": False},
                {"camera_index": 0}
            ],
        ),

        # ==== Visual Servo Controller ====
        Node(
            package='control',
            executable='vs_control',
            name='vs_control',
            parameters=[
                {"kp": 0.7},
                {"speed": 0.2}
            ]
        )
    ])