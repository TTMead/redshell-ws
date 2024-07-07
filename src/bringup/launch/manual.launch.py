
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        # ==== Motor IO ====
        launch_ros.actions.Node(
            package='redshell_io',
            executable='redshell_interface',
        ),

        # ==== Joystick ====
        launch_ros.actions.Node(
            package='teleop_twist_joy',
            executable='teleop_node',

            # Right-Handed 1 stick configuration
            # parameters=[{"enable_button": 5}, {"axis_linear.x": 4}, {"axis_angular.yaw": 3}],

            # 2 stick configuration
            parameters=[{"use_sim_time": False}, {"enable_button": 10}, {"axis_linear.x": 3}, {"axis_angular.yaw": 0}],
        ),
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            parameters=[{"use_sim_time": False}, {"autorepeat_rate": 20.0}],
        ),

        # ==== State Estimation ====
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('state_estimation'), 'launch/state_estimation.launch.py')
            ), 
            launch_arguments={'sitl': 'False'}.items()
        ),

        # ==== Diagnostics ====
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(get_package_share_directory('foxglove_bridge'),'launch/foxglove_bridge_launch.xml')
            )
        )
    ])
