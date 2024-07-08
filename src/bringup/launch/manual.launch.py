import os
import pathlib

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        # ==== Motor IO ====
        Node(
            package='redshell_io',
            executable='redshell_interface',
        ),

        # ==== Joystick ====
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',

            # Right-Handed 1 stick configuration
            # parameters=[{"enable_button": 5}, {"axis_linear.x": 4}, {"axis_angular.yaw": 3}],

            # 2 stick configuration
            parameters=[{"use_sim_time": False}, {"enable_button": 10}, {"axis_linear.x": 3}, {"axis_angular.yaw": 0}],
        ),
        Node(
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

        # ==== Camera Drivers ====
        Node(
            package='vision',
            executable='potential_field_driver',
            name='front_camera_driver',
            parameters=[
                {"is_sitl": False}, 
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
                {"use_sim_time": False},
                {"field_topics": ["/front_field"]}, 
                {"aggregate_frame_id": "/map"}
            ]
        ),

        # ==== Path Planner ====
        Node(
            package='path_planning',
            executable='path_planner',
            name='path_planner',
            parameters=[
                {"use_sim_time": False},
                {"occupancy_grid_topic": "/potential_field_combined"}, 
                {"path_topic": "/path"}
            ]
        ),

        # ==== Diagnostics ====
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(get_package_share_directory('foxglove_bridge'),'launch/foxglove_bridge_launch.xml')
            )
        )
    ])
