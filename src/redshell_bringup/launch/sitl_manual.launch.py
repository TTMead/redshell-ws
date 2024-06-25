import os
import yaml
import pathlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        # ==== Unity Interfacing ====
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            parameters=[{"ROS_IP": "127.0.0.1"}, {"ROS_TCP_PORT": 10000}]
        ),

        # ==== State Estimation ====
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('state_estimation'), 'launch/state_estimation.launch.py')
            )
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
                {"use_sim_time": True},
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
                {"use_sim_time": True},
                {"occupancy_grid_topic": "/potential_field_combined"}, 
                {"path_topic": "/path"}
            ]
        ),

        # ==== Joystick ====
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',

            # "Right-Handed 1-Stick" configuration
            # parameters=[{"enable_button": 5}, {"axis_linear.x": 4}, {"axis_angular.yaw": 3}],

            # "2-Stick" configuration
            parameters=[{"use_sim_time": True}, {"enable_button": 10}, {"axis_linear.x": 3}, {"axis_angular.yaw": 0}]
        ),
        Node(
            package='joy',
            executable='joy_node',
            parameters=[{"use_sim_time": True}, {"autorepeat_rate": 20.0}]
        ),

        # ==== Diagnostics ====
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(get_package_share_directory('foxglove_bridge'),'launch/foxglove_bridge_launch.xml')
            )
        )
    ])