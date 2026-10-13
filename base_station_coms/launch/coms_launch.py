import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=os.path.join('/home/frostlab/config', 'base_station_params.yaml'),
        description='Path to the base station parameter YAML file'
    )

    param_file = LaunchConfiguration('param_file')

    return LaunchDescription([
        param_file_arg,
        Node(
            package='base_station_coms',
            executable='base_station_wifi.py',
            name='base_station_wifi',
            parameters=[param_file],
            output='screen'
        ),
        Node(
            package='base_station_coms',
            executable='base_station_modem',
            name='base_station_modem',
            parameters=[param_file],
            output='screen'
        ),
        Node(
            package='base_station_coms',
            executable='base_station_radio.py',
            name='base_station_radio',
            parameters=[param_file],
            output='screen',
        ),
        Node(
            package='base_station_coms',
            executable='teleop_couguv_key',
            name='teleop_couguv_key',
            parameters=[param_file],
            output='screen'
        ),
    ])
