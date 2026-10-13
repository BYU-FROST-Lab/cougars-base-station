import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'param_file',
            default_value='/home/frostlab/config/base_station_params.yaml',
            description='Path to the base station parameter file'
        ),
        DeclareLaunchArgument(
            'origin',
            default_value='default_site',
            description='Origin for the base station'
        ),
        launch_ros.actions.Node(
            package='base_station_gui',
            executable='gui_node',
            name='base_station_gui',
            output='screen',
            additional_env={
                'BASE_STATION_PARAM_FILE': LaunchConfiguration('param_file')
            },
            parameters=[LaunchConfiguration('origin')],
        ),
    ])
