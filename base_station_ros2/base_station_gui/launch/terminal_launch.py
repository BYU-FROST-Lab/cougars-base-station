
import sys
import os

import launch
import launch_ros.actions

def generate_launch_description():


    param_file = os.path.join('/home/frostlab/config', 'base_station_params.yaml')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='seatrac',
            executable='modem',
            name='modem',
            parameters=[param_file]
        ),
        launch_ros.actions.Node(
            package='base_station_coms',
            executable='base_station_coms',
            name='base_station_coms',
            parameters=[param_file]
        ),
        launch_ros.actions.Node(
            package='base_station_coms',
            executable='base_station_modem',
            name='base_station_modem',
            parameters=[param_file]
        ),        
        launch_ros.actions.Node(
            package='base_station_gui',
            executable='gui_node',
            name='base_station_gui',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='base_station_coms',
            executable='base_station_radio.py',
            name='base_station_radio',
            parameters=[param_file]
        ),
        launch_ros.actions.Node(
            package='base_station_coms',
            executable='base_station_wifi.py',
            name='base_station_wifi',
            parameters=[param_file]
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()

