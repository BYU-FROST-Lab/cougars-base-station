import rclpy
from rclpy.node import Node
import launch
import yaml
import launch_ros.actions
import launch_ros.descriptions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
MAXSUB=99
VISUALIZER_PATH="/home/frostlab/base_station/base-station-ros2/src/base_station_utils/base_station_utils/temp_mission_visualizer"
CONFIG_PATH="/home/frostlab/config/mapvizconfig.mvc"

def generate_launch_description():
    base_params_path = "/home/frostlab/base_station/base-station-ros2/base_station_params.yaml"
    activesubs=[False]*MAXSUB

    rclpy.init()
    dummynode = Node('launch_topic_inspector')
    alltopics = dummynode.get_topic_names_and_types()
    dummynode.destroy_node()
    rclpy.shutdown()
    topicnames=[name for name, _ in alltopics]
    for i in range(MAXSUB): 
        if f"/coug{i}/map_viz_path" in topicnames:
            activesubs[i]=True
            print(f"Coug {i} path showing")
    config = {
        'capture_directory': "~",
        'fixed_frame': 'map',
        'target_frame': '<none>',
        'fix_orientation': False,
        'rotate_90': False,
        'enable_antialiasing': True,
        'show_displays': True,
        'show_status_bar': True,
        'show_capture_tools': True,
        'window_width': 1219,
        'window_height': 600,
        'view_scale': 47.4682999,
        'offset_x': 986.523132,
        'offset_y': 18340.3848,
        'use_latest_transforms': True,
        'background': "#a0a0a4",
        'image_transport': 'raw',
        'displays': [
            {
                'name': "google maps offline",
                'type': "mapviz_plugins/tile_map",
                'enabled': True,
                'config': {
                    'visible': True,
                    'collapsed': True,
                    'custom_sources': [
                        {
                            'base_url': "http://localhost:8080/wmts/gm_layer/gm_grid/{level}/{x}/{y}.png",
                            'max_zoom': 17,
                            'name': "offlinesource",
                            'type': "wmts"
                        }
                    ],
                    'bing_api_key': "",
                    'source': "offlinesource"
                    }
                }
            ]
            }
    for coug in range(MAXSUB):
        if activesubs[coug]:
            #add path config
            pconfig={
                    'name': f"path{coug}",
                    'type': "mapviz_plugins/path",
                    'enabled': True,
                    'config': {
                            'visible': True,
                            'collapsed': True,
                            'topic':f'/coug{coug}/map_viz_path',
                            'color': "#00ff00",
                            'qos_depth': 10,
                            'qos_history': 1,
                            'qos_reliability': 1,
                            'qos_durability': 2
                        }
                    }
            #add gps config
            oconfig={
                'name': f'odom{coug}',
                'type': 'mapviz_plugins/odometry',
                'enabled': True,
                'config': {
                        'visible': True,
                        'collapsed': True,
                        'topic':f'/coug{coug}/smoothed_output',
                        'color': "#00ff00",
                        'qos_depth': 10,
                        'qos_history': 1,
                        'qos_reliability': 1,
                        'qos_durability': 2
                }
            }
            config['displays'].append(pconfig)
            config['displays'].append(oconfig)
    with open(CONFIG_PATH,"w") as file:
        yaml.dump(config,file,sort_keys=False)
    launch_actions=list()
    arg=DeclareLaunchArgument(
        'config',
        default_value=CONFIG_PATH,
        description='Path to a Mapviz config file.'
        )
    launch_actions.extend([ #use period if needed
        launch.actions.ExecuteProcess( #start map tile server
            cmd=['bash',VISUALIZER_PATH+"/launch_server.sh"]
        ),
        arg,
        launch_ros.actions.Node( #mapviz 
            package='mapviz',
            executable='mapviz',
            name='mapviz',
            output='screen',
            parameters= [base_params_path]
            # arguments=[LaunchConfiguration('config')] #path of mapviz config yaml
        ),
        # launch_ros.actions.Node( #origin broadcaster
        #     package='base_station_utils',
        #     executable='originbroadcaster'
        # )
        launch_ros.actions.Node(
            package="swri_transform_util",
            executable="initialize_origin.py",
            name="initialize_origin",
            parameters= [base_params_path,
                {"local_xy_frame": "map"},
                {"local_xy_origin": "swri"},
                {"local_xy_origins": """[
                    {"name": "swri",
                        "latitude": 40.249999 ,
                        "longitude": -111.6499974,
                        "altitude": 1387.0,
                        "heading": 0.0},
                    {"name": "back_40",
                        "latitude": 29.447507,
                        "longitude": -98.629367,
                        "altitude": 1350.0,
                        "heading": 0.0}
                ]"""}]
        ),
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="swri_transform",
            arguments=["0", "0", "0", "0", "0", "0", "map", "origin"]
        )
        ])
    return launch.LaunchDescription(launch_actions)