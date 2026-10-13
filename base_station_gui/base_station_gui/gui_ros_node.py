import base_station_gui.base_station_gui

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import QTimer

import sys
import threading
import signal
import yaml
import os
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as RclParameter, ParameterType

import time
from std_msgs.msg import String, Bool, Int32
from nav_msgs.msg import Odometry

from nav_msgs.msg import Path #used to publish the map viz path
from geographic_msgs.msg import GeoPoint, RouteNetwork, WayPoint, KeyValue
from sensor_msgs.msg import FluidPressure, BatteryState
from geometry_msgs.msg import PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped
from unique_identifier_msgs.msg import UUID
from diagnostic_msgs.msg import DiagnosticStatus

from base_station_interfaces.msg import ConsoleLog, UCommandBase
from cougars_interfaces.msg import SystemStatus, SystemControl, UCommand, MissionFeedback, WaypointFeedback
from dvl_msgs.msg import DVL


def _make_uuid(index: int) -> UUID:
    uuid = UUID()
    uuid.uuid = [0] * 16
    packed = struct.pack('>I', index & 0xFFFFFFFF)
    uuid.uuid[12] = packed[0]
    uuid.uuid[13] = packed[1]
    uuid.uuid[14] = packed[2]
    uuid.uuid[15] = packed[3]
    return uuid


def _kv(key: str, value: str) -> KeyValue:
    kv = KeyValue()
    kv.key = key
    kv.value = value
    return kv

class GuiNode(Node):
    """
    ROS 2 node that connects the GUI to ROS topics and services.
    Handles publishing, subscribing, and service clients for the GUI.
    """
    def __init__(self, window, selected_cougs):
        super().__init__('gui_node')

        # Dynamically create subscriptions and publishers for each selected vehicle (coug)
        for coug_number in selected_cougs:
            # Subscribe to safety status messages for each vehicle
            sub = self.create_subscription(
                SystemStatus,
                f'coug{coug_number}/safety_status',
                lambda msg, n=coug_number: window.recieve_safety_status_message(n, msg),
                10
            )
            setattr(self, f'safety_status_subscription{coug_number}', sub)

            # Subscribe to state estimate messages for each vehicle
            sub = self.create_subscription(
                Odometry,
                f'coug{coug_number}/odometry/global',
                lambda msg, n=coug_number: window.recieve_state_estimate_message(n, msg),
                10
            )
            setattr(self, f'state_estimate_subscription{coug_number}', sub)

            # Subscribe to DVL velocity for each vehicle.
            # The DVL publishes best effort, so a reliable subscription would never connect.
            sub = self.create_subscription(
                DVL,
                f'coug{coug_number}/dvl/data',
                lambda msg, n=coug_number: window.recieve_dvl_velocity(n, msg),
                qos_profile_sensor_data
            )
            setattr(self, f'dvl_vel_subscription{coug_number}', sub)

            # Subscribe to depth data messages for each vehicle (pressure_to_depth publishes Odometry)
            sub = self.create_subscription(
                Odometry,
                f'coug{coug_number}/depth/odom',
                lambda msg, n=coug_number: window.recieve_depth_data_message(n, msg),
                10
            )
            setattr(self, f'depth_data_subscription{coug_number}', sub)            
            
            # Subscribe to pressure data topic for each vehicle
            sub = self.create_subscription(
                FluidPressure,
                f'coug{coug_number}/pressure/data',
                lambda msg, n=coug_number: window.recieve_pressure_data_message(n, msg),
                10
            )
            setattr(self, f'pressure_data_subscription{coug_number}', sub)

            # Subscribe to battery data messages for each vehicle
            sub = self.create_subscription(
                BatteryState,
                f'coug{coug_number}/battery/data',
                lambda msg, n=coug_number: window.recieve_battery_data_message(n, msg),
                10
            )
            setattr(self, f'battery_data_subscription{coug_number}', sub)

            sub = self.create_subscription(
                MissionFeedback,
                f'coug{coug_number}/mission_feedback',
                lambda msg, n=coug_number: window.recieve_mission_feedback(n, msg),
                10
            )
            setattr(self, f'mission_feedback_subscription{coug_number}', sub)

            sub = self.create_subscription(
                WaypointFeedback,
                f'coug{coug_number}/waypoint_feedback',
                lambda msg, n=coug_number: window.recieve_waypoint_feedback(n, msg),
                10
            )
            setattr(self, f'waypoint_feedback_subscription{coug_number}', sub)

            # Publisher for system status messages for each vehicle
            pub = self.create_publisher(
                SystemControl,
                f'/coug{coug_number}/system/status',
                1
            )
            setattr(self, f'coug{coug_number}_publisher_', pub)

            pub = self.create_publisher(
                RouteNetwork,
                f'coug{coug_number}/load_mission',
                10
            )
            setattr(self, f'coug{coug_number}_load_mission_pub', pub)

            pub = self.create_publisher(
                SystemControl,
                f'coug{coug_number}/start_mission',
                10
            )
            setattr(self, f'coug{coug_number}_start_mission_pub', pub)

            pub = self.create_publisher(
                Bool,
                f'coug{coug_number}/emergency_kill',
                10
            )
            setattr(self, f'coug{coug_number}_emergency_kill_pub', pub)

            pub = self.create_publisher(
                Bool,
                f'coug{coug_number}/emergency_surface',
                10
            )
            setattr(self, f'coug{coug_number}_emergency_surface_pub', pub)

            pub = self.create_publisher(
                String,
                f'coug{coug_number}/hardware_control',
                10
            )
            setattr(self, f'coug{coug_number}_hardware_control_pub', pub)

            # Publisher for map visualization paths for each vehicle
            pub = self.create_publisher(
                Path,
                f'/coug{coug_number}/map_viz_path',
                10
            )
            setattr(self, f'coug{coug_number}_path_', pub)

            # Publisher for vehicle fins kinematics command for each vehicle
            pub = self.create_publisher(
                UCommand,
                f'/coug{coug_number}/kinematics/command',
                10
            )
            setattr(self, f'coug{coug_number}_fins_kinematics', pub)

            # Publisher for vehicle fins controls command for each vehicle
            pub = self.create_publisher(
                UCommand,
                f'/coug{coug_number}/controls/command',
                10
            )
            setattr(self, f'coug{coug_number}_fins_controls', pub)

            # Client for setting kinematics parameters for each vehicle
            client = self.create_client(
                SetParameters,
                f'/coug{coug_number}/coug_kinematics'
            )
            setattr(self, f'coug{coug_number}_kinematics_client', client)

            sub = self.create_subscription(
                DiagnosticStatus,
                f'coug{coug_number}/link_status',
                lambda msg, n=coug_number: window.recieve_link_status(n, msg),
                10
            )
            setattr(self, f'link_status_subscription{coug_number}', sub)

        # Subscription for emergency kill confirmation messages
        self.kill_subscription = self.create_subscription(
            Bool,
            'confirm_e_kill',
            window.recieve_kill_confirmation_message,  # Calls the GUI's recieve_kill_confirmation_message method
            10)        
            
        # Subscription for emergency surface confirmation messages
        self.surface_subscription = self.create_subscription(
            Bool,
            'confirm_e_surface',
            window.recieve_surface_confirmation_message,  # Calls the GUI's recieve_surface_confirmation_message method
            10)

        # Subscription for console log updates, specific to vehicles. 0 means send to all
        self.console_log_sub = self.create_subscription(
            ConsoleLog,
            'console_log',
            window.handle_console_log,
            10
        )

        # Publisher for the shared origin topic used by vehicle navigation.
        origin_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.origin_pub = self.create_publisher(GeoPoint, 'send_origin', origin_qos)

        # Teleop status, published by teleop_couguv_key.cpp for the Keyboard Controls tab.
        # Same TRANSIENT_LOCAL QoS as origin_qos so the tab gets current state immediately,
        # even if the GUI was opened after teleop_couguv_key already started.
        self.teleop_vehicle_sub = self.create_subscription(
            Int32,
            '/teleop_status/vehicle_id',
            lambda msg: window.recieve_teleop_vehicle(msg.data),
            origin_qos
        )
        self.teleop_thruster_sub = self.create_subscription(
            Bool,
            '/teleop_status/thruster_enabled',
            lambda msg: window.recieve_teleop_thruster(msg.data),
            origin_qos
        )
        self.teleop_publishing_sub = self.create_subscription(
            Bool,
            '/teleop_status/publishing_enabled',
            lambda msg: window.recieve_teleop_publishing(msg.data),
            origin_qos
        )
        self.teleop_hard_turn_sub = self.create_subscription(
            Bool,
            '/teleop_status/hard_turn_mode',
            lambda msg: window.recieve_teleop_hard_turn(msg.data),
            origin_qos
        )

        # Live fin/thruster command, published by teleop_couguv_key.cpp at its configured
        # publish rate while keyboard controls are enabled. Drives the Keyboard Controls tab's
        # live speed/turn/pitch gauges. Default (reliable, volatile, depth 10) QoS to match
        # the publisher.
        self.teleop_command_sub = self.create_subscription(
            UCommandBase,
            '/keyboard_controls',
            lambda msg: window.recieve_teleop_command(msg),
            10
        )

        # Publisher for console log messages
        self.console_publisher = self.create_publisher(ConsoleLog, 'console_log', 10)

        # Publisher for key press events to teleop
        self.keypress_publisher = self.create_publisher(String, 'gui_keypress', 10)

    def publish_console_log(self, msg_text, msg_num):
        """
        Publishes a console log message to the 'console_log' topic.
        """
        msg = ConsoleLog()
        msg.message = msg_text
        msg.vehicle_number = msg_num
        self.console_publisher.publish(msg)

    def publish_keypress(self, key_text):
        """
        Publishes a key press event to the 'gui_keypress' topic.
        """
        msg = String()
        msg.data = key_text
        self.keypress_publisher.publish(msg)

    def publish_load_mission(self, vehicle_number, mission_file_path):
        if not mission_file_path:
            self.get_logger().error(f"No mission file selected for Coug {vehicle_number}")
            self.publish_console_log(f"Mission loading failed for Coug {vehicle_number}: no file selected", vehicle_number)
            return
        msg = self.load_route_network(mission_file_path)
        getattr(self, f'coug{vehicle_number}_load_mission_pub').publish(msg)
        self.publish_console_log(f"Published load mission command for Coug {vehicle_number}", vehicle_number)

    def publish_start_mission(self, vehicle_number, start_config):
        msg = SystemControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'system_status_input'
        msg.start = Bool(data=start_config["start_node"])
        msg.rosbag_flag = Bool(data=start_config["record_rosbag"])
        msg.rosbag_prefix = start_config["rosbag_prefix"]
        msg.thruster_arm = Bool(data=start_config["arm_thruster"])
        msg.dvl_acoustics = Bool(data=start_config["start_dvl"])
        getattr(self, f'coug{vehicle_number}_start_mission_pub').publish(msg)
        self.publish_console_log(f"Published start mission command for Coug {vehicle_number}", vehicle_number)

    def publish_emergency_kill(self, vehicle_number):
        getattr(self, f'coug{vehicle_number}_emergency_kill_pub').publish(Bool(data=True))
        self.publish_console_log(f"Published emergency kill command for Coug {vehicle_number}", vehicle_number)

    def publish_emergency_surface(self, vehicle_number):
        getattr(self, f'coug{vehicle_number}_emergency_surface_pub').publish(Bool(data=True))
        self.publish_console_log(f"Published emergency surface command for Coug {vehicle_number}", vehicle_number)

    def publish_hardware_control(self, vehicle_number, device, mode):
        """
        Sends a relay/strobe control command to the vehicle over radio.
        device: "RELAY" or "STROBE". mode: "AUTO", "ON", or "OFF".
        """
        msg = String()
        msg.data = f"{device}:{mode}"
        getattr(self, f'coug{vehicle_number}_hardware_control_pub').publish(msg)
        self.publish_console_log(
            f"Sent {device.title()} {mode.title()} command for Coug {vehicle_number} over radio",
            vehicle_number
        )

    def load_route_network(self, mission_file_path):
        self.get_logger().debug(f'Loading mission file: "{mission_file_path}"')
        try:
            with open(mission_file_path, 'r') as f:
                data = yaml.safe_load(f)
        except (OSError, yaml.YAMLError) as e:
            self.get_logger().error(f'Failed to load mission file "{mission_file_path}": {e}')
            return RouteNetwork()

        self.get_logger().debug(f'Parsed mission file type: {type(data).__name__}')
        self.get_logger().debug(f'Parsed mission file content: {data}')

        if not isinstance(data, dict) or not data:
            self.get_logger().error(f'Mission file "{mission_file_path}" must be a non-empty mapping.')
            return RouteNetwork()

        self.get_logger().debug(f'Mission top-level keys: {list(data.keys())}')

        mission_key = None
        mission_value = None
        for key, value in data.items():
            if isinstance(value, dict) and 'waypoints' in value:
                mission_key = key
                mission_value = value
                break
            if isinstance(value, list):
                mission_key = key
                mission_value = value
                break

        if mission_key is None:
            self.get_logger().error(
                f'Mission file "{mission_file_path}" does not contain a mission block with waypoints.'
            )
            return RouteNetwork()

        self.get_logger().debug(f'Mission block key: {mission_key}')
        if isinstance(mission_value, list):
            defaults = {}
            waypoints = mission_value
            self.get_logger().debug(f'Mission format: list with {len(waypoints)} waypoints')
        else:
            defaults = mission_value.get('defaults', {})
            waypoints = mission_value.get('waypoints', [])
            self.get_logger().debug(
                f"Mission format: dict with {len(waypoints)} waypoints and defaults {defaults}"
            )

        return self.build_route_network(defaults, waypoints)

    def build_route_network(self, defaults, waypoints):
        self.get_logger().debug(f'Building RouteNetwork with {len(waypoints)} waypoints')
        msg = RouteNetwork()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'wgs84'

        mission_id = int(defaults.get('mission_id', 0))
        msg.id = _make_uuid(mission_id)

        msg.props.append(_kv('speed', str(defaults.get('speed', 50.0))))
        msg.props.append(_kv('slip_radius', str(defaults.get('slip_radius', 2.0))))
        msg.props.append(_kv('capture_radius', str(defaults.get('capture_radius', 10.0))))

        for i, wp_data in enumerate(waypoints):
            self.get_logger().debug(f'Waypoint {i}: {wp_data}')
            wp = WayPoint()
            wp.id = _make_uuid(i)
            wp.position = GeoPoint()
            wp.position.latitude = float(wp_data.get('lat', 0.0))
            wp.position.longitude = float(wp_data.get('lon', 0.0))
            wp.position.altitude = float(wp_data.get('z', 0.0))
            wp.props.append(_kv('depth_ref', wp_data.get('depth_ref', 'surface')))
            wp.props.append(_kv('park', 'true' if wp_data.get('park', False) else 'false'))
            if 'speed' in wp_data:
                wp.props.append(_kv('speed', str(wp_data['speed'])))
            if 'slip_radius' in wp_data:
                wp.props.append(_kv('slip_radius', str(wp_data['slip_radius'])))
            if 'capture_radius' in wp_data:
                wp.props.append(_kv('capture_radius', str(wp_data['capture_radius'])))
            msg.points.append(wp)

        self.get_logger().debug(
            f'Built RouteNetwork mission_id={mission_id} with {len(msg.points)} points'
        )

        return msg

    def publish_origin(self, origin_msg):
        """
        Publishes the shared origin to the '/origin' topic.
        origin_msg: tuple(float, float) or tuple(float, float, float)
        """
        msg = GeoPoint()
        msg.latitude = origin_msg[0]
        msg.longitude = origin_msg[1]
        msg.altitude = origin_msg[2] if len(origin_msg) > 2 else 0.0
        self.origin_pub.publish(msg)
        self.publish_console_log(
            f"Published origin on /origin: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}",
            0,
        )
        self.get_logger().info(f'Publishing from GUI: "{msg}"')

    def publish_path(self, path_msg, vehicle_number):
        """
        Publishes a path message to the appropriate vehicle's map visualization path topic.
        path_msg: list[tuple(float, float)], vehicle_number: int
        """
        msg = Path()
        for point_tuple in path_msg:
            pose_temp = PoseStamped()
            msg.header.frame_id = 'local_xy_origin'
            pose_temp.pose.position.x = point_tuple[0]
            pose_temp.pose.position.y = point_tuple[1]
            msg.poses.append(pose_temp)

        getattr(self, f'coug{vehicle_number}_path_').publish(msg)
        self.get_logger().info(f'Publishing from GUI: "{msg}"')

    def publish_fins(self, fin_degree, vehicle_number, publish_type):
        """
        Publishes fin commands to either kinematics or controls topic for the specified vehicle.
        fin_degree: list of fin angles, vehicle_number: int, publish_type: int (1 for kinematics, 0 for controls)
        """
        msg = UCommand()
        msg.fin = [fin_degree[0], fin_degree[1], fin_degree[2], float(0)]
        if publish_type: getattr(self, f"coug{vehicle_number}_fins_kinematics").publish(msg)
        else: getattr(self, f"coug{vehicle_number}_fins_controls").publish(msg)

    def set_single_parameter(self, param_name, param_value, coug_number, callback=None):
        """
        Sets a single ROS 2 parameter for the specified vehicle using the SetParameters service.
        param_name: str, param_value: str/int/float, coug_number: int, callback: function (optional)
        """
        # Used by tabbed window in an attempt to ros2 param set the fin angles.
        # TODO: Doesn't seem to be working currently. 

        param = RclParameter()
        param.name = param_name
        # Set the appropriate type for the parameter value
        if isinstance(param_value, str):
            param.value.type = ParameterType.PARAMETER_STRING
            param.value.string_value = param_value
        elif isinstance(param_value, int):
            param.value.type = ParameterType.PARAMETER_INTEGER
            param.value.integer_value = param_value
        elif isinstance(param_value, float):
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = param_value

        req = SetParameters.Request()
        req.parameters = [param]
        client = getattr(self, f"coug{coug_number}_kinematics_client")
        future = client.call_async(req)
        if callback:
            future.add_done_callback(lambda fut: callback(fut.result()))
        return future

def ros_spin_thread(executor):
    """
    Spins the ROS 2 executor in a background thread.
    This allows ROS callbacks to be processed while the Qt event loop runs.
    """
    executor.spin()

def get_vehicles_from_params():
    """
    Reads the vehicles_in_mission parameter from the parameter file or environment.
    Returns a list of vehicle numbers to create tabs for.
    """
    # Try to read directly from the parameter file
    param_file_path = os.environ.get(
        "BASE_STATION_PARAM_FILE",
        os.path.expanduser("~/config/base_station_params.yaml")
    )
    param_file_path = os.path.expanduser(param_file_path)
    
    try:
        if os.path.exists(param_file_path):
            with open(param_file_path, 'r') as file:
                params = yaml.safe_load(file)
                # Navigate the YAML structure: /**/ros__parameters/vehicles_in_mission
                if params and '/**' in params:
                    ros_params = params['/**'].get('ros__parameters', {})
                    vehicles_list = ros_params.get('vehicles_in_mission', [1, 2, 3])
                    print(f"GUI: Read vehicles_in_mission from parameter file: {vehicles_list}")
                    return vehicles_list
                else:
                    print(f"GUI: Parameter file structure not found, using default vehicles [1,2,3]")
                    return [1, 2, 3]
        else:
            print(f"GUI: Parameter file not found at {param_file_path}, using default vehicles [1,2,3]")
            return [1, 2, 3]
            
    except Exception as e:
        print(f"GUI: Failed to read parameter file: {e}, using default [1,2,3]")
        return [1, 2, 3]  # Default fallback

def main():
    """     
    Main entry point for the GUI application.
    Initializes ROS 2, starts the Qt application, and spins ROS in a background thread.
    """
    rclpy.init()

    # Get the vehicles list from ROS parameters
    selected_cougs = get_vehicles_from_params()

    # Create the Qt application and main window (window will be set later)
    app, result = base_station_gui.base_station_gui.OpenWindow(None, selected_cougs, borders=False)

    def after_window_ready():
        """
        Callback to initialize the ROS node and executor after the Qt window is ready.
        """
        window = result.get('window')
        if window is None:
            # Try again shortly if the window is not ready
            QTimer.singleShot(50, after_window_ready)
            return

        # Create the ROS 2 node and assign it to the GUI window
        gui_node = GuiNode(window, selected_cougs)
        window.ros_node = gui_node  # used to access the node from the GUI

        # Create a single-threaded executor and add the node
        executor = SingleThreadedExecutor()
        executor.add_node(gui_node)

        # Spin ROS 2 in a background thread so the Qt event loop can run
        ros_thread = threading.Thread(target=ros_spin_thread, args=(executor,), daemon=True)
        ros_thread.start()

        # Ensure Ctrl+C interrupts the Qt event loop
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        # Start a dummy QTimer to keep the Qt event loop alive
        def start_timer():
            timer = QTimer()
            timer.timeout.connect(lambda: None)
            timer.start(100)
        QTimer.singleShot(0, start_timer)

    # Start polling for the window to be ready
    QTimer.singleShot(0, after_window_ready)

    try:
        # Start the Qt event loop
        exit_code = app.exec()
    finally:
        # Ensure ROS 2 is properly shut down when the application exits
        rclpy.shutdown()
        sys.exit(exit_code)

def SeeAllIcons():
    """
    Optional utility function to display all available QStyle.StandardPixmap icons in a grid.
    Useful for GUI development and icon selection.
    """
    import sys
    from PyQt6.QtWidgets import (QApplication, QGridLayout, QPushButton, QStyle, QWidget)
    class Window(QWidget):
        def __init__(self):
            super().__init__()

            # Get all standard pixmap icon names
            icons = sorted([attr for attr in dir(QStyle.StandardPixmap) if attr.startswith("SP_")])
            layout = QGridLayout()

            # Create a button for each icon, displaying the icon and its name
            for n, name in enumerate(icons):
                btn = QPushButton(name)
                pixmapi = getattr(QStyle.StandardPixmap, name)
                icon = self.style().standardIcon(pixmapi)
                btn.setIcon(icon)
                layout.addWidget(btn, int(n/4), int(n%4))
            self.setLayout(layout)
    app = QApplication(sys.argv)
    w = Window()
    w.show()
    app.exec()

if __name__ == '__main__':
    # Entry point for running the GUI application
    main()
