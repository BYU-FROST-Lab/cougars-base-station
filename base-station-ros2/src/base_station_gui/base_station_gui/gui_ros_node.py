import base_station_gui.base_station_gui

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import QTimer

import sys
import threading
import signal

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

import time
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry

from nav_msgs.msg import Path #used to publish the map viz path
from sensor_msgs.msg import NavSatFix, FluidPressure, BatteryState #NavSatFix used to publish the origin
from geometry_msgs.msg import PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped

from base_station_interfaces.srv import BeaconId, Init, LoadMission
from base_station_interfaces.msg import Connections, ConsoleLog
from frost_interfaces.msg import SystemStatus, SystemControl, UCommand
from dvl_msgs.msg import DVLDR, DVL

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

            # Subscribe to dvl/position messages for each vehicle
            sub = self.create_subscription(
                DVLDR,
                f'coug{coug_number}/dvl/position',
                lambda msg, n=coug_number: window.recieve_smoothed_output_message(n, msg),
                10
            )
            setattr(self, f'smoothed_ouput_subscription{coug_number}', sub)

            # Subscribe to smoothed output messages for each vehicle
            sub = self.create_subscription(
                DVL,
                f'coug{coug_number}/dvl/data',
                lambda msg, n=coug_number: window.recieve_dvl_velocity(n, msg),
                10
            )
            setattr(self, f'dvl_vel_subscription{coug_number}', sub)

            # Subscribe to depth data messages for each vehicle
            sub = self.create_subscription(
                PoseWithCovarianceStamped,
                f'coug{coug_number}/depth_data',
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

            # Publisher for system status messages for each vehicle
            pub = self.create_publisher(
                SystemControl,
                f'/coug{coug_number}/system/status',
                1
            )
            setattr(self, f'coug{coug_number}_publisher_', pub)

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


        self.init_client = self.create_client(
            Init,
            f'init_service'
        )

        self.load_mission_client = self.create_client(
            LoadMission,
            f'load_mission_service'
        )

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

        # Subscription for receiving Connections messages from the 'connections' topic
        self.conn_subscription = self.create_subscription(
            Connections,
            'connections',
            window.recieve_connections,  # Calls the GUI's recieve_connections method
            10)  

        # Subscription for console log updates, specific to vehicles. 0 means send to all
        self.console_log_sub = self.create_subscription(
            ConsoleLog,
            'console_log',
            window.handle_console_log,
            10
        ) 

        # Publisher for the map visualization origin 
        #TODO: Fix the origin to be a transient local publisher
        self.origin_pub = self.create_publisher(NavSatFix, '/map_viz_origin', 10)

        # Publisher for console log messages
        self.console_publisher = self.create_publisher(ConsoleLog, 'console_log', 10)

        # Service clients for emergency kill, surface, and modem shut off services
        self.cli = self.create_client(BeaconId, 'e_kill_service')
        self.cli2 = self.create_client(BeaconId, 'e_surface_service')
        # self.cli3 = self.create_client(ModemControl, 'modem_shut_off_service')

    def publish_console_log(self, msg_text, msg_num):
        """
        Publishes a console log message to the 'console_log' topic.
        """
        msg = ConsoleLog()
        msg.message = msg_text
        msg.vehicle_number = msg_num
        self.console_publisher.publish(msg)

    def publish_origin(self, origin_msg):
        """
        Publishes the map visualization origin to the '/map_viz_origin' topic.
        origin_msg: tuple(float, float)
        """
        msg = NavSatFix()
        msg.latitude = origin_msg[0]
        msg.longitude = origin_msg[1]
        msg.header.frame_id = 'local_xy_origin'
        self.origin_pub.publish(msg)
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

        param = Parameter()
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

def main():
    """     
    Main entry point for the GUI application.
    Initializes ROS 2, starts the Qt application, and spins ROS in a background thread.
    """
    rclpy.init()

    # Create the Qt application and main window (window will be set later)
    app, result, selected_cougs = base_station_gui.base_station_gui.OpenWindow(None, borders=False)

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