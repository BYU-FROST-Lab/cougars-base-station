#!/usr/bin/env python3

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.node import Node
import subprocess
from base_station_interfaces.msg import Connections, ConsoleLog, UCommandBase
from cougars_interfaces.msg import ActuatorCommand, SystemControl
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geographic_msgs.msg import GeoPoint, RouteNetwork
from std_msgs.msg import Header, Empty, Bool, String
from std_srvs.srv import SetBool
import json
import time
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, TimeoutError, as_completed
from base_station_gui import deploy


def diagnostic_level_value(level):
    if isinstance(level, int):
        return level
    if isinstance(level, (bytes, bytearray)):
        return level[0] if level else 0
    if isinstance(level, str):
        return ord(level[0]) if level else 0
    return int(level)


class VehicleWifiConnection:
    def __init__(self, vehicle_id, ip_address, node, max_missed_pings):
        self.vehicle_id = vehicle_id
        self.ip_address = ip_address
        self.node = node
        self.max_missed_pings = max_missed_pings
        self.missed_ping_count = 0
        self.connection_status = False
        self.thruster_enabled = False
        self.modem_connection = False
        self.radio_connection = False
        self.start_time = time.monotonic()
        self.last_ping_time = None

        self.mission_publisher = node.create_publisher(
            RouteNetwork,
            f'coug{vehicle_id}/mission',
            10
        )

        self.reload_params_publisher = node.create_publisher(
            Empty,
            f'coug{vehicle_id}/reload_parameters',
            10
        )
        self.init_publisher = node.create_publisher(
            SystemControl,
            f'coug{vehicle_id}/system/control',
            10
        )
        # coug_kinematics.cpp is the node that actually drives the fins/thruster; it
        # listens on control/u_cmd for ActuatorCommand messages.
        self.keyboard_controls_publisher = node.create_publisher(
            ActuatorCommand,
            f'coug{vehicle_id}/control/u_cmd',
            10
        )
        self.thruster_client = node.create_client(
            SetBool,
            f'coug{vehicle_id}/arm_thruster'
        )
        self.surface_client = node.create_client(
            SetBool,
            f'coug{vehicle_id}/surface'
        )
        self.link_status_publisher = node.create_publisher(
            DiagnosticStatus,
            f'coug{vehicle_id}/link_status',
            10
        )

        # Forwards the GUI's hardware control "intent" topic to the vehicle's actual
        # ROS topic (cougars_coms listens on the namespaced 'hardware_control_cmd').
        self.hardware_control_publisher = node.create_publisher(
            String,
            f'coug{vehicle_id}/hardware_control_cmd',
            10
        )

        # Vehicle /origin subscribers (gps_odom, waypoint_controller, emergency_protocols)
        # are TRANSIENT_LOCAL; a VOLATILE publisher is incompatible and never delivers.
        origin_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.origin_publisher = node.create_publisher(
            GeoPoint,
            f'/origin',
            origin_qos
        )

        self.link_status_subscriber = node.create_subscription(
            DiagnosticStatus,
            f'coug{vehicle_id}/link_status',
            self.link_status_callback,
            10
        )

        self.load_mission_subscriber = node.create_subscription(
            RouteNetwork,
            f'coug{vehicle_id}/load_mission',
            self.load_mission_callback,
            10
        )

        self.start_mission_subscriber = node.create_subscription(
            SystemControl,
            f'coug{vehicle_id}/start_mission',
            self.start_mission_callback,
            10
        )

        self.origin_subscriber = node.create_subscription(
            GeoPoint,
            f'/send_origin',
            self.origin_callback,
            origin_qos
        )

        self.emergency_kill_subscriber = node.create_subscription(
            Bool,
            f'coug{vehicle_id}/emergency_kill',
            self.emergency_kill_callback,
            10
        )

        self.emergency_surface_subscriber = node.create_subscription(
            Bool,
            f'coug{vehicle_id}/emergency_surface',
            self.emergency_surface_callback,
            10
        )

        self.hardware_control_subscriber = node.create_subscription(
            String,
            f'coug{vehicle_id}/hardware_control',
            self.hardware_control_callback,
            10
        )

    def link_status_callback(self, msg):
        connected = diagnostic_level_value(msg.level) == DiagnosticStatus.OK
        if msg.hardware_id == "wifi":
            return
        elif msg.hardware_id == "radio":
            self.radio_connection = connected
        elif msg.hardware_id == "modem":
            self.modem_connection = connected

    def load_mission_callback(self, msg):
        if self.node.is_wifi_enabled() and self.connection_status:
            self.node.get_logger().info(f"Loading mission for vehicle {self.vehicle_id} over WiFi")
            self.mission_publisher.publish(msg)

    def start_mission_callback(self, msg):
        if self.node.is_wifi_enabled() and self.connection_status:
            self.node.get_logger().info(f"Starting mission for vehicle {self.vehicle_id} over WiFi")
            self.init_publisher.publish(msg)

    def origin_callback(self, msg):
        self.node.get_logger().info(f"Received origin for vehicle {self.vehicle_id}: "
                                    f"lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}")
        self.origin_publisher.publish(msg)

    def emergency_kill_callback(self, msg):
        self.node.get_logger().info(
            f"Received emergency kill for vehicle {self.vehicle_id}: "
            f"data={msg.data}, wifi_enabled={self.node.is_wifi_enabled()}, wifi_connected={self.connection_status}"
        )
        if not msg.data:
            return
        if not self.node.is_wifi_enabled() or not self.connection_status:
            self.node.get_logger().warn(
                f"Not sending emergency kill for vehicle {self.vehicle_id} over WiFi because WiFi is disabled or disconnected"
            )
            return
        self.node.get_logger().info(f"Emergency kill for vehicle {self.vehicle_id} over WiFi")
        self.send_e_kill()

    def emergency_surface_callback(self, msg):
        self.node.get_logger().info(
            f"Received emergency surface for vehicle {self.vehicle_id}: "
            f"data={msg.data}, wifi_enabled={self.node.is_wifi_enabled()}, wifi_connected={self.connection_status}"
        )
        if msg.data and self.node.is_wifi_enabled() and self.connection_status:
            self.node.get_logger().info(f"Emergency surface for vehicle {self.vehicle_id} over WiFi")
            self.send_e_surface()
        elif msg.data:
            self.node.get_logger().warn(
                f"Not sending emergency surface for vehicle {self.vehicle_id} over WiFi because WiFi is disabled or disconnected"
            )

    def hardware_control_callback(self, msg):
        if not self.node.is_wifi_enabled() or not self.connection_status:
            self.node.get_logger().warn(
                f"Not sending hardware control for vehicle {self.vehicle_id} over WiFi because WiFi is disabled or disconnected"
            )
            return
        self.node.get_logger().info(
            f"Sending hardware control '{msg.data}' to vehicle {self.vehicle_id} over WiFi"
        )
        self.hardware_control_publisher.publish(msg)
        self.node.console_log.publish(ConsoleLog(
            message=f"Sent hardware control {msg.data} over WiFi",
            vehicle_number=self.vehicle_id))

    def ping(self):
        if not self.node.is_wifi_enabled():
            return False
        try:
            result = subprocess.run(
                ["ping", "-c", "1", "-W", "1", self.ip_address],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            return result.returncode == 0
        except Exception as e:
            self.node.get_logger().warn(f"Exception pinging {self.ip_address}: {e}")
            return False

    def update_connection(self, is_connected):
        previous_status = self.connection_status
        if not self.node.is_wifi_enabled():
            self.missed_ping_count = self.max_missed_pings
            self.connection_status = False
        elif is_connected:
            self.last_ping_time = time.monotonic()
            self.missed_ping_count = 0
            self.connection_status = True
        else:
            self.missed_ping_count += 1
            self.node.get_logger().debug(
                f"WiFi ping failed for vehicle {self.vehicle_id} at {self.ip_address} "
                f"(missed_pings={self.missed_ping_count}/{self.max_missed_pings})"
            )
            if self.missed_ping_count >= self.max_missed_pings:
                self.connection_status = False

        if self.connection_status != previous_status:
            state = "connected" if self.connection_status else "disconnected"
            self.node.get_logger().info(
                f"WiFi vehicle {self.vehicle_id} marked {state} "
                f"(last_ping_success={is_connected}, missed_pings={self.missed_ping_count})"
            )

        self.publish_link_status()

    def seconds_since_ping(self):
        current_time = time.monotonic()
        if self.last_ping_time is None:
            return int(current_time - self.start_time)
        return int(current_time - self.last_ping_time)

    def publish_keyboard_controls(self, msg):
        if not self.node.is_wifi_enabled() or not self.connection_status:
            self.node.get_logger().debug(
                f"Not sending keyboard controls for vehicle {self.vehicle_id} over WiFi because WiFi is disabled or disconnected"
            )
            return
        actuator_msg = ActuatorCommand()
        actuator_msg.header = msg.ucommand.header
        actuator_msg.fin = msg.ucommand.fin
        actuator_msg.thruster = msg.ucommand.thruster
        self.keyboard_controls_publisher.publish(actuator_msg)

        if msg.thruster_enabled != self.thruster_enabled:
            self.node.get_logger().info(
                f"Thruster state change detected for vehicle {self.vehicle_id}: {msg.thruster_enabled}"
            )
            self.thruster_enabled = msg.thruster_enabled
            self.send_thruster_command_async(msg.thruster_enabled)

    def send_thruster_command_async(self, enable):
        service_request = SetBool.Request()
        service_request.data = enable

        if not self.thruster_client.wait_for_service(timeout_sec=0.1):
            self.node.get_logger().error(f"arm_thruster service not available for vehicle {self.vehicle_id}")
            return

        try:
            future = self.thruster_client.call_async(service_request)

            def handle_thruster_response(future_result):
                try:
                    service_response = future_result.result()
                    if service_response.success:
                        state_str = "enabled" if enable else "disabled"
                        self.node.get_logger().info(
                            f"Thruster has been {state_str} for vehicle {self.vehicle_id}."
                        )
                    else:
                        self.node.get_logger().error(
                            f"Failed to change thruster state for vehicle {self.vehicle_id}."
                        )
                except Exception as e:
                    self.node.get_logger().error(
                        f"Error in thruster response callback for vehicle {self.vehicle_id}: {str(e)}"
                    )

            future.add_done_callback(handle_thruster_response)
        except Exception as e:
            self.node.get_logger().error(
                f"Error while trying to change thruster state for vehicle {self.vehicle_id}: {str(e)}"
            )

    def send_e_kill(self):
        service_request = SetBool.Request()
        service_request.data = False

        if not self.thruster_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error("arm_thruster service not available")
            return False

        try:
            future = self.thruster_client.call_async(service_request)
            rclpy.spin_until_future_complete(self.node, future)
            service_response = future.result()

            if service_response.success:
                self.node.get_logger().info("Thruster has been deactivated.")
                return True

            self.node.get_logger().error("Failed to deactivate thruster.")
            return False
        except Exception as e:
            self.node.get_logger().error(f"Error while trying to deactivate thruster: {str(e)}")
            return False

    def send_e_surface(self):
        service_request = SetBool.Request()
        service_request.data = True

        if not self.surface_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error(f"surface service not available for vehicle {self.vehicle_id}")
            return False

        try:
            future = self.surface_client.call_async(service_request)
            rclpy.spin_until_future_complete(self.node, future)
            service_response = future.result()

            if service_response.success:
                self.node.get_logger().info(f"Surface override enabled for vehicle {self.vehicle_id}.")
                return True

            self.node.get_logger().error(
                f"Failed to enable surface override for vehicle {self.vehicle_id}: {service_response.message}"
            )
            return False
        except Exception as e:
            self.node.get_logger().error(
                f"Error while trying to send surface command for vehicle {self.vehicle_id}: {str(e)}"
            )
            return False

    def publish_init(self, request):
        msg = SystemControl()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'system_status_input'
        msg.start = request.start
        msg.rosbag_flag = request.rosbag_flag
        msg.rosbag_prefix = request.rosbag_prefix
        msg.thruster_arm = request.thruster_arm
        msg.dvl_acoustics = request.dvl_acoustics
        self.init_publisher.publish(msg)
        return msg

    def reload_params(self):
        self.reload_params_publisher.publish(Empty())

    def publish_link_status(self):
        status_msg = DiagnosticStatus()
        status_msg.name = f"Coug{self.vehicle_id} WiFi Connection"
        status_msg.hardware_id = "wifi"
        status_msg.level = DiagnosticStatus.OK if self.connection_status else DiagnosticStatus.ERROR
        status_msg.message = "Connected" if self.connection_status else "Disconnected"
        status_msg.values.append(KeyValue(key="last_ping_seconds", value=str(self.seconds_since_ping())))
        self.link_status_publisher.publish(status_msg)


class Base_Station_Wifi(Node):
    def __init__(self):
        super().__init__('base_station_wifi')
        self.get_logger().info("Base Station WiFi Node Initialized")

        self.declare_parameter('vehicles_in_mission', [1,2,3])

        
        
        self.vehicles_in_mission = self.get_parameter('vehicles_in_mission').value
        # publishes connections messages
        self.wifi_connection_publisher = self.create_publisher(Connections, 'connections', 10)

        self.keyboard_controls_subscriber = self.create_subscription(
            UCommandBase,
            'keyboard_controls',
            self.keyboard_controls_callback,
            10
        )

        self.console_log = self.create_publisher(ConsoleLog, 'console_log', 10)

 
        self.ip_addresses = {}
        self.ping_rate_seconds = 2
        self.max_missed_pings = 2
        self.wifi_enabled = self.declare_parameter('wifi_enabled', True).value
        self.disable_auto_link_status = self.declare_parameter('disable_auto_link_status', False).value
        self.vehicle_wifis = {}
        self.get_IP_addresses()
        self.add_on_set_parameters_callback(self.on_parameter_update)

        # Create thread pool executor - use different name to avoid conflict with ROS2's executor
        self.thread_executor = ThreadPoolExecutor(max_workers=10)
        if not self.wifi_enabled:
            self.get_logger().warn("WiFi is disabled by parameter; publishing disconnected WiFi status")
            self.force_wifi_disconnected()
        # timer that calls check connections
        if self.disable_auto_link_status:
            self.get_logger().warn("Automatic WiFi link-status timer disabled")
        else:
            self.create_timer(self.ping_rate_seconds, self.check_connections)

    def is_wifi_enabled(self):
        return bool(self.wifi_enabled)

    def on_parameter_update(self, params):
        for param in params:
            if param.name == 'wifi_enabled':
                self.wifi_enabled = bool(param.value)
                if self.wifi_enabled:
                    self.get_logger().info("WiFi enabled by parameter")
                else:
                    self.get_logger().warn("WiFi disabled by parameter; forcing all WiFi links disconnected")
                    self.force_wifi_disconnected()
        return SetParametersResult(successful=True)

    def force_wifi_disconnected(self):
        for vehicle_wifi in self.vehicle_wifis.values():
            vehicle_wifi.connection_status = False
            vehicle_wifi.missed_ping_count = vehicle_wifi.max_missed_pings
            vehicle_wifi.publish_link_status()
        self.publish_connections()

    def publish_connections(self):
        msg = Connections()
        msg.connection_type = 2
        msg.vehicle_ids = self.vehicles_in_mission
        msg.connections = [
            self.vehicle_wifis[vehicle].connection_status if vehicle in self.vehicle_wifis else False
            for vehicle in self.vehicles_in_mission
        ]
        msg.last_ping = [
            self.vehicle_wifis[vehicle].seconds_since_ping() if vehicle in self.vehicle_wifis else 0
            for vehicle in self.vehicles_in_mission
        ]
        self.wifi_connection_publisher.publish(msg)

    def keyboard_controls_callback(self, msg):
        """Callback for keyboard controls messages, republishes to the appropriate vehicle topic"""
        self.get_logger().debug(f"Received keyboard controls for vehicle {msg.vehicle_id}")
        if not self.is_wifi_enabled():
            self.get_logger().warn("Ignoring keyboard controls because WiFi is disabled")
            return
        vehicle_wifi = self.vehicle_wifis.get(msg.vehicle_id)
        if vehicle_wifi is None:
            self.get_logger().warn(f"Received keyboard controls for unknown vehicle ID {msg.vehicle_id}. Ignoring.")
            return
        self.get_logger().debug(
            f"Thruster enabled: {msg.thruster_enabled}, Current state: {vehicle_wifi.thruster_enabled}"
        )
        vehicle_wifi.publish_keyboard_controls(msg)


    def get_IP_addresses(self):
        """Load IP addresses from config file"""
        config_path = Path.home().joinpath("config", "cougars-config", "base_station", "deploy_config.json")
        try:
            with open(config_path, "r") as f:
                config = json.load(f)
            vehicles = config["vehicles"]

            for num in self.vehicles_in_mission:
                vehicle_info = vehicles.get(f"coug{num}") or vehicles.get(str(num))
                if vehicle_info:
                    ip = vehicle_info['remote_host']
                    self.ip_addresses[num] = ip
                    self.vehicle_wifis[num] = VehicleWifiConnection(
                        num,
                        ip,
                        self,
                        self.max_missed_pings
                    )
                else:
                    err_msg = f"❌ Vehicle {num} not found in config"
                    if self.disable_auto_link_status:
                        self.get_logger().warn(f"{err_msg}; using 127.0.0.1 for logic test")
                        self.create_test_wifi_connection(num)
                    else:
                        self.get_logger().error(err_msg)
        except Exception as e:
            if self.disable_auto_link_status:
                self.get_logger().warn(f"Error loading config: {e}; using test WiFi connections")
                for num in self.vehicles_in_mission:
                    self.create_test_wifi_connection(num)
            else:
                self.get_logger().error(f"Error loading config: {e}")



    def create_test_wifi_connection(self, vehicle_id):
        self.ip_addresses[vehicle_id] = '127.0.0.1'
        self.vehicle_wifis[vehicle_id] = VehicleWifiConnection(
            vehicle_id,
            '127.0.0.1',
            self,
            self.max_missed_pings
        )

    def check_connections(self):
            """Check all connections using thread pool"""
            try:
                if not self.is_wifi_enabled():
                    self.force_wifi_disconnected()
                    return

                # Submit all ping tasks to thread pool
                futures = {
                    self.thread_executor.submit(vehicle_wifi.ping): vehicle
                    for vehicle, vehicle_wifi in self.vehicle_wifis.items()
                }
                
                # Collect results as they complete
                ping_results = {}
                try:
                    for future in as_completed(futures, timeout=2.0):  # 2 second timeout for all pings
                        vehicle = futures[future]
                        ping_results[vehicle] = future.result()
                except TimeoutError:
                    self.get_logger().warn("Timed out waiting for one or more WiFi ping results")

                # Update connection status based on consecutive missed pings
                for vehicle in self.vehicles_in_mission:
                    vehicle_wifi = self.vehicle_wifis.get(vehicle)
                    if vehicle_wifi is not None:
                        vehicle_wifi.update_connection(ping_results.get(vehicle, False))

                self.publish_connections()

            except Exception as e:
                self.get_logger().error(f"Exception in check_connections: {e}")

    def destroy_node(self):
        """Clean up thread pool when node is destroyed"""
        self.thread_executor.shutdown(wait=True)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Base_Station_Wifi()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
