#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
from base_station_interfaces.msg import Connections, ConsoleLog, UCommandBase
from base_station_interfaces.srv import BeaconId, LoadMission
from cougars_interfaces.msg import SystemControl, UCommand
from base_station_interfaces.srv import Init
from std_msgs.msg import Header, Empty
from std_srvs.srv import SetBool
import json
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed
from base_station_gui import deploy


class Base_Station_Wifi(Node):
    def __init__(self):
        super().__init__("base_station_wifi")
        self.get_logger().info("Base Station WiFi Node Initialized")
        self.declare_parameter("vehicles_in_mission", [1, 2, 3])
        self.vehicles_in_mission = self.get_parameter("vehicles_in_mission").value

        # publishes connections messages
        self.wifi_connection_publisher = self.create_publisher(
            Connections, "connections", 10
        )

        # Service to send emergency kill command
        self.e_kill_service = self.create_service(
            BeaconId, "wifi_e_kill", self.send_e_kill_callback
        )

        self.init_service = self.create_service(Init, "wifi_init", self.init_callback)

        self.load_mission_service = self.create_service(
            LoadMission, "wifi_load_mission", self.load_mission_callback
        )

        self.keyboard_controls_publisher = self.create_publisher(
            UCommand, "keyboard_controls", 10
        )

        self.keyboard_controls_subscriber = self.create_subscription(
            UCommandBase, "wifi_keyboard_controls", self.keyboard_controls_callback, 10
        )

        self.console_log = self.create_publisher(ConsoleLog, "console_log", 10)
        self.thruster_enabled = {vehicle: False for vehicle in self.vehicles_in_mission}
        self.init_publishers = {}
        self.thruster_clients = {}
        self.reload_params_publishers = {}
        self.keyboard_controls_publishers = {}
        for vehicle in self.vehicles_in_mission:
            self.reload_params_publishers[vehicle] = self.create_publisher(
                Empty, f"coug{vehicle}/reload_parameters", 10
            )
            self.init_publishers[vehicle] = self.create_publisher(
                SystemControl, f"coug{vehicle}/system/status", 10
            )
            self.keyboard_controls_publishers[vehicle] = self.create_publisher(
                UCommand, f"coug{vehicle}/controls/command", 10
            )
            self.thruster_clients[vehicle] = self.create_client(
                SetBool, f"coug{vehicle}/arm_thruster"
            )

        self.ping_timestamp = {}
        self.ip_addresses = {}
        # Track consecutive missed pings for each vehicle
        self.missed_ping_count = {}
        # Track connection status for each vehicle
        self.connection_status = {}
        self.get_IP_addresses()
        self.start_time = self.get_clock().now()

        # Create thread pool executor - use different name to avoid conflict with ROS2's executor
        self.thread_executor = ThreadPoolExecutor(max_workers=10)
        self.ping_rate_seconds = 2
        self.max_missed_pings = 2
        # timer that calls check connections
        self.create_timer(self.ping_rate_seconds, self.check_connections)

    def ping_single_ip(self, vehicle, ip):
        """Ping a single IP address using system ping command"""
        try:
            # Use system ping command, 1 packet, 1 second timeout
            result = subprocess.run(
                ["ping", "-c", "1", "-W", "1", ip],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            current_time = self.get_clock().now()
            if result.returncode == 0:
                self.ping_timestamp[vehicle] = current_time
                return vehicle, True
            return vehicle, False
        except Exception as e:
            self.get_logger().warn(f"Exception pinging {ip}: {e}")
            return vehicle, False

    def keyboard_controls_callback(self, msg):
        """Callback for keyboard controls messages, republishes to the appropriate vehicle topic"""
        self.get_logger().info(
            f"Received keyboard controls for vehicle {msg.vehicle_id}"
        )
        self.get_logger().info(
            f"Thruster enabled: {msg.thruster_enabled}, Current state: {self.thruster_enabled[msg.vehicle_id]}"
        )

        # Always publish the command first to ensure it gets sent
        command_msg = msg.ucommand
        self.keyboard_controls_publishers[msg.vehicle_id].publish(command_msg)

        # Then handle thruster state change if needed
        if msg.thruster_enabled != self.thruster_enabled[msg.vehicle_id]:
            self.get_logger().info(
                f"Thruster state change detected for vehicle {msg.vehicle_id}: {msg.thruster_enabled}"
            )
            self.thruster_enabled[msg.vehicle_id] = msg.thruster_enabled
            # Call arm_thruster service asynchronously to avoid blocking
            self.send_thruster_command_async(msg.vehicle_id, msg.thruster_enabled)

    def send_thruster_command_async(self, vehicle_id, enable):
        """Send thruster command asynchronously without blocking"""
        thruster_client = self.thruster_clients[vehicle_id]
        service_request = SetBool.Request()
        service_request.data = enable

        if not thruster_client.wait_for_service(timeout_sec=0.1):  # Very short timeout
            self.get_logger().error(
                f"arm_thruster service not available for vehicle {vehicle_id}"
            )
            return

        try:
            # Send async request without blocking
            future = thruster_client.call_async(service_request)

            # Add callback to handle the response
            def handle_thruster_response(future_result):
                try:
                    service_response = future_result.result()
                    if service_response.success:
                        state_str = "enabled" if enable else "disabled"
                        self.get_logger().info(
                            f"Thruster has been {state_str} for vehicle {vehicle_id}."
                        )
                    else:
                        self.get_logger().error(
                            f"Failed to change thruster state for vehicle {vehicle_id}."
                        )
                except Exception as e:
                    self.get_logger().error(
                        f"Error in thruster response callback for vehicle {vehicle_id}: {str(e)}"
                    )

            future.add_done_callback(handle_thruster_response)

        except Exception as e:
            self.get_logger().error(
                f"Error while trying to change thruster state for vehicle {vehicle_id}: {str(e)}"
            )

    def send_e_kill_callback(self, request, response):
        vehicle_id = request.beacon_id

        # Get the correct thruster client for this vehicle
        if vehicle_id not in self.thruster_clients:
            response.success = False
            return response

        thruster_client = self.thruster_clients[vehicle_id]

        service_request = SetBool.Request()
        service_request.data = False

        if not thruster_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("arm_thruster service not available")
            response.success = False
            return response

        try:
            future = thruster_client.call_async(service_request)
            rclpy.spin_until_future_complete(self, future)
            service_response = future.result()

            if service_response.success:
                self.get_logger().info("Thruster has been deactivated.")
                response.success = True
            else:
                self.get_logger().error("Failed to deactivate thruster.")
                response.success = False

        except Exception as e:
            self.get_logger().error(
                f"Error while trying to deactivate thruster: {str(e)}"
            )
            response.success = False

        return response

    def init_callback(self, request, response):
        try:
            msg = SystemControl()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "system_status_input"
            msg.start = request.start
            msg.rosbag_flag = request.rosbag_flag
            msg.rosbag_prefix = request.rosbag_prefix
            msg.thruster_arm = request.thruster_arm
            msg.dvl_acoustics = request.dvl_acoustics
            self.init_publishers[request.vehicle_id].publish(msg)
            self.get_logger().info("Published SystemControl message.")
            self.get_logger().info(
                f"Start: {msg.start.data}, Rosbag Flag: {msg.rosbag_flag.data}, Prefix: {msg.rosbag_prefix}, Thruster: {msg.thruster_arm.data}, DVL: {msg.dvl_acoustics.data}"
            )
            # self.console_log.publish(f"Start: {msg.start.data}, Rosbag Flag: {msg.rosbag_flag.data}, Prefix: {msg.rosbag_prefix}, Thruster: {msg.thruster_arm.data}, DVL: {msg.dvl_acoustics.data}")
            response.success = True
            return response
        except Exception as e:
            self.get_logger().error(f"Error publishing SystemControl message: {e}")
            response.success = False
            return response

    def load_mission_callback(self, request, response):
        # Handle the load mission request
        self.get_logger().info(f"Loading mission for vehicle {request.vehicle_id}")

        try:
            # Call deploy function to send missions to vehicles
            deploy.main(self, [request.vehicle_id], [request.mission_path.data])
            self.console_log.publish(
                ConsoleLog(
                    message="Loading Mission Command Complete",
                    vehicle_number=request.vehicle_id,
                )
            )
            self.reload_params_publishers[request.vehicle_id].publish(Empty())
            response.success = True
        except Exception as e:
            err_msg = f"Mission loading failed: {e}"
            print(err_msg)
            self.console_log.publish(
                ConsoleLog(message=err_msg, vehicle_number=request.vehicle_id)
            )
            response.success = False

        return response

    def get_IP_addresses(self):
        """Load IP addresses from config file"""
        config_path = (
            str(Path.home()) + "/base_station/mission_control/deploy_config.json"
        )
        try:
            with open(config_path, "r") as f:
                config = json.load(f)
            vehicles = config["vehicles"]

            for num in self.vehicles_in_mission:
                if str(num) in vehicles:
                    ip = vehicles[str(num)]["remote_host"]
                    self.ip_addresses[num] = ip
                    # Initialize counters for each vehicle
                    self.missed_ping_count[num] = 0
                    self.connection_status[num] = False
                else:
                    err_msg = f"❌ Vehicle {num} not found in config"
                    self.get_logger().error(err_msg)
        except Exception as e:
            self.get_logger().error(f"Error loading config: {e}")

    def check_connections(self):
        """Check all connections using thread pool"""
        try:
            # Submit all ping tasks to thread pool
            futures = {
                self.thread_executor.submit(self.ping_single_ip, vehicle, ip): vehicle
                for vehicle, ip in self.ip_addresses.items()
            }

            # Collect results as they complete
            ping_results = {}
            for future in as_completed(
                futures, timeout=2.0
            ):  # 2 second timeout for all pings
                vehicle, is_connected = future.result()
                ping_results[vehicle] = is_connected

            # Update connection status based on consecutive missed pings
            final_connections = {}
            for vehicle in self.vehicles_in_mission:
                if vehicle in ping_results:
                    if ping_results[vehicle]:
                        # Ping successful - reset missed count and mark as connected
                        self.missed_ping_count[vehicle] = 0
                        self.connection_status[vehicle] = True
                    else:
                        # Ping failed - increment missed count
                        self.missed_ping_count[vehicle] += 1

                        # Only mark as disconnected after 2 consecutive missed pings
                        if self.missed_ping_count[vehicle] >= self.max_missed_pings:
                            self.connection_status[vehicle] = False
                        # Otherwise keep previous connection status
                else:
                    # No ping result - treat as failed ping
                    self.missed_ping_count[vehicle] += 1
                    if self.missed_ping_count[vehicle] >= self.max_missed_pings:
                        self.connection_status[vehicle] = False

                final_connections[vehicle] = self.connection_status[vehicle]

            # Create and publish the message
            msg = Connections()
            msg.connection_type = 2  # WiFi connections
            msg.vehicle_ids = self.vehicles_in_mission
            msg.connections = [
                self.connection_status[vehicle] for vehicle in self.vehicles_in_mission
            ]  #

            # Calculate time since last successful ping
            current_time = self.get_clock().now()
            msg.last_ping = []
            for vehicle in msg.vehicle_ids:
                if vehicle in self.ping_timestamp:
                    time_diff = (
                        current_time - self.ping_timestamp[vehicle]
                    ).nanoseconds / 1e9
                    msg.last_ping.append(int(time_diff))
                else:
                    msg.last_ping.append(
                        int((current_time - self.start_time).nanoseconds / 1e9)
                    )  # Never pinged successfully

            self.wifi_connection_publisher.publish(msg)

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


if __name__ == "__main__":
    main()
