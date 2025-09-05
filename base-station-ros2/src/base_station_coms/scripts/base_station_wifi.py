#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
from base_station_interfaces.msg import Connections, ConsoleLog
from base_station_interfaces.srv import BeaconId
from frost_interfaces.msg import SystemControl
from base_station_interfaces.srv import Init
from std_msgs.msg import Header
from std_srvs.srv import SetBool
import json
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed

class Base_Station_Wifi(Node):
    def __init__(self):
        super().__init__('base_station_wifi')
        self.get_logger().info("Base Station WiFi Node Initialized")
        self.vehicles_in_mission = self.declare_parameter('vehicles_in_mission', [1,2,3]).value

        # publishes connections messages
        self.wifi_connection_publisher = self.create_publisher(Connections, 'connections', 10)

        # Service to send emergency kill command
        self.e_kill_service = self.create_service(BeaconId, 'wifi_e_kill', self.send_e_kill_callback)

        self.init_service = self.create_service(Init, 'wifi_init', self.init_callback)

        self.console_log = self.create_publisher(ConsoleLog, 'console_log', 10)

        self.init_publishers = {}
        self.thruster_clients = {}
        for vehicle in self.vehicles_in_mission:
            self.init_publishers[vehicle] = self.create_publisher(SystemControl, f'coug{vehicle}/system_status', 10)
            self.thruster_clients[vehicle] = self.create_client(SetBool, f'coug{vehicle}/arm_thruster')

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
            result = subprocess.run([
                "ping", "-c", "1", "-W", "1", ip
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            current_time = self.get_clock().now()
            if result.returncode == 0:
                self.ping_timestamp[vehicle] = current_time
                return vehicle, True
            return vehicle, False
        except Exception as e:
            self.get_logger().warn(f"Exception pinging {ip}: {e}")
            return vehicle, False

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
            self.get_logger().error(f"Error while trying to deactivate thruster: {str(e)}")
            response.success = False
        
        return response

    def init_callback(self, request, response):
        try:
            msg = SystemControl()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'system_status_input'
            msg.start = request.start
            msg.rosbag_flag = request.rosbag_flag
            msg.rosbag_prefix = request.rosbag_prefix
            msg.thruster_arm = request.thruster_arm
            msg.dvl_acoustics = request.dvl_acoustics
            self.init_publishers[request.vehicle_id].publish(msg)
            self.get_logger().info("Published SystemControl message.")
            self.get_logger().info(f"Start: {msg.start.data}, Rosbag Flag: {msg.rosbag_flag.data}, Prefix: {msg.rosbag_prefix}, Thruster: {msg.thruster_arm.data}, DVL: {msg.dvl_acoustics.data}")
            # self.console_log.publish(f"Start: {msg.start.data}, Rosbag Flag: {msg.rosbag_flag.data}, Prefix: {msg.rosbag_prefix}, Thruster: {msg.thruster_arm.data}, DVL: {msg.dvl_acoustics.data}")
            response.success = True
            return response
        except Exception as e:
            self.get_logger().error(f"Error publishing SystemControl message: {e}")
            response.success = False
            return response

    def get_IP_addresses(self):
        """Load IP addresses from config file"""
        config_path = str(Path.home()) + "/base_station/mission_control/deploy_config.json"
        try:
            with open(config_path, "r") as f:
                config = json.load(f)
            vehicles = config["vehicles"]

            for num in self.vehicles_in_mission:
                if str(num) in vehicles:
                    ip = vehicles[str(num)]['remote_host']
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
                futures = {self.thread_executor.submit(self.ping_single_ip, vehicle, ip): vehicle 
                        for vehicle, ip in self.ip_addresses.items()}
                
                # Collect results as they complete
                ping_results = {}
                for future in as_completed(futures, timeout=2.0):  # 2 second timeout for all pings
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
                msg.connections = [final_connections.get(vehicle, False) for vehicle in self.vehicles_in_mission]

                # Calculate time since last successful ping
                current_time = self.get_clock().now()
                msg.last_ping = []
                for vehicle in msg.vehicle_ids:
                    if vehicle in self.ping_timestamp:
                        time_diff = (current_time - self.ping_timestamp[vehicle]).nanoseconds / 1e9
                        msg.last_ping.append(int(time_diff))
                    else:
                        msg.last_ping.append(int((current_time - self.start_time).nanoseconds / 1e9))  # Never pinged successfully

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

if __name__ == '__main__':
    main()