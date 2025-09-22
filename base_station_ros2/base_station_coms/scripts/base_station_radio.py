#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from base_station_interfaces.msg import Connections, ConsoleLog, Status, UCommandRadio
from base_station_interfaces.srv import BeaconId, Init, LoadMission
from std_msgs.msg import String
from std_msgs.msg import Int8
import time


from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice, XBee64BitAddress
from digi.xbee.exception import TransmitException

import json
import traceback
import base64
from pathlib import Path

class RFBridge(Node):
    def __init__(self):
        super().__init__('base_station_rf_bridge')

        # Debug mode
        self.debug_mode = self.declare_parameter('debug_mode', False).value
        if self.debug_mode:
            self.get_logger().info("Debug mode enabled: Will log detailed packet information")

        # QoS profiles
        self.odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)
        self.dvl_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5)

        # list of vehicles in the mission
        self.declare_parameter('vehicles_in_mission', [1,2,3])
        self.vehicles_in_mission = self.get_parameter('vehicles_in_mission').get_parameter_value().integer_array_value

        # Frequency of PING messages keeping track of radio connections
        self.declare_parameter('ping_frequency', 2)
        self.ping_frequency = self.get_parameter('ping_frequency').get_parameter_value().integer_value

        # base station vehicle ID
        self.declare_parameter('vehicle_id', 15)
        self.vehicle_id = self.get_parameter('vehicle_id').get_parameter_value().integer_value

        #XBee configuration
        self.xbee_port = self.declare_parameter('xbee_port', '/dev/frost/xbee_radio').value
        self.xbee_baud = self.declare_parameter('xbee_baud', 9600).value
        self.device = XBeeDevice(self.xbee_port, self.xbee_baud)

        # ROS publishers and subscribers
        self.publisher = self.create_publisher(String, 'rf_received', 10)
        self.init_publisher = self.create_publisher(String, 'init', 10)

        # publishes status messages
        self.status_publisher = self.create_publisher(Status, 'status', 10)

        # publishes connections messages
        self.rf_connection_publisher = self.create_publisher(Connections, 'connections', 10)

        # publishes console log messages to GUI
        self.print_to_gui_publisher = self.create_publisher(ConsoleLog, 'console_log', 10)

        # Service to send emergency kill command
        self.e_kill_service = self.create_service(BeaconId, 'radio_e_kill', self.send_e_kill_callback)

        # Service to request status from a specific vehicle
        self.status_service = self.create_service(BeaconId, 'radio_status_request', self.request_status_callback)
        
        self.load_mission_service = self.create_service(LoadMission, 'radio_load_mission', self.load_mission_callback)

        self.init_service = self.create_service(Init, 'radio_init', self.init_callback)

        self.subscription = self.create_subscription(
            String,
            'rf_transmit',
            self.tx_callback,
            10)
        
        self.key_controls_subscription = self.create_subscription(
            UCommandRadio,
            'radio_key_command',
            self.key_controls_callback,
            10)
        
    
        self.timer = self.create_timer(self.ping_frequency, self.check_connections)
                    # Thread-safe shutdown flag
        self.running = True
        self.ping_timestamp = {}
        self.connections = {}
        self.radio_addresses = {}
        self.max_msgs_missed = 3  # Number of missed messages before considering a vehicle disconnected
        for vehicle in self.vehicles_in_mission:
            self.connections[vehicle] = False
            self.ping_timestamp[vehicle] = self.get_clock().now().nanoseconds / 1e9  # Initialize with current time in seconds

        self.local_fleet_params_path = Path(f"/home/frostlab/base_station/mission_control/params/fleet_params.yaml")
        self.local_vehicle_params_path = Path(f"/home/frostlab/base_station/mission_control/params/")


        try:
            self.device.open()
            self.get_logger().info(f"Opened XBee device on {self.xbee_port} at {self.xbee_baud} baud.")
                    # Register XBee data receive callback
            self.device.add_data_received_callback(self.data_receive_callback)
            self.get_logger().info("RF Bridge node started using digi-xbee library.")
        except Exception as e:
            self.get_logger().error(f"Failed to open XBee device: {e}")




    #transmit function
    def tx_callback(self, msg):
        try:
            message = msg.data
            self.device.send_data_broadcast(message)
            self.get_logger().debug(f"Sent via XBee: {message}")
        except Exception as e:
            self.get_logger().error(f"XBee transmission error: {str(e)}")
            self.get_logger().error(traceback.format_exc())

    # Function to send a message to a specific address
    def send_message(self, msg, address):
        try:
            remote_device = RemoteXBeeDevice(self.device, address)
            self.device.send_data(remote_device, msg)
            self.get_logger().debug(f"Sent via XBee: {msg}")
            return True
        except TransmitException as e:
            self.get_logger().error(f"XBee transmission error - TransmitException: {e}")
            self.get_logger().error(traceback.format_exc())
            return False
        except Exception as e:
            self.get_logger().error(f"XBee transmission error - Exception: {str(e)}")
            self.get_logger().error(traceback.format_exc())
            return False

    # Callback for receiving data from XBee
    def data_receive_callback(self, xbee_message):
        try:
            payload = xbee_message.data.decode('utf-8', errors='replace')
            sender_address = xbee_message.remote_device.get_64bit_addr()
            self.get_logger().debug(f"Received from {sender_address}: {payload}")

            try:
                data = json.loads(payload)
            except json.JSONDecodeError:
                data = payload  # If JSON decoding fails, treat payload as a string

            if isinstance(data, dict):
                message_type = data.get("message")
            else:
                message_type = data

            if message_type == "STATUS":
                self.recieve_status(data)
            elif message_type == "E_KILL":
                self.confirm_e_kill(data)
            elif message_type == "PING":
                self.recieve_ping(data.get("src_id"), sender_address)
            elif message_type == "INIT":
                self.print_to_gui_publisher.publish(ConsoleLog(message="Start mission command was successful", vehicle_number=data.get("src_id")))
            elif message_type == "FILE_ACK":
                # Handle file transfer acknowledgments
                self.get_logger().debug(f"Received file transfer ACK from vehicle {data.get('src_id')}: {data.get('status', 'unknown')}")
                if data.get("status") == "complete":
                    # self.print_to_gui_publisher.publish(
                    #     ConsoleLog(
                    #         message=f"File transfer completed successfully on vehicle {data.get('src_id')}",
                    #         vehicle_number=data.get('src_id', 0)
                    #     )
                    # )
                    pass
                elif data.get("status") == "error":
                    self.print_to_gui_publisher.publish(
                        ConsoleLog(
                            message=f"File transfer failed on vehicle {data.get('src_id')}: {data.get('error', 'unknown error')}",
                            vehicle_number=data.get('src_id', 0)
                        )
                    )
            else:
                self.get_logger().warn(f"Unknown message type: {message_type}")
        except Exception as e:
            self.get_logger().error(f"Error in data_receive_callback: {e}")



    # Function to check connections and send PING messages
    def check_connections(self):
        msg = Connections()
        msg.connection_type = 1
        self.get_logger().debug(f"Sending PING")
        ping = "PING"

        if len(self.radio_addresses) < len(self.vehicles_in_mission):
            try:
                self.device.send_data_broadcast(ping)
            except Exception as e:
                self.get_logger().debug(f"Failed to send broadcast PING: {e}")
        else:
            for vehicle in self.radio_addresses:
                self.send_message(ping, self.radio_addresses[vehicle])

        for vehicle in self.vehicles_in_mission:
            last_ping = int(time.time() - self.ping_timestamp[vehicle])
            if (last_ping >= self.ping_frequency*self.max_msgs_missed):
                self.connections[vehicle] = False 
            msg.connections.append(self.connections[vehicle])
            msg.last_ping.append(last_ping)
        msg.vehicle_ids = list(self.vehicles_in_mission)
        self.rf_connection_publisher.publish(msg)

        if self.debug_mode:
            self.get_logger().debug(f"Connections: {self.connections}")

    # Callback for status requests
    def request_status_callback(self, request, response):

        try:
            target_vehicle_id = request.beacon_id
            if target_vehicle_id is None:
                self.get_logger().error("Status request missing target vehicle ID.")
                response.success = False
                return response

            self.get_logger().debug(f"Requesting for Coug {target_vehicle_id}")

            status_request_msg = "STATUS"

            response.success = self.send_message(status_request_msg, self.radio_addresses.get(target_vehicle_id, None))
        except Exception as e:
            self.get_logger().error(f"Error processing status request: {e}")
            response.success = False

        return response
    
    def load_mission_callback(self, request, response):
        try:
            target_vehicle_id = request.vehicle_id
            mission_file_path = request.mission_path.data
            vehicle_params_path = self.local_vehicle_params_path / f"coug{target_vehicle_id}_params.yaml"

            if target_vehicle_id is None:
                self.get_logger().error("Load mission request missing target vehicle ID.")
                response.success = False
                return response
            
            if target_vehicle_id not in self.radio_addresses:
                self.get_logger().error(f"No radio address found for vehicle {target_vehicle_id}")
                response.success = False
                return response

            self.get_logger().info(f"Loading mission file {mission_file_path} to Coug {target_vehicle_id} via radio")
            
            # Check if file exists
            if not Path(mission_file_path).exists():
                self.get_logger().error(f"Mission file not found: {mission_file_path}")
                response.success = False
                return response

            # Send the mission file
            success0 = self.send_file_over_radio(mission_file_path, target_vehicle_id, "mission.yaml")
            mission_status = "was successful" if success0 else "was not successful"
            self.print_to_gui_publisher.publish(
                    ConsoleLog(
                        message=f"Mission file sent to Coug {target_vehicle_id} via radio {mission_status}",
                        vehicle_number=target_vehicle_id
                    )
                )
            success1 = self.send_file_over_radio(vehicle_params_path, target_vehicle_id, f"coug{target_vehicle_id}_params.yaml")
            params_status = "was successful" if success1 else "was not successful"
            self.print_to_gui_publisher.publish(
                    ConsoleLog(
                        message=f"Vehicle params file sent to Coug {target_vehicle_id} via radio {params_status}",
                        vehicle_number=target_vehicle_id
                    )
                )
            success2 = self.send_file_over_radio(self.local_fleet_params_path, target_vehicle_id, f"fleet_params.yaml")
            fleet_status = "was successful" if success2 else "was not successful"
            self.print_to_gui_publisher.publish(
                    ConsoleLog(
                        message=f"Fleet params file sent to Coug {target_vehicle_id} via radio {fleet_status}",
                        vehicle_number=target_vehicle_id
                    )
                )

            if success0 and success1 and success2:
                self.print_to_gui_publisher.publish(
                    ConsoleLog(
                        message=f"Files successfully sent to Coug {target_vehicle_id} via radio",
                        vehicle_number=target_vehicle_id
                    )
                )
                response.success = True
            else:
                self.print_to_gui_publisher.publish(
                    ConsoleLog(
                        message=f"One or more files failed to send to Coug {target_vehicle_id} via radio",
                        vehicle_number=target_vehicle_id
                    )
                )
                response.success = False

        except Exception as e:
            self.get_logger().error(f"Error loading mission through radio: {e}")
            self.print_to_gui_publisher.publish(
                ConsoleLog(
                    message=f"Error loading mission: {str(e)}",
                    vehicle_number=request.vehicle_id if hasattr(request, 'vehicle_id') else 0
                )
            )
            response.success = False

        return response

    def send_file_over_radio(self, file_path, target_vehicle_id, remote_filename):
        """
        Send a file over XBee radio by chunking it into smaller packets.
        
        Args:
            file_path (str): Path to the file to send
            target_vehicle_id (int): Target vehicle ID
            remote_filename (str): Name to save the file as on the remote end
            
        Returns:
            bool: True if file sent successfully, False otherwise
        """
        try:
            # XBee has a maximum payload size, typically around 100 bytes for reliable transmission
            # We'll use smaller chunks to account for JSON overhead and ensure reliability
            CHUNK_SIZE = 64
            
            # Read the file
            with open(file_path, 'rb') as f:
                file_data = f.read()
            
            # Calculate total chunks
            total_chunks = (len(file_data) + CHUNK_SIZE - 1) // CHUNK_SIZE
            file_size = len(file_data)

            self.print_to_gui_publisher.publish(ConsoleLog(message=f"Sending file {file_path} ({file_size} bytes) in {total_chunks} chunks to vehicle {target_vehicle_id}", vehicle_number=target_vehicle_id))

            # Send file start message
            start_msg = {
                "message": "FILE_START",
                "filename": remote_filename,
                "total_chunks": total_chunks,
                "file_size": file_size,
                "transfer_id": int(time.time())  # Use timestamp as transfer ID
            }
            
            if not self.send_message(json.dumps(start_msg), self.radio_addresses[target_vehicle_id]):
                self.get_logger().error("Failed to send FILE_START message")
                return False
            
            # Wait a bit for the receiver to prepare
            time.sleep(0.1)
            
            # Send file chunks
            for chunk_num in range(total_chunks):
                start_idx = chunk_num * CHUNK_SIZE
                end_idx = min(start_idx + CHUNK_SIZE, len(file_data))
                chunk_data = file_data[start_idx:end_idx]
                
                # Encode chunk data as base64 for JSON transmission
                import base64
                chunk_b64 = base64.b64encode(chunk_data).decode('ascii')
                
                chunk_msg = {
                    "message": "FILE_CHUNK",
                    "transfer_id": start_msg["transfer_id"],
                    "chunk_num": chunk_num,
                    "total_chunks": total_chunks,
                    "data": chunk_b64
                }
                
                # Send chunk with retry logic
                retry_count = 0
                max_retries = 3
                while retry_count < max_retries:
                    if self.send_message(json.dumps(chunk_msg), self.radio_addresses[target_vehicle_id]):
                        self.get_logger().debug(f"Sent chunk {chunk_num + 1}/{total_chunks}")
                        break
                    else:
                        retry_count += 1
                        self.get_logger().warn(f"Failed to send chunk {chunk_num + 1}/{total_chunks}, retry {retry_count}/{max_retries}")
                        time.sleep(0.1)
                
                if retry_count >= max_retries:
                    self.get_logger().error(f"Failed to send chunk {chunk_num + 1}/{total_chunks} after {max_retries} retries")
                    return False

                self.print_to_gui_publisher.publish(ConsoleLog(message=f"Sent chunk {chunk_num + 1}/{total_chunks}", vehicle_number=target_vehicle_id))

                # Small delay between chunks to avoid overwhelming the receiver
                time.sleep(0.05)
            
            # Send file end message
            end_msg = {
                "message": "FILE_END",
                "transfer_id": start_msg["transfer_id"],
                "filename": remote_filename,
                "total_chunks": total_chunks
            }
            
            if not self.send_message(json.dumps(end_msg), self.radio_addresses[target_vehicle_id]):
                self.get_logger().error("Failed to send FILE_END message")
                return False
            
            self.get_logger().info(f"Successfully sent file {remote_filename} to vehicle {target_vehicle_id}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error sending file over radio: {e}")
            return False


    # Helper function to create Int8 message
    def make_int8(self, val):
        msg = Int8()
        msg.data = val
        return msg

    # Function to handle received status messages
    def recieve_status(self, data):
        self.get_logger().debug(f"Coug {data.get('src_id', 'unknown')}'s Status:")
        self.get_logger().debug(f"    Data: {data}")
        safety_status = data.get('s', {})
        dvl_pos = data.get('dv', {})
        battery_state = data.get('b', {})
        depth_data = data.get('d', {})
        pressure_data = data.get('p', {})

        status = Status()
        status.vehicle_id = data.get('src_id', 0)
        status.safety_status.depth_status = self.make_int8(safety_status.get('d_s', 0))
        status.safety_status.gps_status = self.make_int8(safety_status.get('g_s', 0))
        status.safety_status.modem_status = self.make_int8(safety_status.get('m_s', 0))
        status.safety_status.dvl_status = self.make_int8(safety_status.get('d_s', 0))
        status.safety_status.emergency_status = self.make_int8(safety_status.get('e_s', 0))
        status.dvl_pos.position.x = dvl_pos.get('x', 0.0)
        status.dvl_pos.position.y = dvl_pos.get('y', 0.0)
        status.dvl_pos.position.z = dvl_pos.get('z', 0.0)
        status.dvl_pos.roll = dvl_pos.get('r', 0.0)
        status.dvl_pos.pitch = dvl_pos.get('p', 0.0)
        status.dvl_pos.yaw = dvl_pos.get('y', 0.0)
        status.battery_state.voltage = battery_state.get('volt', 0.0)
        status.depth_data.pose.pose.position.z = -depth_data.get('de', 0.0)
        status.pressure.fluid_pressure = pressure_data.get('pres', 0.0)
        self.status_publisher.publish(status)



    # Function to handle received PING messages
    def recieve_ping(self, sender_id, sender_address):
        # self.get_logger().info(f"Received PING from {sender_id}")
        if sender_id not in self.vehicles_in_mission:
            self.get_logger().warn(f"Received PING from unknown vehicle ID {sender_id}. Ignoring.")
            return
        self.radio_addresses[sender_id] = sender_address
        self.ping_timestamp[sender_id] = time.time()
        self.connections[sender_id] = True


    def init_callback(self, request, response):
        try:
            target_vehicle_id = request.vehicle_id
            if target_vehicle_id is None:
                self.get_logger().error("Initialization request missing target vehicle ID.")
                response.success = False
                return response

            self.get_logger().debug(f"Received initialization request for Coug {target_vehicle_id}")

            init_msg = {
                "message" : "INIT",
                "start" : request.start.data,
                "rosbag_flag" : request.rosbag_flag.data,
                "rosbag_prefix" : request.rosbag_prefix,
                "thruster_arm" : request.thruster_arm.data,
                "dvl_acoustics" : request.dvl_acoustics.data,
                }

            self.send_message(json.dumps(init_msg), self.radio_addresses.get(target_vehicle_id, None))
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Error processing init request: {e}")
            response.success = False

        return response

    # Function to handle emergency kill requests
    def send_e_kill_callback(self, request, response):
        try:
            target_vehicle_id = request.beacon_id
            if target_vehicle_id is None:
                self.get_logger().error("Emergency kill request missing target vehicle ID.")
                response.success = False
                return response

            self.get_logger().debug(f"Received emergency kill request for Coug {target_vehicle_id}")

            e_kill_msg = "E_KILL"

            self.send_message(e_kill_msg, self.radio_addresses.get(target_vehicle_id, None))
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Error processing emergency kill request: {e}")
            response.success = False

        return response

    # Function to confirm emergency kill command
    def confirm_e_kill(self, data):
        self.get_logger().info(f"Confirmation emergency kill for Coug {data.get('src_id')} was {'successful' if data.get('success') else 'unsuccessful'}")
        if data.get("success"):
            self.print_to_gui_publisher.publish(
                ConsoleLog(
                    message=f"Emergency kill command sent to Coug {data.get('src_id') } was {'successful' if data.get('success') else 'unsuccessful'}",
                    vehicle_number=data.get('src_id', 0),
                )
            )
        else:
            self.print_to_gui_publisher.publish(
                ConsoleLog(
                    message=f"Emergency kill command sent to Coug {data.get('src_id') } was {'successful' if data.get('success') else 'unsuccessful'}",
                    vehicle_number=data.get('src_id', 0),
                )
            )

    def key_controls_callback(self, msg):
        vehicle_id = msg.vehicle_id
        if vehicle_id not in self.vehicles_in_mission:
            self.get_logger().warn(f"Received keyboard controls for unknown vehicle ID {vehicle_id}. Ignoring.")
            return
        if self.connections.get(vehicle_id, False):
            self.get_logger().debug(f"Sending keyboard controls to Coug {vehicle_id} through radio")
            key_msg = {
                "message": "KEY_CONTROL",
                "command": {
                    "fin": msg.UCommand.fin,
                    "throttle": msg.UCommand.throttle,
                }
            }
            self.send_message(json.dumps(key_msg), self.radio_addresses.get(vehicle_id, None))
        else:
            self.get_logger().warn(f"Cannot send keyboard controls to Coug {vehicle_id} because there is no connection")

    # Function to handle node destruction
    def destroy_node(self):
        self.running = False
        if self.device is not None and self.device.is_open():
            self.device.close()
            self.get_logger().info("XBee device closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RFBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Shutting down RF Bridge node.")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()