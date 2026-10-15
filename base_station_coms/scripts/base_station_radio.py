#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.serialization import serialize_message
from base_station_interfaces.msg import ConsoleLog, UCommandBase
from cougars_interfaces.msg import MissionFeedback, SystemControl, SystemStatus, WaypointFeedback
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from dvl_msgs.msg import DVL
from geographic_msgs.msg import GeoPoint, RouteNetwork
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, FluidPressure
from std_msgs.msg import Bool, String
import time
import math

import radio_protocol as rp


from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice, XBee64BitAddress
from digi.xbee.exception import TransmitException

import json
import traceback
from pathlib import Path
import base64

class VehicleRadioConnection:
    MAX_XBEE_PAYLOAD_BYTES = 90
    FRAGMENT_DATA_BYTES = 18

    def __init__(self, vehicle_id, node, max_missed_messages, print_to_gui_publisher):
        self.vehicle_id = vehicle_id
        self.node = node
        self.max_missed_messages = max_missed_messages
        self.connection_status = False
        self.radio_address = None
        self.last_ping_time = time.time()
        self.last_status_request_time = 0.0
        self.modem_connection = False
        self.wifi_connection = False
        self.print_to_gui_publisher = print_to_gui_publisher

        self.link_status_publisher = node.create_publisher(
            DiagnosticStatus,
            f'coug{vehicle_id}/link_status',
            10
        )

        self.state_estimate_publisher = node.create_publisher(
            Odometry,
            f'coug{vehicle_id}/odometry/global',
            10
        )

        self.pressure_data_publisher = node.create_publisher(
            FluidPressure,
            f'coug{vehicle_id}/pressure/data',
            10
        )

        self.battery_data_publisher = node.create_publisher(
            BatteryState,
            f'coug{vehicle_id}/battery/data',
            10
        )

        self.dvl_publisher = node.create_publisher(
            DVL,
            f'coug{vehicle_id}/dvl',
            10
        )

        self.waypoint_feedback_publisher = node.create_publisher(
            WaypointFeedback,
            f'coug{vehicle_id}/waypoint_feedback',
            10
        )

        self.mission_feedback_publisher = node.create_publisher(
            MissionFeedback,
            f'coug{vehicle_id}/mission_feedback',
            10
        )

        self.safety_status_publisher = node.create_publisher(
            SystemStatus,
            f'coug{vehicle_id}/safety_status',
            10
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
        connected = msg.level == DiagnosticStatus.OK
        if msg.hardware_id == "wifi":
            self.wifi_connection = connected
        elif msg.hardware_id == "modem":
            self.modem_connection = connected

    def load_mission_callback(self, msg):
        if self.wifi_connection or not self.connection_status:
            return

        mission_bytes = bytes(serialize_message(msg))

        if len(mission_bytes) <= self.MAX_XBEE_PAYLOAD_BYTES:
            if self.send_message(self.node.send_message, mission_bytes):
                self.node.get_logger().info(
                    f"Sent RouteNetwork to vehicle {self.vehicle_id} over radio"
                )
            else:
                self.node.get_logger().error(
                    f"Failed to send RouteNetwork to vehicle {self.vehicle_id}"
                )
            return

        transfer_id = f"{time.time_ns() & 0xffffffffffff:x}"
        chunks = [
            mission_bytes[offset:offset + self.FRAGMENT_DATA_BYTES]
            for offset in range(0, len(mission_bytes), self.FRAGMENT_DATA_BYTES)
        ]

        for index, chunk in enumerate(chunks):
            fragment = json.dumps({
                "message": "MISSION_FRAGMENT",
                "id": transfer_id,
                "i": index,
                "n": len(chunks),
                "data": base64.b64encode(chunk).decode('ascii'),
            }, separators=(',', ':')).encode('utf-8')

            if not self.send_message(self.node.send_message, fragment):
                self.node.get_logger().error(
                    f"Failed to send mission fragment {index + 1}/{len(chunks)} to vehicle {self.vehicle_id}"
                )
                return

            time.sleep(0.03)

        if self.send_message(self.node.send_message, b"MISSION_DONE"):
            self.node.get_logger().info(
                f"Sent fragmented RouteNetwork to vehicle {self.vehicle_id} over radio"
            )
        else:
            self.node.get_logger().error(
                f"Failed to send RouteNetwork to vehicle {self.vehicle_id}"
            )

    def start_mission_callback(self, msg):
        if self.wifi_connection or not self.connection_status:
            return

        system_control_message = rp.SystemControlMessage(
            src_id=self.node.vehicle_id,
            start=msg.start.data,
            rosbag_flag=msg.rosbag_flag.data,
            rosbag_prefix=msg.rosbag_prefix,
            thruster_arm=msg.thruster_arm.data,
            dvl_acoustics=msg.dvl_acoustics.data
        )
        payload = system_control_message.pack()

        if self.send_message(self.node.send_message, payload):
            self.node.get_logger().info(
                f"Sent INIT command to vehicle {self.vehicle_id} over radio"
            )
        else:
            self.node.get_logger().error(
                f"Failed to send INIT command to vehicle {self.vehicle_id} over radio"
            )

    def emergency_kill_callback(self, msg):
        if msg.data and not self.wifi_connection and self.connection_status:
            # log sending over radio
            disarm_thruster_message = rp.DisarmThrusterMessage(src_id=self.vehicle_id)
            payload = disarm_thruster_message.pack()
            if self.send_message(self.node.send_message, payload):
                self.node.get_logger().info(f"Vehicle {self.vehicle_id} was disarmed over radio")
    
    def confirm_disarm_thruster(self, data):
        confirmation = rp.ConfirmDisarmThrusterMessage.unpack(data)
        self.node.get_logger().info(f"Vehicle {self.vehicle_id} confirmed disarm thruster over radio")
        self.print_to_gui_publisher.publish(ConsoleLog(
            message="Disarm thruster command was successful",
            vehicle_number=confirmation.src_id))

    def emergency_surface_callback(self, msg):
        if msg.data and not self.wifi_connection and self.connection_status:
            self.node.get_logger().info(f"Emergency surface for vehicle {self.vehicle_id} over radio")

    def hardware_control_callback(self, msg):
        if self.wifi_connection:
            return
        if not self.connection_status:
            self.node.get_logger().warn(
                f"Not sending hardware control for vehicle {self.vehicle_id} over radio because radio is disconnected"
            )
            self.print_to_gui_publisher.publish(ConsoleLog(
                message=f"Hardware control {msg.data} not sent: no WiFi or radio connection",
                vehicle_number=self.vehicle_id))
            return

        try:
            device_str, mode_str = msg.data.split(":", 1)
            device = rp.HardwareDevice[device_str]
            mode = rp.HardwareMode[mode_str]
        except (ValueError, KeyError):
            self.node.get_logger().error(f"Ignoring malformed hardware control command: {msg.data}")
            return

        hardware_control_message = rp.HardwareControlMessage(
            src_id=self.node.vehicle_id,
            device=int(device),
            mode=int(mode)
        )
        payload = hardware_control_message.pack()

        if self.send_message(self.node.send_message, payload):
            self.node.get_logger().info(
                f"Sent {device_str} {mode_str} command to vehicle {self.vehicle_id} over radio"
            )
            self.print_to_gui_publisher.publish(ConsoleLog(
                message=f"Sent hardware control {msg.data} over radio",
                vehicle_number=self.vehicle_id))
        else:
            self.node.get_logger().error(
                f"Failed to send {device_str} {mode_str} command to vehicle {self.vehicle_id} over radio"
            )

    def confirm_hardware_control(self, data):
        confirmation = rp.ConfirmHardwareControlMessage.unpack(data)
        device_name = "Relay" if confirmation.device == int(rp.HardwareDevice.RELAY) else "Strobe"
        mode_name = {
            int(rp.HardwareMode.AUTO): "Auto",
            int(rp.HardwareMode.ON): "On",
            int(rp.HardwareMode.OFF): "Off",
        }.get(confirmation.mode, "Unknown")
        status = "succeeded" if confirmation.success else "failed"
        self.node.get_logger().info(
            f"Vehicle {self.vehicle_id} {device_name} {mode_name} command {status}"
        )
        self.print_to_gui_publisher.publish(ConsoleLog(
            message=f"{device_name} set to {mode_name} ({status})",
            vehicle_number=confirmation.src_id))

    def handle_ping(self, sender_address):
        self.radio_address = sender_address
        self.last_ping_time = time.time()
        self.connection_status = True
        self.publish_link_status()

    def has_address(self):
        return self.radio_address is not None

    def is_connected(self):
        return self.connection_status

    def seconds_since_ping(self):
        return int(time.time() - self.last_ping_time)

    def check_connection(self, ping_frequency):
        if self.seconds_since_ping() >= ping_frequency * self.max_missed_messages:
            self.connection_status = False
        self.publish_link_status()

    def should_request_status(self):
        return self.connection_status and not self.wifi_connection and self.has_address()

    def send_message(self, send_fn, msg):
        if not self.has_address():
            return False
        return send_fn(msg, self.radio_address)

    def request_status(self, send_fn):
        if not self.should_request_status():
            return False

        status_request = rp.RequestStatusMessage(src_id=self.vehicle_id)
        sent = self.send_message(send_fn, status_request.pack())
        if sent:
            self.last_status_request_time = time.time()
            self.node.get_logger().debug(f"Requested status from Coug{self.vehicle_id} over radio")
        return sent

    def publish_link_status(self):
        status_msg = DiagnosticStatus()
        status_msg.name = f"Coug{self.vehicle_id} Radio Connection"
        status_msg.hardware_id = "radio"
        status_msg.level = DiagnosticStatus.OK if self.connection_status else DiagnosticStatus.ERROR
        status_msg.message = "Connected" if self.connection_status else "Disconnected"
        status_msg.values.append(KeyValue(key="last_message_time", value=str(self.last_ping_time)))
        self.link_status_publisher.publish(status_msg)
    
    # Function to handle received status messages
    def recieve_status(self, data):
        

        now = self.node.get_clock().now().to_msg()
        
        status = rp.StatusResponseMessage.unpack(data)

        self.node.get_logger().debug(
            f"Received STATUS from Coug {status.src_id}: {data}"
        )

        odometry_msg = Odometry()
        odometry_msg.header.stamp = now
        odometry_msg.header.frame_id = "map"
        odometry_msg.child_frame_id = f"coug{status.src_id}/base_link"  # Assuming the child frame is named after the vehicle ID
        odometry_msg.pose.pose.position.x = status.x
        odometry_msg.pose.pose.position.y = status.y
        odometry_msg.pose.pose.position.z = status.depth
        odometry_msg.pose.pose.orientation.x = status.orientation_x
        odometry_msg.pose.pose.orientation.y = status.orientation_y
        odometry_msg.pose.pose.orientation.z = status.orientation_z
        odometry_msg.pose.pose.orientation.w = status.orientation_w
        odometry_msg.pose.covariance[0] = status.cov_x
        odometry_msg.pose.covariance[7] = status.cov_y
        odometry_msg.pose.covariance[14] = status.cov_z
        odometry_msg.pose.covariance[21] = status.cov_roll
        odometry_msg.pose.covariance[28] = status.cov_pitch
        odometry_msg.pose.covariance[35] = status.cov_yaw
        self.state_estimate_publisher.publish(odometry_msg)


        fluid_pressure_msg = FluidPressure()
        fluid_pressure_msg.header.stamp = now
        fluid_pressure_msg.fluid_pressure = status.pressure
        self.pressure_data_publisher.publish(fluid_pressure_msg)


        battery_state_msg = BatteryState()
        battery_state_msg.header.stamp = now
        battery_state_msg.voltage = status.battery_voltage
        battery_state_msg.current = status.battery_current
        self.battery_data_publisher.publish(battery_state_msg)


        dvl_msg = DVL()
        dvl_msg.header.stamp = now
        dvl_msg.velocity.x = status.dvl_velocity_x
        dvl_msg.velocity.y = status.dvl_velocity_y
        dvl_msg.velocity.z = status.dvl_velocity_z
        dvl_msg.altitude = status.dvl_altitude
        self.dvl_publisher.publish(dvl_msg)


        waypoint_msg = WaypointFeedback()
        waypoint_msg.header.stamp = now
        waypoint_msg.state = status.waypoint_state
        waypoint_msg.horizontal_distance_error = status.horizontal_distance_error
        waypoint_msg.depth_error = status.depth_error
        waypoint_msg.bearing_error = status.bearing_error
        self.waypoint_feedback_publisher.publish(waypoint_msg)


        mission_msg = MissionFeedback()
        mission_msg.header.stamp = now
        mission_msg.mission_id = str(status.mission_id)
        mission_msg.state = status.mission_state
        mission_msg.waypoints_completed = status.waypoints_completed
        mission_msg.waypoints_total = status.waypoints_total
        mission_msg.elapsed_time = status.elapsed_time
        if waypoint_msg is not None:
            mission_msg.current = waypoint_msg
        self.mission_feedback_publisher.publish(mission_msg)

        # Only publish once the vehicle has actually received a safety status,
        # otherwise the default zeros would show every sensor as healthy
        if status.safety_valid:
            safety_msg = SystemStatus()
            safety_msg.header.stamp = now
            safety_msg.imu_published.data = status.imu_published
            safety_msg.depth_status.data = status.depth_status
            safety_msg.gps_status.data = status.gps_status
            safety_msg.modem_status.data = status.modem_status
            safety_msg.dvl_status.data = status.dvl_status
            safety_msg.emergency_status.data = status.emergency_status
            self.safety_status_publisher.publish(safety_msg)




class RFBridge(Node):
    MAX_XBEE_PAYLOAD_BYTES = 90

    def __init__(self):
        super().__init__('base_station_rf_bridge')

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

        # Whether to request status from vehicles over the radio when WiFi is disconnected
        self.request_status = self.declare_parameter('request_status', True).value
        self.status_request_frequency_seconds = self.declare_parameter(
            'status_request_frequency_seconds',
            2.0
        ).value

        # base station vehicle ID
        self.declare_parameter('vehicle_id', 15)
        self.vehicle_id = self.get_parameter('vehicle_id').get_parameter_value().integer_value

        #XBee configuration
        self.xbee_port = self.declare_parameter('xbee_port', '/dev/frost/xbee_radio').value
        self.xbee_baud = self.declare_parameter('xbee_baud', 9600).value

        # Initialize XBee device
        self.device = XBeeDevice(self.xbee_port, self.xbee_baud)

        # Mapping of radio addresses to vehicle IDs
        self.radio_addresses = {}

        # transfer_id -> {"chunks": {index: bytes}, "total": int}
        self.fragment_transfers = {}

        # publishes console log messages to GUI
        self.print_to_gui_publisher = self.create_publisher(ConsoleLog, 'console_log', 10)
    
        self.running = True
        self.max_msgs_missed = 5  # Number of missed messages before considering a vehicle disconnected

        # Create a VehicleRadioConnection for each vehicle in the mission
        self.vehicle_radios = {
            vehicle: VehicleRadioConnection(vehicle, self, self.max_msgs_missed, self.print_to_gui_publisher)
            for vehicle in self.vehicles_in_mission
        }

        origin_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.origin_subscriber = self.create_subscription(
            GeoPoint,
            '/send_origin',
            self.origin_callback,
            origin_qos,
        )

        # Same UCommandBase teleop stream the WiFi bridge consumes; used as a radio
        # fallback for the vehicle currently being teleoperated when it has no WiFi link.
        self.keyboard_controls_subscriber = self.create_subscription(
            UCommandBase,
            '/keyboard_controls',
            self.keyboard_controls_callback,
            10,
        )

        self.timer = self.create_timer(self.ping_frequency, self.check_connections)

        if self.request_status:
            self.status_request_timer = self.create_timer(
                self.status_request_frequency_seconds,
                self.request_status_when_wifi_disconnected
            )
        else:
            self.get_logger().info("Radio status requests disabled by parameter")

        try:
            self.device.open()
            self.get_logger().info(f"Opened XBee device on {self.xbee_port} at {self.xbee_baud} baud.")
                    # Register XBee data receive callback
            self.device.add_data_received_callback(self.data_receive_callback)
            self.get_logger().info("RF Bridge node started using digi-xbee library.")
        except Exception as e:
            self.get_logger().error(f"Failed to open XBee deviceon {self.xbee_port}: {e}")

    def send_message(self, msg, address):
        payload = bytes(msg)
        try:
            remote_device = RemoteXBeeDevice(self.device, address)
            self.device.send_data(remote_device, payload)
            self.get_logger().debug(f"Sent via XBee: {msg}")
            return True
        except TransmitException as e:
            self.get_logger().debug(f"XBee transmission error - TransmitException: {e}")
            self.get_logger().debug(traceback.format_exc())
            return False
        except Exception as e:
            self.get_logger().debug(f"XBee transmission error - Exception: {str(e)}")
            self.get_logger().debug(traceback.format_exc())
            return False

    # Callback for receiving data from XBee
    def data_receive_callback(self, xbee_message):
        try:
            payload = xbee_message.data
            sender_address = xbee_message.remote_device.get_64bit_addr()

            if isinstance(payload, (bytes, bytearray)) and payload.startswith(b'{"'):
                try:
                    decoded = json.loads(payload.decode('utf-8'))
                except Exception:
                    decoded = None
                if isinstance(decoded, dict) and decoded.get("message") == "FRAGMENT":
                    self.handle_fragment(decoded, sender_address)
                    return

            self.dispatch_payload(payload, sender_address)
        except Exception as e:
            self.get_logger().error(f"Error in data_receive_callback: {e}")
            self.get_logger().error(traceback.format_exc())

    # Reassembles a fragmented message and dispatches it once all chunks arrive
    def handle_fragment(self, fragment, sender_address):
        transfer_id = fragment.get("id")
        chunk_index = fragment.get("i")
        total_chunks = fragment.get("n")
        data = fragment.get("data")

        if transfer_id is None or chunk_index is None or total_chunks is None or data is None:
            self.get_logger().warning("Ignoring malformed FRAGMENT message: missing fields")
            return

        transfer = self.fragment_transfers.setdefault(
            transfer_id, {"chunks": {}, "total": total_chunks}
        )
        transfer["chunks"][chunk_index] = base64.b64decode(data)

        if len(transfer["chunks"]) != transfer["total"]:
            return

        payload = b"".join(transfer["chunks"][i] for i in range(transfer["total"]))
        del self.fragment_transfers[transfer_id]
        self.dispatch_payload(payload, sender_address)

    # Routes a fully-assembled payload to the right handler based on its message ID
    def dispatch_payload(self, payload, sender_address):
        sender_id = payload[1] if len(payload) > 1 else None

        msg_id = payload[0] if len(payload) > 0 else None
        if msg_id is not None:
            self.get_logger().debug(f"Received message ID {msg_id} from {sender_id}")

        if msg_id == int(rp.MessageID.PING):
            self.recieve_ping(payload, sender_address)
        elif msg_id == int(rp.MessageID.STATUS_RESPONSE):
            vehicle_id = self.radio_addresses.get(sender_address, sender_id)
            if vehicle_id in self.vehicle_radios:
                self.vehicle_radios[vehicle_id].recieve_status(payload)
            else:
                self.get_logger().warning(
                    f"Ignoring status from unknown vehicle {sender_id}")
        elif msg_id == int(rp.MessageID.CONFIRM_DISARM_THRUSTER):
            vehicle_id = self.radio_addresses.get(sender_address, sender_id)
            if vehicle_id in self.vehicle_radios:
                self.vehicle_radios[vehicle_id].confirm_disarm_thruster(payload)
            else:
                self.get_logger().warning(
                    f"Ignoring disarm confirmation from unknown vehicle {sender_id}")
        elif msg_id == int(rp.MessageID.CONFIRM_SYSTEM_CONTROL):
            confirmation = rp.ConfirmSystemControlMessage.unpack(payload)
            self.print_to_gui_publisher.publish(ConsoleLog(
                message="Start mission command was successful",
                vehicle_number=confirmation.src_id))
        elif msg_id == int(rp.MessageID.CONFIRM_HARDWARE_CONTROL):
            vehicle_id = self.radio_addresses.get(sender_address, sender_id)
            if vehicle_id in self.vehicle_radios:
                self.vehicle_radios[vehicle_id].confirm_hardware_control(payload)
            else:
                self.get_logger().warning(
                    f"Ignoring hardware control confirmation from unknown vehicle {sender_id}")

    # Function to check connections and send PING messages
    def check_connections(self):
    
        self.get_logger().debug(f"Sending PING")
        ping = rp.PingMessage(src_id=self.vehicle_id).pack()

        if any(not vehicle_radio.has_address() for vehicle_radio in self.vehicle_radios.values()):
            try:
                self.device.send_data_broadcast(ping)
            except Exception as e:
                self.get_logger().debug(f"Failed to send broadcast PING: {e}")
        else:
            for vehicle_radio in self.vehicle_radios.values():
                self.get_logger().debug(f"Sending PING to Coug{vehicle_radio.vehicle_id}")
                vehicle_radio.send_message(self.send_message, ping)

        for vehicle_radio in self.vehicle_radios.values():
            vehicle_radio.check_connection(self.ping_frequency)

    def request_status_when_wifi_disconnected(self):
        for vehicle_radio in self.vehicle_radios.values():
            vehicle_radio.request_status(self.send_message)

    def origin_callback(self, msg):
        origin_message = rp.OriginUpdateMessage(
            src_id=self.vehicle_id,
            latitude=msg.latitude,
            longitude=msg.longitude,
            altitude=msg.altitude
        )
        payload = origin_message.pack()
        sent_to = []
        for vehicle_radio in self.vehicle_radios.values():
            if vehicle_radio.wifi_connection or not vehicle_radio.is_connected():
                continue
            if vehicle_radio.send_message(self.send_message, payload):
                sent_to.append(vehicle_radio.vehicle_id)

        if sent_to:
            self.get_logger().info(
                f"Sent origin over radio to vehicles {sent_to}: "
                f"lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}"
            )
        else:
            self.get_logger().warn(
                "Received origin, but no radio-connected vehicle without WiFi was available"
            )

    def keyboard_controls_callback(self, msg):
        vehicle_radio = self.vehicle_radios.get(msg.vehicle_id)
        if vehicle_radio is None:
            self.get_logger().warn(f"Received keyboard controls for unknown vehicle ID {msg.vehicle_id}. Ignoring.")
            return
        if vehicle_radio.wifi_connection:
            return
        if not vehicle_radio.is_connected():
            self.get_logger().debug(
                f"Not sending keyboard controls for vehicle {msg.vehicle_id} over radio because radio is disconnected"
            )
            return
        self.get_logger().info(f"Sending keyboard controls for vehicle {msg.vehicle_id} over radio: thruster_enabled={msg.thruster_enabled}, thruster={msg.ucommand.thruster}, fin={list(msg.ucommand.fin)}")

        key_control_message = rp.KeyControlMessage(
            src_id=self.vehicle_id,
            thruster_enabled=msg.thruster_enabled,
            thruster=int(msg.ucommand.thruster),
            fin=list(msg.ucommand.fin),
        )
        if not vehicle_radio.send_message(self.send_message, key_control_message.pack()):
            self.get_logger().error(f"Failed to send keyboard controls to vehicle {msg.vehicle_id} over radio")

    # Function to handle received PING messages
    def recieve_ping(self, payload, sender_address):
        ping_message = rp.PingMessage.unpack(payload)
        sender_id = ping_message.src_id
        if sender_id not in self.vehicles_in_mission:
            self.get_logger().warn(f"Received PING from unknown vehicle ID {sender_id}. Ignoring.")
            return
        self.radio_addresses[sender_address] = sender_id
        self.vehicle_radios[sender_id].handle_ping(sender_address)

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
