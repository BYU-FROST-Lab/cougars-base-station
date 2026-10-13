#!/usr/bin/env python3

import math
import random

import rclpy
from rclpy.node import Node

from base_station_interfaces.msg import ConsoleLog
from cougars_interfaces.msg import MissionFeedback, SystemStatus, WaypointFeedback
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from dvl_msgs.msg import DVL
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, FluidPressure
from unique_identifier_msgs.msg import UUID


def make_uuid(value):
    msg = UUID()
    msg.uuid = [0] * 16
    msg.uuid[-1] = value & 0xFF
    return msg


def yaw_to_quaternion(yaw_degrees):
    yaw = math.radians(yaw_degrees)
    half_yaw = yaw / 2.0
    return math.sin(half_yaw), math.cos(half_yaw)


class GuiTester(Node):
    def __init__(self):
        super().__init__('base_station_gui_tester')
        self.declare_parameter('vehicles', [1, 2, 3])
        self.declare_parameter('rate_hz', 1.0)
        self.declare_parameter('randomize_links', True)

        self.vehicles = list(self.get_parameter('vehicles').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.randomize_links = bool(self.get_parameter('randomize_links').value)
        self.tick = 0

        self.test_publishers = {}
        for vehicle in self.vehicles:
            prefix = f'coug{vehicle}'
            self.test_publishers[vehicle] = {
                'safety': self.create_publisher(SystemStatus, f'{prefix}/safety_status', 10),
                'state': self.create_publisher(Odometry, f'{prefix}/odometry/global', 10),
                'dvl': self.create_publisher(DVL, f'{prefix}/dvl/data', 10),
                'pressure': self.create_publisher(FluidPressure, f'{prefix}/pressure/data', 10),
                'battery': self.create_publisher(BatteryState, f'{prefix}/battery/data', 10),
                'mission': self.create_publisher(MissionFeedback, f'{prefix}/mission_feedback', 10),
                'waypoint': self.create_publisher(WaypointFeedback, f'{prefix}/waypoint_feedback', 10),
                'link': self.create_publisher(DiagnosticStatus, f'{prefix}/link_status', 10),
            }

        self.console_pub = self.create_publisher(ConsoleLog, 'console_log', 10)
        period = 1.0 / self.rate_hz if self.rate_hz > 0.0 else 1.0
        self.timer = self.create_timer(period, self.publish_test_data)
        self.get_logger().info(f'GUI tester publishing for vehicles {self.vehicles} at {self.rate_hz} Hz')

    def publish_test_data(self):
        now = self.get_clock().now().to_msg()
        self.tick += 1

        for vehicle in self.vehicles:
            pubs = self.test_publishers[vehicle]
            self.publish_safety(pubs['safety'], now)
            self.publish_state_estimate(pubs['state'], now, vehicle)
            self.publish_dvl(pubs['dvl'], now)
            self.publish_pressure(pubs['pressure'], now)
            self.publish_battery(pubs['battery'], now)
            waypoint = self.make_waypoint_feedback(now, vehicle)
            pubs['waypoint'].publish(waypoint)
            pubs['mission'].publish(self.make_mission_feedback(now, waypoint))
            self.publish_links(pubs['link'], vehicle)

        if self.tick % 10 == 1:
            msg = ConsoleLog()
            msg.vehicle_number = 0
            msg.message = f'GUI tester heartbeat {self.tick}'
            self.console_pub.publish(msg)

    def publish_safety(self, publisher, stamp):
        msg = SystemStatus()
        msg.header.stamp = stamp
        msg.depth_status.data = random.choice([0, 0, 0, 1])
        msg.imu_published.data = random.choice([True, True, True, False])
        msg.gps_status.data = random.choice([0, 0, 0, 1])
        msg.modem_status.data = random.choice([0, 0, 1])
        msg.dvl_status.data = random.choice([0, 0, 0, 1])
        msg.emergency_status.data = random.choice([0, 0, 0, 1, 2])
        publisher.publish(msg)

    def publish_state_estimate(self, publisher, stamp, vehicle):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.child_frame_id = f'coug{vehicle}/base_link'
        msg.pose.pose.position.x = 10.0 * vehicle + math.sin(self.tick / 5.0) * 8.0
        msg.pose.pose.position.y = -5.0 * vehicle + math.cos(self.tick / 6.0) * 6.0
        depth = 2.0 + vehicle + abs(math.sin(self.tick / 8.0)) * 12.0
        msg.pose.pose.position.z = -depth
        heading = (self.tick * 12.0 + vehicle * 25.0) % 360.0
        z, w = yaw_to_quaternion(heading)
        msg.pose.pose.orientation.z = z
        msg.pose.pose.orientation.w = w
        msg.twist.twist.linear.x = random.uniform(-0.8, 0.8)
        msg.twist.twist.linear.y = random.uniform(-0.8, 0.8)
        msg.twist.twist.linear.z = random.uniform(-0.2, 0.2)
        publisher.publish(msg)

    def publish_dvl(self, publisher, stamp):
        msg = DVL()
        msg.header.stamp = stamp
        msg.velocity.x = random.uniform(-1.2, 1.2)
        msg.velocity.y = random.uniform(-1.2, 1.2)
        msg.velocity.z = random.uniform(-0.4, 0.4)
        msg.velocity_valid = True
        msg.altitude = random.uniform(1.0, 20.0)
        msg.fom = random.uniform(0.01, 0.2)
        publisher.publish(msg)

    def publish_pressure(self, publisher, stamp):
        msg = FluidPressure()
        msg.header.stamp = stamp
        msg.fluid_pressure = random.uniform(85000.0, 130000.0)
        msg.variance = 0.1
        publisher.publish(msg)

    def publish_battery(self, publisher, stamp):
        msg = BatteryState()
        msg.header.stamp = stamp
        msg.voltage = random.uniform(6.8, 8.4)
        msg.percentage = random.uniform(0.2, 1.0)
        publisher.publish(msg)

    def make_waypoint_feedback(self, stamp, vehicle):
        msg = WaypointFeedback()
        msg.header.stamp = stamp
        msg.waypoint_id = make_uuid(self.tick + vehicle)
        msg.state = random.choice([
            WaypointFeedback.STATE_TRANSITING,
            WaypointFeedback.STATE_ARRIVED,
            WaypointFeedback.STATE_PARKING,
            WaypointFeedback.STATE_SKIPPED,
        ])
        msg.horizontal_distance_error = random.uniform(0.0, 80.0)
        msg.depth_error = random.uniform(-3.0, 3.0)
        msg.bearing_error = random.uniform(-180.0, 180.0)
        return msg

    def make_mission_feedback(self, stamp, waypoint):
        total = 8
        completed = self.tick % (total + 1)
        msg = MissionFeedback()
        msg.header.stamp = stamp
        msg.mission_id = 'gui_test_mission'
        msg.state = random.choice([
            MissionFeedback.STATE_IDLE,
            MissionFeedback.STATE_RUNNING,
            MissionFeedback.STATE_PAUSED,
            MissionFeedback.STATE_COMPLETE,
            MissionFeedback.STATE_ABORTED,
        ])
        msg.waypoints_completed = completed
        msg.waypoints_total = total
        msg.elapsed_time = float(self.tick)
        msg.current = waypoint
        return msg

    def publish_links(self, publisher, vehicle):
        for hardware_id in ('wifi', 'radio', 'modem'):
            connected = True if not self.randomize_links else random.choice([True, True, True, False])
            msg = DiagnosticStatus()
            msg.name = f'Coug{vehicle} {hardware_id.capitalize()} Connection'
            msg.hardware_id = hardware_id
            msg.level = DiagnosticStatus.OK if connected else DiagnosticStatus.ERROR
            msg.message = 'Connected' if connected else 'Disconnected'
            seconds = 0 if connected else random.randint(3, 60)
            key = 'last_ping_seconds' if hardware_id == 'wifi' else 'last_message_time'
            value = str(seconds) if hardware_id == 'wifi' else str(self.get_clock().now().nanoseconds / 1e9 - seconds)
            msg.values.append(KeyValue(key=key, value=value))
            publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GuiTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
