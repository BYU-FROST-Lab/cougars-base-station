
class ODOM:
    def __init__(self, msg):
        self.timestamp = msg.header.stamp.sec + (10e-9 * msg.header.stamp.nanosec)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

class GPS_FIX:
    def __init__(self, msg):
        self.timestamp = msg.header.stamp.sec + (10e-9 * msg.header.stamp.nanosec)
        self.gps_lat = msg.latitude
        self.gps_long = msg.longitude
        self.elevation = msg.altitude

        