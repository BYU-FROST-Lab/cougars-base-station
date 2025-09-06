import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
class originBroadcaster(Node):

    def __init__(self):
        super().__init__('originBroadcaster')
        self.lat=40.249999 
        self.lon=-111.6499974
        qos=QoSProfile(depth=10,durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        
            
        self.origin_listener=self.create_subscription(NavSatFix,"/map_viz_origin",self.origin_callback,10)
        self.origin_pub = self.create_publisher(PoseStamped, '/local_xy_origin', qos)
        self.pubtimer = self.create_timer(1.0, self.publish_origin)
    def publish_origin(self):
        msg=PoseStamped()
        msg.pose.position.x=self.lat
        msg.pose.position.y=self.lon
        self.origin_pub.publish(msg)
    def origin_callback(self,msg):
        self.lat=msg.latitude
        self.lon=msg.longitude

def main(args=None):
    rclpy.init(args=args)
    node=originBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()