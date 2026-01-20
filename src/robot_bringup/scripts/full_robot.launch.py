import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import tf2_ros
import math

class OdomBridge(Node):
    def __init__(self):
        super().__init__('odom_bridge')
        
        # 1. Subscriptions for Encoder Ticks (from ESP32 via micro-ROS)
        # Using the persistent device /dev/esp32 connection implicitly
        self.create_subscription(Int32, '/encoder_left', self.left_cb, 10)
        self.create_subscription(Int32, '/encoder_right', self.right_cb, 10)
        
        # 2. Publisher for Raw Wheel Odometry
        # EKF node will consume this topic
        self.odom_pub = self.create_publisher(Odometry, '/odom/wheel', 10)
        
        # 3. Robot Kinematics (Meters and SI Units)
        # Corrected units: 3.3cm = 0.033m
        self.declare_parameter('wheel_radius', 0.033) 
        self.declare_parameter('wheel_separation', 0.17)
        self.declare_parameter('ticks_per_rev', 15.0)
        
        self.radius = self.get_parameter('wheel_radius').value
        self.sep = self.get_parameter('wheel_separation').value
        self.tpr = self.get_parameter('ticks_per_rev').value

        # State Variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.l_ticks = 0
        self.r_ticks = 0
        self.prev_l = 0
        self.prev_r = 0
        
        self.last_time = self.get_clock().now()
        self.create_timer(0.05, self.compute_odom) # 20Hz update

    def left_cb(self, msg): self.l_ticks = msg.data
    def right_cb(self, msg): self.r_ticks = msg.data

    def compute_odom(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0: return

        # Calculate distance traveled by each wheel
        # Distance = $\frac{\Delta ticks}{TicksPerRev} \times 2\pi R$
        d_l = ((self.l_ticks - self.prev_l) / self.tpr) * (2 * math.pi * self.radius)
        d_r = ((self.r_ticks - self.prev_r) / self.tpr) * (2 * math.pi * self.radius)
        
        self.prev_l, self.prev_r = self.l_ticks, self.r_ticks

        # Linear and Angular displacement
        d_dist = (d_r + d_l) / 2.0
        d_th = (d_r - d_l) / self.sep

        # Update Pose
        self.x += d_dist * math.cos(self.th)
        self.y += d_dist * math.sin(self.th)
        self.th += d_th

        # Prepare Odometry Message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)

        # Velocity (Twist) - Important for EKF
        odom.twist.twist.linear.x = d_dist / dt
        odom.twist.twist.angular.z = d_th / dt

        self.odom_pub.publish(odom)
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = OdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()