import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class OdomBridge(Node):
    def __init__(self):
        super().__init__('odom_bridge')
        # 1. الاشتراك في التوبيكات القادمة من ESP32 عبر micro-ROS
        self.create_subscription(Int32, '/encoder_left', self.left_cb, 10)
        self.create_subscription(Int32, '/encoder_right', self.right_cb, 10)
        
        # 2. تجهيز ناشر الأودومتري والتحويلات (TF)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 3. ثوابت الروبوت (قم بقياسها بدقة وتعديلها هنا)
        self.wheel_radius = 0.033  # نصف قطر العجلة (3.3 سم)
        self.wheel_sep = 0.17     # المسافة بين العجلتين (17 سم)
        self.ticks_per_rev = 20   # عدد النبضات لكل لفة كاملة

        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.l_ticks, self.r_ticks = 0, 0
        self.prev_l, self.prev_r = 0, 0
        self.create_timer(0.05, self.compute_odom) # تحديث بمعدل 20Hz

    def left_cb(self, msg): self.l_ticks = msg.data
    def right_cb(self, msg): self.r_ticks = msg.data

    def compute_odom(self):
        # حساب المسافة المقطوعة لكل عجلة بالمتر
        d_l = (self.l_ticks - self.prev_l) / self.ticks_per_rev * (2 * math.pi * self.wheel_radius)
        d_r = (self.r_ticks - self.prev_r) / self.ticks_per_rev * (2 * math.pi * self.wheel_radius)
        self.prev_l, self.prev_r = self.l_ticks, self.r_ticks

        d_dist = (d_r + d_l) / 2.0
        d_th = (d_r - d_l) / self.wheel_sep

        self.x += d_dist * math.cos(self.th)
        self.y += d_dist * math.sin(self.th)
        self.th += d_th

        # 4. نشر الـ TF (الجسر الذي يربط الروبوت بالخريطة)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.th / 2.0)
        t.transform.rotation.w = math.cos(self.th / 2.0)
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init(); rclpy.spin(OdomBridge()); rclpy.shutdown()

if __name__ == '__main__': main()