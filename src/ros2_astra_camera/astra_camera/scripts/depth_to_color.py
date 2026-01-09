#!/usr/bin/env python3

import message_filters
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

class D2CTestNode(Node):

    def __init__(self):
        super().__init__("d2c_test_node")
        
        # Initialize CvBridge once to save CPU resources
        self.cv_bridge = CvBridge()
        
        # Subscribers for Color and Depth streams
        self.rgb_sub = message_filters.Subscriber(
            self, Image, "/camera/color/image_raw", qos_profile=qos_profile_sensor_data)
        
        self.depth_sub = message_filters.Subscriber(
            self, Image, "/camera/depth/image_raw", qos_profile=qos_profile_sensor_data)

        # Approximate Time Synchronizer to align frames based on timestamps
        # Queue size of 10 and slop of 0.1 seconds
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 10, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.callback)

        # Publisher for the merged Depth-to-Color image
        self.d2c_pub = self.create_publisher(
            Image, "/camera/depth_to_color/image_raw", 10)
            
        self.get_logger().info("Depth-to-Color Synchronization Node has started.")

    def callback(self, rgb_msg: Image, depth_msg: Image):
        try:
            # Convert ROS Image messages to OpenCV format
            rgb_img = self.cv_bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_img = self.cv_bridge.imgmsg_to_cv2(depth_msg, "16UC1")

            # Validate resolution matching (Crucial for AMR sensor fusion)
            if rgb_img.shape[:2] != depth_img.shape[:2]:
                depth_img = cv2.resize(
                    depth_img, 
                    (rgb_img.shape[1], rgb_img.shape[0]), 
                    interpolation=cv2.INTER_NEAREST
                )

            # Properly normalize 16-bit depth (mm) to 8-bit for visualization
            depth_normalized = cv2.normalize(
                depth_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
            )

            # Convert normalized depth to BGR to allow merging with color image
            gray_depth_bgr = cv2.cvtColor(depth_normalized, cv2.COLOR_GRAY2BGR)

            # Merge images using bitwise OR to overlay depth data on RGB
            d2c_image = cv2.bitwise_or(rgb_img, gray_depth_bgr)

            # Convert result back to ROS message and publish
            d2c_msg = self.cv_bridge.cv2_to_imgmsg(d2c_image, "bgr8")
            
            # Preserve the header and timestamp for TF consistency
            d2c_msg.header = rgb_msg.header 
            self.d2c_pub.publish(d2c_msg)
            
        except Exception as e:
            self.get_logger().error(f"Failed to process D2C frame: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = D2CTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()