#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from astra_camera_msgs.srv import GetString
import json
import sys

class GetSupportedVideoModes(Node):

    def __init__(self):
        super().__init__("get_supported_video_modes_client")
        
        # Validate command-line arguments
        if len(sys.argv) < 2:
            self.get_logger().error("Please provide a stream name (e.g., color, depth, or ir)")
            sys.exit(1)
            
        self.stream = sys.argv[1]
        service_name = f"/camera/get_{self.stream}_supported_video_modes"
        
        self.cli = self.create_client(GetString, service_name)
        
        # Wait for the camera service to be ready
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Service {service_name} not available, waiting...")
            
        self.req = GetString.Request()

    def send_request(self):
        # Asynchronous call for better non-blocking performance
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    video_mode_client = GetSupportedVideoModes()
    
    video_mode_client.get_logger().info(f"Fetching supported modes for: {video_mode_client.stream}")
    response = video_mode_client.send_request()

    if response and response.success:
        try:
            # Parse and display the JSON data
            data = json.loads(response.data)
            video_mode_client.get_logger().info("Supported Video Modes:")
            for mode in data:
                print(f" - {mode}")
        except json.JSONDecodeError:
            video_mode_client.get_logger().error("Received malformed JSON data.")
    else:
        error_msg = response.message if response else "No response"
        video_mode_client.get_logger().error(f"Failed to get modes: {error_msg}")

    # Cleanup resources
    video_mode_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()