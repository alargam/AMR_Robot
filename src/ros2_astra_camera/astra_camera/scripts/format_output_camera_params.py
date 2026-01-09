#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from astra_camera_msgs.srv import GetCameraParams

class GetCameraParamsNode(Node):

    def __init__(self):
        super().__init__("get_camera_params_client")
        
        # Create the client
        self.cli = self.create_client(
            GetCameraParams, "/camera/get_camera_params")
            
        # Wait for the service to become available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
            
        self.req = GetCameraParams.Request()

    def send_request(self):
        # Call the service asynchronously
        self.future = self.cli.call_async(self.req)
        
        # Spin until the response is received
        rclpy.spin_until_future_complete(self, self.future)
        
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    camera_client = GetCameraParamsNode()
    
    camera_client.get_logger().info("Requesting camera parameters...")
    response = camera_client.send_request()

    if response is not None:
        # Using logger instead of print for better system integration
        camera_client.get_logger().info("Received Camera Parameters:")
        print(response) 
    else:
        camera_client.get_logger().error("Failed to receive response from camera.")

    # Cleanup resources
    camera_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()