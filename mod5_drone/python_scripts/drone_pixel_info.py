#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class PixelInfo(Node):
        def __init__(self):
                super().__init__('drone_pixel_info')
                self.subscription = self.create_subscription(
                        CameraInfo,
                        '/world/multicopter/model/X4/link/base_link/sensor/camera_front/camera_info',
                        self.receive_callback,
                        10
                )
                self.subscription

        # Define function
        def receive_callback(self, msg):
                # Find number of rows and columns
                height = msg.height
                width = msg.width
                self.get_logger().info(f'Num of rows: {height}')
                self.get_logger().info (f'Num of col: {width}')

                # Calculate number of pixels
                total = height * width
                self.get_logger().info (f'Total num of pixels: {total}')

                # Force shutdown
                self.get_logger().info('Shutting down node…')
                rclpy.shutdown()

def main():
        rclpy.init()
        node = PixelInfo()
        rclpy.spin(node)

if __name__ == '__main__':
        main()