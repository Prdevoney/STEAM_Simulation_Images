#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNode(Node):
        def __init__(self):
                super().__init__('every_five_seconds')
                self.create_timer(2.0, self.print_message)

        def print_message(self):
                self.get_logger().info('Battery is fully charged…')

def main():
        rclpy.init()
        node = MyNode()

        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == '__main__':
        main()