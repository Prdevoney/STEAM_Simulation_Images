#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SquarePath(Node):
        def __init__(self):
                super().__init__('drone_square_path')
                self.x3_pub = self.create_publisher(
                        Twist,
                        '/X3/gazebo/command/twist',
                        10
                )
                self.x4_pub = self.create_publisher(
                        Twist,
                        '/X4/gazebo/command/twist',
                        10
                )
                time.sleep(1)

        def square_path(self):
                zcmd = Twist()
                ycmd = Twist()
                xcmd = Twist()

                self.get_logger().info('Lifting drones')
                zcmd.linear.z = 0.2
                self.x3_pub.publish(zcmd)
                self.x4_pub.publish(zcmd)
                time.sleep(5)

                zcmd.linear.z = 0.0
                self.x3_pub.publish(zcmd)
                self.x4_pub.publish(zcmd)
                self.get_logger().info('Done lifting drones')

                # Create square path
                self.get_logger().info('Starting square path')
                ycmd.linear.y = 0.5
                xcmd.linear.x = -0.5
                self.x3_pub.publish(ycmd)
                self.x4_pub.publish(xcmd)
                time.sleep(6.5)

                ycmd.linear.y = -0.5
                xcmd.linear.x = -0.5
                self.x3_pub.publish(xcmd)
                self.x4_pub.publish(ycmd)
                time.sleep(6.5)

                ycmd.linear.y = -0.5
                xcmd.linear.x = 0.5
                self.x3_pub.publish(ycmd)
                self.x4_pub.publish(xcmd)
                time.sleep(6.5)

                ycmd.linear.y = 0.5
                xcmd.linear.x = 0.5
                self.x3_pub.publish(xcmd)
                self.x4_pub.publish(ycmd)
                time.sleep(6.5)

                ycmd.linear.y = 0.0
                xcmd.linear.x = 0.0
                self.x3_pub.publish(xcmd)
                self.x4_pub.publish(ycmd)
                self.get_logger().info('Finished path')

                rclpy.shutdown()

def main():
        rclpy.init()
        node = SquarePath()

        try:
                node.square_path()
        except Exception as e:
                node.get_logger().info('FAILED')

if __name__ == '__main__':
        main()