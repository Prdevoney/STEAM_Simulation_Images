#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DroneUp(Node):
        def __init__(self):
                super().__init__('move_drone_up')
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

        def move_up(self):
                self.get_logger().info('Starting to publish...')
                cmd = Twist()

                cmd.linear.z = 0.1
                self.x3_pub.publish(cmd)
                self.x4_pub.publish(cmd)
                time.sleep(5)

                cmd.linear.z = 0.0
                self.x3_pub.publish(cmd)
                self.x4_pub.publish(cmd)

                time.sleep(1)
                rclpy.shutdown()

def main():
        rclpy.init()
        node = DroneUp()

        try:
                node.move_up()
        except Exception as e:
                node.get_logger().info('FAILED')

if __name__ == '__main__':
        main()