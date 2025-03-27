#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Alternate(Node):
        def __init__(self):
                super().__init__('alternate_drones')
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

        def alternate_drones(self):
                cmd = Twist()
                self.get_logger().info('Publishing series of commands...')

                for i in range(2):
                        cmd.linear.z = 0.2
                        self.x3_pub.publish(cmd)
                        time.sleep(5)

                        cmd.linear.z = -0.2
                        self.x3_pub.publish(cmd)
                        time.sleep(5)

                        cmd.linear.z = 0.0
                        self.x3_pub.publish(cmd)

                        cmd.linear.z = 0.2
                        self.x4_pub.publish(cmd)
                        time.sleep(5)

                        cmd.linear.z = -0.2
                        self.x4_pub.publish(cmd)
                        time.sleep(5)

                        cmd.linear.z = 0.0
                        self.x4_pub.publish(cmd)

                self.get_logger().info('Shutting down node...')
                rclpy.shutdown()

def main():
        rclpy.init()
        node = Alternate()

        try:
                node.alternate_drones()
        except Exception as e:
                node.get_logger().info('FAILED')

if __name__ == '__main__':
        main()