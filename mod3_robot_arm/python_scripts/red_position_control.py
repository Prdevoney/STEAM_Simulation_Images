#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class MoveRedBot(Node):
        def __init__(self):
                super().__init__('red_position_control')
                self.publisher_ = self.create_publisher(
                        JointTrajectory,
                        '/model/RR_position_control/joint_trajectory',
                        10
                )
                time.sleep(1)

        def set_position(self, index, pos):
                trajectory = JointTrajectory()
                point = JointTrajectoryPoint()

                # List of joint names
                trajectory.joint_names = ['RR_position_control_joint1','RR_position_control_joint2']
                # Default position values
                point.positions = [0.0,0.0]
                # Set position in radians
                point.positions[index] = pos

                point.time_from_start.sec = 2

                # Add point to trajectory list of points
                trajectory.points.append(point)

                #Publish
                self.publisher_.publish(trajectory)
                self.get_logger().info('Publishing trajectory')
                time.sleep(0.5)

def main():
        rclpy.init()
        node = MoveRedBot()

        try:
                node.set_position(0,-0.785)
                time.sleep(3)
                node.set_position(1,1.57)

                # Manually shutdown
                time.sleep(1)
                rclpy.shutdown()
        except Exception as e:
                node.get_logger().info('FAILED')

if __name__ == '__main__':
        main()