#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class MoveGreenBot(Node):
        def __init__(self):
                super().__init__('green_velocity_control')
                self.publisher_ = self.create_publisher(
                        JointTrajectory,
                        '/model/RR_velocity_control/joint_trajectory',
                        10
                )
                time.sleep(1)

        def set_velocity(self):
                trajectory = JointTrajectory()
                point = JointTrajectoryPoint()

                # List of joint names
                trajectory.joint_names = ['RR_velocity_control_joint1','RR_velocity_control_joint2']
                # Set list of velocities
                point.velocities = [0.1,-0.6]
                point.time_from_start.sec = 2

                # Add point to trajectory list of points
                trajectory.points.append(point)

                #Publish
                self.publisher_.publish(trajectory)
                self.get_logger().info('Publishing trajectory')
                time.sleep(10)

                point.velocities = [0.0,0.0]
                point.time_from_start.sec = 2

                self.publisher_.publish(trajectory)
                time.sleep(1)

                # Manually shutdown
                self.get_logger().info('Shutting down...')
                rclpy.shutdown()

def main():
        rclpy.init()
        node = MoveGreenBot()

        try:
                node.set_velocity()
        except Exception as e:
                node.get_logger().info('FAILED')

if __name__ == '__main__':
        main()