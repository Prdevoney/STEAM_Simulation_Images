#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class MoveBlueBot(Node):
        def __init__(self):
                super().__init__('blue_effort_control')
                self.publisher_ = self.create_publisher(
                        JointTrajectory,
                        '/custom_topic_effort_control',
                        10
                )
                time.sleep(1)

        def set_effort(self):
                trajectory = JointTrajectory()
                point = JointTrajectoryPoint()

                # List of joint names
                trajectory.joint_names = ['RR_effort_control_joint1','RR_effort_control_joint2']
                # Set list of effort
                point.effort = [-0.5,0.5]
                point.time_from_start.sec = 2

                # Add point to trajectory list of points
                trajectory.points.append(point)

                #Publish
                self.publisher_.publish(trajectory)
                self.get_logger().info('Publishing trajectory')
                time.sleep(5)

                # Stop all movements
                point.effort = [0.0,0.0]
                point.time_from_start.sec = 2

                # Publish
                self.publisher_.publish(trajectory)
                time.sleep(1)

                # Manually shutdown
                self.get_logger().info('Shutting down…')
                rclpy.shutdown()

def main():
        # Create node
        rclpy.init()
        node = MoveBlueBot()

        try:
                # Function call
                node.set_effort()
        except Exception as e:
                node.get_logger().info('FAILED')

if __name__ == '__main__':
        main()