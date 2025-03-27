#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Node class
class PoseArm(Node):
        def __init__(self):
                # Initialize node
                super().__init__('red_arm_pose')
                # Create publisher
                self.publisher_ = self.create_publisher(
                        JointTrajectory,
                        '/model/RR_position_control/joint_trajectory',
                        10
                )
                time.sleep(1)

        def pose_arm(self):
                # Create a joint trajectory and a joint trajectory point
                trajectory = JointTrajectory()
                point = JointTrajectoryPoint()

                # List of joint names
                trajectory.joint_names = ['RR_position_control_joint1','RR_position_control_joint2']
                # List of points in radians
                point.positions = [-2.356, -0.52]
                point.time_from_start.sec = 2

                # Add point to trajectory list of points
                trajectory.points.append(point)
                # Publish
                self.publisher_.publish(trajectory)
                self.get_logger().info('Publishing trajectory')

                # Manually shutdown node
                time.sleep(1)
                rclpy.shutdown()

def main():
        # Initialize node
        rclpy.init()
        node = PoseArm()

        # Function call
        try:
                node.pose_arm()
        except e as Exception:
                node.get_logger().info('FAILED')

if __name__ == '__main__':
        main()