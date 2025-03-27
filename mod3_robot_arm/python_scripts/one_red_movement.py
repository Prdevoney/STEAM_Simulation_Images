#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

# Import joint trajectory and joint trajectory point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Node class
class PublisherNode(Node):
        def __init__(self):
                # Initialize node
                super().__init__('one_red_movement')
                # Create publisher
                self.publisher_ = self.create_publisher(
                        JointTrajectory,
                        '/model/RR_position_control/joint_trajectory',
                        10
                )
                time.sleep(1)

        def move_one_point(self, p_data, time_to_move):
                # Create a joint trajectory and a joint trajectory point
                trajectory = JointTrajectory()
                point = JointTrajectoryPoint()

                # List of joint names
                trajectory.joint_names = ['RR_position_control_joint1','RR_position_control_joint2']
                # List of points in radians
                point.positions = p_data
                point.time_from_start.sec = int(time_to_move)

                # Add point to trajectory list of points
                trajectory.points.append(point)

                # Publish trajectory
                self.publisher_.publish(trajectory)
                self.get_logger().info(f'Publishing point: {p_data}')

def main():
        rclpy.init()
        pub = PublisherNode()

        points_data = [
                [0.0,0.0],
                [0.52,0.785],
                [-0.52,1.57],
                [1.047,0.0],
                [0.0,-0.785],
                [0.52,0.0]
        ]

        time_to_move = 2.0
        for p_data in points_data:
                pub.move_one_point(p_data, time_to_move)
                time.sleep(1)
                time_to_move += 2.0

        # Shutdown node after whole trajectory is published
        time.sleep(1)
        rclpy.shutdown()

if __name__ == '__main__':
        main()