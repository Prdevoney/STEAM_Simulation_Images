# Imports
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Twist

class IsDistort(Node):
        def __init__(self):
                super().__init__('is_x4_distorted')
                self.sub = self.create_subscription(
                        CameraInfo,
                        '/world/multicopter/model/X4/link/base_link/sensor/camera_front/camera_info',
                        self.receive_callback,
                        10
                )
                self.pub = self.create_publisher(
                        Twist,
                        '/X4/gazebo/command/twist',
                        10
                )
                time.sleep(1)

        def lift_off_ground(self):
                cmd = Twist()
                cmd.linear.z = 0.2
                self.pub.publish(cmd)
                time.sleep(5)
                cmd.linear.z = 0.0
                self.pub.publish(cmd)

        def yes_move(self):
                self.lift_off_ground()
                cmd = Twist()

                for i in range(2):
                        cmd.linear.z = 0.5
                        self.pub.publish(cmd)
                        time.sleep(3)
                        cmd.linear.z = -0.5
                        self.pub.publish(cmd)
                        time.sleep(6)
                        cmd.linear.z = 0.5
                        self.pub.publish(cmd)
                        time.sleep(3)

                cmd.linear.z = 0.0
                self.pub.publish(cmd)

                time.sleep(1)
                self.get_logger().info('Signal shutdown...')
                rclpy.shutdown()

        def no_move(self):
                self.lift_off_ground()
                cmd = Twist()

                for i in range(2):
                        cmd.linear.y = 0.5
                        self.pub.publish(cmd)
                        time.sleep(3)
                        cmd.linear.y = 0.0
                        self.pub.publish(cmd)
                        time.sleep(1)
                        cmd.linear.y = -0.5
                        self.pub.publish(cmd)
                        time.sleep(6)
                        cmd.linear.y = 0.0
                        self.pub.publish(cmd)
                        time.sleep(1)
                        cmd.linear.y = 0.5
                        self.pub.publish(cmd)
                        time.sleep(3)

                cmd.linear.y = 0.0
                self.pub.publish(cmd)

                time.sleep(1)
                self.get_logger().info('Signal shutdown...')
                rclpy.shutdown()

        # Define callback function
        def receive_callback(self, msg):
                is_distort = False

                arr = msg.d
                for element in arr:
                        if element != 0:
                                self.yes_move()
                                is_distort = True
                                break

                if is_distort == False:
                        self.no_move()

def main():
        rclpy.init()
        node = IsDistort()
        rclpy.spin(node)

if __name__ == '__main__':
        main()