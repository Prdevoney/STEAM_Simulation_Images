#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

class UpdateBattery(Node):
        def __init__(self):
                super().__init__('update_blue_battery')
                self.publisher_ = self.create_publisher(
                        BatteryState,
                        '/model/vehicle_blue/battery/linear_battery/state',
                        10
                )

        def update_battery(self, perc_list):
                msg = BatteryState()

                for perc in perc_list:
                        msg.percentage = perc
                        self.publisher_.publish(msg)

                        percentage = round(perc*100)
                        self.get_logger().info(f'Publishing: {percentage}%')

                        if percentage == 0:
                                self.get_logger().info('Status: DEAD')
                        elif percentage > 0 and percentage <= 20:
                                self.get_logger().info('Status: LOW')
                        elif percentage > 20 and percentage <= 50:
                                self.get_logger().info('Status: OK')
                        elif percentage > 50 and percentage <= 75:
                                self.get_logger().info('Status: GOOD')
                        elif percentage > 75 and percentage <= 99:
                                self.get_logger().info('Status: STRONG')
                        else:
                                self.get_logger().info('Status: FULL')

                self.get_logger().info('Signal shutdown…')
                rclpy.shutdown()

def main():
        rclpy.init()
        pub = UpdateBattery()

        try:
                perc_list = [0.44, 0.78, 1.00, 0.07, 0.00, 0.63]
                pub.update_battery(perc_list)
        except Exception as e:
                pub.get_logger().info('FAILED')

if __name__ == '__main__':
        main()