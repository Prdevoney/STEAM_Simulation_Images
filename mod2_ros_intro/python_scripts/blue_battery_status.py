#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

class BatteryInfo(Node):
        def __init__(self):
                super().__init__('blue_battery_info')
                self.sub = self.create_subscription(
                        BatteryState,
                        '/model/vehicle_blue/battery/linear_battery/state',
                        self.receive_message,
                        10
                )
                self.sub

        def receive_message(self, msg):
                self.get_logger().info(f'Voltage: {msg.voltage}')
                self.get_logger().info(f'Charge: {msg.charge}')
                self.get_logger().info(f'Capacity: {msg.capacity}')
                self.get_logger().info(f'Percentage: {msg.percentage}')
                self.get_logger().info(f'Power Status: {msg.power_supply_status}')

                self.get_logger().info('Signal shutdown...')
                rclpy.shutdown()

def main():
        rclpy.init()
        node = BatteryInfo()
        rclpy.spin(node)

if __name__ == '__main__':
        main()