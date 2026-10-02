#!/usr/bin/env python3
"""
T870 Status Monitoring Node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist


class T870StatusNode(Node):
    def __init__(self):
        super().__init__('t870_status')
        self.sub_steer = self.create_subscription(
            Float32, '/t870/steering_angle', self.steer_callback, 10)
        self.sub_speed = self.create_subscription(
            Float32, '/t870/current_speed', self.speed_callback, 10)

        self.current_steer = 0.0
        self.current_speed = 0.0
        self.get_logger().info("T870 Status Monitoring Node Initialized.")

    def steer_callback(self, msg: Float32):
        self.current_steer = msg.data

    def speed_callback(self, msg: Float32):
        self.current_speed = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = T870StatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
