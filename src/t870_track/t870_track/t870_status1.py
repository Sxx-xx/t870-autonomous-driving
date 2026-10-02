#!/usr/bin/env python3
"""
T870 Status Monitoring Node (Track)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class T870Status1Node(Node):
    def __init__(self):
        super().__init__('t870_status1')
        self.sub_steer = self.create_subscription(
            Float32, '/t870/steering_angle', self.steer_cb, 10)
        self.get_logger().info("T870 Status Monitoring Node 1 Initialized.")

    def steer_cb(self, msg: Float32):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = T870Status1Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
