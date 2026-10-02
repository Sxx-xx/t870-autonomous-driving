#!/usr/bin/env python3
"""
T870 Planner Main Node
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry


class T870PlannerNode(Node):
    def __init__(self):
        super().__init__('t870_planner_main')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(
            Odometry, '/mavros/global_position/local', self.odom_callback, 10)
        self.get_logger().info("T870 Planner Main Node Initialized.")

    def odom_callback(self, msg: Odometry):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = T870PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
