#!/usr/bin/env python3
"""Stationary-steering traffic-light speed test source.

PASS -> 1 km/h with zero steering, RED -> stop.  The command is published to
the remote controller's AUTO source, not directly to the final vehicle topic.
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


class TrafficSpeedTest(Node):
    def __init__(self):
        super().__init__('traffic_speed_test')
        self.declare_parameter('command_topic', '/cmd_vel/mission_auto')
        self.declare_parameter('traffic_topic', '/vision/traffic_stop')
        self.declare_parameter('speed_kph', 1.0)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('detection_timeout_sec', 0.7)
        self.declare_parameter('require_pass_label', False)

        command_topic = str(self.get_parameter('command_topic').value)
        traffic_topic = str(self.get_parameter('traffic_topic').value)
        speed_kph = float(self.get_parameter('speed_kph').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.speed_mps = speed_kph / 3.6
        self.timeout = float(self.get_parameter('detection_timeout_sec').value)
        self.require_pass_label = bool(
            self.get_parameter('require_pass_label').value)

        self.pub = self.create_publisher(Twist, command_topic, 10)
        self.stop = True
        self.label = 'UNKNOWN'
        self.last_detection = 0.0
        self.create_subscription(Bool, traffic_topic, self.stop_callback, 10)
        self.create_subscription(String, '/vision/traffic_light',
                                 self.label_callback, 10)
        self.create_timer(1.0 / max(1.0, rate), self.publish_command)
        self.get_logger().warning(
            f'Traffic test ready: PASS={speed_kph:.2f} km/h, '
            f'RED=STOP, output={command_topic}')

    def stop_callback(self, msg):
        self.stop = bool(msg.data)
        self.last_detection = time.monotonic()

    def label_callback(self, msg):
        self.label = msg.data.strip().upper()
        self.last_detection = time.monotonic()

    def publish_command(self):
        fresh = time.monotonic() - self.last_detection <= self.timeout
        pass_allowed = (not self.stop and fresh and
                        (not self.require_pass_label or self.label == 'PASS'))
        msg = Twist()
        if pass_allowed:
            msg.linear.x = self.speed_mps
            msg.angular.z = 0.0
        self.pub.publish(msg)

    def stop_command(self):
        self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = TrafficSpeedTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_command()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
