#!/usr/bin/env python3
"""Publish only LaserScan returns inside the configured vehicle-space sector."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def vehicle_angle_deg(raw_angle_rad, forward_angle_deg):
    raw_deg = math.degrees(raw_angle_rad)
    return (raw_deg - forward_angle_deg + 180.0) % 360.0 - 180.0


class LidarSectorFilter(Node):
    def __init__(self):
        super().__init__('lidar_sector_filter')
        defaults = {
            'input_topic': '/scan',
            'output_topic': '/parking/scan_window',
            'forward_angle_deg': 180.0,
            'angle_min_deg': -60.0,
            'angle_max_deg': 60.0,
            'range_min_m': 0.05,
            'range_max_m': 4.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.forward = float(self.get_parameter('forward_angle_deg').value)
        self.angle_min = float(self.get_parameter('angle_min_deg').value)
        self.angle_max = float(self.get_parameter('angle_max_deg').value)
        self.range_min = float(self.get_parameter('range_min_m').value)
        self.range_max = float(self.get_parameter('range_max_m').value)
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        if self.angle_min >= self.angle_max or self.range_min >= self.range_max:
            raise ValueError('LiDAR sector limits are invalid')

        self.publisher = self.create_publisher(
            LaserScan, output_topic, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, input_topic, self.on_scan, qos_profile_sensor_data)
        self.get_logger().warning(
            f'LiDAR RViz window: angle={self.angle_min:.1f}..'
            f'{self.angle_max:.1f} deg, range={self.range_min:.1f}..'
            f'{self.range_max:.1f} m -> {output_topic}')

    def on_scan(self, scan):
        filtered = LaserScan()
        filtered.header = scan.header
        filtered.angle_min = scan.angle_min
        filtered.angle_max = scan.angle_max
        filtered.angle_increment = scan.angle_increment
        filtered.time_increment = scan.time_increment
        filtered.scan_time = scan.scan_time
        filtered.range_min = scan.range_min
        filtered.range_max = scan.range_max
        filtered.intensities = list(scan.intensities)
        ranges = []
        for index, distance in enumerate(scan.ranges):
            angle = vehicle_angle_deg(
                scan.angle_min + index * scan.angle_increment, self.forward)
            visible = (
                math.isfinite(distance)
                and self.angle_min <= angle <= self.angle_max
                and self.range_min <= distance <= self.range_max)
            ranges.append(distance if visible else math.inf)
        filtered.ranges = ranges
        self.publisher.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSectorFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
