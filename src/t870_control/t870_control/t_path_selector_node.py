#!/usr/bin/env python3
"""Two-stage LiDAR path selector for t_p(aa/ba/ab/bb) parking routes."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, UInt32


def vehicle_angle_deg(raw_angle_rad, forward_angle_deg):
    raw_deg = math.degrees(raw_angle_rad)
    return (raw_deg - forward_angle_deg + 180.0) % 360.0 - 180.0


def count_points(scan, forward_angle_deg, angle_min_deg, angle_max_deg,
                 range_min_m, range_max_m):
    count = 0
    nearest = math.inf
    for index, distance in enumerate(scan.ranges):
        if not math.isfinite(distance):
            continue
        if not range_min_m <= distance <= range_max_m:
            continue
        angle = vehicle_angle_deg(
            scan.angle_min + index * scan.angle_increment, forward_angle_deg)
        if angle_min_deg <= angle <= angle_max_deg:
            count += 1
            nearest = min(nearest, distance)
    return count, nearest


class TPathSelector(Node):
    def __init__(self):
        super().__init__('t_path_selector')
        defaults = {
            'stage1_start_waypoint': 7,
            'stage1_decision_waypoint': 9,
            'stage2_start_waypoint': 27,
            'stage2_decision_waypoint': 29,
            'stage1_b_block_waypoint': -1,
            'stage1_a_block_waypoint': -1,
            'stage1_default_prefix': 'b',
            'enable_stage2': True,
            'forward_angle_deg': 180.0,
            'angle_min_deg': -60.0,
            'angle_max_deg': 60.0,
            'range_min_m': 0.05,
            'range_max_m': 4.5,
            'min_points_per_scan': 7,
            'path_aa': '',
            'path_ba': '',
            'path_ab': '',
            'path_bb': '',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.stage1_start = int(self.get_parameter(
            'stage1_start_waypoint').value)
        self.stage1_decision = int(self.get_parameter(
            'stage1_decision_waypoint').value)
        self.stage2_start = int(self.get_parameter(
            'stage2_start_waypoint').value)
        self.stage2_decision = int(self.get_parameter(
            'stage2_decision_waypoint').value)
        self.stage1_b_block_wp = int(self.get_parameter(
            'stage1_b_block_waypoint').value)
        self.stage1_a_block_wp = int(self.get_parameter(
            'stage1_a_block_waypoint').value)
        self.stage1_default_prefix = str(self.get_parameter(
            'stage1_default_prefix').value).strip().lower()
        self.enable_stage2 = bool(self.get_parameter('enable_stage2').value)
        self.forward = float(self.get_parameter('forward_angle_deg').value)
        self.angle_min = float(self.get_parameter('angle_min_deg').value)
        self.angle_max = float(self.get_parameter('angle_max_deg').value)
        self.range_min = float(self.get_parameter('range_min_m').value)
        self.range_max = float(self.get_parameter('range_max_m').value)
        self.min_points = int(self.get_parameter('min_points_per_scan').value)
        self.paths = {
            'aa': str(self.get_parameter('path_aa').value),
            'ba': str(self.get_parameter('path_ba').value),
            'ab': str(self.get_parameter('path_ab').value),
            'bb': str(self.get_parameter('path_bb').value),
        }

        if self.angle_min >= self.angle_max:
            raise ValueError('angle_min_deg must be below angle_max_deg')
        if self.range_min >= self.range_max:
            raise ValueError('range_min_m must be below range_max_m')
        if self.stage1_start > self.stage1_decision:
            raise ValueError('stage1 start waypoint must be <= decision waypoint')
        if self.stage2_start > self.stage2_decision:
            raise ValueError('stage2 start waypoint must be <= decision waypoint')
        if self.stage1_default_prefix not in ('a', 'b'):
            raise ValueError('stage1_default_prefix must be a or b')
        missing = [name for name, path in self.paths.items() if not path]
        if missing:
            raise ValueError('missing path parameters: ' + ', '.join(missing))

        self.current_waypoint = 0
        self.stage1_done = False
        self.stage2_done = False
        self.stage1_seen = False
        self.stage2_seen = False
        self.stage1_counts = []
        self.stage1_b_block_counts = []
        self.stage1_a_block_counts = []
        self.stage1_b_block_seen = False
        self.stage1_a_block_seen = False
        self.stage2_counts = []
        self.prefix = None
        self.stage1_slot_mode = (
            self.stage1_b_block_wp >= 0 and self.stage1_a_block_wp >= 0)

        self.path_pub = self.create_publisher(
            String, '/t870/set_gps_path', 10)
        self.result_pub = self.create_publisher(
            String, '/parking/t_path_choice', 10)
        self.create_subscription(
            UInt32, '/gps/current_waypoint', self.on_waypoint, 10)
        self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.get_logger().warning(
            'T-path selector ready: stage1 wp '
            f'{self.stage1_start}->{self.stage1_decision}, stage2 '
            f'{"on" if self.enable_stage2 else "off"} wp '
            f'{self.stage2_start}->{self.stage2_decision}, angle '
            f'{self.angle_min:.1f}..{self.angle_max:.1f} deg, range '
            f'{self.range_min:.2f}..{self.range_max:.2f} m')

    def publish_choice(self, key, reason):
        self.path_pub.publish(String(data=self.paths[key]))
        self.result_pub.publish(String(data=key))
        self.get_logger().warning(
            f'T-PATH SELECT {key}: {reason}; path={self.paths[key]}')

    def decide_stage1(self, reason):
        if self.stage1_done:
            return
        self.stage1_done = True
        if self.stage1_slot_mode:
            if self.stage1_b_block_seen and not self.stage1_a_block_seen:
                self.prefix = 'a'
            elif self.stage1_a_block_seen and not self.stage1_b_block_seen:
                self.prefix = 'b'
            elif self.stage1_b_block_seen and self.stage1_a_block_seen:
                b_max = max(self.stage1_b_block_counts or [0])
                a_max = max(self.stage1_a_block_counts or [0])
                self.prefix = 'a' if b_max >= a_max else 'b'
            else:
                self.prefix = self.stage1_default_prefix
            self.publish_choice(
                self.prefix + 'a',
                f'{reason}; b_block_detected={self.stage1_b_block_seen}; '
                f'a_block_detected={self.stage1_a_block_seen}; '
                f'b_counts={self.stage1_b_block_counts}; '
                f'a_counts={self.stage1_a_block_counts}')
        else:
            self.prefix = 'a' if self.stage1_seen else 'b'
            self.publish_choice(
                self.prefix + 'a',
                f'{reason}; detected={self.stage1_seen}; '
                f'counts={self.stage1_counts}')

    def decide_stage2(self, reason):
        if self.stage2_done:
            return
        self.stage2_done = True
        suffix = 'a' if self.stage2_seen else 'b'
        key = self.prefix + suffix
        self.publish_choice(
            key,
            f'{reason}; detected={self.stage2_seen}; '
            f'counts={self.stage2_counts}')

    def on_waypoint(self, msg):
        self.current_waypoint = int(msg.data)
        if (not self.stage1_done
                and self.current_waypoint > self.stage1_decision):
            self.decide_stage1(
                f'stage1 passed decision wp {self.current_waypoint}')
        if (self.enable_stage2 and self.stage1_done and not self.stage2_done
                and self.current_waypoint > self.stage2_decision):
            self.decide_stage2(
                f'stage2 passed decision wp {self.current_waypoint}')

    def on_scan(self, scan):
        points, nearest = count_points(
            scan, self.forward, self.angle_min, self.angle_max,
            self.range_min, self.range_max)
        hit = points >= self.min_points
        nearest_text = 'none' if not math.isfinite(nearest) else f'{nearest:.2f}m'

        if self.stage1_slot_mode and not self.stage1_done:
            if self.current_waypoint == self.stage1_b_block_wp:
                self.stage1_b_block_counts.append(points)
                if hit:
                    self.stage1_b_block_seen = True
                self.get_logger().info(
                    f'stage1 b-block wp={self.current_waypoint}: '
                    f'points={points}, nearest={nearest_text}, '
                    f'detected={self.stage1_b_block_seen}')
            if self.current_waypoint == self.stage1_a_block_wp:
                self.stage1_a_block_counts.append(points)
                if hit:
                    self.stage1_a_block_seen = True
                self.get_logger().info(
                    f'stage1 a-block wp={self.current_waypoint}: '
                    f'points={points}, nearest={nearest_text}, '
                    f'detected={self.stage1_a_block_seen}')
            if self.current_waypoint >= self.stage1_decision:
                self.decide_stage1(
                    f'stage1 slot decision wp {self.current_waypoint}')

        if (not self.stage1_slot_mode and not self.stage1_done
                and self.stage1_start <= self.current_waypoint
                <= self.stage1_decision):
            self.stage1_counts.append(points)
            if hit:
                self.stage1_seen = True
            self.get_logger().info(
                f'stage1 wp={self.current_waypoint}: points={points}, '
                f'nearest={nearest_text}, detected={self.stage1_seen}')
        if (self.enable_stage2 and self.stage1_done and not self.stage2_done
                and self.stage2_start <= self.current_waypoint
                <= self.stage2_decision):
            self.stage2_counts.append(points)
            if hit:
                self.stage2_seen = True
            self.get_logger().info(
                f'stage2 wp={self.current_waypoint}: points={points}, '
                f'nearest={nearest_text}, detected={self.stage2_seen}')

def main(args=None):
    rclpy.init(args=args)
    node = TPathSelector()
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
