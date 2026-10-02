#!/usr/bin/env python3
"""라바콘이 막은 주차 칸을 피해 반대쪽 경로로 갈아탄다.

주차장이 두 칸으로 나뉘고 그중 하나가 콘으로 막혀 있다. 어느 쪽이 막혔는지는
매번 다르다. 접근 중에 라이다로 콘을 찾아, 비어 있는 칸의 경로 파일을
/t870/set_gps_path 로 한 번 발행한다.

각도 창은 손으로 잰 값이 아니라 경로 파일 두 개의 기하에서 뽑은 것이다.
판정 지점에 섰을 때 각 칸의 안쪽이 차 기준 몇 도에 보이는지 계산하면 나온다.

판정은 한 번만 한다. 콘이 흔들리거나 사람이 지나가 결과가 뒤집히면
경로가 왔다 갔다 하면서 차가 망가진다.

각도 규약: 차 정면 0도, 왼쪽 +, 오른쪽 -.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, UInt32


class ConeParkSelector(Node):
    def __init__(self):
        super().__init__('cone_park_selector')
        defaults = {
            'path_a': '',
            'path_b': '',
            # 이 웨이포인트에 닿으면 판정을 시작한다. 두 칸이 라이다에
            # 다 보이고, 경로가 갈리기 전이어야 한다.
            'decision_waypoint': 8,
            # 각 칸 안쪽이 보이는 각도 창 (도). 경로 기하에서 뽑았다.
            'sector_a_min_deg': -73.0,
            'sector_a_max_deg': -54.0,
            'sector_b_min_deg': -92.0,
            'sector_b_max_deg': -81.0,
            'range_min_m': 3.5,
            'range_max_m': 8.0,
            # 창 안에 이만큼 점이 잡히면 뭔가 있다고 본다.
            'minimum_points': 4,
            # 이 프레임 수만큼 모아 중앙값으로 판정한다. 한 프레임만 보면
            # 잡음에 흔들린다.
            'frames': 8,
            # 라이다에서 차 정면에 해당하는 각도. 런치의
            # lidar_forward_angle_deg 와 같아야 한다.
            'lidar_forward_deg': 180.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.path_a = str(value('path_a'))
        self.path_b = str(value('path_b'))
        self.decision_waypoint = int(value('decision_waypoint'))
        self.sector_a = (float(value('sector_a_min_deg')),
                         float(value('sector_a_max_deg')))
        self.sector_b = (float(value('sector_b_min_deg')),
                         float(value('sector_b_max_deg')))
        self.range_min = float(value('range_min_m'))
        self.range_max = float(value('range_max_m'))
        self.minimum_points = int(value('minimum_points'))
        self.frames = int(value('frames'))
        self.forward_deg = float(value('lidar_forward_deg'))
        if not self.path_a or not self.path_b:
            raise ValueError('path_a and path_b are required')

        self.armed = False
        self.decided = False
        self.counts_a = []
        self.counts_b = []
        self.path_pub = self.create_publisher(String, '/t870/set_gps_path', 10)
        self.choice_pub = self.create_publisher(String, '/t870/parking_choice', 10)
        self.create_subscription(UInt32, '/gps/current_waypoint',
                                 self.on_waypoint, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan,
                                 qos_profile_sensor_data)
        self.get_logger().warning(
            'Cone selector ready. Deciding at waypoint %d. '
            'A sector %.0f~%.0f deg, B sector %.0f~%.0f deg, %.1f~%.1f m'
            % (self.decision_waypoint, self.sector_a[0], self.sector_a[1],
               self.sector_b[0], self.sector_b[1],
               self.range_min, self.range_max))

    def on_waypoint(self, msg):
        if self.decided or self.armed:
            return
        if int(msg.data) >= self.decision_waypoint:
            self.armed = True
            self.get_logger().warning(
                'Reached waypoint %d; looking for the cone' % int(msg.data))

    def count(self, msg, sector):
        low, high = sector
        total = 0
        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            if not self.range_min <= distance <= self.range_max:
                continue
            raw = math.degrees(msg.angle_min + index * msg.angle_increment)
            angle = (raw - self.forward_deg + 180.0) % 360.0 - 180.0
            if low <= angle <= high:
                total += 1
        return total

    def dump_profile(self, msg):
        buckets = {}
        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            if not self.range_min <= distance <= self.range_max:
                continue
            raw = math.degrees(msg.angle_min + index * msg.angle_increment)
            angle = (raw - self.forward_deg + 180.0) % 360.0 - 180.0
            key = int(math.floor(angle / 10.0)) * 10
            near, count = buckets.get(key, (99.0, 0))
            buckets[key] = (min(near, distance), count + 1)
        self.get_logger().warning(
            'Scan profile at decision (%.1f~%.1f m, 10 deg bins):'
            % (self.range_min, self.range_max))
        for key in sorted(buckets, reverse=True):
            near, count = buckets[key]
            inside = ''
            if self.sector_a[0] <= key < self.sector_a[1]:
                inside = '  <- A window'
            elif self.sector_b[0] <= key < self.sector_b[1]:
                inside = '  <- B window'
            self.get_logger().warning(
                '   %+4d deg  %5.2f m  %3d pts%s' % (key, near, count, inside))

    def on_scan(self, msg):
        if not self.armed or self.decided:
            return
        self.counts_a.append(self.count(msg, self.sector_a))
        self.counts_b.append(self.count(msg, self.sector_b))
        if len(self.counts_a) < self.frames:
            return

        # 판정 순간 주변이 어떻게 보였는지 통째로 남긴다. 창을 잘못
        # 잡았을 때 어디로 옮겨야 하는지 이 한 줄들로 바로 나온다.
        self.dump_profile(msg)

        median = lambda values: sorted(values)[len(values) // 2]
        a, b = median(self.counts_a), median(self.counts_b)
        self.decided = True

        if a >= self.minimum_points and b < self.minimum_points:
            blocked, chosen, path = 'A', 'B', self.path_b
        elif b >= self.minimum_points and a < self.minimum_points:
            blocked, chosen, path = 'B', 'A', self.path_a
        elif a > b:
            blocked, chosen, path = 'A', 'B', self.path_b
        elif b > a:
            blocked, chosen, path = 'B', 'A', self.path_a
        else:
            # 구분이 안 된다. 기본 경로를 그대로 둔다. 바꾸지 않는 쪽이
            # 안전하다. 엉뚱한 칸으로 들어가느니 원래 가던 길로 간다.
            self.get_logger().error(
                'Cannot tell the bays apart (A %d pts, B %d pts); keeping the '
                'current path' % (a, b))
            self.choice_pub.publish(String(data='UNKNOWN'))
            return

        self.get_logger().warning(
            'Cone in bay %s (A %d pts, B %d pts); parking in bay %s'
            % (blocked, a, b, chosen))
        self.path_pub.publish(String(data=path))
        self.choice_pub.publish(String(data=chosen))


def main(args=None):
    rclpy.init(args=args)
    node = ConeParkSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
