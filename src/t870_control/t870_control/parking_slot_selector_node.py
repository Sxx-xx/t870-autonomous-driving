#!/usr/bin/env python3
"""Select a parking path by detecting only parking zone B.

The mission guarantees that exactly one of two parking slots is blocked.  At
the decision pose the selector inspects the forward 120-degree sector
(-60..+60 degrees) out to 4.5 metres in vehicle coordinates. Detecting a
dense return selects the zone-A path; otherwise the zone-B path is selected.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, UInt32


def vehicle_angle_deg(raw_angle_rad, forward_angle_deg):
    """Convert a LaserScan angle to vehicle coordinates (-180..180)."""
    raw_deg = math.degrees(raw_angle_rad)
    return (raw_deg - forward_angle_deg + 180.0) % 360.0 - 180.0


def count_points_in_window(scan, forward_angle_deg, angle_min_deg,
                           angle_max_deg, range_min_m, range_max_m):
    """Count finite scan returns inside the calibrated zone-B window."""
    count = 0
    nearest = math.inf
    for index, distance in enumerate(scan.ranges):
        if not math.isfinite(distance):
            continue
        if not range_min_m <= distance <= range_max_m:
            continue
        angle = vehicle_angle_deg(
            scan.angle_min + index * scan.angle_increment,
            forward_angle_deg)
        if angle_min_deg <= angle <= angle_max_deg:
            count += 1
            nearest = min(nearest, distance)
    return count, nearest


class ParkingSlotSelector(Node):
    def __init__(self):
        super().__init__('parking_slot_selector')

        defaults = {
            'observation_start_waypoint': -1,
            'decision_waypoint': 5,
            'observation_scans': 5,
            'blocked_votes_required': 2,
            'decide_after_observation_scans': True,
            'min_points_per_scan': 7,
            'forward_angle_deg': 180.0,
            'zone_b_angle_min_deg': -60.0,
            'zone_b_angle_max_deg': 60.0,
            'zone_b_range_min_m': 0.05,
            'zone_b_range_max_m': 4.5,
            'slot_a_path': '',
            'slot_b_path': '',
            'publish_path': True,
            'choice_topic': '',
            # 선택기를 두 개(T주차/평행) 띄우므로 결과 토픽도 나눠야 한다.
            # 잠근 뒤 계속 재발행하기 때문에, 같은 토픽을 쓰면 두 값이
            # 영원히 뒤섞여 ros2 topic echo 로 볼 때 오판하게 된다.
            'result_topic': '/parking/blocked_zone',
            # 감지됐을 때 choice_topic 으로 내보낼 값. 반대 경우는 나머지
            # 값이 나간다. 기본 1 은 'zone A 선택 = 1번' 이라는 원래
            # 규약이다. 검사 창이 1번 칸을 보는 미션에서는 감지가 곧
            # '1번 칸이 막혔다' 는 뜻이므로 2 로 둔다.
            'choice_when_blocked': 1,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.decision_waypoint = int(
            self.get_parameter('decision_waypoint').value)
        self.observation_start_waypoint = int(
            self.get_parameter('observation_start_waypoint').value)
        if self.observation_start_waypoint < 0:
            self.observation_start_waypoint = self.decision_waypoint
        self.observation_scans = max(
            1, int(self.get_parameter('observation_scans').value))
        self.blocked_votes_required = max(
            1, int(self.get_parameter('blocked_votes_required').value))
        self.decide_after_observation_scans = bool(
            self.get_parameter('decide_after_observation_scans').value)
        self.min_points = max(
            1, int(self.get_parameter('min_points_per_scan').value))
        self.forward_angle_deg = float(
            self.get_parameter('forward_angle_deg').value)
        self.angle_min_deg = float(
            self.get_parameter('zone_b_angle_min_deg').value)
        self.angle_max_deg = float(
            self.get_parameter('zone_b_angle_max_deg').value)
        self.range_min_m = float(
            self.get_parameter('zone_b_range_min_m').value)
        self.range_max_m = float(
            self.get_parameter('zone_b_range_max_m').value)
        self.slot_a_path = str(self.get_parameter('slot_a_path').value)
        self.slot_b_path = str(self.get_parameter('slot_b_path').value)
        self.publish_path = bool(self.get_parameter('publish_path').value)
        self.choice_topic = str(self.get_parameter('choice_topic').value)
        self.choice_when_blocked = int(
            self.get_parameter('choice_when_blocked').value)
        if self.choice_when_blocked not in (1, 2):
            raise ValueError('choice_when_blocked must be 1 or 2')

        if self.angle_min_deg >= self.angle_max_deg:
            raise ValueError('zone B angle minimum must be below maximum')
        if self.range_min_m >= self.range_max_m:
            raise ValueError('zone B range minimum must be below maximum')
        if self.observation_start_waypoint > self.decision_waypoint:
            raise ValueError(
                'observation_start_waypoint must be <= decision_waypoint')
        if not self.slot_a_path or not self.slot_b_path:
            raise ValueError('slot_a_path and slot_b_path must both be set')
        if self.blocked_votes_required > self.observation_scans:
            raise ValueError(
                'blocked_votes_required cannot exceed observation_scans')

        self.active = False
        self.locked = False
        # 결정을 한 번만 쏘면 그 메시지를 놓쳤을 때 복구할 방법이 없다.
        # 미션 매니저는 못 받으면 기본값(choice 1)을 그대로 쓰므로 주차
        # 경로가 통째로 틀린다. 잠근 뒤에도 계속 알린다.
        # (ox_signal_detector 는 원래 타이머로 계속 쏜다.)
        self.decision = None
        self.scans_seen = 0
        self.blocked_votes = 0
        self.point_counts = []

        self.path_pub = self.create_publisher(
            String, '/t870/set_gps_path', 10)
        self.result_pub = self.create_publisher(
            String, str(self.get_parameter('result_topic').value), 10)
        self.choice_pub = None
        if self.choice_topic:
            self.choice_pub = self.create_publisher(
                UInt32, self.choice_topic, 10)
        self.create_subscription(
            UInt32, '/gps/current_waypoint', self.on_waypoint, 10)
        self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        self.get_logger().warning(
            'Parking selector ready: inspect parking zone B from waypoint '
            f'{self.observation_start_waypoint} to {self.decision_waypoint}; '
            f'angle={self.angle_min_deg:.1f}..'
            f'{self.angle_max_deg:.1f} deg, range={self.range_min_m:.2f}..'
            f'{self.range_max_m:.2f} m')

        self.create_timer(0.2, self.republish_decision)

    def on_waypoint(self, msg):
        if self.locked:
            return
        waypoint = int(msg.data)
        if self.active and waypoint > self.decision_waypoint:
            self.finalize_decision(f'passed decision waypoint {waypoint}')
            return
        if (not self.active
                and self.observation_start_waypoint <= waypoint
                <= self.decision_waypoint):
            self.active = True
            self.get_logger().warning(
                f'Parking observation started at waypoint {waypoint}')

    def on_scan(self, scan):
        if not self.active or self.locked:
            return

        points, nearest = count_points_in_window(
            scan,
            self.forward_angle_deg,
            self.angle_min_deg,
            self.angle_max_deg,
            self.range_min_m,
            self.range_max_m)
        blocked = points >= self.min_points
        self.scans_seen += 1
        self.blocked_votes += int(blocked)
        self.point_counts.append(points)

        nearest_text = 'none' if not math.isfinite(nearest) else f'{nearest:.2f}m'
        self.get_logger().info(
            f'zone B sample {self.scans_seen}/{self.observation_scans}: '
            f'{points} points, nearest={nearest_text}, blocked={blocked}')

        if (not self.decide_after_observation_scans
                or self.scans_seen < self.observation_scans):
            return

        self.finalize_decision('observation scan count reached')

    def republish_decision(self):
        """잠근 결정을 계속 알린다. 경로는 제외한다.

        경로까지 다시 보내면 follower 가 최근접 색인을 다시 잡아 버린다.
        여기서 되풀이해도 되는 것은 '어느 칸이냐' 하는 결과뿐이다.
        """
        if self.decision is None:
            return
        blocked_zone, choice = self.decision
        self.result_pub.publish(String(data=blocked_zone))
        if self.choice_pub is not None:
            self.choice_pub.publish(UInt32(data=choice))

    def finalize_decision(self, reason):
        if self.locked:
            return
        zone_b_blocked = self.blocked_votes >= self.blocked_votes_required
        blocked_zone = 'B' if zone_b_blocked else 'A'
        selected_zone = 'A' if zone_b_blocked else 'B'
        selected_path = self.slot_a_path if zone_b_blocked else self.slot_b_path

        # Lock before publishing: changing the follower path resets its nearest
        # waypoint and must never trigger a second decision.
        # choice 는 zone 라벨이 아니라 '감지했는가' 에서 직접 뽑는다.
        # 검사 창이 어느 칸을 보느냐는 미션마다 다르므로 zone A = 1 을
        # 고정하면 매핑을 뒤집을 방법이 없다.
        choice = (self.choice_when_blocked if zone_b_blocked
                  else 3 - self.choice_when_blocked)

        self.locked = True
        self.active = False
        self.decision = (blocked_zone, choice)
        self.result_pub.publish(String(data=blocked_zone))
        if self.publish_path:
            # 경로는 딱 한 번만 보낸다. follower 는 경로를 받을 때마다
            # 최근접 색인을 다시 잡으므로 반복해 보내면 안 된다.
            self.path_pub.publish(String(data=selected_path))
        if self.choice_pub is not None:
            self.choice_pub.publish(UInt32(data=choice))
        self.get_logger().warning(
            f'PARKING DECISION LOCKED: reason={reason}, '
            f'detected={zone_b_blocked}, choice={choice}, '
            f'blocked_zone={blocked_zone}, '
            f'select_zone={selected_zone}, votes={self.blocked_votes}/'
            f'{self.scans_seen}, points={self.point_counts}, '
            f'path={selected_path}')


def main(args=None):
    rclpy.init(args=args)
    node = ParkingSlotSelector()
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
