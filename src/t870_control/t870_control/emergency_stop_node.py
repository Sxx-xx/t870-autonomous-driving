#!/usr/bin/env python3
"""동적장애물 정지. 정면에서 한 번 잡으면 정해진 시간만 서고 복귀한다.

대회 규정 미션이다. 정면에 장애물이 나타나면 서고, 정해진 시간이 지나면
다시 GPS 추종으로 돌아간다.

  구역 진입 -> 정면 감시 -> 장애물 확인 -> 그 순간부터 라이다 무시
    -> stop_duration_sec 동안 정지 -> GPS 추종 복귀 -> 감시 종료

'치워질 때까지' 가 아니라 '정해진 시간만' 인 이유는, 장애물을 치우는
사람이 정면 섹터 안에 들어와 있으면 재출발이 막히기 때문이다.

정지 후 감시를 끝내는 이유는, 이후 평행주차 구간에서 정면 3 m 안에
주차 칸 벽이나 앞차가 들어오면 그걸 장애물로 잡아 차가 서서 안 나가기
때문이다. 단, 장애물이 끝내 나타나지 않으면 감시가 꺼지지 않으므로
estop_ranges 의 끝 WP 가 여전히 안전장치로 필요하다.

각도는 mission_common.scan_points 가 raw 스캔 각도를 그대로 쓰므로
front_center_deg 도 raw 기준이다. 이 프로젝트는 raw 180도가 차량 정면이다
(주차 선택기/섹터 필터의 forward_angle_deg 와 같은 규약).
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from .mission_common import scan_points


class EmergencyStop(Node):
    def __init__(self):
        super().__init__('emergency_stop_node')
        defaults = {
            # raw 180 deg = 차량 정면. scan_points 는 raw 각도를 쓴다.
            'front_center_deg': 180.0,
            'detect_angle_half_deg': 30.0,
            'detect_distance_m': 3.0,
            'detect_min_points': 3,
            # 감지 후 정지 유지 시간. 이 시간이 지나면 라이다에 아직
            # 장애물이 보여도 복귀한다. 치우는 사람이 앞에 있을 수 있다.
            'stop_duration_sec': 3.0,
            # 한 번 서고 나면 감시를 끝낸다. false 로 두면 구역 안에서
            # 몇 번이든 다시 선다.
            'disable_after_stop': True,
            'scan_timeout_sec': 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.active_zone = False
        self.finished = False
        self.hold_until = 0.0
        self.was_stopping = False
        self.last_scan = 0.0

        self.stop_pub = self.create_publisher(Bool, '/t870/mission_estop', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel/estop', 10)
        # 감시가 끝났음을 알린다. 미션 매니저는 이 구간(WP531~588) 전체를
        # 미션 속도로 묶는데, 장애물은 보통 앞쪽에서 처리되고 그 뒤로는
        # 라이다를 아예 안 본다. 그런데도 남은 50 여 m 를 2 km/h 로 기어가
        # 복귀가 한참 걸렸다. 끝난 시점을 알려 주면 거기서부터 올릴 수 있다.
        self.done_pub = self.create_publisher(
            Bool, '/t870/dynamic_obstacle_done', 10)
        self.create_subscription(
            String, '/t870/active_mission', self.on_mission, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_timer(0.05, self.tick)

        self.get_logger().warning(
            'Dynamic obstacle stop ready: raw %.1f deg +/-%.1f, %.2f m, '
            '%d points, stop %.1f s'
            % (float(self.p('front_center_deg')),
               float(self.p('detect_angle_half_deg')),
               float(self.p('detect_distance_m')),
               int(self.p('detect_min_points')),
               float(self.p('stop_duration_sec'))))

    def p(self, name):
        return self.get_parameter(name).value

    def on_mission(self, msg):
        in_zone = (msg.data == 'ESTOP')
        if in_zone and not self.active_zone:
            self.get_logger().warning(
                'Dynamic obstacle zone entered; watching front sector')
        self.active_zone = in_zone

    def watching(self):
        """지금 라이다를 보고 있는가."""
        return (self.active_zone and not self.finished
                and self.hold_until == 0.0)

    def on_scan(self, msg):
        self.last_scan = time.monotonic()
        # 한 번 잡은 뒤에는 라이다를 보지 않는다. 치우는 사람이 섹터에
        # 들어와도 정지가 연장되지 않아야 한다.
        if not self.watching():
            return
        points = scan_points(
            msg,
            float(self.p('front_center_deg')),
            float(self.p('detect_angle_half_deg')),
            max_range=float(self.p('detect_distance_m')))
        if len(points) < int(self.p('detect_min_points')):
            return
        nearest = min(distance for _angle, distance in points)
        duration = float(self.p('stop_duration_sec'))
        self.hold_until = self.last_scan + duration
        self.get_logger().warning(
            'DYNAMIC OBSTACLE: %d points, nearest=%.2f m; stopping %.1f s '
            '(LiDAR ignored from now)' % (len(points), nearest, duration))

    def tick(self):
        now = time.monotonic()
        holding = self.hold_until > 0.0 and now < self.hold_until
        if self.hold_until > 0.0 and not holding and not self.finished:
            self.hold_until = 0.0
            if bool(self.p('disable_after_stop')):
                self.finished = True
                self.get_logger().warning(
                    'Dynamic obstacle handled; resuming GPS tracking and '
                    'front monitoring is now off')
            else:
                self.get_logger().warning(
                    'Dynamic obstacle hold complete; resuming GPS tracking')
        # 감시 중인데 스캔이 끊기면 판단을 못 한다. 그때는 선다. 감시를
        # 끝낸 뒤에는 라이다를 쓰지 않으므로 끊겨도 상관없다.
        stale = (self.watching()
                 and now - self.last_scan > float(self.p('scan_timeout_sec')))
        stop = holding or stale
        self.was_stopping = stop
        self.stop_pub.publish(Bool(data=stop))
        self.done_pub.publish(Bool(data=bool(self.finished)))
        if stop:
            self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_pub.publish(Bool(data=True))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
