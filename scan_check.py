#!/usr/bin/env python3
"""차를 판정 지점에 세우고 라이다에 뭐가 어디에 보이는지 잰다.

콘 판정 노드의 각도 창과 거리 문턱을 추측으로 넣으면 현장에서 안 맞는다.
콘을 A 쪽에 뒀을 때와 B 쪽에 뒀을 때를 각각 찍어 비교하면, 어느 각도에서
몇 m 에 몇 점이 잡히는지가 숫자로 나온다.

차 정면이 0 도, 왼쪽이 +, 오른쪽이 - 다. 라이다 원점이 정면과 다르면
--forward 로 맞춘다 (런치의 lidar_forward_angle_deg 와 같은 값).

ROS 스택이 떠 있어야 한다 (/scan 이 필요하다).

쓰는 법:
    python3 scan_check.py                      # 실시간 표
    python3 scan_check.py --save cone_left     # 스냅샷 저장
    python3 scan_check.py --compare cone_left cone_right
"""
import argparse
import math
import os
import sys

SNAPSHOT_DIR = '/tmp/t870_scan'


def sector_table(ranges, angle_min, angle_increment, forward_deg,
                 max_range, sector_deg):
    """각도 구간마다 최소거리와 점 개수를 센다."""
    buckets = {}
    for index, distance in enumerate(ranges):
        if not math.isfinite(distance) or distance <= 0.05:
            continue
        if distance > max_range:
            continue
        raw = math.degrees(angle_min + index * angle_increment)
        # 차 정면 기준으로 옮기고 -180~180 으로 접는다.
        angle = (raw - forward_deg + 180.0) % 360.0 - 180.0
        key = int(math.floor(angle / sector_deg)) * sector_deg
        near, count = buckets.get(key, (99.0, 0))
        buckets[key] = (min(near, distance), count + 1)
    return buckets


def show(buckets, sector_deg, title=''):
    if title:
        print(title)
    print('   각도구간        최소거리   점수   막대')
    for key in sorted(buckets, reverse=True):
        near, count = buckets[key]
        side = '왼쪽' if key >= 0 else '오른쪽'
        print('  %+4d ~ %+4d도  %6.2f m  %4d   %s  %s'
              % (key, key + sector_deg, near, count, '#' * min(count, 30), side))


def save(buckets, name, sector_deg):
    os.makedirs(SNAPSHOT_DIR, exist_ok=True)
    path = os.path.join(SNAPSHOT_DIR, name + '.csv')
    with open(path, 'w', encoding='utf-8') as stream:
        stream.write('angle_deg,nearest_m,points\n')
        for key in sorted(buckets):
            near, count = buckets[key]
            stream.write('%d,%.3f,%d\n' % (key, near, count))
    print('저장: %s' % path)


def compare(left_name, right_name):
    import csv as csvmod
    def read(name):
        path = os.path.join(SNAPSHOT_DIR, name + '.csv')
        if not os.path.isfile(path):
            raise SystemExit('없다: %s' % path)
        return {int(r['angle_deg']): (float(r['nearest_m']), int(r['points']))
                for r in csvmod.DictReader(open(path, encoding='utf-8'))}
    a, b = read(left_name), read(right_name)
    print('  각도      %-16s %-16s  차이' % (left_name, right_name))
    for key in sorted(set(a) | set(b), reverse=True):
        na, ca = a.get(key, (99.0, 0))
        nb, cb = b.get(key, (99.0, 0))
        mark = ''
        if abs(ca - cb) >= 3 or abs(na - nb) >= 0.5:
            mark = '  <- 여기가 갈린다'
        print('  %+4d도   %5.2f m %3d점    %5.2f m %3d점 %s'
              % (key, na, ca, nb, cb, mark))


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--forward', type=float, default=180.0,
                        help='라이다에서 차 정면에 해당하는 각도 (기본 180)')
    parser.add_argument('--max-range', type=float, default=6.0)
    parser.add_argument('--sector', type=float, default=15.0)
    parser.add_argument('--save', metavar='NAME')
    parser.add_argument('--compare', nargs=2, metavar=('A', 'B'))
    arguments = parser.parse_args()

    if arguments.compare:
        compare(*arguments.compare)
        return

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    class ScanCheck(Node):
        def __init__(self):
            super().__init__('scan_check')
            self.latest = None
            self.create_subscription(LaserScan, '/scan', self.on_scan,
                                     qos_profile_sensor_data)

        def on_scan(self, msg):
            self.latest = msg

    rclpy.init()
    node = ScanCheck()
    try:
        # 스냅샷이면 몇 프레임 받고 끝낸다. 아니면 계속 새로 그린다.
        wanted = 1 if arguments.save else 10 ** 9
        shown = 0
        while rclpy.ok() and shown < wanted:
            rclpy.spin_once(node, timeout_sec=1.0)
            if node.latest is None:
                continue
            msg, node.latest = node.latest, None
            buckets = sector_table(
                msg.ranges, msg.angle_min, msg.angle_increment,
                arguments.forward, arguments.max_range, arguments.sector)
            if arguments.save:
                show(buckets, arguments.sector,
                     '스냅샷 (%s, %.1f m 이내)'
                     % (arguments.save, arguments.max_range))
                save(buckets, arguments.save, arguments.sector)
            else:
                os.system('clear')
                show(buckets, arguments.sector,
                     '실시간 (%.1f m 이내, Ctrl+C 종료)' % arguments.max_range)
            shown += 1
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
