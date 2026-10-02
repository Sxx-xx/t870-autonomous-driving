#!/usr/bin/env python3
"""주행 확인용 한 화면.

상태 한 줄을 주기적으로 찍고, 로그의 중요한 줄은 나오는 즉시 끼워 넣는다.
터미널 하나로 미션 진행과 판단을 같이 볼 수 있다.

  python3 bev_calib/watch.py
  python3 bev_calib/watch.py --every 1.0 --log /tmp/t870_competition.log
"""
import argparse
import os
import re
import sys
import time

sys.path = [p for p in sys.path
            if p != '/usr/local/lib/python3.12/dist-packages']

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float32, String, UInt32

FIX = {-1: 'NO', 0: 'GPS', 1: 'DGPS', 2: 'RTK'}
# 놓치면 안 되는 줄만 고른다.
KEEP = re.compile(
    'AVOID|Traffic light|PARKING DECISION|동적장애물|인계|PATH SWITCH|'
    'COMPETITION E-STOP|카메라를 열 수 없다|읽기실패 [1-9]|'
    'Startup straight|released at|FORCED GO|UnicodeDecodeError|Traceback')
STRIP = re.compile(r'^\[[^\]]+\]\s*\[[A-Z]+\]\s*\[[\d.]+\]\s*\[[^\]]+\]:\s*')


class Watch(Node):
    def __init__(self, every, path):
        super().__init__('t870_watch')
        self.every = every
        self.path = path
        self.log_file = None
        self.last = 0.0
        self.v = {'wp': '-', 'lim': 0.0, 'spd': 0.0, 'mode': '-',
                  'mission': '-', 'fix': '-', 'light': '-', 't': '-',
                  'p': '-', 'estop': False, 'xtrack': 0.0, 'lad': 0.0,
                  'tgt': 0.0, 'pwm': 0.0}

        def sub(kind, topic, key):
            self.create_subscription(
                kind, topic,
                lambda m, k=key: self.v.__setitem__(k, m.data), 10)

        sub(UInt32, '/gps/current_waypoint', 'wp')
        sub(Float32, '/t870/gps_speed_limit', 'lim')
        sub(Float32, '/t870/current_speed', 'spd')
        sub(String, '/t870/control_mode', 'mode')
        sub(String, '/t870/active_mission', 'mission')
        sub(String, '/vision/traffic_light_state', 'light')
        sub(UInt32, '/competition/t_parking_choice', 't')
        sub(UInt32, '/competition/parallel_choice', 'p')
        sub(Bool, '/t870/emergency_stop', 'estop')
        sub(Float32, '/gps/cross_track_error', 'xtrack')
        sub(Float32, '/gps/lookahead_distance', 'lad')
        sub(Float32, '/t870/drive_target_kmh', 'tgt')
        sub(Float32, '/t870/drive_pwm', 'pwm')
        self.create_subscription(NavSatFix, '/gps/fix', self.on_fix, 10)
        self.create_timer(0.2, self.tick)

    def on_fix(self, msg):
        self.v['fix'] = FIX.get(int(msg.status.status), '?')

    def open_log(self):
        """launch 가 나중에 떠도 붙는다. 파일이 바뀌면 다시 연다."""
        try:
            if self.log_file is None and os.path.exists(self.path):
                self.log_file = open(self.path, errors='replace')
                self.log_file.seek(0, os.SEEK_END)
                print('[로그 연결] %s' % self.path)
        except OSError:
            self.log_file = None

    def tick(self):
        self.open_log()
        if self.log_file is not None:
            for line in self.log_file.readlines():
                if KEEP.search(line):
                    print('  ' + STRIP.sub('', line.rstrip())[:130])
        now = time.monotonic()
        if now - self.last >= self.every:
            self.last = now
            v = self.v
            print('WP %-4s %4.1f/%4.1f km/h  %-5s %-8s %-4s  신호 %-7s '
                  'T%s P%s  XT %+5.2f  구동 %.1f/PWM%3.0f %s'
                  % (v['wp'], float(v['lim']) * 3.6, float(v['spd']) * 3.6,
                     v['mode'], v['mission'], v['fix'], v['light'],
                     v['t'], v['p'], float(v['xtrack']),
                     float(v['tgt']), float(v['pwm']),
                     '[E-STOP]' if v['estop'] else ''))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--every', type=float, default=2.0)
    parser.add_argument('--log', default='/tmp/t870_competition.log')
    args = parser.parse_args()
    rclpy.init()
    node = Watch(args.every, args.log)
    print('T870 주행 확인 (Ctrl-C 종료). 상태 %.1f초마다, 중요 로그는 즉시.\n'
          % args.every)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
