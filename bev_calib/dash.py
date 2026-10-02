#!/usr/bin/env python3
"""주행 중 상태를 한 줄로 계속 보여준다.

  python3 bev_calib/dash.py

  WP  속도(제한/실제)  모드  미션  GPS  신호등  주차  E-STOP
"""
import sys

sys.path = [p for p in sys.path
            if p != '/usr/local/lib/python3.12/dist-packages']

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String, UInt32
from sensor_msgs.msg import NavSatFix

FIX = {-1: 'NO', 0: 'GPS', 1: 'DGPS', 2: 'RTK'}


class Dash(Node):
    def __init__(self):
        super().__init__('t870_dash')
        self.v = {'wp': '-', 'lim': 0.0, 'spd': 0.0, 'mode': '-',
                  'mission': '-', 'fix': '-', 'sats': '-', 'light': '-',
                  't': '-', 'p': '-', 'estop': False, 'xtrack': 0.0,
                  'lad': 0.0, 'dyn': False}

        def sub(kind, topic, key, conv=lambda m: m.data):
            self.create_subscription(
                kind, topic, lambda m, k=key, c=conv: self.v.__setitem__(k, c(m)), 10)

        sub(UInt32, '/gps/current_waypoint', 'wp')
        sub(Float32, '/t870/gps_speed_limit', 'lim')
        sub(Float32, '/t870/current_speed', 'spd')
        sub(String, '/t870/control_mode', 'mode')
        sub(String, '/t870/active_mission', 'mission')
        sub(String, '/vision/traffic_light_state', 'light')
        sub(UInt32, '/competition/t_parking_choice', 't')
        sub(UInt32, '/competition/parallel_choice', 'p')
        sub(Bool, '/t870/emergency_stop', 'estop')
        sub(Bool, '/t870/dynamic_obstacle_done', 'dyn')
        sub(Float32, '/gps/cross_track_error', 'xtrack')
        sub(Float32, '/gps/lookahead_distance', 'lad')
        self.create_subscription(NavSatFix, '/gps/fix', self.on_fix, 10)
        self.create_timer(0.3, self.show)

    def on_fix(self, msg):
        self.v['fix'] = FIX.get(int(msg.status.status), '?')

    def show(self):
        v = self.v
        line = ('WP %-4s | %4.1f/%4.1f km/h | %-6s | %-8s | %-4s | '
                '신호 %-7s | T%s P%s | XT %+5.2f | LAD %.1f %s'
                % (v['wp'], float(v['lim']) * 3.6, float(v['spd']) * 3.6,
                   v['mode'], v['mission'], v['fix'], v['light'],
                   v['t'], v['p'], float(v['xtrack']), float(v['lad']),
                   '[E-STOP]' if v['estop'] else ''))
        sys.stdout.write('\r' + line[:150].ljust(150))
        sys.stdout.flush()


def main():
    rclpy.init()
    node = Dash()
    print('T870 주행 상태 (Ctrl-C 로 종료)\n')
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
