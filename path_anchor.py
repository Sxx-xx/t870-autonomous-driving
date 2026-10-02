#!/usr/bin/env python3
"""지금 차가 서 있는 자리에 경로의 한 점을 맞춰 경로 전체를 평행이동한다.

수신기 원점이 세션마다 튀면 경로 모양은 맞는데 위치만 통째로 밀린다.
그럴 때 차를 경로의 기준점(보통 시작점)에 정확히 세워 놓고 이걸 돌리면
그 점이 지금 좌표에 오도록 경로 전체가 따라 옮겨진다.

수신기 시리얼을 직접 읽으므로 ROS 를 띄울 필요가 없다. RTK Fix 가 아니면
멈춘다. 단독측위 좌표에 경로를 맞추면 경로가 더 망가진다.

주석(#) 처리된 점도 같이 옮긴다. 나중에 주석을 풀어도 자리가 맞는다.
원본은 .bak 으로 남는다.

쓰는 법 (conda 말고 시스템 파이썬으로):
    /usr/bin/python3 path_anchor.py                    # idx 1 을 현재 위치로
    /usr/bin/python3 path_anchor.py --index 0          # idx 0 을 현재 위치로
    /usr/bin/python3 path_anchor.py --seconds 20       # 20 초 평균
    /usr/bin/python3 path_anchor.py --dry-run          # 계산만, 파일은 그대로
"""
import argparse
import math
import os
import shutil
import sys
import time

import serial
from pyproj import Transformer

PORT = ('/dev/serial/by-id/'
        'usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00')
QUALITY = {'0': '측위불가', '1': '단독측위', '2': 'DGPS',
           '4': 'RTK Fix', '5': 'RTK Float'}
COMPASS = ['북', '북동', '동', '남동', '남', '남서', '서', '북서']


def nmea_degrees(value, hemi):
    degrees = int(float(value) / 100)
    decimal = degrees + (float(value) - degrees * 100) / 60
    return -decimal if hemi in ('S', 'W') else decimal


def read_position(seconds, allow_float):
    """GGA 를 모아 평균 위치를 낸다. 품질과 기준국도 같이 돌려준다."""
    good, quality, stations = [], set(), set()
    print('%d 초간 위치를 모은다. 차를 움직이지 말 것.' % int(seconds))
    with serial.Serial(PORT, 38400, timeout=1) as port:
        end = time.time() + seconds
        while time.time() < end:
            line = port.readline().decode('ascii', 'ignore').strip()
            if 'GGA' not in line[:7]:
                continue
            field = line.split(',')
            if len(field) < 14 or not field[2] or not field[4]:
                continue
            quality.add(field[6])
            if len(field) > 14:
                stations.add(field[14].split('*')[0])
            if field[6] == '4' or (allow_float and field[6] == '5'):
                good.append((nmea_degrees(field[4], field[5]),
                             nmea_degrees(field[2], field[3])))
    return good, quality, stations


def load_lines(filename):
    lines = open(filename, encoding='utf-8').read().splitlines()
    header = lines[0].split(',')
    return lines[0], lines[1:], header.index('longitude'), header.index('latitude')


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('path_file', nargs='?',
                        default='gps_recordings/path5_comp.csv')
    parser.add_argument('--index', type=int, default=1,
                        help='현재 위치에 맞출 경로점 번호 (기본 1)')
    parser.add_argument('--seconds', type=float, default=10.0)
    parser.add_argument('--allow-float', action='store_true',
                        help='RTK Float 도 받는다. 권하지 않는다')
    parser.add_argument('--dry-run', action='store_true')
    arguments = parser.parse_args()

    good, quality, stations = read_position(arguments.seconds,
                                            arguments.allow_float)
    names = ', '.join(QUALITY.get(q, q) for q in sorted(quality)) or '없음'
    print('  받은 품질: %s' % names)
    print('  기준국   : %s' % (', '.join(sorted(s for s in stations if s)) or '-'))
    if not good:
        raise SystemExit('RTK Fix 표본이 없다. 기준을 못 잡는다. '
                         '정 급하면 --allow-float 를 쓴다.')

    longitude = sum(p[0] for p in good) / len(good)
    latitude = sum(p[1] for p in good) / len(good)
    print('  표본 %d개, 현재 위치 %.7f, %.7f' % (len(good), latitude, longitude))

    to_metres = Transformer.from_crs('EPSG:4326', 'EPSG:32652', always_xy=True)
    to_degrees = Transformer.from_crs('EPSG:32652', 'EPSG:4326', always_xy=True)
    car_x, car_y = to_metres.transform(longitude, latitude)
    # 표본이 얼마나 흩어졌는지. 이게 크면 옮긴 값도 그만큼 못 믿는다.
    scatter = max(math.hypot(*(a - b for a, b in
                               zip(to_metres.transform(*p), (car_x, car_y))))
                  for p in good)

    header, body, longitude_at, latitude_at = load_lines(arguments.path_file)
    if not 0 <= arguments.index < len(body):
        raise SystemExit('idx %d 가 경로 범위(0~%d) 밖이다'
                         % (arguments.index, len(body) - 1))
    anchor = body[arguments.index].lstrip('# ').split(',')
    anchor_x, anchor_y = to_metres.transform(float(anchor[longitude_at]),
                                             float(anchor[latitude_at]))
    east, north = car_x - anchor_x, car_y - anchor_y
    bearing = (math.degrees(math.atan2(east, north)) + 360.0) % 360.0

    print()
    print('idx %d 를 현재 위치로 옮긴다' % arguments.index)
    print('  경로 전체를 동 %+.2f m, 북 %+.2f m' % (east, north))
    print('  = %s쪽(%.0f도) 으로 %.2f m'
          % (COMPASS[int((bearing + 22.5) // 45) % 8], bearing,
             math.hypot(east, north)))
    print('  표본 흩어짐 %.2f m' % scatter)

    # idx 0 으로 잡았으면 얼마나 달라지는지도 같이 보여 준다. 한 점 차이가
    # 1 m 가 넘으므로 어느 쪽을 원했는지 눈으로 확인할 수 있어야 한다.
    for other in (0, 1):
        if other == arguments.index or other >= len(body):
            continue
        field = body[other].lstrip('# ').split(',')
        ox, oy = to_metres.transform(float(field[longitude_at]),
                                     float(field[latitude_at]))
        print('  (참고: idx %d 로 잡았다면 동 %+.2f m, 북 %+.2f m)'
              % (other, car_x - ox, car_y - oy))

    if arguments.dry_run:
        print()
        print('--dry-run 이라 파일은 그대로 두었다.')
        return

    output = []
    for line in body:
        commented = line.lstrip().startswith('#')
        field = line.lstrip('# ').split(',')
        x, y = to_metres.transform(float(field[longitude_at]),
                                   float(field[latitude_at]))
        new_longitude, new_latitude = to_degrees.transform(x + east, y + north)
        field[longitude_at] = '%.10f' % new_longitude
        field[latitude_at] = '%.10f' % new_latitude
        output.append(('# ' if commented else '') + ','.join(field))

    shutil.copy2(arguments.path_file, arguments.path_file + '.bak')
    with open(arguments.path_file, 'w', encoding='utf-8') as stream:
        stream.write('\n'.join([header] + output) + '\n')

    print()
    print('%s 를 옮겼다 (%d점). 원본은 %s.bak'
          % (arguments.path_file, len(output),
             os.path.basename(arguments.path_file)))
    print('되돌리기:  mv %s.bak %s' % (arguments.path_file, arguments.path_file))
    print('확인    :  /usr/bin/python3 path_offset_check.py %s'
          % arguments.path_file)


if __name__ == '__main__':
    main()
