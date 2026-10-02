#!/usr/bin/env python3
"""차를 경로 위에 세워 놓고 수신기가 실제로 그 자리를 찍는지 본다.

ROS 를 띄우지 않고 수신기 시리얼을 직접 읽는다. 눈으로 보기에 차가 경로
위에 있는데 여기서 몇 미터가 찍히면 그건 추종 문제가 아니라 수신기 절대
위치가 틀어진 것이다. 경로 파일을 고칠 게 아니라 RTK 를 봐야 한다.

주석(#) 처리된 경로점은 무시한다.

쓰는 법 (conda 말고 시스템 파이썬으로):
    /usr/bin/python3 path_offset_check.py
    /usr/bin/python3 path_offset_check.py gps_recordings/path5.csv
"""
import csv
import math
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


def load_route(filename):
    """주석 줄은 빼고 (경도, 위도) 만 뽑는다. 원래 idx 는 유지한다."""
    lines = open(filename, encoding='utf-8').read().splitlines()
    header = lines[0].split(',')
    longitude_at = header.index('longitude')
    latitude_at = header.index('latitude')
    route = []
    for index, line in enumerate(lines[1:]):
        if line.lstrip().startswith('#'):
            continue
        field = line.split(',')
        route.append((index,
                      float(field[longitude_at]),
                      float(field[latitude_at])))
    return route


def main():
    path_file = (sys.argv[1] if len(sys.argv) > 1
                 else 'gps_recordings/path5_comp.csv')
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0

    route = load_route(path_file)
    to_metres = Transformer.from_crs('EPSG:4326', 'EPSG:32652', always_xy=True)
    metres = [(idx,) + to_metres.transform(lon, lat) for idx, lon, lat in route]

    print('경로 %s, 살아있는 점 %d개' % (path_file, len(metres)))
    print('%d 초간 본다. 차를 경로 위에 세워 두고 볼 것. Ctrl+C 로 중단.'
          % int(seconds))
    print()
    others = []
    for name in ('path5.csv', 'path5_fix.csv', 'path5_comp.csv'):
        other = 'gps_recordings/' + name
        if other.endswith(path_file.split('/')[-1]):
            continue
        try:
            others.append((name[:-4], [(i,) + to_metres.transform(lo, la)
                                       for i, lo, la in load_route(other)]))
        except (OSError, ValueError, IndexError):
            pass
    print('   품질      age 기준국  위성   위도        경도        최근접  거리   경로 방향')

    samples = []
    spread = {}
    stations = set()
    with serial.Serial(PORT, 38400, timeout=1) as port:
        end = time.time() + seconds
        while time.time() < end:
            line = port.readline().decode('ascii', 'ignore').strip()
            if 'GGA' not in line[:7]:
                continue
            field = line.split(',')
            if len(field) < 14 or not field[2] or not field[4]:
                continue
            latitude = nmea_degrees(field[2], field[3])
            longitude = nmea_degrees(field[4], field[5])
            x, y = to_metres.transform(longitude, latitude)
            idx, px, py = min(metres,
                              key=lambda p: math.hypot(p[1] - x, p[2] - y))
            distance = math.hypot(px - x, py - y)
            bearing = (math.degrees(math.atan2(px - x, py - y)) + 360.0) % 360.0
            samples.append(distance)
            station = field[14].split('*')[0] if len(field) > 14 else '-'
            print('  %-9s %4ss %5s %3s  %.7f  %.7f  idx %2d  %5.2f m  %s쪽(%.0f도)'
                  % (QUALITY.get(field[6], field[6]), field[13] or '-',
                     station or '-', field[7], latitude, longitude, idx,
                     distance, COMPASS[int((bearing + 22.5) // 45) % 8],
                     bearing))
            stations.add(station)
            for name, pts in others:
                near = min(pts, key=lambda p: math.hypot(p[1] - x, p[2] - y))
                spread.setdefault(name, []).append(
                    math.hypot(near[1] - x, near[2] - y))

    if samples:
        print()
        print('평균 %.2f m,  최소 %.2f m,  최대 %.2f m'
              % (sum(samples) / len(samples), min(samples), max(samples)))
        print()
        if spread:
            print('다른 경로 파일까지 재보면')
            for name, values in spread.items():
                print('  %-12s 평균 %5.2f m' % (name, sum(values) / len(values)))
            print()
        if len(stations) > 1:
            print('기준국 ID 가 %s 로 바뀌었다. 보정원이 갈리면 절대 위치가'
                  % ', '.join(sorted(stations)))
            print('미터 단위로 튄다. RTK Fix 여도 그렇다.')
            print()
        if sum(samples) / len(samples) < 1.5:
            print('수신기는 정상이다. 차가 경로 위에 있다고 찍는다.')
        else:
            print('차가 눈으로 경로 위인데 이 값이 나오면 수신기 절대 위치가')
            print('틀어진 것이다. 경로 파일이 아니라 RTK 를 봐야 한다.')
            print('차가 실제로 그만큼 떨어져 있으면 위에 찍힌 방향으로 옮기면 된다.')


if __name__ == '__main__':
    main()
