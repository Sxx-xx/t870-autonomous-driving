#!/usr/bin/env python3
"""경로 전체를 미터 단위로 평행이동한다.

안테나가 차량 중심에서 벗어나 있거나 측위에 계통 편향이 있으면 경로 전체가
한쪽으로 나란히 밀린다. 모양은 맞는데 위치만 틀어진 경우다. 그럴 때 점을
하나씩 고치지 말고 전체를 같은 양만큼 옮긴다.

방향은 지도 기준이다. east 는 동쪽(+)/서쪽(-), north 는 북쪽(+)/남쪽(-).

쓰는 법:
    # 동쪽으로 0.5 m, 북쪽으로 0.3 m
    python3 path_shift.py gps_recordings/path5.csv --east 0.5 --north 0.3

    # 나침반 방위로 주고 싶을 때 (북=0, 시계방향). 북동쪽으로 0.6 m
    python3 path_shift.py gps_recordings/path5.csv --bearing 45 --distance 0.6

    # 결과를 다른 이름으로
    python3 path_shift.py gps_recordings/path5.csv --east 0.5 -o path5b.csv
"""

import argparse
import csv
import math
import os

from pyproj import Transformer


def utm_epsg(longitude, latitude):
    zone = max(1, min(60, int((longitude + 180.0) / 6.0) + 1))
    return (32600 if latitude >= 0.0 else 32700) + zone


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('csv_file')
    parser.add_argument('--east', type=float, default=0.0, help='동쪽 이동 m (음수는 서쪽)')
    parser.add_argument('--north', type=float, default=0.0, help='북쪽 이동 m (음수는 남쪽)')
    parser.add_argument('--bearing', type=float,
                        help='나침반 방위 도. 북=0, 동=90, 시계방향')
    parser.add_argument('--distance', type=float, help='--bearing 과 함께 쓰는 거리 m')
    parser.add_argument('-o', '--output', help='결과 파일 (기본: 원본이름_shift.csv)')
    arguments = parser.parse_args()

    east, north = arguments.east, arguments.north
    if arguments.bearing is not None:
        if arguments.distance is None:
            raise SystemExit('--bearing 을 쓰면 --distance 도 있어야 한다')
        radians = math.radians(arguments.bearing)
        east = arguments.distance * math.sin(radians)
        north = arguments.distance * math.cos(radians)
    if east == 0.0 and north == 0.0:
        raise SystemExit('이동량이 0 이다. --east/--north 또는 --bearing/--distance 를 줄 것')

    with open(arguments.csv_file, newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        fieldnames = reader.fieldnames or []
        rows = list(reader)
    if not rows:
        raise SystemExit('빈 파일이다')
    if not {'longitude', 'latitude'} <= set(fieldnames):
        raise SystemExit('longitude/latitude 컬럼이 없다')

    epsg = utm_epsg(float(rows[0]['longitude']), float(rows[0]['latitude']))
    to_metres = Transformer.from_crs('EPSG:4326', 'EPSG:%d' % epsg, always_xy=True)
    to_degrees = Transformer.from_crs('EPSG:%d' % epsg, 'EPSG:4326', always_xy=True)

    has_utm = {'utm_easting_m', 'utm_northing_m'} <= set(fieldnames)
    for row in rows:
        x, y = to_metres.transform(float(row['longitude']), float(row['latitude']))
        x += east
        y += north
        longitude, latitude = to_degrees.transform(x, y)
        row['longitude'] = '%.10f' % longitude
        row['latitude'] = '%.10f' % latitude
        # follower 는 utm 컬럼이 있으면 그쪽을 먼저 쓴다. 같이 옮기지 않으면
        # 옮긴 좌표가 무시되고 원래 자리로 주행한다.
        if has_utm:
            row['utm_easting_m'] = '%.4f' % x
            row['utm_northing_m'] = '%.4f' % y

    output = arguments.output
    if not output:
        base, extension = os.path.splitext(arguments.csv_file)
        output = base + '_shift' + extension
    elif not os.path.dirname(output):
        output = os.path.join(os.path.dirname(arguments.csv_file) or '.', output)

    with open(output, 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    bearing = (math.degrees(math.atan2(east, north)) + 360.0) % 360.0
    names = ['북', '북동', '동', '남동', '남', '남서', '서', '북서']
    print('%s  ->  %s' % (arguments.csv_file, output))
    print('  %d점을 동 %+.2f m, 북 %+.2f m 이동'% (len(rows), east, north))
    print('  = 나침반 %.0f도(%s) 방향으로 %.2f m'
          % (bearing, names[int((bearing + 22.5) // 45) % 8], math.hypot(east, north)))
    if has_utm:
        print('  utm_easting_m / utm_northing_m 컬럼도 같이 옮겼다')
    print()
    print('확인:  python3 path_to_qgis.py %s' % output)
    print('       python3 path_qgis_project.py %s --open'
          % (os.path.splitext(output)[0] + '.gpkg'))


if __name__ == '__main__':
    main()
