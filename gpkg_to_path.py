#!/usr/bin/env python3
"""QGIS 에서 편집한 GeoPackage 를 주행용 CSV 로 되돌린다.

QGIS 편집 결과는 .gpkg 에만 남는다. 차는 .csv 를 읽으므로 한 번 되돌려야
한다. QGIS 의 내보내기를 쓰면 X/Y 컬럼 이름을 손으로 바꿔야 하고 UTM 컬럼이
옛값으로 남아 주행이 엉뚱한 자리로 가는 함정이 있다. 이 스크립트는 경도와
위도만 써서 그 함정을 없앤다. follower 는 utm 컬럼이 없으면 경위도를 직접
변환하므로 문제가 없다.

점 순서는 idx 필드를 따른다. QGIS 에서 지우거나 옮겨도 원래 순서가 유지된다.

쓰는 법:
    # 그대로 내보내기
    python3 gpkg_to_path.py gps_recordings/path5.gpkg

    # 뒤에서 32점 버리기
    python3 gpkg_to_path.py gps_recordings/path5.gpkg --drop-tail 32

    # 앞 5점도 같이 버리고 이름 지정
    python3 gpkg_to_path.py gps_recordings/path5.gpkg --drop-head 5 --drop-tail 32 \\
        -o gps_recordings/path6.csv
"""

import argparse
import math
import os

from osgeo import ogr

ogr.UseExceptions()

DEFAULT_SPEED = 2.7778  # m/s, 10 km/h


def read_points(gpkg_path):
    source = ogr.Open(gpkg_path)
    if source is None:
        raise SystemExit('열 수 없다: %s' % gpkg_path)
    layer = source.GetLayerByName('points')
    if layer is None:
        raise SystemExit('points 레이어가 없다: %s' % gpkg_path)
    has_speed = layer.GetLayerDefn().GetFieldIndex('target_speed') >= 0
    has_idx = layer.GetLayerDefn().GetFieldIndex('idx') >= 0
    points = []
    for order, feature in enumerate(layer):
        geometry = feature.GetGeometryRef()
        if geometry is None:
            continue
        speed = feature.GetField('target_speed') if has_speed else None
        points.append((
            feature.GetField('idx') if has_idx else order,
            geometry.GetX(), geometry.GetY(),
            float(speed) if speed else DEFAULT_SPEED,
        ))
    points.sort(key=lambda p: p[0])
    return points


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('gpkg_file')
    parser.add_argument('--drop-tail', type=int, default=0, help='뒤에서 버릴 점 수')
    parser.add_argument('--drop-head', type=int, default=0, help='앞에서 버릴 점 수')
    parser.add_argument('-o', '--output', help='결과 CSV (기본: 원본이름_edit.csv)')
    arguments = parser.parse_args()

    points = read_points(arguments.gpkg_file)
    total = len(points)
    if arguments.drop_head + arguments.drop_tail >= total:
        raise SystemExit('버릴 점이 전체(%d)보다 많다' % total)
    kept = points[arguments.drop_head:total - arguments.drop_tail or None]

    output = arguments.output
    if not output:
        output = os.path.splitext(arguments.gpkg_file)[0] + '_edit.csv'

    with open(output, 'w', encoding='utf-8', newline='') as stream:
        stream.write('longitude,latitude,target_speed\n')
        for _, longitude, latitude, speed in kept:
            stream.write('%.10f,%.10f,%.4f\n' % (longitude, latitude, speed))

    # 길이와 끝점 거리는 주행 전에 확인할 값이라 같이 찍어 둔다.
    from pyproj import Transformer
    zone = max(1, min(60, int((kept[0][1] + 180.0) / 6.0) + 1))
    epsg = (32600 if kept[0][2] >= 0 else 32700) + zone
    to_metres = Transformer.from_crs('EPSG:4326', 'EPSG:%d' % epsg, always_xy=True)
    metres = [to_metres.transform(p[1], p[2]) for p in kept]
    length = sum(math.hypot(metres[i][0] - metres[i - 1][0],
                            metres[i][1] - metres[i - 1][1])
                 for i in range(1, len(metres)))
    closed = math.hypot(metres[-1][0] - metres[0][0], metres[-1][1] - metres[0][1])

    print('%s  ->  %s' % (arguments.gpkg_file, output))
    print('  %d점 중 %d점 남김 (앞 %d, 뒤 %d 버림)'
          % (total, len(kept), arguments.drop_head, arguments.drop_tail))
    print('  길이 %.0f m, 시종점 거리 %.1f m (%s)'
          % (length, closed, '순환' if closed < 15 else '개방'))
    print()
    print('곡률 확인:  python3 path_to_qgis.py %s' % output)


if __name__ == '__main__':
    main()
