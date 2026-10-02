#!/usr/bin/env python3
"""기록한 GPS 경로를 QGIS 에서 바로 열리는 GeoPackage 로 만든다.

gps_path_recorder 가 남긴 CSV 를 읽어 같은 이름의 .gpkg 를 만든다.
QGIS 에서 더블클릭하거나 끌어다 놓으면 좌표계 설정 없이 바로 열린다.

레이어 두 개가 들어간다.
  points  점마다 한 개. 곡률 반지름과 필요 조향각이 속성으로 붙어 있어
          어느 코너가 차량 한계에 걸리는지 QGIS 에서 색으로 바로 보인다.
  track   전체 경로를 이은 선 한 개.

쓰는 법:
    python3 path_to_qgis.py gps_recordings/path5.csv
    python3 path_to_qgis.py gps_recordings/path5.csv --wheelbase 0.725
"""

import argparse
import csv
import math
import os
import sys

from osgeo import ogr, osr
from pyproj import Transformer

# GDAL 4.0 부터 기본이 바뀐다는 경고를 끈다. 여기서는 반환값을 직접 검사한다.
ogr.UseExceptions()
osr.UseExceptions()

# 차량 실측값. 09-15 메모 [현재 파라미터 요약] 참조.
WHEELBASE_M = 0.725
STEER_LIMIT_RAD = 0.20
LOOKAHEAD_MAX_M = 5.0


def read_route(filename):
    """CSV 한 줄을 (경도, 위도, 목표속도) 로 읽는다.

    기록기가 쓴 형식과 QGIS 에서 내보낸 형식을 모두 받는다.
    """
    with open(filename, newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(
            line for line in stream if not line.lstrip().startswith('#')))
    if not rows:
        raise SystemExit('빈 파일이다: %s' % filename)
    fields = set(rows[0])
    if not {'longitude', 'latitude'} <= fields:
        raise SystemExit(
            'longitude/latitude 컬럼이 없다. 가진 컬럼: %s' % ', '.join(sorted(fields)))
    route = []
    for row in rows:
        try:
            speed = float(row.get('target_speed') or 0.0)
        except ValueError:
            speed = 0.0
        route.append((float(row['longitude']), float(row['latitude']),
                      speed, row.get('fix_status', '')))
    return route


def curvature_radius(points, index, span=3):
    """세 점을 지나는 원의 반지름. 직선에 가까우면 아주 큰 값이 나온다."""
    if index - span < 0 or index + span >= len(points):
        return None
    ax, ay = points[index][0] - points[index - span][0], \
        points[index][1] - points[index - span][1]
    bx, by = points[index + span][0] - points[index][0], \
        points[index + span][1] - points[index][1]
    cross = ax * by - ay * bx
    if abs(cross) < 1e-9:
        return None
    return (math.hypot(ax, ay) * math.hypot(bx, by)
            * math.hypot(points[index + span][0] - points[index - span][0],
                         points[index + span][1] - points[index - span][1])
            / (2.0 * abs(cross)))


def verdict(radius, wheelbase, limit):
    """이 코너를 차가 돌 수 있는지 한 마디로."""
    if radius is None:
        return '직선', 0.0
    need = math.atan2(wheelbase, radius)
    ratio = need / limit
    if ratio > 1.0:
        return '못 돔', math.degrees(need)
    if ratio > 0.8:
        return '한계 근접', math.degrees(need)
    if radius < LOOKAHEAD_MAX_M:
        return 'LAD보다 급함', math.degrees(need)
    return '여유', math.degrees(need)


def build(csv_path, wheelbase, limit):
    route = read_route(csv_path)
    transformer = Transformer.from_crs(
        'EPSG:4326',
        'EPSG:%d' % ((32600 if route[0][1] >= 0 else 32700)
                     + max(1, min(60, int((route[0][0] + 180.0) / 6.0) + 1))),
        always_xy=True)
    metres = [transformer.transform(lon, lat) for lon, lat, _, _ in route]

    out_path = os.path.splitext(csv_path)[0] + '.gpkg'
    if os.path.exists(out_path):
        os.remove(out_path)

    driver = ogr.GetDriverByName('GPKG')
    source = driver.CreateDataSource(out_path)
    wgs84 = osr.SpatialReference()
    wgs84.ImportFromEPSG(4326)
    wgs84.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)

    points = source.CreateLayer('points', wgs84, ogr.wkbPoint)
    for name, kind in (('idx', ogr.OFTInteger), ('radius_m', ogr.OFTReal),
                       ('steer_deg', ogr.OFTReal), ('limit_pct', ogr.OFTReal),
                       ('verdict', ogr.OFTString), ('target_speed', ogr.OFTReal),
                       ('fix_status', ogr.OFTString)):
        points.CreateField(ogr.FieldDefn(name, kind))

    counts = {}
    tight = []
    for i, (lon, lat, speed, fix) in enumerate(route):
        radius = curvature_radius(metres, i)
        label, steer = verdict(radius, wheelbase, limit)
        counts[label] = counts.get(label, 0) + 1
        if radius is not None and label in ('못 돔', '한계 근접', 'LAD보다 급함'):
            tight.append((radius, i, label))
        feature = ogr.Feature(points.GetLayerDefn())
        feature.SetField('idx', i)
        feature.SetField('radius_m', -1.0 if radius is None else round(radius, 2))
        feature.SetField('steer_deg', round(steer, 2))
        feature.SetField('limit_pct', round(100.0 * math.radians(steer) / limit, 1))
        feature.SetField('verdict', label)
        feature.SetField('target_speed', speed)
        feature.SetField('fix_status', str(fix))
        geometry = ogr.Geometry(ogr.wkbPoint)
        geometry.AddPoint(lon, lat)
        feature.SetGeometry(geometry)
        points.CreateFeature(feature)

    track = source.CreateLayer('track', wgs84, ogr.wkbLineString)
    track.CreateField(ogr.FieldDefn('length_m', ogr.OFTReal))
    track.CreateField(ogr.FieldDefn('points', ogr.OFTInteger))
    line = ogr.Geometry(ogr.wkbLineString)
    for lon, lat, _, _ in route:
        line.AddPoint(lon, lat)
    length = sum(math.hypot(metres[i][0] - metres[i - 1][0],
                            metres[i][1] - metres[i - 1][1])
                 for i in range(1, len(metres)))
    feature = ogr.Feature(track.GetLayerDefn())
    feature.SetField('length_m', round(length, 1))
    feature.SetField('points', len(route))
    feature.SetGeometry(line)
    track.CreateFeature(feature)
    source = None

    return out_path, route, metres, length, counts, tight


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('csv_file', help='gps_path_recorder 가 남긴 CSV')
    parser.add_argument('--wheelbase', type=float, default=WHEELBASE_M)
    parser.add_argument('--steer-limit', type=float, default=STEER_LIMIT_RAD,
                        help='gps_follow_max_steer_rad 와 같은 값')
    arguments = parser.parse_args()

    out_path, route, metres, length, counts, tight = build(
        arguments.csv_file, arguments.wheelbase, arguments.steer_limit)

    gaps = [math.hypot(metres[i][0] - metres[i - 1][0],
                       metres[i][1] - metres[i - 1][1])
            for i in range(1, len(metres))]
    closed = math.hypot(metres[-1][0] - metres[0][0], metres[-1][1] - metres[0][1])
    fixes = {}
    for _, _, _, fix in route:
        fixes[fix] = fixes.get(fix, 0) + 1
    fix_name = {'2': 'RTK', '1': 'DGPS', '0': '단독측위'}

    print('%s  ->  %s' % (arguments.csv_file, out_path))
    print()
    print('  점 %d개, 길이 %.0f m, 시종점 거리 %.1f m (%s)'
          % (len(route), length, closed, '순환' if closed < 15 else '개방'))
    print('  점 간격 최대 %.2f m %s'
          % (max(gaps), '' if max(gaps) < 3 else '  <- 공백 있음'))
    if fixes:
        print('  fix: %s' % ', '.join(
            '%s %d점(%.0f%%)' % (fix_name.get(k, k or '?'), v, 100 * v / len(route))
            for k, v in sorted(fixes.items(), reverse=True)))
    print()
    print('  곡률 판정 (축거 %.3f m, 조향한계 %.2f rad -> 최소 반지름 %.2f m)'
          % (arguments.wheelbase, arguments.steer_limit,
             arguments.wheelbase / math.tan(arguments.steer_limit)))
    for label in ('못 돔', '한계 근접', 'LAD보다 급함', '여유', '직선'):
        if label in counts:
            print('    %-14s %3d점' % (label, counts[label]))
    if tight:
        print()
        print('  급한 곳 (반지름 작은 순)')
        for radius, index, label in sorted(tight)[:8]:
            print('    idx %3d  반지름 %6.2f m  %s' % (index, radius, label))
    print()
    print('QGIS 에서 %s 를 더블클릭하거나 끌어다 놓으면 된다.' % os.path.basename(out_path))
    print('points 레이어를 verdict 나 radius_m 으로 색 분류하면')
    print('어느 코너가 문제인지 바로 보인다.')


if __name__ == '__main__':
    main()
