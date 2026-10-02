#!/usr/bin/env python3
"""곡률에 맞춰 경로 점마다 목표속도를 다시 쓴다.

follower 는 경로 파일의 target_speed 컬럼을 점마다 그대로 읽는다. 전 구간
같은 값을 넣으면 직선이 코너에 발목을 잡히고, 코너에 맞춰 올리면 코너에서
핸들을 한 번에 꺾게 된다.

세 가지로 속도를 깎는다.
  횡가속도   v <= sqrt(a_lat * R).    코너에서 밀리지 않을 속도.
  예견시간   v <= LAD / t_preview.    조향을 미리 시작할 시간을 남긴다.
             전방주시거리를 속도로 나눈 값이 예견시간이다. 이게 짧으면
             코너를 코앞에서 보고 핸들을 한 번에 꺾는다.
  감가속도   앞뒤로 훑어 v 변화가 a_long 을 넘지 않게 한다. 코너 앞에서
             미리 줄이고 빠져나오며 올린다.

쓰는 법:
    python3 path_speed_profile.py gps_recordings/p1.csv --max-speed 2.0
    python3 path_speed_profile.py gps_recordings/p1.csv --dry-run
"""
import argparse
import csv
import math
import shutil

from pyproj import Transformer


def curvature_radius(points, index, span=3):
    if index - span < 0 or index + span >= len(points):
        return None
    a, b, c = points[index - span], points[index], points[index + span]
    ax, ay = b[0] - a[0], b[1] - a[1]
    bx, by = c[0] - b[0], c[1] - b[1]
    cross = ax * by - ay * bx
    if abs(cross) < 1e-9:
        return None
    return (math.hypot(ax, ay) * math.hypot(bx, by)
            * math.hypot(c[0] - a[0], c[1] - a[1]) / (2.0 * abs(cross)))


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('csv_file')
    parser.add_argument('--max-speed', type=float, default=2.0,
                        help='직선 목표속도 m/s')
    parser.add_argument('--min-speed', type=float, default=1.0,
                        help='코너에서도 이 밑으로는 안 내린다 m/s')
    parser.add_argument('--lateral-accel', type=float, default=1.5,
                        help='허용 횡가속도 m/s^2')
    parser.add_argument('--long-accel', type=float, default=0.8,
                        help='허용 가감속도 m/s^2')
    parser.add_argument('--lookahead', type=float, default=3.5,
                        help='코너에서 쓰이는 LAD m')
    parser.add_argument('--preview-sec', type=float, default=2.5,
                        help='코너 전에 확보할 예견시간 s')
    parser.add_argument('--dry-run', action='store_true')
    arguments = parser.parse_args()

    with open(arguments.csv_file, newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        fieldnames = reader.fieldnames
        rows = list(reader)
    transformer = Transformer.from_crs('EPSG:4326', 'EPSG:32652', always_xy=True)
    points = [transformer.transform(float(r['longitude']), float(r['latitude']))
              for r in rows]
    step = [math.hypot(points[i][0] - points[i - 1][0],
                       points[i][1] - points[i - 1][1])
            for i in range(1, len(points))]

    preview_cap = arguments.lookahead / arguments.preview_sec
    radii = [curvature_radius(points, i) for i in range(len(points))]
    speed = []
    for radius in radii:
        if radius is None:
            speed.append(arguments.max_speed)
            continue
        limit = min(arguments.max_speed, math.sqrt(arguments.lateral_accel * radius))
        # 급한 코너에서만 예견시간을 건다. 완만한 곳까지 묶을 이유가 없다.
        if radius < 2.0 * arguments.lookahead:
            limit = min(limit, preview_cap)
        speed.append(max(arguments.min_speed, limit))

    # 뒤에서 앞으로: 코너 앞에서 미리 감속한다.
    for i in range(len(speed) - 2, -1, -1):
        speed[i] = min(speed[i], math.sqrt(
            speed[i + 1] ** 2 + 2.0 * arguments.long_accel * step[i]))
    # 앞에서 뒤로: 코너를 빠져나오며 서서히 올린다.
    for i in range(1, len(speed)):
        speed[i] = min(speed[i], math.sqrt(
            speed[i - 1] ** 2 + 2.0 * arguments.long_accel * step[i - 1]))

    print('%s  %d점' % (arguments.csv_file, len(rows)))
    print('  직선 %.2f  코너 최저 %.2f  평균 %.2f m/s'
          % (max(speed), min(speed), sum(speed) / len(speed)))
    travel = sum(step[i] / (0.5 * (speed[i] + speed[i + 1]))
                 for i in range(len(step)))
    flat = sum(step) / arguments.max_speed
    print('  예상 주행시간 %.0f초 (전 구간 %.1f m/s 면 %.0f초)'
          % (travel, arguments.max_speed, flat))
    print()
    print('  idx   반지름     속도')
    previous = None
    for i, (radius, value) in enumerate(zip(radii, speed)):
        mark = '%.2f' % value
        if previous is None or abs(value - previous) > 0.15:
            print('   %3d  %7s  %5s m/s (%.1f km/h)'
                  % (i, '직선' if radius is None else '%.2f m' % radius,
                     mark, value * 3.6))
            previous = value

    if arguments.dry_run:
        print()
        print('--dry-run 이라 파일은 그대로 두었다.')
        return

    for row, value in zip(rows, speed):
        row['target_speed'] = '%.4f' % value
    shutil.copy2(arguments.csv_file, arguments.csv_file + '.bak')
    with open(arguments.csv_file, 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print()
    print('%s 에 반영했다. 원본은 %s.bak'
          % (arguments.csv_file, arguments.csv_file))


if __name__ == '__main__':
    main()
