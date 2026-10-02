#!/usr/bin/env python3
"""Round sharp corners in a distance/angle relative route CSV."""

import argparse
import csv
import math
from pathlib import Path


def load_relative_route(filename):
    with Path(filename).open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 2 or not {'distance', 'angle'} <= set(rows[0]):
        raise ValueError('input must contain distance and angle columns')

    points = [(0.0, 0.0)]
    previous_distance = float(rows[0]['distance'])
    previous_bearing = math.radians(float(rows[0]['angle']))
    for row in rows[1:]:
        distance = float(row['distance'])
        step = distance - previous_distance
        if step < 0.0:
            raise ValueError('distance must increase')
        east, north = points[-1]
        points.append((
            east + step * math.sin(previous_bearing),
            north + step * math.cos(previous_bearing)))
        previous_distance = distance
        previous_bearing = math.radians(float(row['angle']))
    return points


def smooth_points(points, passes):
    result = list(points)
    for _ in range(passes):
        updated = [result[0]]
        for previous, current, following in zip(
                result, result[1:], result[2:]):
            updated.append((
                0.25 * previous[0] + 0.5 * current[0] + 0.25 * following[0],
                0.25 * previous[1] + 0.5 * current[1] + 0.25 * following[1]))
        updated.append(result[-1])
        result = updated
    return result


def resample(points, spacing):
    cumulative = [0.0]
    for first, second in zip(points, points[1:]):
        cumulative.append(cumulative[-1] + math.hypot(
            second[0] - first[0], second[1] - first[1]))

    targets = [index * spacing for index in range(int(cumulative[-1] / spacing) + 1)]
    if cumulative[-1] - targets[-1] > 1e-6:
        targets.append(cumulative[-1])
    result = []
    segment = 0
    for target in targets:
        while segment + 1 < len(cumulative) - 1 and cumulative[segment + 1] < target:
            segment += 1
        length = cumulative[segment + 1] - cumulative[segment]
        ratio = 0.0 if length == 0.0 else (target - cumulative[segment]) / length
        x0, y0 = points[segment]
        x1, y1 = points[segment + 1]
        result.append((target, x0 + ratio * (x1 - x0), y0 + ratio * (y1 - y0)))
    return result


def write_relative_route(filename, points):
    with Path(filename).open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(('fid', 'id', 'distance', 'angle'))
        for index, (distance, x, y) in enumerate(points):
            if index + 1 < len(points):
                next_x, next_y = points[index + 1][1:]
                bearing = math.degrees(math.atan2(next_x - x, next_y - y)) % 360.0
            else:
                bearing = math.degrees(math.atan2(
                    x - points[index - 1][1], y - points[index - 1][2])) % 360.0
            writer.writerow((index + 1, 1, f'{distance:.6f}', f'{bearing:.9f}'))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('input')
    parser.add_argument('output')
    parser.add_argument('--passes', type=int, default=192)
    parser.add_argument('--spacing', type=float, default=1.0)
    args = parser.parse_args()
    if args.passes < 1 or args.spacing <= 0.0:
        parser.error('passes and spacing must be positive')
    points = load_relative_route(args.input)
    write_relative_route(
        args.output, resample(smooth_points(points, args.passes), args.spacing))


if __name__ == '__main__':
    main()
