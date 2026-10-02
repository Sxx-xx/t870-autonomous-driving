#!/usr/bin/env python3
"""Derive the real steering lock angle from a full-lock circle recording.

Drive the vehicle in MANUAL with the steering stick held hard over, record
with t870_gps_record.launch.py, then run this on the resulting CSV. The
bicycle model gives the road wheel angle straight from the turn radius:

    radius = wheelbase / tan(steer_angle)

That angle is what belongs in max_steer_angle_rad, because MANUAL full lock
and the Pixhawk PWM limit map to the same POT_SOFT_MIN/POT_SOFT_MAX endpoints
in the firmware.
"""

import argparse
import csv
import math
from pathlib import Path


def load_points(filename):
    """Read easting/northing/time from a gps_path_recorder CSV."""
    with Path(filename).open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    fields = set(rows[0]) if rows else set()
    if not {'utm_easting_m', 'utm_northing_m'} <= fields:
        raise ValueError('CSV must contain utm_easting_m and utm_northing_m')
    points = []
    for index, row in enumerate(rows):
        stamp = float(row['ros_time_sec']) if 'ros_time_sec' in fields else float(index)
        points.append((
            float(row['utm_easting_m']), float(row['utm_northing_m']), stamp))
    return points


def advance(points, start, distance):
    """Index of the first point at least `distance` metres past `start`."""
    x0, y0, _ = points[start]
    for index in range(start + 1, len(points)):
        if math.hypot(points[index][0] - x0, points[index][1] - y0) >= distance:
            return index
    return None


def turn_signs(points, baseline):
    """Signed turn direction using a fixed metric baseline.

    Adjacent samples are useless here: at 0.2 m spacing with metre-level GPS
    noise the sign of the local curvature is random. Comparing headings taken
    `baseline` metres apart puts the real turn well above the noise.
    """
    signs = [0] * len(points)
    for i in range(len(points)):
        j = advance(points, i, baseline)
        if j is None:
            break
        k = advance(points, j, baseline)
        if k is None:
            break
        ax, ay = points[j][0] - points[i][0], points[j][1] - points[i][1]
        bx, by = points[k][0] - points[j][0], points[k][1] - points[j][1]
        cross = ax * by - ay * bx
        dot = ax * bx + ay * by
        angle = math.atan2(cross, dot)
        if abs(angle) > math.radians(5.0):
            signs[i] = 1 if angle > 0 else -1
    return signs


def segment_arcs(points, minimum_points, baseline, time_gap, confirm):
    """Split the track into runs that keep turning the same way.

    A pause between circles shows up as a time gap rather than as points,
    because the recorder only writes a row once the vehicle has moved. Brief
    opposite-sign blips inside one circle are noise, so runs shorter than
    `confirm` are dropped and the neighbours on either side are rejoined.
    """
    signs = turn_signs(points, baseline)
    runs = []
    for index, (point, sign) in enumerate(zip(points, signs)):
        if sign == 0:
            continue
        gap = index > 0 and point[2] - points[index - 1][2] > time_gap
        if runs and runs[-1][0] == sign and not gap:
            runs[-1][1].append(point)
        else:
            runs.append([sign, [point], gap])

    kept = [run for run in runs if len(run[1]) >= confirm]
    merged = []
    for run in kept:
        if (merged and merged[-1][0] == run[0] and not run[2]):
            merged[-1][1].extend(run[1])
        else:
            merged.append(run)
    return [(sign, arc) for sign, arc, _ in merged if len(arc) >= minimum_points]


def split_on_gaps(points, time_gap, minimum_points):
    """Split only where the recording paused, keeping each straight run whole."""
    runs = [[]]
    for index, point in enumerate(points):
        if index > 0 and point[2] - points[index - 1][2] > time_gap:
            runs.append([])
        runs[-1].append(point)
    return [run for run in runs if len(run) >= minimum_points]


def fit_circle(points):
    """Algebraic least-squares circle fit (Kasa). Returns centre and radius."""
    n = len(points)
    mean_x = sum(p[0] for p in points) / n
    mean_y = sum(p[1] for p in points) / n
    suu = suv = svv = suuu = svvv = suvv = svuu = 0.0
    for x, y, _ in points:
        u, v = x - mean_x, y - mean_y
        suu += u * u
        svv += v * v
        suv += u * v
        suuu += u * u * u
        svvv += v * v * v
        suvv += u * v * v
        svuu += v * u * u
    determinant = 2.0 * (suu * svv - suv * suv)
    if abs(determinant) < 1e-12:
        raise ValueError('points are collinear; no circle to fit')
    rhs_u = suuu + suvv
    rhs_v = svvv + svuu
    centre_u = (svv * rhs_u - suv * rhs_v) / determinant
    centre_v = (suu * rhs_v - suv * rhs_u) / determinant
    centre_x, centre_y = centre_u + mean_x, centre_v + mean_y
    radii = [math.hypot(x - centre_x, y - centre_y) for x, y, _ in points]
    radius = sum(radii) / n
    spread = math.sqrt(sum((r - radius) ** 2 for r in radii) / n)
    return centre_x, centre_y, radius, spread


def arc_sweep(points, centre_x, centre_y):
    """Total turned angle in degrees, so partial laps can be spotted."""
    total = 0.0
    for previous, current in zip(points, points[1:]):
        a = math.atan2(previous[1] - centre_y, previous[0] - centre_x)
        b = math.atan2(current[1] - centre_y, current[0] - centre_x)
        total += (b - a + math.pi) % (2.0 * math.pi) - math.pi
    return math.degrees(abs(total))


def report_straight_runs(points, arguments):
    """Measure how far a nominally straight run actually curves.

    Driving in MANUAL with the steering stick centred parks the rack at
    POT_AT_CENTER, so any residual curvature is the centre calibration error.
    A perfectly centred rack gives an enormous radius; a biased one arcs.
    """
    runs = split_on_gaps(points, arguments.time_gap, arguments.min_points)
    if not runs:
        raise SystemExit('no run long enough; check the recording')
    for index, run in enumerate(runs, start=1):
        length = sum(
            math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(run, run[1:]))
        straight = math.hypot(run[-1][0] - run[0][0], run[-1][1] - run[0][1])
        print(f'\nRun {index}: {len(run)} samples, '
              f'{length:.1f} m driven, {straight:.1f} m end to end')
        if length < arguments.min_length:
            print(f'  SKIPPED: shorter than --min-length {arguments.min_length} m')
            continue
        try:
            centre_x, centre_y, radius, spread = fit_circle(run)
        except ValueError:
            print('  perfectly straight within the fit tolerance')
            continue
        sign = signed_side(run, centre_x, centre_y)
        steer = math.atan2(arguments.wheelbase, radius)
        counts = math.degrees(steer) / arguments.degrees_per_count
        side = 'LEFT' if sign > 0 else 'RIGHT'
        print(f'  radius {radius:.1f} m curving {side} '
              f'(fit spread {spread:.3f} m)')
        print(f'  -> residual steering {math.degrees(steer):.2f} deg '
              f'= {counts:+.0f} POT counts of centre error')
        print(f'  -> POT_AT_CENTER should move '
              f'{"UP (right)" if sign > 0 else "DOWN (left)"} by {abs(counts):.0f}')
        if spread > 0.5:
            print('  WARNING: loose fit; drive slower and straighter')


def signed_side(run, centre_x, centre_y):
    """+1 if the path curves anticlockwise (left), -1 if clockwise."""
    total = 0.0
    for previous, current in zip(run, run[1:]):
        a = math.atan2(previous[1] - centre_y, previous[0] - centre_x)
        b = math.atan2(current[1] - centre_y, current[0] - centre_x)
        total += (b - a + math.pi) % (2.0 * math.pi) - math.pi
    return 1.0 if total > 0 else -1.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv_file', help='gps_path_recorder output CSV')
    parser.add_argument('--wheelbase', type=float, default=1.0,
                        help='front-to-rear axle distance in metres')
    parser.add_argument('--min-points', type=int, default=25,
                        help='ignore arcs shorter than this many samples')
    parser.add_argument('--baseline', type=float, default=2.0,
                        help='metres between headings used to detect turning')
    parser.add_argument('--time-gap', type=float, default=2.0,
                        help='seconds of no motion that separates two circles')
    parser.add_argument('--confirm', type=int, default=12,
                        help='samples a direction change must hold to count')
    parser.add_argument('--straight', action='store_true',
                        help='measure centre error from a nominally straight run')
    parser.add_argument('--min-length', type=float, default=15.0,
                        help='straight mode: shortest usable run in metres')
    parser.add_argument('--degrees-per-count', type=float, default=0.03533,
                        help='steering degrees per POT count (16.93 deg / 479)')
    parser.add_argument('--follower-limit', type=float, default=0.12,
                        help='current gps_follow_max_steer_rad, for comparison')
    arguments = parser.parse_args()

    points = load_points(arguments.csv_file)
    print(f'{len(points)} samples from {arguments.csv_file}')
    if arguments.straight:
        report_straight_runs(points, arguments)
        return
    runs = segment_arcs(points, arguments.min_points, arguments.baseline,
                        arguments.time_gap, arguments.confirm)
    if not runs:
        raise SystemExit('no sustained arc found; check the recording')

    results = []
    for index, (sign, arc) in enumerate(runs, start=1):
        label = 'LEFT (CCW)' if sign > 0 else 'RIGHT (CW)'
        centre_x, centre_y, radius, spread = fit_circle(arc)
        steer = math.atan2(arguments.wheelbase, radius)
        sweep = arc_sweep(arc, centre_x, centre_y)
        duration = arc[-1][2] - arc[0][2]
        print(f'\nArc {index}: {label}')
        print(f'  samples {len(arc)}, sweep {sweep:.0f} deg '
              f'({sweep / 360.0:.2f} laps), {duration:.1f} s')
        print(f'  radius {radius:.3f} m  (fit spread {spread:.3f} m)')
        print(f'  -> steering angle {math.degrees(steer):.2f} deg '
              f'= {steer:.4f} rad')
        if sweep < 270.0:
            print('  WARNING: less than 3/4 of a lap; radius is unreliable')
        if spread > 0.25 * radius:
            print('  WARNING: poor fit; the stick may not have been held steady')
        results.append((sign, steer, radius))

    lefts = [s for sign, s, _ in results if sign > 0]
    rights = [s for sign, s, _ in results if sign < 0]
    print('\n=== summary ===')
    for name, values in (('LEFT ', lefts), ('RIGHT', rights)):
        if values:
            mean = sum(values) / len(values)
            print(f'{name} lock: {math.degrees(mean):6.2f} deg '
                  f'= {mean:.4f} rad   ({len(values)} arc(s))')
    if lefts and rights:
        left_mean = sum(lefts) / len(lefts)
        right_mean = sum(rights) / len(rights)
        smaller = min(left_mean, right_mean)
        print(f'asymmetry: left/right = {left_mean / right_mean:.3f}')
        print(f'\nUse the SMALLER lock so neither side saturates early:')
        print(f'  max_steer_angle_rad: {smaller:.4f}   '
              f'({math.degrees(smaller):.2f} deg)')
        current_actual = arguments.follower_limit * (smaller / 0.25)
        print(f'\nWith the present max_steer_angle_rad of 0.25, a follower '
              f'command of {arguments.follower_limit} rad')
        print(f'is really steering {math.degrees(current_actual):.2f} deg '
              f'-> loop gain {current_actual / arguments.follower_limit:.2f}x')
        print(f'After the fix, ask for that same real angle with '
              f'gps_follow_max_steer_rad:={current_actual:.3f}')


if __name__ == '__main__':
    main()
