#!/usr/bin/env python3
"""Smooth a recorded UTM route, offset it laterally, and sample every metre."""

import argparse
import csv
import math
from pathlib import Path

import numpy as np
from pyproj import Transformer


def smooth(values, window):
    window = max(1, int(window))
    if window % 2 == 0:
        window += 1
    if window == 1:
        return values.copy()
    radius = window // 2
    padded = np.pad(values, (radius, radius), mode='edge')
    weights = np.hanning(window)
    weights /= weights.sum()
    return np.convolve(padded, weights, mode='valid')


def cumulative_distance(x, y):
    distances = np.hypot(np.diff(x), np.diff(y))
    return np.concatenate(([0.0], np.cumsum(distances)))


def interpolate_by_distance(x, y, spacing, include_endpoint=True):
    distance = cumulative_distance(x, y)
    keep = np.concatenate(([True], np.diff(distance) > 1e-6))
    x, y, distance = x[keep], y[keep], distance[keep]
    samples = np.arange(0.0, distance[-1], spacing)
    if include_endpoint and distance[-1] - samples[-1] > spacing * 0.25:
        samples = np.append(samples, distance[-1])
    return np.interp(samples, distance, x), np.interp(samples, distance, y), samples


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('input_csv', type=Path)
    parser.add_argument('output_stem', type=Path)
    parser.add_argument('--right-offset-m', type=float, default=1.0)
    parser.add_argument('--spacing-m', type=float, default=1.0)
    parser.add_argument('--smooth-window', type=int, default=21)
    parser.add_argument('--target-speed', type=float, default=0.30)
    args = parser.parse_args()

    with args.input_csv.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 3:
        raise ValueError('route requires at least three points')

    x = np.array([float(row['utm_easting_m']) for row in rows])
    y = np.array([float(row['utm_northing_m']) for row in rows])
    x = smooth(x, args.smooth_window)
    y = smooth(y, args.smooth_window)

    # Work on a uniform intermediate path so tangent estimates are stable.
    x, y, _ = interpolate_by_distance(x, y, 0.20)
    tx = np.gradient(x)
    ty = np.gradient(y)
    norm = np.hypot(tx, ty)
    norm[norm < 1e-9] = 1.0

    # In East/North coordinates the right normal of tangent (tx, ty) is
    # (ty, -tx). Positive offset therefore means right of travel.
    x = x + args.right_offset_m * ty / norm
    y = y - args.right_offset_m * tx / norm
    x, y, chainage = interpolate_by_distance(
        x, y, args.spacing_m, include_endpoint=False)

    zone = rows[0].get('utm_zone', '52N')
    zone_number = int(''.join(character for character in zone if character.isdigit()))
    northern = zone.upper().endswith('N')
    epsg = (32600 if northern else 32700) + zone_number
    to_wgs84 = Transformer.from_crs(f'EPSG:{epsg}', 'EPSG:4326', always_xy=True)
    longitude, latitude = to_wgs84.transform(x, y)

    csv_path = args.output_stem.with_suffix('.csv')
    txt_path = args.output_stem.with_suffix('.txt')
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open('x', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow([
            'index', 'distance_m', 'latitude', 'longitude',
            'utm_easting_m', 'utm_northing_m', 'utm_zone', 'target_speed',
        ])
        for index in range(len(x)):
            writer.writerow([
                index, f'{chainage[index]:.3f}', f'{latitude[index]:.10f}',
                f'{longitude[index]:.10f}', f'{x[index]:.4f}', f'{y[index]:.4f}',
                zone, f'{args.target_speed:.3f}',
            ])
    with txt_path.open('x', encoding='utf-8') as stream:
        for east, north in zip(x, y):
            stream.write(f'{east:.4f}\t{north:.4f}\t{args.target_speed:.3f}\n')

    segment_lengths = np.hypot(np.diff(x), np.diff(y))
    print(f'created {len(x)} points: {csv_path}')
    print(f'route length: {chainage[-1]:.2f} m')
    print(f'interval min/mean/max: {segment_lengths.min():.3f}/'
          f'{segment_lengths.mean():.3f}/{segment_lengths.max():.3f} m')


if __name__ == '__main__':
    main()
