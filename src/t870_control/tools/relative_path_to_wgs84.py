#!/usr/bin/env python3
"""Anchor a relative distance/bearing route and export WGS84 lon/lat."""

import argparse
import csv
import math
from pathlib import Path

from pyproj import Transformer


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('input')
    parser.add_argument('output')
    parser.add_argument('--origin-easting', type=float, required=True)
    parser.add_argument('--origin-northing', type=float, required=True)
    parser.add_argument('--source-epsg', type=int, default=32652)
    args = parser.parse_args()

    with Path(args.input).open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 2 or not {'distance', 'angle'} <= set(rows[0]):
        raise ValueError('input must contain distance and angle columns')

    transformer = Transformer.from_crs(
        f'EPSG:{args.source_epsg}', 'EPSG:4326', always_xy=True)
    east = args.origin_easting
    north = args.origin_northing
    previous_distance = float(rows[0]['distance'])
    previous_bearing = math.radians(float(rows[0]['angle']))

    with Path(args.output).open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(('longitude', 'latitude', 'id', 'distance', 'angle'))
        for index, row in enumerate(rows):
            distance = float(row['distance'])
            if index:
                step = distance - previous_distance
                east += step * math.sin(previous_bearing)
                north += step * math.cos(previous_bearing)
            longitude, latitude = transformer.transform(east, north)
            writer.writerow((
                f'{longitude:.12f}', f'{latitude:.12f}',
                row.get('id', '1'), row['distance'], row['angle']))
            previous_distance = distance
            previous_bearing = math.radians(float(row['angle']))


if __name__ == '__main__':
    main()
