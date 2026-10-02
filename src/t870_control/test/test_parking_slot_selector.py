import math
from types import SimpleNamespace

import pytest

from t870_control.parking_slot_selector_node import (
    count_points_in_window,
    vehicle_angle_deg,
)


def test_vehicle_angle_uses_front_as_zero():
    assert vehicle_angle_deg(math.radians(180.0), 180.0) == 0.0
    assert vehicle_angle_deg(math.radians(135.0), 180.0) == -45.0
    assert vehicle_angle_deg(math.radians(240.0), 180.0) == pytest.approx(60.0)


def test_counts_only_zone_b_window_and_range():
    # Raw scan spans 130..150 degrees. With forward=180 this is -50..-30.
    scan = SimpleNamespace(
        angle_min=math.radians(130.0),
        angle_increment=math.radians(5.0),
        ranges=[3.0, 3.1, 3.2, 3.3, 8.0],
    )
    count, nearest = count_points_in_window(
        scan, 180.0, -45.0, -30.0, 2.5, 3.7)
    assert count == 3
    assert nearest == 3.1


def test_ignores_invalid_ranges():
    scan = SimpleNamespace(
        angle_min=math.radians(135.0),
        angle_increment=math.radians(5.0),
        ranges=[math.nan, math.inf, 2.0, 4.0],
    )
    count, nearest = count_points_in_window(
        scan, 180.0, -45.0, -30.0, 2.5, 3.7)
    assert count == 0
    assert math.isinf(nearest)


def test_counts_driving_zone_b_window():
    # During the real wp5 approach the zone-B cone is in -55..-30 degrees.
    scan = SimpleNamespace(
        angle_min=math.radians(125.0),
        angle_increment=math.radians(5.0),
        ranges=[3.1, 3.2, 3.3, 3.4, 4.8],
    )
    count, nearest = count_points_in_window(
        scan, 180.0, -55.0, -30.0, 2.8, 4.5)
    assert count == 4
    assert nearest == 3.1
