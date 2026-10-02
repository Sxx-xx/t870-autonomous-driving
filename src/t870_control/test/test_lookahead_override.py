"""특정 WP 구간에서만 LAD 를 짧게 쓰는 기능.

WP199~235 는 S자(헤어핀, 반경 약 7 m)다. 2 km/h 속도 밴드 LAD 5.0 m 는
여기서 너무 길어 순수추종이 코너를 크게 자른다. 실측 최대 횡오차

    LAD 5.0 m -> 0.80 m    3.0 m -> 0.32 m
    LAD 1.5 m -> 0.25 m    1.0 m -> 0.22 m

조향률 제한(maximum_steering_rate_rad_s 0.60)이 있어 1.0 m 에서도 진동하지
않는다(부호 반전 3회, 조향 포화 4%).
"""
import io
import re

import pytest

from t870_control.gps_path_follower_node import (
    lookahead_for_waypoint, parse_waypoint_ranges)

LAUNCH = 'src/t870_control/launch/t870_competition.launch.py'


def test_parses_a_single_range():
    assert parse_waypoint_ranges('199:235') == [(199, 235)]


def test_parses_several_ranges_and_dashes():
    assert parse_waypoint_ranges('10:20, 30-40') == [(10, 20), (30, 40)]


def test_empty_means_no_override():
    assert parse_waypoint_ranges('') == []


@pytest.mark.parametrize('text', ['199', '199:', '235:199'])
def test_bad_range_is_rejected(text):
    with pytest.raises(ValueError):
        parse_waypoint_ranges(text)


@pytest.mark.parametrize(
    ('waypoint', 'expected'),
    [(198, 5.0), (199, 1.0), (215, 1.0), (235, 1.0), (236, 5.0)])
def test_override_applies_only_inside_the_range(waypoint, expected):
    ranges = parse_waypoint_ranges('199:235')
    assert lookahead_for_waypoint(waypoint, ranges, 1.0, 5.0) == expected


def test_zero_override_keeps_the_speed_band_value():
    """0 이하면 기능을 끈다. 기존 동작 그대로."""
    ranges = parse_waypoint_ranges('199:235')
    assert lookahead_for_waypoint(215, ranges, 0.0, 5.0) == 5.0


def test_no_range_keeps_the_speed_band_value():
    assert lookahead_for_waypoint(215, [], 1.0, 5.0) == 5.0


def test_competition_launch_covers_the_s_course():
    """대회 launch 가 S자 구간에 짧은 LAD 를 지정하는지."""
    launch = io.open(LAUNCH, encoding='utf-8').read()
    ranges = re.search(
        r"'gps_lookahead_override_ranges': '([^']*)'", launch).group(1)
    override = float(re.search(
        r"'gps_lookahead_override_m': '([^']*)'", launch).group(1))
    parsed = parse_waypoint_ranges(ranges)
    assert parsed, 'S자 구간 지정이 비어 있다'
    start, stop = parsed[0]
    assert start <= 201 and stop >= 227, (
        '정적장애물 구간(201~227)을 덮어야 한다: %s' % parsed)
    assert 0.0 < override <= 2.0, (
        'LAD %.2f m. 너무 길면 코너를 자르고, 휠베이스(0.73) 아래로 내리면 '
        '순수추종이 불안정해진다' % override)
