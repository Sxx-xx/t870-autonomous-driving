import math

import pytest

from t870_control.gps_path_follower_node import (
    bounded_nearest_index, direction_run_bounds, directions_from_cusps,
    gps_update_is_plausible, load_route, parse_path_switch_request,
    rate_limit_angle, reverse_flags_from_ranges,
    speed_banded_lookahead)


def test_competition_path_switch_request_is_atomic():
    assert parse_path_switch_request(
        '/tmp/P.csv::start=638::reverse=649:655') == (
            '/tmp/P.csv', 638, '649:655')
    assert parse_path_switch_request('/tmp/plain.csv') == (
        '/tmp/plain.csv', None, None)


def test_explicit_reverse_waypoint_ranges():
    flags, cusps = reverse_flags_from_ranges(12, '3:5,8:9')
    assert [index for index, reverse in enumerate(flags) if reverse] == [
        3, 4, 5, 8, 9]
    assert cusps == [2, 5, 7, 9]


@pytest.mark.parametrize(
    ('ranges', 'point_count', 'reverse_shift_wp', 'forward_shift_wp'),
    [
        ('306:318', 697, 305, 318),   # T_Parking1  T주차
        ('309:321', 702, 308, 321),   # T_Parking2  T주차
        ('641:647', 716, 640, 647),   # P_Parking1  평행주차
        ('649:655', 717, 648, 655),   # P_Parking2  평행주차
    ])
def test_competition_shift_waypoints(
        ranges, point_count, reverse_shift_wp, forward_shift_wp):
    """대회 후진 범위가 실제로 기어를 바꾸는 WP 번호를 고정한다.

    'start:stop' 은 후진으로 '지나가는' WP 집합이다. 기어가 바뀌는 자리는
    그 양 끝의 첨점, 즉 start-1 과 stop 이다. control_tick 은 최근접점이
    첨점에 닿으면 look 을 cusp+1 로 밀어 그 너머 방향을 즉시 쓰므로,
    후진 전환은 start-1 에서, 전진 복귀는 stop 에서 일어난다.
    """
    flags, cusps = reverse_flags_from_ranges(point_count, ranges)

    assert cusps == [reverse_shift_wp, forward_shift_wp]
    # 첨점에 닿은 순간 적용되는 방향은 cusp+1 의 플래그다.
    assert flags[reverse_shift_wp + 1] is True
    assert flags[forward_shift_wp + 1] is False
    # 첨점 자체는 아직 이전 방향으로 도착하는 정지점이다.
    assert flags[reverse_shift_wp] is False
    assert flags[forward_shift_wp] is True


@pytest.mark.parametrize(
    ('speed', 'expected'),
    [
        (0.0, 1.5),
        (2.0 / 3.6, 1.5),
        (3.0 / 3.6, 2.2),
        (4.0 / 3.6, 2.2),
        (5.0 / 3.6, 3.2),
        (7.0 / 3.6, 4.4),
        (10.0 / 3.6, 6.0),
        (12.0 / 3.6, 6.0),
    ],
)
def test_speed_banded_lookahead(speed, expected):
    assert speed_banded_lookahead(
        speed,
        tuple(kmh / 3.6 for kmh in (2, 4, 6, 8, 10)),
        (1.5, 2.2, 3.2, 4.4, 6.0)) == pytest.approx(expected)


def test_load_longitude_latitude_route(tmp_path):
    route = tmp_path / 'route.csv'
    route.write_text(
        'longitude,latitude,id\n'
        '129.087006707083,35.072957858301,true\n'
        '129.087016872311,35.072961245460,true\n',
        encoding='utf-8')

    points, relative, has_speed = load_route(route)

    assert not relative
    assert not has_speed
    assert len(points) == 2
    assert points[0][0] == pytest.approx(507932.453, abs=0.01)
    assert points[0][1] == pytest.approx(3881137.323, abs=0.01)
    assert points[0][2] == 0.0
    assert points[1][0] > points[0][0]


def test_competition_utm_route_reports_no_speed_column(tmp_path):
    # QGIS truncates field names to ten characters, so the competition
    # exports carry 'target_spe' (a boolean) instead of 'target_speed'.
    # Every point would otherwise load as speed 0 and be treated as a stop
    # waypoint, holding the vehicle for waypoint_hold_sec at every metre.
    route = tmp_path / 'competition.csv'
    route.write_text(
        'index,easting,northing,target_spe\n'
        '0,332262.3837,4128603.3390,true\n'
        '1,332261.4603,4128603.7210,true\n',
        encoding='utf-8')

    points, relative, has_speed = load_route(route)

    assert not relative
    assert not has_speed
    assert [point[2] for point in points] == [0.0, 0.0]


def test_utm_route_with_speed_column_reports_speed(tmp_path):
    route = tmp_path / 'recorded.csv'
    route.write_text(
        'easting,northing,target_speed\n'
        '332262.3837,4128603.3390,2.2222\n'
        '332261.4603,4128603.7210,0.0\n',
        encoding='utf-8')

    points, relative, has_speed = load_route(route)

    assert not relative
    assert has_speed
    assert points[0][2] == pytest.approx(2.2222)
    assert points[1][2] == 0.0


def test_rate_limit_angle_handles_wraparound():
    result = rate_limit_angle(
        math.radians(179.0), math.radians(-170.0), 0.1, math.radians(20.0))
    assert math.degrees(result) == pytest.approx(-179.0)


def test_rate_limit_angle_limits_large_jump():
    result = rate_limit_angle(0.0, 2.0, 0.1, 1.0)
    assert result == pytest.approx(0.1)


def test_gps_position_jump_guard():
    assert gps_update_is_plausible((0.0, 0.0), (4.0, 0.0), 0.25, 3.0, 6.0)
    assert not gps_update_is_plausible(
        (0.0, 0.0), (12.0, 0.0), 0.25, 3.0, 6.0)


def test_nearest_index_uses_local_window_and_advance_limit():
    points = [(float(index), 0.0, 0.0) for index in range(100)]
    assert bounded_nearest_index(
        points, 73.0, 0.0, 0, False, 5, 30, 10) == 73
    assert bounded_nearest_index(
        points, 73.0, 0.0, 20, True, 5, 30, 10) == 30


def test_nearest_index_never_moves_back_after_initialization():
    points = [(float(index), 0.0, 0.0) for index in range(20)]
    assert bounded_nearest_index(
        points, 7.0, 0.0, 10, True, 5, 10, 4) == 10


def test_cusp_point_keeps_old_direction_and_next_point_reverses():
    # WP1에서 되짚어 나가는 간단한 cusp. 정지/방향 전환은 cusp 다음인
    # WP2부터 적용되어야 한다.
    points = [
        (0.0, 0.0, 1.0),
        (1.0, 0.0, 1.0),
        (0.0, 0.0, 1.0),
        (-1.0, 0.0, 1.0),
    ]

    reverse, cusps = directions_from_cusps(points, math.radians(45.0))

    assert cusps == [1]
    assert reverse == [False, False, True, True]


def test_direction_run_caps_forward_lookahead_at_cusp():
    # p_b: WP11까지 전진, WP12~16 후진, WP17부터 다시 전진.
    flags = [False] * 12 + [True] * 5 + [False] * 4

    assert direction_run_bounds(flags, 9, False) == (0, 11)
    assert direction_run_bounds(flags, 12, True) == (12, 16)
    assert direction_run_bounds(flags, 17, False) == (17, 20)
