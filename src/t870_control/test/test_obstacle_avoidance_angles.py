"""정적장애물 회피가 차량 앞쪽 좌우를 보는지, 회피 방향이 맞는지 고정한다.

mission_common.scan_points 는 raw 스캔 각도를 쓴다. 이 프로젝트는 raw
180도가 차량 정면이므로 forward_angle_deg 를 더해 넘겨야 한다. 보정이
없으면 뒤를 보고, 좌우까지 뒤바뀌어 장애물 쪽으로 조향한다.
"""
import math
import time

import pytest

from t870_control.obstacle_avoidance_node import (
    ObstacleAvoidance, blended_command)


class FakeScan:
    def __init__(self, distance, count, raw_center_deg):
        self.angle_min = 0.0
        self.angle_increment = math.radians(1.0)
        self.range_min = 0.05
        self.range_max = 12.0
        self.ranges = [float('inf')] * 360
        start = int(raw_center_deg) - count // 2
        for offset in range(count):
            self.ranges[(start + offset) % 360] = distance


class FakeTwist:
    def __init__(self, x=0.56, z=0.0):
        self.linear = type('L', (), {'x': x})()
        self.angular = type('A', (), {'z': z})()


def make_node(**params):
    values = {
        'speed_mps': 0.56,
        'steer_rad': 0.12,
        'maximum_steering_rad': 0.20,
        'correction_step_rad': 0.025,
        'detect_distance_m': 1.2,
        'detection_source': 'lidar',
        'forward_angle_deg': 180.0,
        'side_center_deg': 55.0,
        'sector_half_deg': 25.0,
        'min_points': 3,
        'sensor_timeout_sec': 0.5,
        # 근접 인계. 기본은 꺼진 상태로 두어 기존 테스트의 전제를 지킨다.
        'takeover_distance_m': 0.0,
        'takeover_gps_weight': 0.2,
        'takeover_steer_rad': 0.24,
        'takeover_gain': 1.0,
        'takeover_weight_rate': 2.0,
        'takeover_hold_sec': 3.2,
    }
    values.update(params)
    node = object.__new__(ObstacleAvoidance)
    node.active = True
    node.source = values['detection_source']
    node.frame = None
    node.frame_time = time.monotonic()
    node.scan = None
    node.scan_time = time.monotonic()
    node.gps_command = FakeTwist()
    node.gps_time = time.monotonic()
    node.correction = 0.0
    # 정지/검출 로그가 생겨서 bare 생성 노드에도 로거가 필요하다.
    node.get_logger = lambda: type('L', (), {'warning': lambda _s, *a: None})()
    node.stop_log_time = 0.0
    node.detect_log_time = 0.0
    node.last_label = None
    node.takeover = False
    node.gps_weight = 1.0
    node.takeover_seen = 0.0
    node.value = values.get
    node.published = []
    node.pub = type('P', (), {
        'publish': lambda _s, m: node.published.append(m)})()
    return node


def settle(node, ticks=20):
    """보정이 목표까지 차오르도록 여러 틱 돌린다."""
    for _ in range(ticks):
        node.scan_time = time.monotonic()
        node.frame_time = time.monotonic()
        node.gps_time = time.monotonic()
        node.tick()
    return node.published[-1]


def test_left_obstacle_steers_right():
    # 차량 왼쪽 +55도 = raw 235도.
    node = make_node()
    node.scan = FakeScan(0.8, 20, raw_center_deg=235.0)
    command = settle(node)
    # 프로젝트 규약: +angular.z = 오른쪽. 왼쪽 장애물이면 오른쪽으로 피한다.
    assert command.angular.z > 0.0
    assert command.angular.z == pytest.approx(0.12, abs=1e-6)


def test_right_obstacle_steers_left():
    # 차량 오른쪽 -55도 = raw 125도.
    node = make_node()
    node.scan = FakeScan(0.8, 20, raw_center_deg=125.0)
    command = settle(node)
    assert command.angular.z < 0.0
    assert command.angular.z == pytest.approx(-0.12, abs=1e-6)


def test_rear_object_is_ignored():
    # 보정이 없던 시절 보던 자리(raw 55도 = 차량 -125도, 오른쪽 뒤).
    node = make_node()
    node.scan = FakeScan(0.8, 20, raw_center_deg=55.0)
    command = settle(node)
    assert command.angular.z == pytest.approx(0.0, abs=1e-6)


def test_clear_path_keeps_gps_steering():
    node = make_node()
    node.scan = FakeScan(5.0, 20, raw_center_deg=235.0)   # 1.2 m 밖
    node.gps_command = FakeTwist(z=0.05)
    command = settle(node)
    assert command.angular.z == pytest.approx(0.05, abs=1e-6)


def test_gps_steering_is_kept_and_correction_added():
    node = make_node()
    node.scan = FakeScan(0.8, 20, raw_center_deg=235.0)
    node.gps_command = FakeTwist(z=0.05)
    command = settle(node)
    # GPS 곡선을 유지한 채 보정만 더한다.
    assert command.angular.z == pytest.approx(0.05 + 0.12, abs=1e-6)


def test_final_steering_is_clamped():
    node = make_node()
    node.scan = FakeScan(0.8, 20, raw_center_deg=235.0)
    node.gps_command = FakeTwist(z=0.19)
    command = settle(node)
    assert command.angular.z == pytest.approx(0.20, abs=1e-6)


def test_correction_ramps_instead_of_jumping():
    node = make_node()
    node.scan = FakeScan(0.8, 20, raw_center_deg=235.0)
    node.tick()
    assert node.published[-1].angular.z == pytest.approx(0.025, abs=1e-6)


def test_speed_is_limited_to_lidar_mission_speed():
    node = make_node()
    node.scan = FakeScan(5.0, 20, raw_center_deg=235.0)
    node.gps_command = FakeTwist(x=2.2222)
    command = settle(node)
    assert command.linear.x == pytest.approx(0.56)


def test_stale_sensor_keeps_following_the_gps_path():
    """센서가 끊겨도 서지 않고 GPS 경로로 간다.

    예전에는 속도 0 을 냈다. 그러면 카메라가 0.5 초만 끊겨도 정적장애물
    구간에서 차가 완전히 서고 그대로 주행이 끝난다. 카메라는 회피 보조일
    뿐이고 GPS 경로는 그 자체로 유효하다.
    """
    node = make_node()
    node.scan = FakeScan(0.8, 20, raw_center_deg=235.0)
    node.gps_command = FakeTwist(x=0.56, z=0.05)
    node.scan_time = time.monotonic() - 1.0
    node.tick()
    assert node.published[-1].linear.x == pytest.approx(0.56)
    assert node.published[-1].angular.z == pytest.approx(0.05, abs=1e-6)


def test_stale_sensor_ramps_the_correction_back_to_zero():
    """틀어 둔 조향을 한 번에 0 으로 되돌리면 그 자체가 급조향이다."""
    node = make_node()
    node.scan = FakeScan(0.8, 20, raw_center_deg=235.0)
    settle(node)                       # 보정을 0.12 까지 채운다
    assert node.correction == pytest.approx(0.12, abs=1e-6)
    node.scan_time = time.monotonic() - 1.0
    node.tick()
    # correction_step_rad 만큼만 줄어든다.
    assert node.correction == pytest.approx(0.12 - 0.025, abs=1e-6)


def test_missing_gps_command_still_stops():
    """GPS 명령 자체가 없으면 따라갈 경로가 없다. 이때는 서야 한다."""
    node = make_node()
    node.scan = FakeScan(0.8, 20, raw_center_deg=235.0)
    node.gps_command = None
    node.tick()
    assert node.published[-1].linear.x == 0.0
    assert node.published[-1].angular.z == 0.0


def test_stale_gps_command_still_stops():
    node = make_node()
    node.scan = FakeScan(0.8, 20, raw_center_deg=235.0)
    node.gps_time = time.monotonic() - 1.0
    node.tick()
    assert node.published[-1].linear.x == 0.0


def test_blended_command_clamps_both_ends():
    assert blended_command(9.0, 0.0, 0.0, 0.56, 0.20).linear.x == 0.56
    assert blended_command(-1.0, 0.0, 0.0, 0.56, 0.20).linear.x == 0.0
    assert blended_command(0.5, 0.3, 0.3, 0.56, 0.20).angular.z == 0.20


def test_default_forward_angle_matches_project_convention():
    # 주차 선택기/섹터 필터와 같은 180 을 쓴다. 이것은 수평 기준각이며
    # 라이다를 아래로 기울이는 수직각과는 무관하다.
    from t870_control import obstacle_avoidance_node as module
    source = open(module.__file__, encoding='utf-8').read()
    assert "'forward_angle_deg': 180.0," in source


def test_windows_follow_forward_angle():
    # forward 를 옮기면 창 두 개가 함께 수평으로 회전한다.
    node = make_node(forward_angle_deg=190.0)
    node.scan = FakeScan(0.8, 20, raw_center_deg=245.0)
    assert settle(node).angular.z == pytest.approx(0.12, abs=1e-6)

    node = make_node(forward_angle_deg=190.0)
    node.scan = FakeScan(0.8, 20, raw_center_deg=135.0)
    assert settle(node).angular.z == pytest.approx(-0.12, abs=1e-6)
