"""장애물 근접 시 GPS 조향 주도권을 넘기는 동작.

보정을 더하기만 해서는 실제로 안 비켜진다. 순수추종이 경로로 되당기는
힘이 보정을 거의 그대로 상쇄하기 때문이다(모의: 지시 1.41 m -> 실제 0.3 m).

장애물 0.8 m 치우침, 필요 이격 1.128 m 기준 모의 결과
    인계 없음                        최근접 0.96 m   부딪힘
    인계 3 m, w 0.2, cap 0.24        최근접 1.30 m   통과

w 를 0 으로 두면 중앙 장애물도 피하지만 차선을 벗어난다. 이 대회에서
장애물은 절대 중앙에 놓이지 않으므로 0.2 를 쓴다.
"""
import inspect
import math
import re

import pytest

from t870_control.obstacle_avoidance_node import (
    ObstacleAvoidance, blended_command)

SOURCE = inspect.getsource(ObstacleAvoidance.__init__)


def default(key):
    return float(re.search(r"'%s':\s*([0-9.]+)" % key, SOURCE).group(1))


def make():
    node = object.__new__(ObstacleAvoidance)
    node.takeover = False
    node.gps_weight = 1.0
    node.logged = []
    node.get_logger = lambda: type('L', (), {
        'warning': lambda _s, *a: node.logged.append(a[0] if a else '')})()
    node.value = {
        'takeover_distance_m': 3.0,
        'takeover_gps_weight': 0.2,
        'takeover_steer_rad': 0.24,
        'takeover_gain': 1.0,
        'takeover_weight_rate': 2.0,
        'takeover_hold_sec': 3.2,
    }.get
    return node


# --- 가중치 혼합 -------------------------------------------------------

def test_weight_one_is_the_old_behaviour():
    assert blended_command(0.5, 0.10, 0.05, 1.0, 0.24).angular.z == \
        pytest.approx(blended_command(0.5, 0.10, 0.05, 1.0, 0.24, 1.0).angular.z)


def test_weight_reduces_the_gps_share_only():
    full = blended_command(0.5, 0.10, 0.05, 1.0, 0.24, 1.0).angular.z
    weak = blended_command(0.5, 0.10, 0.05, 1.0, 0.24, 0.2).angular.z
    assert full == pytest.approx(0.15)
    assert weak == pytest.approx(0.10 * 0.2 + 0.05)
    assert abs(weak) < abs(full), 'GPS 몫만 줄고 보정은 그대로여야 한다'


def test_correction_survives_a_strong_opposing_gps_steer():
    """되당김이 보정을 상쇄하던 것이 이 기능의 이유다."""
    assert blended_command(0.5, -0.12, 0.12, 1.0, 0.24, 1.0).angular.z == \
        pytest.approx(0.0)
    assert blended_command(0.5, -0.12, 0.12, 1.0, 0.24, 0.2).angular.z > 0.09


# --- 래치 --------------------------------------------------------------

def test_latches_inside_the_takeover_distance():
    node = make()
    node.set_takeover(True, 2.4)
    assert node.takeover
    assert '인계' in node.logged[0]


def test_releases_when_the_obstacle_is_gone():
    node = make()
    node.set_takeover(True, 2.4)
    node.set_takeover(False, None)
    assert not node.takeover
    assert '해제' in node.logged[-1]


def test_repeated_calls_log_once():
    node = make()
    for _ in range(5):
        node.set_takeover(True, 2.0)
    assert len(node.logged) == 1


# --- 가중치 램프 -------------------------------------------------------

def test_weight_ramps_down_and_back():
    node = make()
    node.set_takeover(True, 2.0)
    first = node.update_gps_weight(0.05)
    assert first == pytest.approx(0.9), '계단이 아니라 램프여야 한다'
    for _ in range(50):
        node.update_gps_weight(0.05)
    assert node.gps_weight == pytest.approx(0.2)
    node.set_takeover(False, None)
    for _ in range(50):
        node.update_gps_weight(0.05)
    assert node.gps_weight == pytest.approx(1.0)


# --- 기본값 ------------------------------------------------------------

def test_takeover_cap_is_not_throttled_by_the_final_clamp():
    """인계 중에는 보정이 곧 전체 조향이다. 최종 클램프가 더 낮으면
    takeover_steer_rad 를 올린 의미가 없다."""
    assert default('maximum_steering_rad') >= default('takeover_steer_rad')


def test_takeover_is_stronger_than_the_cruise_setting():
    assert default('takeover_steer_rad') > default('steer_rad')
    assert default('takeover_gain') > default('bev_gain')


def test_some_path_following_is_kept():
    """w=0 이면 중앙 장애물도 피하지만 차선을 벗어난다."""
    assert 0.0 < default('takeover_gps_weight') <= 0.5


# --- 통과까지 유지 --------------------------------------------------------
# 검출은 장애물이 센서 앞 0.3 m 안으로 들어오면 끊긴다. 그 시점에 앞범퍼만
# 지난 상태라, 바로 놓으면 차체가 아직 옆을 지나는 중에 조향이 돌아가
# 다시 붙는다.
#
#   앞범퍼가 장애물 뒷면 통과  1.76 m -> 3.2 초 (전체 통과의 56%)
#   뒷범퍼까지 완전 통과       3.16 m -> 5.7 초
#
# 앞범퍼만 지나면 복귀해도 된다. 후륜축 기준 회전이라 장애물 쪽으로 틀면
# 뒷 모서리는 오히려 반대쪽으로 벌어진다.

def test_hold_covers_the_front_bumper_passing_the_obstacle():
    """3.2 초가 어디서 나온 값인지 고정한다."""
    speed = 2.0 / 3.6
    minimum_distance = 0.3
    obstacle_along_travel = math.hypot(1.26, 0.73)     # 최악 각도 = 대각선
    needed = (minimum_distance + obstacle_along_travel) / speed
    assert default('takeover_hold_sec') >= needed - 0.05, (
        '유지 %.1f 초로는 앞범퍼가 장애물을 못 지난다 (%.1f 초 필요)'
        % (default('takeover_hold_sec'), needed))


def test_hold_is_not_long_enough_to_lose_the_path():
    """완전 통과(5.7 초)까지 물면 차가 61도 돌아 헤어핀에서 경로를 잃는다."""
    full = (0.3 + math.hypot(1.26, 0.73) + 1.40) / (2.0 / 3.6)
    assert default('takeover_hold_sec') < full * 0.75, (
        '유지가 너무 길다. %.1f 초는 완전통과 %.1f 초의 %.0f%% 다'
        % (default('takeover_hold_sec'), full,
           100 * default('takeover_hold_sec') / full))
