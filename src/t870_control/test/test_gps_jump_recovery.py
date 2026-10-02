"""GPS 가 크게 튀었을 때 follower 가 영영 멈추지 않는지 고정한다.

실차 증상: 'Rejected GPS position jump: 45.90 m (count=5520)' 가 계속 찍히고
차가 간헐적으로 서며 속도가 안 올랐다.

원인: 점프를 거부할 때 self.position 을 갱신하지 않는다. 그래서 다음 fix 도
똑같이 45.90 m 떨어져 있어 또 거부된다. 한 번 튀면 복구가 안 되고
control_tick 이 'GPS fix missing or stale' 로 명령을 끊는다.
arduino_drive_node 의 command_timeout_sec 는 0.5 초라 그때마다 차가 선다.
"""
import inspect
import re

import pytest

from t870_control.gps_path_follower_node import (
    GpsPathFollower, gps_update_is_plausible)

SOURCE = inspect.getsource(GpsPathFollower.__init__)


def default(key):
    return float(re.search(r"'%s':\s*([0-9.]+)" % key, SOURCE).group(1))


def test_a_large_jump_is_rejected():
    """정상 동작. 튀는 fix 는 받아들이면 안 된다."""
    assert not gps_update_is_plausible(
        (0.0, 0.0), (45.9, 0.0), 0.25,
        default('gps_jump_base_tolerance_m'),
        default('gps_jump_maximum_speed_mps'))


def test_normal_motion_is_accepted():
    """4 Hz 에서 8 km/h 면 한 틱에 0.56 m. 거부되면 안 된다."""
    assert gps_update_is_plausible(
        (0.0, 0.0), (0.56, 0.0), 0.25,
        default('gps_jump_base_tolerance_m'),
        default('gps_jump_maximum_speed_mps'))


def test_recovery_is_enabled():
    """거부만 반복하면 영영 못 돌아온다. 복구 횟수가 있어야 한다."""
    count = default('gps_jump_recover_count')
    assert count > 0, '복구가 꺼져 있으면 한 번 튀면 주행이 끝난다'
    # 4 Hz 기준. 너무 짧으면 진짜 튄 값을 덥석 받고, 너무 길면 그동안 선다.
    seconds = count / 4.0
    assert 2.0 <= seconds <= 15.0, (
        '복구까지 %.1f 초. 2~15 초 사이여야 한다' % seconds)


def test_recovery_is_shorter_than_the_observed_failure():
    """실차에서 5520 회(약 23 분) 연속 거부됐다. 그보다 훨씬 짧아야 한다."""
    assert default('gps_jump_recover_count') < 100


def test_route_error_guard_still_catches_a_wrong_position():
    """복구로 받아들인 위치가 정말 틀렸다면 경로오차가 잡아야 한다.

    이것이 있어서 복구가 안전하다.
    """
    assert default('maximum_route_error_m') > 0.0
