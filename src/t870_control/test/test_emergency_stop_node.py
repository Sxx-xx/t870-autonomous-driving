"""동적장애물 정지 검증.

정면에서 한 번 잡으면 정해진 시간만 서고 복귀한다. 그 뒤로는 감시를 끈다.
"""
import math

import pytest

from t870_control.emergency_stop_node import EmergencyStop


class FakeScan:
    """정면(raw 180도) 기준으로 점을 만드는 최소 LaserScan."""

    def __init__(self, distance, count, center_deg=180.0):
        self.angle_min = 0.0
        self.angle_increment = math.radians(1.0)
        self.range_min = 0.05
        self.range_max = 12.0
        self.ranges = [float('inf')] * 360
        start = int(center_deg) - count // 2
        for offset in range(count):
            self.ranges[(start + offset) % 360] = distance


class FakeClock:
    def __init__(self):
        self.value = 100.0

    def __call__(self):
        return self.value


class Msg:
    def __init__(self, data):
        self.data = data


def make_node(monkeypatch, clock, **params):
    values = {
        'front_center_deg': 180.0,
        'detect_angle_half_deg': 30.0,
        'detect_distance_m': 3.0,
        'detect_min_points': 3,
        'stop_duration_sec': 3.0,
        'disable_after_stop': True,
        'scan_timeout_sec': 0.5,
    }
    values.update(params)
    node = object.__new__(EmergencyStop)
    node.active_zone = False
    node.finished = False
    node.hold_until = 0.0
    node.was_stopping = False
    node.last_scan = 0.0
    node.p = values.get
    node.published = []
    node.stop_pub = type('P', (), {
        'publish': lambda _s, m: node.published.append(m.data)})()
    node.cmd_pub = type('P', (), {'publish': lambda _s, m: None})()
    # 감시 종료를 미션 매니저에 알리는 발행자. 그쪽이 이 신호로 남은
    # 구간의 속도 제한을 일찍 푼다.
    node.done_published = []
    node.done_pub = type('P', (), {
        'publish': lambda _s, m: node.done_published.append(m.data)})()
    node.get_logger = lambda: type('L', (), {'warning': lambda _s, *a: None})()
    monkeypatch.setattr('t870_control.emergency_stop_node.time.monotonic',
                        clock)
    return node


def stopping(node):
    """tick 을 한 번 돌리고 정지 여부를 돌려준다."""
    node.published.clear()
    node.tick()
    return node.published[-1]


def test_no_stop_outside_zone(monkeypatch):
    clock = FakeClock()
    node = make_node(monkeypatch, clock)
    node.on_mission(Msg('NONE'))
    node.on_scan(FakeScan(1.0, 20))
    assert node.hold_until == 0.0
    assert stopping(node) is False


def test_obstacle_in_zone_stops(monkeypatch):
    clock = FakeClock()
    node = make_node(monkeypatch, clock)
    node.on_mission(Msg('ESTOP'))
    node.on_scan(FakeScan(1.5, 20))
    assert node.hold_until == pytest.approx(clock.value + 3.0)
    assert stopping(node) is True


def test_obstacle_still_present_does_not_extend_stop(monkeypatch):
    # 치우는 사람이 섹터 안에 있어도 정지가 연장되면 안 된다.
    clock = FakeClock()
    node = make_node(monkeypatch, clock)
    node.on_mission(Msg('ESTOP'))
    node.on_scan(FakeScan(1.5, 20))
    release = node.hold_until
    for _ in range(20):
        clock.value += 0.1
        node.on_scan(FakeScan(1.0, 20))
    assert node.hold_until == pytest.approx(release)


def test_resumes_after_three_seconds(monkeypatch):
    clock = FakeClock()
    node = make_node(monkeypatch, clock)
    node.on_mission(Msg('ESTOP'))
    node.on_scan(FakeScan(1.5, 20))
    clock.value += 2.9
    assert stopping(node) is True
    clock.value += 0.2          # 합계 3.1 s > 3.0 s
    assert stopping(node) is False


def test_monitoring_off_after_one_stop(monkeypatch):
    # 복귀 후에는 평행주차 구간 벽을 잡아 다시 서면 안 된다.
    clock = FakeClock()
    node = make_node(monkeypatch, clock)
    node.on_mission(Msg('ESTOP'))
    node.on_scan(FakeScan(1.5, 20))
    clock.value += 3.1
    assert stopping(node) is False
    assert node.finished is True

    clock.value += 1.0
    node.on_scan(FakeScan(1.0, 20))
    assert node.hold_until == 0.0
    assert stopping(node) is False


def test_disable_after_stop_false_allows_second_stop(monkeypatch):
    clock = FakeClock()
    node = make_node(monkeypatch, clock, disable_after_stop=False)
    node.on_mission(Msg('ESTOP'))
    node.on_scan(FakeScan(1.5, 20))
    # 정지 중에도 라이다는 10 Hz 로 계속 들어온다. 치우는 중이라 물체가
    # 아직 보인다고 두자. 래치 때문에 정지는 연장되지 않아야 한다.
    for _ in range(31):
        clock.value += 0.1
        node.on_scan(FakeScan(1.0, 20))
    assert stopping(node) is False
    assert node.finished is False

    # 감시가 살아 있으므로 다시 나타나면 또 선다.
    clock.value += 0.1
    node.on_scan(FakeScan(1.0, 20))
    assert stopping(node) is True


def test_beyond_detect_distance_is_ignored(monkeypatch):
    clock = FakeClock()
    node = make_node(monkeypatch, clock)
    node.on_mission(Msg('ESTOP'))
    node.on_scan(FakeScan(4.0, 20))     # 3.0 m 밖
    assert node.hold_until == 0.0


def test_too_few_points_is_ignored(monkeypatch):
    clock = FakeClock()
    node = make_node(monkeypatch, clock)
    node.on_mission(Msg('ESTOP'))
    node.on_scan(FakeScan(1.5, 2))      # 3점 미만
    assert node.hold_until == 0.0


def test_detects_across_full_60_degree_window(monkeypatch):
    # 좌우 30도씩. 정면에서 25도 벗어난 물체도 잡혀야 한다.
    clock = FakeClock()
    node = make_node(monkeypatch, clock)
    node.on_mission(Msg('ESTOP'))
    node.on_scan(FakeScan(1.5, 6, center_deg=205.0))
    assert node.hold_until > 0.0


def test_outside_window_is_ignored(monkeypatch):
    # 정면에서 50도 벗어난 물체는 창(30도) 밖이다.
    clock = FakeClock()
    node = make_node(monkeypatch, clock)
    node.on_mission(Msg('ESTOP'))
    node.on_scan(FakeScan(1.5, 6, center_deg=230.0))
    assert node.hold_until == 0.0


def test_rear_object_is_ignored_with_front_180(monkeypatch):
    clock = FakeClock()
    node = make_node(monkeypatch, clock)
    node.on_mission(Msg('ESTOP'))
    node.on_scan(FakeScan(1.0, 20, center_deg=0.0))
    assert node.hold_until == 0.0


def test_stale_scan_stops_while_watching(monkeypatch):
    clock = FakeClock()
    node = make_node(monkeypatch, clock)
    node.on_mission(Msg('ESTOP'))
    node.on_scan(FakeScan(9.0, 20))
    assert stopping(node) is False
    clock.value += 1.0                  # scan_timeout 0.5 s 초과
    assert stopping(node) is True


def test_stale_scan_after_finish_does_not_stop(monkeypatch):
    clock = FakeClock()
    node = make_node(monkeypatch, clock)
    node.on_mission(Msg('ESTOP'))
    node.on_scan(FakeScan(1.5, 20))
    clock.value += 3.1
    assert stopping(node) is False
    clock.value += 5.0                  # 스캔이 끊겨도
    assert stopping(node) is False


def test_completion_is_published_for_the_mission_manager(monkeypatch):
    """감시가 끝나면 그 사실을 알려야 한다.

    미션 매니저는 WP531~588 전체를 미션 속도로 묶는다. 장애물은 보통
    앞쪽에서 처리되고 그 뒤로는 라이다를 아예 안 보는데도 남은 50 여 m 를
    2 km/h 로 기어가 복귀가 한참 걸렸다.
    """
    clock = FakeClock()
    node = make_node(monkeypatch, clock)
    node.on_mission(Msg('ESTOP'))
    node.last_scan = clock()

    node.tick()
    assert node.done_published[-1] is False        # 아직 안 끝났다

    node.on_scan(FakeScan(1.5, 20))                # 장애물
    node.tick()
    assert node.done_published[-1] is False        # 정지 중
    clock.value += 3.5
    node.tick()
    assert node.done_published[-1] is True         # 끝났다
    node.tick()
    assert node.done_published[-1] is True         # 계속 알린다
