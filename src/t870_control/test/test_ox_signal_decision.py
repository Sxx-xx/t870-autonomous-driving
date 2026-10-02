"""마지막 WP 표지(화살표/X) 좌우 배치 판단 검증.

  왼쪽부터 빨강, 초록 -> OX_right 로 전환 (ox_straight=True)
  왼쪽부터 초록, 빨강 -> 가던 경로 그대로 (ox_straight=False)

여기는 신호등이 아니다. 표지가 원형 램프가 아니라 화살표와 X 라서
원형도/종횡비 필터를 쓰면 안 된다. 그 점을 합성 프레임으로 고정한다.
"""
import sys

sys.path = [p for p in sys.path
            if p != '/usr/local/lib/python3.12/dist-packages']

import cv2
import numpy as np
import pytest

from t870_control.ox_signal_detector_node import (
    color_masks, decide_straight, largest_mark)
from t870_control.competition_mission_manager_node import (
    CompetitionMissionManager)


SEP = 20.0      # 최소 좌우 간격 픽셀
RISE = 100.0    # 최대 세로 어긋남 픽셀


def mark(cx, cy=120.0, area=400):
    """largest_mark 형식의 가짜 표지."""
    return (area, cx, cy, (int(cx) - 10, int(cy) - 10, 20, 20))


def test_red_left_green_right_goes_ox():
    decided, straight, _ = decide_straight(mark(100.0), mark(400.0), SEP, RISE)
    assert decided is True
    assert straight is True


def test_green_left_red_right_keeps_path():
    decided, straight, _ = decide_straight(mark(400.0), mark(100.0), SEP, RISE)
    assert decided is True
    assert straight is False


def test_missing_red_is_undecided():
    assert decide_straight(None, mark(400.0), SEP, RISE)[:2] == (False, False)


def test_missing_green_is_undecided():
    assert decide_straight(mark(100.0), None, SEP, RISE)[:2] == (False, False)


def test_both_missing_is_undecided():
    assert decide_straight(None, None, SEP, RISE)[:2] == (False, False)


def test_overlapping_marks_are_undecided():
    decided, _, reason = decide_straight(mark(200.0), mark(210.0), SEP, RISE)
    assert decided is False
    assert reason == 'overlapping'


def test_vertically_separated_marks_are_undecided():
    # 두 표지는 나란히 붙어 있다. 세로로 크게 어긋나면 하나는 다른 물체다.
    decided, _, reason = decide_straight(
        mark(100.0, cy=50.0), mark(400.0, cy=400.0), SEP, RISE)
    assert decided is False
    assert reason == 'not side by side'


def draw_x(frame, cx, cy, size, color):
    half = size // 2
    cv2.line(frame, (cx - half, cy - half), (cx + half, cy + half), color, 6)
    cv2.line(frame, (cx - half, cy + half), (cx + half, cy - half), color, 6)


def draw_arrow(frame, cx, cy, size, color):
    half = size // 2
    cv2.arrowedLine(frame, (cx - half, cy), (cx + half, cy), color, 8,
                    tipLength=0.4)


def detect(frame, merge=15):
    red_mask, green_mask = color_masks(frame, merge_kernel_px=merge)
    area = frame.shape[0] * frame.shape[1]
    lo, hi = max(30, int(area * 0.0006)), int(area * 0.15)
    return largest_mark(red_mask, lo, hi), largest_mark(green_mask, lo, hi)


def test_red_x_left_green_arrow_right():
    frame = np.zeros((480, 640, 3), np.uint8)
    draw_x(frame, 180, 200, 70, (0, 0, 255))          # BGR 빨강 X, 왼쪽
    draw_arrow(frame, 460, 200, 70, (0, 255, 0))      # BGR 초록 화살표, 오른쪽
    red, green = detect(frame)
    assert red is not None, '빨강 X 를 못 찾음'
    assert green is not None, '초록 화살표를 못 찾음'
    decided, straight, _ = decide_straight(red, green, 640 * 0.05, 480 * 0.25)
    assert decided is True
    assert straight is True


def test_green_arrow_left_red_x_right():
    frame = np.zeros((480, 640, 3), np.uint8)
    draw_arrow(frame, 180, 200, 70, (0, 255, 0))      # 초록 화살표, 왼쪽
    draw_x(frame, 460, 200, 70, (0, 0, 255))          # 빨강 X, 오른쪽
    red, green = detect(frame)
    assert red is not None and green is not None
    decided, straight, _ = decide_straight(red, green, 640 * 0.05, 480 * 0.25)
    assert decided is True
    assert straight is False


def test_x_and_arrow_survive_shape_filters():
    """신호등 로직의 원형도/종횡비였다면 걸러졌을 모양이다."""
    import math
    frame = np.zeros((480, 640, 3), np.uint8)
    draw_x(frame, 320, 200, 70, (0, 0, 255))
    red_mask, _ = color_masks(frame)
    contours, _ = cv2.findContours(
        red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    assert contours
    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    circularity = 4.0 * math.pi * area / (perimeter * perimeter)
    # 신호등 판정의 문턱은 0.42 다. X 는 그 아래라 그 로직으로는 못 잡는다.
    assert circularity < 0.42
    # 이 노드는 모양을 안 보므로 잡힌다.
    red, _ = detect(frame)
    assert red is not None


def switch_target(ox_straight, route='P1', waypoint=693):
    """그 판단으로 미션 매니저가 실제로 경로를 바꾸는지 본다."""
    manager = object.__new__(CompetitionMissionManager)
    manager.route = route
    manager.t_choice = 1
    manager.p_choice = 1
    manager.ox_straight = ox_straight
    manager.hill_wp = 39
    manager.timed_estop_wp = -1
    manager.stop_duration = 3.0
    manager.completed = {'hill_wp39'}
    manager.traffic_red_seen = {1: True, 2: True, 3: True}
    manager.traffic_green = True
    manager.estop_until = 0.0
    manager.current_waypoint = 0
    switched = []
    manager.switch = lambda route_name, key, start, reverse='': (
        switched.append((route_name, start)),
        setattr(manager, 'route', route_name))
    manager.timed_stop = lambda key: None
    manager.traffic = lambda *args: None

    class Msg:
        data = waypoint

    manager.on_waypoint(Msg())
    return switched


def test_true_switches_to_ox_from_p1():
    assert switch_target(True, route='P1') == [('OX', 697)]


def test_true_switches_to_ox_from_p2():
    assert switch_target(True, route='P2') == [('OX', 697)]


def test_false_keeps_current_path():
    assert switch_target(False, route='P1') == []
    assert switch_target(False, route='P2') == []


def test_no_switch_before_decision_waypoint():
    assert switch_target(True, route='P1', waypoint=692) == []


@pytest.mark.parametrize('route', ['T1', 'T2'])
def test_no_ox_switch_before_parallel_section(route):
    assert switch_target(True, route=route, waypoint=320) == []


def band(frame, period=14, thickness=6):
    """LED 밴딩을 흉내낸다. 가로 줄마다 화면을 지운다.

    카메라 노출 주파수가 LED 주파수와 안 맞으면 획이 이렇게 끊겨 보인다.
    """
    for y in range(0, frame.shape[0], period):
        frame[y:y + thickness, :] = 0
    return frame


def test_banded_led_marks_are_still_ordered():
    """밴딩으로 획이 끊겨도 좌우 순서는 나와야 한다."""
    frame = np.zeros((480, 640, 3), np.uint8)
    draw_x(frame, 180, 200, 80, (0, 0, 255))
    draw_arrow(frame, 460, 200, 80, (0, 255, 0))
    band(frame)
    red, green = detect(frame)
    assert red is not None, '밴딩된 빨강 X 를 못 찾음'
    assert green is not None, '밴딩된 초록 화살표를 못 찾음'
    decided, straight, reason = decide_straight(
        red, green, 640 * 0.05, 480 * 0.25)
    assert decided is True, reason
    assert straight is True


def test_banded_marks_reversed_order():
    frame = np.zeros((480, 640, 3), np.uint8)
    draw_arrow(frame, 180, 200, 80, (0, 255, 0))
    draw_x(frame, 460, 200, 80, (0, 0, 255))
    band(frame)
    red, green = detect(frame)
    assert red is not None and green is not None
    decided, straight, _ = decide_straight(red, green, 640 * 0.05, 480 * 0.25)
    assert decided is True
    assert straight is False


def test_merge_kernel_is_what_rejoins_fragments():
    """커널이 작으면 조각이 안 붙어 면적 하한에 걸린다.

    merge_kernel_px 를 왜 키워야 하는지 고정한다.
    """
    frame = np.zeros((480, 640, 3), np.uint8)
    draw_x(frame, 180, 200, 80, (0, 0, 255))
    draw_arrow(frame, 460, 200, 80, (0, 255, 0))
    band(frame, period=14, thickness=8)
    small = detect(frame, merge=1)
    large = detect(frame, merge=15)
    small_area = small[0][0] if small[0] else 0
    large_area = large[0][0] if large[0] else 0
    assert large_area > small_area, '큰 커널이 조각을 더 묶어야 한다'


def make_detector(minimum_votes=5):
    """OxSignalDetector 의 누적 투표 부분만 떼어 쓴다."""
    from t870_control.ox_signal_detector_node import OxSignalDetector
    node = object.__new__(OxSignalDetector)
    node.minimum_votes = minimum_votes
    node.straight = False
    node.red_left_votes = 0
    node.green_left_votes = 0
    return node


def cast(node, straight):
    """on_image 의 투표 부분과 같은 식."""
    if straight:
        node.red_left_votes += 1
    else:
        node.green_left_votes += 1
    total = node.red_left_votes + node.green_left_votes
    if total >= node.minimum_votes:
        node.straight = node.red_left_votes > node.green_left_votes
    return node.straight


def test_votes_below_minimum_do_not_decide():
    node = make_detector(minimum_votes=5)
    for _ in range(4):
        assert cast(node, True) is False        # 아직 표가 모자라다
    assert cast(node, True) is True             # 5표째에 확정


def test_majority_red_left_wins():
    node = make_detector()
    for _ in range(7):
        cast(node, True)
    for _ in range(3):
        cast(node, False)
    assert node.straight is True
    assert (node.red_left_votes, node.green_left_votes) == (7, 3)


def test_majority_green_left_wins():
    node = make_detector()
    for _ in range(3):
        cast(node, True)
    for _ in range(7):
        cast(node, False)
    assert node.straight is False


def test_late_noise_does_not_flip_accumulated_majority():
    """누적이라 뒤쪽 몇 프레임이 튀어도 뒤집히지 않는다.

    슬라이딩 윈도우였다면 마지막 몇 프레임만으로 뒤집혔을 상황이다.
    """
    node = make_detector()
    for _ in range(20):
        cast(node, True)
    for _ in range(9):
        cast(node, False)
    assert node.straight is True
    assert node.red_left_votes > node.green_left_votes


def test_tie_keeps_safe_default():
    """동률이면 경로를 바꾸지 않는다."""
    node = make_detector()
    for _ in range(5):
        cast(node, True)
    for _ in range(5):
        cast(node, False)
    assert node.straight is False
