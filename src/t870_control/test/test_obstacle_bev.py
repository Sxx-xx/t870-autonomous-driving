"""버드아이뷰 기반 정적장애물 판단 검증.

캘리브레이션 숫자는 아직 없다. 가상의 카메라 장착값으로 호모그래피를
만들어 기하와 제어식만 못박는다. 실제 사진이 오면 숫자만 채우면 된다.
"""
import math
import sys

sys.path = [p for p in sys.path
            if p != '/usr/local/lib/python3.12/dist-packages']

import numpy as np
import pytest

from t870_control.obstacle_avoidance_node import (
    all_blobs, bev_correction, ground_point, homography_from_points,
    select_obstacle)


# 가상 장착: 전방 1~3 m, 좌우 +/-0.6 m 의 네 점이 화면에 이렇게 찍혔다고 둔다.
# 가까울수록 화면 아래, 넓게 퍼진다(원근).
IMAGE_POINTS = [
    140.0, 430.0,   # (1.0, +0.6)  가까운 왼쪽
    500.0, 430.0,   # (1.0, -0.6)  가까운 오른쪽
    268.0, 300.0,   # (3.0, +0.6)  먼 왼쪽
    372.0, 300.0,   # (3.0, -0.6)  먼 오른쪽
]
GROUND_POINTS = [
    1.0, 0.6,
    1.0, -0.6,
    3.0, 0.6,
    3.0, -0.6,
]
CLEARANCE = 0.55
GAIN = 0.30
MIN_DIST = 0.3
MAX_CORR = 0.12


@pytest.fixture
def homography():
    matrix = homography_from_points(IMAGE_POINTS, GROUND_POINTS)
    assert matrix is not None
    return matrix


def test_uncalibrated_returns_none():
    assert homography_from_points([0.0], [0.0]) is None
    assert homography_from_points([], []) is None
    assert homography_from_points(IMAGE_POINTS, GROUND_POINTS[:6]) is None


def test_calibration_points_map_back_exactly(homography):
    """넣은 대응점은 오차 없이 되돌아와야 한다."""
    for index in range(0, len(IMAGE_POINTS), 2):
        distance, offset = ground_point(
            homography, IMAGE_POINTS[index], IMAGE_POINTS[index + 1])
        assert distance == pytest.approx(GROUND_POINTS[index], abs=0.01)
        assert offset == pytest.approx(GROUND_POINTS[index + 1], abs=0.01)


def test_more_points_use_least_squares():
    """4점 초과면 findHomography 로 푼다. 잔차가 작아야 한다."""
    image = IMAGE_POINTS + [204.0, 350.0, 436.0, 350.0]
    ground = GROUND_POINTS + [1.9, 0.6, 1.9, -0.6]
    matrix = homography_from_points(image, ground)
    assert matrix is not None
    worst = 0.0
    for index in range(0, len(image), 2):
        distance, offset = ground_point(
            matrix, image[index], image[index + 1])
        worst = max(worst,
                    abs(distance - ground[index]),
                    abs(offset - ground[index + 1]))
    assert worst < 0.30, f'잔차 {worst:.3f} m'


def test_bottom_edge_matters_not_centre(homography):
    """떠 있는 점을 넣으면 실제보다 멀게 나온다.

    라바콘 꼭대기를 쓰면 안 되는 이유다.
    """
    bottom_distance, _ = ground_point(homography, 320.0, 430.0)
    centre_distance, _ = ground_point(homography, 320.0, 380.0)
    assert centre_distance > bottom_distance


def test_obstacle_in_path_gets_correction():
    # 전방 2 m, 왼쪽 0.2 m -> 더 비켜야 한다 -> 오른쪽(+)
    correction = bev_correction(2.0, 0.2, CLEARANCE, GAIN, MIN_DIST, MAX_CORR)
    assert correction > 0.0


def test_obstacle_already_clear_gets_nothing():
    """충분히 옆으로 비켜 있으면 보정하지 않는다.

    화면 한쪽에 보이기만 하면 무조건 틀던 이진 방식과의 차이다.
    """
    assert bev_correction(2.0, 1.5, CLEARANCE, GAIN, MIN_DIST, MAX_CORR) == 0.0
    assert bev_correction(2.0, -1.5, CLEARANCE, GAIN, MIN_DIST, MAX_CORR) == 0.0


def test_right_obstacle_steers_left():
    # offset 음수 = 오른쪽 -> 왼쪽(-)으로 피한다
    assert bev_correction(2.0, -0.2, CLEARANCE, GAIN, MIN_DIST, MAX_CORR) < 0.0


def test_closer_obstacle_gets_stronger_correction():
    far = bev_correction(3.0, 0.2, CLEARANCE, GAIN, MIN_DIST, MAX_CORR)
    near = bev_correction(1.0, 0.2, CLEARANCE, GAIN, MIN_DIST, MAX_CORR)
    assert near > far > 0.0


def test_more_intrusion_gets_stronger_correction():
    edge = bev_correction(2.0, 0.5, CLEARANCE, GAIN, MIN_DIST, MAX_CORR)
    centre = bev_correction(2.0, 0.0, CLEARANCE, GAIN, MIN_DIST, MAX_CORR)
    assert centre > edge > 0.0


def test_correction_is_clamped():
    # 아주 가깝고 정면이면 포화되어야 한다.
    assert bev_correction(
        0.1, 0.0, CLEARANCE, GAIN, MIN_DIST, MAX_CORR) == pytest.approx(MAX_CORR)


def test_dead_centre_uses_default_side():
    assert bev_correction(
        2.0, 0.0, CLEARANCE, GAIN, MIN_DIST, MAX_CORR, +1.0) > 0.0
    assert bev_correction(
        2.0, 0.0, CLEARANCE, GAIN, MIN_DIST, MAX_CORR, -1.0) < 0.0


def test_selects_nearest_in_path_not_biggest():
    """크기가 아니라 실거리로 고른다."""
    marks = [
        (3.2, 0.1, 'far'),
        (1.4, 0.2, 'near'),
        (2.0, 1.8, 'far off to the side'),
    ]
    chosen = select_obstacle(marks, MIN_DIST, 4.0, CLEARANCE)
    assert chosen is not None and chosen[2] == 'near'


def test_ignores_obstacles_outside_corridor():
    marks = [(2.0, 1.4, 'beside'), (2.5, -1.9, 'beside')]
    assert select_obstacle(marks, MIN_DIST, 4.0, CLEARANCE) is None


def test_ignores_too_far_and_too_close():
    marks = [(9.0, 0.0, 'far'), (0.1, 0.0, 'own bumper')]
    assert select_obstacle(marks, MIN_DIST, 4.0, CLEARANCE) is None


def test_all_blobs_reports_bottom_edge():
    mask = np.zeros((200, 200), np.uint8)
    mask[60:140, 80:120] = 255
    blobs = all_blobs(mask, 100)
    assert len(blobs) == 1
    _area, cx, bottom, box = blobs[0]
    assert cx == pytest.approx(99.5, abs=1.0)
    assert bottom == pytest.approx(140.0, abs=1.0)
    assert box == (80, 60, 40, 80)


def test_shipped_calibration_residuals_are_small():
    """노드에 박아넣은 캘리브레이션이 실제로 맞는지 확인한다.

    카메라를 옮기고 값을 안 고치면 여기서 걸린다.
    """
    import re
    from t870_control import obstacle_avoidance_node as module
    source = open(module.__file__, encoding='utf-8').read()

    def numbers(key):
        block = re.search(r"'%s': \[(.*?)\]," % key, source, re.S).group(1)
        # 주석의 숫자('전방 1.0 m, 좌 +0.6')가 섞이지 않게 걷어낸다.
        body = '\n'.join(line.split('#')[0] for line in block.splitlines())
        return [float(v) for v in re.findall(r'-?\d+\.\d+', body)]

    image = numbers('bev_image_points')
    ground = numbers('bev_ground_points')
    assert len(image) == len(ground) >= 8, '캘리브레이션이 비어 있다'

    matrix = homography_from_points(image, ground)
    assert matrix is not None
    worst = 0.0
    for index in range(0, len(image), 2):
        distance, offset = ground_point(matrix, image[index], image[index + 1])
        worst = max(worst,
                    abs(distance - ground[index]),
                    abs(offset - ground[index + 1]))
    assert worst < 0.10, f'잔차 {worst * 100:.1f} cm 가 너무 크다'


def test_shipped_calibration_handles_mounting_offset():
    """카메라가 차량 중심에서 치우쳐 있어도 호모그래피가 보정한다.

    이 차는 화면 정중앙(x=320)이 지면에서 좌 +0.15~0.26 m 다. 즉 카메라가
    왼쪽으로 약 0.2 m 치우쳐 있다. 표식 6개의 중점이 x=359~363 으로
    일관되므로 검출 오류가 아니라 실제 장착 오프셋이다. 이런 것을 자동으로
    처리하라고 캘리브레이션을 하는 것이다.

    여기서 검증하는 것은 '차량 중심선이 화면 어디로 오는가' 가 각 거리에서
    일관된가다. 크게 흔들리면 호모그래피가 틀린 것이다.
    """
    import re
    from t870_control import obstacle_avoidance_node as module
    source = open(module.__file__, encoding='utf-8').read()

    def numbers(key):
        block = re.search(r"'%s': \[(.*?)\]," % key, source, re.S).group(1)
        # 주석의 숫자('전방 1.0 m, 좌 +0.6')가 섞이지 않게 걷어낸다.
        body = '\n'.join(line.split('#')[0] for line in block.splitlines())
        return [float(v) for v in re.findall(r'-?\d+\.\d+', body)]

    matrix = homography_from_points(numbers('bev_image_points'),
                                    numbers('bev_ground_points'))
    # 화면 정중앙이 지면에서 어디인지는 장착에 따라 다르다. 다만 그 값이
    # 거리마다 크게 달라지면 호모그래피가 틀린 것이다.
    offsets = [ground_point(matrix, 320.0, pixel_y)[1]
               for pixel_y in (431.0, 337.0, 291.0)]
    assert max(offsets) - min(offsets) < 0.25, (
        f'거리마다 중앙 오프셋이 너무 다르다: {offsets}')
    # 그리고 오프셋 자체가 차폭 절반(0.39 m)을 넘으면 장착이 잘못된 것이다.
    assert all(abs(value) < 0.39 for value in offsets), offsets
