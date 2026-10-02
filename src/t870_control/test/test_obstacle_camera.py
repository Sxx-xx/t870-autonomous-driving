"""정적장애물 카메라 판단 검증.

장애물이 연석 높이라 수평 라이다 스캔면이 그 위로 지나갈 수 있다.
구역 안에서는 카메라로 색 덩어리를 찾아 좌우 위치로 회피 방향을 정한다.
프로젝트 규약은 +angular.z = 오른쪽이다.
"""
import sys

sys.path = [p for p in sys.path
            if p != '/usr/local/lib/python3.12/dist-packages']

import cv2
import numpy as np
import pytest

from t870_control.obstacle_avoidance_node import (
    camera_correction, colour_mask, largest_blob)


ORANGE_LOW, ORANGE_HIGH = [5, 120, 90], [25, 255, 255]
NONE_LOW, NONE_HIGH = [0, 0, 0], [0, 0, 0]


def frame_with_blob(cx, cy=300, size=80, bgr=(0, 140, 255)):
    """BGR (0,140,255) = 주황. 기본 HSV 범위에 들어간다."""
    frame = np.zeros((480, 640, 3), np.uint8)
    cv2.circle(frame, (cx, cy), size // 2, bgr, -1)
    return frame


def detect(frame, min_area=400):
    mask = colour_mask(frame, ORANGE_LOW, ORANGE_HIGH, NONE_LOW, NONE_HIGH)
    return largest_blob(mask, min_area)


def test_left_obstacle_steers_right():
    blob = detect(frame_with_blob(160))
    assert blob is not None, '주황 덩어리를 못 찾음'
    assert camera_correction(blob, 640, 0.12, 0.04) == pytest.approx(0.12)


def test_right_obstacle_steers_left():
    blob = detect(frame_with_blob(480))
    assert blob is not None
    assert camera_correction(blob, 640, 0.12, 0.04) == pytest.approx(-0.12)


def test_centre_obstacle_gives_no_correction():
    # 정면 한가운데면 어느 쪽으로 피할지 정할 수 없다.
    blob = detect(frame_with_blob(320))
    assert blob is not None
    assert camera_correction(blob, 640, 0.12, 0.04) == 0.0


def test_just_outside_deadband_corrects():
    # 불감대는 화면폭의 4% = 25.6 px.
    blob = (500, 320.0 - 30.0)
    assert camera_correction(blob, 640, 0.12, 0.04) == pytest.approx(0.12)


def test_no_blob_gives_no_correction():
    assert camera_correction(None, 640, 0.12, 0.04) == 0.0


def test_other_colour_is_ignored():
    # 파란 물체는 주황 범위 밖이다.
    assert detect(frame_with_blob(160, bgr=(255, 0, 0))) is None


def test_small_blob_is_ignored():
    frame = frame_with_blob(160, size=6)
    assert detect(frame, min_area=400) is None


def test_largest_blob_wins():
    """큰 쪽(가까운 쪽)을 고른다."""
    frame = np.zeros((480, 640, 3), np.uint8)
    cv2.circle(frame, (160, 300), 20, (0, 140, 255), -1)   # 작음, 왼쪽
    cv2.circle(frame, (480, 300), 60, (0, 140, 255), -1)   # 큼, 오른쪽
    blob = detect(frame)
    assert blob is not None
    assert blob[1] > 320, '큰 덩어리는 오른쪽에 있다'
    assert camera_correction(blob, 640, 0.12, 0.04) == pytest.approx(-0.12)


def test_second_hue_range_supports_wrapping_red():
    """빨강은 hue 가 0/180 양쪽에 걸친다. 두 번째 범위로 받는다."""
    frame = np.zeros((480, 640, 3), np.uint8)
    cv2.circle(frame, (160, 300), 40, (0, 0, 255), -1)     # BGR 빨강
    one = largest_blob(
        colour_mask(frame, [0, 120, 90], [10, 255, 255],
                    NONE_LOW, NONE_HIGH), 400)
    two = largest_blob(
        colour_mask(frame, [0, 120, 90], [10, 255, 255],
                    [170, 120, 90], [180, 255, 255]), 400)
    assert one is not None and two is not None
    assert two[0] >= one[0], '두 번째 범위를 더하면 면적이 줄지 않아야 한다'
