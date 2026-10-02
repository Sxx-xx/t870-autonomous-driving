"""카메라 ROI 가 실제로 쓸모 있는 거리 대역만 보는지 고정한다.

ROI 상단이 look_distance_m 보다 먼 곳을 보고 있으면, 후보가 될 수 없는
배경(잔디밭, 생울타리, 나무)이 색에 걸려 근거리 덩어리와 하나로 이어진다.
그러면 덩어리 아랫변이 엉뚱한 곳을 가리키고 최대 조향이 걸린다.

실차에서 정적장애물 구간에 들어갈 때 카메라가 풀밭을 먼저 봤다.
"""
import re
import sys

sys.path = [p for p in sys.path
            if p != '/usr/local/lib/python3.12/dist-packages']

import cv2
import numpy as np
import pytest

from t870_control.obstacle_avoidance_node import (
    ObstacleAvoidance, homography_from_points)

WIDTH, HEIGHT = 640, 480


def defaults():
    import inspect
    source = inspect.getsource(ObstacleAvoidance.__init__)

    def number(key):
        return float(re.search(r"'%s':\s*([0-9.]+)" % key, source).group(1))

    def points(key):
        body = re.search(r"'%s':\s*\[(.*?)\]" % key, source, re.S).group(1)
        body = re.sub(r'#[^\n]*', '', body)
        return [float(x) for x in body.replace('\n', ' ').split(',')
                if x.strip()]

    return number, points


def forward_distance(homography, y):
    point = np.array([[[WIDTH / 2.0, float(y)]]], dtype=np.float32)
    return float(cv2.perspectiveTransform(point, homography)[0, 0][0])


@pytest.fixture
def setup():
    number, points = defaults()
    homography = homography_from_points(
        points('bev_image_points'), points('bev_ground_points'))
    return number, homography


def test_roi_top_is_not_beyond_the_look_distance(setup):
    """ROI 상단이 보는 거리가 look_distance_m 을 크게 넘으면 안 된다."""
    number, homography = setup
    top = forward_distance(homography, HEIGHT * number('roi_y_min'))
    look = number('look_distance_m')
    assert 0.0 < top <= look * 1.3, (
        'ROI 상단이 %.1f m 를 본다. look_distance 는 %.1f m 라 그 너머는 '
        '후보가 못 되면서 배경만 들어온다' % (top, look))


def test_roi_top_still_covers_the_look_distance(setup):
    """반대로 너무 내리면 4 m 장애물을 놓친다."""
    number, homography = setup
    top = forward_distance(homography, HEIGHT * number('roi_y_min'))
    assert top >= number('look_distance_m') * 0.95, (
        'ROI 상단이 %.1f m 밖에 안 본다. look_distance 까지는 봐야 한다'
        % top)


def test_roi_bottom_reaches_the_minimum_distance(setup):
    """아래쪽은 minimum_distance_m 까지 닿아야 한다."""
    number, homography = setup
    bottom = forward_distance(homography, HEIGHT * number('roi_y_max') - 1)
    assert bottom <= number('minimum_distance_m') * 3.0


def test_old_roi_saw_past_the_horizon(setup):
    """회귀 증거. 옛 0.35 는 22 m 를, 그 위는 지평선 너머를 봤다."""
    _, homography = setup
    assert forward_distance(homography, HEIGHT * 0.35) > 20.0
    # 지평선 위에서는 부호가 뒤집혀 거리가 음수로 나온다.
    assert forward_distance(homography, HEIGHT * 0.29) < 0.0
