"""주황 장애물 차량과 마른 잔디를 색으로 가르는 경계를 고정한다.

실차에서 정적장애물 구간(WP201~227) 카메라가 마른 잔디를 보고 섰다.
photo/IMG_8848~8858 의 픽셀을 직접 재어 잡은 값이다.

  햇빛 받은 차체   H 중앙 10   S 중앙 158
  마른 잔디        H 중앙 25   S 중앙  76

분리자는 채도가 아니라 hue 다. 채도로 가르면(S>=150) 잔디는 빠지지만
햇빛에 바랜 차체도 같이 빠져 그늘진 실내만 남는다.
"""
import numpy as np
import pytest

import sys
sys.path = [p for p in sys.path
            if p != '/usr/local/lib/python3.12/dist-packages']
import cv2

from t870_control.obstacle_avoidance_node import ObstacleAvoidance


def defaults():
    """노드 기본값을 읽는다. launch 가 이 노드에 파라미터를 안 주므로
    기본값이 곧 실차에서 도는 값이다."""
    import inspect
    source = inspect.getsource(ObstacleAvoidance.__init__)
    scope = {}
    for line in source.splitlines():
        line = line.strip().rstrip(',')
        for key in ('hsv_low', 'hsv_high'):
            if line.startswith("'%s'" % key):
                scope[key] = eval(line.split(':', 1)[1].strip())
    return scope['hsv_low'], scope['hsv_high']


def patch(hue, sat, val):
    """그 HSV 한 색으로 채운 작은 이미지."""
    image = np.zeros((20, 20, 3), dtype=np.uint8)
    image[:, :] = (hue, sat, val)
    return cv2.cvtColor(image, cv2.COLOR_HSV2BGR)


def passes(bgr):
    low, high = defaults()
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(low), np.array(high))
    return mask.mean() > 127.0


# 실측 중앙값. 차체는 통과, 잔디는 탈락해야 한다.
def test_sunlit_car_body_is_detected():
    assert passes(patch(10, 158, 128))


def test_dried_grass_is_rejected():
    assert not passes(patch(25, 76, 130))


@pytest.mark.parametrize('sat', [80, 120, 160, 200, 250])
def test_car_is_detected_across_the_lighting_range(sat):
    """햇빛/그늘로 채도가 흔들려도 hue 가 맞으면 잡아야 한다.

    옛 설정(S>=150)은 여기서 80~120 을 놓쳐 그늘진 실내만 남았다.
    """
    assert passes(patch(10, sat, 128))


@pytest.mark.parametrize('hue', [20, 25, 30])
def test_grass_hues_are_rejected_whatever_the_saturation(hue):
    for sat in (60, 100, 140):
        assert not passes(patch(hue, sat, 130)), (
            'H%d S%d 가 새어 들어왔다' % (hue, sat))


def test_hue_ceiling_is_the_separator_not_saturation():
    """경계 근거. hue 상한을 18 로 올리면 잔디가 새기 시작한다."""
    low, high = defaults()
    assert high[0] <= 16, 'hue 상한이 16 을 넘으면 마른 잔디가 들어온다'
    assert low[1] <= 100, '채도 하한이 높으면 햇빛에 바랜 차체를 놓친다'
