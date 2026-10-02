"""켜진 초록 램프가 배경의 붉은 물체에 가려지지 않는지 고정한다.

실차에서 초록불로 바뀌었는데 GREEN 이 한 번도 안 나와 정지선에서 못
빠져나온 적이 있다. 원인은 후보를 색 구분 없이 area*circularity 최대
하나로 골랐기 때문이다. 신호등에 잡혀 서 있는 동안에는 WP 가 늘지 않아
해제 창도 쓸 수 없으므로, 이 오판은 곧 교착이다.
"""
import numpy as np
import pytest

from t870_control.traffic_lamp import detect_traffic_light


def blank():
    return np.zeros((480, 640, 3), dtype=np.uint8)


def disc(frame, centre, radius, bgr):
    y, x = np.ogrid[:frame.shape[0], :frame.shape[1]]
    inside = (x - centre[0]) ** 2 + (y - centre[1]) ** 2 <= radius ** 2
    frame[inside] = bgr


GREEN_BGR = (60, 230, 60)
RED_BGR = (40, 40, 230)


def test_lone_green_lamp_is_green():
    frame = blank()
    disc(frame, (320, 120), 9, GREEN_BGR)
    assert detect_traffic_light(frame)[0] == 'GREEN'


def test_lone_red_lamp_is_red():
    frame = blank()
    disc(frame, (320, 120), 9, RED_BGR)
    assert detect_traffic_light(frame)[0] == 'RED'


def test_bigger_red_background_does_not_mask_green_lamp():
    # 배경의 붉은 물체가 초록 램프보다 크다. 그래도 초록을 택해야 한다.
    # 우대 0.5 에서 r=9 램프(점수 245)는 r=13 빨강(419)까지 이긴다.
    frame = blank()
    disc(frame, (320, 120), 9, GREEN_BGR)
    disc(frame, (120, 90), 13, RED_BGR)
    assert detect_traffic_light(frame)[0] == 'GREEN'


def test_much_bigger_red_still_wins():
    # 초록 우대에도 한계가 있다. 문턱 0.5 는 '붉은 점수가 램프의 2 배를
    # 넘으면 진다' 는 뜻이다. r=14(501) 부터 넘는다.
    #
    # 면적 상한을 없앴으므로 이제 임의로 큰 붉은 덩어리가 경쟁에
    # 들어온다. 그 대가를 여기에 고정해 둔다. 초록을 더 세게 밀려면
    # green_preference 를 '내려야' 한다(0.25 면 4 배까지 버틴다).
    frame = blank()
    disc(frame, (320, 120), 9, GREEN_BGR)
    disc(frame, (120, 90), 14, RED_BGR)
    assert detect_traffic_light(frame)[0] == 'RED'
    assert detect_traffic_light(frame, green_preference=0.25)[0] == 'GREEN'


def test_green_preference_can_be_disabled():
    # 0 이면 예전처럼 최대 점수 하나로 고른다. 회귀 비교용.
    frame = blank()
    disc(frame, (320, 120), 9, GREEN_BGR)
    disc(frame, (120, 90), 14, RED_BGR)
    assert detect_traffic_light(frame, green_preference=0.0)[0] == 'RED'


def test_tiny_green_speck_does_not_beat_a_solid_red_lamp():
    # 초록 우대가 잡음까지 이기게 하면 안 된다. 점수비 0.5 미만은 탈락.
    frame = blank()
    disc(frame, (320, 120), 16, RED_BGR)
    disc(frame, (120, 90), 4, GREEN_BGR)
    assert detect_traffic_light(frame)[0] == 'RED'


def test_no_lamp_is_unknown():
    assert detect_traffic_light(blank())[0] == 'UNKNOWN'


# --- 정지선에서 램프가 커지는 문제 -------------------------------------
# 640x480, HFOV 60 도, 표준 램프 300 mm 기준 화면상 반지름:
#   10 m -> 8 px, 5 m -> 17 px, 4 m -> 21 px, 3 m -> 28 px, 2 m -> 42 px

@pytest.mark.parametrize(
    ('radius', 'distance_m'),
    [(8, 10.0), (17, 5.0), (21, 4.0), (28, 3.0), (42, 2.0)])
def test_green_lamp_is_seen_at_every_approach_distance(radius, distance_m):
    """정지선에 서도 초록 램프를 봐야 한다.

    옛 면적 상한(0.004)에서는 4 m 안쪽부터 램프가 '너무 커서' 탈락해
    UNKNOWN 이 됐다. 그래서 초록으로 바뀐 것을 영영 못 봤다.
    """
    frame = blank()
    disc(frame, (320, 150), radius, GREEN_BGR)
    assert detect_traffic_light(frame)[0] == 'GREEN', (
        '%.0f m 에서 초록 램프를 놓쳤다' % distance_m)


def test_ceiling_still_accepts_a_very_close_lamp():
    """상한 0.05 는 정지선 거리의 램프를 버리지 않아야 한다.

    r=60 은 면적 약 11300 px = 프레임의 0.037 로, 램프로 치면 약 1.4 m
    다. 옛 상한 0.004(1228 px) 에서는 4.5 m 안쪽이 전부 버려졌다.
    """
    frame = blank()
    disc(frame, (320, 150), 60, GREEN_BGR)
    assert detect_traffic_light(frame)[0] == 'GREEN'


def test_ceiling_rejects_an_implausibly_large_blob():
    """램프일 수 없는 거대한 영역은 계속 버린다.

    r=110 은 면적 약 38000 px = 프레임의 0.124 로 상한 0.05 를 넘는다.
    램프로 치면 0.75 m 거리인데 그런 일은 없다.
    """
    frame = blank()
    disc(frame, (320, 180), 110, GREEN_BGR)
    assert detect_traffic_light(frame)[0] == 'UNKNOWN'
    assert detect_traffic_light(frame, area_ceiling_ratio=0.0)[0] == 'GREEN'


def test_old_ceiling_is_what_dropped_the_close_lamp():
    """회귀 증거. 옛 상한을 그대로 주면 가까운 램프가 사라진다."""
    frame = blank()
    disc(frame, (320, 150), 28, GREEN_BGR)     # 3 m
    assert detect_traffic_light(
        frame, area_ceiling_ratio=0.004)[0] == 'UNKNOWN'
    assert detect_traffic_light(frame)[0] == 'GREEN'


def test_low_mounted_lamp_below_old_roi_is_seen():
    """이 코스의 신호등은 낮다. 0.55 컷이면 잘려 나간다."""
    frame = blank()
    disc(frame, (320, 300), 17, GREEN_BGR)     # 화면 62% 높이
    assert detect_traffic_light(
        frame, roi_height_ratio=0.55)[0] == 'UNKNOWN'
    assert detect_traffic_light(frame)[0] == 'GREEN'


def test_bottom_fifth_of_the_frame_is_ignored():
    """화면 아래 1/5 는 노면이다. 주황 라바콘이 YELLOW 로 잡히면 안 된다.

    480 * 0.8 = 384. 그보다 아래에 있는 것은 후보에 못 든다.
    """
    frame = blank()
    disc(frame, (320, 430), 17, GREEN_BGR)
    assert detect_traffic_light(frame)[0] == 'UNKNOWN'
    disc(frame, (320, 350), 17, GREEN_BGR)     # 컷 위쪽이면 보인다
    assert detect_traffic_light(frame)[0] == 'GREEN'


def test_large_non_round_colour_patch_is_still_rejected():
    """상한을 없앤 뒤로는 모양 검사가 유일한 방어선이다."""
    frame = blank()
    frame[60:200, 100:400] = RED_BGR          # 300x140 사각형
    assert detect_traffic_light(frame)[0] == 'UNKNOWN'
