"""ROI 를 지면 통로 모양(사다리꼴)으로 자르는지 고정한다.

원근 때문에 같은 좌우 폭이라도 멀수록 화면에서 좁아진다. 직사각형 ROI 를
쓰면 먼 쪽 좌우 끝이 통로 밖(갓길 잔디, 연석, 담장)이 된다. 그 배경이
색에 걸려 근거리 덩어리와 하나로 이어지면, 덩어리 아랫변이 실제 장애물이
아닌 곳을 가리키고 지면 변환이 그 점을 아주 가깝다고 읽는다.
"""
import inspect
import math
import re
import sys

sys.path = [p for p in sys.path
            if p != '/usr/local/lib/python3.12/dist-packages']

import cv2
import numpy as np
import pytest

from t870_control.obstacle_avoidance_node import (
    ObstacleAvoidance, corridor_mask, corridor_polygon, homography_from_points)

WIDTH, HEIGHT = 640, 480


def config():
    source = inspect.getsource(ObstacleAvoidance.__init__)

    def number(key):
        return float(re.search(r"'%s':\s*([0-9.]+)" % key, source).group(1))

    def points(key):
        body = re.search(r"'%s':\s*\[(.*?)\]" % key, source, re.S).group(1)
        body = re.sub(r'#[^\n]*', '', body)
        return [float(x) for x in body.replace('\n', ' ').split(',')
                if x.strip()]

    homography = homography_from_points(
        points('bev_image_points'), points('bev_ground_points'))
    return number, np.linalg.inv(homography)


@pytest.fixture
def setup():
    number, inverse = config()
    # 노드와 같은 여유(1.2배)를 준다. 윗변을 look_distance 에 딱 맞추면
    # 그 거리의 장애물이 경계선상에 놓여 잘린다.
    corners = corridor_polygon(
        inverse, number('minimum_distance_m'),
        number('look_distance_m') * 1.2, number('roi_corridor_half_m'))
    return number, inverse, corners


def test_far_edge_is_narrower_than_the_near_edge(setup):
    """사다리꼴이어야 한다. 먼 쪽이 좁고 가까운 쪽이 넓다."""
    _, _, corners = setup
    far = abs(corners[0][0] - corners[1][0])
    near = abs(corners[2][0] - corners[3][0])
    assert far < near, '먼 쪽이 더 좁아야 한다 (먼 %d, 가까운 %d)' % (far, near)


def test_far_edge_drops_the_screen_corners(setup):
    """먼 쪽에서 화면 좌우 끝이 잘려야 한다. 거기가 갓길/담장이다."""
    _, _, corners = setup
    left = min(corners[0][0], corners[1][0])
    right = max(corners[0][0], corners[1][0])
    assert left > 0, '먼 쪽 왼쪽 끝이 화면 밖까지 열려 있다'
    assert right < WIDTH, '먼 쪽 오른쪽 끝이 화면 밖까지 열려 있다'


def test_corridor_keeps_the_whole_collision_corridor(setup):
    """충돌 통로(±desired_clearance) 는 통째로 살아 있어야 한다.

    잘려 나가면 덩어리 중심이 안쪽으로 밀려 offset 이 틀리게 나온다.
    """
    number, inverse, corners = setup
    mask = corridor_mask((HEIGHT, WIDTH, 3), corners)
    clearance = number('desired_clearance_m')
    for distance in (1.0, 2.0, 3.0, number('look_distance_m')):
        for offset in (-clearance, 0.0, clearance):
            point = np.array([[[distance, offset]]], dtype=np.float32)
            x, y = cv2.perspectiveTransform(point, inverse)[0, 0]
            if not (0 <= int(y) < HEIGHT and 0 <= int(x) < WIDTH):
                continue
            assert mask[int(y), int(x)] > 0, (
                '전방 %.1f m, 좌우 %+.2f m 가 통로 밖으로 잘렸다'
                % (distance, offset))


# 실측 (2026-09-20). 장애물 1.26 x 0.73 m, 높이 0.28 m. 차폭 0.8 m.
# 차선 폭 4.2 m.
#
# 장애물은 비스듬히 놓인다. 직사각형의 횡방향 최대 투영은 대각선 길이라
# 정면으로 막을 때(1.26)보다 오히려 넓다. 최악을 설계값으로 쓴다.
OBSTACLE_LONG_M = 1.26
OBSTACLE_SHORT_M = 0.73
OBSTACLE_WIDTH_M = math.hypot(OBSTACLE_LONG_M, OBSTACLE_SHORT_M)   # 1.456
VEHICLE_WIDTH_M = 0.8
LANE_WIDTH_M = 4.2


def test_diagonal_is_the_worst_lateral_extent():
    """비스듬히 놓이면 더 넓게 막는다는 근거를 고정한다."""
    worst = max(
        OBSTACLE_LONG_M * abs(math.cos(math.radians(phi)))
        + OBSTACLE_SHORT_M * abs(math.sin(math.radians(phi)))
        for phi in range(0, 91))
    assert worst == pytest.approx(OBSTACLE_WIDTH_M, abs=0.005)
    assert worst > OBSTACLE_LONG_M, '정면으로 막을 때보다 넓어야 한다'


def test_clearance_keeps_the_vehicle_inside_the_lane():
    """비켜도 차선을 벗어나면 안 된다.

    차선 4.2 m 에서 내 차 중심이 있을 수 있는 범위는 1.13~1.70 m 뿐이다.
    폭이 0.57 m 라 clearance 를 키우기만 하면 반대쪽으로 나간다.
    """
    number, _ = config()
    clearance = number('desired_clearance_m')
    to_lane = LANE_WIDTH_M / 2.0 - clearance - VEHICLE_WIDTH_M / 2.0
    assert to_lane > 0.1, (
        'clearance %.2f m 면 차선 가장자리까지 %.2f m 밖에 안 남는다'
        % (clearance, to_lane))


def test_corridor_half_covers_an_obstacle_at_the_clearance_limit(setup):
    """통로 끝에 걸친 장애물의 가장자리까지 담아야 한다.

    offset 은 장애물의 '중심' 이다. 중심이 desired_clearance 에 있으면
    가장자리는 거기서 반폭만큼 더 나간다. 사다리꼴이 그보다 좁으면
    덩어리가 잘려 중심이 안쪽으로 밀리고 offset 이 작게 나온다.
    """
    number, _, _ = setup
    needed = number('desired_clearance_m') + OBSTACLE_WIDTH_M / 2.0
    assert number('roi_corridor_half_m') >= needed, (
        '사다리꼴 %.2f m 로는 통로 끝 장애물의 가장자리 %.2f m 가 잘린다'
        % (number('roi_corridor_half_m'), needed))


def test_clearance_actually_clears_the_obstacle(setup):
    """회피에 성공했을 때 실제로 안 닿아야 한다.

    이전 값 0.55 는 이 장애물에 맞지 않았다. 반폭이 0.63 m 라 중심을
    0.55 m 비켜도 가장자리가 -0.08 m, 즉 진로 위에 남았다.
    """
    number, _, _ = setup
    gap = (number('desired_clearance_m')
           - OBSTACLE_WIDTH_M / 2.0 - VEHICLE_WIDTH_M / 2.0)
    assert gap > 0.1, (
        'desired_clearance %.2f m 로 비켜도 여유가 %.2f m 뿐이다'
        % (number('desired_clearance_m'), gap))


def test_look_distance_gives_room_to_steer_clear(setup):
    """필요한 횡변위를 보정 상한 안에서 낼 수 있어야 한다.

    y ~ s^2 / (2R), R = wheelbase / tan(steer).
    4.0 m 에서 발견하면 1.25 m 를 비키는 데 0.18 rad 이 필요해 보정
    상한 0.12 를 넘는다. 그래서 5.5 m 로 늘렸다.
    """
    import math
    number, _, _ = setup
    wheelbase = 1.0                      # 문서상 0.725~1.0. 보수적으로.
    run = number('look_distance_m') - number('minimum_distance_m')
    radius = run * run / (2.0 * number('desired_clearance_m'))
    needed = math.atan2(wheelbase, radius)
    assert needed <= number('steer_rad'), (
        'look_distance %.1f m 로는 조향 %.3f rad 이 필요한데 보정 상한은 '
        '%.3f rad 다' % (number('look_distance_m'), needed,
                        number('steer_rad')))


def test_zero_half_width_means_no_trapezoid():
    """0 이하면 예전처럼 전폭 직사각형을 쓴다."""
    node = object.__new__(ObstacleAvoidance)
    _, inverse = config()
    node.inverse_homography = inverse
    node.value = {'roi_corridor_half_m': 0.0, 'minimum_distance_m': 0.3,
                  'look_distance_m': 4.0}.get
    assert node.corridor_corners(0) is None


def test_missing_calibration_means_no_trapezoid():
    node = object.__new__(ObstacleAvoidance)
    node.inverse_homography = None
    node.value = {'roi_corridor_half_m': 1.5}.get
    assert node.corridor_corners(0) is None


# --- 장애물이 차선 좌우 어디에 놓이든 통과하는가 ---------------------
# 실차에서 장애물은 차선 중앙이 아니라 좌우 중 한쪽에 무작위로 놓인다.
# bev_correction 의 side = copysign(1, offset) 이 감지된 반대쪽을 고르므로
# 항상 여유가 넓은 쪽으로 피한다. 그 결과를 수치로 고정한다.

def final_position(offset, clearance):
    """보정이 수렴한 뒤 차가 서 있을 차선 내 위치.

    offset 은 차 기준 장애물 중심의 좌우 거리다. 목표는 |offset| 이
    clearance 가 되는 것이므로, 차는 반대쪽으로 (clearance - |offset|)
    만큼 움직인다.
    """
    if abs(offset) >= clearance:
        return 0.0                       # 후보가 아니다. 그대로 간다.
    side = 1.0 if offset > 0 else -1.0
    return -side * (clearance - abs(offset))


@pytest.mark.parametrize(
    'obstacle_at',
    [-1.6, -1.2, -0.8, -0.5, -0.2, 0.0, 0.2, 0.5, 0.8, 1.2, 1.6])
def test_passes_whatever_side_the_obstacle_is_on(setup, obstacle_at):
    number, _, _ = setup
    clearance = number('desired_clearance_m')
    car = final_position(obstacle_at, clearance)
    half_obstacle = OBSTACLE_WIDTH_M / 2.0
    half_car = VEHICLE_WIDTH_M / 2.0

    to_obstacle = abs(obstacle_at - car) - half_obstacle - half_car
    to_lane = LANE_WIDTH_M / 2.0 - abs(car) - half_car

    assert to_obstacle > 0.1, (
        '장애물이 %+.1f m 에 있을 때 여유가 %.2f m 뿐이다'
        % (obstacle_at, to_obstacle))
    assert to_lane > 0.1, (
        '장애물이 %+.1f m 에 있을 때 차선까지 %.2f m 뿐이다'
        % (obstacle_at, to_lane))


def test_centre_is_the_worst_case(setup):
    """중앙이 가장 빡빡하다. 그래서 중앙 기준으로 값을 잡았다."""
    number, _, _ = setup
    clearance = number('desired_clearance_m')
    margins = {}
    for obstacle_at in (0.0, 0.5, 1.0):
        car = final_position(obstacle_at, clearance)
        margins[obstacle_at] = (LANE_WIDTH_M / 2.0 - abs(car)
                                - VEHICLE_WIDTH_M / 2.0)
    assert margins[0.0] < margins[0.5] < margins[1.0]


def test_ignored_obstacle_is_genuinely_clear(setup):
    """clearance 밖이라 무시하는 장애물이 실제로 안 닿아야 한다."""
    number, _, _ = setup
    clearance = number('desired_clearance_m')
    gap = clearance - OBSTACLE_WIDTH_M / 2.0 - VEHICLE_WIDTH_M / 2.0
    assert gap > 0.1, (
        '|offset| > %.2f 를 무시하는데 그 경계에서 여유가 %.2f m 뿐이다'
        % (clearance, gap))
