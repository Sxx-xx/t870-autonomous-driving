#!/usr/bin/env python3
"""GPS 경로 추종에 장애물 회피 보정각만 더한다.

감지원은 두 가지다. detection_source 로 고른다.

  lidar   : /scan 의 좌우 측면 창(기본 +/-55도, 1.2 m)
  camera  : /camera/front/image_raw 에서 특정 색 덩어리를 찾는다

정적장애물이 연석 높이라 수평 스캔면이 그 위로 지나갈 수 있다. 라이다를
기울일 수 없어 카메라로 판단한다. 제어(보정 램프, 클램프, stale 정지)는
두 경우 모두 같다. 검증된 쪽을 그대로 쓴다.

어느 쪽이든 GPS 조향은 유지하고 보정만 더한다. 장애물이 화면/스캔의
왼쪽에 있으면 오른쪽으로(+), 오른쪽에 있으면 왼쪽으로(-) 피한다.
프로젝트 규약은 +angular.z = 오른쪽이다.
"""

import math
import time

import sys

sys.path = [p for p in sys.path
            if p != '/usr/local/lib/python3.12/dist-packages']

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

from .mission_common import scan_points


def image_to_bgr(msg):
    channels = 1 if msg.encoding in ('mono8', '8UC1') else 3
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    row_width = msg.step if msg.step else msg.width * channels
    image = raw.reshape((msg.height, row_width))[:, :msg.width * channels]
    image = image.reshape((msg.height, msg.width, channels))
    if channels == 1:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if msg.encoding in ('rgb8', 'RGB8'):
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image.copy()


def colour_mask(roi, hsv_low, hsv_high, hsv_low_2, hsv_high_2,
                open_px=3, close_px=7):
    """설정한 HSV 범위의 마스크. 빨강처럼 hue 가 0/180 양쪽에 걸치는
    색을 위해 두 번째 범위를 선택적으로 더한다."""
    hsv = cv2.cvtColor(cv2.GaussianBlur(roi, (5, 5), 0), cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, tuple(hsv_low), tuple(hsv_high))
    if hsv_high_2 and any(hsv_high_2):
        mask = cv2.bitwise_or(
            mask, cv2.inRange(hsv, tuple(hsv_low_2), tuple(hsv_high_2)))
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_OPEN, np.ones((open_px, open_px), np.uint8))
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_CLOSE, np.ones((close_px, close_px), np.uint8))
    return mask


def largest_blob(mask, min_area):
    """가장 큰 덩어리의 (면적, 중심x) 를 돌려준다. 없으면 None."""
    count, _, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    best = None
    for index in range(1, count):
        area = int(stats[index, cv2.CC_STAT_AREA])
        if area >= min_area and (best is None or area > best[0]):
            best = (area, float(centroids[index][0]))
    return best


def homography_from_points(image_points, ground_points):
    """대응점으로 지면 호모그래피를 만든다. 없거나 모자라면 None.

    image_points  : 화면 픽셀 [x1,y1, x2,y2, ...]
    ground_points : 앞범퍼 중앙 기준 실측 [전방m, 좌우m, ...] 왼쪽이 +

    4점이면 정확히 맞추고, 그 이상이면 최소제곱으로 푼다. 점이 많을수록
    한 점의 측정 오차가 덜 퍼진다.
    """
    if not image_points or not ground_points:
        return None
    if len(image_points) != len(ground_points):
        return None
    if len(image_points) < 8 or len(image_points) % 2:
        return None
    source = np.array(image_points, dtype=np.float32).reshape(-1, 2)
    target = np.array(ground_points, dtype=np.float32).reshape(-1, 2)
    if len(source) == 4:
        return cv2.getPerspectiveTransform(source, target)
    matrix, _ = cv2.findHomography(source, target, 0)
    return matrix


def ground_point(homography, pixel_x, pixel_y):
    """화면 점을 지면 좌표로 옮긴다. 반환은 (전방 m, 좌우 m).

    [주의] 지면 호모그래피는 지면에 있는 점만 맞다. 라바콘 꼭대기처럼
    떠 있는 점을 넣으면 실제보다 훨씬 멀게 나온다. 반드시 물체가 지면에
    닿는 점(덩어리 아랫변)을 넣어야 한다.
    """
    point = np.array([[[float(pixel_x), float(pixel_y)]]], dtype=np.float32)
    mapped = cv2.perspectiveTransform(point, homography)[0, 0]
    return float(mapped[0]), float(mapped[1])


def select_obstacle(marks, minimum_distance, look_distance, corridor_half_m):
    """피해야 하는 것 중 가장 가까운 것을 고른다.

    marks 는 (전방 m, 좌우 m, 부가정보) 목록이다. 진로 밖으로 충분히
    비켜 있으면 애초에 후보가 아니다. 거리로 고르므로 화면에서 크게
    보인다고 뽑히지 않는다.
    """
    best = None
    for distance, offset, extra in marks:
        if distance < minimum_distance or distance > look_distance:
            continue
        if abs(offset) > corridor_half_m:
            continue
        if best is None or distance < best[0]:
            best = (distance, offset, extra)
    return best


def bev_correction(distance, offset, desired_clearance, gain,
                   minimum_distance, max_correction, default_side=1.0):
    """실거리 기반 비례 보정각.

    need 는 '더 비켜야 하는 양'이다. 이미 충분히 옆으로 비켜 있으면 0 이라
    보정하지 않는다. 화면 한쪽에 보이기만 하면 무조건 틀던 것과 다르다.

    거리로 나누는 것은 순수추종과 같은 이유다. 멀면 완만하게, 가까우면
    급하게 든다.

    부호: offset 은 왼쪽이 +, 조향은 오른쪽이 + 다. 왼쪽 장애물이면
    오른쪽으로 피해야 하므로 그대로 곱한다.
    """
    need = desired_clearance - abs(offset)
    if need <= 0.0:
        return 0.0
    side = default_side if offset == 0.0 else math.copysign(1.0, offset)
    magnitude = gain * need / max(distance, minimum_distance)
    return min(magnitude, max_correction) * side


def corridor_polygon(inverse_homography, near, far, half_width,
                     y_offset=0):
    """지면 통로 사각형을 화면으로 되돌려 사다리꼴 네 점을 만든다.

    원근 때문에 같은 좌우 폭이라도 멀수록 화면에서 좁아진다. 그래서
    직사각형 ROI 를 쓰면 먼 쪽 좌우 끝이 통로 밖(갓길 잔디, 담장, 나무)
    이 된다. 그 배경이 색에 걸리면 근거리 덩어리와 하나로 이어지고,
    그러면 덩어리 아랫변이 실제 장애물이 아닌 곳을 가리킨다.

    반환은 ROI 좌표계((0,0) 이 ROI 왼쪽 위)의 네 점이다.
    """
    corners = []
    for distance, offset in ((far, half_width), (far, -half_width),
                             (near, -half_width), (near, half_width)):
        point = np.array([[[float(distance), float(offset)]]],
                         dtype=np.float32)
        mapped = cv2.perspectiveTransform(point, inverse_homography)[0, 0]
        # 아주 가까운 거리는 화면 밖으로 크게 벗어난다. fillConvexPoly 가
        # 잘라 주지만 int32 로 넘치지 않게 한 번 묶어 둔다.
        corners.append([
            int(max(-10000.0, min(10000.0, float(mapped[0])))),
            int(max(-10000.0, min(10000.0, float(mapped[1]) - y_offset)))])
    return corners


def corridor_mask(shape, corners):
    """사다리꼴 안쪽만 255 인 마스크."""
    mask = np.zeros(shape[:2], dtype=np.uint8)
    cv2.fillConvexPoly(mask, np.array(corners, dtype=np.int32), 255)
    return mask


def all_blobs(mask, min_area):
    """덩어리마다 (면적, 중심x, 아랫변y, 박스) 를 돌려준다."""
    count, _, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    found = []
    for index in range(1, count):
        area = int(stats[index, cv2.CC_STAT_AREA])
        if area < min_area:
            continue
        left = int(stats[index, cv2.CC_STAT_LEFT])
        top = int(stats[index, cv2.CC_STAT_TOP])
        width = int(stats[index, cv2.CC_STAT_WIDTH])
        height = int(stats[index, cv2.CC_STAT_HEIGHT])
        found.append((area, float(centroids[index][0]),
                      float(top + height), (left, top, width, height)))
    return found


def camera_correction(blob, image_width, correction_rad, centre_deadband_ratio):
    """장애물이 화면 어느 쪽에 있는지로 보정 방향을 정한다.

    화면 왼쪽에 있으면 오른쪽으로 피한다(+). 가운데 불감대 안이면 어느
    쪽으로 피할지 정할 수 없으므로 보정하지 않는다.
    """
    if blob is None:
        return 0.0
    offset = blob[1] - image_width * 0.5
    if abs(offset) < image_width * centre_deadband_ratio:
        return 0.0
    return correction_rad if offset < 0.0 else -correction_rad


def blended_command(gps_speed, gps_steer, correction, speed_limit, max_steer,
                    gps_weight=1.0):
    """GPS 곡선을 유지한 채 제한된 보정만 더한다.

    gps_weight 는 장애물 바로 앞에서 GPS 조향의 영향력을 낮추기 위한
    것이다. 1.0 이면 예전과 같다.

    [왜 필요한가] 보정을 더하기만 해서는 실제로 안 비켜진다. 순수추종이
    경로로 되당기는 힘(LAD 1.0 에서 매우 강하다)이 보정을 거의 그대로
    상쇄하기 때문이다. 실측(모의) 로 지시 1.41 m 에 실제 0.3 m 였다.
    """
    msg = Twist()
    msg.linear.x = min(max(0.0, gps_speed), speed_limit)
    msg.angular.z = max(-max_steer,
                        min(max_steer, gps_steer * gps_weight + correction))
    return msg


class ObstacleAvoidance(Node):
    # 로그 억제용 상태. 클래스 속성으로 둔 것은 테스트가 __init__ 을
    # 건너뛰고 객체를 만들기 때문이다. 인스턴스에서 덮어쓴다.
    stop_log_time = 0.0
    detect_log_time = 0.0
    last_label = None
    takeover = False
    gps_weight = 1.0
    takeover_seen = 0.0

    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        defaults = {
            'gps_input_topic': '/cmd_vel/gps_safe',
            'speed_mps': 0.56,
            'steer_rad': 0.12,
            # [2026-09-20] 0.20 -> 0.24. follower 자신이 0.24 를 쓰고
            # 기계 락은 0.267 이다. 회피 노드만 더 조일 이유가 없었고,
            # 굽은 구간에서 보정 몫을 스스로 깎고 있었다.
            # 인계 중에는 이 값이 실질 조향 한계가 된다.
            'maximum_steering_rad': 0.24,
            'correction_step_rad': 0.025,
            'detect_distance_m': 1.2,
            # side_center_deg / sector_half_deg 는 '차량 기준' 각도다.
            # 정면 0, 왼쪽 +, 오른쪽 -. mission_common.scan_points 는 raw
            # 스캔 각도를 쓰므로 forward_angle_deg 를 더해서 넘긴다.
            #
            # 이 프로젝트는 raw 180도가 차량 정면이다(주차 선택기/섹터
            # 필터의 forward_angle_deg 와 같은 규약).
            #
            # 이것은 수평(yaw) 기준각이다. 라이다를 아래로 기울이는
            # 수직각과는 무관하다. RPLIDAR C1 은 2D 라이다라 수직각을
            # 소프트웨어로 바꿀 수 없다.
            #
            # tick()이 매번 get_parameter 로 읽으므로 실차에서 노드를 다시
            # 띄우지 않고 바꿀 수 있다.
            #   ros2 param set /obstacle_avoidance_node forward_angle_deg 185.0
            'forward_angle_deg': 180.0,
            'side_center_deg': 55.0,
            'sector_half_deg': 25.0,
            'min_points': 3,
            'sensor_timeout_sec': 0.5,
            # lidar | camera. 정적장애물이 연석 높이라 수평 스캔면이 그
            # 위로 지나갈 수 있어 대회에서는 camera 를 쓴다.
            'detection_source': 'camera',
            'image_topic': '/camera/front/image_raw',
            # 장애물 색 (HSV). 두 번째 범위는 빨강처럼 hue 가 0/180 양쪽에
            # 걸치는 색을 위한 것이다. 안 쓰면 0 으로 둔다.
            # 실차에서 /vision/avoid_debug 를 보며 맞춘다.
            # [2026-09-19] 실차 사진(photo/IMG_8848~8858)에서 주황 차량과
            # 마른 잔디의 픽셀을 직접 재어 잡았다. 그전 주석은 '구분되는
            # 것은 채도' 라고 했는데 측정 결과는 반대다. 분리자는 hue 다.
            #
            #   햇빛 받은 차체   H 중앙 10   S 중앙 158
            #   마른 잔디        H 중앙 25   S 중앙  76
            #
            # 채도로 가르면(S>=150) 마른 잔디는 확실히 빠지지만 햇빛에
            # 바랜 차체까지 같이 빠진다. 그 설정에서는 차체의 43.8% 만
            # 잡혔고, 그것도 대부분 '그늘진 실내' 라 덩어리가 작고 조각나
            # 위치도 틀렸다(바닥 접점이 차 밑이 아니라 실내 모서리가 됨).
            #
            # hue 로 가르면 둘이 깨끗이 갈린다. 격자 탐색 결과
            #   H 5~16, S>=80, V>=80  ->  차체 73.0%, 마른 잔디 0.10%
            #   H 5~22, S>=150, V>=110 -> 차체 43.8%, 마른 잔디 0.00%
            # 베이지색 방호벽과 흙도 이 범위에 안 들어온다.
            #
            # hue 상한 16 이 핵심이다. 18 로만 올려도 잔디가 새기 시작한다.
            # 실차에서 장애물을 놓치면 채도 하한을 60 쯤으로 먼저 내리고,
            # hue 상한은 건드리지 않는 편이 낫다.
            #   ros2 param set /obstacle_avoidance_node hsv_low "[5,60,80]"
            'hsv_low': [5, 80, 80],
            'hsv_high': [16, 255, 255],
            'hsv_low_2': [0, 0, 0],
            'hsv_high_2': [0, 0, 0],
            # 이 비율보다 작은 덩어리는 무시한다 (ROI 면적 대비).
            'min_area_ratio': 0.002,
            # 화면 가운데 이 비율 안이면 어느 쪽으로 피할지 못 정한다.
            'centre_deadband_ratio': 0.04,
            # [2026-09-19] roi_y_min 을 0.35 -> 0.55 로 내렸다.
            #
            # 출하된 BEV 캘리브레이션으로 화면 행이 실제 몇 m 인지 재보니
            # 위쪽 절반이 통째로 쓸모가 없었다(640x480, x=320 기준).
            #   y=140 (0.29) -> -402 m   지평선 위. 부호가 뒤집힌다
            #   y=168 (0.35) ->  +22 m   옛 ROI 상단
            #   y=240 (0.50) -> +5.2 m
            #   y=264 (0.55) -> +4.0 m   look_distance_m 과 같은 지점
            #   y=291 (0.61) -> +3.0 m   캘리브 최원점
            #   y=431 (0.90) -> +1.0 m   캘리브 최근점
            #   y=479 (1.00) -> +0.7 m
            #
            # select_obstacle 이 look_distance_m(4.0) 너머를 어차피 버리므로
            # 0.55 위쪽은 후보가 될 수 없는 배경만 넣고 있었다. 그런데
            # 그 배경(잔디밭, 생울타리, 나무)이 색에 걸리면 근거리 덩어리와
            # 하나로 이어져 버린다. 그러면 덩어리 아랫변이 실제 장애물이
            # 아닌 곳을 가리키고, 지면 변환이 그 점을 아주 가깝다고 읽어
            # 최대 조향이 걸린다.
            #
            # 아랫변만 쓰므로 위를 잘라도 검출 거리는 안 줄어든다.
            # 4 m 짜리 장애물은 아랫변이 y=264 라 그대로 잡힌다.
            # [2026-09-20] look_distance_m 을 5.5 m 로 늘리면서 같이 내렸다.
            # ROI 상단이 look_distance 보다 가까우면 그만큼 못 본다.
            #   0.55 -> 3.93 m   0.51 -> 4.88 m   0.49 -> 5.49 m
            #   0.50 -> 5.17 m   0.48 -> 5.85 m   0.47 -> 6.25 m
            # [실측 2026-09-20] 실제 유효 검출 거리는 약 5.2 m 다.
            # look_distance_m 5.5 보다 조금 짧다. 덩어리의 아랫변이
            # ROI 상단(5.49 m)에 닿는 거리부터는 위가 잘려 min_area 를
            # 못 넘기기 때문이다. 더 멀리 보려면 0.48(5.85 m) 로 내리면
            # 되지만, 모의에서 look 3.0 과 5.5 의 결과가 같았다. 실제
            # 회피는 3 m 근접 인계가 만들어내므로 그대로 둔다.
            'roi_y_min': 0.49,
            'roi_y_max': 1.0,
            # ROI 를 직사각형이 아니라 지면 통로 모양(사다리꼴)로 자른다.
            # 원근 때문에 먼 쪽일수록 화면에서 좁아지므로, 직사각형이면
            # 상단 좌우 끝이 갓길/담장/나무가 된다.
            #
            # 실측(4.0 m 지점, 640 폭): ±1.0 m -> 249 px(39%),
            # ±1.5 m -> 374 px(58%), ±2.0 m -> 499 px(78%)
            #
            # 1.5 는 충돌 통로(desired_clearance_m 0.55)의 약 3 배다.
            # select_obstacle 이 어차피 ±0.55 m 밖을 버리므로 잘려서
            # 놓칠 장애물은 없고, 먼 쪽 화면의 42% 를 덜어낸다.
            # 0 이하로 두면 예전처럼 전폭 직사각형을 쓴다.
            # [2026-09-20] 1.5 -> 2.0. desired_clearance_m 이 1.25 가 되면서
            # 통로 끝에 걸친 장애물의 가장자리가 1.25+0.63 = 1.88 m 까지
            # 간다. 1.5 로 자르면 덩어리가 잘려 중심이 안쪽으로 밀리고
            # offset 이 실제보다 작게 나온다.
            # 중심이 통로 끝(1.41 m)에 걸친 장애물의 가장자리는
            # 1.41 + 0.728 = 2.14 m 까지 간다. 그보다 좁게 자르면 덩어리가
            # 잘려 중심이 안쪽으로 밀리고 offset 이 작게 나온다.
            # 5.5 m 지점에서 화면 폭의 66% 다.
            'roi_corridor_half_m': 2.2,
            'publish_debug_image': True,
            # --- 버드아이뷰 캘리브레이션 ---
            # 앞범퍼가 보이게 장착하고 지면에 실측 표식을 놓아 채운다.
            # 둘 다 8개(4점) 이상이고 길이가 같아야 쓴다. 아니면 아래의
            # 화면 좌우 이진 판단으로 자동 폴백한다. 장착/캘리브레이션
            # 전에도 동작이 깨지지 않는다.
            #   bev_image_points  화면 픽셀  [x1,y1, x2,y2, ...]
            #   bev_ground_points 실측       [전방m, 좌우m, ...] 왼쪽이 +
            #                                원점은 앞범퍼 중앙, 지면
            # [캘리브레이션 2026-09-19] C920 전방 장착, 640x480.
            # 지면 표식 6개(전방 1/2/3 m, 좌우 +/-0.6 m)를 찍어 풀었다.
            # 역투영 잔차 전방 1.8 cm / 좌우 1.0 cm.
            # 카메라를 옮기거나 각도가 바뀌면 전부 다시 재야 한다.
            #   python3 bev_calib/grab.py       프레임 저장
            #   python3 bev_calib/pick_points.py  픽셀 좌표 추출
            'bev_image_points': [
                183.5, 431.0,   # 전방 1.0 m, 좌 +0.6
                539.1, 431.0,   # 전방 1.0 m, 우 -0.6
                241.4, 337.0,   # 전방 2.0 m, 좌 +0.6
                485.4, 341.0,   # 전방 2.0 m, 우 -0.6
                265.0, 291.0,   # 전방 3.0 m, 좌 +0.6
                452.2, 293.0,   # 전방 3.0 m, 우 -0.6
            ],
            'bev_ground_points': [
                1.0, 0.6,
                1.0, -0.6,
                2.0, 0.6,
                2.0, -0.6,
                3.0, 0.6,
                3.0, -0.6,
            ],
            # 이만큼은 옆으로 비켜서 지나가고 싶다 (m). 차폭 0.78 m 이므로
            # 반폭 0.39 m 에 여유를 더한 값이다.
            # [2026-09-20] 실측값으로 계산했다.
            #   장애물 1.26 x 0.73 m, 높이 0.28 m, 비스듬히 놓인다
            #   차폭 0.8 m, 차선 폭 4.2 m
            #
            # 핵심 1: select_obstacle 과 bev_correction 이 보는 offset 은
            # 장애물의 '중심' 이다. 옛 0.55 로는 중심을 비켜도 가장자리가
            # 진로에 남아, 회피에 '성공' 해도 부딪혔다.
            #
            # 핵심 2: 비스듬히 놓이면 정면으로 막을 때보다 더 넓게 막는다.
            # 직사각형의 횡방향 최대 투영은 대각선 길이다.
            #   긴변이 횡축과 0도  -> 1.260 m
            #                 15도 -> 1.406 m
            #                 30도 -> 1.456 m   <- 최대 (= 대각선)
            #                 45도 -> 1.407 m
            #                 90도 -> 0.730 m
            # 그래서 횡폭 1.456 m (반폭 0.728) 을 설계값으로 쓴다.
            #
            # 차선 4.2 m 안에서 내 차 중심이 있을 수 있는 범위
            #   하한 0.728 + 0.40 = 1.13 m   (장애물에 안 닿는다)
            #   상한 2.10  - 0.40 = 1.70 m   (차선 안에 남는다)
            # 폭이 0.57 m 뿐이다. 양쪽 여유가 같아지는 1.41 m 를 쓴다.
            #   장애물까지 +0.28 m,  차선까지 +0.29 m
            #
            # 이만큼 비키려면 그만한 거리가 필요하다. y ~ s^2/(2R),
            # R = wheelbase/tan(steer), wheelbase 1.0 보수적으로 잡으면
            #   look 5.0 m -> 0.127 rad   보정 상한 0.12 초과
            #   look 5.5 m -> 0.104 rad   OK
            #   look 6.0 m -> 0.087 rad   여유
            # 5.5 m 로 잡고 roi_y_min 도 같이 내렸다(0.49 = 5.49 m).
            #
            # 5.5 m 에서도 검출은 충분하다. 주황 픽셀 약 1400 개로
            # min_area(276) 의 5 배다. 거리 추정 오차도 -0.07 m 다.
            'desired_clearance_m': 1.41,
            # 이 거리 안의 장애물만 본다 (m).
            'look_distance_m': 5.5,
            # 이보다 가까우면 차체이거나 이미 지나친 것이다 (m).
            'minimum_distance_m': 0.3,
            # --- 근접 인계 ---------------------------------------------
            # 장애물이 이 거리 안으로 들어오면 GPS 조향의 영향력을 낮추고
            # 보정이 주도권을 갖는다. 카메라에서 장애물이 사라지면 원래대로
            # 되돌린다(래치).
            #
            # 그냥 보정을 더하기만 하면 순수추종이 되당겨서 실제로는 거의
            # 안 비켜진다. 모의 결과(장애물 0.8 m 치우침, 필요 1.128 m):
            #   인계 없음                      최근접 0.96 m  부딪힘
            #   인계 3 m, w 0.2, cap 0.24      최근접 1.30 m  통과
            #
            # w 를 0 으로 두면 중앙 장애물도 피하지만 차선을 벗어난다.
            # 이 대회에서 장애물은 절대 중앙이 아니므로 0.2 를 쓴다.
            'takeover_distance_m': 3.0,
            'takeover_gps_weight': 0.2,
            # 인계 중에는 보정이 곧 전체 조향이다. 평소 상한 0.12 를 그대로
            # 쓰면 그게 조향 한계가 되어 버린다. follower 와 같은 0.24 로 푼다.
            'takeover_steer_rad': 0.24,
            'takeover_gain': 1.0,
            # 가중치를 계단으로 바꾸면 조향이 튄다.
            'takeover_weight_rate': 2.0,
            # 장애물이 시야에서 사라져도 이만큼은 인계와 마지막 보정을
            # 유지한다. 안 그러면 차체가 아직 장애물 옆을 지나는 중에
            # 조향이 원래대로 돌아가 다시 붙는다.
            #
            # 검출은 장애물이 센서 앞 minimum_distance_m(0.3 m) 안으로
            # 들어오면 끊긴다. 그 시점에 앞범퍼만 지난 상태다.
            #   앞범퍼가 장애물 뒷면 통과  1.76 m -> 3.2 초 (전체의 56%)
            #   뒷범퍼까지 완전 통과       3.16 m -> 5.7 초
            # 앞범퍼만 지나면 복귀해도 된다. 후륜축 기준 회전이라 장애물
            # 쪽으로 dψ 틀면 뒷 모서리(후륜축 뒤 0.37 m)는 오히려
            # 0.37*dψ 만큼 반대쪽으로 벌어진다.
            #   20도 복귀 -> 뒷모서리 0.13 m 바깥
            #
            # 3.2 초는 장애물이 가장 나쁜 각도(60도, 진행방향 1.46 m)로
            # 놓였을 때 값이다. 5.7 초를 물면 차가 61도 돌아 헤어핀에서
            # 경로를 잃는다(3.2 초는 34도).
            'takeover_hold_sec': 3.2,
            # 보정각 = gain * need / distance. rad*m/m 단위다.
            'bev_gain': 0.30,
            # 정확히 정면(offset 0)일 때 어느 쪽으로 피할지. +1 오른쪽.
            'bev_default_side': 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.active = False
        self.scan = None
        self.scan_time = 0.0
        self.gps_command = None
        self.gps_time = 0.0
        self.correction = 0.0
        self.source = str(self.value('detection_source')).strip().lower()
        if self.source not in ('lidar', 'camera'):
            raise ValueError("detection_source must be 'lidar' or 'camera'")
        self.homography = homography_from_points(
            list(self.value('bev_image_points')),
            list(self.value('bev_ground_points')))
        self.inverse_homography = (
            None if self.homography is None
            else np.linalg.inv(self.homography))
        self.frame = None
        self.frame_time = 0.0
        # 정지/검출 로그는 매 틱 찍으면 로그가 넘친다.
        self.stop_log_time = 0.0
        self.detect_log_time = 0.0
        self.last_label = None
        self.pub = self.create_publisher(Twist, '/cmd_vel/avoid', 10)
        self.debug_pub = self.create_publisher(Image, '/vision/avoid_debug', 2)
        self.create_subscription(
            String, '/t870/active_mission', self.mode_callback, 10)
        if self.source == 'lidar':
            self.create_subscription(
                LaserScan, '/scan', self.scan_callback, 10)
        else:
            self.create_subscription(
                Image, str(self.value('image_topic')),
                self.image_callback, 5)
        self.create_subscription(
            Twist, str(self.get_parameter('gps_input_topic').value),
            self.gps_callback, 10)
        self.create_timer(0.05, self.tick)

    def value(self, name):
        return self.get_parameter(name).value

    def mode_callback(self, msg):
        self.active = msg.data == 'AVOID'
        if not self.active:
            self.correction = 0.0
            # 구간을 벗어나면 인계도 반드시 푼다. 남아 있으면 다음 구간에서
            # GPS 조향이 약한 채로 달리게 된다.
            self.set_takeover(False, None)
            self.gps_weight = 1.0

    def scan_callback(self, msg):
        self.scan = msg
        self.scan_time = time.monotonic()

    def image_callback(self, msg):
        self.frame = msg
        self.frame_time = time.monotonic()

    def set_takeover(self, engaged, distance):
        """근접 인계 래치. 걸리고 풀리는 순간만 로그로 남긴다."""
        if engaged == self.takeover:
            return
        self.takeover = engaged
        if engaged:
            self.get_logger().warning(
                'AVOID 인계: 장애물 %.2f m. GPS 조향 가중치를 %.1f 로 낮춘다'
                % (distance, float(self.value('takeover_gps_weight'))))
        else:
            self.get_logger().warning(
                'AVOID 인계 해제: 장애물이 안 보인다. GPS 조향 복귀')

    def update_gps_weight(self, dt):
        """가중치를 램프로 바꾼다. 계단으로 바꾸면 조향이 튄다."""
        target = (float(self.value('takeover_gps_weight'))
                  if self.takeover else 1.0)
        step = abs(float(self.value('takeover_weight_rate'))) * max(0.0, dt)
        self.gps_weight += max(-step, min(step, target - self.gps_weight))
        return self.gps_weight

    def corridor_corners(self, y_offset):
        """사다리꼴 네 점. 캘리브레이션이나 설정이 없으면 None."""
        if self.inverse_homography is None:
            return None
        half = float(self.value('roi_corridor_half_m'))
        if half <= 0.0:
            return None
        # 윗변을 look_distance 에 딱 맞추면 그 거리의 장애물이 경계선상에
        # 놓여 일부가 잘린다. 여유를 둬서 '사다리꼴은 폭만, ROI 세로는
        # 깊이만' 담당하게 한다.
        #
        # 지면의 직선은 호모그래피로 화면에서도 직선이 되므로, 옆변은
        # offset=±half 인 지면 직선의 정확한 상이다. 윗변만 밀어 올리는
        # 것이라 통로 폭은 어느 거리에서도 달라지지 않는다.
        return corridor_polygon(
            self.inverse_homography,
            float(self.value('minimum_distance_m')),
            float(self.value('look_distance_m')) * 1.2,
            half, y_offset)

    def camera_target(self):
        """카메라에서 색 덩어리를 찾아 보정 목표각을 돌려준다.

        캘리브레이션이 있으면 실거리 기반 비례 보정, 없으면 화면 좌우
        이진 판단으로 폴백한다.
        """
        frame = image_to_bgr(self.frame)
        height, width = frame.shape[:2]
        y0 = int(height * float(self.value('roi_y_min')))
        y1 = int(height * float(self.value('roi_y_max')))
        roi = frame[y0:y1, :]
        mask = colour_mask(
            roi,
            list(self.value('hsv_low')), list(self.value('hsv_high')),
            list(self.value('hsv_low_2')), list(self.value('hsv_high_2')))
        # 지면 통로 모양으로 한 번 더 자른다. 직사각형 ROI 의 먼 쪽 좌우
        # 끝은 통로 밖이라, 거기 걸린 배경이 근거리 덩어리와 이어지면
        # 아랫변이 엉뚱한 곳을 가리킨다.
        corridor = self.corridor_corners(y0)
        if corridor is not None:
            mask = cv2.bitwise_and(mask, corridor_mask(roi.shape, corridor))
        min_area = max(
            40, int(roi.shape[0] * roi.shape[1]
                    * float(self.value('min_area_ratio'))))

        chosen = None
        if self.homography is None:
            blob = largest_blob(mask, min_area)
            target = camera_correction(
                blob, width, float(self.value('steer_rad')),
                float(self.value('centre_deadband_ratio')))
            label = 'no BEV calib (left/right only)'
            if blob is not None:
                chosen = (0.0, 0.0, (int(blob[1]) - 10, 0, 20, y1 - y0))
        else:
            clearance = float(self.value('desired_clearance_m'))
            marks = []
            for area, cx, bottom, box in all_blobs(mask, min_area):
                # 지면에 닿는 점만 변환이 맞다. 덩어리 아랫변을 쓴다.
                distance, offset = ground_point(
                    self.homography, cx, bottom + y0)
                marks.append((distance, offset, box))
            chosen = select_obstacle(
                marks,
                float(self.value('minimum_distance_m')),
                float(self.value('look_distance_m')),
                clearance)
            if chosen is None:
                hold = float(self.value('takeover_hold_sec'))
                left = hold - (time.monotonic() - self.takeover_seen)
                if self.takeover and left > 0.0:
                    # 차체가 아직 장애물 옆을 지나는 중이다. 마지막 보정을
                    # 그대로 물고 간다. 여기서 놓으면 다시 붙는다.
                    target = self.correction
                    label = 'clear, 통과까지 유지 %.1f s' % left
                else:
                    target = 0.0
                    label = 'clear'
                    self.set_takeover(False, None)
            else:
                # 장애물이 인계 거리 안으로 들어오면 주도권을 넘긴다.
                # 한 번 걸리면 카메라에서 사라질 때까지 유지한다.
                self.takeover_seen = time.monotonic()
                if chosen[0] <= float(self.value('takeover_distance_m')):
                    self.set_takeover(True, chosen[0])
                gain = (float(self.value('takeover_gain')) if self.takeover
                        else float(self.value('bev_gain')))
                cap = (float(self.value('takeover_steer_rad'))
                       if self.takeover else float(self.value('steer_rad')))
                target = bev_correction(
                    chosen[0], chosen[1], clearance, gain,
                    float(self.value('minimum_distance_m')), cap,
                    float(self.value('bev_default_side')))
                label = 'ahead %.2f m, offset %+.2f m%s' % (
                    chosen[0], chosen[1], ' [인계]' if self.takeover else '')

        if bool(self.value('publish_debug_image')):
            self.publish_debug(frame, y0, y1, chosen, target, label)
        # 디버그 이미지를 현장에서 못 볼 때가 많다. 무엇을 보고 얼마나
        # 틀고 있는지 글로도 남긴다. 바뀔 때, 아니면 2 초마다.
        now = time.monotonic()
        if label != self.last_label or now - self.detect_log_time >= 2.0:
            self.last_label = label
            self.detect_log_time = now
            self.get_logger().warning(
                'AVOID 카메라: %s, 보정 %+.3f rad' % (label, target))
        return target

    def publish_debug(self, frame, y0, y1, chosen, target, label):
        height, width = frame.shape[:2]
        debug = frame.copy()
        cv2.rectangle(debug, (0, y0), (width - 1, y1 - 1), (255, 180, 0), 1)
        corridor = self.corridor_corners(0)
        if corridor is not None:
            cv2.polylines(debug, [np.array(corridor, dtype=np.int32)],
                          True, (255, 180, 0), 2)
        if self.inverse_homography is None:
            cv2.line(debug, (width // 2, y0), (width // 2, y1),
                     (200, 200, 200), 1)
        else:
            self.draw_ground_guides(debug)
        if chosen is not None:
            bx, by, bw, bh = chosen[2]
            cv2.rectangle(debug, (bx, y0 + by), (bx + bw, y0 + by + bh),
                          (0, 0, 255), 3)
            cv2.circle(debug, (int(bx + bw * 0.5), int(y0 + by + bh)), 6,
                       (0, 255, 255), -1)
        for row, text in enumerate((label,
                                    'correction %+.3f rad' % target)):
            cv2.putText(debug, text, (16, 30 + row * 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2,
                        cv2.LINE_AA)
        out = Image()
        out.header = self.frame.header
        out.height, out.width = debug.shape[:2]
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = out.width * 3
        out.data = debug.tobytes()
        self.debug_pub.publish(out)

    def draw_ground_guides(self, debug):
        """차폭 통로와 거리 눈금을 사진 위에 되돌려 그린다.

        캘리브레이션이 맞는지 눈으로 바로 확인하는 용도다. 통로선이
        실제 노면과 나란하지 않으면 호모그래피가 틀린 것이다.
        """
        clearance = float(self.value('desired_clearance_m'))
        near = float(self.value('minimum_distance_m'))
        far = float(self.value('look_distance_m'))

        def to_pixel(distance, offset):
            point = np.array([[[float(distance), float(offset)]]],
                             dtype=np.float32)
            mapped = cv2.perspectiveTransform(
                point, self.inverse_homography)[0, 0]
            return int(mapped[0]), int(mapped[1])

        for offset in (-clearance, clearance):
            cv2.line(debug, to_pixel(near, offset), to_pixel(far, offset),
                     (0, 200, 255), 2)
        step = 1.0
        distance = math.ceil(near / step) * step
        while distance <= far:
            left = to_pixel(distance, clearance)
            right = to_pixel(distance, -clearance)
            cv2.line(debug, left, right, (0, 200, 255), 1)
            cv2.putText(debug, '%.0fm' % distance,
                        (right[0] + 6, right[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1,
                        cv2.LINE_AA)
            distance += step

    def lidar_target(self):
        center = float(self.value('side_center_deg'))
        half = float(self.value('sector_half_deg'))
        distance = float(self.value('detect_distance_m'))
        forward = float(self.value('forward_angle_deg'))
        # 차량 기준 각도를 raw 로 옮긴다. 왼쪽이 +, 오른쪽이 -.
        left = scan_points(
            self.scan, forward + center, half, max_range=distance)
        right = scan_points(
            self.scan, forward - center, half, max_range=distance)
        minimum = int(self.value('min_points'))
        if len(left) < minimum and len(right) < minimum:
            return 0.0
        left_nearest = min((point[1] for point in left), default=999.0)
        right_nearest = min((point[1] for point in right), default=999.0)
        correction = float(self.value('steer_rad'))
        # Positive steering is right: turn away from the nearer side.
        return correction if left_nearest < right_nearest else -correction

    def gps_callback(self, msg):
        if math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z):
            self.gps_command = msg
            self.gps_time = time.monotonic()

    def approach(self, target):
        step = float(self.value('correction_step_rad'))
        delta = max(-step, min(step, target - self.correction))
        self.correction += delta

    def tick(self):
        if not self.active:
            return
        now = time.monotonic()
        timeout = float(self.value('sensor_timeout_sec'))
        if self.source == 'lidar':
            sensor_stale = (self.scan is None
                            or now - self.scan_time > timeout)
        else:
            sensor_stale = (self.frame is None
                            or now - self.frame_time > timeout)
        if self.gps_command is None or now - self.gps_time > timeout:
            # GPS 명령 자체가 없으면 따라갈 경로가 없다. 이때만 선다.
            self.pub.publish(Twist())
            if now - self.stop_log_time >= 2.0:
                self.stop_log_time = now
                self.get_logger().warning(
                    'AVOID 정지 (속도 0 발행): GPS 명령 %s'
                    % ('없음' if self.gps_command is None
                       else '%.1f 초 끊김' % (now - self.gps_time)))
            return

        if sensor_stale:
            # [2026-09-19] 예전에는 여기서도 속도 0 을 냈다. 그러면 카메라가
            # 0.5 초만 끊겨도 정적장애물 구간에서 차가 완전히 서고 그대로
            # 주행이 끝난다. C920 은 대회 준비 중에만 세 번 끊겼다(유령
            # 프로세스의 장치 점유, 연속 읽기 실패, device_name 충돌).
            #
            # 카메라는 회피 보조일 뿐이고 GPS 경로는 그 자체로 유효하다.
            # 완전 정지는 실패가 확정이지만 경로를 계속 따라가면 장애물을
            # 피해 갈 여지가 있다. 그래서 보정만 0 으로 되돌리고 간다.
            # 장애물을 스칠 수 있다는 것은 감수한 위험이다.
            #
            # 되돌릴 때도 approach 로 램프를 태운다. 틀어 둔 조향을 한 번에
            # 0 으로 되돌리면 그 자체가 급조향이다.
            # 볼 수 없으면 인계도 풀어야 한다. 장애물이 어디 있는지
            # 모르는 채로 GPS 조향만 약하게 두면 경로를 놓친다.
            self.set_takeover(False, None)
            self.approach(0.0)
            self.pub.publish(blended_command(
                self.gps_command.linear.x,
                self.gps_command.angular.z,
                self.correction,
                float(self.value('speed_mps')),
                float(self.value('maximum_steering_rad')),
                self.update_gps_weight(0.05)))
            if now - self.stop_log_time >= 2.0:
                self.stop_log_time = now
                source = '라이다' if self.source == 'lidar' else '카메라'
                data = self.scan if self.source == 'lidar' else self.frame
                stamp = (self.scan_time if self.source == 'lidar'
                         else self.frame_time)
                self.get_logger().warning(
                    'AVOID 센서 끊김 (%s %s). 회피를 끄고 GPS 경로로 간다'
                    % (source, '입력 없음' if data is None
                       else '%.1f 초 지연' % (now - stamp)))
            return

        try:
            target = (self.lidar_target() if self.source == 'lidar'
                      else self.camera_target())
        except (ValueError, cv2.error) as error:
            # 센서 끊김과 같은 이유로 여기서도 서지 않는다. 볼 수 없다는
            # 것은 같고, 완전 정지는 실패가 확정이다.
            self.get_logger().warning(
                'AVOID 검출 실패(%s). 회피를 끄고 GPS 경로로 간다' % error)
            self.set_takeover(False, None)
            self.approach(0.0)
            self.pub.publish(blended_command(
                self.gps_command.linear.x,
                self.gps_command.angular.z,
                self.correction,
                float(self.value('speed_mps')),
                float(self.value('maximum_steering_rad')),
                self.update_gps_weight(0.05)))
            return
        self.approach(target)
        self.pub.publish(blended_command(
            self.gps_command.linear.x,
            self.gps_command.angular.z,
            self.correction,
            float(self.value('speed_mps')),
            float(self.value('maximum_steering_rad')),
            self.update_gps_weight(0.05)))


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
