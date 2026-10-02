#!/usr/bin/env python3
"""신호등 램프 색 판정 (원형 램프 전용).

lane_camera_preview 에 있던 detect_traffic_light 을 그대로 옮겼다. 대회에서
신호등을 노트북 카메라로 옮기면서 두 번째 사용처가 생겼다.
lane_camera_preview 를 import 하면 torch 와 LaneNet 까지 딸려온다.

[주의] 이것은 원형 램프 전용이다. 마지막 WP 의 화살표/X 표지에는 쓸 수
없다. 종횡비 0.45~1.65, 원형도 0.42 로 거르기 때문에 길쭉한 화살표와
획이 갈라진 X 는 전부 탈락한다. 그쪽은 ox_signal_detector_node 가 맡는다.
"""
import math
import sys

# /usr/local 에 깨진 opencv 가 있어 libopencv_hdf 를 못 찾는다.
sys.path = [p for p in sys.path
            if p != '/usr/local/lib/python3.12/dist-packages']

import cv2
import numpy as np


def detect_traffic_light(frame, green_preference=0.5,
                         roi_height_ratio=0.8, area_ceiling_ratio=0.05):
    """Detect a compact red/yellow/green lamp in the upper camera image.

    Traffic lamps are deliberately detected before the ground-plane warp:
    elevated objects are not geometrically meaningful in a bird-eye image.
    """
    # [2026-09-19] 면적 상한을 0.004 -> 0.05 로 풀고 ROI 를 0.55 -> 0.8
    # 로 넓혔다.
    #
    # 면적 상한은 0.05 다. '가까운 램프를 버리지 않으면서 명백히 램프일
    # 수 없는 덩어리만 버리는' 값이다. 640x480, HFOV 60 도, 램프 300 mm
    # 기준 화면상 면적은 21642/d^2 px 이므로
    #   3 m -> 2400(0.008), 2 m -> 5410(0.018), 1.2 m -> 15000(0.049)
    # 즉 0.05 는 1.2 m 까지 덮는다(화각 50 도로 좁아도 1.5 m). 옛 값
    # 0.004 는 4.5 m 라 정지선에서 검출기가 꺼졌다. 12 배 넉넉해진 것이다.
    # 그러면서 프레임의 5%(약 124x124 px) 를 넘는 것은 계속 버린다.
    #
    #
    # 실차에서 정지선에 선 뒤로는 초록불이 켜져도 GREEN 이 한 번도 안
    # 나왔다. 램프가 '너무 커서' 탈락하고 있었다. 화면상 면적은 거리의
    # 제곱에 반비례하므로 프레임 대비 고정 비율 상한은 사실상 '최소
    # 동작 거리' 를 박아 넣는다. 옛 상한 0.004(1228 px) 는 640x480,
    # HFOV 60 도, 램프 300 mm 기준 약 4.5 m 였다. 하필 그 거리가
    # 정지선이라, 검출기가 가장 필요한 지점에서 꺼져 있었다.
    # 게다가 실패가 조용하다. 상한을 넘으면 색을 틀리게 보는 게 아니라
    # UNKNOWN 이 되는데, UNKNOWN 은 표도 안 주고 해제도 안 시킨다.
    #
    # 상한이 잡으려던 큰 색덩어리는 뒤의 종횡비 0.45~1.65 와 원형도
    # 0.42 가 이미 거의 다 거른다. 그리고 이 판정은 판단 구간(각 6 WP)
    # 과 정지선에서만 쓰이므로 그 밖의 오검출은 무시된다.
    # area_ceiling_ratio 가 0 이하면 상한 없음이다.
    #
    # ROI 는 남긴다. 상한과 달리 실제 정보가 있다. 신호등은 지평선 위에
    # 있고 화면 아래는 노면, 정지선 도색, 잔디, 주황 라바콘이다.
    # 정적장애물이 주황이라 노면을 열면 YELLOW 마스크에 걸린다.
    # 0.8 = 화면 아래 1/5 만 잘라낸다.
    height, width = frame.shape[:2]
    y_limit = int(height * roi_height_ratio)
    x_left, x_right = int(width * 0.05), int(width * 0.95)
    roi = frame[:y_limit, x_left:x_right]
    blurred = cv2.GaussianBlur(roi, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    masks = {
        'RED': cv2.bitwise_or(
            cv2.inRange(hsv, (0, 120, 100), (12, 255, 255)),
            cv2.inRange(hsv, (168, 120, 100), (180, 255, 255))),
        'YELLOW': cv2.inRange(hsv, (14, 120, 110), (38, 255, 255)),
        # [2026-09-19] 초록 채도 하한을 90 -> 55 로 낮췄다. 실차에서
        # 초록불인데 GREEN 이 한 번도 안 잡혀 정지선에서 못 나간 적이
        # 있다. 햇빛에 바랜 LED 는 채도가 50 대까지 떨어진다(BEV 표식도
        # 43 까지 내려갔다). hue 상한도 100 으로 조금 넓혔다.
        'GREEN': cv2.inRange(hsv, (38, 55, 60), (100, 255, 255)),
    }
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    candidates = []
    frame_area = width * height
    area_ceiling = (frame_area * area_ceiling_ratio
                    if area_ceiling_ratio > 0.0 else float('inf'))
    for state, mask in masks.items():
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < max(8.0, frame_area * 0.000025) or area > area_ceiling:
                continue
            x, y, box_width, box_height = cv2.boundingRect(contour)
            aspect = box_width / max(1.0, float(box_height))
            perimeter = cv2.arcLength(contour, True)
            circularity = (
                4.0 * math.pi * area / (perimeter * perimeter)
                if perimeter > 0.0 else 0.0)
            if not 0.45 <= aspect <= 1.65 or circularity < 0.42:
                continue
            candidates.append((
                area * circularity, state,
                (x + x_left, y, box_width, box_height)))
    if not candidates:
        return 'UNKNOWN', None
    best = max(candidates, key=lambda item: item[0])
    # [2026-09-19] 실차에서 초록불로 바뀌었는데 GREEN 이 한 번도 안 나와
    # 정지선에서 못 빠져나왔다. 후보를 색 구분 없이 area*circularity 최대
    # 하나로 고르기 때문에, 배경의 붉은 물체가 켜진 초록 램프보다 크고
    # 둥글면 초록이 통째로 가려진다.
    #
    # 신호등은 한 번에 한 램프만 켜진다. 적색과 녹색 후보가 동시에
    # 잡혔다면 둘 중 하나는 반드시 배경이다. 그리고 두 오판의 대가가
    # 다르다. 초록을 놓치면 정지 중이라 WP 가 안 늘어 해제 창도 못 쓰고
    # 제한시간까지 차가 묶인다. 반대로 잘못 출발하는 것은 제한시간
    # 강제출발과 같은 결과다. 그래서 초록 후보가 최고점의
    # green_preference 배 이상이면 초록을 택한다.
    #
    # [방향 주의] green_preference 는 문턱값이다. 숫자를 올리면 초록
    # 우대가 '약해진다'. 0.25 는 붉은 물체가 램프 점수의 4 배까지 커도
    # 초록이 이긴다는 뜻이고, 0.5 는 2 배까지만 이긴다는 뜻이다.
    # 0 이면 우대 없음(예전처럼 최고점 하나).
    if green_preference > 0.0:
        green = [item for item in candidates if item[1] == 'GREEN']
        if green:
            best_green = max(green, key=lambda item: item[0])
            if best_green[0] >= best[0] * green_preference:
                best = best_green
    _, state, box = best
    return state, box
