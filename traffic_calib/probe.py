#!/usr/bin/env python3
"""신호등 초록불이 왜 안 잡히는지 어느 필터가 죽이는지 찍어준다.

실차에서 초록불인데 GREEN 이 한 번도 안 나와 정지선에서 못 빠져나왔다.
detect_traffic_light 은 통과한 것만 돌려주므로 탈락 이유를 알 수 없다.
여기서는 같은 순서로 돌리되 각 단계에서 몇 개가 죽었는지 센다.

    ros2 run 이 아니라 그냥 실행한다(통합 launch 를 띄운 채로):
        /usr/bin/python3 traffic_calib/probe.py

    신호등이 초록일 때 화면에 찍히는 줄을 보면 된다. 예:
        GREEN 후보 0 | 색통과 1 -> 면적상한 1  ... 램프가 너무 커서 탈락
        GREEN 후보 0 | 색통과 0                ... HSV 에서 아예 안 잡힘
        GREEN 후보 0 | ROI밖 1                 ... 램프가 화면 아래쪽
"""
import sys

sys.path = [p for p in sys.path
            if p != '/usr/local/lib/python3.12/dist-packages']

import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# traffic_lamp 과 같은 값이어야 의미가 있다. 바꾸면 여기도 바꿀 것.
Y_LIMIT_RATIO = 0.8
AREA_CEIL_RATIO = 0.05   # 0 이면 상한 없음
ASPECT_LO, ASPECT_HI = 0.45, 1.65
CIRCULARITY_MIN = 0.42
MASKS = {
    'RED': [((0, 120, 100), (12, 255, 255)), ((168, 120, 100), (180, 255, 255))],
    'YELLOW': [((14, 120, 110), (38, 255, 255))],
    'GREEN': [((38, 55, 60), (100, 255, 255))],
}


def image_to_bgr(msg):
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    step = msg.step if msg.step else msg.width * 3
    image = raw.reshape((msg.height, step))[:, :msg.width * 3]
    image = image.reshape((msg.height, msg.width, 3))
    if msg.encoding in ('rgb8', 'RGB8'):
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image.copy()


def probe(frame):
    """색깔별로 어느 단계에서 몇 개가 죽었는지 센다."""
    height, width = frame.shape[:2]
    frame_area = width * height
    y_limit = int(height * Y_LIMIT_RATIO)
    x_left, x_right = int(width * 0.05), int(width * 0.95)
    full_hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0),
                            cv2.COLOR_BGR2HSV)
    roi_hsv = full_hsv[:y_limit, x_left:x_right]
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    report = {}
    for state, ranges in MASKS.items():
        # ROI 밖에 있는지 보려고 전체 화면에서도 한 번 센다.
        counts = {'색통과': 0, 'ROI밖': 0, '면적하한': 0, '면적상한': 0,
                  '종횡비': 0, '원형도': 0, '후보': 0}
        best = None
        for hsv, in_roi in ((roi_hsv, True), (full_hsv[y_limit:], False)):
            mask = None
            for lo, hi in ranges:
                part = cv2.inRange(hsv, lo, hi)
                mask = part if mask is None else cv2.bitwise_or(mask, part)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < 8.0:
                    continue
                if not in_roi:
                    counts['ROI밖'] += 1
                    continue
                counts['색통과'] += 1
                if area < max(8.0, frame_area * 0.000025):
                    counts['면적하한'] += 1
                    continue
                if (AREA_CEIL_RATIO > 0.0
                        and area > frame_area * AREA_CEIL_RATIO):
                    counts['면적상한'] += 1
                    continue
                _, _, box_w, box_h = cv2.boundingRect(contour)
                aspect = box_w / max(1.0, float(box_h))
                perimeter = cv2.arcLength(contour, True)
                circularity = (4.0 * math.pi * area / (perimeter * perimeter)
                               if perimeter > 0.0 else 0.0)
                if not ASPECT_LO <= aspect <= ASPECT_HI:
                    counts['종횡비'] += 1
                    continue
                if circularity < CIRCULARITY_MIN:
                    counts['원형도'] += 1
                    continue
                counts['후보'] += 1
                score = area * circularity
                if best is None or score > best[0]:
                    best = (score, area, circularity, aspect)
        report[state] = (counts, best)
    return report


class Probe(Node):
    def __init__(self, topic):
        super().__init__('traffic_probe')
        self.create_subscription(Image, topic, self.on_image, 5)
        self.timer_frames = 0
        self.latest = None
        self.create_timer(1.0, self.report)
        print('구독: %s   (초록불을 비추고 GREEN 줄을 보세요)' % topic)

    def on_image(self, msg):
        self.latest = probe(image_to_bgr(msg))
        self.timer_frames += 1

    def report(self):
        if self.latest is None:
            print('프레임 없음. 카메라 노드가 떠 있는지 확인.')
            return
        print('--- %d fps ---' % self.timer_frames)
        self.timer_frames = 0
        for state in ('RED', 'YELLOW', 'GREEN'):
            counts, best = self.latest[state]
            killed = ' '.join('%s %d' % (k, v) for k, v in counts.items()
                              if v and k not in ('후보', '색통과'))
            line = '%-6s 후보 %d | 색통과 %d' % (
                state, counts['후보'], counts['색통과'])
            if killed:
                line += ' -> ' + killed
            if best:
                line += '  | 최고 area=%.0f circ=%.2f 종횡=%.2f score=%.0f' % (
                    best[1], best[2], best[3], best[0])
            print(line)


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/camera/laptop/image_raw'
    rclpy.init()
    node = Probe(topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
