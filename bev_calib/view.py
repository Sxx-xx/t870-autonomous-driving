#!/usr/bin/env python3
"""카메라를 맞출 때 쓰는 실시간 뷰어.

/vision/avoid_debug 는 미션 구간(WP201~227) 안에서만 발행되므로 카메라를
정렬할 때는 안 나온다. 이 도구는 원본 영상에 같은 안내선을 직접 그린다.

  ROI 상단선   여기부터 아래만 본다
  사다리꼴     지면 통로(roi_corridor_half_m) 를 화면으로 되돌린 것
  거리 눈금    1/2/3/4/5 m
  통로선       desired_clearance_m 좌우

  python3 bev_calib/view.py                       # 정적장애물 카메라
  python3 bev_calib/view.py --topic /camera/laptop/image_raw
  python3 bev_calib/view.py --save shot.png       # q 누르면 저장

launch 가 떠 있어야 한다(카메라 노드가 장치를 잡고 있으므로 토픽으로 받는다).
"""
import argparse
import inspect
import re
import sys

sys.path = [p for p in sys.path
            if p != '/usr/local/lib/python3.12/dist-packages']

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def settings():
    from t870_control.obstacle_avoidance_node import (
        ObstacleAvoidance, homography_from_points)
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


def overlay(frame, number, inverse):
    from t870_control.obstacle_avoidance_node import corridor_polygon
    height, width = frame.shape[:2]
    out = frame.copy()

    def to_pixel(distance, offset):
        point = np.array([[[float(distance), float(offset)]]],
                         dtype=np.float32)
        mapped = cv2.perspectiveTransform(point, inverse)[0, 0]
        return int(mapped[0]), int(mapped[1])

    near = number('minimum_distance_m')
    far = number('look_distance_m')
    clearance = number('desired_clearance_m')

    # 거리 눈금
    distance = 1.0
    while distance <= far + 1.0:
        try:
            _, y = to_pixel(distance, 0.0)
        except cv2.error:
            break
        if 0 <= y < height:
            cv2.line(out, (0, y), (width, y), (255, 255, 0), 1)
            cv2.putText(out, '%.0fm' % distance, (width - 58, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        distance += 1.0

    # 충돌 통로 (desired_clearance)
    for offset in (-clearance, clearance):
        cv2.line(out, to_pixel(near, offset), to_pixel(far, offset),
                 (0, 200, 255), 2)

    # 사다리꼴 ROI
    half = number('roi_corridor_half_m')
    if half > 0.0:
        corners = corridor_polygon(inverse, near, far * 1.2, half)
        cv2.polylines(out, [np.array(corners, dtype=np.int32)], True,
                      (255, 180, 0), 2)

    # ROI 상단
    y_top = int(height * number('roi_y_min'))
    cv2.line(out, (0, y_top), (width, y_top), (0, 255, 0), 2)
    cv2.putText(out, 'ROI top %.2f (%.1f m)' % (number('roi_y_min'), far),
                (10, max(18, y_top - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (0, 255, 0), 2)
    cv2.putText(out, 'orange mask = magenta', (10, height - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return out


def paint_mask(frame, number, source):
    """주황 마스크를 자홍으로 칠한다. 색 설정이 맞는지 바로 보인다."""
    low = re.search(r"'hsv_low':\s*\[([^\]]*)\]", source).group(1)
    high = re.search(r"'hsv_high':\s*\[([^\]]*)\]", source).group(1)
    low = np.array([int(v) for v in low.split(',')])
    high = np.array([int(v) for v in high.split(',')])
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, low, high)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    frame[mask > 0] = (255, 0, 255)
    return frame


class Viewer(Node):
    def __init__(self, topic, save):
        super().__init__('bev_viewer')
        self.number, self.inverse = settings()
        from t870_control.obstacle_avoidance_node import ObstacleAvoidance
        self.source = inspect.getsource(ObstacleAvoidance.__init__)
        self.save = save
        self.frame = None
        self.create_subscription(Image, topic, self.on_image, 5)
        print('구독: %s   (창에서 q 를 누르면 종료)' % topic)

    def on_image(self, msg):
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        step = msg.step if msg.step else msg.width * 3
        image = raw.reshape((msg.height, step))[:, :msg.width * 3]
        self.frame = image.reshape((msg.height, msg.width, 3)).copy()

    def show(self):
        if self.frame is None:
            return True
        view = paint_mask(self.frame.copy(), self.number, self.source)
        view = overlay(view, self.number, self.inverse)
        cv2.imshow('camera aim (q to quit)', view)
        key = cv2.waitKey(1) & 0xff
        if key in (ord('q'), 27):
            if self.save:
                cv2.imwrite(self.save, self.frame)
                print('저장: %s' % self.save)
            return False
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/camera/front/image_raw')
    parser.add_argument('--save', default='')
    args = parser.parse_args()
    rclpy.init()
    node = Viewer(args.topic, args.save)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if not node.show():
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
