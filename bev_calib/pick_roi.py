#!/usr/bin/env python3
"""정적장애물 카메라의 ROI 세로 범위를 사진에서 직접 고른다.

ROI 비율을 눈대중으로 정하면 안 되는 이유가 있다. 화면 한 행이 지면에서
몇 m 인지는 카메라 장착각에 따라 완전히 달라진다. 실제로 옛 값 0.35 는
지면 22 m 를, 그 조금 위는 지평선 너머(거리가 음수)를 보고 있었다.
select_obstacle 은 look_distance_m 너머를 어차피 버리므로, 그 위를 보는
것은 배경만 들이는 셈이다. 그 배경이 색에 걸리면 근거리 덩어리와 하나로
이어져 덩어리 아랫변이 엉뚱한 곳을 가리킨다.

그래서 BEV 호모그래피로 각 행의 실제 거리를 구해서 고른다.

  1) 거리 눈금이 그려진 사진 보기 (권장)
       python3 bev_calib/pick_roi.py bev_calib/front.png

  2) 원하는 거리로 바로 계산
       python3 bev_calib/pick_roi.py bev_calib/front.png --distance 4.0

  3) 창에서 클릭해서 고르기
       python3 bev_calib/pick_roi.py bev_calib/front.png --click

나온 roi_y_min 을 obstacle_avoidance_node.py 기본값에 넣거나, 실행 중이면
  ros2 param set /obstacle_avoidance_node roi_y_min 0.55
"""
import argparse
import inspect
import re
import sys

sys.path = [p for p in sys.path
            if p != '/usr/local/lib/python3.12/dist-packages']

import cv2
import numpy as np


def node_defaults():
    """노드 기본값을 그대로 읽는다. 런치가 이 노드에 파라미터를 안 주므로
    기본값이 곧 실차에서 도는 값이다."""
    from t870_control.obstacle_avoidance_node import (
        ObstacleAvoidance, homography_from_points)
    source = inspect.getsource(ObstacleAvoidance.__init__)

    def points(key):
        body = re.search(r"'%s':\s*\[(.*?)\]" % key, source, re.S).group(1)
        body = re.sub(r'#[^\n]*', '', body)
        return [float(x) for x in body.replace('\n', ' ').split(',')
                if x.strip()]

    def number(key):
        return float(re.search(r"'%s':\s*([0-9.]+)" % key, source).group(1))

    homography = homography_from_points(
        points('bev_image_points'), points('bev_ground_points'))
    return homography, number


def forward_at(homography, x, y):
    """화면 점 -> 지면 전방거리 m. 지평선 위면 음수가 나온다."""
    point = np.array([[[float(x), float(y)]]], dtype=np.float32)
    return float(cv2.perspectiveTransform(point, homography)[0, 0][0])


def row_for_distance(homography, width, height, target):
    """그 거리가 보이는 화면 행을 찾는다. 거리는 y 가 커질수록 줄어든다."""
    best = None
    for y in range(height):
        distance = forward_at(homography, width / 2.0, y)
        if distance <= 0.0:
            continue                      # 지평선 위
        error = abs(distance - target)
        if best is None or error < best[0]:
            best = (error, y, distance)
    return best


def annotate(image, homography, marks, roi_y_min, roi_y_max):
    height, width = image.shape[:2]
    out = image.copy()
    for distance in marks:
        found = row_for_distance(homography, width, height, distance)
        if found is None or found[0] > distance * 0.15:
            continue
        y = found[1]
        cv2.line(out, (0, y), (width, y), (255, 255, 0), 1)
        cv2.putText(out, '%.1f m' % distance, (width - 78, y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    for ratio, colour, text in (
            (roi_y_min, (0, 255, 0), 'ROI top'),
            (roi_y_max, (0, 165, 255), 'ROI bottom')):
        y = min(height - 1, int(height * ratio))
        cv2.line(out, (0, y), (width, y), colour, 2)
        distance = forward_at(homography, width / 2.0, y)
        cv2.putText(out, '%s %.2f  (%.2f m)' % (text, ratio, distance),
                    (10, max(18, y - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, colour, 2)
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('image', nargs='?', default='bev_calib/front.png')
    parser.add_argument('--distance', type=float,
                        help='이 거리가 ROI 상단이 되게 하는 비율을 구한다')
    parser.add_argument('--click', action='store_true',
                        help='창에서 클릭한 행을 ROI 상단으로 삼는다')
    parser.add_argument('--out', default='bev_calib/roi.png')
    args = parser.parse_args()

    image = cv2.imread(args.image)
    if image is None:
        print('사진을 읽을 수 없다: %s' % args.image)
        return 1
    height, width = image.shape[:2]
    homography, number = node_defaults()
    look = number('look_distance_m')
    near = number('minimum_distance_m')
    roi_y_min, roi_y_max = number('roi_y_min'), number('roi_y_max')

    print('사진 %s  %dx%d' % (args.image, width, height))
    print('현재 설정  roi_y_min %.2f -> %+.2f m,  roi_y_max %.2f -> %+.2f m'
          % (roi_y_min, forward_at(homography, width / 2.0, height * roi_y_min),
             roi_y_max,
             forward_at(homography, width / 2.0, min(height - 1,
                                                     height * roi_y_max))))
    print('look_distance_m %.1f, minimum_distance_m %.1f' % (look, near))

    if args.distance is not None:
        found = row_for_distance(homography, width, height, args.distance)
        if found is None:
            print('%.1f m 에 해당하는 행이 없다' % args.distance)
            return 1
        _, y, actual = found
        print()
        print('%.1f m -> 화면 y %d -> roi_y_min %.3f (실제 %.2f m)'
              % (args.distance, y, y / float(height), actual))
        print('  ros2 param set /obstacle_avoidance_node roi_y_min %.3f'
              % (y / float(height)))
        return 0

    print()
    print('  비율   y     전방거리')
    for ratio in [r / 100.0 for r in range(25, 101, 5)]:
        y = min(height - 1, int(height * ratio))
        distance = forward_at(homography, width / 2.0, y)
        note = ''
        if distance <= 0.0:
            note = '  <- 지평선 너머. 거리가 뒤집힌다'
        elif distance > look:
            note = '  <- look_distance 밖. 후보가 못 된다'
        elif distance < near:
            note = '  <- minimum_distance 안. 버려진다'
        print('  %.2f  %3d   %+8.2f m%s' % (ratio, y, distance, note))

    marked = annotate(image, homography, (1.0, 2.0, 3.0, 4.0, 5.0),
                      roi_y_min, roi_y_max)
    cv2.imwrite(args.out, marked)
    print()
    print('눈금 그린 사진 저장: %s' % args.out)

    if args.click:
        chosen = {}

        def on_mouse(event, _x, y, _flags, _param):
            if event == cv2.EVENT_LBUTTONDOWN:
                chosen['y'] = y
                print('y %d -> roi_y_min %.3f (%.2f m)'
                      % (y, y / float(height),
                         forward_at(homography, width / 2.0, y)))

        cv2.namedWindow('pick roi top')
        cv2.setMouseCallback('pick roi top', on_mouse)
        print('ROI 상단으로 삼을 행을 클릭. q 로 종료.')
        while True:
            view = marked.copy()
            if 'y' in chosen:
                cv2.line(view, (0, chosen['y']), (width, chosen['y']),
                         (0, 0, 255), 2)
            cv2.imshow('pick roi top', view)
            if cv2.waitKey(30) & 0xff in (ord('q'), 27):
                break
        cv2.destroyAllWindows()
        if 'y' in chosen:
            print()
            print('  ros2 param set /obstacle_avoidance_node roi_y_min %.3f'
                  % (chosen['y'] / float(height)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
