#!/usr/bin/env python3
"""BEV 캘리브레이션 사진에서 표식의 픽셀 좌표를 뽑는다.

두 가지 방법이 있다.

  1) 색 자동 검출 (권장)
     표식을 장애물과 다른 색(파랑/초록 등)으로 두고 촬영하면 자동으로
     중심을 찾는다. 눈대중보다 정확하다.

       python3 bev_calib/pick_points.py front.png --colour blue

  2) 클릭
     자동이 안 되면 창에서 표식을 클릭한다. 찍은 순서대로 출력된다.

       python3 bev_calib/pick_points.py front.png --click

출력된 픽셀 좌표를 실측 지면 좌표와 짝지어 launch 에 넣는다.
  bev_image_points  : 여기서 나온 값
  bev_ground_points : 자로 잰 값 [전방m, 좌우m, ...] 왼쪽이 +
"""
import argparse
import sys

sys.path = [p for p in sys.path
            if p != '/usr/local/lib/python3.12/dist-packages']

import cv2
import numpy as np


COLOURS = {
    # 이름: (HSV 하한, HSV 상한, 두 번째 범위 또는 None)
    'blue':  ((95, 110, 60), (130, 255, 255), None),
    'green': ((40, 80, 60), (85, 255, 255), None),
    'pink':  ((160, 90, 90), (175, 255, 255), None),
    'red':   ((0, 120, 80), (10, 255, 255), ((170, 120, 80), (180, 255, 255))),
}


def detect(image, name, min_area):
    low, high, second = COLOURS[name]
    hsv = cv2.cvtColor(cv2.GaussianBlur(image, (5, 5), 0), cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, low, high)
    if second is not None:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, second[0], second[1]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    count, _, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    found = []
    for index in range(1, count):
        area = int(stats[index, cv2.CC_STAT_AREA])
        if area < min_area:
            continue
        # 표식이 지면에 있으므로 아랫변이 접지점이다.
        bottom = (int(stats[index, cv2.CC_STAT_TOP])
                  + int(stats[index, cv2.CC_STAT_HEIGHT]))
        found.append((area, float(centroids[index][0]), float(bottom)))
    # 먼 것(화면 위)부터, 같은 줄에서는 왼쪽부터 정렬한다.
    found.sort(key=lambda item: (round(item[2] / 20.0), item[1]))
    return found, mask


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('image')
    parser.add_argument('--colour', choices=sorted(COLOURS))
    parser.add_argument('--click', action='store_true')
    parser.add_argument('--min-area', type=int, default=60)
    parser.add_argument('--save', default='')
    args = parser.parse_args()

    image = cv2.imread(args.image)
    if image is None:
        raise SystemExit(f'이미지를 열 수 없다: {args.image}')
    print(f'해상도 {image.shape[1]}x{image.shape[0]}')

    points = []
    if args.click:
        def on_mouse(event, x, y, _flags, _param):
            if event == cv2.EVENT_LBUTTONDOWN:
                points.append((float(x), float(y)))
                print(f'  {len(points)}: {x}, {y}')
        cv2.namedWindow('pick')
        cv2.setMouseCallback('pick', on_mouse)
        print('표식을 순서대로 클릭하고 q 로 끝낸다.')
        while True:
            preview = image.copy()
            for index, (x, y) in enumerate(points, start=1):
                cv2.circle(preview, (int(x), int(y)), 6, (0, 255, 255), -1)
                cv2.putText(preview, str(index), (int(x) + 8, int(y)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow('pick', preview)
            if cv2.waitKey(20) & 0xff in (ord('q'), 27):
                break
        cv2.destroyAllWindows()
    else:
        if not args.colour:
            raise SystemExit('--colour 또는 --click 중 하나가 필요하다')
        found, mask = detect(image, args.colour, args.min_area)
        print(f'{args.colour} 표식 {len(found)}개 (먼 것부터, 왼쪽부터)')
        preview = image.copy()
        for index, (area, cx, bottom) in enumerate(found, start=1):
            points.append((cx, bottom))
            print(f'  {index}: {cx:.1f}, {bottom:.1f}   (면적 {area})')
            cv2.circle(preview, (int(cx), int(bottom)), 6, (0, 255, 255), -1)
            cv2.putText(preview, str(index), (int(cx) + 8, int(bottom)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        if args.save:
            cv2.imwrite(args.save, preview)
            cv2.imwrite(args.save.replace('.', '_mask.'), mask)
            print(f'확인용 이미지 저장: {args.save}')

    if points:
        flat = ', '.join('%.1f' % value for point in points for value in point)
        print()
        print("launch 에 넣을 값:")
        print(f"  'bev_image_points': [{flat}],")
        print("  'bev_ground_points': [전방m, 좌우m, ...],   # 같은 순서로")


if __name__ == '__main__':
    main()
