#!/usr/bin/env python3
"""마지막 WP의 좌우 표지(화살표 / X) 색 배치로 직선통과 여부를 판단한다.

표지는 빨강 하나와 초록 하나다. 화면에서 둘의 좌우 순서만 본다.

  왼쪽부터 빨강, 초록  ->  OX_right.csv 로 전환   (/competition/ox_straight=True)
  왼쪽부터 초록, 빨강  ->  가던 P 경로 그대로     (/competition/ox_straight=False)

[중요] 여기는 신호등이 아니다. 표지가 원형 램프가 아니라 화살표와 X 다.
lane_camera_preview.detect_traffic_light 은 원형도(circularity >= 0.42)와
종횡비(0.45~1.65)로 '동그란 램프' 만 남기므로 여기에 쓸 수 없다. 화살표는
길쭉해서 종횡비에 걸리고, X 는 획이 갈라져 원형도가 낮다.

그래서 모양 필터 대신 아래 세 가지로 거른다.
  1) 면적 하한/상한
  2) 두 표지의 세로 위치가 비슷할 것 (나란히 붙어 있다)
  3) 좌우가 충분히 떨어져 있을 것 (순서를 가를 수 있다)

색을 못 찾거나 확신이 없으면 False 를 유지한다. 그러면 미션 매니저가
경로를 바꾸지 않고 기존 경로를 계속 간다. 안전한 기본값이다.

카메라는 lane_camera_preview 가 장치를 독점하므로 그 노드가 내보내는
/camera/image_raw 를 받는다(publish_camera_image=true 필요).
"""
import sys

# /usr/local 에 깨진 opencv 가 있어 libopencv_hdf 를 못 찾는다.
sys.path = [p for p in sys.path
            if p != '/usr/local/lib/python3.12/dist-packages']

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String, UInt32


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


def color_masks(roi, merge_kernel_px=15, open_kernel_px=3):
    """빨강/초록 마스크. 모양은 보지 않는다.

    표지는 LED 다. 카메라 노출 주파수와 맞지 않아 획이 가로 줄무늬로
    끊겨 보인다(롤링 셔터 밴딩). 그래서 두 가지를 한다.

      1) OPEN 커널은 작게 둔다. 크면 얇게 남은 획을 지워 버린다.
      2) CLOSE 커널을 크게 잡아 끊긴 조각을 한 덩어리로 다시 묶는다.
         이 값이 조각 사이 간격보다 커야 한다. 다만 너무 키우면 근처의
         다른 광원(신호등)까지 붙어 버리므로 무한정 키우면 안 된다.

    HSV 만으로는 조명에 따라 흔들리므로 채널 우세도 함께 본다. LED 는
    포화되어 흰색에 가깝게 찍히는 경우가 있어 채도 하한을 낮게 잡았다.
    """
    hsv = cv2.cvtColor(cv2.GaussianBlur(roi, (5, 5), 0), cv2.COLOR_BGR2HSV)
    blue, green, red = [channel.astype(np.int16) for channel in cv2.split(roi)]

    red_mask = cv2.inRange(hsv, (0, 70, 60), (13, 255, 255))
    red_mask |= cv2.inRange(hsv, (167, 70, 60), (180, 255, 255))
    red_mask &= ((red - green >= 25) & (red - blue >= 18)).astype(np.uint8) * 255

    green_mask = cv2.inRange(hsv, (38, 55, 60), (95, 255, 255))
    green_mask &= ((green - red >= 20)
                   & (green - blue >= 8)).astype(np.uint8) * 255

    open_size = max(1, int(open_kernel_px))
    merge_size = max(1, int(merge_kernel_px))
    open_kernel = np.ones((open_size, open_size), np.uint8)
    close_kernel = np.ones((merge_size, merge_size), np.uint8)
    out = []
    for mask in (red_mask, green_mask):
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
        out.append(mask)
    return out[0], out[1]


def largest_mark(mask, min_area, max_area):
    """가장 큰 덩어리를 (면적, 중심x, 중심y, 박스) 로 돌려준다.

    모양은 따지지 않는다. 화살표든 X 든 상관없이 그 색의 가장 큰 표지를
    고르면 된다. 좌우 순서만 필요하기 때문이다.
    """
    count, _, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    best = None
    for index in range(1, count):
        area = int(stats[index, cv2.CC_STAT_AREA])
        if area < min_area or area > max_area:
            continue
        if best is not None and area <= best[0]:
            continue
        box = (int(stats[index, cv2.CC_STAT_LEFT]),
               int(stats[index, cv2.CC_STAT_TOP]),
               int(stats[index, cv2.CC_STAT_WIDTH]),
               int(stats[index, cv2.CC_STAT_HEIGHT]))
        best = (area, float(centroids[index][0]),
                float(centroids[index][1]), box)
    return best


def decide_straight(red, green, minimum_separation_px, maximum_rise_px):
    """좌우 순서로 직선통과 여부를 정한다.

    red/green 은 largest_mark 결과 또는 None. 화면 x 는 왼쪽이 0 이다.

    반환값은 (판단했는가, 직선통과인가, 사유) 다.
      왼쪽부터 빨강, 초록 -> True  (OX_right 로 전환)
      왼쪽부터 초록, 빨강 -> False (가던 경로 그대로)
    """
    if red is None and green is None:
        return False, False, 'no marks'
    if red is None:
        return False, False, 'no red'
    if green is None:
        return False, False, 'no green'
    if abs(red[1] - green[1]) < minimum_separation_px:
        # 좌우로 겹쳐 보인다. 순서를 가를 수 없다.
        return False, False, 'overlapping'
    if abs(red[2] - green[2]) > maximum_rise_px:
        # 두 표지는 나란히 붙어 있다. 세로로 크게 어긋나면 둘 중 하나는
        # 표지가 아니라 다른 물체다.
        return False, False, 'not side by side'
    return True, red[1] < green[1], 'red left' if red[1] < green[1] else 'green left'


class OxSignalDetector(Node):
    def __init__(self):
        super().__init__('ox_signal_detector')
        defaults = {
            'image_topic': '/camera/image_raw',
            # 이 구간 안에서만 본다. 코스 앞부분의 빨강/초록에 반응하지
            # 않게 막는다. 미션 매니저의 분기 실행은 WP693 이고 경로가
            # 실제로 갈라지는 곳은 WP697 이다.
            'observation_start_waypoint': 680,
            'decision_waypoint': 692,
            # 구간 전체에서 많이 나온 쪽으로 정한다. 표가 이 수보다 적으면
            # 아직 판단하지 않는다. LED 밴딩으로 몇 프레임은 비어 있다.
            'minimum_votes': 5,
            # 표지는 램프보다 크다. ROI 면적 대비 비율이다.
            'min_area_ratio': 0.0006,
            'max_area_ratio': 0.15,
            # LED 밴딩으로 끊긴 획을 다시 묶는 CLOSE 커널 크기(픽셀).
            # 조각 사이 간격보다 커야 한다. 실차에서 /vision/ox_debug 를
            # 보며 맞춘다. 너무 키우면 신호등 불빛까지 붙는다.
            'merge_kernel_px': 15,
            'open_kernel_px': 3,
            # 두 표지 중심의 좌우 최소 간격 (ROI 폭 대비)
            'minimum_separation_ratio': 0.05,
            # 두 표지 중심의 세로 최대 어긋남 (ROI 높이 대비)
            'maximum_rise_ratio': 0.25,
            'roi_x_min': 0.0,
            'roi_x_max': 1.0,
            'roi_y_min': 0.0,
            'roi_y_max': 0.75,
            'publish_debug_image': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.start_wp = int(self.get_parameter(
            'observation_start_waypoint').value)
        self.decision_wp = int(self.get_parameter('decision_waypoint').value)
        if self.start_wp > self.decision_wp:
            raise ValueError(
                'observation_start_waypoint must be <= decision_waypoint')
        self.minimum_votes = int(self.get_parameter('minimum_votes').value)
        self.debug_enabled = bool(
            self.get_parameter('publish_debug_image').value)

        self.active = False
        self.locked = False
        self.straight = False
        self.reason = 'idle'
        self.red_left_votes = 0
        self.green_left_votes = 0

        self.choice_pub = self.create_publisher(
            Bool, '/competition/ox_straight', 10)
        self.state_pub = self.create_publisher(
            String, '/vision/ox_signal_state', 10)
        self.debug_pub = self.create_publisher(Image, '/vision/ox_debug', 2)
        self.create_subscription(
            Image, str(self.get_parameter('image_topic').value),
            self.on_image, 5)
        self.create_subscription(
            UInt32, '/gps/current_waypoint', self.on_waypoint, 10)
        self.create_timer(0.1, self.publish_choice)

        self.get_logger().warning(
            'OX mark detector ready (arrow/X, not round lamps): watch '
            'WP%d~%d; left RED,GREEN -> OX_right, left GREEN,RED -> keep path'
            % (self.start_wp, self.decision_wp))

    def p(self, name):
        return self.get_parameter(name).value

    def on_waypoint(self, msg):
        if self.locked:
            return
        waypoint = int(msg.data)
        if self.active and waypoint > self.decision_wp:
            self.locked = True
            self.active = False
            self.get_logger().warning(
                'OX DECISION LOCKED at waypoint %d: ox_straight=%s '
                '(red-left %d / green-left %d, last=%s)'
                % (waypoint, self.straight, self.red_left_votes,
                   self.green_left_votes, self.reason))
            return
        if not self.active and self.start_wp <= waypoint <= self.decision_wp:
            self.active = True
            self.get_logger().warning(
                'OX mark observation started at waypoint %d' % waypoint)

    def find_marks(self, frame):
        height, width = frame.shape[:2]
        x0 = int(width * float(self.p('roi_x_min')))
        x1 = int(width * float(self.p('roi_x_max')))
        y0 = int(height * float(self.p('roi_y_min')))
        y1 = int(height * float(self.p('roi_y_max')))
        roi = frame[y0:y1, x0:x1]
        roi_area = max(1, roi.shape[0] * roi.shape[1])
        min_area = max(30, int(roi_area * float(self.p('min_area_ratio'))))
        max_area = int(roi_area * float(self.p('max_area_ratio')))

        red_mask, green_mask = color_masks(
            roi,
            merge_kernel_px=int(self.p('merge_kernel_px')),
            open_kernel_px=int(self.p('open_kernel_px')))
        red = largest_mark(red_mask, min_area, max_area)
        green = largest_mark(green_mask, min_area, max_area)

        debug = frame.copy()
        cv2.rectangle(debug, (x0, y0), (x1, y1), (255, 180, 0), 2)
        for mark, color in ((red, (0, 0, 255)), (green, (0, 200, 0))):
            if mark is None:
                continue
            bx, by, bw, bh = mark[3]
            cv2.rectangle(debug, (x0 + bx, y0 + by),
                          (x0 + bx + bw, y0 + by + bh), color, 3)
            cv2.line(debug, (int(x0 + mark[1]), y0),
                     (int(x0 + mark[1]), y1), color, 1)
        return red, green, debug, roi.shape[1], roi.shape[0]

    def on_image(self, msg):
        if self.locked or not self.active:
            return
        try:
            frame = image_to_bgr(msg)
            red, green, debug, roi_w, roi_h = self.find_marks(frame)
        except (ValueError, cv2.error) as error:
            self.get_logger().warning(f'camera frame decode failed: {error}')
            return

        decided, straight, reason = decide_straight(
            red, green,
            roi_w * float(self.p('minimum_separation_ratio')),
            roi_h * float(self.p('maximum_rise_ratio')))
        self.reason = reason
        if decided:
            if straight:
                self.red_left_votes += 1
            else:
                self.green_left_votes += 1
            # 구간 전체 누적 최빈값. 슬라이딩 윈도우가 아니라 누적이라
            # 뒤쪽 몇 프레임이 튀어도 결과가 뒤집히지 않는다.
            total = self.red_left_votes + self.green_left_votes
            if total >= self.minimum_votes:
                self.straight = self.red_left_votes > self.green_left_votes

        self.state_pub.publish(String(data=(
            'RED_LEFT' if decided and straight
            else 'GREEN_LEFT' if decided else f'SEARCH:{reason}')))
        if self.debug_enabled:
            out = Image()
            out.header = msg.header
            out.height, out.width = debug.shape[:2]
            out.encoding = 'bgr8'
            out.is_bigendian = 0
            out.step = out.width * 3
            out.data = debug.tobytes()
            self.debug_pub.publish(out)

    def publish_choice(self):
        self.choice_pub.publish(Bool(data=self.straight))


def main(args=None):
    rclpy.init(args=args)
    node = OxSignalDetector()
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
