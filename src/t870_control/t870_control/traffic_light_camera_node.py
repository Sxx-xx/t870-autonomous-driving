#!/usr/bin/env python3
"""노트북 카메라로 신호등 색을 판정한다.

대회에서는 카메라를 두 대 쓴다.
  C920(외장)  -> 정적장애물 판단
  노트북 내장  -> 신호등, X/화살표

그전에는 lane_camera_preview 가 C920 을 독점하면서 차선과 신호등을 같이
처리했다. C920 이 정적장애물용으로 넘어가면서 신호등만 따로 떼어냈다.

판정 자체는 lane_camera_preview 가 쓰던 것과 같다(traffic_lamp). 원형
램프 전용이며 화살표/X 에는 쓸 수 없다.

미션 매니저는 /vision/traffic_light_state 를 구독하고 판단 구간에서
최빈값으로 정한다. 여기서는 프레임마다 본 것을 그대로 내보내면 된다.
다만 한 프레임 튄 값이 그대로 나가지 않게 confirm_frames 만큼 연속으로
같은 색이 나와야 상태를 바꾼다.
"""
from collections import deque
import sys

sys.path = [p for p in sys.path
            if p != '/usr/local/lib/python3.12/dist-packages']

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .traffic_lamp import detect_traffic_light


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


def confirmed_state(history, confirm_frames, previous):
    """연속으로 같은 색이 confirm_frames 만큼 나와야 상태를 바꾼다.

    한 프레임 튄 값이 그대로 나가면 미션 매니저의 표를 오염시킨다.
    """
    if len(history) < confirm_frames:
        return previous
    recent = list(history)[-confirm_frames:]
    if all(state == recent[0] for state in recent):
        return recent[0]
    return previous


class TrafficLightCamera(Node):
    def __init__(self):
        super().__init__('traffic_light_camera')
        defaults = {
            'image_topic': '/camera/laptop/image_raw',
            'state_topic': '/vision/traffic_light_state',
            'confirm_frames': 3,
            'publish_debug_image': True,
            # 0.05 = 1.2 m 까지 램프를 받는다(면적 21642/d^2 px).
            # 옛 0.004 는 4.5 m 라 정지선에서 꺼졌다. 0 이면 상한 없음.
            'area_ceiling_ratio': 0.05,
            # 화면 아래 1/5 만 잘라낸다.
            'roi_height_ratio': 0.8,
            # [방향 주의] 문턱값이라 올리면 초록 우대가 약해진다.
            # 0.5 = 붉은 물체가 램프 점수의 2 배까지는 초록이 이김.
            'green_preference': 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.confirm_frames = max(
            1, int(self.get_parameter('confirm_frames').value))
        self.history = deque(maxlen=self.confirm_frames * 3)
        self.state = 'UNKNOWN'
        self.debug_enabled = bool(
            self.get_parameter('publish_debug_image').value)
        self.detect_kwargs = {
            'area_ceiling_ratio': float(
                self.get_parameter('area_ceiling_ratio').value),
            'roi_height_ratio': float(
                self.get_parameter('roi_height_ratio').value),
            'green_preference': float(
                self.get_parameter('green_preference').value),
        }

        self.state_pub = self.create_publisher(
            String, str(self.get_parameter('state_topic').value), 10)
        self.debug_pub = self.create_publisher(
            Image, '/vision/traffic_light_debug', 2)
        self.create_subscription(
            Image, str(self.get_parameter('image_topic').value),
            self.on_image, 5)
        self.create_timer(0.1, self.publish_state)

        self.get_logger().warning(
            'Traffic light camera ready: %s -> %s (confirm %d frames, '
            'area_ceiling %.3f, roi %.2f, green_pref %.2f)'
            % (str(self.get_parameter('image_topic').value),
               str(self.get_parameter('state_topic').value),
               self.confirm_frames,
               self.detect_kwargs['area_ceiling_ratio'],
               self.detect_kwargs['roi_height_ratio'],
               self.detect_kwargs['green_preference']))

    def on_image(self, msg):
        try:
            frame = image_to_bgr(msg)
            state, box = detect_traffic_light(
                frame, **self.detect_kwargs)
        except (ValueError, cv2.error) as error:
            self.get_logger().warning(f'camera frame decode failed: {error}')
            return

        self.history.append(state)
        previous = self.state
        self.state = confirmed_state(
            self.history, self.confirm_frames, self.state)
        if self.state != previous:
            self.get_logger().warning(
                'Traffic light: %s -> %s' % (previous, self.state))

        if self.debug_enabled:
            debug = frame.copy()
            if box is not None:
                x, y, w, h = box
                colour = {'RED': (0, 0, 255), 'YELLOW': (0, 200, 255),
                          'GREEN': (0, 200, 0)}.get(state, (200, 200, 200))
                cv2.rectangle(debug, (x, y), (x + w, y + h), colour, 3)
            cv2.putText(debug, self.state, (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2,
                        cv2.LINE_AA)
            out = Image()
            out.header = msg.header
            out.height, out.width = debug.shape[:2]
            out.encoding = 'bgr8'
            out.is_bigendian = 0
            out.step = out.width * 3
            out.data = debug.tobytes()
            self.debug_pub.publish(out)

    def publish_state(self):
        self.state_pub.publish(String(data=self.state))


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightCamera()
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
