#!/usr/bin/env python3
"""Red-only upper-center XO/finish marker detector.

This node only reports a confirmed red marker. It does not publish vehicle
velocity or stop commands, so the finish action can be connected separately.
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
from std_msgs.msg import Bool, String


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


def publish_bgr(publisher, source, frame):
    msg = Image()
    msg.header = source.header
    msg.height, msg.width = frame.shape[:2]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = frame.tobytes()
    publisher.publish(msg)


class FinishRedDetector(Node):
    def __init__(self):
        super().__init__('finish_red_detector')
        defaults = {
            'image_topic': '/camera/image_raw',
            'confirm_frames': 5,
            'window_frames': 8,
            'min_red_area_ratio': 0.00035,
            'roi_x_min': 0.20,
            'roi_x_max': 0.80,
            'roi_y_min': 0.00,
            'roi_y_max': 0.48,
            'publish_debug_image': True,
            'latch_detection': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        topic = str(self.get_parameter('image_topic').value)
        self.confirm_frames = int(self.get_parameter('confirm_frames').value)
        self.history = deque(maxlen=int(self.get_parameter('window_frames').value))
        self.min_area_ratio = float(self.get_parameter('min_red_area_ratio').value)
        self.latched = bool(self.get_parameter('latch_detection').value)
        self.detected = False
        self.debug_enabled = bool(self.get_parameter('publish_debug_image').value)

        self.detect_pub = self.create_publisher(Bool, '/vision/xo_detected', 10)
        self.state_pub = self.create_publisher(String, '/vision/xo_state', 10)
        self.debug_pub = self.create_publisher(Image, '/vision/xo_debug', 2)
        self.create_subscription(Image, topic, self.on_image, 5)
        self.get_logger().info(f'Listening to {topic}; upper-center red marker ready')

    def detect_red_marker(self, frame):
        height, width = frame.shape[:2]
        x0 = int(width * float(self.get_parameter('roi_x_min').value))
        x1 = int(width * float(self.get_parameter('roi_x_max').value))
        y0 = int(height * float(self.get_parameter('roi_y_min').value))
        y1 = int(height * float(self.get_parameter('roi_y_max').value))
        roi = frame[y0:y1, x0:x1]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        b, g, r = cv2.split(roi)
        red_hue = cv2.inRange(hsv, (0, 10, 45), (13, 255, 255))
        red_hue |= cv2.inRange(hsv, (167, 45, 45), (180, 255, 255))
        brightness_floor = max(45, min(120, int(np.percentile(v, 75) * 0.35)))
        dominance = ((r.astype(np.int16) - g.astype(np.int16) >= 28)
                     & (r.astype(np.int16) - b.astype(np.int16) >= 20)
                     & (s >= 55) & (v >= brightness_floor))
        mask = red_hue & (dominance.astype(np.uint8) * 255)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))

        min_area = max(20, int(roi.shape[0] * roi.shape[1] * self.min_area_ratio))
        count, _, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        best = None
        for index in range(1, count):
            area = int(stats[index, cv2.CC_STAT_AREA])
            if area >= min_area and (best is None or area > best[0]):
                best = (area, stats[index].copy())

        debug = frame.copy()
        cv2.rectangle(debug, (x0, y0), (x1, y1), (255, 180, 0), 2)
        if best is not None:
            _, stat = best
            bx, by, bw, bh = [int(value) for value in stat[:4]]
            cv2.rectangle(debug, (x0 + bx, y0 + by),
                          (x0 + bx + bw, y0 + by + bh), (0, 0, 255), 3)
        return best is not None, debug

    def on_image(self, msg):
        try:
            frame = image_to_bgr(msg)
            detected, debug = self.detect_red_marker(frame)
        except (ValueError, cv2.error) as error:
            self.get_logger().warning(f'camera frame decode failed: {error}')
            return

        self.history.append(detected)
        confirmed = sum(self.history) >= self.confirm_frames
        if confirmed and self.latched:
            self.detected = True
        elif not self.latched:
            self.detected = confirmed

        self.detect_pub.publish(Bool(data=self.detected))
        self.state_pub.publish(String(data='XO_RED' if self.detected else 'SEARCH'))
        if self.debug_enabled:
            publish_bgr(self.debug_pub, msg, debug)


def main(args=None):
    rclpy.init(args=args)
    node = FinishRedDetector()
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
