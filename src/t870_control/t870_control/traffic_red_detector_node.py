#!/usr/bin/env python3
"""Red-only traffic-light detector for a ROS 2 USB camera stream.

The detector publishes a stop request only after temporal confirmation. Green
and yellow are deliberately treated as pass-through states.
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


def publish_bgr(node, publisher, source, frame):
    msg = Image()
    msg.header = source.header
    msg.height, msg.width = frame.shape[:2]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = frame.tobytes()
    publisher.publish(msg)


class TrafficRedDetector(Node):
    def __init__(self):
        super().__init__('traffic_red_detector')
        defaults = {
            'image_topic': '/camera/image_raw',
            'min_red_area_ratio': 0.00012,
            'min_mean_saturation': 100.0,
            'min_mean_red_excess': 45.0,
            'min_mean_value': 125.0,
            'min_local_value_contrast': 18.0,
            'min_circularity': 0.42,
            'max_red_area_ratio': 0.012,
            'confirm_frames': 5,
            'window_frames': 8,
            'clear_frames': 8,
            'roi_x_min': 0.08,
            'roi_x_max': 0.92,
            'roi_y_min': 0.02,
            'roi_y_max': 0.68,
            'publish_debug_image': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        topic = str(self.get_parameter('image_topic').value)
        self.min_area_ratio = float(self.get_parameter('min_red_area_ratio').value)
        self.min_mean_saturation = float(
            self.get_parameter('min_mean_saturation').value)
        self.min_mean_red_excess = float(
            self.get_parameter('min_mean_red_excess').value)
        self.min_mean_value = float(self.get_parameter('min_mean_value').value)
        self.min_local_value_contrast = float(
            self.get_parameter('min_local_value_contrast').value)
        self.min_circularity = float(self.get_parameter('min_circularity').value)
        self.max_red_area_ratio = float(
            self.get_parameter('max_red_area_ratio').value)
        self.confirm_frames = int(self.get_parameter('confirm_frames').value)
        self.history = deque(maxlen=int(self.get_parameter('window_frames').value))
        self.clear_frames = int(self.get_parameter('clear_frames').value)
        self.red_stop = False
        self.clear_count = 0
        self.debug_enabled = bool(self.get_parameter('publish_debug_image').value)

        self.stop_pub = self.create_publisher(Bool, '/vision/traffic_stop', 10)
        self.state_pub = self.create_publisher(String, '/vision/traffic_light', 10)
        self.debug_pub = self.create_publisher(Image, '/vision/traffic_debug', 2)
        self.create_subscription(Image, topic, self.on_image, 5)
        self.get_logger().info(f'Listening to {topic}; red-only stop detector ready')

    def detect_red(self, frame):
        height, width = frame.shape[:2]
        x0, x1 = int(width * float(self.get_parameter('roi_x_min').value)), int(
            width * float(self.get_parameter('roi_x_max').value))
        y0, y1 = int(height * float(self.get_parameter('roi_y_min').value)), int(
            height * float(self.get_parameter('roi_y_max').value))
        roi = frame[y0:y1, x0:x1]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        b, g, r = cv2.split(roi)

        # HSV catches hue under changing exposure; RGB dominance rejects most
        # orange/white reflections that happen to have a red-ish hue.
        red_hue = cv2.inRange(hsv, (0, 8, 55), (12, 255, 255))
        red_hue |= cv2.inRange(hsv, (168, 40, 55), (180, 255, 255))
        brightness_floor = max(75, min(150, int(np.percentile(v, 75) * 0.50)))
        color_red = ((r.astype(np.int16) - g.astype(np.int16) >= 45)
                     & (r.astype(np.int16) - b.astype(np.int16) >= 35)
                     & (s >= 100) & (v >= brightness_floor))
        mask = red_hue & (color_red.astype(np.uint8) * 255)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        min_area = max(8, int(roi.shape[0] * roi.shape[1] * self.min_area_ratio))
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        best = None
        for index in range(1, count):
            area = int(stats[index, cv2.CC_STAT_AREA])
            if area < min_area:
                continue
            if area > roi.shape[0] * roi.shape[1] * self.max_red_area_ratio:
                continue
            bw = int(stats[index, cv2.CC_STAT_WIDTH])
            bh = int(stats[index, cv2.CC_STAT_HEIGHT])
            if bw == 0 or bh == 0 or not 0.45 <= bw / bh <= 2.2:
                continue
            component_mask = (labels == index).astype(np.uint8) * 255
            contours, _ = cv2.findContours(
                component_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            perimeter = cv2.arcLength(max(contours, key=cv2.contourArea), True)
            circularity = 4.0 * np.pi * area / (perimeter * perimeter) \
                if perimeter > 0.0 else 0.0
            if circularity < self.min_circularity:
                continue
            component = labels == index
            mean_s = float(np.mean(s[component]))
            mean_v = float(np.mean(v[component]))
            mean_excess = float(np.mean(r[component].astype(np.float32)
                                       - g[component].astype(np.float32)))
            mean_value = float(np.mean(v[component]))
            ring = cv2.dilate(component_mask, np.ones((9, 9), np.uint8),
                              iterations=1).astype(bool) & ~component
            ring_value = float(np.mean(v[ring])) if np.any(ring) else 0.0
            if (mean_s < self.min_mean_saturation
                    or mean_excess < self.min_mean_red_excess
                    or mean_v < brightness_floor
                    or mean_value < self.min_mean_value
                    or mean_value - ring_value < self.min_local_value_contrast):
                continue
            if best is None or area > best[0]:
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
            detected, debug = self.detect_red(frame)
        except (ValueError, cv2.error) as error:
            self.get_logger().warning(f'camera frame decode failed: {error}')
            return

        self.history.append(detected)
        confirmed = sum(self.history) >= self.confirm_frames
        if confirmed:
            self.red_stop = True
            self.clear_count = 0
        elif self.red_stop:
            self.clear_count += 1
            if self.clear_count >= self.clear_frames:
                self.red_stop = False

        self.stop_pub.publish(Bool(data=self.red_stop))
        self.state_pub.publish(String(data='RED' if self.red_stop else 'PASS'))
        if self.debug_enabled:
            publish_bgr(self, self.debug_pub, msg, debug)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficRedDetector()
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
