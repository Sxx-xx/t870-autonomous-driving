#!/usr/bin/env python3
"""BEV 캘리브레이션용 프레임을 ROS 토픽에서 저장한다.

launch 가 떠 있으면 camera_front 가 C920 을 독점하므로 다른 프로그램으로는
카메라를 열 수 없다. 노드가 실제로 보는 프레임을 그대로 받아 저장한다.
해상도와 카메라 설정이 런타임과 같아야 호모그래피가 맞는다.

  python3 bev_calib/grab.py                          # 기본 경로에 저장
  python3 bev_calib/grab.py --topic /camera/laptop/image_raw --out laptop.png
"""
import argparse
import sys

sys.path = [p for p in sys.path
            if p != '/usr/local/lib/python3.12/dist-packages']

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


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


class Grabber(Node):
    def __init__(self, topic, out, skip):
        super().__init__('bev_grab')
        self.out = out
        self.skip = skip
        self.count = 0
        self.done = False
        self.create_subscription(Image, topic, self.on_image, 5)
        self.get_logger().info(f'{topic} 대기 중...')

    def on_image(self, msg):
        if self.done:
            return
        self.count += 1
        # 처음 몇 프레임은 노출이 안정되기 전이라 버린다.
        if self.count <= self.skip:
            return
        frame = image_to_bgr(msg)
        cv2.imwrite(self.out, frame)
        self.get_logger().info(
            f'저장: {self.out}  ({frame.shape[1]}x{frame.shape[0]})')
        self.done = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/camera/front/image_raw')
    parser.add_argument('--out', default='bev_calib/front.png')
    parser.add_argument('--skip', type=int, default=10)
    args = parser.parse_args()

    rclpy.init()
    node = Grabber(args.topic, args.out, args.skip)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
