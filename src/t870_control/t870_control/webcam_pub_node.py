#!/usr/bin/env python3
"""Small OpenCV USB-camera publisher without a cv_bridge dependency.

대회에서는 카메라를 두 대 쓴다. 한 대가 한 장치를 독점하므로 각각 이
노드를 띄우고 topic/device 를 다르게 준다.

  C920(외장)      -> /camera/front/image_raw   정적장애물 판단
  노트북 내장      -> /camera/laptop/image_raw  신호등, X/화살표

device_name 을 주면 /dev/v4l/by-id 와 /sys 의 장치 이름에서 찾는다.
/dev/video 번호는 꽂는 순서에 따라 바뀌므로 번호로 고정하면 안 된다.
"""
import sys
import time

# The locally installed OpenCV build requires an unavailable HDF library.
# Prefer the working Ubuntu/ROS OpenCV binding.
sys.path = [p for p in sys.path
            if p != '/usr/local/lib/python3.12/dist-packages']

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from glob import glob
from pathlib import Path


def resolve_device(device_name, device_index):
    """장치 이름으로 찾는다. 못 찾으면 device_index 로 떨어진다.

    /dev/video 번호는 꽂는 순서에 따라 바뀐다. 이름으로 잡아야 두 카메라가
    서로 바뀌지 않는다.
    """
    key = (device_name or '').strip().lower()
    if not key:
        return int(device_index)
    for link in sorted(glob('/dev/v4l/by-id/*')):
        name = Path(link).name.lower()
        if key in name and ('index0' in name or 'video-index0' in name):
            return str(Path(link).resolve())
    for device in sorted(glob('/dev/video*')):
        sys_name = Path('/sys/class/video4linux') / Path(device).name / 'name'
        try:
            name = sys_name.read_text(encoding='utf-8').strip().lower()
        except OSError:
            continue
        if key in name:
            return device
    return int(device_index)

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_pub_node')
        for k,v in {'device_index':0,'device_name':'','image_topic':'/camera/image_raw','fps':15.0,'frame_id':'camera','width':640,'height':480,'show_window':False,'window_name':'T870 camera','max_read_failures':30}.items():self.declare_parameter(k,v)
        self.pub=self.create_publisher(Image,str(self.get_parameter('image_topic').value),10);self.cap=None;self.last_retry=0.
        # MJPG 스트림은 열린 직후 몇 프레임이 실패한다. 한 번 실패했다고
        # 닫아 버리면 열기/닫기를 1초마다 반복해 1 Hz 밖에 안 나온다.
        self.read_fail=0
        self.show_window=bool(self.get_parameter('show_window').value)
        self.window_name=str(self.get_parameter('window_name').value)
        self.traffic_red=False
        self.xo_detected=False
        self.create_subscription(Bool,'/vision/traffic_stop',self.traffic_callback,10)
        self.create_subscription(Bool,'/vision/xo_detected',self.xo_callback,10)
        self.create_timer(1.0/float(self.get_parameter('fps').value),self.tick)
        self.published=0; self.last_report=time.monotonic()
        self.get_logger().warning(
            'cv2 %s from %s' % (cv2.__version__, getattr(cv2, '__file__', '?')))
        self.get_logger().warning(
            'Camera publisher: device_name=%r -> %r, topic=%s'
            % (str(self.get_parameter('device_name').value),
               resolve_device(str(self.get_parameter('device_name').value),
                              int(self.get_parameter('device_index').value)),
               str(self.get_parameter('image_topic').value)))

    def traffic_callback(self,msg):
        self.traffic_red=bool(msg.data)

    def xo_callback(self,msg):
        self.xo_detected=bool(msg.data)
    def open(self):
        if time.monotonic()-self.last_retry<1.:return
        self.last_retry=time.monotonic()
        target=resolve_device(str(self.get_parameter('device_name').value),
                              int(self.get_parameter('device_index').value))
        # 백엔드를 명시한다. 문자열 경로를 그냥 주면 OpenCV 가 V4L2 가
        # 아니라 GStreamer 로 열려고 해서 'Could not read from resource'
        # 로 실패한다. MJPG 도 같이 지정해야 C920 이 640x480 을 낸다.
        self.cap=cv2.VideoCapture(target, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,int(self.get_parameter('width').value));self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,int(self.get_parameter('height').value))
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            self.cap.release(); self.cap=None
            # 다른 노드가 이미 잡고 있으면 장치는 있는데 열리지 않는다.
            # device_name 이 두 카메라에 모두 걸리는지 확인할 것.
            busy = ''
            try:
                import subprocess
                held = subprocess.run(['fuser', str(target)],
                                      capture_output=True, timeout=1).stdout
                if held.strip():
                    busy = ' (다른 프로세스가 점유 중)'
            except Exception:
                pass
            self.get_logger().warning(f'카메라를 열 수 없다: {target}{busy}')
            return
        self.read_fail=0
        # 스트림이 올라올 때까지 몇 장 버린다.
        for _ in range(5):
            self.cap.read()
    def tick(self):
        if self.cap is None or not self.cap.isOpened():self.open();return
        ok,frame=self.cap.read()
        if not ok:
            self.read_fail+=1
            now=time.monotonic()
            if now-self.last_report>=5.0:
                self.get_logger().warning(
                    '프레임 안 나옴. 연속 읽기실패 %d' % self.read_fail)
                self.last_report=now
            if self.read_fail>=int(self.get_parameter('max_read_failures').value):
                self.get_logger().warning('연속 읽기 실패. 카메라를 다시 연다')
                self.cap.release();self.cap=None;self.read_fail=0
            return
        self.read_fail=0
        msg=Image();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id=str(self.get_parameter('frame_id').value)
        msg.height,msg.width=frame.shape[:2];msg.encoding='bgr8';msg.is_bigendian=0;msg.step=msg.width*3;msg.data=frame.tobytes();self.pub.publish(msg)
        self.published+=1
        now=time.monotonic()
        if now-self.last_report>=5.0:
            self.get_logger().warning(
                '발행 %d 프레임 (%.1f Hz), 읽기실패 %d'
                % (self.published, self.published/(now-self.last_report),
                   self.read_fail))
            self.published=0; self.last_report=now
        if self.show_window:
            traffic_text='TRAFFIC: RED' if self.traffic_red else 'TRAFFIC: PASS'
            xo_text='XO: DETECTED' if self.xo_detected else 'XO: SEARCH'
            cv2.putText(frame, traffic_text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX,
                        0.9, (0, 0, 255) if self.traffic_red else (0, 220, 0), 2,
                        cv2.LINE_AA)
            cv2.putText(frame, xo_text, (20, 72), cv2.FONT_HERSHEY_SIMPLEX,
                        0.9, (0, 0, 255) if self.xo_detected else (0, 220, 0), 2,
                        cv2.LINE_AA)
            cv2.imshow(self.window_name, frame)
            # Keep the GUI responsive. Press q or ESC to close the preview.
            key=cv2.waitKey(1) & 0xff
            if key in (ord('q'), 27):
                self.show_window=False
                cv2.destroyWindow(self.window_name)
    def destroy_node(self):
        if self.cap is not None:self.cap.release()
        if self.show_window:cv2.destroyAllWindows()
        super().destroy_node()
def main(args=None):
    rclpy.init(args=args);n=WebcamPublisher()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown() if rclpy.ok() else None
