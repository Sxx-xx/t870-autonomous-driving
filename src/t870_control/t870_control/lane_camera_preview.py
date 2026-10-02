#!/usr/bin/env python3
"""C920e LaneNet preview and opt-in, low-speed lane command generator."""

import math
import os
import sys
import threading
import time
from collections import deque
from glob import glob
from pathlib import Path

# A locally installed OpenCV 4.13 binding is incomplete on this machine.
# Prefer Ubuntu's working OpenCV 4.6 binding used by ROS Jazzy.
sys.path = [
    path for path in sys.path
    if path != '/usr/local/lib/python3.12/dist-packages'
]

import cv2
import numpy as np
import torch
from ament_index_python.packages import get_package_share_directory
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String

from .model.lanenet.LaneNet import LaneNet
from .traffic_lamp import detect_traffic_light


class LatestFrameCamera:
    def __init__(self, device, width=640, height=480, fps=30):
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.capture = None
        self.lock = threading.Lock()
        self.frame = None
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _open(self):
        resolved_device = resolve_camera_device(self.device)
        if resolved_device is None:
            return None
        capture = cv2.VideoCapture(resolved_device, cv2.CAP_V4L2)
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not capture.isOpened():
            capture.release()
            return None
        return capture

    def _capture_loop(self):
        while self.running:
            if self.capture is None:
                self.capture = self._open()
                if self.capture is None:
                    time.sleep(1.0)
                    continue
            ok, frame = self.capture.read()
            if not ok:
                self.capture.release()
                self.capture = None
                with self.lock:
                    self.frame = None
                time.sleep(0.5)
                continue
            with self.lock:
                self.frame = frame

    def latest(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def close(self):
        self.running = False
        self.thread.join(timeout=1.0)
        if self.capture is not None:
            self.capture.release()


def enhance_shadows(frame):
    """Lift local road contrast without washing out lane-marking colors."""
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    lightness, channel_a, channel_b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    lightness = clahe.apply(lightness)
    return cv2.cvtColor(
        cv2.merge((lightness, channel_a, channel_b)), cv2.COLOR_LAB2BGR)


def trapezoid_roi(width, height):
    """Road ROI calibrated for the rear-mounted T870 camera.

    Lens height is about 0.97 m and the lens is 1.11 m behind the front
    bumper. Keep a wide road area so both markings remain visible when the
    vehicle starts off-center or enters a curve. Exclude only the vehicle body.
    """
    return np.array([[
        (int(width * 0.05), int(height * 0.78)),
        (int(width * 0.20), int(height * 0.24)),
        (int(width * 0.80), int(height * 0.24)),
        (int(width * 0.95), int(height * 0.78)),
    ]], dtype=np.int32)


def apply_lane_roi(mask):
    """Remove detections outside the vehicle's forward road corridor."""
    height, width = mask.shape[:2]
    roi = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(roi, trapezoid_roi(width, height), 255)
    return cv2.bitwise_and(mask, mask, mask=roi)


def bird_eye_matrices(width, height):
    """Return camera-to-ground and ground-to-camera perspective matrices."""
    polygon = trapezoid_roi(width, height)[0].astype(np.float32)
    left = width * 0.03
    right = width * 0.97
    destination = np.array([
        (left, height - 1),
        (left, 0),
        (right, 0),
        (right, height - 1),
    ], dtype=np.float32)
    return (
        cv2.getPerspectiveTransform(polygon, destination),
        cv2.getPerspectiveTransform(destination, polygon),
    )




def warp_to_bird(image, interpolation=cv2.INTER_LINEAR):
    height, width = image.shape[:2]
    transform, _ = bird_eye_matrices(width, height)
    return cv2.warpPerspective(
        image, transform, (width, height), flags=interpolation,
        borderMode=cv2.BORDER_CONSTANT)


def bird_point_to_camera(x, y, width, height):
    """Map a point calculated in bird-eye space back into camera space."""
    _, inverse = bird_eye_matrices(width, height)
    point = np.array([[[float(x), float(y)]]], dtype=np.float32)
    mapped = cv2.perspectiveTransform(point, inverse)[0, 0]
    return float(mapped[0]), float(mapped[1])


def birds_eye_view(image):
    """Rectify the calibrated road trapezoid into a top-down rectangle."""
    height, width = image.shape[:2]
    bird = warp_to_bird(image)
    cv2.line(
        bird, (width // 2, 0), (width // 2, height - 1),
        (255, 0, 255), 2)
    cv2.putText(
        bird, 'BIRD EYE - vehicle center', (15, 28),
        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)
    return bird


def resolve_camera_device(preferred='auto'):
    """Resolve the Logitech C920 even when its /dev/video number changes."""
    if preferred and preferred != 'auto' and Path(preferred).exists():
        return preferred
    for link in sorted(glob('/dev/v4l/by-id/*')):
        name = Path(link).name.lower()
        if ('c920' in name or '046d' in name or 'logitech' in name) \
                and ('index0' in name or 'video-index0' in name):
            return str(Path(link).resolve())
    for device in sorted(glob('/dev/video*')):
        sys_name = Path('/sys/class/video4linux') / Path(device).name / 'name'
        try:
            name = sys_name.read_text(encoding='utf-8').strip().lower()
        except OSError:
            continue
        if 'c920' in name or 'hd pro webcam' in name:
            return device
    return None




def detect_stop_line(bird_frame):
    """Find a broad, near-horizontal white stop line on the bird-eye road."""
    height, width = bird_frame.shape[:2]
    hsv = cv2.cvtColor(bird_frame, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(hsv, (0, 0, 165), (180, 85, 255))
    region = np.zeros_like(white)
    cv2.rectangle(
        region, (int(width * 0.12), int(height * 0.45)),
        (int(width * 0.88), int(height * 0.92)), 255, -1)
    white = cv2.bitwise_and(white, region)
    white = cv2.morphologyEx(
        white, cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (25, 3)))
    contours, _ = cv2.findContours(
        white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for contour in contours:
        x, y, line_width, line_height = cv2.boundingRect(contour)
        if (line_width < width * 0.32
                or line_height > height * 0.10
                or line_width / max(1.0, line_height) < 5.0):
            continue
        score = line_width * (y + line_height * 0.5) / height
        if best is None or score > best[0]:
            best = (score, (x, y, line_width, line_height))
    return (best is not None, None if best is None else best[1])


def lane_color_mask(frame, width, height):
    """Conservative yellow-marking fallback for shadowed pavement."""
    image = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
    enhanced = enhance_shadows(image)
    hsv = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv, (10, 70, 50), (40, 255, 255))
    # Do not use a generic white threshold here: clothing, vehicles and glare
    # can otherwise become false lanes and generate unsafe steering commands.
    mask = apply_lane_roi(yellow)
    open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel, iterations=2)
    return (mask > 0).astype(np.uint8)


def preprocess(frame, width, height, device):
    image = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
    image = enhance_shadows(image)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    image = (image - np.array([0.485, 0.456, 0.406], np.float32)) / np.array(
        [0.229, 0.224, 0.225], np.float32)
    tensor = torch.from_numpy(image.transpose(2, 0, 1)).unsqueeze(0)
    return tensor.to(device)


def find_weights():
    candidates = [
        Path(get_package_share_directory('t870_control')) / 'log' /
        'lanenet_ENet_Focal_epoch100_batchsize8.pth',
        Path(__file__).resolve().parent / 'log' /
        'lanenet_ENet_Focal_epoch100_batchsize8.pth',
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError('LaneNet ENet weights were not installed')


def estimate_lane(mask):
    """Return the centerline between the innermost left/right markings."""
    height, width = mask.shape
    midpoint = width // 2
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8), connectivity=8)
    minimum_area = max(35, int(width * height * 0.0005))
    minimum_height = int(height * 0.18)
    left_candidates = []
    right_candidates = []
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        component_height = int(stats[label, cv2.CC_STAT_HEIGHT])
        component_bottom = int(stats[label, cv2.CC_STAT_TOP]) + component_height
        center_x = float(centroids[label][0])
        if (area < minimum_area or component_height < minimum_height
                or component_bottom < height * 0.48):
            continue
        # Keep the component position first: if several parallel markings are
        # visible, the drivable corridor is bounded by the two closest to the
        # vehicle center, not necessarily by the largest blobs.
        candidate = (center_x, area, label)
        if center_x < midpoint:
            left_candidates.append(candidate)
        else:
            right_candidates.append(candidate)
    if not left_candidates or not right_candidates:
        return None
    left_label = max(left_candidates, key=lambda item: item[0])[2]
    right_label = min(right_candidates, key=lambda item: item[0])[2]
    left_y, left_x = np.where(labels == left_label)
    right_y, right_x = np.where(labels == right_label)
    left_fit = np.polyfit(left_y, left_x, 2)
    right_fit = np.polyfit(right_y, right_x, 2)
    # Do not assume both markings reach the image bottom. The C920 can see a
    # near marking leave the image earlier on curves or after camera pitch
    # changes. Evaluate only inside the vertical interval observed by both
    # fitted markings, avoiding unsafe extrapolation.
    overlap_low = max(float(np.percentile(left_y, 5)),
                      float(np.percentile(right_y, 5)))
    overlap_high = min(float(np.percentile(left_y, 95)),
                       float(np.percentile(right_y, 95)))
    if overlap_high - overlap_low < height * 0.10:
        return None
    near_y = int(overlap_high)
    far_y = int(overlap_low)
    left_near = float(np.polyval(left_fit, near_y))
    right_near = float(np.polyval(right_fit, near_y))
    left_far = float(np.polyval(left_fit, far_y))
    right_far = float(np.polyval(right_fit, far_y))
    left_slope = float(2.0 * left_fit[0] * near_y + left_fit[1])
    right_slope = float(2.0 * right_fit[0] * near_y + right_fit[1])
    lane_width = right_near - left_near
    far_width = right_far - left_far
    if not width * 0.20 <= lane_width <= width * 0.94:
        return None
    # A real road corridor remains ordered and normally gets wider toward the
    # vehicle. This rejects foliage, lamps and isolated colored objects.
    if far_width <= width * 0.08 or lane_width < far_width * 0.85:
        return None
    # Lane boundaries must converge toward the image center in the distance.
    # Horizontal curb ramps and sidewalk entrances often pass the color test,
    # but fail this perspective-direction test.
    if left_slope > -0.20 or right_slope < 0.20:
        return None
    if left_near >= width * 0.48 or right_near <= width * 0.52:
        return None
    confidence = min(1.0, min(left_x.size, right_x.size) / 180.0)
    return {
        'near': (left_near + right_near) * 0.5,
        'far': (left_far + right_far) * 0.5,
        'near_y': near_y,
        'far_y': far_y,
        'left_near': left_near,
        'right_near': right_near,
        'confidence': confidence,
        'source': 'neural_pair',
    }


def estimate_single_yellow_boundary(mask):
    """Infer the vehicle path from one long yellow road-edge marking."""
    height, width = mask.shape
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8), connectivity=8)
    candidates = []
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        top = int(stats[label, cv2.CC_STAT_TOP])
        component_height = int(stats[label, cv2.CC_STAT_HEIGHT])
        bottom = top + component_height
        if (area >= 70 and component_height >= height * 0.22
                and bottom >= height * 0.65):
            candidates.append((area, label))
    if not candidates:
        return None

    selected_area, label = max(candidates)
    pixel_y, pixel_x = np.where(labels == label)
    fit = np.polyfit(pixel_y, pixel_x, 2)
    near_y = int(np.percentile(pixel_y, 92))
    far_y = int(np.percentile(pixel_y, 12))
    if near_y - far_y < height * 0.12:
        return None
    boundary_near = float(np.polyval(fit, near_y))
    boundary_far = float(np.polyval(fit, far_y))
    slope = float(2.0 * fit[0] * near_y + fit[1])
    midpoint = width * 0.5
    # Expected edge position expands from about 55% near the horizon to 75%
    # at the image bottom. This follows perspective instead of using one fixed
    # pixel target for every camera pitch.
    vertical_ratio_near = max(0.45, min(1.0, near_y / height))
    vertical_ratio_far = max(0.45, min(1.0, far_y / height))
    right_target_near = width * (
        0.55 + (vertical_ratio_near - 0.45) * (0.20 / 0.55))
    right_target_far = width * (
        0.55 + (vertical_ratio_far - 0.45) * (0.20 / 0.55))

    if boundary_near > midpoint and slope >= 0.20:
        # Right road edge: it should appear near 82% of image width when the
        # vehicle is at the desired lateral distance.
        near = midpoint + boundary_near - right_target_near
        far = midpoint + boundary_far - right_target_far
    elif boundary_near < midpoint and slope <= -0.20:
        near = midpoint + boundary_near - (width - right_target_near)
        far = midpoint + boundary_far - (width - right_target_far)
    else:
        return None

    confidence = min(0.80, 0.45 + selected_area / 1500.0)
    return {
        'near': near,
        'far': far,
        'near_y': near_y,
        'far_y': far_y,
        'left_near': near,
        'right_near': near,
        'confidence': confidence,
        'source': 'single_yellow',
    }


def estimate_hybrid_yellow_lane(left_mask, right_mask=None):
    """Rebuild a fragmented right marking and pair it with a solid left one."""
    if right_mask is None:
        right_mask = left_mask
    height, width = left_mask.shape
    midpoint = width * 0.5
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(
        left_mask.astype(np.uint8), connectivity=8)

    left_labels = [
        label for label in range(1, count)
        if centroids[label][0] < midpoint
        and stats[label, cv2.CC_STAT_AREA] >= 100
        and stats[label, cv2.CC_STAT_HEIGHT] >= height * 0.25
    ]
    if not left_labels:
        return None
    # Select the innermost valid left boundary. An outer curb/flower-bed edge
    # can have more pixels, but must not pull the commanded path toward it.
    left_label = max(left_labels, key=lambda label: centroids[label][0])
    left_y, left_x = np.where(labels == left_label)
    left_fit = np.polyfit(left_y, left_x, 2)

    # The outdoor right marking is partly occluded and arrives as small pieces.
    # Use component centers and RANSAC-like alignment instead of demanding one
    # connected blob. Very large edge blobs (hands/bodywork) are excluded.
    right_count, right_labels, right_stats, right_centroids = \
        cv2.connectedComponentsWithStats(
            right_mask.astype(np.uint8), connectivity=8)
    pieces = []
    for label in range(1, right_count):
        area = int(right_stats[label, cv2.CC_STAT_AREA])
        center_x, center_y = map(float, right_centroids[label])
        if (center_x > midpoint and height * 0.25 < center_y < height * 0.72
                and 4 <= area <= 220):
            pieces.append((label, center_x, center_y))
    if len(pieces) < 3:
        return None

    best_inliers = []
    best_span = 0.0
    for first_index, first in enumerate(pieces):
        for second in pieces[first_index + 1:]:
            delta_y = second[2] - first[2]
            if abs(delta_y) < height * 0.04:
                continue
            slope = (second[1] - first[1]) / delta_y
            if not 0.25 <= slope <= 4.0:
                continue
            intercept = first[1] - slope * first[2]
            inliers = [
                piece for piece in pieces
                if abs(piece[1] - (slope * piece[2] + intercept)) <= 8.0
            ]
            span = max(p[2] for p in inliers) - min(p[2] for p in inliers)
            if (len(inliers), span) > (len(best_inliers), best_span):
                best_inliers = inliers
                best_span = span
    if len(best_inliers) < 3 or best_span < height * 0.12:
        return None

    right_pixel_y = []
    right_pixel_x = []
    for label, _, _ in best_inliers:
        component_y, component_x = np.where(right_labels == label)
        right_pixel_y.append(component_y)
        right_pixel_x.append(component_x)
    right_y = np.concatenate(right_pixel_y)
    right_x = np.concatenate(right_pixel_x)
    right_fit = np.polyfit(right_y, right_x, 1)

    # The right marking can leave the image early because it is close to the
    # vehicle. Fit its aligned upper fragments, then extrapolate only across
    # the vertical range supported by the solid left marking.
    far_y = int(max(height * 0.45, np.percentile(left_y, 15)))
    near_y = int(min(height * 0.80, np.percentile(left_y, 90)))
    if near_y - far_y < height * 0.08:
        return None
    left_near = float(np.polyval(left_fit, near_y))
    right_near = float(np.polyval(right_fit, near_y))
    left_far = float(np.polyval(left_fit, far_y))
    right_far = float(np.polyval(right_fit, far_y))
    if right_near <= left_near or right_far <= left_far:
        return None
    near_width = right_near - left_near
    far_width = right_far - left_far
    if (far_width < width * 0.10 or near_width < far_width * 0.85
            or near_width > width * 1.60):
        return None
    return {
        'near': (left_near + right_near) * 0.5,
        'far': (left_far + right_far) * 0.5,
        'near_y': near_y,
        'far_y': far_y,
        'left_near': left_near,
        'right_near': right_near,
        'confidence': min(0.90, 0.55 + 0.07 * len(best_inliers)),
        'source': 'hybrid_pair',
    }


class LaneCommandNode(Node):
    def __init__(self):
        super().__init__('lane_camera_preview')
        defaults = {
            'camera_device': 'auto',
            'control_enabled': False,
            'target_speed_mps': 0.30,
            'max_steer_rad': 0.18,
            'kp_offset': 0.30,
            'kp_heading': 0.12,
            'minimum_confidence': 0.40,
            'lane_hold_sec': 0.80,
            'lane_smoothing': 0.35,
            # A detected left/right pair defines its own exact midpoint.
            'lane_center_bias_ratio': 0.0,
            'save_diagnostics': False,
            'diagnostic_interval_sec': 2.0,
            'diagnostic_directory': '/tmp/t870_lane_diagnostics',
            'traffic_control_enabled': True,
            'signal_confirm_frames': 4,
            'stop_line_confirm_frames': 3,
            # 이 노드가 카메라를 독점한다. 프레임이 필요한 다른 노드를
            # 위해 원본을 /camera/image_raw 로 내보낸다.
            'publish_camera_image': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.control_enabled = bool(self.get_parameter('control_enabled').value)
        self.camera_device = str(self.get_parameter('camera_device').value)
        self.target_speed = float(self.get_parameter('target_speed_mps').value)
        self.max_steer = float(self.get_parameter('max_steer_rad').value)
        self.kp_offset = float(self.get_parameter('kp_offset').value)
        self.kp_heading = float(self.get_parameter('kp_heading').value)
        self.minimum_confidence = float(
            self.get_parameter('minimum_confidence').value)
        self.lane_hold_sec = float(self.get_parameter('lane_hold_sec').value)
        self.lane_smoothing = float(self.get_parameter('lane_smoothing').value)
        self.lane_center_bias = float(
            self.get_parameter('lane_center_bias_ratio').value)
        self.last_estimate = None
        self.last_estimate_time = 0.0
        self.save_diagnostics = bool(
            self.get_parameter('save_diagnostics').value)
        self.diagnostic_interval = float(
            self.get_parameter('diagnostic_interval_sec').value)
        self.diagnostic_directory = Path(str(
            self.get_parameter('diagnostic_directory').value))
        self.last_diagnostic_time = 0.0
        self.traffic_control_enabled = bool(
            self.get_parameter('traffic_control_enabled').value)
        self.signal_confirm_frames = int(
            self.get_parameter('signal_confirm_frames').value)
        self.stop_line_confirm_frames = int(
            self.get_parameter('stop_line_confirm_frames').value)
        history_size = max(6, self.signal_confirm_frames + 2)
        self.signal_history = deque(maxlen=history_size)
        self.stop_line_history = deque(maxlen=max(
            5, self.stop_line_confirm_frames + 2))
        self.traffic_light_state = 'UNKNOWN'
        self.traffic_stop_latched = False
        if self.save_diagnostics:
            self.diagnostic_directory.mkdir(parents=True, exist_ok=True)
        if self.target_speed < 0.0 or self.max_steer <= 0.0:
            raise ValueError('lane speed must be nonnegative and max steer positive')
        self.command_pub = self.create_publisher(Twist, '/cmd_vel/lane', 10)
        self.detected_pub = self.create_publisher(Bool, '/vision/lane_detected', 10)
        self.offset_pub = self.create_publisher(Float32, '/vision/lane_offset', 10)
        self.confidence_pub = self.create_publisher(
            Float32, '/vision/lane_confidence', 10)
        self.steering_pub = self.create_publisher(
            Float32, '/vision/lane_steering', 10)
        self.source_pub = self.create_publisher(
            String, '/vision/lane_source', 10)
        self.traffic_light_pub = self.create_publisher(
            String, '/vision/traffic_light_state', 10)
        self.stop_line_pub = self.create_publisher(
            Bool, '/vision/stop_line_detected', 10)
        self.traffic_stop_pub = self.create_publisher(
            Bool, '/vision/traffic_stop', 10)
        # 이 노드가 카메라 장치를 독점하므로, 프레임이 필요한 다른 노드는
        # 여기서 내보내는 토픽을 받아야 한다. 대역폭 때문에 기본은 끈다.
        self.publish_camera_image = bool(
            self.get_parameter('publish_camera_image').value)
        self.image_pub = self.create_publisher(
            Image, '/camera/image_raw', 2) if self.publish_camera_image else None
        mode = 'ENABLED' if self.control_enabled else 'DISABLED (preview only)'
        self.get_logger().warning(f'Lane command output {mode}')

    def publish_camera_frame(self, frame):
        """원본 프레임을 /camera/image_raw 로 내보낸다.

        이 노드가 카메라 장치를 독점하므로 webcam_pub_node 를 같이 띄울 수
        없다. 프레임이 필요한 노드(ox_signal_detector 등)는 이 토픽을 받는다.
        """
        if self.image_pub is None:
            return
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = frame.tobytes()
        self.image_pub.publish(msg)

    def stabilize_estimate(self, estimate):
        """Smooth valid detections and bridge only very short mask dropouts."""
        now = time.monotonic()
        if estimate is None:
            if (self.last_estimate is not None
                    and now - self.last_estimate_time <= self.lane_hold_sec):
                held = dict(self.last_estimate)
                age_ratio = (now - self.last_estimate_time) / self.lane_hold_sec
                held['confidence'] = max(
                    self.minimum_confidence,
                    held['confidence'] * (1.0 - 0.5 * age_ratio))
                return held
            return None

        if self.last_estimate is not None:
            alpha = max(0.0, min(1.0, self.lane_smoothing))
            estimate = dict(estimate)
            for key in ('near', 'far', 'left_near', 'right_near'):
                estimate[key] = (
                    alpha * estimate[key]
                    + (1.0 - alpha) * self.last_estimate[key])
        self.last_estimate = dict(estimate)
        self.last_estimate_time = now
        return estimate

    def apply_center_bias(self, estimate, image_width):
        if estimate is None:
            return None
        shifted = dict(estimate)
        shift = self.lane_center_bias * image_width
        for key in ('near', 'far', 'left_near', 'right_near'):
            shifted[key] = max(
                0.0, min(float(image_width - 1), shifted[key] + shift))
        return shifted

    def update_traffic_control(self, raw_signal, raw_stop_line):
        self.signal_history.append(raw_signal)
        self.stop_line_history.append(bool(raw_stop_line))
        for candidate in ('RED', 'YELLOW', 'GREEN'):
            if self.signal_history.count(candidate) >= self.signal_confirm_frames:
                self.traffic_light_state = candidate
                break
        else:
            if self.signal_history.count('UNKNOWN') >= self.signal_confirm_frames:
                self.traffic_light_state = 'UNKNOWN'

        stop_line = (
            sum(self.stop_line_history) >= self.stop_line_confirm_frames)
        if (self.traffic_control_enabled and stop_line
                and self.traffic_light_state in ('RED', 'YELLOW')):
            if not self.traffic_stop_latched:
                self.get_logger().warning(
                    f'{self.traffic_light_state} signal + stop line: STOP latched')
            self.traffic_stop_latched = True
        elif (self.traffic_stop_latched
                and self.traffic_light_state == 'GREEN'):
            self.traffic_stop_latched = False
            self.stop_line_history.clear()
            self.get_logger().info('GREEN signal confirmed: traffic STOP cleared')

        self.traffic_light_pub.publish(
            String(data=self.traffic_light_state))
        self.stop_line_pub.publish(Bool(data=stop_line))
        self.traffic_stop_pub.publish(
            Bool(data=self.traffic_stop_latched))
        return stop_line, self.traffic_stop_latched

    def publish_result(self, estimate, image_width, traffic_stop=False):
        valid = estimate is not None and estimate['confidence'] >= self.minimum_confidence
        command = Twist()
        offset = 0.0
        steering = 0.0
        if valid:
            half_width = image_width * 0.5
            offset = (estimate['near'] - half_width) / half_width
            heading = (estimate['near'] - estimate['far']) / half_width
            steering = self.kp_offset * offset + self.kp_heading * heading
            steering = max(-self.max_steer, min(self.max_steer, steering))
            if self.control_enabled and not traffic_stop:
                command.linear.x = self.target_speed
                command.angular.z = steering
        self.command_pub.publish(command)
        self.detected_pub.publish(Bool(data=valid))
        self.offset_pub.publish(Float32(data=float(offset)))
        confidence = 0.0 if estimate is None else estimate['confidence']
        self.confidence_pub.publish(Float32(data=float(confidence)))
        self.steering_pub.publish(Float32(data=float(steering)))
        source = 'none' if estimate is None else estimate.get('source', 'unknown')
        self.source_pub.publish(String(data=source))
        return valid, offset, steering

    def stop(self):
        for _ in range(3):
            self.command_pub.publish(Twist())

    def save_diagnostic(self, frame, neural_mask, color_mask, result):
        if not self.save_diagnostics:
            return
        now = time.monotonic()
        if now - self.last_diagnostic_time < self.diagnostic_interval:
            return
        self.last_diagnostic_time = now
        cv2.imwrite(str(self.diagnostic_directory / 'latest_camera.jpg'), frame)
        cv2.imwrite(str(self.diagnostic_directory / 'latest_neural.png'),
                    neural_mask * 255)
        cv2.imwrite(str(self.diagnostic_directory / 'latest_yellow.png'),
                    color_mask * 255)
        cv2.imwrite(str(self.diagnostic_directory / 'latest_overlay.jpg'), result)


def main():
    rclpy.init()
    node = LaneCommandNode()
    device_path = os.environ.get('T870_CAMERA_DEVICE', node.camera_device)
    inference_width = int(os.environ.get('T870_LANE_WIDTH', '320'))
    inference_height = int(os.environ.get('T870_LANE_HEIGHT', '240'))
    torch_device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')

    print(f'Lane preview: camera={device_path}, inference='
          f'{inference_width}x{inference_height}, device={torch_device}')
    print('Vehicle commands require explicit control_enabled:=true')

    model = LaneNet(arch='ENet')
    state = torch.load(find_weights(), map_location=torch_device, weights_only=True)
    model.load_state_dict(state)
    model.to(torch_device).eval()

    camera = LatestFrameCamera(device_path)
    last_time = time.monotonic()
    displayed_fps = 0.0
    try:
        while camera.running:
            frame = camera.latest()
            if frame is not None:
                node.publish_camera_frame(frame)
            if frame is None:
                # Keep publishing an explicit zero command while USB is absent;
                # the capture thread will reconnect automatically.
                node.publish_result(None, inference_width)
                rclpy.spin_once(node, timeout_sec=0.0)
                time.sleep(0.05)
                continue
            with torch.inference_mode():
                output = model(preprocess(
                    frame, inference_width, inference_height, torch_device))
            camera_neural_mask = output['binary_seg_pred'][0, 0].to(
                'cpu').numpy().astype(np.uint8)
            camera_neural_mask = (
                apply_lane_roi(camera_neural_mask * 255) > 0).astype(np.uint8)
            camera_color_mask = lane_color_mask(
                frame, inference_width, inference_height)

            # All lane selection, center-path estimation and steering are
            # calculated in the rectified ground plane.
            neural_mask = (
                warp_to_bird(camera_neural_mask * 255, cv2.INTER_NEAREST) > 0
            ).astype(np.uint8)
            color_mask = (
                warp_to_bird(camera_color_mask * 255, cv2.INTER_NEAREST) > 0
            ).astype(np.uint8)
            bird_mask = cv2.bitwise_or(neural_mask, color_mask)
            camera_mask = cv2.bitwise_or(
                camera_neural_mask, camera_color_mask)
            raw_bird_frame = warp_to_bird(frame)
            raw_signal, signal_box = detect_traffic_light(frame)
            raw_stop_line, stop_line_box = detect_stop_line(raw_bird_frame)
            stop_line, traffic_stop = node.update_traffic_control(
                raw_signal, raw_stop_line)
            # First enforce the geometric rule requested for vehicle control:
            # split every recognized boundary at the image center, select the
            # rightmost one on the left and the leftmost one on the right, and
            # command the exact midpoint of that innermost pair.
            raw_estimate = estimate_lane(bird_mask)
            if raw_estimate is None:
                raw_estimate = estimate_hybrid_yellow_lane(
                    color_mask, neural_mask)
            if raw_estimate is None:
                raw_estimate = estimate_single_yellow_boundary(color_mask)
            raw_estimate = node.apply_center_bias(
                raw_estimate, inference_width)
            estimate = node.stabilize_estimate(raw_estimate)
            valid, offset, steering = node.publish_result(
                estimate, inference_width, traffic_stop)
            display_mask = cv2.resize(
                camera_mask, (frame.shape[1], frame.shape[0]),
                interpolation=cv2.INTER_NEAREST)

            overlay = frame.copy()
            overlay[display_mask > 0] = (0, 255, 0)
            result = cv2.addWeighted(frame, 0.65, overlay, 0.35, 0.0)
            cv2.polylines(
                result,
                [trapezoid_roi(frame.shape[1], frame.shape[0])[0]],
                True, (255, 255, 0), 2)
            if signal_box is not None:
                sx, sy, sw, sh = signal_box
                signal_color = {
                    'RED': (0, 0, 255),
                    'YELLOW': (0, 255, 255),
                    'GREEN': (0, 255, 0),
                }.get(raw_signal, (180, 180, 180))
                cv2.rectangle(
                    result, (sx, sy), (sx + sw, sy + sh), signal_color, 2)
            if estimate is not None:
                scale_x = frame.shape[1] / inference_width
                scale_y = frame.shape[0] / inference_height
                near_camera = bird_point_to_camera(
                    estimate['near'], estimate['near_y'],
                    inference_width, inference_height)
                far_camera = bird_point_to_camera(
                    estimate['far'], estimate['far_y'],
                    inference_width, inference_height)
                near = (int(near_camera[0] * scale_x),
                        int(near_camera[1] * scale_y))
                far = (int(far_camera[0] * scale_x),
                       int(far_camera[1] * scale_y))
                cv2.line(result, near, far, (0, 0, 255), 3)
                cv2.circle(result, near, 7, (255, 0, 255), -1)
            now = time.monotonic()
            elapsed = now - last_time
            if elapsed > 0.0:
                displayed_fps = 1.0 / elapsed
            last_time = now
            cv2.putText(result, f'LaneNet {displayed_fps:.1f} FPS', (15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            lane_source = 'none' if estimate is None else estimate.get('source', 'unknown')
            state = 'VALID' if valid else 'LOST/LOW CONFIDENCE'
            cv2.putText(result, f'{state} {lane_source} offset={offset:+.2f} steer={steering:+.2f}',
                        (15, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 255, 0) if valid else (0, 0, 255), 2)
            control_text = 'CONTROL ENABLED' if node.control_enabled else 'PREVIEW ONLY'
            cv2.putText(result, control_text, (15, 88),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 0, 255) if node.control_enabled else (0, 255, 255), 2)
            cv2.putText(
                result,
                f'SIGNAL={node.traffic_light_state} '
                f'STOP_LINE={"YES" if stop_line else "NO"}',
                (15, 116), cv2.FONT_HERSHEY_SIMPLEX, 0.58,
                (0, 0, 255) if traffic_stop else (255, 255, 0), 2)
            bird_eye = birds_eye_view(frame)
            bird_display_mask = cv2.resize(
                bird_mask, (frame.shape[1], frame.shape[0]),
                interpolation=cv2.INTER_NEAREST)
            bird_overlay = bird_eye.copy()
            bird_overlay[bird_display_mask > 0] = (0, 255, 0)
            bird_eye = cv2.addWeighted(
                bird_eye, 0.65, bird_overlay, 0.35, 0.0)
            if stop_line_box is not None:
                bx, by, bw, bh = stop_line_box
                cv2.rectangle(
                    bird_eye, (bx, by), (bx + bw, by + bh),
                    (0, 165, 255), 3)
            if estimate is not None:
                bird_near = (
                    int(estimate['near'] * scale_x),
                    int(estimate['near_y'] * scale_y))
                bird_far = (
                    int(estimate['far'] * scale_x),
                    int(estimate['far_y'] * scale_y))
                cv2.line(bird_eye, bird_near, bird_far, (0, 0, 255), 3)
                cv2.circle(bird_eye, bird_near, 7, (255, 0, 255), -1)
            cv2.putText(
                bird_eye, 'DETECTION + CONTROL IN BIRD EYE', (15, 56),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            if traffic_stop:
                cv2.putText(
                    bird_eye, 'TRAFFIC STOP', (15, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
            node.save_diagnostic(frame, neural_mask, color_mask, result)
            cv2.imshow('T870 C920e Lane Preview', result)
            cv2.imshow('T870 Bird Eye Lane View', bird_eye)
            rclpy.spin_once(node, timeout_sec=0.0)
            if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                break
    finally:
        node.stop()
        camera.close()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
