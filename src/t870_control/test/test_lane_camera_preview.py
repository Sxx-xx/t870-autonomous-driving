import numpy as np

from t870_control.lane_camera_preview import (
    cv2,
    detect_stop_line,
    detect_traffic_light,
    estimate_lane,
    estimate_hybrid_yellow_lane,
    estimate_single_yellow_boundary,
    trapezoid_roi,
)


def test_lane_center_uses_innermost_parallel_boundaries():
    mask = np.zeros((240, 320), dtype=np.uint8)
    # Outer road/curb-like markings.
    cv2.line(mask, (40, 230), (120, 60), 1, 7)
    cv2.line(mask, (280, 230), (200, 60), 1, 7)
    # Inner lane markings which must define the commanded center.
    cv2.line(mask, (100, 230), (140, 60), 1, 7)
    cv2.line(mask, (220, 230), (180, 60), 1, 7)

    estimate = estimate_lane(mask)

    assert estimate is not None
    assert abs(estimate['near'] - 160.0) < 5.0
    assert estimate['left_near'] > 90.0
    assert estimate['right_near'] < 230.0


def test_road_roi_keeps_full_lane_width_through_five_meters():
    polygon = trapezoid_roi(640, 480)[0]
    assert polygon[0, 0] == 32
    assert polygon[3, 0] == 608
    assert polygon[1, 0] == 128
    assert polygon[1, 1] == 115
    assert polygon[2, 0] == 512
    assert polygon[2, 1] == 115


def test_hybrid_lane_uses_solid_left_and_fragmented_right():
    height, width = 240, 320
    yellow = np.zeros((height, width), dtype=np.uint8)
    neural = np.zeros_like(yellow)
    cv2.line(yellow, (4, 220), (145, 100), 1, 7)
    for y in (70, 84, 98, 112, 126):
        x = int(1.35 * y + 75)
        cv2.line(neural, (x - 5, y - 2), (x + 5, y + 2), 1, 3)

    estimate = estimate_hybrid_yellow_lane(yellow, neural)

    assert estimate is not None
    assert estimate['source'] == 'hybrid_pair'
    assert estimate['left_near'] < estimate['near'] < estimate['right_near']
    assert estimate['confidence'] >= 0.55


def test_horizontal_curb_is_not_a_single_lane_boundary():
    mask = np.zeros((240, 320), dtype=np.uint8)
    cv2.line(mask, (20, 190), (300, 190), 1, 8)

    assert estimate_single_yellow_boundary(mask) is None


def test_bird_eye_horizontal_white_bar_is_stop_line():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(image, (130, 330), (510, 350), (255, 255, 255), -1)

    detected, box = detect_stop_line(image)

    assert detected
    assert box is not None


def test_vertical_white_lane_is_not_stop_line():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(image, (300, 220), (320, 440), (255, 255, 255), -1)

    detected, _ = detect_stop_line(image)

    assert not detected


def test_red_traffic_lamp_in_upper_image():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.circle(image, (320, 90), 12, (0, 0, 255), -1)

    state, box = detect_traffic_light(image)

    assert state == 'RED'
    assert box is not None
