#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge

class StopLineDetector:
    def __init__(self):
        self.bridge = CvBridge()

        # --- test.py와 동일한 구독자 ---
        self.image_sub = rospy.Subscriber(
            "/usb_cam/image_raw", Image, self.image_callback,
            queue_size=1, buff_size=2**24
        )

        # --- 퍼블리셔 ---
        self.stop_line_pub = rospy.Publisher("/stop_line", Bool, queue_size=1)
        self.debug_img_pub = rospy.Publisher("/stop_line/debug_image", Image, queue_size=1)

        # --- 파라미터(필요 시 rosparam으로 조정) ---
        self.roi_y0, self.roi_y1 = rospy.get_param("~roi_y0", 0), rospy.get_param("~roi_y1", 360)
        self.roi_x0, self.roi_x1 = rospy.get_param("~roi_x0", 100), rospy.get_param("~roi_x1", 540)

        self.white_lower = np.array(rospy.get_param("~white_lower", [0, 0, 100]), dtype=np.uint8)
        self.white_upper = np.array(rospy.get_param("~white_upper", [180, 105,225]), dtype=np.uint8)

        # self.white_lower = np.array(rospy.get_param("~white_lower", [0, 5, 140]), dtype=np.uint8)
        # self.white_upper = np.array(rospy.get_param("~white_upper", [100, 105, 225]), dtype=np.uint8)

        self.canny_low  = rospy.get_param("~canny_low", 50)
        self.canny_high = rospy.get_param("~canny_high", 150)

        self.hough_rho        = rospy.get_param("~hough_rho", 1)
        self.hough_theta_deg  = rospy.get_param("~hough_theta_deg", 1.0)
        self.hough_threshold  = rospy.get_param("~hough_threshold", 100)
        self.min_line_length  = rospy.get_param("~min_line_length", 100)
        self.max_line_gap     = rospy.get_param("~max_line_gap", 10)

        self.angle_tol   = rospy.get_param("~angle_tol", 2)     # 수평선 각도 허용 오차(도)
        self.dx_dy_ratio = rospy.get_param("~dx_dy_ratio", 4)    # 가로/세로 비율 필터

    # --- image_callback은 구조 동일 ---
    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("cv_bridge error: %s", e)
            return

        h, w = frame.shape[:2]
        y0, y1 = np.clip(self.roi_y0, 0, h), np.clip(self.roi_y1, 0, h)
        x0, x1 = np.clip(self.roi_x0, 0, w), np.clip(self.roi_x1, 0, w)
        if y1 <= y0 or x1 <= x0:
            rospy.logwarn("Invalid ROI (%d:%d, %d:%d); skip.", y0, y1, x0, x1)
            return

        roi = frame[y0:y1, x0:x1].copy()

        # 1) HSV 흰색 마스크
        hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.white_lower, self.white_upper)

        # 2) Canny edge
        edges = cv2.Canny(mask, self.canny_low, self.canny_high)

        # 3) HoughLinesP
        theta = np.deg2rad(self.hough_theta_deg)
        lines = cv2.HoughLinesP(edges,
                                rho=self.hough_rho,
                                theta=theta,
                                threshold=self.hough_threshold,
                                minLineLength=self.min_line_length,
                                maxLineGap=self.max_line_gap)

        # 디버그용 ROI/최종 프레임 생성
        debug_roi  = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        final_frame = frame.copy()

        detected = False
        if lines is not None:
            for x1, y1_, x2, y2_ in lines[:, 0]:
                dx, dy = abs(x2 - x1), abs(y2_ - y1_)
                angle = abs(np.degrees(np.arctan2((y2_ - y1_), (x2 - x1))))

                # 수평선 필터
                if not (angle <= self.angle_tol or angle >= (180 - self.angle_tol)):
                    continue
                # 가로/세로 비율
                if dy > 0 and dx < self.dx_dy_ratio * dy:
                    continue
                # 길이 필터
                if np.hypot(dx, dy) < self.min_line_length:
                    continue

                # --- ROI 좌표계에서 박스 ---
                x_min, x_max = min(x1, x2), max(x1, x2)
                y_min, y_max = min(y1_, y2_), max(y1_, y2_)

                # ROI 디버그(마스크 3채널화)에 빨간 네모
                cv2.rectangle(debug_roi, (x_min, y_min), (x_max, y_max), (0, 0, 255), 2)

                # --- 최종 프레임(원본 좌표)에도 빨간 네모 ---
                cv2.rectangle(final_frame,
                              (x0 + x_min, y0 + y_min),
                              (x0 + x_max, y0 + y_max),
                              (0, 0, 255), 2)

                detected = True

        # --- stop_line Bool 퍼블리시 ---
        self.stop_line_pub.publish(Bool(data=detected))

        # --- 디버그 이미지 퍼블리시(최종 프레임) ---
        self.debug_img_pub.publish(self.bridge.cv2_to_imgmsg(final_frame, "bgr8"))

        # --- imshow 창들 ---
        cv2.imshow("ROI", roi)
        cv2.imshow("Mask", mask)
        cv2.imshow("Edges", edges)
        cv2.imshow("Final (with red boxes)", final_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("User pressed q")

def main():
    rospy.init_node("stop_line_detector", anonymous=False)
    StopLineDetector()
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
