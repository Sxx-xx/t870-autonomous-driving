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
        self.stop_line_pub = rospy.Publisher("/stop_line", Bool, queue_size=1)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size=1, buff_size=2**24)

        # ROI & 조건 설정
        self.resize_height = 480
        self.resize_width = 640
        self.stop_length_thresh = 100

    def detect_stop_line(self, frame):
        # 1) 관심영역(ROI)
        roi = frame[0:480, 100:540]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # 2) 흰색 필터링
        white_lower = np.array([0, 0, 120])
        white_upper = np.array([190, 105, 255])
        mask = cv2.inRange(hsv, white_lower, white_upper)

        # 3) Canny 에지
        edges = cv2.Canny(mask, 50, 150)

        # 4) 허프 직선 검출
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi/180,
            threshold=100,
            minLineLength=self.stop_length_thresh,
            maxLineGap=10
        )

        detected = False
        if lines is not None:
            for x1, y1, x2, y2 in lines[:,0]:
                dx = abs(x2 - x1)
                dy = abs(y2 - y1)
                angle = abs(np.degrees(np.arctan2((y2 - y1), (x2 - x1))))

                # 가로선만 필터링
                if not (-10 <= angle <= 10 or 170 <= angle <= 190):
                    continue
                if dx < 1.5 * dy:  # 길이 비율
                    continue
                length = np.hypot(dx, dy)
                if length < self.stop_length_thresh:
                    continue

                detected = True
                cv2.rectangle(roi, (min(x1,x2), min(y1,y2)), (max(x1,x2), max(y1,y2)), (0,0,255), 2)

        # 디버깅 시각화
        cv2.imshow("ROI_with_Box", roi)
        cv2.imshow("Mask", mask)
        cv2.imshow("Edges", edges)
        cv2.waitKey(1)

        return detected

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            stop_line_detected = self.detect_stop_line(cv_image)
            self.stop_line_pub.publish(Bool(data=stop_line_detected))

            if stop_line_detected:
                rospy.loginfo("✅ 정지선 감지됨")
            else:
                rospy.loginfo("❌ 정지선 없음")
        except Exception as e:
            rospy.logerr(e)

    def main(self):
        rospy.init_node("stop_line_detector", anonymous=False)
        rospy.loginfo("StopLineDetector 노드 시작")
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down")
        finally:
            cv2.destroyAllWindows()

if __name__ == "__main__":
    detector = StopLineDetector()
    detector.main()
