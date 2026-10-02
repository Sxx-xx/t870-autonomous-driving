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
        rospy.init_node('stop_line_detector')
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/stop_line', Bool, queue_size=1)
        rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback, queue_size=1)
        rospy.loginfo("StopLineDetector 노드 시작")
        rospy.spin()

    def image_callback(self, img_msg):
        frame = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(blur, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50,
                                minLineLength=100, maxLineGap=10)
        detected = False
        if lines is not None:
            for l in lines:
                x1, y1, x2, y2 = l[0]
                angle = abs(np.arctan2(y2-y1, x2-x1) * 180. / np.pi)
                if angle < 10:
                    detected = True
                    break
        self.pub.publish(Bool(detected))
        if detected:
            rospy.logdebug("정지선 검출!")

if __name__ == '__main__':
    try:
        StopLineDetector()
    except rospy.ROSInterruptException:
        pass
