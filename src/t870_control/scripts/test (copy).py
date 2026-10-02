#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import torch
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import time
from torchvision import transforms
from PIL import Image as PILImage
import numpy as np
from model.lanenet.LaneNet import LaneNet  # 적절한 모델 임포트

# GPU 사용 가능 여부 확인
DEVICE = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
print(DEVICE)

class LaneDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.resize_height = 480
        self.resize_width = 640

        # 모델 불러오기
        self.model = LaneNet(arch='DeepLabv3+')
        self.model.load_state_dict(
            torch.load(
                '/home/navigator/catkin_ws/src/erp42_control_ob/scripts/log/lanenet_DeepLabv3+_Focal_epoch100_batchsize8.pth',
                map_location=torch.device('cpu')  # GPU 대신 CPU로 매핑
            )
        )
        self.model.eval()
        self.model.to(DEVICE)

        # 전처리 transform
        self.data_transform = transforms.Compose([
            transforms.Resize((self.resize_height, self.resize_width)),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ])

        # ROS 토픽 설정
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size=1, buff_size=2**24)
        self.image_pub = rospy.Publisher("/lane_detection/output", Image, queue_size=1)
        self.ipm_pub = rospy.Publisher("/ipm_image", Image, queue_size=1)
        self.lane_pub = rospy.Publisher("/lane_image", Image, queue_size=1)
        self.center_points_pub = rospy.Publisher("/lane_center_points", Point, queue_size=1)

        # IPM용 포인트 설정
        self.height = self.resize_height
        self.width = self.resize_width

        self.src_points = np.float32([
            [0, self.height],
            [self.width, self.height],
            [self.width * 0.25, self.height * 0.6],
            [self.width * 0.75, self.height * 0.6]
        ])
        self.dst_points = np.float32([
            [self.width * 0.25, self.height],
            [self.width * 0.75, self.height],
            [0, 0],
            [self.width, 0]
        ])

        # IPM 매트릭스
        self.M = cv2.getPerspectiveTransform(self.src_points, self.dst_points)

    def load_test_data(self, img, transform):
        img = PILImage.fromarray(img)
        img = transform(img)
        return img

    def apply_ipm(self, image):
        return cv2.warpPerspective(image, self.M, (self.width, self.height), flags=cv2.INTER_LINEAR)

    def sliding_window(self, binary_warped):
        histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
        out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
        midpoint = int(histogram.shape[0]/2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        nwindows = 10
        window_height = int(binary_warped.shape[0]/nwindows)
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        leftx_current = leftx_base
        rightx_current = rightx_base
        margin = 50
        minpix = 10
        left_lane_inds = []
        right_lane_inds = []
        center_points = []

        for window in range(nwindows):
            win_y_low = binary_warped.shape[0] - (window+1)*window_height
            win_y_high = binary_warped.shape[0] - window*window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

            cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0,255,0), 2)
            cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0,255,0), 2)

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

            center_x = (leftx_current + rightx_current) // 2
            center_y = (win_y_low + win_y_high) // 2
            center_points.append((center_x, center_y))

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        out_img[lefty, leftx] = [255, 0, 0]
        out_img[righty, rightx] = [0, 0, 255]

        for point in center_points:
            cv2.circle(out_img, point, 5, (0, 255, 0), -1)

        return out_img, center_points

    def process_frame(self, frame):
        input_img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        input_img = PILImage.fromarray(input_img)
        input_img = input_img.resize((self.width, self.height))
        input_np = np.array(input_img)

        dummy_input = self.load_test_data(input_np, self.data_transform).to(DEVICE)
        dummy_input = torch.unsqueeze(dummy_input, dim=0)
        outputs = self.model(dummy_input)

        binary_pred = torch.squeeze(outputs['binary_seg_pred']).to('cpu').numpy()
        binary_pred = (binary_pred * 255).astype(np.uint8)

        ipm_binary = self.apply_ipm(binary_pred)
        lane_image, center_points = self.sliding_window(ipm_binary)

        binary_colormap = cv2.applyColorMap(ipm_binary, cv2.COLORMAP_JET)
        original_img = self.apply_ipm(frame)
        overlay = cv2.addWeighted(original_img, 0.6, lane_image, 0.4, 0)

        for point in center_points:
            center_point_msg = Point(x=point[0], y=point[1], z=0)
            self.center_points_pub.publish(center_point_msg)

        return original_img, binary_colormap, lane_image, overlay

    def image_callback(self, msg):
        try:
            start_time = time.time()

            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            original_img, binary_img, lane_img, result_img = self.process_frame(cv_image)

            self.image_pub.publish(self.bridge.cv2_to_imgmsg(result_img, "bgr8"))
            self.ipm_pub.publish(self.bridge.cv2_to_imgmsg(original_img, "bgr8"))
            self.lane_pub.publish(self.bridge.cv2_to_imgmsg(lane_img, "bgr8"))

            cv2.imshow('Original IPM Image', original_img)
            cv2.imshow('Binary Image', binary_img)
            cv2.imshow('Sliding Window Result', lane_img)
            cv2.imshow('Final Result', result_img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown('User pressed Q')

            end_time = time.time()
            fps = 1 / (end_time - start_time)
            rospy.loginfo(f"FPS: {fps:.2f}")

        except Exception as e:
            rospy.logerr(e)

    def main(self):
        rospy.init_node('lane_detector', anonymous=False)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        finally:
            cv2.destroyAllWindows()

if __name__ == "__main__":
    lane_detector = LaneDetector()
    lane_detector.main()
