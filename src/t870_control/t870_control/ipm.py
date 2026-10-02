#!/usr/bin/env python3
# -*- coding: utf-8 -*-


# from screeninfo import get_monitors
# import cv2
# import multiprocessing
# import os

# def list_monitors():
#     """
#     연결된 모든 모니터의 정보를 출력합니다.
#     """
#     monitors = get_monitors()
#     print(f"연결된 모니터 개수: {len(monitors)}\n")
#     for idx, m in enumerate(monitors):
#         print(f"모니터 {idx}:")
#         print(f"  해상도: {m.width}x{m.height}")
#         print(f"  위치: ({m.x}, {m.y})")
#         print(f"  이름: {m.name}\n")
#     return monitors

# def play_video_on_monitor(video_path, monitor_index=0, window_name="Test Video"):
#     """
#     지정된 모니터에서 영상을 풀스크린으로 재생합니다.
    
#     :param video_path: 재생할 비디오 파일의 경로
#     :param monitor_index: 영상을 표시할 모니터의 인덱스 (기본값: 0)
#     :param window_name: OpenCV 창의 이름 (고유해야 함)
#     """
#     monitors = get_monitors()
    
#     if monitor_index >= len(monitors) or monitor_index < 0:
#         print(f"선택한 모니터 인덱스 {monitor_index}가 범위를 벗어났습니다. 기본 모니터 0을 사용합니다.")
#         selected = monitors[0]
#         selected_index = 0
#     else:
#         selected = monitors[monitor_index]
#         selected_index = monitor_index
    
#     monitor_x = selected.x
#     monitor_y = selected.y
#     monitor_width = selected.width
#     monitor_height = selected.height

#     print(f"모니터 {selected_index}에 '{window_name}' 영상을 풀스크린으로 표시합니다: 해상도 {monitor_width}x{monitor_height}, 위치 ({monitor_x}, {monitor_y})")

#     # 비디오 파일 열기
#     if not os.path.exists(video_path):
#         print(f"비디오 파일이 존재하지 않습니다: {video_path}")
#         return

#     cap = cv2.VideoCapture(video_path)
#     if not cap.isOpened():
#         print(f"비디오 파일을 열 수 없습니다: {video_path}")
#         return

#     # 창 생성 및 위치 설정
#     cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
#     cv2.moveWindow(window_name, monitor_x, monitor_y)

#     # 풀스크린으로 전환
#     cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             # 비디오가 끝나면 처음부터 다시 시작
#             cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
#             ret, frame = cap.read()
#             if not ret:
#                 print("비디오 프레임을 읽을 수 없습니다.")
#                 break
#         cv2.imshow(window_name, frame)

#         # 'q' 키를 누르면 영상 재생 종료
#         if cv2.waitKey(30) & 0xFF == ord('q'):
#             break

#     cap.release()
#     cv2.destroyWindow(window_name)

# if __name__ == "__main__":
#     monitors = list_monitors()
    
#     front_video_file = "/home/navigator/catkin_ws/src/erp42_control_ob/path/front_1.mp4"  # 재생할 비디오 파일의 실제 경로
#     back_video_file = "/home/navigator/catkin_ws/src/erp42_control_ob/path/back_1.mp4"
    
#     front_monitor_index = 0  # 모니터 0: HDMI-0
#     back_monitor_index = 1   # 모니터 1: DP-4
    
#     # 비디오 파일 존재 여부 확인
#     import sys
#     if not os.path.exists(front_video_file):
#         print(f"프론트 비디오 파일이 존재하지 않습니다: {front_video_file}")
#         sys.exit(1)
#     if not os.path.exists(back_video_file):
#         print(f"백 비디오 파일이 존재하지 않습니다: {back_video_file}")
#         sys.exit(1)
    
#     # 멀티프로세스 사용
#     front_process = multiprocessing.Process(target=play_video_on_monitor, args=(front_video_file, front_monitor_index, "Front Video"))
#     back_process = multiprocessing.Process(target=play_video_on_monitor, args=(back_video_file, back_monitor_index, "Back Video"))
    
#     front_process.start()
#     back_process.start()
    
#     front_process.join()
#     back_process.join()


#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from control_msgs.msg import Velocity
from nav_msgs.msg import Odometry
from ublox_msgs.msg import NavPVT

class WaypointPublisher:
    def __init__(self):
        rospy.init_node('waypoint_publisher', anonymous=True)
        
        # 웨이포인트 퍼블리셔
        self.waypoint_pub = rospy.Publisher('/current_waypoint', Int32, queue_size=10)
        
        # 속도 퍼블리셔
        self.velocity_pub = rospy.Publisher('/ERP42_velocity', Velocity, queue_size=10)
        
        # Odometry 퍼블리셔 (실험용 데이터)
        #self.odom_pub = rospy.Publisher('/odom/filtered', Odometry, queue_size=10)
        
        # NavPVT 퍼블리셔 (실험용 데이터)
        #self.navpvt_pub = rospy.Publisher('/ublox/navpvt', NavPVT, queue_size=10)
        
        # 초기 웨이포인트
        self.current_waypoint = 0
        
        # 고정 속도
        self.fixed_velocity = 150
        
        # 주기 설정: 1초당 5번 (0.2초 간격)
        self.rate = rospy.Rate(5)  # 5Hz

    def publish_data(self):
        while not rospy.is_shutdown():
            # 웨이포인트 증가
            self.current_waypoint += 1
            
            # 웨이포인트 메시지 생성
            waypoint_msg = Int32()
            waypoint_msg.data = self.current_waypoint
            
            # 속도 메시지 생성
            velocity_msg = Velocity()
            velocity_msg.velocity = self.fixed_velocity
            
            # # Odometry 메시지 생성 (실험용)
            # odom_msg = Odometry()
            # odom_msg.header.stamp = rospy.Time.now()
            # odom_msg.header.frame_id = "odom"
            # odom_msg.pose.pose.position.x += 0.1  # 임의의 변화
            # odom_msg.pose.pose.position.y += 0.1
            # odom_msg.pose.pose.orientation.w = 1.0  # 정면
            
    
            
            # 퍼블리시
            self.waypoint_pub.publish(waypoint_msg)
            self.velocity_pub.publish(velocity_msg)
            #self.odom_pub.publish(odom_msg)
            #self.navpvt_pub.publish(navpvt_msg)
            
            rospy.loginfo(f"Published waypoint: {self.current_waypoint}, velocity: {self.fixed_velocity}")
            
            # 주기 대기
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = WaypointPublisher()
        publisher.publish_data()
    except rospy.ROSInterruptException:
        pass
