#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# import sys
# import os
# import rospy
# import rospkg
# import math
# import time
# import serial
# import pygame
# import cv2
import threading
# from screeninfo import get_monitors  # Added import

# from nav_msgs.msg import Path, Odometry
# from std_msgs.msg import Float64, Int16, Float32MultiArray
# from control_msgs.msg import Velocity, Gear
# from geometry_msgs.msg import PoseStamped, Point, Pose, PoseArray, TwistWithCovarianceStamped
# from morai_msgs.msg import CtrlCmd, LocalControl
# from ublox_msgs.msg import NavPVT
# from lib.utils_main import pathReader, findLocalPath, purePursuit, pidController

# class ERPPlanner:
#     def __init__(self):
#         rospy.init_node('ERP42_planner')
#         self.setup_parameters()
#         self.setup_publishers()
#         self.setup_path()
#         self.setup_subscribers()
#         self.setup_music()
#         self.setup_video()

#         self.rate = rospy.Rate(20)  # 40Hz

#         self.brake_start_time = None
#         self.brake_duration = rospy.Duration(10)  # 10초 브레이크 지속 시간
#         self.accel_start_time = None
#         self.accel_duration = rospy.Duration(1)   # 1초 가속 지속 시간
#         self.state = "DRIVING"
#         self.waypoint_81_reached = False
#         self.brake_applied = False

#         self.run()

#     def setup_parameters(self):
#         self.global_path_name = '24_kcity_siyeon'
#         self.path_name = '24_kcity_siyeon'
#         self.ctrl_msg = CtrlCmd()
#         self.pose_msg = Odometry()
#         self.curvel_msg = Velocity()
#         self.yaw_msg = NavPVT()
#         self.current_waypoint = 0
#         self.global_mode = True
#         self.yaw_file_path = '/home/navigator/yaw.txt'
#         self.local_path = Path()

#     def setup_publishers(self):
#         self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
#         self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
#         self.ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
#         self.local_control_pub = rospy.Publisher('/local_control', LocalControl, queue_size=1)

#     def setup_subscribers(self):
#         rospy.Subscriber("/odom/filtered", Odometry, self.pose_callback)
#         rospy.Subscriber("/ERP42_velocity", Velocity, self.velocity_callback)
#         rospy.Subscriber("/ublox/navpvt", NavPVT, self.yaw_callback)

#     def setup_path(self):
#         self.path_reader = pathReader('erp42_control_ob')
#         self.global_path = self.path_reader.read_txt(f"{self.path_name}.txt")
#         self.pure_pursuit = purePursuit()
#         self.pid = pidController()

#     def setup_music(self):
#         pygame.mixer.init()
#         pygame.mixer.set_num_channels(2)  # Allocate two channels: one for music, one for warning sound

#         # Background music files
#         self.music_files = {
#             'driving': '/home/navigator/catkin_ws/src/erp42_control_ob/path/driving.mp3',
#             'driving_warning': '/home/navigator/catkin_ws/src/erp42_control_ob/path/driving+warning.mp3'
#         }

#         # Warning sound file
#         self.sound_files = {
#             'warning_sound': '/home/navigator/catkin_ws/src/erp42_control_ob/path/warning.mp3'  # Ensure this file exists
#         }

#         self.current_music = None
#         self.warning_sound_channel = pygame.mixer.Channel(1)  # Channel 1 for warning sound

#     def setup_video(self):
#         self.video_files = {
#             'warning': '/home/navigator/catkin_ws/src/erp42_control_ob/path/front_siyeon.mp4'  # 실제 경고 비디오 파일 경로로 변경해야 합니다
#         }
#         self.current_video = None
#         self.video_thread = None
#         self.terminate_video_flag = threading.Event()
        
#         # Monitor selection
#         self.selected_monitor_index = 0  # Change this to the desired monitor index (0-based)
#         self.monitor = self.get_monitor(self.selected_monitor_index)
#         if self.monitor is None:
#             rospy.logwarn(f"Monitor index {self.selected_monitor_index} not found. Using primary monitor.")
#             self.monitor = get_monitors()[0]
#         rospy.loginfo(f"Selected monitor: {self.monitor.name}, Resolution: {self.monitor.width}x{self.monitor.height}, Position: ({self.monitor.x}, {self.monitor.y})")

#     def get_monitor(self, index):
#         monitors = get_monitors()
#         if index < len(monitors):
#             return monitors[index]
#         else:
#             return None

#     def run(self):
#         while not rospy.is_shutdown():
#             try:
#                 self.process_path()
#                 self.publish_messages()
#                 self.manage_music()
#                 self.manage_video()
#                 self.rate.sleep()
#             except rospy.ROSInterruptException:
#                 rospy.loginfo("Node interrupted")
#                 break
#             except Exception as e:
#                 rospy.logerr(f"An error occurred: {e}")

#     def process_path(self):
#         local_path, self.current_waypoint = findLocalPath(self.global_path, self.pose_msg, self.current_waypoint)
#         self.local_path = local_path
#         target_velocity = self.global_path.poses[self.current_waypoint].pose.position.z

#         control_input = self.pid.pid(target_velocity, self.curvel_msg.velocity)

#         self.pure_pursuit.getPath(local_path)
#         self.pure_pursuit.getPoseStatus(self.pose_msg)
#         self.pure_pursuit.getVelStatus(self.curvel_msg)
#         self.pure_pursuit.getYawStatus(self.yaw_msg)

#         self.ctrl_msg.seq = self.pose_msg.header.seq
#         self.ctrl_msg.steering, _, self.target_angle_index = self.pure_pursuit.steering_angle(self.current_waypoint)

#         self.control_velocity(control_input, target_velocity)

#     def control_velocity(self, control_input, target_velocity):
#         current_time = rospy.Time.now()

#         if self.state == "BRAKING" and (current_time - self.brake_start_time < self.brake_duration):
#             self.apply_brake()
#             elapsed_time = (current_time - self.brake_start_time).to_sec()
#             rospy.loginfo(f"Applying brake for {elapsed_time:.2f} seconds")
#         elif self.state == "BRAKING" and (current_time - self.brake_start_time >= self.brake_duration):
#             self.state = "ACCELERATING"
#             self.accel_start_time = current_time
#             rospy.loginfo("Brake time completed, starting acceleration")
#             self.apply_acceleration(control_input)  # Use PID control input for acceleration
#         elif self.state == "ACCELERATING" and (current_time - self.accel_start_time < self.accel_duration):
#             self.apply_acceleration(control_input)  # Continue using PID control input
#             elapsed_time = (current_time - self.accel_start_time).to_sec()
#             rospy.loginfo(f"Applying acceleration for {elapsed_time:.2f} seconds")
#         elif self.state == "ACCELERATING" and (current_time - self.accel_start_time >= self.accel_duration):
#             self.resume_normal_driving()
#             self.apply_control_input(control_input, target_velocity)
#         else:
#             if self.current_waypoint == 170 and not self.waypoint_81_reached:
#                 self.waypoint_81_reached = True
#                 self.brake_start_time = current_time
#                 self.state = "BRAKING"
#                 rospy.loginfo("Started braking at waypoint 170")
#                 self.apply_brake()
#             else:
#                 self.apply_control_input(control_input, target_velocity)

#     def apply_brake(self):
#         self.ctrl_msg.accel = 0
#         self.ctrl_msg.brake = 200
#         self.ctrl_msg.steering = 0.0

#     def apply_acceleration(self, control_input):
#         self.ctrl_msg.accel = control_input  # Use PID controller output
#         self.ctrl_msg.brake = 0
#         self.ctrl_msg.steering = 0.0

#     def resume_normal_driving(self):
#         self.state = "DRIVING"
#         self.brake_applied = True
#         rospy.loginfo("Acceleration time completed, resuming normal driving")

#     def apply_control_input(self, control_input, target_velocity):
#         if 0 < control_input <= 200:
#             self.ctrl_msg.accel = control_input
#             self.ctrl_msg.brake = 0
#         elif control_input > 200:
#             self.ctrl_msg.accel = target_velocity
#             self.ctrl_msg.brake = 0
#         elif control_input < -200:
#             self.ctrl_msg.accel = 0
#             self.ctrl_msg.brake = 200
#         else:
#             self.ctrl_msg.accel = 0
#             self.ctrl_msg.brake = -control_input

#     def publish_messages(self):
#         self.ctrl_pub.publish(self.ctrl_msg)
#         self.global_path_pub.publish(self.global_path)
#         self.local_path_pub.publish(self.local_path)

#         local_control_msg = LocalControl()
#         local_control_msg.seq = self.pose_msg.header.seq
#         local_control_msg.gps_x = self.pose_msg.pose.pose.position.x
#         local_control_msg.gps_y = self.pose_msg.pose.pose.position.y
#         local_control_msg.gps_z = self.pose_msg.pose.pose.position.z
#         local_control_msg.velocity = self.curvel_msg.velocity / 10
#         local_control_msg.waypoint_size = len(self.global_path.poses)
#         local_control_msg.cur_waypoint = self.current_waypoint
#         self.local_control_pub.publish(local_control_msg)

#     def pose_callback(self, data):
#         self.pose_msg = data

#     def velocity_callback(self, speed_data):
#         self.curvel_msg = speed_data

#     def yaw_callback(self, yaw_data):
#         self.yaw_msg = yaw_data
#         self.ctrl_pub.publish(self.ctrl_msg)

#         current_yaw = self.yaw_msg.heading * 1e-5
#         target_yaw = math.atan2(
#             self.global_path.poses[self.target_angle_index].pose.position.y - self.pose_msg.pose.pose.position.y,
#             self.global_path.poses[self.target_angle_index].pose.position.x - self.pose_msg.pose.pose.position.x
#         )
#         target_yaw = math.degrees(target_yaw)

#         try:
#             with open(self.yaw_file_path, 'w') as f:
#                 f.write(f"Target Yaw: {target_yaw:.2f}\n")
#                 f.write(f"Current Yaw: {current_yaw:.2f}\n")
#         except IOError as e:
#             rospy.logerr(f"Error writing to yaw file: {e}")

#     def play_music(self, music_type):
#         if self.current_music != music_type:
#             try:
#                 pygame.mixer.music.load(self.music_files[music_type])
#                 pygame.mixer.music.play(-1)  # Loop indefinitely
#                 self.current_music = music_type
#                 rospy.loginfo(f"Playing music: {music_type}")
#             except Exception as e:
#                 rospy.logerr(f"Error playing music: {e}")

#     def stop_music(self):
#         if pygame.mixer.music.get_busy():
#             pygame.mixer.music.stop()
#             self.current_music = None
#             rospy.loginfo("Music stopped")

#     def start_music(self, music_type):
#         if self.current_music != music_type:
#             self.stop_music()
#             self.play_music(music_type)

#     def manage_music(self):
#         if self.state == "BRAKING":
#             self.stop_music()            # Stop background music
#             self.play_warning_sound()    # Play warning sound
#         elif 150 <= self.current_waypoint <= 169:
#             self.start_music('driving_warning')
#             self.stop_warning_sound()
#         elif self.state == "ACCELERATING":
#             self.start_music('driving')
#             self.stop_warning_sound()
#         elif 0 <= self.current_waypoint <= 149:
#             self.start_music('driving')
#             self.stop_warning_sound()
#         elif (self.current_waypoint >= 173 or self.brake_applied):
#             self.start_music('driving')
#             self.stop_warning_sound()
#         else:
#             self.stop_music()
#             self.stop_warning_sound()

#     def play_warning_sound(self):
#         if not self.warning_sound_channel.get_busy():
#             try:
#                 warning_sound = pygame.mixer.Sound(self.sound_files['warning_sound'])
#                 self.warning_sound_channel.play(warning_sound, loops=-1)  # Loop indefinitely
#                 rospy.loginfo("Playing warning sound")
#             except Exception as e:
#                 rospy.logerr(f"Error playing warning sound: {e}")

#     def stop_warning_sound(self):
#         if self.warning_sound_channel.get_busy():
#             self.warning_sound_channel.stop()
#             rospy.loginfo("Stopped warning sound")

#     def play_video(self, video_file):
#         def video_player():
#             # Open video file
#             cap = cv2.VideoCapture(video_file)
            
#             # Set the window for full screen
#             window_name = "Warning Video"
#             cv2.namedWindow(window_name, cv2.WND_PROP_FULLSCREEN)
#             cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            
#             # Move the window to the selected monitor's position
#             cv2.moveWindow(window_name, self.monitor.x, self.monitor.y)

#             while not self.terminate_video_flag.is_set():
#                 ret, frame = cap.read()
#                 if not ret:
#                     cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Restart video if it ends
#                     continue
#                 cv2.imshow(window_name, frame)

#                 if cv2.waitKey(30) & 0xFF == ord('q'):
#                     break

#             cap.release()
#             cv2.destroyAllWindows()

#         self.terminate_video_flag.clear()
#         self.video_thread = threading.Thread(target=video_player)
#         self.video_thread.start()

#     def terminate_video(self):
#         if self.video_thread and self.video_thread.is_alive():
#             self.terminate_video_flag.set()
#             self.video_thread.join()
#             self.current_video = None
#         cv2.destroyAllWindows()

#     def start_video(self, video_type):
#         if self.current_video != video_type:
#             self.terminate_video()
#             self.play_video(self.video_files[video_type])
#             self.current_video = video_type
#             rospy.loginfo(f"Started playing video: {video_type}")

#     def manage_video(self):
#         if self.state == "BRAKING":
#             if self.current_video != 'warning':
#                 self.start_video('warning')  # Play warning video during braking
#         else:
#             if self.current_video is not None:
#                 self.terminate_video()  # Stop video when not braking

# if __name__ == '__main__':
#     try:
#         kcity_pathtracking = ERPPlanner()
#     except rospy.ROSInterruptException:
#         pass
#!/usr/bin/env python3

import sys
import os
import rospy
import math
import time
import cv2
import multiprocessing
from screeninfo import get_monitors
import threading

from nav_msgs.msg import Path, Odometry
from control_msgs.msg import Velocity
from morai_msgs.msg import CtrlCmd, LocalControl
from ublox_msgs.msg import NavPVT
from std_msgs.msg import Int32  # 웨이포인트 퍼블리시를 위한 메시지 타입
from geometry_msgs.msg import Pose
from lib.utils_main import pathReader, findLocalPath, purePursuit, pidController

import pygame  # 소리 관련 라이브러리 추가

def video_player(video_file, monitor, window_name, terminate_event):
    """
    비디오를 지정된 모니터에서 풀스크린으로 재생하는 함수.

    :param video_file: 재생할 비디오 파일의 경로
    :param monitor: screeninfo로 가져온 모니터 객체
    :param window_name: OpenCV 창의 이름
    :param terminate_event: 재생 종료를 위한 이벤트 객체
    """
    try:
        cap = cv2.VideoCapture(video_file)
        if not cap.isOpened():
            rospy.logerr(f"비디오 열기 실패: {video_file}")
            return

        # 비디오의 FPS 가져오기
        fps = cap.get(cv2.CAP_PROP_FPS)
        if fps > 0:
            delay = int(1000 / fps)
        else:
            rospy.logwarn(f"비디오 FPS를 가져올 수 없습니다. 기본 지연 시간 30ms 사용.")
            delay = 30  # 기본 지연 시간 설정

        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.moveWindow(window_name, monitor.x, monitor.y)
        cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        rospy.loginfo(f"비디오 재생 시작 - {window_name} at ({monitor.x}, {monitor.y}), FPS: {fps}, Delay: {delay}ms")

        while not terminate_event.is_set() and not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                # 비디오가 끝나면 처음부터 다시 시작
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = cap.read()
                if not ret:
                    rospy.logerr("비디오 프레임을 읽을 수 없습니다.")
                    break

            # 프레임을 모니터 해상도에 맞게 리사이즈
            frame = cv2.resize(frame, (monitor.width, monitor.height))
            cv2.imshow(window_name, frame)

            # 설정된 지연 시간만큼 대기
            key = cv2.waitKey(delay) & 0xFF
            if key == 27:  # 'ESC' 키 입력 시 재생 종료
                rospy.loginfo(f"'{window_name}' 영상 재생 종료 (ESC 키 입력)")
                terminate_event.set()

    except Exception as e:
        rospy.logerr(f"비디오 재생 에러: {e}")
    finally:
        if 'cap' in locals():
            cap.release()
        cv2.destroyWindow(window_name)
        rospy.loginfo(f"비디오 재생 종료 - {window_name}")

def sound_player(sound_file, terminate_event):
    """
    소리를 재생하는 함수.

    :param sound_file: 재생할 소리 파일의 경로
    :param terminate_event: 재생 종료를 위한 이벤트 객체
    """
    try:
        if not os.path.exists(sound_file):
            rospy.logerr(f"소리 파일 없음: {sound_file}")
            return

        pygame.mixer.music.load(sound_file)
        pygame.mixer.music.play(-1)  # 무한 반복 재생

        rospy.loginfo(f"소리 재생 시작: {sound_file}")

        while not terminate_event.is_set() and pygame.mixer.music.get_busy() and not rospy.is_shutdown():
            time.sleep(0.1)  # 소리 재생 중 대기

    except Exception as e:
        rospy.logerr(f"소리 재생 에러: {e}")
    finally:
        pygame.mixer.music.stop()
        rospy.loginfo(f"소리 재생 종료: {sound_file}")

class ERPPlanner:
    def __init__(self):
        rospy.init_node('ERP42_planner')
        self.setup_parameters()
        self.setup_publishers()
        self.setup_path()
        self.setup_subscribers()
        self.setup_music()
        self.setup_video()

        self.rate = rospy.Rate(20)  # 20Hz

        self.brake_start_time = None
        self.brake_duration = rospy.Duration(24)  # Default 브레이크 지속 시간 (170이 아닐 경우)
        self.accel_start_time = None
        self.accel_duration = rospy.Duration(3)   # 1초 가속 지속 시간
        self.state = "DRIVING"
        self.waypoint_170_reached = False

        # 현재 재생 중인 비디오 및 소리 상태를 추적
        self.current_media_state = {
            'screen2': None,
            'screen3': None,
            'sound': None
        }

        self.video_processes = {
            'screen2': None,
            'screen3': None
        }
        self.sound_process = None

        self.terminate_video_flags = {
            'screen2': multiprocessing.Event(),
            'screen3': multiprocessing.Event()
        }
        self.terminate_sound_flag = multiprocessing.Event()

        self.run()

    def setup_parameters(self):
        self.global_path_name = '24_kcity_siyeon'
        self.path_name = '24_kcity_siyeon'
        self.ctrl_msg = CtrlCmd()
        self.pose_msg = Odometry()
        self.curvel_msg = Velocity()
        self.yaw_msg = NavPVT()
        self.current_waypoint = 0
        self.global_mode = True
        self.yaw_file_path = '/home/navigator/yaw.txt'
        self.local_path = Path()

    def setup_publishers(self):
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.local_control_pub = rospy.Publisher('/local_control', LocalControl, queue_size=1)

    def setup_subscribers(self):
        rospy.Subscriber("/odom/filtered", Odometry, self.pose_callback)
        rospy.Subscriber("/ERP42_velocity", Velocity, self.velocity_callback)
        rospy.Subscriber("/ublox/navpvt", NavPVT, self.yaw_callback)
        rospy.Subscriber("/current_waypoint", Int32, self.waypoint_callback)  # 새로운 웨이포인트 토픽 구독

    def setup_path(self):
        self.path_reader = pathReader('erp42_control_ob')
        self.global_path = self.path_reader.read_txt(f"{self.path_name}.txt")
        self.pure_pursuit = purePursuit()
        self.pid = pidController()

    def setup_music(self):
        pygame.mixer.init()
        rospy.loginfo("pygame mixer initialized for sound playback.")

        base_path = "/home/navigator/catkin_ws/src/erp42_control_ob/path"
        self.sound_files = {
            'sound_1': f"{base_path}/driving.mp3",
            'sound_2': f"{base_path}/driving+warning.mp3",
            'sound_3': f"{base_path}/warning.mp3"
        }

    def play_sound(self, sound_type):
        """소리 재생 함수"""
        try:
            if sound_type not in self.sound_files:
                rospy.logerr(f"정의되지 않은 소리 타입: {sound_type}")
                return

            desired_sound = self.sound_files[sound_type]

            if self.current_media_state['sound'] != desired_sound:
                # 기존 소리 스레드가 실행 중이면 종료
                self.terminate_sound()

                if desired_sound:
                    # 소리 재생 스레드 시작
                    self.sound_thread = threading.Thread(
                        target=sound_player,
                        args=(desired_sound, self.terminate_sound_flag),
                        daemon=True  # 메인 스레드 종료 시 자동 종료
                    )
                    self.sound_thread.start()
                    self.current_media_state['sound'] = desired_sound
                    rospy.loginfo(f"소리 스레드 시작: {desired_sound}")
        except Exception as e:
            rospy.logerr(f"소리 재생 중 에러 발생: {e}")


    def stop_sound(self):
        """소리 정지 함수"""
        try:
            if self.current_media_state['sound']:
                self.terminate_sound()
                self.current_media_state['sound'] = None
        except Exception as e:
            rospy.logerr(f"소리 정지 중 에러 발생: {e}")

    def terminate_sound(self):
        """소리 재생 종료 함수"""
        if self.sound_process and self.sound_process.is_alive():
            self.terminate_sound_flag.set()
            self.sound_process.join(timeout=1.0)
            if self.sound_process.is_alive():
                self.sound_process.terminate()
            rospy.loginfo("소리 프로세스 종료.")
            self.sound_process = None
            self.terminate_sound_flag.clear()

    def setup_video(self):
        # 모니터 정보 출력
        monitors = get_monitors()
        rospy.loginfo(f"검색된 모니터 수: {len(monitors)}")
        for i, m in enumerate(monitors):
            rospy.loginfo(f"모니터 {i}: {m.name}, 위치: ({m.x}, {m.y}), 해상도: {m.width}x{m.height}")
        
        # x 좌표로 정렬하여 왼쪽부터 순서대로 나열
        sorted_monitors = sorted(monitors, key=lambda m: m.x)
        
        # 메인 모니터(가운데)를 제외한 나머지 모니터 선택
        # 여기서는 가운데 모니터가 sorted_monitors[1]이라고 가정
        if len(sorted_monitors) < 3:
            rospy.logerr("충분한 수의 모니터가 감지되지 않았습니다. 최소 3개 이상의 모니터가 필요합니다.")
            sys.exit(1)
        
        side_monitors = [m for m in sorted_monitors if m != sorted_monitors[1]]  # 가운데 모니터 제외

        if len(side_monitors) < 2:
            rospy.logerr("충분한 수의 사이드 모니터가 감지되지 않았습니다.")
            sys.exit(1)
        
        self.video_files = {
            'screen2': '/home/navigator/catkin_ws/src/erp42_control_ob/path/front_1.mp4',
            'screen3': '/home/navigator/catkin_ws/src/erp42_control_ob/path/back_1.mp4'
        }
        
        # 비디오 모니터 매핑
        self.video_monitor_map = {
            'screen2': side_monitors[0],    # 왼쪽 모니터
            'screen3': side_monitors[1]     # 오른쪽 모니터
        }
        rospy.loginfo(f"screen2 (왼쪽) 모니터 위치: ({self.video_monitor_map['screen2'].x}, {self.video_monitor_map['screen2'].y})")
        rospy.loginfo(f"screen3 (오른쪽) 모니터 위치: ({self.video_monitor_map['screen3'].x}, {self.video_monitor_map['screen3'].y})")

    def play_video(self, screen, video_file):
        """비디오 재생 함수 (멀티프로세스 사용)"""
        desired_video = video_file

        # 원하는 비디오 파일이 현재 재생 중인 비디오와 다를 경우에만 재생
        if self.current_media_state[screen] != desired_video:
            # 기존 프로세스가 실행 중이면 종료
            self.terminate_video(screen)

            if desired_video:
                if not os.path.exists(desired_video):
                    rospy.logerr(f"비디오 파일 없음: {desired_video}")
                    return

                monitor = self.video_monitor_map.get(screen)
                if not monitor:
                    rospy.logerr(f"유효하지 않은 스크린: {screen}")
                    return

                window_name = f"Video_{screen}_{monitor.x}"

                # 종료 이벤트 초기화
                self.terminate_video_flags[screen].clear()

                # 비디오 재생 프로세스 시작
                process = multiprocessing.Process(
                    target=video_player,
                    args=(desired_video, monitor, window_name, self.terminate_video_flags[screen]),
                    daemon=True  # 메인 프로세스 종료 시 자동 종료
                )
                process.start()
                self.video_processes[screen] = process
                self.current_media_state[screen] = desired_video
                rospy.loginfo(f"비디오 프로세스 시작: {window_name}")
            else:
                # 비디오 파일이 없을 경우 프로세스 종료
                self.current_media_state[screen] = None

    def terminate_video(self, screen):
        """비디오 재생 종료 함수 (멀티프로세스 사용)"""
        if screen in self.terminate_video_flags:
            self.terminate_video_flags[screen].set()
            if screen in self.video_processes and self.video_processes[screen]:
                self.video_processes[screen].join(timeout=1.0)
                if self.video_processes[screen].is_alive():
                    self.video_processes[screen].terminate()
                self.video_processes[screen] = None
                rospy.loginfo(f"비디오 프로세스 종료: {screen}")
            self.current_media_state[screen] = None

    def manage_media(self):
        """미디어 관리 함수"""
        try:
            if not hasattr(self, 'video_files') or not hasattr(self, 'video_monitor_map'):
                rospy.logerr("비디오 파일 또는 모니터 매핑이 초기화되지 않았습니다.")
                return

            # 현재 웨이포인트에 따라 원하는 비디오 및 소리 파일 결정
            desired_media = {}
            desired_sound = None

            # 현재 속도가 0인지 확인
            if self.curvel_msg.velocity == 0:
                # 속도가 0이면 소리를 재생하지 않음
                desired_sound = None
            else:
                # 기존 로직에 따라 소리 결정
                if 0 <= self.current_waypoint <= 144:
                    desired_media = {
                        'screen2': self.video_files['screen2'],
                        'screen3': self.video_files['screen3']
                    }
                    desired_sound = 'sound_1'
                elif 145 <= self.current_waypoint <= 170:
                    if self.state == 'BRAKING':
                        desired_media = {
                            'screen2': '/home/navigator/catkin_ws/src/erp42_control_ob/path/front_3.mp4',
                            'screen3': '/home/navigator/catkin_ws/src/erp42_control_ob/path/back_3.mp4'
                        }
                        desired_sound = 'sound_3'  # 경고음은 속도와 무관하게 재생
                    else:
                        desired_media = {
                            'screen2': '/home/navigator/catkin_ws/src/erp42_control_ob/path/front_2.mp4',
                            'screen3': '/home/navigator/catkin_ws/src/erp42_control_ob/path/back_2.mp4'
                        }
                        desired_sound = 'sound_2'
                elif 170 <= self.current_waypoint:
                    desired_media = {
                        'screen2': None,
                        'screen3': None
                    }
                    desired_sound = 'sound_1'
                else:
                    desired_media = {
                        'screen2': None,
                        'screen3': None
                    }
                    desired_sound = None

            # 소리 재생 또는 중지
            if desired_sound:
                self.play_sound(desired_sound)
            else:
                self.stop_sound()

            # 비디오 재생 및 종료는 기존 로직대로 수행
            for screen, video_file in desired_media.items():
                self.play_video(screen, video_file)

        except Exception as e:
            rospy.logerr(f"미디어 관리 중 에러 발생: {str(e)}")
            rospy.logerr(f"에러 위치: {type(e).__name__}")


    def run(self):
        while not rospy.is_shutdown():
            try:
                self.process_path()
                self.publish_messages()
                self.manage_media()  # 미디어 관리 메서드 호출
                self.rate.sleep()
            except rospy.ROSInterruptException:
                rospy.loginfo("Node interrupted")
                break
            except Exception as e:
                rospy.logerr(f"An error occurred: {e}")

        # 노드 종료 시 모든 비디오 및 소리 프로세스 종료
        self.terminate_all_videos()
        self.terminate_sound()

    def terminate_all_videos(self):
        """모든 비디오 프로세스 종료"""
        for screen in self.video_processes.keys():
            self.terminate_video(screen)

    def process_path(self):
        local_path, self.current_waypoint = findLocalPath(self.global_path, self.pose_msg, self.current_waypoint)
        self.local_path = local_path
        target_velocity = self.global_path.poses[self.current_waypoint].pose.position.z

        control_input = self.pid.pid(target_velocity, self.curvel_msg.velocity)

        self.pure_pursuit.getPath(local_path)
        self.pure_pursuit.getPoseStatus(self.pose_msg)
        self.pure_pursuit.getVelStatus(self.curvel_msg)
        self.pure_pursuit.getYawStatus(self.yaw_msg)

        self.ctrl_msg.seq = self.pose_msg.header.seq
        self.ctrl_msg.steering, _, self.target_angle_index = self.pure_pursuit.steering_angle(self.current_waypoint)

        self.control_velocity(control_input, target_velocity)

    def apply_acceleration(self):
        self.ctrl_msg.accel = 100  # 가속 값을 100으로 설정하여 재출발
        self.ctrl_msg.brake = 0
        #self.ctrl_msg.steering = 0.0

    def control_velocity(self, control_input, target_velocity):
        current_time = rospy.Time.now()

        if self.state == "BRAKING" and (current_time - self.brake_start_time < self.brake_duration):
            self.apply_brake()
            elapsed_time = (current_time - self.brake_start_time).to_sec()
            rospy.loginfo(f"Applying brake for {elapsed_time:.2f} seconds")
        elif self.state == "BRAKING" and (current_time - self.brake_start_time >= self.brake_duration):
            self.state = "ACCELERATING"
            self.accel_start_time = current_time
            rospy.loginfo("Brake time completed, starting acceleration")
            self.apply_acceleration()
        elif self.state == "ACCELERATING" and (current_time - self.accel_start_time < self.accel_duration):
            self.apply_acceleration()
            elapsed_time = (current_time - self.accel_start_time).to_sec()
            rospy.loginfo(f"Applying acceleration for {elapsed_time:.2f} seconds")
        elif self.state == "ACCELERATING" and (current_time - self.accel_start_time >= self.accel_duration):
            self.resume_normal_driving()
            self.apply_control_input(control_input, target_velocity)
        else:
            if self.current_waypoint == 170 and not self.waypoint_170_reached:
                self.waypoint_170_reached = True
                self.brake_start_time = current_time
                self.state = "BRAKING"
                rospy.loginfo("Started braking at waypoint 170")
                self.apply_brake()
            else:
                self.apply_control_input(control_input, target_velocity)

    def apply_brake(self):
        self.ctrl_msg.accel = 0
        self.ctrl_msg.brake = 200
        #self.ctrl_msg.steering = 0.0

    def resume_normal_driving(self):
        self.state = "DRIVING"
        self.waypoint_170_reached = False
        rospy.loginfo("Acceleration time completed, resuming normal driving")

    def apply_control_input(self, control_input, target_velocity):
        if 0 < control_input <= 200:
            self.ctrl_msg.accel = control_input
            self.ctrl_msg.brake = 0
        elif control_input > 200:
            self.ctrl_msg.accel = target_velocity
            self.ctrl_msg.brake = 0
        elif control_input < -200:
            self.ctrl_msg.accel = 0
            self.ctrl_msg.brake = 200
        else:
            self.ctrl_msg.accel = 0
            self.ctrl_msg.brake = -control_input

    def publish_messages(self):
        self.ctrl_pub.publish(self.ctrl_msg)
        self.global_path_pub.publish(self.global_path)
        self.local_path_pub.publish(self.local_path)

        local_control_msg = LocalControl()
        local_control_msg.seq = self.pose_msg.header.seq
        local_control_msg.gps_x = self.pose_msg.pose.pose.position.x
        local_control_msg.gps_y = self.pose_msg.pose.pose.position.y
        local_control_msg.gps_z = self.pose_msg.pose.pose.position.z
        local_control_msg.velocity = self.curvel_msg.velocity / 10
        local_control_msg.waypoint_size = len(self.global_path.poses)
        local_control_msg.cur_waypoint = self.current_waypoint
        self.local_control_pub.publish(local_control_msg)

    def pose_callback(self, data):
        self.pose_msg = data

    def velocity_callback(self, speed_data):
        self.curvel_msg = speed_data

    def yaw_callback(self, yaw_data):
        self.yaw_msg = yaw_data
        self.ctrl_pub.publish(self.ctrl_msg)

        current_yaw = self.yaw_msg.heading * 1e-5
        target_yaw = math.atan2(
            self.global_path.poses[self.target_angle_index].pose.position.y - self.pose_msg.pose.pose.position.y,
            self.global_path.poses[self.target_angle_index].pose.position.x - self.pose_msg.pose.pose.position.x
        )
        target_yaw = math.degrees(target_yaw)

        try:
            with open(self.yaw_file_path, 'w') as f:
                f.write(f"Target Yaw: {target_yaw:.2f}\n")
                f.write(f"Current Yaw: {current_yaw:.2f}\n")
        except IOError as e:
            rospy.logerr(f"Error writing to yaw file: {e}")

    def waypoint_callback(self, msg):
        """웨이포인트 콜백 함수"""
        self.current_waypoint = msg.data
        rospy.loginfo(f"Received waypoint: {self.current_waypoint}")

if __name__ == '__main__':
    try:
        # 멀티프로세싱 시작 전에 ROS 노드 초기화
        planner = ERPPlanner()
    except rospy.ROSInterruptException:
        pass
    finally:
        # 노드 종료 시 모든 비디오 및 소리 프로세스 종료
        if 'planner' in locals():
            planner.terminate_all_videos()
            planner.terminate_sound()
