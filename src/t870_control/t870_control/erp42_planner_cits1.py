#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import time
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64, Int16, Float32MultiArray
from control_msgs.msg import Velocity, Gear
from geometry_msgs.msg import PoseStamped, Point, Pose
from morai_msgs.msg import CtrlCmd, LocalControl
from ublox_msgs.msg import NavPVT
from vehicle_msgs.msg import Track, WaypointsArray, Waypoint
from v2x_msgs.msg import Spat
from lib.utils_main import pathReader, findLocalPath, purePursuit, pidController, purePursuit_nogps

class ERPPlanner:
    def __init__(self):
        rospy.init_node('ERP42_planner')
        self.setup_parameters()
        self.setup_publishers()
        self.setup_path()
        self.setup_subscribers()

        # V2X 관련 변수들
        self.current_signal_state = None
        self.current_remaining_time = None  
        self.stopped_for_signal = False
        self.in_gps_mode = True

        # 현재 모드 초기화
        self.current_mode = "GPS"

        # 교차로 정보
        self.waypoint_intersection_info = {
            284: {"intersection_id": 200, "signal_group": 14, "movement_type": "LEFT"},
            359: {"intersection_id": 300, "signal_group": 11, "movement_type": "STR"},
            497: {"intersection_id": 400, "signal_group": 1, "movement_type": "STR"},
            603: {"intersection_id": 500, "signal_group": 2, "movement_type": "LEFT"},
            709: {"intersection_id": 610, "signal_group": 4, "movement_type": "LEFT"},
            1075: {"intersection_id": 300, "signal_group": 15, "movement_type": "STR"},
            1121: {"intersection_id": 200, "signal_group": 11, "movement_type": "STR"},
        }

        # 정지 관련 변수들
        self.time_stop_waypoints = {237, 258, 933}
        self.stop_for_time = False
        self.stop_start_time = None
        self.completed_stops = set()  # 이미 정지한 웨이포인트 추적

        # 장애물 관련 변수들
        self.dynamic_obstacle_detected = False
        self.brake_duration = 3.0 
        self.obstacle_avoid_start_time = None
        self.obstacle_clear_delay = 2.0
        self.obstacle_cleared = False
        self.obstacle_cleared_time = None

        self.obstacle_mode = False
        self.obstacles = []

        self.start_ob_waypoint = 153
        self.end_ob_waypoint = 170

        self.target_velocity_ob = 60

        self.avoidance_target_waypoint = None

        self.rate = rospy.Rate(20)
        self.run()

    def setup_parameters(self):
        self.ctrl_msg = CtrlCmd()
        self.pose_msg = Odometry()
        self.curvel_msg = Velocity()
        self.yaw_msg = NavPVT()
        self.current_waypoint = 0
        self.current_mode = "GPS"
        
    def setup_publishers(self):
        self.ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.local_control_pub = rospy.Publisher('/local_control', LocalControl, queue_size=1)

    def setup_subscribers(self):
        rospy.Subscriber("/v2x_data", Spat, self.v2x_message_callback)
        rospy.Subscriber("/odom/filtered", Odometry, self.pose_callback)
        rospy.Subscriber("/ERP42_velocity", Velocity, self.velocity_callback)
        rospy.Subscriber("/ublox/navpvt", NavPVT, self.yaw_callback)
        rospy.Subscriber("/track", Track, self.track_callback)
        rospy.Subscriber("/newwaypoints", WaypointsArray, self.waypoints_callback)

    def setup_path(self):
        self.path_reader = pathReader('erp42_control_ob')
        self.global_path = self.path_reader.read_txt('24_kcity_bon.txt')
        self.global_path_pub.publish(self.global_path)
        self.pure_pursuit = purePursuit()
        self.pure_pursuit_nogps = purePursuit_nogps()
        self.pid = pidController()

    def run(self):
        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()
            try:
                self.update_current_waypoint()
                self.process_path()
                self.control_vehicle(current_time)
                self.publish_messages()
                self.rate.sleep()
            except rospy.ROSInterruptException:
                rospy.loginfo("Node interrupted")
                break
            except Exception as e:
                rospy.logerr(f"An error occurred: {e}")

    def update_current_waypoint(self):
        min_dist = float('inf')
        closest_waypoint = self.current_waypoint
        for i, waypoint in enumerate(self.global_path.poses):
            dx = self.pose_msg.pose.pose.position.x - waypoint.pose.position.x
            dy = self.pose_msg.pose.pose.position.y - waypoint.pose.position.y
            dist = math.sqrt(dx**2 + dy**2)
            if dist < min_dist:
                min_dist = dist
                closest_waypoint = i
        self.current_waypoint = closest_waypoint

    def process_path(self):
        local_path, self.current_waypoint = findLocalPath(self.global_path, self.pose_msg, self.current_waypoint)
        self.local_path_pub.publish(local_path)

    def control_vehicle(self, current_time):
        # 현재 웨이포인트가 정지해야 할 웨이포인트인지 확인
        current_stop_waypoint = None
        for wp in self.time_stop_waypoints:
            if abs(self.current_waypoint - wp) <= 4 and wp not in self.completed_stops:
                current_stop_waypoint = wp
                break

        # 정지해야 할 웨이포인트에 도달했고, 아직 정지하지 않은 경우
        if current_stop_waypoint is not None and not self.stop_for_time:
            rospy.loginfo(f"웨이포인트 {current_stop_waypoint}에서 3초간 정지합니다.")
            self.apply_brake()
            self.stop_for_time = True
            self.stop_start_time = time.time()
            return

        # 정지 중인 경우 시간 체크
        if self.stop_for_time:
            elapsed_time = time.time() - self.stop_start_time
            if elapsed_time >= 4.0:
                rospy.loginfo(f"웨이포인트 {current_stop_waypoint}에서의 3초 정지가 완료되었습니다.")
                self.stop_for_time = False
                self.stop_start_time = None
                if current_stop_waypoint:
                    self.completed_stops.add(current_stop_waypoint)
                self.in_gps_mode = True
            else:
                rospy.loginfo(f"정지 중... {elapsed_time:.1f}/3.0 초")
                self.apply_brake()
                return

        # 장애물 모드 처리
        if self.obstacle_mode:
            self.handle_obstacle_mode(current_time)
            return

        # V2X 신호 처리
        for waypoint, intersection_info in self.waypoint_intersection_info.items():
            if abs(self.current_waypoint - waypoint) <= 5:
                self.decide_to_stop_or_drive(intersection_info)

        # 정상 주행 모드
        if self.in_gps_mode:
            self.follow_gps_path()
        else:
            self.apply_brake()

    def handle_obstacle_mode(self, current_time):
        if self.dynamic_obstacle_detected:
            if current_time - self.stop_time < self.brake_duration:
                self.ctrl_msg.accel = 0
                self.ctrl_msg.brake = 200
                self.ctrl_msg.steering = 0
                rospy.loginfo(f"장애물 감지 후 정지 중... {current_time - self.stop_time:.1f}/{self.brake_duration}초")
            else:
                self.dynamic_obstacle_detected = False
                rospy.loginfo("초기 정지 완료: 회피 시작")
        else:
            if self.obstacles:
                target_waypoint = self.waypoints[0] if self.waypoints else None
                if target_waypoint:
                    self.ctrl_msg.steering, _ = self.pure_pursuit_nogps.steering_angle(target_waypoint)
                else:
                    self.ctrl_msg.steering = 0  # 기본값 설정

                # 목표 속도로 target_velocity_ob를 사용
                target_velocity = self.target_velocity_ob
                control_input = self.pid.pid(target_velocity, self.curvel_msg.velocity)

                if 0 < control_input <= 200:
                    self.ctrl_msg.accel = control_input
                    self.ctrl_msg.brake = 0
                elif control_input > 200:
                    self.ctrl_msg.accel = target_velocity  # target_velocity_ob로 가속도 제한
                    self.ctrl_msg.brake = 0
                elif control_input < -200:
                    self.ctrl_msg.accel = 0
                    self.ctrl_msg.brake = 200  # 최대 제동
                else:
                    self.ctrl_msg.accel = 0
                    self.ctrl_msg.brake = -control_input

                rospy.loginfo(f"장애물 회피 중 - Steering angle: {self.ctrl_msg.steering:.2f}, "
                              f"Accel: {self.ctrl_msg.accel}, Brake: {self.ctrl_msg.brake}, Target Velocity: {target_velocity}")
            else:
                rospy.loginfo("장애물이 감지되지 않음: GPS 모드로 복귀")
                self.switch_to_gps_mode()

    def apply_brake(self):
        self.ctrl_msg.accel = 0
        self.ctrl_msg.brake = 200
        rospy.loginfo("브레이크 적용됨.")

    def follow_gps_path(self):
        target_velocity = self.global_path.poses[self.current_waypoint].pose.position.z
        control_input = self.pid.pid(target_velocity, self.curvel_msg.velocity)

        local_path, _ = findLocalPath(self.global_path, self.pose_msg, self.current_waypoint)
        self.pure_pursuit.getPath(local_path)
        self.pure_pursuit.getPoseStatus(self.pose_msg)
        self.pure_pursuit.getVelStatus(self.curvel_msg)
        self.pure_pursuit.getYawStatus(self.yaw_msg)
        steering_angle, _, _ = self.pure_pursuit.steering_angle(self.current_waypoint)

        if math.isnan(steering_angle):
            rospy.logwarn("전방 포인트를 찾을 수 없습니다. 기본 조향각 사용.")
            steering_angle = 0.0

        self.ctrl_msg.steering = steering_angle
        self.ctrl_msg.seq = self.pose_msg.header.seq

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

        rospy.loginfo(f"GPS 경로 따라가기. 조향각: {steering_angle:.2f}, 가속: {self.ctrl_msg.accel}, 브레이크: {self.ctrl_msg.brake}")

    def publish_messages(self):
        self.ctrl_pub.publish(self.ctrl_msg)
        rospy.loginfo(f"현재 모드: {self.current_mode}, 웨이포인트: {self.current_waypoint}, 조향각: {self.ctrl_msg.steering:.2f}, "
                      f"속도: {self.curvel_msg.velocity:.2f}, 목표 속도: {self.global_path.poses[self.current_waypoint].pose.position.z:.2f}, "
                      f"신호 상태: {self.current_signal_state}, 신호로 인한 정지: {self.stopped_for_signal}, "
                      f"GPS 모드: {self.in_gps_mode}, 장애물 모드: {self.obstacle_mode}")

    def pose_callback(self, data):
        self.pose_msg = data
        self.pure_pursuit.getPoseStatus(data)

    def velocity_callback(self, speed_data):
        self.curvel_msg = speed_data
        self.pure_pursuit.getVelStatus(speed_data)
        self.pure_pursuit_nogps.getVelStatus(speed_data)

    def yaw_callback(self, yaw_data):
        self.yaw_msg = yaw_data
        self.pure_pursuit.getYawStatus(yaw_data)

    def waypoints_callback(self, msg):
        self.waypoints = msg.waypoints

    def decide_to_stop_or_drive(self, intersection_info):
        if self.current_signal_state == "stop-And-Remain":
            rospy.loginfo(f"웨이포인트 {self.current_waypoint}에서 빨간불 감지, 정지합니다.")
            self.apply_brake()
            self.stopped_for_signal = True
            self.in_gps_mode = False
        elif self.current_signal_state == "protected-Movement-Allowed":
            if intersection_info["movement_type"] in ["STR", "LEFT"]:
                rospy.loginfo(f"웨이포인트 {self.current_waypoint}에서 초록불 감지, 진행합니다.")
                self.in_gps_mode = True
                self.stopped_for_signal = False
        else:
            rospy.loginfo(f"웨이포인트 {self.current_waypoint}에서 노란불 감지, 정지 계획 중입니다.")
            self.apply_brake()
            self.stopped_for_signal = True
            self.in_gps_mode = False

    def v2x_message_callback(self, msg):
        # 현재 웨이포인트가 목표 웨이포인트 근처에 있는지 확인
        for waypoint, intersection_info in self.waypoint_intersection_info.items():
            if abs(self.current_waypoint - waypoint) <= 7:  # 웨이포인트 주변 5 이내일 때만 처리
                intersection_id = intersection_info["intersection_id"]
                signal_group = intersection_info["signal_group"]

                # V2X 메시지에서 교차로 정보 확인
                for intersection in msg.interchanges:
                    if intersection.id.id == intersection_id:
                        for state in intersection.states:
                            if state.signalGroup == signal_group:
                                rospy.loginfo(f"교차로 ID: {intersection_id}, 신호 그룹: {state.signalGroup}, 웨이포인트: {self.current_waypoint}에서 신호 수신")
                                self.current_signal_state = state.state_time_speed.event_state
                                self.current_remaining_time = state.state_time_speed.timing.minEndTime

                                # 신호 상태별 제어 플래그 설정
                                self.decide_to_stop_or_drive(intersection_info)
                                return

    def track_callback(self, msg):
        self.last_track_msg_time = rospy.Time.now().to_sec()
        self.obstacles = msg.cones
        current_obstacle_count = len(self.obstacles)

        if current_obstacle_count == 0:
            if self.obstacle_mode:
                self.switch_to_gps_mode()
                rospy.loginfo("장애물이 사라짐: GPS 모드로 전환")
        else:
            if self.start_ob_waypoint <= self.current_waypoint <= self.end_ob_waypoint:
                if not self.obstacle_mode:
                    self.switch_to_obstacle_mode()
                    self.dynamic_obstacle_detected = True
                    self.stop_time = rospy.Time.now().to_sec()
                    rospy.loginfo("장애물 감지: 긴급 정지 후 회피 시작")
            else:
                rospy.loginfo("장애물이 감지되었으나, 장애물 회피 범위 외의 웨이포인트에 위치")

    def switch_to_gps_mode(self):
        self.obstacle_mode = False
        self.in_gps_mode = True
        self.current_mode = "GPS"
        rospy.loginfo("GPS-based Control 모드로 전환")

    def switch_to_obstacle_mode(self):
        self.obstacle_mode = True
        self.in_gps_mode = False
        self.current_mode = "Obstacle"
        self.obstacle_avoid_start_time = rospy.Time.now().to_sec()
        rospy.loginfo("장애물 회피 모드로 전환")

if __name__ == '__main__':
    try:
        planner = ERPPlanner()
        # planner.run()  # 이미 __init__에서 run() 호출
    except rospy.ROSInterruptException:
        pass


# import rospy
# import math
# import time
# from nav_msgs.msg import Path, Odometry
# from std_msgs.msg import Float64, Int16, Float32MultiArray
# from control_msgs.msg import Velocity, Gear
# from geometry_msgs.msg import PoseStamped, Point, Pose
# from morai_msgs.msg import CtrlCmd, LocalControl
# from ublox_msgs.msg import NavPVT
# from vehicle_msgs.msg import Track, WaypointsArray, Waypoint
# from v2x_msgs.msg import Spat
# from lib.utils_main import pathReader, findLocalPath, purePursuit, pidController, purePursuit_nogps

# class erp_planner():
#     def __init__(self):
#         rospy.init_node('ERP42_planner')
        
#         # 메시지 객체 초기화
#         self.ctrl_msg = CtrlCmd()
#         self.pose_msg = Odometry()
#         self.curvel_msg = Velocity()
#         self.yaw_msg = NavPVT()

#         # 현재 웨이포인트 및 모드 설정
#         self.current_waypoint = 0
#         self.current_mode = "GPS"
#         self.in_gps_mode = True

#         # V2X 관련 변수들
#         self.current_signal_state = None
#         self.current_remaining_time = None  
#         self.stopped_for_signal = False

#         # 교차로 정보
#         self.waypoint_intersection_info = {
#             284: {"intersection_id": 200, "signal_group": 14, "movement_type": "LEFT"},
#             359: {"intersection_id": 300, "signal_group": 11, "movement_type": "STR"},
#             497: {"intersection_id": 400, "signal_group": 1, "movement_type": "STR"},
#             603: {"intersection_id": 500, "signal_group": 2, "movement_type": "LEFT"},
#             709: {"intersection_id": 610, "signal_group": 4, "movement_type": "LEFT"},
#             1075: {"intersection_id": 300, "signal_group": 15, "movement_type": "STR"},
#             1121: {"intersection_id": 200, "signal_group": 11, "movement_type": "STR"},
#         }

#         # 정지 관련 변수들
#         self.time_stop_waypoints = {237, 258, 933}
#         self.stop_for_time = False
#         self.stop_start_time = None
#         self.completed_stops = set()  # 이미 정지한 웨이포인트 추적

#         # 장애물 관련 변수들
#         self.dynamic_obstacle_detected = False
#         self.brake_duration = 3.0 
#         self.obstacle_avoid_start_time = None
#         self.obstacle_clear_delay = 2.0
#         self.obstacle_cleared = False
#         self.obstacle_cleared_time = None
#         self.obstacle_mode = False
#         self.obstacles = []

#         self.start_ob_waypoint = 153
#         self.end_ob_waypoint = 170
#         self.target_velocity_ob = 60
#         self.avoidance_target_waypoint = None

#         # 경로 설정
#         self.path_reader = pathReader('erp42_control_ob')
#         self.global_path = self.path_reader.read_txt('24_kcity_bon.txt')
#         self.current_waypoint = 0

#         # 제어 객체 초기화
#         self.pure_pursuit = purePursuit()
#         self.pure_pursuit_nogps = purePursuit_nogps()
#         self.pid = pidController()

#         # 퍼블리셔 설정
#         self.ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
#         self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
#         self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
#         self.local_control_pub = rospy.Publisher('/local_control', LocalControl, queue_size=1)

#         # 서브스크라이버 설정
#         rospy.Subscriber("/v2x_data", Spat, self.v2x_message_callback)
#         rospy.Subscriber("/odom/filtered", Odometry, self.pose_callback)
#         rospy.Subscriber("/ERP42_velocity", Velocity, self.velocity_callback)
#         rospy.Subscriber("/ublox/navpvt", NavPVT, self.yaw_callback)
#         rospy.Subscriber("/track", Track, self.track_callback)
#         rospy.Subscriber("/newwaypoints", WaypointsArray, self.waypoints_callback)

#         # 기타 변수 초기화
#         self.waypoints = []
#         self.rate = rospy.Rate(20)

#     def pose_callback(self, data):
#         self.pose_msg = data
#         self.pure_pursuit.getPoseStatus(data)

#     def velocity_callback(self, speed_data):
#         self.curvel_msg = speed_data
#         self.pure_pursuit.getVelStatus(speed_data)
#         self.pure_pursuit_nogps.getVelStatus(speed_data)

#     def yaw_callback(self, yaw_data):
#         self.yaw_msg = yaw_data
#         self.pure_pursuit.getYawStatus(yaw_data)

#     def waypoints_callback(self, msg):
#         self.waypoints = msg.waypoints

#     def v2x_message_callback(self, msg):
#         for waypoint, intersection_info in self.waypoint_intersection_info.items():
#             if abs(self.current_waypoint - waypoint) <= 7:
#                 intersection_id = intersection_info["intersection_id"]
#                 signal_group = intersection_info["signal_group"]

#                 for intersection in msg.interchanges:
#                     if intersection.id.id == intersection_id:
#                         for state in intersection.states:
#                             if state.signalGroup == signal_group:
#                                 rospy.loginfo(f"교차로 ID: {intersection_id}, 신호 그룹: {state.signalGroup}, 웨이포인트: {self.current_waypoint}에서 신호 수신")
#                                 self.current_signal_state = state.state_time_speed.event_state
#                                 self.current_remaining_time = state.state_time_speed.timing.minEndTime
#                                 self.decide_to_stop_or_drive(intersection_info)
#                                 return

#     def track_callback(self, msg):
#         self.last_track_msg_time = rospy.Time.now().to_sec()
#         self.obstacles = msg.cones
#         current_obstacle_count = len(self.obstacles)

#         if current_obstacle_count == 0:
#             if self.obstacle_mode:
#                 self.switch_to_gps_mode()
#                 rospy.loginfo("장애물이 사라짐: GPS 모드로 전환")
#         else:
#             if self.start_ob_waypoint <= self.current_waypoint <= self.end_ob_waypoint:
#                 if not self.obstacle_mode:
#                     self.switch_to_obstacle_mode()
#                     self.dynamic_obstacle_detected = True
#                     self.stop_time = rospy.Time.now().to_sec()
#                     rospy.loginfo("장애물 감지: 긴급 정지 후 회피 시작")
#             else:
#                 rospy.loginfo("장애물이 감지되었으나, 장애물 회피 범위 외의 웨이포인트에 위치")

#     def update_current_waypoint(self):
#         min_dist = float('inf')
#         closest_waypoint = self.current_waypoint
#         for i, waypoint in enumerate(self.global_path.poses):
#             dx = self.pose_msg.pose.pose.position.x - waypoint.pose.position.x
#             dy = self.pose_msg.pose.pose.position.y - waypoint.pose.position.y
#             dist = math.sqrt(dx**2 + dy**2)
#             if dist < min_dist:
#                 min_dist = dist
#                 closest_waypoint = i
#         self.current_waypoint = closest_waypoint

#     def process_path(self):
#         local_path, self.current_waypoint = findLocalPath(self.global_path, self.pose_msg, self.current_waypoint)
#         self.local_path_pub.publish(local_path)

#     def control_vehicle(self, current_time):
#         current_stop_waypoint = None
#         for wp in self.time_stop_waypoints:
#             if abs(self.current_waypoint - wp) <= 4 and wp not in self.completed_stops:
#                 current_stop_waypoint = wp
#                 break

#         if current_stop_waypoint is not None and not self.stop_for_time:
#             rospy.loginfo(f"웨이포인트 {current_stop_waypoint}에서 3초간 정지합니다.")
#             self.apply_brake()
#             self.stop_for_time = True
#             self.stop_start_time = time.time()
#             return

#         if self.stop_for_time:
#             elapsed_time = time.time() - self.stop_start_time
#             if elapsed_time >= 4.0:
#                 rospy.loginfo(f"웨이포인트 {current_stop_waypoint}에서의 3초 정지가 완료되었습니다.")
#                 self.stop_for_time = False
#                 self.stop_start_time = None
#                 if current_stop_waypoint:
#                     self.completed_stops.add(current_stop_waypoint)
#                 self.in_gps_mode = True
#             else:
#                 rospy.loginfo(f"정지 중... {elapsed_time:.1f}/3.0 초")
#                 self.apply_brake()
#                 return

#         if self.obstacle_mode:
#             self.handle_obstacle_mode(current_time)
#             return

#         for waypoint, intersection_info in self.waypoint_intersection_info.items():
#             if abs(self.current_waypoint - waypoint) <= 5:
#                 self.decide_to_stop_or_drive(intersection_info)

#         if self.in_gps_mode:
#             self.follow_gps_path()
#         else:
#             self.apply_brake()

#     def handle_obstacle_mode(self, current_time):
#         if self.dynamic_obstacle_detected:
#             if current_time - self.stop_time < self.brake_duration:
#                 self.ctrl_msg.accel = 0
#                 self.ctrl_msg.brake = 200
#                 self.ctrl_msg.steering = 0
#                 rospy.loginfo(f"장애물 감지 후 정지 중... {current_time - self.stop_time:.1f}/{self.brake_duration}초")
#             else:
#                 self.dynamic_obstacle_detected = False
#                 rospy.loginfo("초기 정지 완료: 회피 시작")
#         else:
#             if self.obstacles:
#                 target_waypoint = self.waypoints[0] if self.waypoints else None
#                 if target_waypoint:
#                     self.ctrl_msg.steering, _ = self.pure_pursuit_nogps.steering_angle(target_waypoint)
#                 else:
#                     self.ctrl_msg.steering = 0

#                 target_velocity = self.target_velocity_ob
#                 control_input = self.pid.pid(target_velocity, self.curvel_msg.velocity)

#                 if 0 < control_input <= 200:
#                     self.ctrl_msg.accel = control_input
#                     self.ctrl_msg.brake = 0
#                 elif control_input > 200:
#                     self.ctrl_msg.accel = target_velocity
#                     self.ctrl_msg.brake = 0
#                 elif control_input < -200:
#                     self.ctrl_msg.accel = 0
#                     self.ctrl_msg.brake = 200
#                 else:
#                     self.ctrl_msg.accel = 0
#                     self.ctrl_msg.brake = -control_input

#                 rospy.loginfo(f"장애물 회피 중 - Steering angle: {self.ctrl_msg.steering:.2f}, "
#                               f"Accel: {self.ctrl_msg.accel}, Brake: {self.ctrl_msg.brake}, Target Velocity: {target_velocity}")
#             else:
#                 rospy.loginfo("장애물이 감지되지 않음: GPS 모드로 복귀")
#                 self.switch_to_gps_mode()

#     def apply_brake(self):
#         self.ctrl_msg.accel = 0
#         self.ctrl_msg.brake = 200
#         rospy.loginfo("브레이크 적용됨.")

#     def follow_gps_path(self):
#         target_velocity = self.global_path.poses[self.current_waypoint].pose.position.z
#         control_input = self.pid.pid(target_velocity, self.curvel_msg.velocity)

#         local_path, _ = findLocalPath(self.global_path, self.pose_msg, self.current_waypoint)
#         self.pure_pursuit.getPath(local_path)
#         self.pure_pursuit.getPoseStatus(self.pose_msg)
#         self.pure_pursuit.getVelStatus(self.curvel_msg)
#         self.pure_pursuit.getYawStatus(self.yaw_msg)
#         steering_angle, _, _ = self.pure_pursuit.steering_angle(self.current_waypoint)

#         if math.isnan(steering_angle):
#             rospy.logwarn("전방 포인트를 찾을 수 없습니다. 기본 조향각 사용.")
#             steering_angle = 0.0

#         self.ctrl_msg.steering = steering_angle
#         self.ctrl_msg.seq = self.pose_msg.header.seq

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

#         rospy.loginfo(f"GPS 경로 따라가기. 조향각: {steering_angle:.2f}, 가속: {self.ctrl_msg.accel}, 브레이크: {self.ctrl_msg.brake}")

#     def decide_to_stop_or_drive(self, intersection_info):
#         if self.current_signal_state == "stop-And-Remain":
#             rospy.loginfo(f"웨이포인트 {self.current_waypoint}에서 빨간불 감지, 정지합니다.")
#             self.apply_brake()
#             self.stopped_for_signal = True
#             self.in_gps_mode = False
#         elif self.current_signal_state == "protected-Movement-Allowed":
#             if intersection_info["movement_type"] in ["STR", "LEFT"]:
#                 rospy.loginfo(f"웨이포인트 {self.current_waypoint}에서 초록불 감지, 진행합니다.")
#                 self.in_gps_mode = True
#                 self.stopped_for_signal = False
#         else:
#             rospy.loginfo(f"웨이포인트 {self.current_waypoint}에서 노란불 감지, 정지 계획 중입니다.")
#             self.apply_brake()
#             self.stopped_for_signal = True
#             self.in_gps_mode = False

#     def switch_to_gps_mode(self):
#         self.obstacle_mode = False
#         self.in_gps_mode = True
#         self.current_mode = "GPS"
#         rospy.loginfo("GPS-based Control 모드로 전환")

#     def switch_to_obstacle_mode(self):
#         self.obstacle_mode = True
#         self.in_gps_mode = False
#         self.current_mode = "Obstacle"
#         self.obstacle_avoid_start_time = rospy.Time.now().to_sec()
#         rospy.loginfo("장애물 회피 모드로 전환")

#     def publish_messages(self):
#         self.ctrl_pub.publish(self.ctrl_msg)
#         rospy.loginfo(f"현재 모드: {self.current_mode}, 웨이포인트: {self.current_waypoint}, 조향각: {self.ctrl_msg.steering:.2f}, "
#                       f"속도: {self.curvel_msg.velocity:.2f}, 목표 속도: {self.global_path.poses[self.current_waypoint].pose.position.z:.2f}, "
#                       f"신호 상태: {self.current_signal_state}, 신호로 인한 정지: {self.stopped_for_signal}, "
#                       f"GPS 모드: {self.in_gps_mode}, 장애물 모드: {self.obstacle_mode}")

#     def run(self):
#         while not rospy.is_shutdown():
#             current_time = rospy.Time.now().to_sec()
#             try:
#                 self.update_current_waypoint()
#                 self.process_path()
#                 self.control_vehicle(current_time)
#                 self.publish_messages()
#                 self.rate.sleep()
#             except rospy.ROSInterruptException:
#                 rospy.loginfo("Node interrupted")
#                 break
#             except Exception as e:
#                 rospy.logerr(f"An error occurred: {e}")

# if __name__ == '__main__':
#     try:
#         planner = erp_planner()
#         planner.run()
#     except rospy.ROSInterruptException:
#         pass
