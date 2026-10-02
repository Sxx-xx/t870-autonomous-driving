#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import time  #  V2X 기능을 위해 추가
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64, Int16, Float32MultiArray,String
from control_msgs.msg import Velocity, Gear
from geometry_msgs.msg import Point
from morai_msgs.msg import CtrlCmd, LocalControl
from ublox_msgs.msg import NavPVT
from lib.utils_main import pathReader, findLocalPath, purePursuit, pidController, purePursuit_nogps
from vehicle_msgs.msg import Track, WaypointsArray, Waypoint
from std_msgs.msg import Bool
from v2x_msgs.msg import Spat_new  #  V2X 기능을 위해 추가
from collections import deque # 배달 미션을 위해 추가
class erp_planner():
    def __init__(self):

        self.current_text = 'none'
        self.goal_delivery = ''
        self.stop_line_detected = False 

        rospy.init_node('ERP42_planner')

                # ✅ 2.  waypoint: {신호등 id , 신호그룹, 움직임}
        self.waypoint_intersection_info = {
        111: {"intersection_id": 200, "signal_group": 1, "movement_type": "STR"}, #143
        164: {"intersection_id": 300, "signal_group": 1, "movement_type": "STR"},
        300: {"intersection_id": 400, "signal_group": 1, "movement_type": "STR"},
        560: {"intersection_id": 1200, "signal_group": 1, "movement_type": "STR"},
        721: {"intersection_id": 1300, "signal_group": 1, "movement_type": "STR"},
        771: {"intersection_id": 1400, "signal_group": 1, "movement_type": "STR"}

        }
        # Path settings
        self.global_path_name = 'k-city_1m_final'
        self.path_name = 'k-city_1m_final' 
            
            
        # Message objects
        self.ctrl_msg = CtrlCmd()   
        self.pose_msg = Odometry()
        self.curvel_msg = Velocity()
        self.yaw_msg = NavPVT()

        # ✅ V2X 관련 변수들 추가 (기존)
        self.current_signal_state = None
        self.current_remaining_time = None  
        self.stopped_for_signal = False

        # ✅ V2X 예측 관련 변수들 추가 (새로 추가)
        self.signal_predictions = []  # 다음 신호들의 예측 정보
        self.distance_to_intersection = 0  # 교차로까지의 거리
        self.approach_speed = 0  # 교차로 접근 속도
        self.time_to_intersection = 0  # 교차로 도달 예상 시간
        self.should_slow_down = False  # 감속 필요 여부
        self.optimal_approach_speed = 0  # 최적 접근 속도
        # self.temp_target_velocity = None  # 임시 목표 속도


        # # 기존 self.stop_line_detected 는 삭제하거나 유지 가능
        # self.stop_line_zones = [
        #     {"start": 0, "end": 1, "stop_wp": 0, "stop_time": 5.0},
        #     # 필요한 만큼 구간 및 정지 웨이포인트, 정지시간 추가
        # ]

        # self.active_stop_line = None  # 현재 접근 중인 정지선 zone

        # === [REPLACE / ADD] OA 트리거: 단일 집합 → 다중 구간 ===
        # 예시: 두 구간 [(시작, 끝), (시작, 끝)]
        self.oa_trigger_ranges = [(208,238),(850,865)]#[(1305,1499)]  #1270, 1500 필요 구간으로 교체
        self.oa_wait_duration = 3  #1.5
        self.oa_exit_tolerance = 0     # 경계 ±허용치(인덱스)self.oa_wait_duration
        self.oa_active = False
        self.oa_trigger_wp_range = None  # (start,end) 저장
        self.oa_start_time = 0.0

        self.rrt_waypoints_buffer = []
        self.last_rrt_waypoints_stamp = 0.0


        self.fixed_velocity_mode = False
        self.stop_time = 0.0                 # 정지 시작 시각 기본값
        self.brake_duration = 0.8 #1.5            # 정지 유지 시간(s) 기본값 (원하는 값으로)

        # ✅ 1. 이 waypoint 일단 정지
        self.time_stop_waypoints = {421,437,456}
        self.stop_for_time = False
        self.stop_start_time = None
        self.completed_stops = set()  # 이미 정지한 웨이포인트 추적

        # U-turn mode variables
        self.u_turn_mode = False
        self.u_turn_completed = False
        self.u_turn_executed = False
        self.u_turn_start_time = None
        self.u_turn_velocity = 70
        self.u_turn_duration = 5.0
        self.u_turn_start_waypoint =0    #314 /2025에 맞게 수정이 필요합니다.
        self.u_turn_end_waypoint =-1  #418 /2025에 맞게 수정이 필요합니다.
        self.min_obstacles_for_uturn = 3
        self.u_turn_steering_first = 50
        self.u_turn_steering_second = 10
        self.u_turn_first_phase_duration = 4.5
        self.u_turn_cleared = False
        self.u_turn_cleared_time = None
        self.obstacle_last_detected_time = None

        # Class objects
        self.pure_pursuit = purePursuit()
        self.pure_pursuit_nogps = purePursuit_nogps()
        self.pid = pidController()
        self.path_reader = pathReader('erp42_control_ob')

        self.dynamic_obstacle_detected = False
        self.brake_duration = 3.0
        self.obstacle_avoid_start_time = None
        self.obstacle_clear_delay = 2.0
        self.obstacle_tcleared = False
        self.obstacle_cleared_time = None
        self.obstacle_avoid_cooldown = 7.0
        self.obstacle_avoid_start_time = None
        self.can_stop_for_obstacle = True
        self.fixed_velocity_start_time = None
        self.target_velocity = 0.0


        # Lane detection variables
        self.lane_center_points = []
        self.max_center_points = 15
        self.waypoints = []

        # Publishers
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.local_control_pub = rospy.Publisher('/local_control', LocalControl, queue_size=1)

        rospy.Subscriber("/v2x_data", Spat_new, self.v2x_message_callback)  # ✅ V2X 구독자 추가
        
        # Path and waypoint variables
        self.global_path = self.path_reader.read_txt(self.path_name + ".txt")
        self.current_waypoint = 0
        rospy.Subscriber("/stop_line", Bool, self.stop_line_callback)  ##임의 수정

        # Mode flags
        self.obstacle_mode = False
        self.obstacles = [] 
        self.gps_mode = True
        self.lane_mode = False
        self.current_mode = "GPS"
        
        # Control parameters /2025에 맞게 수정이 필요합니다.
        self.target_velocity_gps = self.global_path.poses[self.current_waypoint].pose.position.z
        self.target_velocity_lane = 50
        self.target_velocity_ob = 30
        self.target_velocity = 0
        # Mode transition waypoints /2025에 맞게 수정이 필요합니다.
        self.lane_start_waypoint = 9999 #139   #600  #737   #677   
        self.lane_end_waypoint =  99991#229   #870    #1009  #833


        # Emergency brake
        self.emergency_brake = False
        self.emergency_brake_start_time = None
        self.last_track_msg_time = None
        self.track_timeout = 0.22
        
        self.delivery_a = ''
        self.delivery_b = ''
        self.delivery_active_a = False
        self.delivery_active_b = False

        self.pick_up_waypoint_start =  46#401
        self.pick_up_waypoint_end = 54#460

        self.delivery_waypoint_start =505 #566
        self.delivery_waypoint_end = 556#737

        self.parking_waypoint_start = 920#61#160
        self.parking_waypoint_end = 943#80#387

        self.parking_finish = False

        self.window5_A = deque(maxlen=3) # 연속적으로 표지판 값이 들어올 공간
        self.window5_B = deque(maxlen=3) # 연속적으로 표지판 값이 들어올 공간
        self.stable_a = None # 연속적으로 들어온 값들이 들어올 값
        self.stable_b = None # 연속적으로 들어온 값들이 들어올 값

        self.is_obstacle = False
        self.obstacle_window = deque(maxlen=5) # 연속적으로 표지판 값이 들어올 공간


        # Subscribers
        rospy.Subscriber("/odom/filtered", Odometry, self.pose_callback)
        rospy.Subscriber("/ERP42_velocity", Velocity, self.velocity_callback)
        rospy.Subscriber("/ublox/navpvt", NavPVT, self.yaw_callback)
        rospy.Subscriber("/lane_center_points", Point, self.lane_center_callback)
        rospy.Subscriber("/track", Track, self.track_callback)
        rospy.Subscriber("/newwaypoints", WaypointsArray, self.waypoints_callback)
        rospy.Subscriber("/rrt_newwaypoints", WaypointsArray, self.rrt_waypoints_callback)
        rospy.Subscriber('/yolo/current_text_a', String, self.current_text_callback_A)
        rospy.Subscriber('/yolo/current_text_b', String, self.current_text_callback_B)
        rospy.Subscriber("/adaptive_clustering/box_drawn", Bool, self.box_drawn_callback)


    def is_ahead(self, current_wp, target_wp, total_len, max_range):
        diff = (target_wp - current_wp + total_len) % total_len
        return 0 < diff <= max_range
    
    def v2x_message_callback(self, msg):
        total_len = len(self.global_path.poses)

        for waypoint, intersection_info in self.waypoint_intersection_info.items():
            rospy.loginfo(f"[V2X] 현재 waypoint: {self.current_waypoint}, C-ITS 신호 waypoint: {waypoint}")
            rospy.loginfo(f"[V2X] 현재 waypoint: {self.current_waypoint}, C-ITS 신호 waypoint: {waypoint}")
            rospy.loginfo(f"[V2X] 현재 waypoint: {self.current_waypoint}, C-ITS 신호 waypoint: {waypoint}")
            rospy.loginfo(f"[V2X] 현재 waypoint: {self.current_waypoint}, C-ITS 신호 waypoint: {waypoint}")
            if self.is_ahead(self.current_waypoint, waypoint, total_len, 12):  # ✅ 진행 방향 기준으로만 
                rospy.loginfo("✅✅✅✅cits 만나는 중!!✅✅✅✅")

                
                intersection_id = intersection_info["intersection_id"]
                signal_group = intersection_info["signal_group"]

                for intersection in msg.interchanges:
                    if intersection.id.id == intersection_id:
                        for state in intersection.states:
                            if state.signalGroup == signal_group:
                                self.current_signal_state = state.state_time_speed[0].event_state
                                self.current_remaining_time = state.state_time_speed[0].timing.minEndTime / 10.0
                                return



    def _in_any_oa_range(self, wp_idx):
        for rng in self.oa_trigger_ranges:
            s, e = rng
            if s <= wp_idx <= e:
                return rng
        return None

    def _outside_current_oa_range(self, wp_idx):
        if not self.oa_trigger_wp_range:
            return True
        s, e = self.oa_trigger_wp_range
        return wp_idx < (s - self.oa_exit_tolerance) or wp_idx > (e + self.oa_exit_tolerance)
    

    def pose_callback(self, data):
        self.pose_msg = data
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y
        self.pure_pursuit.getPoseStatus(data)

    def velocity_callback(self, speed_data):
        self.curvel_msg = speed_data
        self.pure_pursuit.getVelStatus(speed_data)

    def yaw_callback(self, yaw_data):
        self.yaw_msg = yaw_data
        self.pure_pursuit.getYawStatus(yaw_data)

    def lane_center_callback(self, point):
        self.lane_center_points.append((point.x, point.y))
        if len(self.lane_center_points) > self.max_center_points:
            self.lane_center_points.pop(0)

    def waypoints_callback(self, msg):
        self.waypoints = msg.waypoints

    def rrt_waypoints_callback(self, msg):
        """RRT OA에서 전달된 새 웨이포인트 저장"""
        self.rrt_waypoints_buffer = [(w.x, w.y) for w in msg.waypoints]
        self.last_rrt_waypoints_stamp = rospy.Time.now().to_sec()  # 써도 되고 안 써도 됨
                
    def current_text_callback_A(self, msg: String):
            if msg.data in ['A1','A2','A3']:
                self.delivery_a = msg.data
                

    def current_text_callback_B(self, msg: String):
            if msg.data in ['B1','B2','B3']:
                self.delivery_b = msg.data  


    def box_drawn_callback(self, msg: Bool):
        val = bool(msg.data)                 # 안전하게 bool 변환
        self.obstacle_window.append(val)     # deque에 추가
        self.is_obstacle = val               # 내부 플래그 업데이트
        rospy.loginfo(f"[BoxDrawn] box_drawn={val}, window={list(self.obstacle_window)}")

    def stop_line_callback(self, msg: Bool):
        self.stop_line_detected = msg.data

    def track_callback(self, msg):
        self.last_track_msg_time = rospy.Time.now().to_sec()
        self.obstacles = msg.cones
        current_obstacle_count = len(self.obstacles)

        # === [MODIFY] 장애물 + 구간 트리거에서 OA 시나리오 진입 ===
        rng = self._in_any_oa_range(self.current_waypoint)
        if (self.gps_mode and not self.oa_active
            and current_obstacle_count > 0
            and rng is not None):
            self.oa_active = True
            self.oa_trigger_wp_range = rng   # (start,end)
            self.oa_start_time = rospy.Time.now().to_sec()

            self.ctrl_msg.accel = 0
            self.ctrl_msg.brake = 120
            self.ctrl_msg.steering = 0
            self.ctrl_pub.publish(self.ctrl_msg)

            rospy.loginfo(f"[OA] 구간 {rng}에서 장애물 감지 → {self.oa_wait_duration:.1f}s 대기 후 OA 조향")
            return


        # U-turn trigger
        if (self.u_turn_start_waypoint <= self.current_waypoint <= self.u_turn_end_waypoint and
            not self.u_turn_completed and not self.u_turn_executed):
            if current_obstacle_count >= self.min_obstacles_for_uturn:
                if any(ob.y > 0 for ob in self.obstacles):
                    self.u_turn_executed = True
                    self.start_u_turn()
                    rospy.loginfo(f"양수 y좌표 장애물 포함 총 {current_obstacle_count}개 감지: 유턴 시작")

        if current_obstacle_count == 0:
            if self.obstacle_mode and not self.obstacle_cleared:
                self.obstacle_cleared = True
                self.obstacle_cleared_time = rospy.Time.now().to_sec()
                rospy.loginfo("장애물이 사라짐: 차선 모드로 전환 대기 시작")
        else:
            self.obstacle_cleared = False
            if not self.obstacle_mode and self.lane_mode:
                self.switch_to_obstacle_mode()
                self.dynamic_obstacle_detected = True
                self.stop_time = rospy.Time.now().to_sec()
                rospy.loginfo("장애물 감지: 긴급 정지")
            elif self.gps_mode:
                rospy.loginfo("GPS 모드에서 장애물 감지: GPS 모드 유지")

    def calculate_steering_angle(self):
        if not self.lane_center_points:
            return 0
        latest_point = self.lane_center_points[-1]
        image_center = 320
        error = latest_point[0] - image_center
        rospy.loginfo("차선 에러: %.2f", error)
        steering_angle = (error / image_center) * 22
        return max(min(steering_angle, 25), -25)

    def switch_to_obstacle_mode(self):
        self.obstacle_mode = True
        self.lane_mode = False
        self.gps_mode = False
        self.current_mode = "Obstacle"
        self.obstacle_avoid_start_time = rospy.Time.now().to_sec()
        rospy.loginfo("장애물 회피 모드로 전환")

    def switch_to_lane_mode(self):
        self.obstacle_mode = False
        self.lane_mode = True
        self.gps_mode = False
        self.current_mode = "Lane"
        self.dynamic_obstacle_detected = False
        self.waypoints = []
        self.fixed_velocity_mode = True
        self.fixed_velocity_start_time = rospy.Time.now().to_sec()
        rospy.loginfo("Lane-based Control 모드로 전환")

    def switch_to_gps_mode(self):
        self.obstacle_mode = False
        self.lane_mode = False
        self.gps_mode = True
        self.current_mode = "GPS"
        rospy.loginfo("GPS-based Control 모드로 전환")

    def start_u_turn(self):
        self.gps_mode = False
        self.u_turn_mode = True
        self.u_turn_start_time = rospy.Time.now().to_sec()
        rospy.loginfo("유턴 모드 시작")

    def check_u_turn_completion(self):
        if self.u_turn_mode:
            current_time = rospy.Time.now().to_sec()
            if current_time - self.u_turn_start_time >= self.u_turn_duration:
                self.u_turn_completed = True
                self.switch_to_gps_mode()
                self.current_mode = "GPS"
                rospy.loginfo("유턴 완료: GPS 모드로 전환")

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

    def decide_to_stop_or_drive(self, intersection_info):
        """신호등에 따른 주행 결정 (예측 기능 포함)"""
        if self.current_signal_state == "stop-And-Remain":
            rospy.loginfo(f"🔴🔴🔴🔴빨간불 감지 (거리: {self.distance_to_intersection:.1f}m)")
            rospy.loginfo(f"🔴🔴🔴🔴빨간불 감지 (거리: {self.distance_to_intersection:.1f}m)")
            rospy.loginfo(f"🔴🔴🔴🔴빨간불 감지 (거리: {self.distance_to_intersection:.1f}m)")
            self.stopped_for_signal = True
            self.gps_mode = False
            self.ctrl_msg.accel = 0
            self.ctrl_msg.brake = 150
            self.ctrl_msg.steering = 0
            self.ctrl_pub.publish(self.ctrl_msg)

            
        elif self.current_signal_state == "protected-Movement-Allowed":
                    rospy.loginfo(f"🟢🟢🟢🟢초록불, 정상 진행")
                    self.gps_mode = True
                    self.stopped_for_signal = False
                    # self.temp_target_velocity = None
        else:
            # permissive-clearance (노란불)
            if self.current_remaining_time is not None and self.current_remaining_time > 1.5 :
                # 가까우면 통과
                rospy.loginfo(f"🟡 노랑불이지만 시간이 여유로워 통과 (시간: {self.time_to_intersection:.1f}m)")
                rospy.loginfo(f"🟡 노랑불이지만 시간이 여유로워 통과 (시간: {self.time_to_intersection:.1f}m)")
                rospy.loginfo(f"🟡 노랑불이지만 시간이불 여유로워 통과 (시간: {self.time_to_intersection:.1f}m)")

                self.gps_mode = True
                self.stopped_for_signal = False
                self.ctrl_msg.accel = self.target_velocity_lane + 110
                self.ctrl_msg.seq = self.pose_msg.header.seq
                self.ctrl_pub.publish(self.ctrl_msg)
            elif self.current_remaining_time is not None and self.current_remaining_time < 1.5 :
                # 멀면 정지
                rospy.loginfo(f"🟡노랑불, 정지 (거리: {self.distance_to_intersection:.1f}m)")
                self.stopped_for_signal = True
                self.gps_mode = False
                self.ctrl_msg.accel = 0
                self.ctrl_msg.brake = 100
                self.ctrl_msg.steering = 0
                self.ctrl_pub.publish(self.ctrl_msg)

                # self.switch_to_lane_mode()

    def delivery_parking_a(self):
        
            self.obstacle_mode = False
            self.lane_mode = False
            self.gps_mode = False

            self.ctrl_msg.accel = 0
            self.ctrl_msg.brake = 100
            self.ctrl_msg.seq = self.pose_msg.header.seq
            self.ctrl_pub.publish(self.ctrl_msg)
            rospy.sleep(2)


            self.ctrl_msg.gear = 0
            self.ctrl_msg.accel = 80
            self.ctrl_msg.brake = 0
            self.ctrl_msg.steering = 32
            self.ctrl_msg.seq = self.pose_msg.header.seq
            self.ctrl_pub.publish(self.ctrl_msg)
            rospy.loginfo(f"🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢{self.delivery_a}🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢")
            rospy.sleep(3)  # 20Hz로 accel 전송

            self.ctrl_msg.gear = 0
            self.ctrl_msg.accel = 60
            self.ctrl_msg.brake = 0
            self.ctrl_msg.steering = -32
            self.ctrl_msg.seq = self.pose_msg.header.seq
            self.ctrl_pub.publish(self.ctrl_msg)
            rospy.loginfo(f"🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢{self.delivery_a}🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢")
            rospy.sleep(2.8)  # 20Hz로 accel 전송

            # self.ctrl_msg.gear = 0
            # self.ctrl_msg.accel = 40
            # self.ctrl_msg.brake = 0
            # self.ctrl_msg.steering = 0
            # self.ctrl_msg.seq = self.pose_msg.header.seq
            # self.ctrl_pub.publish(self.ctrl_msg)
            # rospy.loginfo(f"🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢{self.delivery_a}🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢")
            # rospy.sleep(1)  # 20Hz로 accel 전송


            rospy.loginfo(f"🟢🔴🟢🔴🟢🔴🟢🔴{self.delivery_a}🟢🔴🟢🔴🟢🔴🟢🔴🟢🔴")
            self.ctrl_msg.accel = 0
            self.ctrl_msg.brake = 100
            self.ctrl_msg.seq = self.pose_msg.header.seq
            self.ctrl_pub.publish(self.ctrl_msg)
            rospy.sleep(5)

            self.ctrl_msg.gear = 0
            self.ctrl_msg.accel = 60
            self.ctrl_msg.brake = 0
            self.ctrl_msg.steering = -32
            self.ctrl_msg.seq = self.pose_msg.header.seq
            self.ctrl_pub.publish(self.ctrl_msg)
            rospy.loginfo(f"🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢{self.delivery_a}🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢")
            rospy.sleep(4)  # 20Hz로 accel 전송

            
            self.gps_mode = True
            self.delivery_active_a = True



    def delivery_parking_b(self):
        if self.delivery_active_b == False:
            self.obstacle_mode = False
            self.lane_mode = False
            self.gps_mode = False

            self.ctrl_msg.accel = 0
            self.ctrl_msg.brake = 100
            self.ctrl_msg.seq = self.pose_msg.header.seq
            self.ctrl_pub.publish(self.ctrl_msg)
            rospy.sleep(2)

            self.ctrl_msg.accel = 80
            self.ctrl_msg.brake = 0
            self.ctrl_msg.seq = self.pose_msg.header.seq
            self.ctrl_pub.publish(self.ctrl_msg)
            rospy.sleep(4)
            rospy.loginfo(f"🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴{self.delivery_a}🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢")

            self.ctrl_msg.gear = 0
            self.ctrl_msg.accel = 70
            self.ctrl_msg.brake = 0
            self.ctrl_msg.steering = 40
            self.ctrl_msg.seq = self.pose_msg.header.seq
            self.ctrl_pub.publish(self.ctrl_msg)
            rospy.loginfo(f"🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢{self.delivery_a}🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢")
            rospy.sleep(1.2)  # 20Hz로 accel 전송

            self.ctrl_msg.gear = 0
            self.ctrl_msg.accel = 70
            self.ctrl_msg.brake = 0
            self.ctrl_msg.steering = -20
            self.ctrl_msg.seq = self.pose_msg.header.seq
            self.ctrl_pub.publish(self.ctrl_msg)
            rospy.loginfo(f"🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢{self.delivery_a}🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢")
            rospy.sleep(1.3)  # 20Hz로 accel 전송

            rospy.loginfo(f"🟢🔴🟢🔴🟢🔴🟢🔴{self.delivery_a}🟢🔴🟢🔴🟢🔴🟢🔴🟢🔴")
            self.ctrl_msg.accel = 0
            self.ctrl_msg.brake = 100
            self.ctrl_msg.seq = self.pose_msg.header.seq
            self.ctrl_pub.publish(self.ctrl_msg)
            rospy.sleep(5)



            self.gps_mode = True
            self.delivery_active_b = True

    def parking(self):
        rospy.loginfo(f"🟢🔴🟢🔴🟢🔴🟢🔴🟢🔴🟢🔴🟢🔴🟢🔴🟢🔴")
        self.ctrl_msg.steering = 0
        rospy.sleep(4)



        self.ctrl_msg.accel = 0
        self.ctrl_msg.brake = 100   
        self.ctrl_msg.steering = 0
        self.ctrl_msg.seq = self.pose_msg.header.seq
        self.ctrl_pub.publish(self.ctrl_msg)
        rospy.sleep(2)

        self.ctrl_msg.gear = 0
        self.ctrl_msg.accel = 50
        self.ctrl_msg.brake = 0
        self.ctrl_msg.steering = -25
        self.ctrl_msg.seq = self.pose_msg.header.seq
        self.ctrl_pub.publish(self.ctrl_msg)
        rospy.loginfo(f"🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢")
        rospy.sleep(8)
        

        self.ctrl_msg.accel = 0
        self.ctrl_msg.brake = 100
        self.ctrl_msg.steering = 0
        self.ctrl_msg.seq = self.pose_msg.header.seq
        self.ctrl_pub.publish(self.ctrl_msg)
        rospy.sleep(2)


        self.ctrl_msg.gear = 2
        self.ctrl_msg.accel = 50
        self.ctrl_msg.brake = 0
        self.ctrl_msg.steering = 0
        self.ctrl_msg.seq = self.pose_msg.header.seq
        self.ctrl_pub.publish(self.ctrl_msg)
        rospy.loginfo(f"🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴")
        rospy.sleep(4.5)

        self.ctrl_msg.gear = 2
        self.ctrl_msg.accel = 50
        self.ctrl_msg.brake = 0
        self.ctrl_msg.steering = 20
        self.ctrl_msg.seq = self.pose_msg.header.seq
        self.ctrl_pub.publish(self.ctrl_msg)
        rospy.loginfo(f"🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴")
        rospy.sleep(3)



        # 주행 시작 시간 기록
        start_time = time.time()

        # 4.5초 동안 반복 실행 (도중에 stop_line_detected True 시 즉시 종료)
        while time.time() - start_time < 4.5 and self.stop_line_detected ==False:
            self.ctrl_msg.gear = 2
            self.ctrl_msg.accel = 50
            self.ctrl_msg.brake = 0
            self.ctrl_msg.steering = -35
            self.ctrl_msg.seq = self.pose_msg.header.seq
            self.ctrl_pub.publish(self.ctrl_msg)
            rospy.loginfo("🟢🟢🟢 주행 중 (stop_line_detected=%s)", self.stop_line_detected)

        # 반복문 종료 후 정지 명령
        self.ctrl_msg.accel = 0
        self.ctrl_msg.brake = 100
        self.ctrl_msg.steering = 0
        self.ctrl_msg.seq = self.pose_msg.header.seq
        self.ctrl_pub.publish(self.ctrl_msg)
        rospy.loginfo("🛑 정지 (이유: 시간 만료 or stop_line_detected)")

        
        self.ctrl_msg.accel = 0
        self.ctrl_msg.brake = 100
        self.ctrl_msg.steering = 0
        self.ctrl_msg.seq = self.pose_msg.header.seq
        self.ctrl_pub.publish(self.ctrl_msg)
        rospy.sleep(5)

        self.ctrl_msg.gear = 0
        self.ctrl_msg.accel = 60
        self.ctrl_msg.brake = 0
        self.ctrl_msg.steering = -25
        self.ctrl_msg.seq = self.pose_msg.header.seq
        self.ctrl_pub.publish(self.ctrl_msg)
        rospy.loginfo(f"🟢🟢🟢🟢🟢🟢🟢🟢🔴🔴🔴🔴🔴🔴🔴🔴🔴")
        rospy.sleep(5.9)

        self.ctrl_msg.gear = 0
        self.ctrl_msg.accel = 60
        self.ctrl_msg.brake = 0
        self.ctrl_msg.steering = 30
        self.ctrl_msg.seq = self.pose_msg.header.seq
        self.ctrl_pub.publish(self.ctrl_msg)
        rospy.loginfo(f"🔴🔴🔴🔴🔴🔴🔴🔴🔴🟢🟢🟢🟢🟢🟢🟢🟢")
        rospy.sleep(3)



        self.parking_finish = True


    def is_obstacle_callback(self, msg: Bool):
        val = bool(msg.data)                 # 안전하게 bool화
        self.obstacle_window.append(val)     # ✅ 덮어쓰기(X) → deque에 추가(O)
        self.is_obstacle =msg.data
        rospy.loginfo(f"현재 window 상태{type(self.window5_B)}")
        rospy.loginfo(f"현재 window 상태{type(self.obstacle_window)}")
    



    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()

        
            if self.stop_time is None:
                self.stop_time = current_time
            if self.brake_duration is None:
                self.brake_duration = 0.8 #1.5

            if self.parking_waypoint_start <= self.current_waypoint <= self.parking_waypoint_end and self.parking_finish == False:
                self.obstacle_mode = False
                self.lane_mode = False
                self.gps_mode = True

                # rospy.loginfo(f"🟡🟡🟡🟡🟡🟡🟡🟡🟡🟡🟡🟡🟡🟡🟡🟡🟡🟡🟡🟡")

                

                rospy.loginfo(f"{set(self.obstacle_window)}")
                rospy.loginfo(f"{self.obstacle_window}")
                rospy.loginfo(f"{self.obstacle_window}")
                rospy.loginfo(f"{self.obstacle_window}")

                # self.ctrl_msg.gear = 0
                # self.ctrl_msg.accel = 50
                # self.ctrl_msg.brake = 0
                # self.ctrl_msg.steering = 0
                # self.ctrl_pub.publish(self.ctrl_msg)
                if set(self.obstacle_window) == {False}:
                    self.parking()


            if self.pick_up_waypoint_start <= self.current_waypoint < self.pick_up_waypoint_end:
                self.window5_A.append(self.delivery_a)
                if len(self.window5_A) == self.window5_A.maxlen and len(set(self.window5_A)) == 1:
                    #개수가 5개이고 and set()으로 중복제거 했을 때 개수가 하나라면
                    self.stable_a = self.window5_A[0]     # 동일 값으로 설정 [2]
                    # 필요시 부가 행동: 예) 특정 모드 전환, 로그, 제어값 변경 등
                    rospy.loginfo(f"[stable] 최근 3개 동일 = {self.stable_a}")



            if self.delivery_waypoint_start <= self.current_waypoint < self.delivery_waypoint_end:
                self.window5_B.append(self.delivery_b)
                if len(self.window5_B) == self.window5_B.maxlen and len(set(self.window5_B)) == 1:
                    #개수가 5개이고 and set()으로 중복제거 했을 때 개수가 하나라면
                    self.stable_b = self.window5_B[0]     # 동일 값으로 설정 [2]
                    # 필요시 부가 행동: 예) 특정 모드 전환, 로그, 제어값 변경 등
                    rospy.loginfo(f"[stable] 최근 5개 동일 = {self.stable_b}")


            if self.pick_up_waypoint_start <= self.current_waypoint < self.pick_up_waypoint_end and self.stable_a in ['A1','A2','A3']:
                self.ctrl_msg.accel = 0
                self.ctrl_msg.brake = 200
                self.ctrl_msg.steering = 0
                self.ctrl_msg.seq = self.pose_msg.header.seq
                self.ctrl_pub.publish(self.ctrl_msg) 
                if self.delivery_active_a == False:             
                    if self.stable_a == 'A1':
                        rospy.loginfo(f"🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴{self.delivery_a}🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴")
                        self.delivery_parking_a()
                    elif self.stable_a == 'A2':
                        rospy.loginfo(f"🟡🟡🟡🟡🟡🟡🟡🟡🟡🟡{self.delivery_a}🟡🟡🟡🟡🟡🟡🟡🟡🟡🟡")
                        self.delivery_parking_a()
                    elif self.stable_a == 'A3':
                        rospy.loginfo(f"🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢{self.delivery_a}🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢")
                        self.delivery_parking_a()

            if self.delivery_waypoint_start <= self.current_waypoint < self.delivery_waypoint_end and self.stable_b in ['B1','B2','B3']:
                if self.delivery_active_b == False:
                    if self.stable_a == 'A1' and self.stable_b == 'B1':
                        rospy.loginfo(f"🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴{self.delivery_b}🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴")
                        self.delivery_parking_b()
                    elif self.stable_a == 'A2' and self.stable_b == 'B2':
                        rospy.loginfo(f"🟡🟡🟡🟡🟡🟡🟡🟡🟡🟡{self.delivery_b}🟡🟡🟡🟡🟡🟡🟡🟡🟡🟡")
                        self.delivery_parking_b()
                    elif self.stable_a == 'A3' and self.stable_b == 'B3':
                        rospy.loginfo(f"🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢{self.delivery_b}🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢")
                        self.delivery_parking_b()

            self.update_current_waypoint()
            self.check_u_turn_completion()

            if self.oa_active:
                now = rospy.Time.now().to_sec()

                # 1) 진입 대기
                if now - self.oa_start_time < self.oa_wait_duration:
                    self.ctrl_msg.accel = 0
                    self.ctrl_msg.steering = 0
                    self.ctrl_msg.brake = 120
                    self.ctrl_pub.publish(self.ctrl_msg)
                    rate.sleep()
                    continue

                # 2) OA 조향 (RRT 웨이포인트 사용)
                oa_wps = []
                if self.rrt_waypoints_buffer:
                    oa_wps = list(self.rrt_waypoints_buffer)
                elif self.waypoints:
                    # /newwaypoints가 Waypoint 메시지면 (x,y)로 변환
                    oa_wps = [(w.x, w.y) for w in self.waypoints]

                if oa_wps:
                    target_xy = oa_wps[0]
                    try:
                        wp = Waypoint()
                        wp.x, wp.y = target_xy  # target_xy가 (x, y) 튜플
                        steer_val, _ = self.pure_pursuit_nogps.steering_angle(wp)
                    except ZeroDivisionError:
                        steer_val = 0.0
                    self.ctrl_msg.steering = steer_val
                    self.ctrl_msg.accel = 40 #20
                    self.ctrl_msg.brake = 0
                    self.ctrl_pub.publish(self.ctrl_msg)
                else:
                    # 아무것도 못 받았으면 안전정지
                    self.ctrl_msg.accel = 0
                    self.ctrl_msg.brake = 100
                    self.ctrl_msg.steering = 0
                    self.ctrl_pub.publish(self.ctrl_msg)

                # 3) 구간 이탈 시 GPS 복귀
                if self._outside_current_oa_range(self.current_waypoint):
                    self.oa_active = False
                    self.oa_trigger_wp_range = None
                    self.waypoints = []
                    self.rrt_waypoints_buffer = []
                    self.last_rrt_waypoints_stamp = 0.0
                    self.ctrl_msg.steering = -2.9
                    self.ctrl_pub.publish(self.ctrl_msg)
                    
                    self.switch_to_gps_mode()
                    rospy.loginfo("[OA] 트리거 구간 이탈 → GPS 모드 복귀")

                rate.sleep()
                continue
            # ✅ 시간 기반 정지 처리 추가
            current_stop_waypoint = None
            for wp in self.time_stop_waypoints:
                if abs(self.current_waypoint - wp) <= 4 and wp not in self.completed_stops:
                    current_stop_waypoint = wp
                    break

            # 정지해야 할 웨이포인트에 도달했고, 아직 정지하지 않은 경우
            if current_stop_waypoint is not None and not self.stop_for_time:
                rospy.loginfo(f"웨이포인트 {current_stop_waypoint}에서 3초간 정지합니다.")
                # self.apply_brake()
                self.stop_for_time = True
                self.stop_start_time = time.time()
                continue

            # 정지 중인 경우 시간 체크
            if self.stop_for_time:
                elapsed_time = time.time() - self.stop_start_time
                if elapsed_time >= 4.0:
                    rospy.loginfo(f"웨이포인트 {current_stop_waypoint}에서의 3초 정지가 완료되었습니다.")
                    self.stop_for_time = False
                    self.stop_start_time = None
                    self.completed_stops.add(current_stop_waypoint)
                    # if current_stop_waypoint:
                    #     self.completed_stops.add(current_stop_waypoint)
                    # self.in_gps_mode = True
                else:
                    rospy.loginfo(f"정지 중... {elapsed_time:.1f}/3.0 초")
                    self.ctrl_msg.accel, self.ctrl_msg.brake, self.ctrl_msg.steering = 0, 100, 0
                    self.ctrl_pub.publish(self.ctrl_msg)
                    # self.apply_brake()
                    continue

            if self.last_track_msg_time and (current_time - self.last_track_msg_time) > self.track_timeout:
                if self.obstacle_mode:
                    rospy.loginfo("장애물이 감지되지 않음: 차선 모드로 전환")
                    self.switch_to_lane_mode()
                    self.fixed_velocity_mode = True
                    self.fixed_velocity_start_time = rospy.Time.now().to_sec()

            if self.lane_start_waypoint <= self.current_waypoint < self.lane_end_waypoint:
                if not self.lane_mode and not self.obstacle_mode and not self.u_turn_mode:
                    self.switch_to_lane_mode()
            else:
                if not self.gps_mode and not self.u_turn_mode:
                    self.switch_to_gps_mode()

            # # ✅ V2X 신호 처리 추가
            for waypoint, intersection_info in self.waypoint_intersection_info.items():
                if 0<= waypoint - self.current_waypoint <= 9:
                    self.decide_to_stop_or_drive(intersection_info)

            if self.gps_mode:
                # 기본 GPS 주행 제어
                local_path, self.current_waypoint = findLocalPath(self.global_path, self.pose_msg, self.current_waypoint)
                self.pure_pursuit.getPath(local_path)
                i0 = self.current_waypoint
                i1 = min(i0 + 1, len(self.global_path.poses) - 1)
                p0 = self.global_path.poses[i0].pose.position
                p1 = self.global_path.poses[i1].pose.position
                self.pure_pursuit.target_yaw = math.degrees(math.atan2(p1.y - p0.y, p1.x - p0.x))
                if not hasattr(self.pure_pursuit, "gps_degree"):
                    self.pure_pursuit.gps_degree = 0.0
                try:
                    steer_val, look_steering_point, self.target_angle_index = self.pure_pursuit.steering_angle(0)
                except ZeroDivisionError:
                    rospy.logwarn("ZeroDivisionError 발생 - 조향각을 0으로 설정")
                    steer_val = 0.0
                self.ctrl_msg.steering = steer_val
                self.target_velocity = self.global_path.poses[self.current_waypoint].pose.position.z
                self.ctrl_msg.seq = self.pose_msg.header.seq

            if self.u_turn_mode:
                current_u_turn_time = current_time - self.u_turn_start_time
                if current_u_turn_time < self.u_turn_first_phase_duration:
                    self.ctrl_msg.steering = self.u_turn_steering_first
                else:
                    self.ctrl_msg.steering = self.u_turn_steering_second
                self.global_path_pubtarget_velocity = self.u_turn_velocity
                control_input = self.pid.pid(self.target_velocity, self.curvel_msg.velocity)
                if 0 < control_input <= 200:
                    self.ctrl_msg.accel = control_input
                    self.ctrl_msg.brake = 0
                elif control_input > 200:
                    self.ctrl_msg.accel = self.target_velocity
                    self.ctrl_msg.brake = 0
                else:
                    self.ctrl_msg.accel = 0
                    self.ctrl_msg.brake = min(-control_input, 200)
                if current_time - self.u_turn_start_time >= self.u_turn_duration:
                    self.u_turn_mode = False
                    self.u_turn_completed = True
                    self.switch_to_gps_mode()
                    rospy.loginfo("유턴 완료: GPS 모드로 전환")

            elif self.lane_mode or self.obstacle_mode:
                if self.dynamic_obstacle_detected:
                    self.ctrl_msg.accel = 0
                    self.ctrl_msg.brake = 200
                    self.ctrl_msg.steering = 0
                    self.target_velocity = 0
                if current_time - self.stop_time >= self.brake_duration:
                    self.dynamic_obstacle_detected = False
                    # === [추가] RRT OA 사용 조건 확인 ===
                    current_time = rospy.Time.now().to_sec()
                    if self.rrt_waypoints_buffer:
                        self.waypoints = list(self.rrt_waypoints_buffer)  # (x,y) 튜플 리스트
                        self.fixed_velocity_mode = False
                        rospy.loginfo("RRT OA 웨이포인트로 회피 시작")
                    else:
                        self.fixed_velocity_mode = True
                        self.ctrl_msg.brake = 0
                        self.ctrl_msg.accel = 50

                        rospy.loginfo("기본 회피 로직 사용 (RRT OA 웨이포인트 없음)")

                elif self.obstacle_mode:
                    if self.obstacle_cleared and current_time - self.obstacle_cleared_time >= self.obstacle_clear_delay:
                        self.switch_to_lane_mode()
                        self.rrt_waypoints_buffer = []
                        self.last_rrt_waypoints_stamp = 0.0
                        rospy.loginfo("장애물이 사라진 후 2초 경과: Lane 모드로 전환")
                        self.fixed_velocity_mode = True
                        self.fixed_velocity_start_time = rospy.Time.now().to_sec()
                    if self.fixed_velocity_mode:    
                        if current_time - self.fixed_velocity_start_time <= 1.0:
                            # self.ctrl_msg.steering = 11
                            # self.ctrl_msg.accel = 20
                            # self.ctrl_msg.brake = 0
                            self.target_velocity = 27
                        else:
                            self.fixed_velocity_mode = False
                            self.target_velocity = 27
                            rospy.loginfo("고정 속도 및 조향 유지 완료: 정상 주행 재개")
                    elif self.waypoints:
                        wps = [(w.x, w.y) if hasattr(w, "x") else (w[0], w[1]) for w in self.waypoints]
                        tx, ty = wps[0]
                        wp = Waypoint()
                        wp.x, wp.y = tx, ty
                        try:
                            steer_val, _ = self.pure_pursuit_nogps.steering_angle(wp)
                        except ZeroDivisionError:
                            steer_val = 0.0
                        self.ctrl_msg.steering = steer_val
                        rospy.loginfo("장애물 회피 중 - Steering angle: %.2f", self.ctrl_msg.steering)
                    self.target_velocity = self.target_velocity_ob
                else:
                    if self.emergency_brake and current_time - self.emergency_brake_start_time < 1.0:
                        self.ctrl_msg.accel = 0
                        self.ctrl_msg.brake = 200
                        self.target_velocity = 0
                    else:
                        if self.emergency_brake:
                            self.emergency_brake = False
                            rospy.loginfo("급제동 종료: 정상 주행 재개")
                        self.ctrl_msg.steering = self.calculate_steering_angle()
                        self.target_velocity = self.target_velocity_lane

            # ✅ V2X 정지 상태가 아닐 때만 속도 제어 (조건 수정)
            if (not self.dynamic_obstacle_detected and not self.emergency_brake and 
                not self.u_turn_mode and self.gps_mode and self.target_velocity > 0):
                control_input = self.pid.pid(self.target_velocity, self.curvel_msg.velocity)
                if 0 < control_input <= 200:
                    self.ctrl_msg.accel = control_input
                    self.ctrl_msg.brake = 0
                elif control_input > 200:
                    self.ctrl_msg.accel = self.target_velocity
                    self.ctrl_msg.brake = 0
                elif control_input < -200:
                    self.ctrl_msg.accel = 0
                    self.ctrl_msg.brake = 100
                else:
                    self.ctrl_msg.accel = 0
                    self.ctrl_msg.brake = -control_input

            self.ctrl_pub.publish(self.ctrl_msg)

            if self.u_turn_mode:
                elapsed_time = current_time - self.u_turn_start_time
                rospy.loginfo("유턴 진행 중: %.2f초 경과", elapsed_time)

            current_mode = self.current_mode
            if self.u_turn_mode:
                current_mode = "U-Turn"
            current_time = rospy.Time.now().to_sec()
            rospy.loginfo("stop_line_detected = %s, stopped_for_signal = %s", 
              self.stop_line_detected, self.stopped_for_signal)
            # if self.stop_line_detected and self.stopped_for_signal:
            #     rospy.loginfo("정지선 감지: 5초간 정지 시작")
            #     self.ctrl_msg.accel = 0
            #     self.ctrl_msg.steering = 0
            #     self.ctrl_msg.brake = 80
            #     self.ctrl_msg.seq = self.pose_msg.header.seq
            #     self.ctrl_pub.publish(self.ctrl_msg)

            # if self.current_signal_state == "permissive-clearance" and self.current_remaining_time is not None and self.current_remaining_time > 1.5:
            #     rospy.loginfo("🟢🔴🟢🔴🟢🔴🟢🔴🟢🔴풀가속으로 1초🟢🔴🟢🔴🟢🔴🟢🔴🟢🔴🟢🔴")
            #     rospy.loginfo("🟢🔴🟢🔴🟢🔴🟢🔴🟢🔴풀가속으로 1초🟢🔴🟢🔴🟢🔴🟢🔴🟢🔴🟢🔴")
            #     self.ctrl_msg.accel = 210
            #     self.ctrl_msg.brake = 0
            #     self.ctrl_msg.seq = self.pose_msg.header.seq
            #     self.ctrl_pub.publish(self.ctrl_msg)
            #     rospy.sleep(2)  # 20Hz로 accel 전송

            rospy.loginfo(f"current_text: 표지판 읽기:{self.delivery_b}\n")

            # ✅ V2X 예측 정보를 로그에 추가
            rospy.loginfo("Current Mode: %s, Waypoint: %d, Steering: %.2f, Velocity: %.2f, Target Velocity: %.2f, "
                          "신호 상태: %s, 신호로 인한 정지: %s, GPS 모드: %s, 교차로 거리: %.1fm, 도달시간: %.1fs, 최적속도: %.1fkm/h",
                          current_mode, self.current_waypoint, self.ctrl_msg.steering, self.curvel_msg.velocity, self.target_velocity,
                          self.current_signal_state, self.stopped_for_signal, self.gps_mode,
                          self.distance_to_intersection, self.time_to_intersection, self.optimal_approach_speed)

            rate.sleep()

if __name__ == '__main__':
    try:
        planner = erp_planner()
        planner.run()

    except rospy.ROSInterruptException:
        pass