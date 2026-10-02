#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64, Int16, Float32MultiArray
from control_msgs.msg import Velocity, Gear
from geometry_msgs.msg import PoseStamped, Point, Pose
from morai_msgs.msg import CtrlCmd, LocalControl
from ublox_msgs.msg import NavPVT
from lib.utils_main import pathReader, findLocalPath, purePursuit, pidController, purePursuit_nogps
from vehicle_msgs.msg import Track, WaypointsArray, Waypoint

class erp_planner():
    def __init__(self):
        rospy.init_node('ERP42_planner')
        
        # Path settings
        self.global_path_name = 'fmtc_uturn_test'
        self.path_name = 'fmtc_uturn_test'
        
        # Message objects
        self.ctrl_msg = CtrlCmd()
        self.pose_msg = Odometry()
        self.curvel_msg = Velocity()
        self.yaw_msg = NavPVT()

        #유턴모드 변수
        self.u_turn_prepare_mode = False
        self.u_turn_mode = False
        self.u_turn_completed = False
        self.u_turn_executed = False  # 유턴 한 번만 실행하기 위한 플래그
        self.u_turn_start_time = None
        self.u_turn_delay_start_time = None
        self.u_turn_delay = 2.0
        self.u_turn_steering = 18
        self.u_turn_velocity = 70
        self.u_turn_duration = 4.5
        self.u_turn_start_waypoint = 0
        self.u_turn_end_waypoint = 1
        self.min_obstacles_for_uturn = 3  # 추가: 유턴을 시작하기 위한 최소 장애물 수
        self.u_turn_steering_first = 45
        self.u_turn_steering_second = 10
        self.u_turn_first_phase_duration = 1.5
        self.u_turn_cleared = False
        self.u_turn_cleared_time = None
        self.obstacle_last_detected_time = None  # 추가

        # Class objects
        self.pure_pursuit = purePursuit()
        self.pure_pursuit_nogps = purePursuit_nogps()
        self.pid = pidController()
        self.path_reader = pathReader('erp42_control_ob')

        self.dynamic_obstacle_detected = False
        self.stop_time = None
        self.brake_duration = 3.0 
        self.obstacle_avoid_start_time = None
        self.obstacle_clear_delay = 2.0  # 2초 대기
        self.obstacle_cleared = False
        self.obstacle_cleared_time = None
        self.obstacle_avoid_cooldown = 7.0  # 7초 쿨다운
        self.obstacle_avoid_start_time = None
        self.can_stop_for_obstacle = True
        self.fixed_velocity_mode = False
        self.fixed_velocity_start_time = None
        
        # Publishers
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.local_control_pub = rospy.Publisher('/local_control', LocalControl, queue_size=1)
        
        # Subscribers
        rospy.Subscriber("/odom/filtered", Odometry, self.pose_callback)
        rospy.Subscriber("/ERP42_velocity", Velocity, self.velocity_callback)
        rospy.Subscriber("/ublox/navpvt", NavPVT, self.yaw_callback)
        rospy.Subscriber("/lane_center_points", Point, self.lane_center_callback)
        
        # Path and waypoint variables
        self.global_path = self.path_reader.read_txt(self.path_name + ".txt")
        self.current_waypoint = 0

        self.obstacle_mode = False
        self.obstacles = []
        rospy.Subscriber("/track", Track, self.track_callback)
        rospy.Subscriber("/newwaypoints", WaypointsArray, self.waypoints_callback)
        
        # Mode flags
        self.gps_mode = True
        self.lane_mode = False
        self.current_mode = "GPS"
        
        # Lane detection variables
        self.lane_center_points = []
        self.max_center_points = 15  # 저장할 최대 중앙 차선 점 수
        
        self.waypoints = []
        
        # Control parameters
        self.target_velocity_gps = self.global_path.poses[self.current_waypoint].pose.position.z  # km/h for GPS mode
        self.target_velocity_lane = 80  # km/h for Lane mode
        self.target_velocity_ob = 60
        
        # Mode transition waypoints
        self.lane_start_waypoint = 0
        self.lane_end_waypoint = 9999

        # 새로운 플래그 추가
        self.emergency_brake = False
        self.emergency_brake_start_time = None

        self.last_track_msg_time = None
        self.track_timeout = 0.22  # 0.22초 동안 새로운 메시지가 없으면 장애물이 없다고 판단

    def pose_callback(self, data):
        self.pose_msg = data
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

    def track_callback(self, msg):
        self.last_track_msg_time = rospy.Time.now().to_sec()
        self.obstacles = msg.cones
        current_obstacle_count = len(self.obstacles)

        if (self.u_turn_start_waypoint <= self.current_waypoint <= self.u_turn_end_waypoint and 
            not self.u_turn_completed and 
            not self.u_turn_executed):  # 유턴을 아직 실행하지 않았을 때만
            
            negative_y_obstacles = sum(1 for obstacle in self.obstacles if obstacle.y < 0)
            
            if current_obstacle_count >= self.min_obstacles_for_uturn:
                if negative_y_obstacles >= self.min_obstacles_for_uturn:
                    # y좌표가 음수인 장애물이 3개 이상일 경우 2초 대기 후 유턴
                    if not self.u_turn_prepare_mode:
                        self.u_turn_prepare_mode = True
                        self.u_turn_delay_start_time = self.last_track_msg_time
                        self.u_turn_executed = True  # 여기에 추가
                        rospy.loginfo(f"음수 y좌표 장애물 {negative_y_obstacles}개 감지: 2초 후 유턴 시작")
                else:
                    # 총 장애물이 3개 이상이지만 음수 y좌표가 3개 미만일 경우 즉시 유턴
                    self.u_turn_prepare_mode = False
                    self.u_turn_executed = True  # 여기에 추가
                    self.start_u_turn()
                    rospy.loginfo(f"전체 장애물 {current_obstacle_count}개 감지: 즉시 유턴 시작")
        # 일반 장애물 처리
        if current_obstacle_count == 0:
            if self.obstacle_mode and self.obstacle_cleared:
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
        
        steering_angle = (error / image_center) * 15

        return max(min(steering_angle, 20), -20)

    def switch_to_obstacle_mode(self):
        self.obstacle_mode = True
        self.lane_mode = False
        self.gps_mode = False
        self.current_mode = "Obstacle"
        self.obstacle_avoid_start_time = rospy.Time.now().to_sec()
        rospy.loginfo("장애물 회피 모드로 전환")

    # def switch_to_lane_mode(self):
    #     self.obstacle_mode = False
    #     self.lane_mode = True
    #     self.gps_mode = False
    #     self.current_mode = "Lane"
    #     self.dynamic_obstacle_detected = False

    def switch_to_lane_mode(self):
        self.obstacle_mode = False
        self.lane_mode = True
        self.gps_mode = False
        self.current_mode = "Lane"
        self.dynamic_obstacle_detected = False
        
        # waypoints 리스트 비우기 추가
        self.waypoints = []

        # 고정 속도 및 조향 모드 활성화
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
        self.u_turn_prepare_mode = False  # 여기서 False로 설정됨
        self.u_turn_mode = True
        self.u_turn_start_time = rospy.Time.now().to_sec()
        rospy.loginfo("유턴 모드 시작")

    def start_u_turn(self):
        self.gps_mode = False
        self.u_turn_prepare_mode = False
        self.u_turn_mode = True
        self.u_turn_start_time = rospy.Time.now().to_sec()
        rospy.loginfo("유턴 모드 시작")

    def check_u_turn_completion(self):
        if self.u_turn_mode:
            current_time = rospy.Time.now().to_sec()
            if current_time - self.u_turn_start_time >= self.u_turn_duration:
                self.u_turn_mode = False
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

    def run(self):
        rate = rospy.Rate(20)
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()

            self.update_current_waypoint()
            self.check_u_turn_completion()

            # 유턴 모드 처리
            if self.u_turn_prepare_mode and not self.u_turn_mode:
                if current_time - self.u_turn_delay_start_time >= self.u_turn_delay:
                    self.start_u_turn()
                    rospy.loginfo("2초 대기 완료: 유턴 모드 시작")

            
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

            if self.gps_mode:
                local_path, self.current_waypoint = findLocalPath(self.global_path, self.pose_msg, self.current_waypoint)
                self.pure_pursuit.getPath(local_path)
                self.ctrl_msg.steering, look_steering_point, self.target_angle_index = self.pure_pursuit.steering_angle(self.current_waypoint)
                target_velocity = self.global_path.poses[self.current_waypoint].pose.position.z
                self.ctrl_msg.seq = self.pose_msg.header.seq

            elif self.u_turn_mode:
                current_u_turn_time = current_time - self.u_turn_start_time
                
                if not self.u_turn_prepare_mode:  # 2초 대기 후 유턴 (-y 장애물 3개 이상)
                    self.ctrl_msg.steering = self.u_turn_steering
                else:  # 즉시 유턴 (전체 3개 이상, -y 3개 미만)
                    if current_u_turn_time < self.u_turn_first_phase_duration:
                        self.ctrl_msg.steering = self.u_turn_steering_first
                    else:
                        self.ctrl_msg.steering = self.u_turn_steering_second
                
                target_velocity = self.u_turn_velocity

                # PID 컨트롤러를 사용하여 속도 제어
                control_input = self.pid.pid(target_velocity, self.curvel_msg.velocity)
                if 0 < control_input <= 200:
                    self.ctrl_msg.accel = control_input
                    self.ctrl_msg.brake = 0
                elif control_input > 200:
                    self.ctrl_msg.accel = target_velocity
                    self.ctrl_msg.brake = 0
                else:
                    self.ctrl_msg.accel = 0
                    self.ctrl_msg.brake = min(-control_input, 200)
                
                # 유턴 완료 시간 체크
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
                    target_velocity = 0

                    if current_time - self.stop_time >= self.brake_duration:
                        self.dynamic_obstacle_detected = False
                        self.switch_to_obstacle_mode()
                        rospy.loginfo("5초 경과: 장애물 회피 모드 시작")

                elif self.obstacle_mode:
                    if self.obstacle_cleared and current_time - self.obstacle_cleared_time >= self.obstacle_clear_delay:
                        self.switch_to_lane_mode()
                        rospy.loginfo("장애물이 사라진 후 2초 경과: Lane 모드로 전환")
                        
                        self.fixed_velocity_mode = True
                        self.fixed_velocity_start_time = rospy.Time.now().to_sec()
                    
                    if self.fixed_velocity_mode:
                        if current_time - self.fixed_velocity_start_time <= 1.0:
                            self.ctrl_msg.steering = 11 # 고정 조향각
                            self.ctrl_msg.accel = 20    # 고정 속도
                            self.ctrl_msg.brake = 0
                            target_velocity = 20
                        else:
                            # 1초 후 고정 모드 해제
                            self.fixed_velocity_mode = False
                            rospy.loginfo("고정 속도 및 조향 유지 완료: 정상 주행 재개")
                    
                    elif self.waypoints:
                        target_waypoint = self.waypoints[0]
                        self.ctrl_msg.steering, _ = self.pure_pursuit_nogps.steering_angle(target_waypoint)
                        rospy.loginfo("장애물 회피 중 - Steering angle: %.2f", self.ctrl_msg.steering)
                    target_velocity = self.target_velocity_ob

                else:  # Lane 모드
                    if self.emergency_brake and current_time - self.emergency_brake_start_time < 1.0:
                        self.ctrl_msg.accel = 0
                        self.ctrl_msg.brake = 200
                        target_velocity = 0
                    else:
                        if self.emergency_brake:
                            self.emergency_brake = False
                            rospy.loginfo("급제동 종료: 정상 주행 재개")
                        self.ctrl_msg.steering = self.calculate_steering_angle()
                        target_velocity = self.target_velocity_lane

            if not self.dynamic_obstacle_detected and not self.emergency_brake:
                control_input = self.pid.pid(target_velocity, self.curvel_msg.velocity)
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

            self.ctrl_pub.publish(self.ctrl_msg)

            # 유턴 관련 로그 추가
            if self.u_turn_mode:
                elapsed_time = current_time - self.u_turn_start_time
                rospy.loginfo("유턴 진행 중: %.2f초 경과", elapsed_time)
            elif self.u_turn_prepare_mode:
                if self.obstacle_last_detected_time:
                    time_since_last_obstacle = current_time - self.obstacle_last_detected_time
                    rospy.loginfo("유턴 준비 중: 마지막 장애물 감지 후 %.2f초 경과", time_since_last_obstacle)

            # 현재 모드 결정
            current_mode = self.current_mode
            if self.u_turn_mode:
                current_mode = "U-Turn"
            elif self.u_turn_prepare_mode:
                current_mode = "U-Turn Prepare"

            # 최종 상태 로그
            rospy.loginfo("Current Mode: %s, Waypoint: %d, Steering: %.2f, Velocity: %.2f, Target Velocity: %.2f", 
                        current_mode, self.current_waypoint, self.ctrl_msg.steering, self.curvel_msg.velocity, target_velocity)

            rate.sleep()

if __name__ == '__main__':
    try:
        planner = erp_planner()
        planner.run()
    except rospy.ROSInterruptException:
        pass