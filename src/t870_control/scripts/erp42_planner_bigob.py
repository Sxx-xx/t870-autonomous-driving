#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from nav_msgs.msg import Path, Odometry
from control_msgs.msg import Velocity, Gear
from morai_msgs.msg import CtrlCmd, LocalControl
from ublox_msgs.msg import NavPVT
from lib.utils_main import pathReader, findLocalPath, purePursuit, pidController
from vehicle_msgs.msg import Track, WaypointsArray, Waypoint

class erp_planner():
    def __init__(self):
        rospy.init_node('ERP42_planner')

        # 경로 설정
        self.global_path_name = '2025univ2'
        self.path_name = '2025univ2'

        # 메시지 객체 초기화
        self.ctrl_msg = CtrlCmd()
        self.pose_msg = Odometry()
        self.curvel_msg = Velocity()
        self.yaw_msg = NavPVT()

        # 클래스 객체 초기화
        self.pure_pursuit = purePursuit()
        self.pid = pidController()
        self.path_reader = pathReader('erp42_control_ob')

        # 퍼블리셔 설정
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.local_control_pub = rospy.Publisher('/local_control', LocalControl, queue_size=1)

        # 서브스크라이버 설정
        rospy.Subscriber("/odom/filtered", Odometry, self.pose_callback)
        rospy.Subscriber("/ERP42_velocity", Velocity, self.velocity_callback)
        rospy.Subscriber("/ublox/navpvt", NavPVT, self.yaw_callback)
        rospy.Subscriber("/track", Track, self.track_callback)
        rospy.Subscriber("/newwaypoints", WaypointsArray, self.waypoints_callback)

        # 경로 및 웨이포인트 변수 초기화
        self.global_path = self.path_reader.read_txt(self.path_name + ".txt")
        self.current_waypoint = 0

        # 모드 플래그 초기화
        self.gps_mode = True
        self.bigob_mode = False
        self.current_mode = "GPS"

        self.waypoints = []

        # 제어 파라미터 설정
        # self.target_velocity_gps = self.global_path.poses[self.current_waypoint].pose.position.z # GPS 모드에서의 목표 속도 (km/h)
        self.target_velocity_ob = 70     # 장애물 회피 모드에서의 목표 속도 (km/h)

        # 장애물 감지 시 전환할 웨이포인트 범위 설정
        self.obstacle_start_waypoint = 420
        self.obstacle_end_waypoint = 640

        # 조향각 방향 설정 (초기값: +20도)
        self.next_steering_direction = 25  # 다음 장애물 감지 시 사용할 조향각 방향
        self.next_steering_direction2 = 22
        self.steering_direction = None     # 현재 장애물 회피 시 사용할 조향각
        self.bigob_mode_start_time = None

        # 장애물 감지 타임아웃 설정
        self.last_track_msg_time = None
        self.track_timeout = 0.22  # 0.22초 동안 새로운 메시지가 없으면 장애물이 없다고 판단

        # 장애물 회피 단계 설정
        self.obstacle_stage = 0  # 0: 초기화, 1: +20도, 2: GPS 따라감, 3: -20도

        # 장애물 회피 완료 플래그
        self.obstacle_avoidance_done = False  # 장애물 회피 완료 여부

    def pose_callback(self, data):
        self.pose_msg = data
        self.pure_pursuit.getPoseStatus(data)

    def velocity_callback(self, speed_data):
        self.curvel_msg = speed_data
        self.pure_pursuit.getVelStatus(speed_data)

    def yaw_callback(self, yaw_data):
        self.yaw_msg = yaw_data
        self.pure_pursuit.getYawStatus(yaw_data)

    def waypoints_callback(self, msg):
        self.waypoints = msg.waypoints

    def track_callback(self, msg):
        self.last_track_msg_time = rospy.Time.now().to_sec()
        self.obstacles = msg.cones
        current_obstacle_count = len(self.obstacles)

        if current_obstacle_count == 0:
            if self.bigob_mode and not getattr(self, 'obstacle_cleared', False):
                self.obstacle_cleared = True
                rospy.loginfo("장애물이 사라짐: GPS 모드로 전환 대기 시작")
        else:
            # 장애물 회피 모드가 활성화되지 않고, 아직 장애물 회피를 하지 않은 경우에만 처리
            if (not self.bigob_mode and
                self.obstacle_start_waypoint <= self.current_waypoint <= self.obstacle_end_waypoint and
                self.gps_mode and
                not self.obstacle_avoidance_done):
                self.switch_to_bigob_mode()
                rospy.loginfo("특정 웨이포인트 내에서 장애물 감지: 장애물 회피 모드로 전환")
            else:
                rospy.loginfo("장애물이 감지되었으나, 현재 웨이포인트가 범위 외이거나 이미 장애물 회피 중이므로 무시")

    def switch_to_bigob_mode(self):
        self.bigob_mode = True
        self.gps_mode = False
        self.current_mode = "Obstacle"
        self.bigob_mode_start_time = rospy.Time.now().to_sec()
        self.steering_direction = self.next_steering_direction
        self.obstacle_stage = 1  # 장애물 회피 단계 초기화
        rospy.loginfo("장애물 회피 모드로 전환")
        rospy.loginfo(f"장애물 회피 단계 1: 조향각 설정: {self.steering_direction}도")

    def switch_to_gps_mode(self):
        self.bigob_mode = False
        self.gps_mode = True
        self.current_mode = "GPS"
        self.bigob_mode_start_time = None
        self.obstacle_stage = 0  # 장애물 회피 단계 초기화
        # 조향각 방향을 반전시켜 다음 장애물 감지 시 반대 방향 적용
        self.next_steering_direction *= -1.0
        rospy.loginfo("GPS 기반 제어 모드로 전환")
        rospy.loginfo(f"다음 장애물 회피 시 조향각 방향: {self.next_steering_direction}도")
        self.obstacle_avoidance_done = True  # 장애물 회피 완료 표시

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
        rate = rospy.Rate(20)  # 20Hz

        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()

            self.update_current_waypoint()

            # 장애물 회피 완료 후 특정 웨이포인트를 지나면 플래그 재설정 (필요에 따라 사용)
            # if self.current_waypoint > self.obstacle_end_waypoint:
            #     self.obstacle_avoidance_done = False

            if self.gps_mode:
                # GPS 모드에서는 Pure Pursuit를 사용하여 조향각 계산
                local_path, self.current_waypoint = findLocalPath(self.global_path, self.pose_msg, self.current_waypoint)
                self.pure_pursuit.getPath(local_path)
                steering_angle, _, self.target_angle_index = self.pure_pursuit.steering_angle(self.current_waypoint)
                self.ctrl_msg.steering = steering_angle
                target_velocity = self.global_path.poses[self.current_waypoint].pose.position.z 
                self.ctrl_msg.seq = self.pose_msg.header.seq

            elif self.bigob_mode:
                if self.bigob_mode_start_time is not None:
                    elapsed_time = current_time - self.bigob_mode_start_time

                    if self.obstacle_stage == 1:
                        if elapsed_time < 1.5:
                            # 첫 1.5초 동안 +20도 조향
                            self.ctrl_msg.steering = self.steering_direction
                            rospy.loginfo(f"장애물 회피 중 - 단계 1: 조향각: {self.ctrl_msg.steering}도, 시간: {elapsed_time:.2f}초")
                        else:
                            # 다음 단계로 이동
                            self.obstacle_stage = 2
                            self.bigob_mode_start_time = current_time  # 타이머 재설정

                    elif self.obstacle_stage == 2:
                        if elapsed_time < 4.5:
                            # 다음 4.5초 동안 GPS 따라감
                            local_path, self.current_waypoint = findLocalPath(self.global_path, self.pose_msg, self.current_waypoint)
                            self.pure_pursuit.getPath(local_path)
                            steering_angle, _, self.target_angle_index = self.pure_pursuit.steering_angle(self.current_waypoint)
                            self.ctrl_msg.steering = steering_angle
                            rospy.loginfo(f"장애물 회피 중 - 단계 2: GPS 따라감, 조향각: {self.ctrl_msg.steering:.2f}도, 시간: {elapsed_time:.2f}초")
                        else:
                            # 다음 단계로 이동
                            self.obstacle_stage = 3
                            self.bigob_mode_start_time = current_time  # 타이머 재설정

                    elif self.obstacle_stage == 3:
                        if elapsed_time < 1.7:
                            # 다음 1.8초 동안 -20도 조향
                            self.ctrl_msg.steering = -self.next_steering_direction2
                            rospy.loginfo(f"장애물 회피 중 - 단계 3: 조향각: {self.ctrl_msg.steering}도, 시간: {elapsed_time:.2f}초")
                        else:
                            # 장애물 회피 완료 후 GPS 모드로 전환
                            self.switch_to_gps_mode()
                            rospy.loginfo(f"장애물 회피 완료: 조향각 방향을 {self.next_steering_direction}도로 변경")
                else:
                    # 장애물 모드 시작 시 조향각 설정
                    self.switch_to_bigob_mode()

                target_velocity = self.target_velocity_ob

            # 속도 제어 (PID 제어 사용)
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

            rospy.loginfo("현재 모드: %s, 웨이포인트: %d, 조향각: %.2f, 속도: %.2f, 목표 속도: %.2f",
                          self.current_mode, self.current_waypoint, self.ctrl_msg.steering, self.curvel_msg.velocity, target_velocity)

            rate.sleep()

if __name__ == '__main__':
    try:
        planner = erp_planner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
