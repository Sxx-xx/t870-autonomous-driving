#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import math
from std_msgs.msg import Float64, Int16, Float32MultiArray
from control_msgs.msg import Velocity
from morai_msgs.msg import CtrlCmd, LocalTrack, LocalControl
from lib.utils_track import purePursuit_nogps as purePursuit, pidController
from vehicle_msgs.msg import WaypointsArray, Waypoint

class ERPPlanner():
    def __init__(self):
        rospy.init_node('ERP42_track')
        
        self.rate = rospy.Rate(20)

        self.ctrl_msg = CtrlCmd()
        self.local_track_msg = LocalTrack()
        self.points_msg = WaypointsArray()
        self.curvel_msg = Velocity()
        self.index = 0

        self.pure_pursuit = purePursuit()
        self.pid = pidController()
        self.look_steering_point = Waypoint()  # Steering 계산에 기준이 되는 포인트
        self.constant_velocity = 40  # 일정한 속도 설정 (6km/h)
        self.curvature_threshold = 2.8   # 곡률 임계값 4.2
        self.brake_duration = 2.0  # 브레이크 지속 시간 (초)
        self.brake_cooldown = 0.1  # 브레이크 쿨다운 시간 (초)
        self.last_brake_time = 0  # 마지막으로 브레이크를 밟은 시간
        self.min_brake = 0

        self.ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.local_track_pub = rospy.Publisher('/local_control', LocalTrack, queue_size=1)
        self.steer_way_pub = rospy.Publisher('/steer_waypoint', Waypoint, queue_size=1)

        rospy.Subscriber("/newwaypoints", WaypointsArray, self.points_callback)
        rospy.Subscriber("/ERP42_velocity", Velocity, self.velocity_callback)

    def calculate_curvature(self, waypoints):
        """
        웨이포인트들로부터 곡률을 계산하는 함수
        - 가장 가까운 3개의 웨이포인트를 사용하여 곡률을 계산합니다.
        """
        curvatures = []
        num_points = min(len(waypoints), 3)  # 최대 3개의 웨이포인트 사용
        for i in range(num_points - 1):
            x1, y1 = waypoints[i].x, waypoints[i].y
            x2, y2 = waypoints[i+1].x, waypoints[i+1].y
            dx = x2 - x1
            dy = y2 - y1
            dist_sq = dx**2 + dy**2
            if dist_sq == 0:  # 0으로 나누는 것을 피하기 위한 처리
                continue
            curvature = (dy * x1 - dx * y1) / (dist_sq**(3/2))

            curvatures.append(curvature)
        return curvatures

    def run(self):
        while not rospy.is_shutdown():
            self.pure_pursuit.getVelStatus(self.curvel_msg)

            self.ctrl_msg.steering, self.look_steering_point = self.pure_pursuit.steering_angle(self.points_msg)
            self.ctrl_msg.seq += self.index

            # 현재 웨이포인트들로부터 곡률 계산
            curvatures = self.calculate_curvature(self.points_msg.waypoints)

            current_time = rospy.get_time()
            time_since_last_brake = current_time - self.last_brake_time

            current_speed = self.curvel_msg.velocity # km/h로 변환 //////////////

            if curvatures and abs(curvatures[-1]) >= self.curvature_threshold and time_since_last_brake >= self.brake_cooldown:
                # 곡률이 임계값 이상이고, 마지막 브레이크 후 쿨다운 시간이 지났다면
                self.ctrl_msg.accel = 10
                # 현재 속도에 따라 브레이크 값 설정, 최소값 보장
                self.ctrl_msg.brake = 0
                self.last_brake_time = current_time
            elif time_since_last_brake < self.brake_duration:
                # 브레이크를 밟은 지 정해진 지속 시간 이내라면 계속 브레이크 유지
                self.ctrl_msg.accel = 10
                self.ctrl_msg.brake = 0
            else:
                # 일정한 속도로 주행
                self.ctrl_msg.accel = self.constant_velocity
                self.ctrl_msg.brake = 0

            self.ctrl_pub.publish(self.ctrl_msg)

            rospy.loginfo("현재 속도: {0:.2f}, 가속: {1}, 브레이크: {2}, 마지막 브레이크 후 경과 시간: {3:.2f}".format(
                current_speed, self.ctrl_msg.accel, self.ctrl_msg.brake, time_since_last_brake))

            self.steering_angle = self.ctrl_msg.steering
            self.control_input = self.ctrl_msg.accel

            self.local_track_msg.seq = self.ctrl_msg.seq
            self.local_track_msg.velocity = self.curvel_msg.velocity / 10
            self.local_track_msg.steer_angle = self.steering_angle
            self.local_track_msg.way_x = self.look_steering_point.x
            self.local_track_msg.way_y = self.look_steering_point.y

            self.local_track_pub.publish(self.local_track_msg)
            self.steer_way_pub.publish(self.look_steering_point)
            self.rate.sleep()

    def points_callback(self, data):
        self.points_msg = WaypointsArray()
        self.points_msg = data

    def velocity_callback(self, speed_data):
        self.curvel_msg = speed_data

if __name__ == '__main__':
    try:
        planner = ERPPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass