#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import math
from std_msgs.msg import Float64, Int16, Float32MultiArray
from control_msgs.msg import Velocity
from morai_msgs.msg import CtrlCmd, LocalTrack, LocalControl
from lib.utils_track import purePursuit_nogps, pidController
from vehicle_msgs.msg import WaypointsArray, Waypoint
from vehicle_msgs.msg import Track


class ERPPlanner():
    def __init__(self):
        rospy.init_node('ERP42_track')
        
        self.rate = rospy.Rate(5)

        self.ctrl_msg = CtrlCmd()
        self.local_track_msg = LocalTrack()
        self.points_msg = WaypointsArray()
        self.curvel_msg = Velocity()
        self.index = 0

        self.pure_pursuit = purePursuit_nogps()
        self.pid = pidController()
        self.look_steering_point = Waypoint()  # Steering 계산에 기준이 되는 포인트
        self.max_velocity = 50  # 최대 속도 설정 (50m/s)
        self.min_velocity = 50

        self.ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.local_track_pub = rospy.Publisher('/local_control', LocalTrack, queue_size=1)
        self.steer_way_pub = rospy.Publisher('/steer_waypoint', Waypoint, queue_size=1)

        rospy.Subscriber("/newwaypoints", WaypointsArray, self.points_callback)
        rospy.Subscriber("/ERP42_velocity", Velocity, self.velocity_callback)
        rospy.Subscriber("/track", Track, self.map_callback)

        self.obstacles = {}
        self.dynamic_obstacle_detected = False
        self.stop_time = None
        self.obstacle_distance_threshold = 4.0
        self.brake_duration = 3
        self.dynamic_obstacle_speed_threshold = 0.08  # m/s, 2동적 장애물로 판단할 속도 임계값
        self.new_obstacle_distance_threshold = 0.17  # m, 새로운 장애물로 판단할 거리 임계값

    def calculate_relative_position(self, x, y):
        # 차량의 현재 위치를 (0, 0)으로 가정하고 상대 좌표 계산
        # 실제 구현에서는 차량의 현재 위치를 고려해야 합니다
        return x, y

    def is_new_obstacle(self, x, y):
        for obs_key in self.obstacles:
            obs_x, obs_y, _ = self.obstacles[obs_key]
            if math.sqrt((x - obs_x)**2 + (y - obs_y)**2) < self.new_obstacle_distance_threshold:
                return False
        return True

    def map_callback(self, track):
        """
        장애물 정보를 처리하고 동적 장애물을 감지하는 콜백 함수
        """
        current_time = rospy.Time.now().to_sec()
        dynamic_obstacle_detected = False
        new_obstacle_detected = False
        
        for cone in track.cones:
            rel_x, rel_y = self.calculate_relative_position(cone.x, cone.y)
            cone_key = (round(rel_x, 2), round(rel_y, 2))
            distance = math.sqrt(rel_x**2 + rel_y**2)
            
            if distance < self.obstacle_distance_threshold:
                if cone_key in self.obstacles:
                    # 기존 장애물의 위치와 시간 정보
                    prev_x, prev_y, prev_time = self.obstacles[cone_key]
                    
                    # 속도 계산 (x축과 y축 모두 고려)
                    time_diff = current_time - prev_time
                    if time_diff > 0:
                        speed = math.sqrt((rel_x - prev_x)**2 + (rel_y - prev_y)**2) / time_diff
                        
                        # 동적 장애물 감지
                        if speed > self.dynamic_obstacle_speed_threshold:
                            dynamic_obstacle_detected = True
                            rospy.loginfo(f"동적 장애물 감지 (위치: {cone_key}, 속도: {speed:.2f} m/s)")
                elif self.is_new_obstacle(rel_x, rel_y):
                    # 새로운 장애물 감지
                    new_obstacle_detected = True
                    rospy.loginfo(f"새로운 장애물 감지 (위치: {cone_key})")
                
                # 현재 장애물 정보 업데이트
                self.obstacles[cone_key] = (rel_x, rel_y, current_time)
            else:
                # 거리 임계값을 벗어난 장애물 제거
                self.obstacles.pop(cone_key, None)
        
        # 동적 장애물이나 새로운 장애물이 감지되면 정지
        if dynamic_obstacle_detected or new_obstacle_detected:
            self.dynamic_obstacle_detected = True
            self.stop_time = current_time
            rospy.loginfo("정지 명령 발행")
        
        # 모든 장애물이 제거되었다면 동적 장애물 감지 상태 해제
        if not self.obstacles:
            self.dynamic_obstacle_detected = False

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

    def get_target_velocity(self, curvature):
        """
        곡률에 따른 목표 속도를 계산하는 함수
        """
        # 곡률이 0.005 이상이면 속도를 min_velocity로, 그렇지 않으면 max_velocity로 설정
        if abs(curvature) >= 0.005:
            return self.min_velocity 
        else:
            return self.max_velocity

    def run(self):
        while not rospy.is_shutdown():
            self.pure_pursuit = purePursuit_nogps()

            if self.dynamic_obstacle_detected:
                # 동적 장애물 감지 시 정지 명령
                self.ctrl_msg.accel = 0
                self.ctrl_msg.brake = 200
                self.ctrl_msg.steering = 0

                # 5초가 경과하면 차량을 다시 진행하게 함
                current_time = rospy.Time.now().to_sec()
                if current_time - self.stop_time >= self.brake_duration:
                    self.dynamic_obstacle_detected = False
                    rospy.loginfo("5초 경과: 차량 재출발")
                    # 속도를 max_velocity로 설정하고 브레이크를 해제
                    self.ctrl_msg.accel = self.max_velocity
                    self.ctrl_msg.brake = 0

            if not self.dynamic_obstacle_detected:
                self.ctrl_msg.steering, self.look_steering_point = self.pure_pursuit.steering_angle(self.points_msg)
                self.ctrl_msg.seq += self.index

                # 현재 웨이포인트들로부터 곡률 계산
                curvatures = self.calculate_curvature(self.points_msg.waypoints)

                # 곡률에 따른 목표 속도 계산
                if curvatures:
                    target_velocity = self.get_target_velocity(curvatures[-1])
                else:
                    target_velocity = self.max_velocity

                # PID 제어기를 이용하여 속도 제어
                control_input = self.pid.pid(self.curvel_msg.velocity, target_velocity)

                if control_input > 0:
                    self.ctrl_msg.accel = min(control_input, self.max_velocity)
                    self.ctrl_msg.brake = 0
                else:
                    self.ctrl_msg.accel = 0
                    self.ctrl_msg.brake = min(abs(control_input), 200)

            self.ctrl_pub.publish(self.ctrl_msg)

            rospy.loginfo("현재 속도: {0}, 목표 속도: {1}, 제어 입력: {2}".format(self.curvel_msg.velocity, target_velocity, control_input))

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
        self.points_msg = data

    def velocity_callback(self, speed_data):
        self.curvel_msg = speed_data

if __name__ == '__main__':
    try:
        planner = ERPPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass