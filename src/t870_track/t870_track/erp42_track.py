#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float64, Int16, Float32MultiArray
# 참고: 'control_msgs/Velocity'는 표준 메시지가 아니므로, 'vehicle_msgs/Velocity'를 사용한다고 가정합니다.
from vehicle_msgs.msg import Velocity
from morai_msgs.msg import CtrlCmd, LocalTrack
# from lib.utils_track import purePursuit_nogps as purePursuit, pidController # ROS2 패키지 구조에 맞게 수정 필요
from .lib.utils_track import purePursuit_nogps as purePursuit, pidController

from vehicle_msgs.msg import WaypointsArray, Waypoint

class ERPPlanner(Node):
    def __init__(self):
        super().__init__('ERP42_track')
        
        self.ctrl_msg = CtrlCmd()
        self.local_track_msg = LocalTrack()
        self.points_msg = WaypointsArray()
        self.curvel_msg = Velocity()
        self.index = 0

        self.pure_pursuit = purePursuit()
        self.pid = pidController()
        self.look_steering_point = Waypoint()
        self.constant_velocity = 60.0
        self.curvature_threshold = 4.2
        self.brake_duration = 2.0
        self.brake_cooldown = 0.1
        self.last_brake_time = 0.0
        self.min_brake = 0.0

        # ROS2 퍼블리셔
        self.ctrl_pub = self.create_publisher(CtrlCmd, '/ctrl_cmd', 10)
        self.local_track_pub = self.create_publisher(LocalTrack, '/local_control', 10)
        self.steer_way_pub = self.create_publisher(Waypoint, '/steer_waypoint', 10)

        # ROS2 구독자
        self.create_subscription(WaypointsArray, "/newwaypoints", self.points_callback, 10)
        self.create_subscription(Velocity, "/ERP42_velocity", self.velocity_callback, 10)

        # ROS2 타이머를 사용하여 주기적으로 run 메소드 실행
        self.timer = self.create_timer(1.0/20.0, self.run)
        self.get_logger().info("ERP42 Track Node has been started.")

    def calculate_curvature(self, waypoints):
        curvatures = []
        num_points = min(len(waypoints), 3)
        for i in range(num_points - 1):
            x1, y1 = waypoints[i].x, waypoints[i].y
            x2, y2 = waypoints[i+1].x, waypoints[i+1].y
            dx = x2 - x1
            dy = y2 - y1
            dist_sq = dx**2 + dy**2
            if dist_sq == 0:
                continue
            curvature = (dy * x1 - dx * y1) / (dist_sq**(3/2))
            curvatures.append(curvature)
        return curvatures

    def run(self):
        if not self.points_msg.waypoints:
            self.get_logger().info("Waiting for waypoints...", throttle_duration_sec=1)
            return

        self.pure_pursuit.getVelStatus(self.curvel_msg)

        self.ctrl_msg.steering, self.look_steering_point = self.pure_pursuit.steering_angle(self.points_msg)
        
        curvatures = self.calculate_curvature(self.points_msg.waypoints)
        current_time = self.get_clock().now().nanoseconds / 1e9
        time_since_last_brake = current_time - self.last_brake_time
        
        current_speed = self.curvel_msg.velocity 

        if curvatures and abs(curvatures[-1]) >= self.curvature_threshold and time_since_last_brake >= self.brake_cooldown:
            self.ctrl_msg.accel = 40.0
            self.ctrl_msg.brake = 0.0
            self.last_brake_time = current_time
        elif time_since_last_brake < self.brake_duration:
            self.ctrl_msg.accel = 40.0
            self.ctrl_msg.brake = 0.0
        else:
            self.ctrl_msg.accel = self.constant_velocity
            self.ctrl_msg.brake = 0.0

        self.ctrl_pub.publish(self.ctrl_msg)

        self.get_logger().info(f"Speed: {current_speed:.2f}, Accel: {self.ctrl_msg.accel}, Brake: {self.ctrl_msg.brake}", throttle_duration_sec=1)

        steering_angle = self.ctrl_msg.steering
        
        self.local_track_msg.velocity = self.curvel_msg.velocity / 10.0
        self.local_track_msg.steer_angle = steering_angle
        self.local_track_msg.way_x = self.look_steering_point.x
        self.local_track_msg.way_y = self.look_steering_point.y

        self.local_track_pub.publish(self.local_track_msg)
        self.steer_way_pub.publish(self.look_steering_point)

    def points_callback(self, data):
        self.points_msg = data

    def velocity_callback(self, speed_data):
        self.curvel_msg = speed_data

def main(args=None):
    rclpy.init(args=args)
    try:
        planner = ERPPlanner()
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
