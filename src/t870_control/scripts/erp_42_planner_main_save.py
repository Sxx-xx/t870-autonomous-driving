# # !/usr/bin/env python3
# # -*- coding: utf-8 -*-

# import rospy
# import math
# from nav_msgs.msg import Path, Odometry
# from std_msgs.msg import Float64, Int16, Float32MultiArray
# from control_msgs.msg import Velocity, Gear
# from geometry_msgs.msg import Point
# from morai_msgs.msg import CtrlCmd, LocalControl
# from ublox_msgs.msg import NavPVT
# from lib.utils_main import pathReader, findLocalPath, purePursuit, pidController, purePursuit_nogps
# from vehicle_msgs.msg import Track, WaypointsArray, Waypoint
# from v2x_msgs.msg import Spat_new
# from std_msgs.msg import Bool  # 상단에 추가



# class erp_planner():
#     def __init__(self):
#         rospy.init_node('ERP42_planner')
#         # __init__ 내부
#         self.stop_line_detected = False
#         self.last_stop_line_msg_time = 0

#         rospy.Subscriber("/stop_line", Bool, self.stop_line_callback)



        
#         # Path settings
#         self.global_path_name = 'left_traffic'
#         self.path_name = 'left_traffic'  

#         #CITS
#         self.target_signal_group = 4  # 관심 있는 signal group ID
#         self.current_signal_state = None  # 현재 신호등 상태
#         rospy.Subscriber('/v2x_data', Spat_new, self.v2x_callback)
        
#         # Message objects
#         self.ctrl_msg = CtrlCmd()
#         self.ctrl_msg.longlCmdType = 2
#         self.pose_msg = Odometry()
#         self.curvel_msg = Velocity()
#         self.yaw_msg = NavPVT()

#         # U-turn mode variables
#         self.u_turn_mode = False
#         self.u_turn_completed = False
#         self.u_turn_executed = False
#         self.u_turn_start_time = None
#         self.u_turn_velocity = 70
#         self.u_turn_duration = 5.0
#         self.u_turn_start_waypoint =0    #314 /2025에 맞게 수정이 필요합니다.
#         self.u_turn_end_waypoint =-1  #418 /2025에 맞게 수정이 필요합니다.
#         self.min_obstacles_for_uturn = 3
#         self.u_turn_steering_first = 50
#         self.u_turn_steering_second = 10
#         self.u_turn_first_phase_duration = 4.5
#         self.u_turn_cleared = False
#         self.u_turn_cleared_time = None
#         self.obstacle_last_detected_time = None

#         # Class objects
#         self.pure_pursuit = purePursuit()
#         self.pure_pursuit_nogps = purePursuit_nogps()
#         self.pid = pidController()
#         self.path_reader = pathReader('erp42_control_ob')

#         self.dynamic_obstacle_detected = False
#         self.stop_time = None
#         self.brake_duration = 3.0
#         self.obstacle_avoid_start_time = None
#         self.obstacle_clear_delay = 2.0
#         self.obstacle_cleared = False
#         self.obstacle_cleared_time = None
#         self.obstacle_avoid_cooldown = 7.0
#         self.obstacle_avoid_start_time = None
#         self.can_stop_for_obstacle = True
#         self.fixed_velocity_start_time = None
        
#         # Publishers
#         self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
#         self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
#         self.ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
#         self.local_control_pub = rospy.Publisher('/local_control', LocalControl, queue_size=1)
        
#         # Subscribers
#         rospy.Subscriber("/odom/filtered", Odometry, self.pose_callback)
#         rospy.Subscriber("/ERP42_velocity", Velocity, self.velocity_callback)
#         rospy.Subscriber("/ublox/navpvt", NavPVT, self.yaw_callback)
#         rospy.Subscriber("/lane_center_points", Point, self.lane_center_callback)
#         rospy.Subscriber("/track", Track, self.track_callback)
#         rospy.Subscriber("/newwaypoints", WaypointsArray, self.waypoints_callback)
#         rospy.Subscriber("/stop_line", Bool, self.stop_line_callback)
#         # Path and waypoint variables
#         self.global_path = self.path_reader.read_txt(self.path_name + ".txt")
#         self.current_waypoint = 0

#         # Mode flags
#         self.obstacle_mode = False
#         self.obstacles = [] 
#         self.gps_mode = True
#         self.lane_mode = False
#         self.current_mode = "GPS"
        
#         # Lane detection variables
#         self.lane_center_points = []
#         self.max_center_points = 15
#         self.waypoints = []
        
#         # Control parameters /2025에 맞게 수정이 필요합니다.
#         self.target_velocity_gps = self.global_path.poses[self.current_waypoint].pose.position.z
#         self.target_velocity_lane = 80
#         self.target_velocity_ob = 60
        
#         # Mode transition waypoints /2025에 맞게 수정이 필요합니다.
#         self.lane_start_waypoint = 600  #737   #677   
#         self.lane_end_waypoint =870    #1009  #833

#         # Emergency brake
#         self.emergency_brake = False
#         self.emergency_brake_start_time = None
#         self.last_track_msg_time = None
#         self.track_timeout = 0.22

#         rospy.loginfo("✅ ERP42 카메라 + C-ITS 제어 노드 시작")


#     # 콜백 함수
#     def stop_line_callback(self, msg):
#         self.stop_line_detected = msg.data
#         self.last_stop_line_msg_time = rospy.Time.now().to_sec()



#     def pose_callback(self, data):
#         self.pose_msg = data
#         self.pure_pursuit.getPoseStatus(data)

#     def velocity_callback(self, speed_data):
#         self.curvel_msg = speed_data
#         self.pure_pursuit.getVelStatus(speed_data)

#     def yaw_callback(self, yaw_data):
#         self.yaw_msg = yaw_data
#         self.pure_pursuit.getYawStatus(yaw_data)

#     def lane_center_callback(self, point):
#         self.lane_center_points.append((point.x, point.y))
#         if len(self.lane_center_points) > self.max_center_points:
#             self.lane_center_points.pop(0)

#     def waypoints_callback(self, msg):
#         self.waypoints = msg.waypoints

#     def track_callback(self, msg):
#         self.last_track_msg_time = rospy.Time.now().to_sec()
#         self.obstacles = msg.cones
#         current_obstacle_count = len(self.obstacles)
#         # U-turn trigger
#         if (self.u_turn_start_waypoint <= self.current_waypoint <= self.u_turn_end_waypoint and
#             not self.u_turn_completed and not self.u_turn_executed):
#             if current_obstacle_count >= self.min_obstacles_for_uturn:
#                 if any(ob.y > 0 for ob in self.obstacles):
#                     self.u_turn_executed = True
#                     self.start_u_turn()
#                     rospy.loginfo(f"양수 y좌표 장애물 포함 총 {current_obstacle_count}개 감지: 유턴 시작")

#         if current_obstacle_count == 0:
#             if self.obstacle_mode and self.obstacle_cleared:
#                 self.obstacle_cleared = True
#                 self.obstacle_cleared_time = rospy.Time.now().to_sec()
#                 rospy.loginfo("장애물이 사라짐: 차선 모드로 전환 대기 시작")
#         else:
#             self.obstacle_cleared = False
#             if not self.obstacle_mode and self.lane_mode:
#                 self.switch_to_obstacle_mode()
#                 self.dynamic_obstacle_detected = True
#                 self.stop_time = rospy.Time.now().to_sec()
#                 rospy.loginfo("장애물 감지: 긴급 정지")
#             elif self.gps_mode:
#                 rospy.loginfo("GPS 모드에서 장애물 감지: GPS 모드 유지")

#     def calculate_steering_angle(self):
#         if not self.lane_center_points:
#             return 0
#         latest_point = self.lane_center_points[-1]
#         image_center = 320
#         error = latest_point[0] - image_center
#         rospy.loginfo("차선 에러: %.2f", error)
#         steering_angle = (error / image_center) * 22
#         return max(min(steering_angle, 550), -550)

#     def switch_to_obstacle_mode(self):
#         self.obstacle_mode = True
#         self.lane_mode = False
#         self.gps_mode = False
#         self.current_mode = "Obstacle"
#         self.obstacle_avoid_start_time = rospy.Time.now().to_sec()
#         rospy.loginfo("장애물 회피 모드로 전환")

#     def switch_to_lane_mode(self):
#         self.obstacle_mode = False
#         self.lane_mode = True
#         self.gps_mode = False
#         self.current_mode = "Lane"
#         self.dynamic_obstacle_detected = False
#         self.waypoints = []
#         self.fixed_velocity_mode = True
#         self.fixed_velocity_start_time = rospy.Time.now().to_sec()
#         rospy.loginfo("Lane-based Control 모드로 전환")

#     def switch_to_gps_mode(self):
#         self.obstacle_mode = False
#         self.lane_mode = False
#         self.gps_mode = True
#         self.current_mode = "GPS"
#         rospy.loginfo("GPS-based Control 모드로 전환")

#     def start_u_turn(self):
#         self.gps_mode = False
#         self.u_turn_mode = True
#         self.u_turn_start_time = rospy.Time.now().to_sec()
#         rospy.loginfo("유턴 모드 시작")

#     def check_u_turn_completion(self):
#         if self.u_turn_mode:
#             current_time = rospy.Time.now().to_sec()
#             if current_time - self.u_turn_start_time >= self.u_turn_duration:
#                 self.u_turn_completed = True
#                 self.switch_to_gps_mode()
#                 self.current_mode = "GPS"
#                 rospy.loginfo("유턴 완료: GPS 모드로 전환")

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

#     # def run(self):
#     #     rate = rospy.Rate(20)
#     #     sec = 0
#     #     while not rospy.is_shutdown():
#     #         current_time = rospy.Time.now().to_sec()
#     #                 # ✅ 정지선 인식 시 정지 (가장 먼저 넣어야 함)
#     #         if self.stop_line_detected:
#     #             rospy.sleep(2.3)
#     #             rospy.loginfo("정지선 감지: 5초간 정지 시작")
#     #             start_time = rospy.Time.now().to_sec()
#     #             while rospy.Time.now().to_sec() - start_time < 5.0:
#     #                 self.ctrl_msg.accel = 0
#     #                 self.ctrl_msg.steering = 0
#     #                 self.ctrl_msg.brake = 80#100a
#     #                 self.ctrl_msg.seq = self.pose_msg.header.seq
#     #                 self.ctrl_pub.publish(self.ctrl_msg)
#     #                 rospy.sleep(5)  # 20Hz로 brake 전송

#     #             rospy.loginfo("정지 후 출발: 5초간 전진")
#     #             start_time = rospy.Time.now().to_sec()
#     #             while rospy.Time.now().to_sec() - start_time < 5.0:
#     #                 self.ctrl_msg.accel = 80
#     #                 self.ctrl_msg.brake = 0
#     #                 self.ctrl_msg.steering = 0
#     #                 self.ctrl_msg.seq = self.pose_msg.header.seq
#     #                 self.ctrl_pub.publish(self.ctrl_msg)
#     #                 rospy.sleep(0.05)  # 20Hz로 accel 전송
#     def v2x_callback(self, msg):
#         for intersection in msg.interchanges:
#             for state in intersection.states:
#                 if state.signalGroup == self.target_signal_group:
#                     if len(state.state_time_speed) == 0:
#                         rospy.logwarn("❗ SPaT 정보 없음")
#                         return
#                     signal = state.state_time_speed[0].event_state
#                     self.current_signal_state = signal
#                     rospy.loginfo(f"[V2X] signalGroup: {state.signalGroup}, event_state: {signal}")
#                     return
    

#     def run(self):
        
#         rate = rospy.Rate(20)
        
#         while not rospy.is_shutdown():
#             current_time = rospy.Time.now().to_sec()
#             recent_msg = (current_time - self.last_stop_line_msg_time) < 1.0
#             if self.stop_line_detected and recent_msg:
#                 rospy.loginfo("📍 정지선 인식 - C-ITS 신호 따라 행동")
                
#                 if self.current_signal_state == "stop-And-Remain":
#                     self.ctrl_msg.velocity = 0
#                     self.ctrl_msg.accel = 0
#                     self.ctrl_msg.brake = 80
#                     self.ctrl_msg.steering = 0

#                 elif self.current_signal_state == "permissive-clearance":
#                     self.ctrl_msg.velocity = 20
#                     self.ctrl_msg.accel = 10
#                     self.ctrl_msg.brake = 20
#                     self.ctrl_msg.steering = 0

#                 elif self.current_signal_state == "protected-Movement-Allowed":
#                     self.ctrl_msg.velocity = 50
#                     self.ctrl_msg.accel = 30
#                     self.ctrl_msg.brake = 0
#                     self.ctrl_msg.steering = 0

#                 else:
#                     self.ctrl_msg.velocity = 0
#                     self.ctrl_msg.accel = 0
#                     self.ctrl_msg.brake = 80
#                     self.ctrl_msg.steering = 0
#             else:
#                 # 정지선이 감지되지 않은 경우: 주행은 유지하되 V2X 상태 무시하고 기본 주행
#                 self.ctrl_msg.velocity = 50
#                 self.ctrl_msg.accel = 30
#                 self.ctrl_msg.brake = 0
#                 self.ctrl_msg.steering = 0
#                 # 2. run() 디버깅 로그
#                 rospy.loginfo(f"[디버깅] stop_line_detected: {self.stop_line_detected}, last_msg_time: {self.last_stop_line_msg_time}")

#                 rospy.loginfo("🟢 정지선 미인식 - 전진 유지")

#             self.ctrl_pub.publish(self.ctrl_msg)
#             rate.sleep()                
#             # while rospy.Time.now().to_sec() - start_time < 3.0:
#             #          # 1. Local Path 설정
#             #         local_path, _ = findLocalPath(self.global_path, self.pose_msg, self.current_waypoint)
#             #         self.pure_pursuit.getPath(local_path)
    
#             #          # 2. 조향각 계산
#             #         try:
#             #             steer_val, _, _ = self.pure_pursuit.steering_angle(self.current_waypoint)
#             #         except ZeroDivisionError:
#             #             rospy.logwarn("우회전 중 ZeroDivisionError: 조향 0으로 설정")
#             #             steer_val = 0.0
    
#             #         # 3. 메시지 전송
#             #         self.ctrl_msg.accel = 100
#             #         self.ctrl_msg.brake = 0
#             #         self.ctrl_msg.steering = steer_val
#             #         self.ctrl_msg.seq = self.pose_msg.header.seq
#             #         self.ctrl_pub.publish(self.ctrl_msg)
#             #         rospy.sleep(0.05)


#                 # rospy.loginfo("우회전 후 직진: 3초")
#                 # start_time = rospy.Time.now().to_sec()
#                 # while rospy.Time.now().to_sec() - start_time < 3.0:
#                 #     self.ctrl_msg.accel = 100
#                 #     self.ctrl_msg.brake = 0
#                 #     self.ctrl_msg.steering = 0
#                 #     self.ctrl_msg.seq = self.pose_msg.header.seq
#                 #     self.ctrl_pub.publish(self.ctrl_msg)
#                 #     rospy.sleep(0.05)

#             self.stop_line_detected = False
#             continue

                           

#             self.update_current_waypoint()
#             self.check_u_turn_completion()

#             if self.last_track_msg_time and (current_time - self.last_track_msg_time) > self.track_timeout:
#                 if self.obstacle_mode:
#                     rospy.loginfo("장애물이 감지되지 않음: 차선 모드로 전환")
#                     self.switch_to_lane_mode()
#                     self.fixed_velocity_mode = True
#                     self.fixed_velocity_start_time = rospy.Time.now().to_sec()

#             if self.lane_start_waypoint <= self.current_waypoint < self.lane_end_waypoint:
#                 if not self.lane_mode and not self.obstacle_mode and not self.u_turn_mode:
#                     self.switch_to_lane_mode()
#             else:
#                 if not self.gps_mode and not self.u_turn_mode:
#                     self.switch_to_gps_mode()

#             if self.gps_mode:
#                 local_path, self.current_waypoint = findLocalPath(self.global_path, self.pose_msg, self.current_waypoint)
#                 self.pure_pursuit.getPath(local_path)
#                 # ZeroDivisionError 방지
#                 try:
#                     steer_val, look_steering_point, self.target_angle_index = self.pure_pursuit.steering_angle(self.current_waypoint)
#                 except ZeroDivisionError:
#                     rospy.logwarn("ZeroDivisionError 발생 - 조향각을 0으로 설정")
#                     steer_val = 0.0
#                     look_steering_point = None
#                     self.target_angle_index = self.current_waypoint
#                 self.ctrl_msg.steering = steer_val
#                 target_velocity = self.global_path.poses[self.current_waypoint].pose.position.z
#                 self.ctrl_msg.seq = self.pose_msg.header.seq

#             elif self.u_turn_mode:
#                 current_u_turn_time = current_time - self.u_turn_start_time
#                 if current_u_turn_time < self.u_turn_first_phase_duration:
#                     self.ctrl_msg.steering = self.u_turn_steering_first
#                 else:
#                     self.ctrl_msg.steering = self.u_turn_steering_second
#                 target_velocity = self.u_turn_velocity
#                 control_input = self.pid.pid(target_velocity, self.curvel_msg.velocity)
#                 if 0 < control_input <= 200:
#                     self.ctrl_msg.accel = control_input
#                     self.ctrl_msg.brake = 0
#                 elif control_input > 200:
#                     self.ctrl_msg.accel = target_velocity
#                     self.ctrl_msg.brake = 0
#                 else:
#                     self.ctrl_msg.accel = 0
#                     self.ctrl_msg.brake = min(-control_input, 200)
#                 if current_time - self.u_turn_start_time >= self.u_turn_duration:
#                     self.u_turn_mode = False
#                     self.u_turn_completed = True
#                     self.switch_to_gps_mode()
#                     rospy.loginfo("유턴 완료: GPS 모드로 전환")

#             elif self.lane_mode or self.obstacle_mode:
#                 if self.dynamic_obstacle_detected:
#                     self.ctrl_msg.accel = 0
#                     self.ctrl_msg.brake = 200
#                     self.ctrl_msg.steering = 0
#                     target_velocity = 0
#                     if current_time - self.stop_time >= self.brake_duration:
#                         self.dynamic_obstacle_detected = False
#                         self.switch_to_obstacle_mode()
#                         rospy.loginfo("5초 경과: 장애물 회피 모드 시작")
#                 elif self.obstacle_mode:
#                     if self.obstacle_cleared and current_time - self.obstacle_cleared_time >= self.obstacle_clear_delay:
#                         self.switch_to_lane_mode()
#                         rospy.loginfo("장애물이 사라진 후 2초 경과: Lane 모드로 전환")
#                         self.fixed_velocity_mode = True
#                         self.fixed_velocity_start_time = rospy.Time.now().to_sec()
#                     if self.fixed_velocity_mode:
#                         if current_time - self.fixed_velocity_start_time <= 1.0:
#                             self.ctrl_msg.steering = 11
#                             self.ctrl_msg.accel = 20
#                             self.ctrl_msg.brake = 0
#                             target_velocity = 20
#                         else:
#                             self.fixed_velocity_mode = False
#                             rospy.loginfo("고정 속도 및 조향 유지 완료: 정상 주행 재개")
#                     elif self.waypoints:
#                         target_waypoint = self.waypoints[0]
#                         try:
#                             steer_val, _ = self.pure_pursuit_nogps.steering_angle(target_waypoint)
#                         except ZeroDivisionError:
#                             rospy.logwarn("ZeroDivisionError 발생 - 장애물모드 조향각을 0으로 설정")
#                             steer_val = 0.0
#                         self.ctrl_msg.steering = steer_val
#                         rospy.loginfo("장애물 회피 중 - Steering angle: %.2f", self.ctrl_msg.steering)
#                     target_velocity = self.target_velocity_ob
#                 else:
#                     if self.emergency_brake and current_time - self.emergency_brake_start_time < 1.0:
#                         self.ctrl_msg.accel = 0
#                         self.ctrl_msg.brake = 200
#                         target_velocity = 0
#                     else:
#                         if self.emergency_brake:
#                             self.emergency_brake = False
#                             rospy.loginfo("급제동 종료: 정상 주행 재개")
#                         self.ctrl_msg.steering = self.calculate_steering_angle()
#                         target_velocity = self.target_velocity_lane

#             if not self.dynamic_obstacle_detected and not self.emergency_brake and not self.u_turn_mode:
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

#             self.ctrl_pub.publish(self.ctrl_msg)

#             if self.u_turn_mode:
#                 elapsed_time = current_time - self.u_turn_start_time
#                 rospy.loginfo("유턴 진행 중: %.2f초 경과", elapsed_time)

#             current_mode = self.current_mode
#             if self.u_turn_mode:
#                 current_mode = "U-Turn"
#             rospy.loginfo("Current Mode: %s, Waypoint: %d, Steering: %.2f, Velocity: %.2f, Target Velocity: %.2f",
#                           current_mode, self.current_waypoint, self.ctrl_msg.steering, self.curvel_msg.velocity, target_velocity)

#             rate.sleep()

# if __name__ == '__main__':
#     try:
#         planner = erp_planner()
#         planner.run()
#     except rospy.ROSInterruptException:
#         pass

# # !/usr/bin/env python3
# # -*- coding: utf-8 -*-

# # import rospy
# # import math
# # from nav_msgs.msg import Path, Odometry
# # from std_msgs.msg import Float64, Int16, Float32MultiArray, Bool
# # from control_msgs.msg import Velocity, Gear
# # from geometry_msgs.msg import Point
# # from morai_msgs.msg import CtrlCmd, LocalControl
# # from ublox_msgs.msg import NavPVT
# # from lib.utils_main import pathReader, findLocalPath, purePursuit, pidController, purePursuit_nogps
# # from vehicle_msgs.msg import Track, WaypointsArray, Waypoint
# # from v2x_msgs.msg import Spat_new

# # class erp_planner():
# #     def __init__(self):
# #         rospy.init_node('ERP42_planner')

# #         self.stop_line_detected = False
# #         self.last_stop_line_msg_time = 0
# #         self.current_signal_state = None
# #         self.target_signal_group = 4

# #         self.ctrl_msg = CtrlCmd(); self.ctrl_msg.longlCmdType = 2
# #         self.pose_msg = Odometry()
# #         self.curvel_msg = Velocity()
# #         self.yaw_msg = NavPVT()

# #         self.path_reader = pathReader('erp42_control_ob')
# #         self.path_name = 'left_traffic'
# #         self.global_path = self.path_reader.read_txt(self.path_name + ".txt")
# #         self.current_waypoint = 0

# #         self.pure_pursuit = purePursuit()
# #         self.pid = pidController()

# #         self.ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)

# #         rospy.Subscriber("/odom/filtered", Odometry, self.pose_callback)
# #         rospy.Subscriber("/ERP42_velocity", Velocity, self.velocity_callback)
# #         rospy.Subscriber("/ublox/navpvt", NavPVT, self.yaw_callback)
# #         rospy.Subscriber("/stop_line", Bool, self.stop_line_callback)
# #         rospy.Subscriber("/v2x_data", Spat_new, self.v2x_callback)

# #         rospy.loginfo("✅ ERP42 통합 제어 노드 시작")

# #     def stop_line_callback(self, msg):
# #         self.stop_line_detected = msg.data
# #         self.last_stop_line_msg_time = rospy.Time.now().to_sec()

# #     def pose_callback(self, data):
# #         self.pose_msg = data
# #         self.pure_pursuit.getPoseStatus(data)

# #     def velocity_callback(self, data):
# #         self.curvel_msg = data
# #         self.pure_pursuit.getVelStatus(data)

# #     def yaw_callback(self, data):
# #         self.yaw_msg = data
# #         self.pure_pursuit.getYawStatus(data)

# #     def v2x_callback(self, msg):
# #         for intersection in msg.interchanges:
# #             for state in intersection.states:
# #                 if state.signalGroup == self.target_signal_group and len(state.state_time_speed) > 0:
# #                     self.current_signal_state = state.state_time_speed[0].event_state
# #                     rospy.loginfo(f"[V2X] signalGroup: {state.signalGroup}, event_state: {self.current_signal_state}")
# #                     return

# #     def update_current_waypoint(self):
# #         min_dist = float('inf')
# #         closest_wp = self.current_waypoint
# #         for i, wp in enumerate(self.global_path.poses):
# #             dx = self.pose_msg.pose.pose.position.x - wp.pose.position.x
# #             dy = self.pose_msg.pose.pose.position.y - wp.pose.position.y
# #             dist = math.sqrt(dx**2 + dy**2)
# #             if dist < min_dist:
# #                 min_dist = dist
# #                 closest_wp = i
# #         self.current_waypoint = closest_wp

# #     def run(self):
# #         rate = rospy.Rate(20)
# #         stop_line_hold_time = 2.0

# #         while not rospy.is_shutdown():
# #             current_time = rospy.Time.now().to_sec()
# #             recent_msg = (current_time - self.last_stop_line_msg_time) < stop_line_hold_time

# #             if self.stop_line_active and self.stop_line_detected and recent_msg:
# #                 rospy.loginfo("📍 정지선 인식 - C-ITS 신호 따라 행동")
# #                 if self.current_signal_state == "stop-And-Remain":
# #                     self.ctrl_msg.velocity, self.ctrl_msg.accel, self.ctrl_msg.brake = 0, 0, 80
# #                 elif self.current_signal_state == "permissive-clearance":
# #                     self.ctrl_msg.velocity, self.ctrl_msg.accel, self.ctrl_msg.brake = 20, 10, 20
# #                 elif self.current_signal_state == "protected-Movement-Allowed":
# #                     self.ctrl_msg.velocity, self.ctrl_msg.accel, self.ctrl_msg.brake = 50, 30, 0
# #                 else:
# #                     self.ctrl_msg.velocity, self.ctrl_msg.accel, self.ctrl_msg.brake = 0, 0, 80
# #                 self.ctrl_msg.steering = 0
# #             else:
# #                 rospy.loginfo(f"[디버깅] stop_line_detected: {self.stop_line_detected}, last_msg_time: {self.last_stop_line_msg_time}")
# #                 rospy.loginfo("🟢 정지선 미인식 - 기본 주행")

# #                 self.update_current_waypoint()
# #                 local_path, self.current_waypoint = findLocalPath(self.global_path, self.pose_msg, self.current_waypoint)
# #                 self.pure_pursuit.getPath(local_path)
# #                 try:
# #                     steer_val, _, _ = self.pure_pursuit.steering_angle(self.current_waypoint)
# #                 except ZeroDivisionError:
# #                     steer_val = 0.0
# #                 self.ctrl_msg.steering = steer_val

# #                 target_velocity = self.global_path.poses[self.current_waypoint].pose.position.z
# #                 control_input = self.pid.pid(target_velocity, self.curvel_msg.velocity)
# #                 if 0 < control_input <= 200:
# #                     self.ctrl_msg.accel, self.ctrl_msg.brake = control_input, 0
# #                 elif control_input > 200:
# #                     self.ctrl_msg.accel, self.ctrl_msg.brake = target_velocity, 0
# #                 elif control_input < -200:
# #                     self.ctrl_msg.accel, self.ctrl_msg.brake = 0, 200
# #                 else:
# #                     self.ctrl_msg.accel, self.ctrl_msg.brake = 0, -control_inputself.stop_line_active = True

# #                 self.ctrl_msg.velocity = target_velocity

# #             if not recent_msg:
# #                 self.stop_line_detected = False

# #             self.ctrl_msg.seq = self.pose_msg.header.seq
# #             self.ctrl_pub.publish(self.ctrl_msg)
# #             rate.sleep()

# # if __name__ == '__main__':
# #     try:
# #         planner = erp_planner()
# #         planner.run()
# #     except rospy.ROSInterruptException:
# #         pass