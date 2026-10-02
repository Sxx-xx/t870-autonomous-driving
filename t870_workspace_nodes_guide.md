# Henes Broon T870 ROS 2 워크스페이스 노드 가이드 & 실행 명령어 매뉴얼

본 문서는 `C:\colcon_ws\colcon_ws` 워크스페이스 내 2D LiDAR + IMU 센서 시스템 및 긴급정지 로직이 적용된 주요 패키지 및 노드들의 기능 설명과 ROS 2 실행 명령어 모음입니다.

---

## 1. t870_control 패키지 (차량 제어, 2D 라이다 긴급정지 & 플래너)

| 노드 이름 (Node Name) | 엔트리포인트 (Script) | 노드 기능 설명 | ROS 2 실행 명령어 |
| :--- | :--- | :--- | :--- |
| `roboclaw_steering_node` | `roboclaw_steering_node.py` | **RoboClaw Solo 30A & Pixhawk 하드웨어 드라이버 + 2D E-Stop**<br>- `/cmd_vel` (`geometry_msgs/msg/Twist`) 구독<br>- `angular.z` ➡️ RoboClaw 포텐셔미터 위치 제어 (Soft Limit 적용)<br>- `linear.x` ➡️ Pixhawk RC Overrride (`/mavros/rc/override`) 스로틀 제어<br>- **2D LaserScan (`/scan`) 감지**: 전방 30° 내 비상거리(<1.2m) 돌발 어린이 더미 감지 시 E-Stop 즉시 발동 | `ros2 run t870_control roboclaw_steering_node` |
| `t870_planner_main` | `t870_planner_main.py` | **T870 자율주행 메인 경로 계획 노드**<br>- Pure Pursuit 및 PID 제어를 통해 `/cmd_vel` 생성 | `ros2 run t870_control t870_planner_main` |
| `t870_status` | `t870_status.py` | **차량 조향, 속도 및 비상정지 텔레메트리 모니터링 노드**<br>- `/t870/steering_angle`, `/t870/current_speed`, `/t870/emergency_stop` 구독 | `ros2 run t870_control t870_status` |

---

## 2. ma_rrt_path_plan 패키지 (2D LaserScan + IMU RRT 경로 계획)

| 노드 이름 (Node Name) | 엔트리포인트 (Script) | 노드 기능 설명 | ROS 2 실행 명령어 |
| :--- | :--- | :--- | :--- |
| `MaRRTPathPlanNode` | `MaRRTPathPlanNode.py` | **2D 라이다 + IMU 기반 RRT 장애물 회피 경로 생성**<br>- 2D LaserScan (`/scan`) ➡️ Cartesian 2D 장애물 좌표 변환<br>- IMU (`/mavros/imu/data`) ➡️ 쿼터니언 자세 융합을 통한 헤딩(Yaw) 보정<br>- 대형 정적 장애물 (T870 장애물) 우회 2D RRT 트리 및 최적 경로 생성 | `ros2 run ma_rrt_path_plan ma_rrt_path_plan_node` |

---

## 3. t870_track 패키지 (경로 추종 및 장애물 회피)

| 노드 이름 (Node Name) | 엔트리포인트 (Script) | 노드 기능 설명 | ROS 2 실행 명령어 |
| :--- | :--- | :--- | :--- |
| `t870_tracker` | `t870_track.py` | **웨이포인트 추종 노드**<br>- 설정된 경로 웨이포인트를 추종하며 `/cmd_vel` 토픽 발행 | `ros2 run t870_track t870_tracker` |
| `t870_tracker_oa` | `t870_track_oa.py` | **장애물 회피 추종 노드**<br>- 라이다/센서 입력 기반 동적 장애물 회피 경로 추종 | `ros2 run t870_track t870_tracker_oa` |
| `t870_status1` | `t870_status1.py` | **트래킹 전용 상태 모니터링 노드** | `ros2 run t870_track t870_status1` |

---

## 4. 환경 변수 설정 및 실행 순서

### 1) 환경 변수 세팅 (PowerShell 기준)
```powershell
# ROS 2 환경 설치 경로 로드
call C:\opt\ros\humble\x64\local_setup.bat

# 워크스페이스 설치 경로 로드
. .\install\setup.ps1
```

### 2) T870 통합 주행 실행 예시
```powershell
# 터미널 1: RoboClaw & Pixhawk 인터페이스 제어 및 2D E-stop 노드 실행
ros2 run t870_control roboclaw_steering_node

# 터미널 2: 2D LaserScan + IMU 기반 RRT 경로 플래너 실행
ros2 run ma_rrt_path_plan ma_rrt_path_plan_node

# 터미널 3: 메인 주행 플래너 실행
ros2 run t870_control t870_planner_main
```
