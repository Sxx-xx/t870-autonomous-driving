# T870 미션 기능 설정

`mission.txt` 명세를 현재 `t870_control` 구조에 통합했다. 최종 차량 명령은
기존 휴대폰 `AUTO/GPS/STOP/MANUAL` 노드를 반드시 통과하므로 STOP과 MANUAL
제어 및 Arduino 리시버 단절 급정지는 그대로 유지된다.

## 구현 노드

- `mission_zone_manager_node`: `/gps/current_waypoint` → `/t870/active_mission`
- `mission_mux_node`: ESTOP > PARALLEL > TPARK > AVOID > TRAFFIC_LIGHT > LANE_SIGNAL
- `obstacle_avoidance_node`: LiDAR 좌우 회피 및 IMU 기준 방향 복귀
- `emergency_stop_node`: 전방 15도/2 m, 3점 감지, 제거 후 3.5초 정지
- `parallel_park_node`, `t_park_node`: LiDAR 표식 확인 후 저속 시간 시퀀스
- `traffic_light_node`: 타이머 또는 `/vision/traffic_light` 라벨 입력
- `lane_signal_node`: `/vision/lane_signal` 10프레임 확인 후 GPS 경로 교체
- `webcam_pub_node`: OpenCV USB 카메라 → `/camera/image_raw`
- `gps_path_follower`: `/t870/set_gps_path` 경로 핫리로드 추가

## 안전 기본값

- 모든 미션 waypoint 범위는 빈 값이므로 기본 실행에서는 미션이 작동하지 않는다.
- 주차는 `parking_armed:=false`이므로 범위가 지정돼도 정지 출력만 낸다.
- 주차는 LiDAR 데이터가 최신이고 지정 측면에서 표식을 확인해야 시작한다.
- 미션 명령이 0.5초 이상 끊기면 Mux는 속도 0을 출력한다.
- ESTOP 구간은 감시 구간이며 실제 장애물이 감지될 때만 제어권을 가져간다.

## 현장 실행 인자 예시

범위 형식은 `시작:끝`이며 여러 구간은 쉼표로 구분한다. 아래 숫자는 예시일
뿐이므로 실제 기록 경로의 waypoint 번호로 반드시 교체한다.

```bash
ros2 launch t870_control t870_lane_autonomy.launch.py \
  avoid_ranges:=100:130 \
  estop_ranges:=200:230 \
  parallel_ranges:=300:320 \
  tpark_ranges:=400:420 \
  traffic_light_ranges:=500:520 \
  lane_signal_ranges:=600:630 \
  parking_armed:=false \
  lane_signal_path_file:=/절대경로/1_lane_path.csv
```

주차 시퀀스는 바퀴를 띄운 시험과 저속 폐쇄구역 검증을 마친 뒤에만
`parking_armed:=true`로 바꾼다. 비전 모드는 각 노드의
`use_vision_detection` 파라미터와 외부 검출 라벨 토픽을 사용한다.
