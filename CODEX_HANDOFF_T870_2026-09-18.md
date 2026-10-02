# T870 Codex Handoff

작성일: 2026-09-18 (Asia/Seoul)
작업공간: `/home/sxx/Desktop/colcon_ws./colcon_ws/colcon_ws`

## 다음 Codex에게

사용자는 T870 차량의 GPS path tracking, T자 주차, 평행주차를 실차에서 시험 중이다.
사용자가 "구동코드"라고 하면 설명보다 바로 실행 가능한 정확한 명령을 먼저 제시한다.
경로/포트/로그를 추측하지 말고 이 파일과 최신 로그를 먼저 읽는다.

## 핵심 파일

- T자 메모: `t870_parking_notes.txt`
- 진행 메모: `t870_progress_2026-09-18.txt`
- T자 경로: `gps_recordings/t_p(aa).csv`, `t_p(ba).csv`
- `t_p(ba).csv`는 2026-09-19 남아 있던 `t_p(ba).txt`의 UTM 좌표
  72점으로 복구했고 follower 로드 검증을 통과했다.
- 평행주차 경로: `gps_recordings/p_a.csv`, `p_b.csv`
- 평행주차 최종 메모: `t870_parallel_parking_notes.txt`
- GPS follower source: `src/t870_control/t870_control/gps_path_follower_node.py`
- GPS follower installed copy: `install_t870/t870_control/lib/python3.12/site-packages/t870_control/gps_path_follower_node.py`
- 주행 launch source: `src/t870_control/launch/t870_lane_autonomy.launch.py`
- 주행 launch installed copy: `install_t870/t870_control/share/t870_control/launch/t870_lane_autonomy.launch.py`

## 장치 포트

- Pixhawk: `/dev/serial/by-id/usb-Holybro_Pixhawk6C_45002F001451333531373234-if00`
- Arduino UNO: `/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_34331323136351211280-if00`
- u-blox/SMC-2000 GPS: `/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00`
- LiDAR: `/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_dc29dffaaa31f111b5fb935f30d20014-if00-port0`

## MAVROS

`install_t870`를 MAVROS 터미널에서 source 하면 symbol lookup 오류가 날 수 있다.
MAVROS는 시스템 설치본만 사용하고, MAVROS 터미널에서는 `source install_t870/setup.bash`를 하지 않는다.

```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=/tmp/ros_mavros_logs
/opt/ros/jazzy/lib/mavros/mavros_node --ros-args \
  -r __ns:=/mavros \
  --params-file /opt/ros/jazzy/share/mavros/launch/apm_pluginlists.yaml \
  --params-file /opt/ros/jazzy/share/mavros/launch/apm_config.yaml \
  -p fcu_url:=/dev/serial/by-id/usb-Holybro_Pixhawk6C_45002F001451333531373234-if00:921600 \
  -p system_id:=255 \
  -p component_id:=191 \
  -p target_system_id:=1 \
  -p target_component_id:=1
```

## 현재 T자 주차 기준

실차에서 A/B 양쪽 감지와 진입 성공을 확인한 최종 기준:

- WP13에서 감지: B가 막힌 것으로 보고 `t_p(aa).csv` 선택
- WP15에서 감지: A가 막힌 것으로 보고 `t_p(ba).csv` 선택
- WP20에서 최종 결정
- LiDAR 각도: 80~100 deg
- 거리: 0.05~4.0 m
- 최소 점 수: 8
- stage2 판정: 비활성화
- 주차 launch: `lidar_safety_enabled=false`, `enable_mission_stack=false`, `enable_lane_preview=false`

T자 실행 예:

```bash
cd /home/sxx/Desktop/colcon_ws./colcon_ws/colcon_ws
source /opt/ros/jazzy/setup.bash
source install_t870/setup.bash
ros2 launch t870_control t870_parking.launch.py \
  path:=/home/sxx/Desktop/colcon_ws./colcon_ws/colcon_ws/gps_recordings/t_p\(aa\).csv \
  gps_speed_mps:=0.56 \
  2>&1 | tee /tmp/t870_run.log
```

## 현재 평행주차 상태

- `p_a.csv`: 새 경로 기록 완료. 먼저 단독 추종 시험.
- `p_b.csv`: 새 경로 기록 완료. 이후 LiDAR 미감지 분기로 사용.
- p_b를 2026-09-19에 다시 기록했다. 32점(WP0~31), target_speed 전 구간
  `0.560 m/s`, cusp는 WP12/WP19이다. WP12 E-stop 후 WP13~19를 후진하고
  WP19 E-stop/정지 후 WP20부터 전진한다.
- p_b 첫 시험에서 WP9의 전진 LAD가 cusp WP11을 넘어 WP14를 겨냥하여
  정지 전에 오른쪽 최대 조향이 발생했다. `active_run`이 아직 없는 초기
  전진 구간도 `direction_run()`으로 목표점을 WP11에 제한하도록 follower를
  수정했고, `install_t870` 재빌드 및 단위 테스트 16개 통과했다.
- 다음 시험에서 `gps_cusp_brake_distance_m=0.0` 때문에 WP11 최근접점에
  고정된 채 계속 전진했다. p_b는 WP11/WP16 반경 0.35 m에서 전환 제동을
  시작하도록 `gps_cusp_brake_distance_m:=0.35`를 사용한다. 전환 첫 틱부터
  조향을 0으로 강제하도록 follower도 수정했다.
- 0.35 m 설정이 노드에 실제 적용됐는데도 WP11에서 want_reverse=0이던
  로그가 다시 확인됐다. 양수 cusp brake 설정에서는 nearest가 cusp 자체에
  도달하면 거리 판정과 별개로 즉시 방향 전환하도록 보강했다.
- 사용자 요구에 따라 모든 전/후진 방향 전환은 전용 하드웨어 E-stop을
  먼저 건다. follower가 `/t870/shift_estop=True`를 발행하면 Arduino bridge가
  펌웨어에 `E`를 보내 PWM을 즉시 차단한다. 최소 0.3초가 지나고 엔코더
  절대속도가 0.15 m/s 이하일 때만 새 방향을 확정하고 다음 틱부터 출발한다.
  WP16 후진 종료의 PARK_BRAKE/PARKED 동안에도 shift E-stop을 유지한다.
  전체 테스트 30개 통과 및 install_t870 재빌드 완료.
- p_b는 자동 cusp에만 의존하지 않고 `gps_reverse_start_waypoint:=12`로
  WP12 후진 전환을 한 번만 명시적으로 강제한다. 종료는
  `gps_reverse_stop_waypoint:=19`이다.
- 첫 명시 전환 시험에서 REVERSE 완료 0.05초 뒤 WP11의 전진 cusp flag를
  다시 읽어 FORWARD E-stop이 걸린 버그를 확인했다. 시작된 active reverse
  run을 WP16 종료 처리 전까지 래치하도록 수정했다.
- p_a의 target_speed 열은 전 구간 `0.560 m/s`로 수정됨.
- p_a의 큰 방향 전환 때문에 follower가 WP19~23을 자동 후진 구간으로 판정한다.
- p_a 단독 시험에서 자동 selector는 사용하지 않는다.
- p_a/p_b LiDAR 자동 선택은 2026-09-19 실차 성공했다.
- 2026-09-19 자동 선택 launch를 최신화했다. WP4에서 0~40 deg,
  0.05~5.5 m 창을 관찰하고 WP5 진입 시 확정한다. 감지되면 p_a,
  미감지면 p_b이다. 파일명은 p_a.csv/p_b.csv로 수정했고, 경로별 cusp가
  다르므로 명시 reverse start/stop은 -1로 두어 자동 판정한다. shift E-stop
  0.3초, cusp brake 0.35 m, 최대 조향 0.20 rad를 사용한다.

최근 확인된 문제와 현재 설정:

- 후진 조향은 `gps_invert_reverse_steering:=true`가 차량 기준으로 맞다.
- 정지 후 조향은 0으로 강제된다.
- 후진 lookahead는 최신 시험값 `1.0 m`를 사용한다.
- WP21 진입 오른쪽 꺾임을 줄이기 위해 최대 조향을 `0.20 rad`로 낮췄다.
- 최신 `/tmp/p_a_2kph_tracking.log`에서는 WP23 후진 종료 후 WP24로 재개하자마자
  WP23으로 되돌아가 REVERSE/FORWARD 전환을 반복했다. 원인은 PARKED 후 전진
  재개 시 `shift_hold_index`가 이전 WP23으로 남아 있던 것이다.
- source와 installed copy 모두 `self.shift_hold_index = resume`으로 수정했고
  `python3 -m py_compile` 통과했다.
- p_a 2 kph 실행 명령:

```bash
cd /home/sxx/Desktop/colcon_ws./colcon_ws/colcon_ws
source /opt/ros/jazzy/setup.bash
source install_t870/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONUNBUFFERED=1
export ROS_LOG_DIR=/tmp/ros_t870_logs

ros2 launch t870_control t870_lane_autonomy.launch.py \
  lidar_serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_dc29dffaaa31f111b5fb935f30d20014-if00-port0 \
  arduino_serial_port:=/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_34331323136351211280-if00 \
  record_gps_path:=false \
  enable_localization:=true \
  enable_gps_follower:=true \
  gps_follow_path_file:=/home/sxx/Desktop/colcon_ws./colcon_ws/colcon_ws/gps_recordings/p_a.csv \
  gps_follow_speed_mps:=0.56 \
  gps_allow_reverse:=true \
  gps_reverse_lookahead_m:=1.0 \
  gps_invert_reverse_steering:=true \
  gps_reverse_start_waypoint:=11 \
  gps_reverse_overshoot_m:=-0.35 \
  gps_reverse_cusp_deg:=45.0 \
  gps_cusp_brake_distance_m:=0.0 \
  gps_shift_settle_sec:=0.3 \
  gps_waypoint_hold_sec:=1.0 \
  gps_nearest_forward_points:=5 \
  gps_nearest_backward_points:=2 \
  gps_max_waypoint_advance:=2 \
  gps_lad_2kmh_m:=2.0 \
  gps_lad_4kmh_m:=2.0 \
  gps_lad_6kmh_m:=2.5 \
  gps_startup_straight_sec:=2.0 \
  gps_follow_max_steer_rad:=0.20 \
  lidar_safety_enabled:=false \
  enable_mission_stack:=false \
  enable_lane_preview:=false \
  require_operator_heartbeat:=false \
  2>&1 | tee /tmp/p_a_2kph_tracking.log
```

웹 제어 화면에서 `GPS PATH`를 선택한다. 최신 tracking CSV는 `gps_recordings/gps_tracking_*.csv`에 생성된다.

## 최근 로그에서 확인한 사실

로그 `/tmp/p_a_tracking.log`, tracking CSV `gps_recordings/gps_tracking_20260918_223633.csv` 기준:

- follower가 `Cusps over 45 deg at waypoints 18, 23; REVERSE segments 19~23`라고 판정했다.
- WP19에서 정지 후 REVERSE 전환은 정상 실행됐다.
- 이전 명령에서 `gps_invert_reverse_steering:=false`를 사용해 후진 조향이 반대였고, 이후 `true`로 수정했다.
- 이전 p_a CSV는 target_speed가 0.300 m/s라 실제 명령 속도가 느렸고, 현재 p_a는 0.560으로 수정했다.
- 확인할 다음 로그 항목: WP21 진입 시 `raw_steering_rad`, `filtered_steering_rad`, `command_speed_mps`, `state=REVERSE`.

## 다시 시작할 때의 순서

1. 이 파일과 `t870_parking_notes.txt`를 읽는다.
2. MAVROS를 먼저 별도 터미널에서 실행한다.
3. T자 시험이면 T자 launch, 평행 시험이면 위 p_a launch를 실행한다.
4. 실차 로그가 생기면 최신 `gps_tracking_*.csv`와 `/tmp/p_a_2kph_tracking.log`를 분석한다.
   특히 WP23 후 `Current waypoint: 24` 다음에 WP23으로 되돌아가지 않는지 확인한다.
5. p_a가 안정화된 뒤 `t870_parallel_parking.launch.py`의 LiDAR selector로 p_a/p_b 분기를 시험한다.

## 주의

- 사용자가 터미널과 Codex를 꺼도 이 파일, 두 메모, GPS CSV는 작업공간에 남는다.
- 새 Codex 계정은 이전 대화 자체를 자동 복구하지 않을 수 있다. 새 대화 첫 메시지에 이 파일 경로를 알려주거나 파일 내용을 첨부하면 된다.
- MAVROS/Arduino 포트가 점유되어 있으면 중복 실행하지 않는다.
- 실제 주행 전에는 차량 주변을 비우고 STOP 상태에서 연결/조향 방향을 확인한다.
