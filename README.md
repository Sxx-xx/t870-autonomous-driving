# T870 자율주행 워크스페이스

Henes Broon T870 전동차를 ROS 2 Jazzy로 자율주행시키는 colcon 워크스페이스다.
ERP42/ROS 1 기반 코드를 이식한 뒤, RTK GPS 경로 추종에 LiDAR·카메라 미션
(신호등, 정적/동적 장애물, T주차, 평행주차, 표지 판정)을 얹어 대회용으로 통합했다.

- 환경: Ubuntu 24.04, ROS 2 Jazzy, Python 3.12
- 핵심 패키지: `src/t870_control`
- 실행 진입점: `./run.sh`

## 하드웨어 구성

| 역할 | 장치 |
| :--- | :--- |
| 자세·상태 추정 | Pixhawk 6C (내장 IMU + IST8310), MAVROS로 연결 |
| 위치 | u-blox ZED-F9P RTK 수신기 |
| LiDAR | SLAMTEC RPLIDAR C1 (`/scan`) |
| 카메라 | Logitech C920e(전방) + 보조 카메라 1대 |
| 구동·조향 | Arduino UNO — 엔코더 속도 PI 제어, 포텐셔미터 조향 위치 제어 |
| 수동 개입 | RC 수신기 → 보조 Arduino → I2C 브리지 (AUTO/MANUAL, 단절 시 급정지) |

구동 계통이 Pixhawk 직접 구동에서 Arduino 제어로 바뀐 경위는
`t870_actuator_redesign.txt`에 있다.

## 폴더 구조

```
.
├── run.sh / watch.sh / t870_cleanup.sh   실행, 주행 모니터, 노드 정리
├── src/
│   ├── t870_control/        차량 제어·미션 노드, launch, 설정, 테스트
│   ├── t870_track/          웨이포인트 추종(초기 버전)
│   ├── ma_rrt_path_plan/    LiDAR + IMU 기반 RRT 로컬 경로 계획
│   └── (외부 패키지)        아래 "외부 패키지" 참조
├── gps_recordings/          기록한 GPS 경로, QGIS 프로젝트, 주행 로그
│   └── 대회용/              대회 주행에 쓰는 경로 CSV
├── bev_calib/               카메라 BEV·ROI 캘리브레이션, 상태 대시보드
├── traffic_calib/           신호등 검출 진단
├── sketch_aug/              주 Arduino 펌웨어 (구동 + 조향)
├── pwm_receiver_bridge/     RC 수신기 → I2C 브리지 펌웨어
├── ble_remote_bridge/       BLE 리모컨 → I2C 브리지 펌웨어
├── encoder_check/, pixhawk_pwm_check/, pixhawk_steering_dry_run/   배선 진단 스케치
├── pixhawk_backup/          Pixhawk 파라미터 백업
├── path_*.py, gpkg_to_path.py   경로 편집 도구
├── rtk_*.py, ubx_config.py, scan_check.py, arduino_check.py   센서 진단 도구
└── *.txt, *.md              작업 기록과 인수인계 문서
```

## 받기

모델 가중치(`.pth`), 압축 파일, 영상, 사진은 Git LFS로 저장돼 있다.
LFS 없이 클론하면 이 파일들이 포인터로만 받아진다.

```bash
sudo apt install git-lfs
git lfs install
git clone https://github.com/Sxx-xx/new.git
```

## 빌드

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src/t870_control \
  --build-base build_t870 --install-base install_t870
source install_t870/setup.bash
```

`--packages-select`는 이 워크스페이스에서 동작하지 않으므로 `--base-paths`로 범위를 지정한다.
빌드 산출물(`build_*`, `install_*`)은 저장소에 없으므로, 새로 클론했다면 MAVROS,
sllidar_ros2 등 의존 패키지도 `--base-paths`에 함께 넣어 먼저 빌드해야 한다.
conda가 활성화돼 있으면 ROS가 다른 파이썬을 잡으므로 끄고 빌드한다.

## 실행

```bash
./run.sh              # 장치 점검 → MAVROS → 통합 launch → 대시보드
./run.sh --no-mavros  # MAVROS가 이미 떠 있을 때
./run.sh --log        # 대시보드 대신 필터링한 로그 보기
./watch.sh            # 다른 터미널에서 주행 상태 모니터
./t870_cleanup.sh     # 터미널을 그냥 닫아 노드가 남았을 때 정리
```

조종기 AUTO가 주행, MANUAL이 정지다. `Ctrl-C` 한 번이면 전부 종료된다.

`run.sh`, `watch.sh`와 일부 launch 파일에는 워크스페이스 절대경로
(`/home/sxx/Desktop/colcon_ws./colcon_ws/colcon_ws`)와 USB 장치 시리얼이 들어 있다.
다른 PC나 다른 장치에서 쓰려면 이 값들을 먼저 고쳐야 한다.

개별 launch:

| launch | 용도 |
| :--- | :--- |
| `t870_competition.launch.py` | 대회 통합 실행 |
| `t870_lane_autonomy.launch.py` | GPS 경로 추종 + 미션 구간 인자 |
| `t870_gps_record.launch.py` | GPS 경로 기록 |
| `t870_parking.launch.py`, `t870_parallel_parking.launch.py` | 주차 단독 시험 |
| `t870_indoor_lidar.launch.py` | 실내 LiDAR 시험 |
| `t870_localization.launch.py` | EKF / navsat 위치 추정 |

미션 구간 인자와 안전 기본값은 `T870_MISSIONS.md`에 정리돼 있다.

## 대회 미션 흐름

`competition_mission_manager`가 현재 웨이포인트(WP) 번호에 따라 미션을 켜고 끈다.
경로는 `gps_recordings/대회용`의 CSV(UTM 52N, EPSG:32652)를 쓴다.

| 구간 | 동작 |
| :--- | :--- |
| 기본 | `T_Parking1.csv`를 8 km/h로 GPS 추종 |
| 신호등 | 카메라로 적/녹 판정, 적색이면 정지선에서 정지 후 녹색에 재출발 |
| 정적 장애물 | GPS 조향에 LiDAR 회피 보정각을 더해 통과 |
| T주차 | LiDAR로 주차 칸을 판단해 `T_Parking2.csv`로 경로 전환 |
| 동적 장애물 | 정면 ±30°, 3 m 안에 물체가 잡히면 정지 후 복귀 |
| 평행주차 | LiDAR 판단 후 `P_Parking1/2.csv`로 경로 전환 |
| 마지막 표지 | 카메라로 적/녹 좌우 순서를 보고 `OX_right.csv` 분기 결정 |

LiDAR 미션 구간에서는 약 2 km/h로 감속한다. WP 번호, 속도, 판정 기준의 세부 값과
변경 이력은 `T870_COMPETITION_HANDOFF_2026-09-19.txt`에 있다.

## 테스트

```bash
PYTHONPATH=src/t870_control:$PYTHONPATH /usr/bin/python3 -m pytest \
  src/t870_control/test -q
```

## 경로 작업 도구

| 스크립트 | 용도 |
| :--- | :--- |
| `path_to_qgis.py` | 기록한 경로 CSV → QGIS용 GeoPackage |
| `path_qgis_project.py` | GeoPackage → 위성지도 배경이 붙은 QGIS 프로젝트 |
| `gpkg_to_path.py` | QGIS 편집 결과 → 주행용 CSV |
| `path_speed_profile.py` | 곡률에 맞춰 점별 목표속도 재계산 |
| `path_mark_reverse.py` | 경로의 특정 구간을 후진으로 표시 |
| `path_shift.py`, `path_anchor.py` | 경로 전체 평행이동 (수동 / 현재 위치 기준) |
| `path_offset_check.py` | 차를 경로 위에 세우고 수신기 위치 오차 확인 |
| `rtk_monitor.py`, `rtk_walk_check.py` | RTK 끊김 기록 |
| `ubx_config.py` | u-blox 수신기 설정 조회·변경 |
| `scan_check.py` | 판정 지점에서 LiDAR에 보이는 물체의 각도·거리 측정 |
| `arduino_check.py` | Arduino 텔레메트리로 구동이 끊긴 단계 진단 |

각 스크립트 맨 위 주석에 사용법이 있다.

## 문서

| 파일 | 내용 |
| :--- | :--- |
| `T870_COMPETITION_HANDOFF_2026-09-19.txt` | 대회 통합 작업 기록과 인수인계 (가장 최신) |
| `t870_competition_notes.txt` | 대회 현장 메모 |
| `T870_MISSIONS.md` | 미션 노드와 실행 인자 |
| `t870_parking_notes.txt`, `t870_parallel_parking_notes.txt` | 주차 미션 |
| `traffic_light_notes.txt` | 신호등 검출 |
| `t870_actuator_redesign.txt` | 구동·조향 계통 재설계 |
| `t870_migration_summary.txt` | ERP42/ROS 1 → T870/ROS 2 이식 정리 |
| `t870_progress_*.txt` | 날짜별 작업 기록 |
| `t870_workspace_nodes_guide.md` | 초기(Windows/Humble 시절) 노드 가이드. 현재 구성과 다르다 |

## 외부 패키지

`src` 아래의 다음 패키지는 외부 저장소에서 가져온 것이며, 일부는 이 차량에 맞게 수정했다.
각 패키지의 라이선스는 해당 폴더의 LICENSE를 따른다.

| 패키지 | 원본 |
| :--- | :--- |
| `mavros` | https://github.com/mavlink/mavros |
| `sllidar_ros2` | https://github.com/Slamtec/sllidar_ros2 |
| `ublox` | https://github.com/KumarRobotics/ublox |
| `gps_umd` | https://github.com/swri-robotics/gps_umd |
| `rtcm_msgs` | https://github.com/tilk/rtcm_msgs |
| `HesaiLidar_General_ROS` | https://github.com/HesaiTechnology/HesaiLidar_General_ROS |
| `adaptive_clustering` | https://github.com/yzrobot/adaptive_clustering |
| `lanenet-lane-detection-pytorch` | https://github.com/IrohXu/lanenet-lane-detection-pytorch |
| `morai_msgs` | https://github.com/morai-developergroup/morai_msgs |
| `ma_rrt/ma_rrt_path_plan` | https://github.com/ekampourakis/ma_rrt_path_plan |
| `t870_track/common_msgs` | https://github.com/ros/common_msgs |
