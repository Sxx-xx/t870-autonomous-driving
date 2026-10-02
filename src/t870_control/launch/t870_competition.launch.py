"""Combined waypoint-driven T870 competition mission.

MAVROS must be started separately. The initial route is T1.csv.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


WORKSPACE = '/home/sxx/Desktop/colcon_ws./colcon_ws/colcon_ws'
ROUTES = WORKSPACE + '/gps_recordings/대회용'
LIDAR = ('/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_'
         'Controller_dc29dffaaa31f111b5fb935f30d20014-if00-port0')
ARDUINO = ('/dev/serial/by-id/'
           'usb-Arduino__www.arduino.cc__0043_34331323136351211280-if00')


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gps_speed_mps', default_value='2.2222',
            description='Normal GPS cruise speed; 2.2222 m/s = 8 km/h'),
        DeclareLaunchArgument(
            'gps_escape_speed_mps', default_value='2.7777',
            description='WP39 오르막 정차 탈출 속도; 2.7777 m/s = 약 10 km/h. 펌웨어 MAX_SPEED_KMH(10.0) 바로 아래로 둔다'),
        DeclareLaunchArgument(
            'gps_escape_duration_sec', default_value='3.0'),
        DeclareLaunchArgument('reverse_lad', default_value='1.0'),
        Node(
            package='t870_control',
            executable='competition_mission_manager',
            name='competition_mission_manager',
            output='screen',
            parameters=[{
                't_path_1': ROUTES + '/T1.csv',
                't_path_2': ROUTES + '/T2.csv',
                'p_path_1': ROUTES + '/P1.csv',
                'p_path_2': ROUTES + '/P2.csv',
                'ox_path': ROUTES + '/OX_right.csv',
                'hill_stop_wp': 39,
                # WP532 의 무조건 3초 정차는 껐다(-1). 그 항목은 원래
                # 동적장애물 미션을 시간 정차로 흉내 낸 것이었고, 이제
                # emergency_stop_node 가 라이다로 실제 장애물을 보고
                # 판단한다(estop_ranges WP531~588, 정지 3초).
                # 음수는 미션 매니저에서 '그 정차 없음' 으로 처리한다.
                'timed_estop_wp': -1,
                # 신호등 정지선에 잡혀 서면 WP 가 안 늘어 해제 창
                # (traffic_release_window_wp) 은 못 쓴다. GREEN 이 안
                # 잡히면 이 제한시간만이 탈출구다. 실차에서 초록으로
                # 바뀌었는데 못 나온 적이 있으므로 반드시 켜 둔다.
                # 0 이하로 두면 제한시간 없음 = 영영 교착.
                'traffic_max_hold_sec': 25.0,
                # 이미 지나쳐 버린 신호등을 닫는 창. 수동으로 정지선을
                # 넘겼을 때 그 신호등이 이후 전 구간을 잡지 않게 한다.
                'traffic_release_window_wp': 5,
                # hill_stop_wp(WP39) 에만 적용되는 정차 시간이다.
                'estop_duration_sec': 3.0,
                'cruise_speed_mps': 2.2222,
                'lidar_speed_mps': 0.56,
                # 규정 정차 직후 탈출 가속. 오르막에서 순항속도로는 다시
                # 붙기 어렵다. 정차가 풀린 순간부터 이 시간만 허용한다.
                'escape_speed_mps': ParameterValue(
                    LaunchConfiguration('gps_escape_speed_mps'),
                    value_type=float),
                'escape_duration_sec': ParameterValue(
                    LaunchConfiguration('gps_escape_duration_sec'),
                    value_type=float),
            }],
        ),
        # T parking: the successful test window is applied at global WP299.
        # Detection means slot B is blocked, therefore select T route 1/A.
        Node(
            package='t870_control',
            executable='parking_slot_selector',
            name='competition_t_selector',
            output='screen',
            parameters=[{
                # 관측은 WP297 부터 시작하고 확정은 WP299 를 넘어설 때 한다.
                # 시작을 299 로 두면 최근접 색인이 298 -> 300 으로 한 번만
                # 건너뛰어도(gps_max_waypoint_advance=2) 관측이 시작조차
                # 되지 않고 choice 가 발행되지 않는다. 그러면 t_choice 가
                # 기본값 1 에 남아 LiDAR 판단이 조용히 무시된다.
                'observation_start_waypoint': 297,
                'decision_waypoint': 299,
                'decide_after_observation_scans': False,
                # WP297~299 는 감속 램프 구간이라 통과에 약 3.7 초 걸린다.
                # C1 이 10 Hz 이므로 약 37 scan 이 쌓인다. observation_scans
                # 는 decide_after_observation_scans=False 일 때 로그 표시와
                # blocked_votes_required 상한으로만 쓰인다.
                'observation_scans': 30,
                # 1 표는 단발 노이즈 한 번에 확정돼 버린다. 0.5 초 연속
                # 감지를 요구한다. 장애물이 WP299 한 칸에서만 보여도
                # 18 scan 이 나오므로 놓칠 여유는 충분하다.
                'blocked_votes_required': 5,
                'min_points_per_scan': 8,
                'forward_angle_deg': 180.0,
                'zone_b_angle_min_deg': 80.0,
                'zone_b_angle_max_deg': 100.0,
                'zone_b_range_min_m': 0.05,
                'zone_b_range_max_m': 4.0,
                # 검사 창(+80~+100도)은 T1 이 들어갈 칸을 본다.
                # 거기에 뭔가 있으면 1번 칸이 막힌 것이므로 2번으로 간다.
                #   감지   -> choice 2 -> T2 WP299 전환
                #   미감지 -> choice 1 -> T1 유지
                # slot_a_path 는 '감지됐을 때 고르는 경로' 이므로 로그가
                # 실제 동작과 맞도록 T2 를 둔다.
                'choice_when_blocked': 2,
                'slot_a_path': ROUTES + '/T2.csv',
                'slot_b_path': ROUTES + '/T1.csv',
                'publish_path': False,
                'choice_topic': '/competition/t_parking_choice',
                # 선택기가 두 개다. 결정을 잠근 뒤 계속 재발행하므로
                # 결과 토픽도 나눠야 두 값이 안 섞인다.
                'result_topic': '/parking/t_blocked_zone',
            }],
        ),
        # Parallel parking: successful WP4-relative window mapped to the
        # supplied global decision WP638.
        Node(
            package='t870_control',
            executable='parking_slot_selector',
            name='competition_parallel_selector',
            output='screen',
            parameters=[{
                # 관측 WP632~634, 확정은 WP634 를 넘어설 때(WP635).
                # 미션 매니저의 분기 실행은 WP638 이므로 판단이 3 WP 먼저
                # 끝난다. 경로가 실제로 갈라지는 곳은 WP641 이다.
                'observation_start_waypoint': 632,
                'decision_waypoint': 634,
                'decide_after_observation_scans': False,
                # 이 구간은 아직 8 km/h 다. 감속 램프는 WP633 부터지만
                # 시작점이라 속도가 그대로다. 1 m 씩 3 칸 = 약 1.43 초,
                # C1 10 Hz 기준 약 14 scan 밖에 안 쌓인다. T주차 구간
                # (37 scan) 보다 적으므로 votes 를 같이 낮춰 잡는다.
                'observation_scans': 14,
                # 1 표는 단발 노이즈 한 번에 확정된다. 0.3 초 연속 감지를
                # 요구한다. 14 scan 중 3 이므로 놓칠 여유는 남는다.
                'blocked_votes_required': 3,
                'min_points_per_scan': 8,
                'forward_angle_deg': 180.0,
                'zone_b_angle_min_deg': -15.0,
                'zone_b_angle_max_deg': 15.0,
                'zone_b_range_min_m': 0.05,
                'zone_b_range_max_m': 5.5,
                # 검사 창(정면 -15~+15도)은 P1 이 들어갈 칸을 본다.
                # 거기에 뭔가 있으면 1번 칸이 막힌 것이므로 2번으로 간다.
                #   감지   -> choice 2 -> P2 WP638 전환
                #   미감지 -> choice 1 -> P1 유지
                # slot_a_path 는 '감지됐을 때 고르는 경로' 이므로 로그가
                # 실제 동작과 맞도록 P2 를 둔다.
                'choice_when_blocked': 2,
                'slot_a_path': ROUTES + '/P2.csv',
                'slot_b_path': ROUTES + '/P1.csv',
                'publish_path': False,
                'choice_topic': '/competition/parallel_choice',
                'result_topic': '/parking/parallel_blocked_zone',
            }],
        ),
        # 카메라 2대. 한 노드가 한 장치를 독점하므로 따로 띄운다.
        # 장치 이름으로 잡는다. /dev/video 번호는 꽂는 순서에 따라 바뀐다.
        Node(
            package='t870_control', executable='webcam_pub_node',
            name='camera_front', output='screen',
            parameters=[{
                'device_name': 'c920',      # 외장 C920 -> 정적장애물
                'device_index': 2,
                'image_topic': '/camera/front/image_raw',
                'frame_id': 'camera_front',
                'fps': 15.0, 'width': 640, 'height': 480,
            }],
        ),
        Node(
            package='t870_control', executable='webcam_pub_node',
            name='camera_laptop', output='screen',
            parameters=[{
                # 'webcam' 은 C920 의 by-id 이름(HD_Pro_Webcam_C920)에도
                # 들어 있어 두 노드가 같은 장치를 잡는다. 제조사명으로
                # 고유하게 지정한다.
                'device_name': 'sonix',     # 노트북 내장 -> 신호등, X/화살표
                'device_index': 0,
                'image_topic': '/camera/laptop/image_raw',
                'frame_id': 'camera_laptop',
                'fps': 15.0, 'width': 640, 'height': 480,
            }],
        ),
        # 신호등 판정. lane_camera_preview 가 C920 을 독점하던 것을 떼어내
        # 노트북 카메라로 옮겼다. 원형 램프 전용 로직이다.
        Node(
            package='t870_control', executable='traffic_light_camera',
            name='traffic_light_camera', output='screen',
            parameters=[{
                'image_topic': '/camera/laptop/image_raw',
                'state_topic': '/vision/traffic_light_state',
                'confirm_frames': 3,
                'publish_debug_image': True,
                # [2026-09-19] 실차에서 정지선에 선 뒤로 초록불이 켜져도
                # GREEN 이 한 번도 안 나와 못 빠져나왔다. 램프가 가까워서
                # '너무 크다' 고 탈락하고 있었다(옛 상한 0.004 = 1228 px,
                # 4 m 램프가 1357 px). 프레임 대비 고정 비율 상한은 곧
                # '최소 동작 거리' 라 정지선에서 검출기가 꺼졌다.
                # 0.05 = 1.2 m 까지 덮는다(램프 면적 21642/d^2 px).
                # 그러면서 프레임의 5% 를 넘는 덩어리는 계속 버린다.
                # 0 으로 두면 상한 없음.
                'area_ceiling_ratio': 0.05,
                # 화면 아래 1/5 만 잘라낸다. 그 아래는 노면, 정지선 도색,
                # 잔디, 주황 라바콘이라 YELLOW 마스크에 걸린다.
                'roi_height_ratio': 0.8,
                # 배경의 붉은 물체가 켜진 초록 램프를 가리지 않게 한다.
                # [방향 주의] 문턱값이라 올리면 우대가 약해진다.
                # 0.5 = 붉은 물체가 램프 점수의 2 배까지는 초록이 이김.
                # 0.25 면 4 배까지. 0 이면 우대 없음.
                'green_preference': 0.5,
            }],
        ),
        # 마지막 WP 직선통과 판단. 표지(화살표/X)의 좌우 색 순서를 본다.
        #   왼쪽부터 빨강, 초록 -> OX_right.csv WP697 로 전환
        #   왼쪽부터 초록, 빨강 -> 가던 P 경로 그대로
        # 못 찾거나 확신이 없으면 False 를 유지해 경로를 바꾸지 않는다.
        # 신호등(원형 램프)과 다른 로직이다. 화살표는 길쭉하고 X 는 획이
        # 갈라져서, lane_camera_preview 의 원형도/종횡비 필터로는 못 잡는다.
        Node(
            package='t870_control',
            executable='ox_signal_detector',
            name='ox_signal_detector',
            output='screen',
            parameters=[{
                'image_topic': '/camera/laptop/image_raw',
                # X/화살표 전용 구간. 그 전까지는 신호등 판정만 쓴다.
                # 미션 매니저의 분기 실행은 WP693 부터 계속 검사하므로,
                # 이 구간 안에서 확정되는 즉시 전환된다. 경로가 실제로
                # 갈라지는 곳은 WP697 이다.
                'observation_start_waypoint': 690,
                'decision_waypoint': 695,
                # 구간 전체에서 많이 나온 쪽으로 정한다. 표가 이 수보다
                # 적으면 판단하지 않는다.
                'minimum_votes': 5,
                'min_area_ratio': 0.0006,
                'max_area_ratio': 0.15,
                # 표지가 LED 라 카메라 주파수와 안 맞아 획이 가로 줄무늬로
                # 끊겨 보인다. 이 커널로 조각을 다시 묶는다. 실차에서
                # /vision/ox_debug 를 보며 맞춘다.
                'merge_kernel_px': 15,
                'open_kernel_px': 3,
                'minimum_separation_ratio': 0.05,
                'maximum_rise_ratio': 0.25,
                'roi_x_min': 0.0,
                'roi_x_max': 1.0,
                'roi_y_min': 0.0,
                'roi_y_max': 0.75,
                'publish_debug_image': True,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('t870_control'), 'launch',
                't870_lane_autonomy.launch.py'])),
            launch_arguments={
                'lidar_serial_port': LIDAR,
                'arduino_serial_port': ARDUINO,
                'record_gps_path': 'false',
                'enable_localization': 'true',
                'enable_gps_follower': 'true',
                'gps_follow_path_file': ROUTES + '/T1.csv',
                # [2026-09-20] WP199~235 는 S자(헤어핀, 반경 약 7 m)다.
                # 2 km/h 속도 밴드 LAD 5.0 m 는 여기서 너무 길어 순수추종이
                # 코너를 크게 자른다. 실측 최대 횡오차
                #   LAD 5.0 -> 0.80 m,  3.0 -> 0.32 m,  1.5 -> 0.25 m,  1.0 -> 0.22 m
                # 조향률 제한이 있어 1.0 m 에서도 진동하지 않는다.
                # [주의] LAD 가 짧으면 경로로 되당기는 힘이 세져 같은 구간의
                # 카메라 회피 보정이 그만큼 안 먹는다. 38 절 참조.
                'gps_lookahead_override_ranges': '199:235',
                'gps_lookahead_override_m': '1.0',
                # follower 는 min(자체상한, 미션매니저 제한) 을 쓴다. 자체
                # 상한을 탈출속도까지 올려두고, 평소 속도는 미션 매니저가
                # /t870/gps_speed_limit 으로 계속 제한한다. 이걸 순항속도로
                # 두면 탈출 가속이 8 km/h 에서 잘린다.
                'gps_follow_speed_mps': LaunchConfiguration(
                    'gps_escape_speed_mps'),
                'require_operator_heartbeat': 'false',
                'lidar_safety_enabled': 'false',
                'enable_mission_stack': 'true',
                # 차선 인식은 대회(GPS 모드)에서 구동에 쓰이지 않는다.
                # 신호등 판정을 노트북 카메라로 옮겼으므로 이 노드가 할
                # 일이 없다. C920 을 정적장애물용으로 비워 주고 torch/
                # LaneNet 로딩도 없앤다.
                'enable_lane_preview': 'false',
                # Static obstacle avoidance owns control only in WP201..227.
                # [2026-09-19] 200 -> 201. 구간에 들어서는 순간 카메라가
                # 풀밭을 정면으로 봤다. 한 WP(약 1 m) 늦게 켠다.
                # 미션 매니저의 LIDAR_MISSIONS 표와 반드시 같아야 한다.
                # 한쪽만 바꾸면 감속만 하고 미션이 안 켜진다.
                'avoid_ranges': '201:227',
                # 동적장애물. WP531부터 정면 감시를 켠다. 장애물을 한 번
                # 서서 보내고 나면 emergency_stop_node 가 스스로 감시를
                # 끄므로 보통은 588까지 가지 않는다. 588은 장애물이 끝내
                # 나타나지 않았을 때를 위한 안전장치다. 평행주차 판단
                # 관측이 WP632에서 시작하는데, 그 구간까지 정면 3 m 감시가
                # 살아 있으면 주차 칸 벽/앞차를 장애물로 잡아 차가 서서
                # 안 나간다.
                'estop_ranges': '531:588',
                'parallel_ranges': '',
                'tpark_ranges': '',
                'traffic_light_ranges': '',
                'lane_signal_ranges': '',
                'parking_armed': 'false',
                'gps_allow_reverse': 'true',
                'gps_reverse_waypoint_ranges': '306:318',
                'gps_reverse_lookahead_m': LaunchConfiguration('reverse_lad'),
                'gps_invert_reverse_steering': 'true',
                'gps_reverse_cusp_deg': '0.0',
                'gps_reverse_start_waypoint': '-1',
                'gps_reverse_stop_waypoint': '-1',
                'gps_cusp_brake_distance_m': '0.35',
                'gps_shift_settle_sec': '0.3',
                'gps_waypoint_hold_sec': '1.0',
                'gps_nearest_forward_points': '5',
                'gps_nearest_backward_points': '2',
                'gps_max_waypoint_advance': '2',
                'gps_lad_2kmh_m': '2.0',
                'gps_lad_4kmh_m': '2.0',
                'gps_lad_6kmh_m': '2.5',
                'gps_startup_straight_sec': '2.0',
                'gps_follow_max_steer_rad': '0.20',
            }.items(),
        ),
    ])
