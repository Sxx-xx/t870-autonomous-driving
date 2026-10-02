"""T870 주차 주행. 값이 전부 박혀 있어 인자 없이 돌린다.

주행용(t870_lane_autonomy.launch.py 기본값)과 주차용은 필요한 값이
정반대다. 주행은 10 km/h 에 LAD 5~8 m 로 멀리 보고, 주차는 3.6 km/h 에
LAD 2 m 로 바짝 붙는다. 매번 인자 스무 개를 손으로 치면 하나 빠뜨렸을 때
원인을 찾느라 시간을 버린다. 그래서 따로 뒀다.

  ros2 launch t870_control t870_parking.launch.py

바꾸고 싶은 것만 인자로 준다.

  ros2 launch t870_control t870_parking.launch.py reverse_lad:=1.5
  ros2 launch t870_control t870_parking.launch.py path:=.../t_p.csv
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

WORKSPACE = '/home/sxx/Desktop/colcon_ws./colcon_ws/colcon_ws'
LIDAR = ('/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_'
         'Controller_dc29dffaaa31f111b5fb935f30d20014-if00-port0')
ARDUINO = ('/dev/serial/by-id/'
           'usb-Arduino__www.arduino.cc__0043_34331323136351211280-if00')


def generate_launch_description():
    return LaunchDescription([
        # 자주 바꾸는 둘만 인자로 남긴다.
        DeclareLaunchArgument(
            'path', default_value=WORKSPACE + '/gps_recordings/t_p(aa).csv'),
        DeclareLaunchArgument(
            'cone_select', default_value='false',
            description='라이다로 콘을 보고 A/B 칸을 고른다'),
        DeclareLaunchArgument('decision_waypoint', default_value='8'),
        DeclareLaunchArgument(
            'reverse_lad', default_value='2.0',
            description='후진 LAD. 양수=고정, 0=다음 점만, 음수=전진과 같은 밴드'),
        DeclareLaunchArgument(
            'gps_speed_mps', default_value='1.0',
            description='GPS 주차 추종 최고속도(m/s); 1 km/h=0.2778 m/s'),
        # 주차 런치와 함께 라이다 화면을 바로 확인한다. lidar.rviz는
        # 원본 /scan과 주차 감지 범위 /parking/scan_window를 함께 보인다.
        Node(
            package='rviz2',
            executable='rviz2',
            name='parking_rviz',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('t870_control'), 'rviz', 'lidar.rviz'])],
        ),
        Node(
            package='t870_control',
            executable='lidar_sector_filter',
            name='lidar_sector_filter',
            output='screen',
            parameters=[{
                'forward_angle_deg': 180.0,
                'angle_min_deg': 80.0,
                'angle_max_deg': 100.0,
                'range_min_m': 0.05,
                'range_max_m': 4.0,
            }],
        ),
        Node(
            package='t870_control',
            executable='t_path_selector',
            name='t_path_selector',
            output='screen',
            parameters=[{
                # 1차 T자 선택: WP13에서 보이면 B가 막힘 -> A 경로,
                # WP15에서 보이면 A가 막힘 -> B 경로. WP20에서 결정한다.
                'stage1_start_waypoint': 13,
                'stage1_decision_waypoint': 20,
                'stage1_b_block_waypoint': 13,
                'stage1_a_block_waypoint': 15,
                'stage1_default_prefix': 'b',
                'stage2_start_waypoint': 27,
                'stage2_decision_waypoint': 29,
                'enable_stage2': False,
                'min_points_per_scan': 8,
                'forward_angle_deg': 180.0,
                # 반대편 감지창.
                'angle_min_deg': 80.0,
                'angle_max_deg': 100.0,
                'range_min_m': 0.05,
                'range_max_m': 4.0,
                'path_aa': WORKSPACE + '/gps_recordings/t_p(aa).csv',
                'path_ba': WORKSPACE + '/gps_recordings/t_p(ba).csv',
                'path_ab': WORKSPACE + '/gps_recordings/t_p(ab).csv',
                'path_bb': WORKSPACE + '/gps_recordings/t_p(bb).csv',
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
                'gps_follow_path_file': LaunchConfiguration('path'),
                'gps_follow_speed_mps': LaunchConfiguration('gps_speed_mps'),
                'require_operator_heartbeat': 'false',
                'lidar_safety_enabled': 'false',
                'enable_mission_stack': 'false',
                'enable_lane_preview': 'false',

                # 후진
                'gps_allow_reverse': 'true',
                'gps_reverse_lookahead_m': LaunchConfiguration('reverse_lad'),
                'gps_reverse_cusp_deg': '45.0',
                # T자 네 경로는 파일마다 후진 끝점이 다르다. 고정 WP를
                # 쓰면 WP22/24 전환부에서 후진/전진 루프가 생기므로,
                # follower가 계산한 reverse segment 끝까지 간다.
                'gps_reverse_stop_waypoint': '-1',
                # 판정 뒤에도 제동 거리만큼 더 간다. 09-16 실측에서
                # 판정 +0.06 m -> 최종 +0.44 m 로 0.38 m 밀렸다.
                'gps_reverse_overshoot_m': '-0.38',
                # 첨점 접근 중 미리 방향을 바꾸면 WP12에서 조기 후진한다.
                # 주차 경로의 방향 표식대로 WP14에서만 전환한다.
                'gps_cusp_brake_distance_m': '0.0',
                'gps_shift_settle_sec': '1.2',
                'gps_waypoint_hold_sec': '1.0',

                # 경로가 자기 위로 되돌아오므로 최근접 탐색을 좁힌다.
                # 넓으면 후진 도중에 탈출 구간 점이 더 가깝게 잡혀 색인이
                # 건너뛴다.
                'gps_nearest_forward_points': '5',
                'gps_nearest_backward_points': '2',
                'gps_max_waypoint_advance': '2',

                # 좁은 조작이라 가까이 본다.
                'gps_lad_2kmh_m': '2.0',
                'gps_lad_4kmh_m': '2.0',
                'gps_lad_6kmh_m': '2.5',
                'gps_startup_straight_sec': '2.0',
                'gps_follow_max_steer_rad': '0.26',
            }.items(),
        ),
        Node(
            package='t870_control',
            executable='cone_park_selector',
            name='cone_park_selector',
            output='screen',
            condition=IfCondition(LaunchConfiguration('cone_select')),
            parameters=[{
                'path_a': WORKSPACE + '/gps_recordings/park_a.csv',
                'path_b': WORKSPACE + '/gps_recordings/park_b.csv',
                'decision_waypoint': ParameterValue(
                    LaunchConfiguration('decision_waypoint'), value_type=int),
                # 경로 기하에서 뽑은 값이다. 판정 웨이포인트 8 에서
                # A 칸은 -67.6~-58.7도, B 칸은 -87.3~-86.4도에 보인다.
                # 여기에 여유를 5도씩 더했다.
                'sector_a_min_deg': -73.0,
                'sector_a_max_deg': -54.0,
                'sector_b_min_deg': -92.0,
                'sector_b_max_deg': -81.0,
                'range_min_m': 3.5,
                'range_max_m': 8.0,
                'minimum_points': 4,
                'frames': 8,
                'lidar_forward_deg': 180.0,
            }],
        ),
    ])
