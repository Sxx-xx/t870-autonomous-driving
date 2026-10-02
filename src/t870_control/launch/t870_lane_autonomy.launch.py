"""T870 LiDAR safety stack with C920 lane following as the AUTO source.

MAVROS must already be running with system_id=255. The remote controller still
starts in STOP; AUTO must be selected explicitly from the phone interface.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lidar_port = LaunchConfiguration('lidar_serial_port')
    arduino_port = LaunchConfiguration('arduino_serial_port')
    camera_device = LaunchConfiguration('camera_device')
    target_speed = LaunchConfiguration('target_speed_mps')
    save_diagnostics = LaunchConfiguration('save_diagnostics')
    record_gps_path = LaunchConfiguration('record_gps_path')
    gps_device = LaunchConfiguration('gps_device')
    gps_path_name = LaunchConfiguration('gps_path_name')
    gps_output_directory = LaunchConfiguration('gps_output_directory')
    enable_gps_follower = LaunchConfiguration('enable_gps_follower')
    gps_follow_path_file = LaunchConfiguration('gps_follow_path_file')
    gps_follow_speed = LaunchConfiguration('gps_follow_speed_mps')
    gps_follow_max_steer = LaunchConfiguration('gps_follow_max_steer_rad')
    gps_lad_2kmh = LaunchConfiguration('gps_lad_2kmh_m')
    gps_lad_4kmh = LaunchConfiguration('gps_lad_4kmh_m')
    gps_lad_6kmh = LaunchConfiguration('gps_lad_6kmh_m')
    gps_lad_8kmh = LaunchConfiguration('gps_lad_8kmh_m')
    gps_lad_10kmh = LaunchConfiguration('gps_lad_10kmh_m')
    enable_localization = LaunchConfiguration('enable_localization')
    enable_lane_preview = LaunchConfiguration('enable_lane_preview')
    enable_mission_stack = LaunchConfiguration('enable_mission_stack')
    wheelbase = LaunchConfiguration('wheelbase_m')
    startup_mode = LaunchConfiguration('startup_control_mode')
    require_heartbeat = LaunchConfiguration('require_operator_heartbeat')
    lidar_safety = LaunchConfiguration('lidar_safety_enabled')
    rc_arm_enabled = LaunchConfiguration('rc_arm_enabled')
    rc_arm_mode = LaunchConfiguration('rc_arm_mode')
    mission_launch = PathJoinSubstitution([
        FindPackageShare('t870_control'), 'launch', 't870_missions.launch.py'])
    gps_enabled = PythonExpression([
        "'", record_gps_path, "'.lower() == 'true' or '",
        enable_gps_follower, "'.lower() == 'true'",
    ])

    indoor_launch = PathJoinSubstitution([
        FindPackageShare('t870_control'), 'launch',
        't870_indoor_lidar.launch.py',
    ])
    lane_launch = PathJoinSubstitution([
        FindPackageShare('t870_control'), 'launch',
        't870_lane_preview.launch.py',
    ])
    gps_launch = PathJoinSubstitution([
        FindPackageShare('t870_control'), 'launch',
        't870_smc2000_gps.launch.py',
    ])
    localization_launch = PathJoinSubstitution([
        FindPackageShare('t870_control'), 'launch',
        't870_localization.launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument(
            'arduino_serial_port',
            default_value='/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_34331323136351211280-if00'),
        DeclareLaunchArgument('camera_device', default_value='auto'),
        DeclareLaunchArgument('target_speed_mps', default_value='0.30'),
        DeclareLaunchArgument('save_diagnostics', default_value='false'),
        DeclareLaunchArgument(
            'record_gps_path', default_value='true',
            description='Record /gps/fix as CSV and T870 TXT at 1 m spacing'),
        DeclareLaunchArgument(
            'enable_mission_stack', default_value='true',
            description='Start mission LiDAR/obstacle safety nodes'),
        DeclareLaunchArgument(
            'enable_lane_preview', default_value='true',
            description='Start camera lane and traffic-stop node'),
        DeclareLaunchArgument(
            'gps_device',
            default_value='/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00'),
        DeclareLaunchArgument('gps_path_name', default_value=''),
        DeclareLaunchArgument(
            'gps_start_recording', default_value='false',
            description='런치와 동시에 녹화 시작. Ctrl+C 로 저장하고 끝낸다'),
        DeclareLaunchArgument(
            'gps_record_min_fix', default_value='0',
            description='녹화에 쓸 최저 fix. 2=RTK, 1=DGPS, 0=아무거나'),
        DeclareLaunchArgument(
            'gps_output_directory',
            default_value='/home/sxx/Desktop/colcon_ws./colcon_ws/colcon_ws/gps_recordings'),
        DeclareLaunchArgument(
            'enable_gps_follower', default_value='false',
            description='Start saved GPS route follower and enable GPS PATH mode'),
        DeclareLaunchArgument(
            'gps_follow_path_file',
            default_value='/home/sxx/Desktop/colcon_ws./colcon_ws/colcon_ws/gps_recordings/1.csv',
            description='Absolute CSV or TXT route file to follow'),
        # 2.7778 m/s is approximately 10.0 km/h.
        DeclareLaunchArgument('gps_follow_speed_mps', default_value='2.7778'),
        DeclareLaunchArgument(
            'gps_follow_max_steer_rad', default_value='0.24',
            description='GPS follower steering limit; 0.24 rad = min radius 2.96 m (lock 0.267)'),
        DeclareLaunchArgument(
            'minimum_fix_status', default_value='2',
            description='Lowest NavSatStatus to drive on. 2=RTK, 1=DGPS, 0=any'),
        DeclareLaunchArgument(
            'rtk_restart_enabled', default_value='false',
            description='RTK 가 끊기면 GPS 노드를 죽였다 바로 다시 띄운다'),
        DeclareLaunchArgument(
            'rtk_restart_after_sec', default_value='5.0',
            description='이 시간 동안 RTK 가 없으면 GPS 노드 재시작'),
        DeclareLaunchArgument(
            'gps_allow_reverse', default_value='false',
            description='경로의 target_speed 가 음수인 구간을 후진으로 주행'),
        DeclareLaunchArgument(
            'gps_reverse_waypoint_ranges', default_value='',
            description='명시적 후진 WP 범위. 예: 306:318,641:647'),
        DeclareLaunchArgument(
            'gps_reverse_cusp_deg', default_value='45.0',
            description='이웃 구간이 이만큼 꺾이면 첨점으로 보고 후진으로 전환. 0 이면 끔'),
        DeclareLaunchArgument(
            'gps_nearest_forward_points', default_value='30',
            description='최근접 탐색 창(앞). 주차처럼 경로가 겹치면 작게'),
        DeclareLaunchArgument(
            'gps_nearest_backward_points', default_value='5',
            description='최근접 탐색 창(뒤)'),
        DeclareLaunchArgument(
            'gps_max_waypoint_advance', default_value='10',
            description='한 제어 주기에 넘어갈 수 있는 웨이포인트 수'),
        DeclareLaunchArgument(
            'gps_reverse_overshoot_m', default_value='0.0',
            description='정지 판정을 이만큼 앞당긴다. 음수면 일찍 건다'),
        DeclareLaunchArgument(
            'gps_reverse_start_waypoint', default_value='-1',
            description='이 웨이포인트에서 E-stop 후 후진 시작. -1이면 cusp 자동 판정'),
        DeclareLaunchArgument(
            'gps_reverse_stop_waypoint', default_value='-1',
            description='후진을 멈출 웨이포인트 번호. -1 이면 후진 구간 끝점'),
        DeclareLaunchArgument(
            'gps_reverse_lookahead_m', default_value='0.0',
            description='후진 중 LAD. 양수=고정, 0=다음 웨이포인트만, 음수=전진과 같은 밴드'),
        DeclareLaunchArgument(
            'gps_invert_reverse_steering', default_value='true',
            description='후진 조향 부호 반전. 평행주차에서 반대로 꺾이면 false'),
        DeclareLaunchArgument(
            'gps_reverse_course_speed_mps', default_value='0.6',
            description='후진 중 이 속도 아래의 GPS 방위는 잡음으로 보고 안 쓴다'),
        DeclareLaunchArgument(
            'gps_cusp_brake_distance_m', default_value='1.5',
            description='다음 첨점이 이 거리 안이면 미리 제동을 시작한다'),
        DeclareLaunchArgument(
            'gps_shift_settle_sec', default_value='0.3',
            description='전/후진 전환 직후 조향을 중립으로 묶는 시간'),
        DeclareLaunchArgument(
            'gps_waypoint_hold_sec', default_value='3.0',
            description='target_speed 가 0 인 웨이포인트에서 멈춰 있을 시간'),
        DeclareLaunchArgument(
            'gps_startup_straight_sec', default_value='5.0',
            description='AUTO 전환 후 조향 0 으로 직진하는 시간. 짧은 경로에서는 줄인다'),
        DeclareLaunchArgument('gps_lad_2kmh_m', default_value='5.0'),
        # 특정 WP 구간에서만 LAD 를 짧게 쓴다. S자/헤어핀용.
        DeclareLaunchArgument('gps_lookahead_override_ranges', default_value=''),
        DeclareLaunchArgument('gps_lookahead_override_m', default_value='0.0'),
        DeclareLaunchArgument('gps_lad_4kmh_m', default_value='6.0'),
        DeclareLaunchArgument('gps_lad_6kmh_m', default_value='7.0'),
        DeclareLaunchArgument('gps_lad_8kmh_m', default_value='7.0'),
        DeclareLaunchArgument('gps_lad_10kmh_m', default_value='8.0'),
        DeclareLaunchArgument(
            'enable_localization', default_value='false',
            description='Start GPS+Pixhawk IMU+Arduino encoder dual EKF'),
        DeclareLaunchArgument(
            'wheelbase_m', default_value='0.725',
            description='Distance between front and rear axle centers'),
        DeclareLaunchArgument('avoid_ranges', default_value=''),
        DeclareLaunchArgument('estop_ranges', default_value=''),
        # lane_camera_preview 가 카메라를 독점한다. 프레임이 필요한
        # 노드(ox_signal_detector)를 위해 원본 발행을 켤 수 있다.
        DeclareLaunchArgument(
            'lane_publish_camera_image', default_value='false'),
        DeclareLaunchArgument('parallel_ranges', default_value=''),
        DeclareLaunchArgument('tpark_ranges', default_value=''),
        DeclareLaunchArgument('traffic_light_ranges', default_value=''),
        DeclareLaunchArgument('lane_signal_ranges', default_value=''),
        DeclareLaunchArgument('parking_armed', default_value='false'),
        DeclareLaunchArgument('lane_signal_path_file', default_value=''),
        DeclareLaunchArgument('startup_control_mode', default_value='STOP'),
        DeclareLaunchArgument('require_operator_heartbeat', default_value='true'),
        DeclareLaunchArgument(
            'use_imu_heading', default_value='false',
            description='Use compass heading when GPS course is unavailable'),
        DeclareLaunchArgument(
            'lidar_safety_enabled', default_value='true',
            description='false = LiDAR obstacle stop off, GPS tracking only'),
        DeclareLaunchArgument('rc_arm_enabled', default_value='true'),
        DeclareLaunchArgument('rc_arm_mode', default_value='GPS'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(indoor_launch),
            launch_arguments={
                'lidar_serial_port': lidar_port,
                'arduino_serial_port': arduino_port,
                'auto_command_topic': '/cmd_vel/mission_auto',
                # When the mission stack is enabled, route GPS commands
                # through its mux so AVOID can temporarily take control and
                # automatically return to nominal GPS tracking afterward.
                'gps_command_topic': PythonExpression([
                    "'/cmd_vel/mission_gps' if '", enable_mission_stack,
                    "'.lower() in ('true','1') else '/cmd_vel/gps'",
                ]),
                'rrt_command_topic': '/cmd_vel/rrt',
                'startup_control_mode': startup_mode,
                'require_operator_heartbeat': require_heartbeat,
                'lidar_safety_enabled': lidar_safety,
                'vehicle_estop_enabled': 'false',
                'rc_arm_enabled': rc_arm_enabled,
                'rc_arm_mode': rc_arm_mode,
                'enable_lidar_planner': 'false',
                'start_gps_recording': LaunchConfiguration(
                    'gps_start_recording'),
            }.items(),
        ),
        Node(
            package='t870_control',
            executable='rrt_avoidance_mux',
            name='rrt_avoidance_mux',
            output='screen',
            parameters=[{
                'avoidance_speed_mps': 0.25,
                'detour_enter_lateral_m': 0.20,
                'detour_exit_lateral_m': 0.10,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mission_launch),
            condition=IfCondition(LaunchConfiguration('enable_mission_stack')),
            launch_arguments={
                'avoid_ranges': LaunchConfiguration('avoid_ranges'),
                'estop_ranges': LaunchConfiguration('estop_ranges'),
                'parallel_ranges': LaunchConfiguration('parallel_ranges'),
                'tpark_ranges': LaunchConfiguration('tpark_ranges'),
                'traffic_light_ranges': LaunchConfiguration('traffic_light_ranges'),
                'lane_signal_ranges': LaunchConfiguration('lane_signal_ranges'),
                'parking_armed': LaunchConfiguration('parking_armed'),
                'lane_signal_path_file': LaunchConfiguration('lane_signal_path_file'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            condition=IfCondition(enable_localization),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lane_launch),
            condition=IfCondition(enable_lane_preview),
            launch_arguments={
                'camera_device': camera_device,
                'control_enabled': 'true',
                'target_speed_mps': target_speed,
                'save_diagnostics': save_diagnostics,
                'publish_camera_image': LaunchConfiguration(
                    'lane_publish_camera_image'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gps_launch),
            condition=IfCondition(gps_enabled),
            launch_arguments={
                'device': gps_device,
                'rtk_restart_enabled': LaunchConfiguration('rtk_restart_enabled'),
                'rtk_restart_after_sec': LaunchConfiguration(
                    'rtk_restart_after_sec'),
            }.items(),
        ),
        Node(
            package='t870_control',
            executable='gps_path_recorder',
            name='gps_path_recorder',
            output='screen',
            condition=IfCondition(record_gps_path),
            parameters=[{
                'fix_topic': '/gps/fix',
                'output_directory': gps_output_directory,
                'path_name': gps_path_name,
                'minimum_distance_m': 1.0,
                'target_speed': ParameterValue(target_speed, value_type=float),
                'minimum_fix_status': ParameterValue(
                    LaunchConfiguration('gps_record_min_fix'), value_type=int),
                'start_recording': ParameterValue(
                    LaunchConfiguration('gps_start_recording'),
                    value_type=bool),
            }],
        ),
        Node(
            package='t870_control',
            executable='gps_path_follower',
            name='gps_path_follower',
            output='screen',
            condition=IfCondition(enable_gps_follower),
            parameters=[{
                'path_file': gps_follow_path_file,
                'target_speed_mps': ParameterValue(
                    gps_follow_speed, value_type=float),
                'wheelbase_m': ParameterValue(wheelbase, value_type=float),
                'maximum_steering_rad': ParameterValue(
                    gps_follow_max_steer, value_type=float),
                'minimum_fix_status': ParameterValue(
                    LaunchConfiguration('minimum_fix_status'), value_type=int),
                'use_imu_heading': ParameterValue(
                    LaunchConfiguration('use_imu_heading'), value_type=bool),
                'reverse_cusp_deg': ParameterValue(
                    LaunchConfiguration('gps_reverse_cusp_deg'),
                    value_type=float),
                'nearest_search_forward_points': ParameterValue(
                    LaunchConfiguration('gps_nearest_forward_points'),
                    value_type=int),
                'nearest_search_backward_points': ParameterValue(
                    LaunchConfiguration('gps_nearest_backward_points'),
                    value_type=int),
                'maximum_waypoint_advance_points': ParameterValue(
                    LaunchConfiguration('gps_max_waypoint_advance'),
                    value_type=int),
                'reverse_overshoot_m': ParameterValue(
                    LaunchConfiguration('gps_reverse_overshoot_m'),
                    value_type=float),
                'reverse_stop_waypoint': ParameterValue(
                    LaunchConfiguration('gps_reverse_stop_waypoint'),
                    value_type=int),
                'reverse_start_waypoint': ParameterValue(
                    LaunchConfiguration('gps_reverse_start_waypoint'),
                    value_type=int),
                'reverse_lookahead_m': ParameterValue(
                    LaunchConfiguration('gps_reverse_lookahead_m'),
                    value_type=float),
                'invert_reverse_steering': ParameterValue(
                    LaunchConfiguration('gps_invert_reverse_steering'),
                    value_type=bool),
                'reverse_course_speed_mps': ParameterValue(
                    LaunchConfiguration('gps_reverse_course_speed_mps'),
                    value_type=float),
                'cusp_brake_distance_m': ParameterValue(
                    LaunchConfiguration('gps_cusp_brake_distance_m'),
                    value_type=float),
                'shift_settle_sec': ParameterValue(
                    LaunchConfiguration('gps_shift_settle_sec'),
                    value_type=float),
                'waypoint_hold_sec': ParameterValue(
                    LaunchConfiguration('gps_waypoint_hold_sec'),
                    value_type=float),
                'startup_straight_sec': ParameterValue(
                    LaunchConfiguration('gps_startup_straight_sec'),
                    value_type=float),
                'allow_reverse': ParameterValue(
                    LaunchConfiguration('gps_allow_reverse'), value_type=bool),
                'reverse_waypoint_ranges': LaunchConfiguration(
                    'gps_reverse_waypoint_ranges'),
                'use_speed_banded_lookahead': True,
                'lookahead_override_ranges': ParameterValue(
                    LaunchConfiguration('gps_lookahead_override_ranges'),
                    value_type=str),
                'lookahead_override_m': ParameterValue(
                    LaunchConfiguration('gps_lookahead_override_m'),
                    value_type=float),
                'lookahead_2kmh_m': ParameterValue(
                    gps_lad_2kmh, value_type=float),
                'lookahead_4kmh_m': ParameterValue(
                    gps_lad_4kmh, value_type=float),
                'lookahead_6kmh_m': ParameterValue(
                    gps_lad_6kmh, value_type=float),
                'lookahead_8kmh_m': ParameterValue(
                    gps_lad_8kmh, value_type=float),
                'lookahead_10kmh_m': ParameterValue(
                    gps_lad_10kmh, value_type=float),
                'output_topic': '/cmd_vel/gps',
                'positive_steering_is_right': True,
            }],
        ),
    ])
