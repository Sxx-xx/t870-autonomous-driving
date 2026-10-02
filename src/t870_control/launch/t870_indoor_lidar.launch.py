"""Low-speed indoor LiDAR avoidance for the T870.

MAVROS must already be connected with system_id=255. Startup mode is STOP;
the operator must explicitly select AUTO from the phone web interface.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lidar_port = LaunchConfiguration('lidar_serial_port')
    arduino_port = LaunchConfiguration('arduino_serial_port')
    web_port = LaunchConfiguration('remote_web_port')
    control_pin = LaunchConfiguration('remote_control_pin')
    auto_command_topic = LaunchConfiguration('auto_command_topic')
    gps_command_topic = LaunchConfiguration('gps_command_topic')
    rrt_command_topic = LaunchConfiguration('rrt_command_topic')
    startup_mode = LaunchConfiguration('startup_control_mode')
    require_heartbeat = LaunchConfiguration('require_operator_heartbeat')
    lidar_safety = LaunchConfiguration('lidar_safety_enabled')
    rc_arm_enabled = LaunchConfiguration('rc_arm_enabled')
    rc_arm_mode = LaunchConfiguration('rc_arm_mode')
    enable_lidar_planner = LaunchConfiguration('enable_lidar_planner')
    vehicle_estop_enabled = LaunchConfiguration('vehicle_estop_enabled')

    return LaunchDescription([
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument(
            'arduino_serial_port',
            default_value='/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_34331323136351211280-if00'),
        DeclareLaunchArgument('remote_web_port', default_value='8080'),
        DeclareLaunchArgument('remote_control_pin', default_value='8700'),
        DeclareLaunchArgument('auto_command_topic', default_value='/cmd_vel/auto'),
        DeclareLaunchArgument('gps_command_topic', default_value='/cmd_vel/gps'),
        DeclareLaunchArgument('rrt_command_topic', default_value='/cmd_vel/auto'),
        DeclareLaunchArgument('startup_control_mode', default_value='STOP'),
        DeclareLaunchArgument('require_operator_heartbeat', default_value='true'),
        DeclareLaunchArgument(
            'lidar_safety_enabled', default_value='true',
            description='LiDAR obstacle E-stop. false leaves GPS tracking '
                        'entirely on its own, with no obstacle stop at all'),
        DeclareLaunchArgument(
            'rc_arm_enabled', default_value='true',
            description='Let the physical transmitter AUTO switch arm the mode'),
        DeclareLaunchArgument(
            'start_gps_recording', default_value='false',
            description='켜면 런치와 동시에 GPS 경로 녹화가 시작된다'),
        DeclareLaunchArgument(
            'rc_arm_mode', default_value='GPS',
            description='Mode selected when the transmitter switches to AUTO'),
        DeclareLaunchArgument(
            'enable_lidar_planner', default_value='true',
            description='Start the RRT LiDAR planner/tracker'),
        DeclareLaunchArgument(
            'vehicle_estop_enabled', default_value='true',
            description='Enable Pixhawk vehicle E-stop logic'),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='rplidar_c1',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': lidar_port,
                'serial_baudrate': 460800,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
            remappings=[('scan', '/scan')],
        ),
        Node(
            package='ma_rrt_path_plan',
            executable='MaRRTPathPlanNode',
            name='ma_rrt_path_plan_node',
            output='screen',
            condition=IfCondition(enable_lidar_planner),
            parameters=[{
                'indoor_lidar_mode': True,
                'bench_heading_yaw': 3.14159,
                'world_frame': 'laser',
                'base_frame': 'laser',
                'planning_distance': 2.0,
                'planning_rate_hz': 5.0,
                'obstacle_max_range': 2.5,
                # Measured vehicle width is 0.78 m (0.39 m half-width).
                # Add about 0.09 m low-speed clearance around the footprint.
                'obstacle_radius': 0.48,
                'obstacle_voxel_size': 0.15,
                # Measured LiDAR-to-side body extent is about 0.38 m. Remove
                # returns inside the vehicle footprint so the inflated 0.48 m
                # obstacles cannot place the planner start inside its own body.
                'self_filter_radius': 0.50,
            }],
        ),
        Node(
            package='t870_track',
            executable='t870_tracker',
            name='t870_tracker',
            output='screen',
            condition=IfCondition(enable_lidar_planner),
            parameters=[{
                'target_velocity': 0.15,
                # Indoor RRT paths are only about 2 m long. The outdoor 1.2 m
                # default can skip the nearby detour and aim at a later,
                # nearly straight waypoint.
                # RRT expands in roughly 0.5 m steps. Stay beyond the first
                # straight node so Pure Pursuit selects the first detour node.
                'minimum_lookahead': 0.65,
                'maximum_steering_angle': 0.25,
                'input_timeout': 1.0,
                'use_speed_pid': False,
                'positive_steering_is_right': True,
            }],
            remappings=[('/cmd_vel', rrt_command_topic)],
        ),
        Node(
            package='t870_control',
            executable='remote_mode_control_node',
            name='remote_mode_control_node',
            output='screen',
            parameters=[{
                'web_port': ParameterValue(web_port, value_type=int),
                'control_pin': ParameterValue(control_pin, value_type=str),
                'max_manual_speed_mps': 2.78,
                'require_operator_heartbeat': ParameterValue(
                    require_heartbeat, value_type=bool),
                'startup_mode': startup_mode,
                'rc_arm_enabled': ParameterValue(
                    rc_arm_enabled, value_type=bool),
                'rc_arm_mode': ParameterValue(rc_arm_mode, value_type=str),
                'start_gps_recording': ParameterValue(
                    LaunchConfiguration('start_gps_recording'),
                    value_type=bool),
                'operator_timeout_sec': 5.0,
                'auto_command_topic': auto_command_topic,
                'gps_command_topic': gps_command_topic,
            }],
        ),
        Node(
            package='t870_control',
            executable='pixhawk_vehicle_interface_node',
            name='pixhawk_vehicle_interface_node',
            output='screen',
            parameters=[{
                # lidar_safety_enabled:=false 로 주면 require_lidar 가 꺼지고
                # 정적/동적 장애물 거리가 사실상 0 이 되어 LiDAR 가 주행에
                # 전혀 개입하지 않는다. GPS 추종만 따로 보고 싶을 때 쓴다.
                'require_lidar': ParameterValue(lidar_safety, value_type=bool),
                'vehicle_estop_enabled': ParameterValue(
                    vehicle_estop_enabled, value_type=bool),
                'lidar_forward_angle_deg': 180.0,
                'estop_distance_m': ParameterValue(
                    PythonExpression(
                        ["'0.50' if '", lidar_safety, "'.lower() in ",
                         "('true','1') else '0.01'"]),
                    value_type=float),
                # Protect a wider front-corner sector at the 0.50 m emergency
                # distance while still allowing obstacles alongside the rear.
                'estop_angle_deg': 60.0,
                # Sudden person/object intrusion: latch E-stop for 3 seconds.
                'dynamic_stop_distance_m': ParameterValue(
                    PythonExpression(
                        ["'2.0' if '", lidar_safety, "'.lower() in ",
                         "('true','1') else '0.01'"]),
                    value_type=float),
                'dynamic_stop_angle_deg': 45.0,
                'dynamic_stop_hold_sec': 3.0,
                'dynamic_min_closing_speed_mps': 1.2,
                'dynamic_min_range_jump_m': 0.35,
                'dynamic_min_points': 3,
                # The firmware reads 1100/1500/1900 us as lock-to-lock
                # (sketch_aug.ino PIXHAWK_PWM_MIN/CENTER/MAX_US). The old
                # 1000/2000 defaults made every command 1.25x wider than the
                # span the firmware maps, so the wheels overshot the commanded
                # angle and Pure Pursuit oscillated on straight sections.
                'steering_pwm_min': 1100,
                'steering_pwm_trim': 1500,
                'steering_pwm_max': 1900,
                # Measured 2026-09-14 from full-lock circles: 2.605 m radius
                # left and 2.651 m right. At the re-measured 0.725 m wheelbase
                # that is 15.55 deg and 15.30 deg. The smaller of the two goes
                # here so neither side saturates early; commanded and actual
                # steering now match 1:1.
                'max_steer_angle_rad': 0.2670,
            }],
        ),
        Node(
            package='t870_control',
            executable='arduino_drive_node',
            name='arduino_drive_node',
            output='screen',
            parameters=[{
                'serial_port': arduino_port,
                # 2.78 m/s is approximately 10.0 km/h.
                'max_speed_kmh': 10.0,
            }],
        ),
    ])
