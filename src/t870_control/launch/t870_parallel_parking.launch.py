"""T870 GPS parallel parking with LiDAR path selection.

  ros2 launch t870_control t870_parallel_parking.launch.py

LiDAR selection:
  - watch waypoint 4 segment, decide when waypoint advances to 5
  - detected -> p_a.csv
  - not detected -> p_b.csv
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


WORKSPACE = '/home/sxx/Desktop/colcon_ws./colcon_ws/colcon_ws'
LIDAR = ('/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_'
         'Controller_dc29dffaaa31f111b5fb935f30d20014-if00-port0')
ARDUINO = ('/dev/serial/by-id/'
           'usb-Arduino__www.arduino.cc__0043_34331323136351211280-if00')


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'path', default_value=WORKSPACE + '/gps_recordings/p_a.csv'),
        DeclareLaunchArgument(
            'gps_speed_mps', default_value='0.56',
            description='GPS parallel parking follow speed; 0.56 m/s ~= 2 km/h'),
        DeclareLaunchArgument(
            'reverse_lad', default_value='1.0',
            description='Reverse LAD for tight parking maneuvers'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='parallel_parking_rviz',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('t870_control'), 'rviz', 'lidar.rviz'])],
        ),
        Node(
            package='t870_control',
            executable='lidar_sector_filter',
            name='parallel_lidar_sector_filter',
            output='screen',
            parameters=[{
                'forward_angle_deg': 180.0,
                'angle_min_deg': -15.0,
                'angle_max_deg': 15.0,
                'range_min_m': 0.05,
                'range_max_m': 5.5,
            }],
        ),
        Node(
            package='t870_control',
            executable='parking_slot_selector',
            name='parallel_path_selector',
            output='screen',
            parameters=[{
                'observation_start_waypoint': 4,
                'decision_waypoint': 4,
                'decide_after_observation_scans': False,
                'observation_scans': 5,
                'blocked_votes_required': 1,
                'min_points_per_scan': 8,
                'forward_angle_deg': 180.0,
                'zone_b_angle_min_deg': -15.0,
                'zone_b_angle_max_deg': 15.0,
                'zone_b_range_min_m': 0.05,
                'zone_b_range_max_m': 5.5,
                'slot_a_path': WORKSPACE + '/gps_recordings/p_a.csv',
                'slot_b_path': WORKSPACE + '/gps_recordings/p_b.csv',
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
                'gps_allow_reverse': 'true',
                'gps_reverse_lookahead_m': LaunchConfiguration('reverse_lad'),
                'gps_invert_reverse_steering': 'true',
                'gps_reverse_cusp_deg': '45.0',
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
