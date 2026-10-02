"""T870 autonomy pipeline for Pixhawk/ArduRover and SLAMTEC RPLIDAR C1.

MAVROS is expected to be running with the Pixhawk connection configured for the
navigation sensors. This launch starts the C1 driver and autonomy/control nodes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    serial_port = LaunchConfiguration('lidar_serial_port')
    target_velocity = LaunchConfiguration('target_velocity')
    require_rtk = LaunchConfiguration('require_rtk_quality')
    lidar_x = LaunchConfiguration('lidar_x')
    lidar_y = LaunchConfiguration('lidar_y')
    lidar_z = LaunchConfiguration('lidar_z')
    lidar_yaw = LaunchConfiguration('lidar_yaw')
    arduino_serial_port = LaunchConfiguration('arduino_serial_port')
    arduino_baud_rate = LaunchConfiguration('arduino_baud_rate')
    max_drive_speed_kmh = LaunchConfiguration('max_drive_speed_kmh')
    remote_web_port = LaunchConfiguration('remote_web_port')
    remote_control_pin = LaunchConfiguration('remote_control_pin')

    return LaunchDescription([
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/rplidar'),
        DeclareLaunchArgument('target_velocity', default_value='1.0'),
        DeclareLaunchArgument('require_rtk_quality', default_value='true'),
        # Measured from rear axle center (base_link) to the scan rotation axis.
        DeclareLaunchArgument('lidar_x', default_value='0.83'),
        # LiDAR is 0.75 cm to the left of the vehicle centerline.
        DeclareLaunchArgument('lidar_y', default_value='0.0075'),
        DeclareLaunchArgument('lidar_z', default_value='0.49'),
        # The housing is mounted at physical yaw 0 deg. The C1 scan frame +X
        # was previously verified to point rearward, hence the ROS frame
        # convention correction of pi radians remains required.
        DeclareLaunchArgument('lidar_yaw', default_value='3.14159'),
        DeclareLaunchArgument(
            'arduino_serial_port',
            default_value='/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_34331323136351211280-if00'),
        DeclareLaunchArgument('arduino_baud_rate', default_value='115200'),
        DeclareLaunchArgument('max_drive_speed_kmh', default_value='2.0'),
        DeclareLaunchArgument('remote_web_port', default_value='8080'),
        DeclareLaunchArgument('remote_control_pin', default_value='8700'),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='rplidar_c1',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': serial_port,
                'serial_baudrate': 460800,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
            remappings=[('scan', '/scan')],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['--x', lidar_x, '--y', lidar_y, '--z', lidar_z,
                       '--yaw', lidar_yaw, '--pitch', '0', '--roll', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        ),
        Node(
            package='ma_rrt_path_plan',
            executable='MaRRTPathPlanNode',
            name='ma_rrt_path_plan_node',
            output='screen',
            parameters=[{
                'require_rtk_quality': ParameterValue(require_rtk, value_type=bool),
                'lidar_x': ParameterValue(lidar_x, value_type=float),
                'lidar_y': ParameterValue(lidar_y, value_type=float),
                'lidar_yaw': ParameterValue(lidar_yaw, value_type=float),
            }],
        ),
        Node(
            package='t870_track',
            executable='t870_tracker',
            name='t870_tracker',
            output='screen',
            parameters=[{'target_velocity': ParameterValue(target_velocity, value_type=float)}],
            remappings=[('/cmd_vel', '/cmd_vel/auto')],
        ),
        Node(
            package='t870_control',
            executable='remote_mode_control_node',
            name='remote_mode_control_node',
            output='screen',
            parameters=[{
                'web_port': ParameterValue(remote_web_port, value_type=int),
                'control_pin': ParameterValue(remote_control_pin, value_type=str),
            }],
        ),
        Node(
            package='t870_control',
            executable='pixhawk_vehicle_interface_node',
            name='pixhawk_vehicle_interface_node',
            output='screen',
            parameters=[{
                'dynamic_stop_distance_m': 1.5,
                'dynamic_stop_angle_deg': 45.0,
                'dynamic_stop_hold_sec': 3.0,
                'dynamic_min_closing_speed_mps': 1.2,
                'dynamic_min_range_jump_m': 0.35,
                'dynamic_min_points': 3,
            }],
        ),
        Node(
            package='t870_control',
            executable='arduino_drive_node',
            name='arduino_drive_node',
            output='screen',
            parameters=[{
                'serial_port': arduino_serial_port,
                'baud_rate': ParameterValue(arduino_baud_rate, value_type=int),
                'max_speed_kmh': ParameterValue(max_drive_speed_kmh, value_type=float),
            }],
        ),
    ])
