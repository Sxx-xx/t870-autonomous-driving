"""Start only the parking LiDAR, its calibrated sector filter, and RViz."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


LIDAR = ('/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_'
         'Controller_dc29dffaaa31f111b5fb935f30d20014-if00-port0')


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='rplidar_c1',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': LIDAR,
                'serial_baudrate': 460800,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
            remappings=[('scan', '/scan')],
        ),
        Node(
            package='t870_control',
            executable='lidar_sector_filter',
            name='lidar_sector_filter',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='parking_rviz',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('t870_control'), 'rviz', 'lidar.rviz'])],
        ),
    ])
