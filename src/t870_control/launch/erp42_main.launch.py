from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='erp42_control_ob',
            executable='erp42_status',
            name='erp_status',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB0'},
                {'baudrate': 115200}
            ]
        ),
        Node(
            package='erp42_control_ob',
            executable='erp42_planner_main',
            name='erp_planner',
            output='screen'
        ),
    ])
