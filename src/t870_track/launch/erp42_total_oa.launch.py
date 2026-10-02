import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='erp42_track',
            executable='erp42_tracker_oa',
            name='erp42_track_oa',
            output='screen'
        ),
        Node(
            package='erp42_track',
            executable='erp42_status',
            name='erp42_status',
            output='screen'
        ),
    ])
