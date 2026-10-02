import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Nodes
    ma_rrt_path_plan_node = Node(
        package='ma_rrt_path_plan',
        executable='main',
        name='ma_rrt_path_plan_node',
        output='screen',
        parameters=[
            {'desiredWaypointsFrequency': 5},
            {'publishWaypoints': True},
            {'publishPredefined': True},
            {'path': PathJoinSubstitution([
                FindPackageShare('ma_rrt_path_plan'),
                'waypoints',
                '' # Empty string to point to the directory
            ])},
            {'filename': 'fsg18_waypoints.csv'}
        ]
    )

    return LaunchDescription([
        ma_rrt_path_plan_node
    ])
