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
            {'publishWaypoints': True}
        ]
    )

    erp42_location_node = Node(
        package='ma_rrt_path_plan',
        executable='track',
        name='erp42_location',
        output='screen'
    )

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('ma_rrt_path_plan'),
        'rviz',
        'innovation.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        ma_rrt_path_plan_node,
        erp42_location_node,
        rviz_node
    ])
