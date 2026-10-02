import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch Arguments
    pcap_file_arg = DeclareLaunchArgument(
        'pcap_file', default_value='', description='Path to pcap file for Hesai Lidar'
    )
    server_ip_arg = DeclareLaunchArgument(
        'server_ip', default_value='192.168.1.201', description='Hesai Lidar server IP'
    )
    lidar_recv_port_arg = DeclareLaunchArgument(
        'lidar_recv_port', default_value='2368', description='Hesai Lidar receive port'
    )
    gps_port_arg = DeclareLaunchArgument(
        'gps_port', default_value='10110', description='Hesai Lidar GPS port'
    )
    start_angle_arg = DeclareLaunchArgument(
        'start_angle', default_value='0.0', description='Hesai Lidar start angle'
    )
    lidar_type_arg = DeclareLaunchArgument(
        'lidar_type', default_value='PandarXT-16', description='Hesai Lidar type'
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id', default_value='PandarXT-16', description='Hesai Lidar frame ID'
    )
    pcldata_type_arg = DeclareLaunchArgument(
        'pcldata_type', default_value='0', description='Hesai Lidar PCL data type'
    )
    publish_type_arg = DeclareLaunchArgument(
        'publish_type', default_value='points', description='Hesai Lidar publish type'
    )
    timestamp_type_arg = DeclareLaunchArgument(
        'timestamp_type', default_value='', description='Hesai Lidar timestamp type'
    )
    data_type_arg = DeclareLaunchArgument(
        'data_type', default_value='', description='Hesai Lidar data type'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='hesai', description='Hesai Lidar namespace'
    )
    lidar_correction_file_arg = DeclareLaunchArgument(
        'lidar_correction_file', 
        default_value=PathJoinSubstitution([
            FindPackageShare('hesai_lidar'),
            'config',
            'Pandar16.csv' # Original was Pandar16.csv, but config has Pandar40.csv etc.
        ]),
        description='Hesai Lidar correction file'
    )
    multicast_ip_arg = DeclareLaunchArgument(
        'multicast_ip', default_value='', description='Hesai Lidar multicast IP'
    )
    coordinate_correction_flag_arg = DeclareLaunchArgument(
        'coordinate_correction_flag', default_value='false', description='Hesai Lidar coordinate correction flag'
    )
    fixed_frame_arg = DeclareLaunchArgument(
        'fixed_frame', default_value='', description='Hesai Lidar fixed frame'
    )
    target_frame_arg = DeclareLaunchArgument(
        'target_frame', default_value='', description='Hesai Lidar target frame'
    )

    # Nodes
    adaptive_clustering_node = Node(
        package='adaptive_clustering',
        executable='adaptive_clustering_oa',
        name='adaptive_clustering_oa',
        output='screen',
        parameters=[
            {'print_fps': True}
        ]
    )

    ma_rrt_path_plan_node = Node(
        package='ma_rrt_path_plan',
        executable='main_oa', # Assuming main_oa.py is the executable
        name='ma_rrt_path_plan_node',
        output='screen',
        parameters=[
            {'desiredWaypointsFrequency': 5},
            {'odom_topic': '/odometry'},
            {'world_frame': 'PandarXT-16'},
            {'publishWaypoints': True}
        ]
    )

    erp42_location_node = Node(
        package='ma_rrt_path_plan',
        executable='track_oa', # Assuming track_oa.py is the executable
        name='erp42_location',
        output='screen'
    )

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('ma_rrt_path_plan'),
        'rviz',
        'innovation.rviz'
    ])

    rviz_node = Node(
        package='rviz2', # ROS2 RViz package name
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    hesai_lidar_node = Node(
        package='hesai_lidar',
        executable='hesai_lidar_node',
        name='hesai_lidar',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'pcap_file': LaunchConfiguration('pcap_file'),
            'server_ip': LaunchConfiguration('server_ip'),
            'lidar_recv_port': LaunchConfiguration('lidar_recv_port'),
            'gps_port': LaunchConfiguration('gps_port'),
            'start_angle': LaunchConfiguration('start_angle'),
            'lidar_type': LaunchConfiguration('lidar_type'),
            'frame_id': LaunchConfiguration('frame_id'),
            'pcldata_type': LaunchConfiguration('pcldata_type'),
            'publish_type': LaunchConfiguration('publish_type'),
            'timestamp_type': LaunchConfiguration('timestamp_type'),
            'data_type': LaunchConfiguration('data_type'),
            'lidar_correction_file': LaunchConfiguration('lidar_correction_file'),
            'multicast_ip': LaunchConfiguration('multicast_ip'),
            'coordinate_correction_flag': LaunchConfiguration('coordinate_correction_flag'),
            'fixed_frame': LaunchConfiguration('fixed_frame'),
            'target_frame': LaunchConfiguration('target_frame'),
        }]
    )

    return LaunchDescription([
        pcap_file_arg,
        server_ip_arg,
        lidar_recv_port_arg,
        gps_port_arg,
        start_angle_arg,
        lidar_type_arg,
        frame_id_arg,
        pcldata_type_arg,
        publish_type_arg,
        timestamp_type_arg,
        data_type_arg,
        namespace_arg,
        lidar_correction_file_arg,
        multicast_ip_arg,
        coordinate_correction_flag_arg,
        fixed_frame_arg,
        target_frame_arg,
        
        adaptive_clustering_node,
        ma_rrt_path_plan_node,
        erp42_location_node,
        rviz_node,
        hesai_lidar_node
    ])
