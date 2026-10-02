from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gps_launch = PathJoinSubstitution([
        FindPackageShare('t870_control'), 'launch',
        't870_smc2000_gps.launch.py',
    ])
    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value='/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00'),
        DeclareLaunchArgument(
            'output_directory',
            default_value='/home/sxx/Desktop/colcon_ws./colcon_ws/colcon_ws/gps_recordings'),
        DeclareLaunchArgument('path_name', default_value=''),
        DeclareLaunchArgument('minimum_distance_m', default_value='1.0'),
        DeclareLaunchArgument('target_speed', default_value='1.0'),
        DeclareLaunchArgument('start_recording', default_value='false'),
        DeclareLaunchArgument('overwrite_existing', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gps_launch),
            launch_arguments={'device': LaunchConfiguration('device')}.items(),
        ),
        Node(
            package='t870_control',
            executable='gps_path_recorder',
            name='gps_path_recorder',
            output='screen',
            parameters=[{
                'fix_topic': '/gps/fix',
                'output_directory': LaunchConfiguration('output_directory'),
                'path_name': LaunchConfiguration('path_name'),
                'minimum_distance_m': ParameterValue(
                    LaunchConfiguration('minimum_distance_m'), value_type=float),
                'target_speed': ParameterValue(
                    LaunchConfiguration('target_speed'), value_type=float),
                'start_recording': ParameterValue(
                    LaunchConfiguration('start_recording'), value_type=bool),
                'overwrite_existing': ParameterValue(
                    LaunchConfiguration('overwrite_existing'), value_type=bool),
            }],
        ),
    ])
