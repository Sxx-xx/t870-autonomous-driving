"""GPS + Pixhawk IMU + Arduino wheel encoder dual-EKF localization."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    gps_x = LaunchConfiguration('gps_x')
    gps_y = LaunchConfiguration('gps_y')
    gps_z = LaunchConfiguration('gps_z')
    imu_x = LaunchConfiguration('imu_x')
    imu_y = LaunchConfiguration('imu_y')
    imu_z = LaunchConfiguration('imu_z')
    config_dir = PathJoinSubstitution([
        FindPackageShare('t870_control'), 'config'])
    return LaunchDescription([
        # Measured from rear axle center (base_link) to antenna center.
        DeclareLaunchArgument('gps_x', default_value='0.693'),
        DeclareLaunchArgument('gps_y', default_value='0.0'),
        DeclareLaunchArgument('gps_z', default_value='0.412'),
        DeclareLaunchArgument('imu_x', default_value='0.0'),
        DeclareLaunchArgument('imu_y', default_value='0.0'),
        DeclareLaunchArgument('imu_z', default_value='0.0'),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_to_gps',
            arguments=['--x', gps_x, '--y', gps_y, '--z', gps_z,
                       '--yaw', '0', '--pitch', '0', '--roll', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'gps_link']),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_to_imu',
            arguments=['--x', imu_x, '--y', imu_y, '--z', imu_z,
                       '--yaw', '0', '--pitch', '0', '--roll', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'imu_link']),
        Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_local', output='screen',
            parameters=[PathJoinSubstitution([config_dir, 'ekf_local.yaml'])],
            remappings=[('odometry/filtered', '/odometry/local')]),
        Node(
            package='robot_localization', executable='navsat_transform_node',
            name='navsat_transform', output='screen',
            parameters=[PathJoinSubstitution([config_dir, 'navsat_transform.yaml'])],
            remappings=[('imu', '/mavros/imu/data'),
                        ('gps/fix', '/gps/fix'),
                        ('odometry/filtered', '/odometry/local'),
                        ('odometry/gps', '/odometry/gps')]),
        Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_global', output='screen',
            parameters=[PathJoinSubstitution([config_dir, 'ekf_global.yaml'])],
            remappings=[('odometry/filtered', '/odometry/global')]),
    ])
