from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('control_enabled', default_value='false'),
        DeclareLaunchArgument('camera_device', default_value='auto'),
        DeclareLaunchArgument('target_speed_mps', default_value='0.30'),
        DeclareLaunchArgument('max_steer_rad', default_value='0.18'),
        DeclareLaunchArgument('kp_offset', default_value='0.30'),
        DeclareLaunchArgument('kp_heading', default_value='0.12'),
        DeclareLaunchArgument('minimum_confidence', default_value='0.40'),
        DeclareLaunchArgument('lane_hold_sec', default_value='0.80'),
        DeclareLaunchArgument('lane_smoothing', default_value='0.35'),
        DeclareLaunchArgument('lane_center_bias_ratio', default_value='0.0'),
        DeclareLaunchArgument('traffic_control_enabled', default_value='true'),
        DeclareLaunchArgument('signal_confirm_frames', default_value='4'),
        DeclareLaunchArgument('stop_line_confirm_frames', default_value='3'),
        DeclareLaunchArgument('save_diagnostics', default_value='false'),
        DeclareLaunchArgument('publish_camera_image', default_value='false'),
        DeclareLaunchArgument(
            'diagnostic_directory', default_value='/tmp/t870_lane_diagnostics'),
        Node(
            package='t870_control',
            executable='lane_camera_preview',
            name='lane_camera_preview',
            output='screen',
            parameters=[{
                'control_enabled': ParameterValue(
                    LaunchConfiguration('control_enabled'), value_type=bool),
                'camera_device': LaunchConfiguration('camera_device'),
                'publish_camera_image': ParameterValue(
                    LaunchConfiguration('publish_camera_image'),
                    value_type=bool),
                'target_speed_mps': ParameterValue(
                    LaunchConfiguration('target_speed_mps'), value_type=float),
                'max_steer_rad': ParameterValue(
                    LaunchConfiguration('max_steer_rad'), value_type=float),
                'kp_offset': ParameterValue(
                    LaunchConfiguration('kp_offset'), value_type=float),
                'kp_heading': ParameterValue(
                    LaunchConfiguration('kp_heading'), value_type=float),
                'minimum_confidence': ParameterValue(
                    LaunchConfiguration('minimum_confidence'), value_type=float),
                'lane_hold_sec': ParameterValue(
                    LaunchConfiguration('lane_hold_sec'), value_type=float),
                'lane_smoothing': ParameterValue(
                    LaunchConfiguration('lane_smoothing'), value_type=float),
                'lane_center_bias_ratio': ParameterValue(
                    LaunchConfiguration('lane_center_bias_ratio'),
                    value_type=float),
                'traffic_control_enabled': ParameterValue(
                    LaunchConfiguration('traffic_control_enabled'),
                    value_type=bool),
                'signal_confirm_frames': ParameterValue(
                    LaunchConfiguration('signal_confirm_frames'),
                    value_type=int),
                'stop_line_confirm_frames': ParameterValue(
                    LaunchConfiguration('stop_line_confirm_frames'),
                    value_type=int),
                'save_diagnostics': ParameterValue(
                    LaunchConfiguration('save_diagnostics'), value_type=bool),
                'diagnostic_directory': LaunchConfiguration(
                    'diagnostic_directory'),
            }],
        ),
    ])
