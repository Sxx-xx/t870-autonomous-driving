import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('t870_control'), 'config', 'c920e.yaml'
    )
    video_device = LaunchConfiguration('video_device')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    framerate = LaunchConfiguration('framerate')
    pixel_format = LaunchConfiguration('pixel_format')

    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video2',
            description='C920e V4L2 device; /dev/v4l/by-id path is preferred',
        ),
        DeclareLaunchArgument('image_width', default_value='640'),
        DeclareLaunchArgument('image_height', default_value='480'),
        DeclareLaunchArgument('framerate', default_value='30.0'),
        DeclareLaunchArgument(
            'pixel_format',
            default_value='yuyv2rgb',
            description='Change only after checking the formats advertised by the camera',
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            namespace='camera',
            name='c920e',
            output='screen',
            parameters=[
                config_file,
                {
                    'video_device': video_device,
                    'image_width': ParameterValue(image_width, value_type=int),
                    'image_height': ParameterValue(image_height, value_type=int),
                    'framerate': ParameterValue(framerate, value_type=float),
                    'pixel_format': pixel_format,
                },
            ],
        ),
    ])
