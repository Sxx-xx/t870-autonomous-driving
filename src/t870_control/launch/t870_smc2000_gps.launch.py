from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    device = LaunchConfiguration('device')
    restart = LaunchConfiguration('rtk_restart_enabled')
    after = LaunchConfiguration('rtk_restart_after_sec')

    def gps_node(respawn):
        # 노드를 두 벌 적어 두고 조건으로 하나만 띄운다. respawn 은 런치
        # 인자로 바로 못 받는 자리라 이 방법이 확실하다.
        return Node(
            package='t870_control',
            executable='smc2000_gps_node',
            name='smc2000_gps_node',
            output='screen',
            respawn=respawn,
            respawn_delay=1.0,
            condition=IfCondition(restart) if respawn else UnlessCondition(restart),
            parameters=[{
                'device': device,
                'baudrate': 115200,
                'frame_id': 'gps_link',
                'rtk_restart_enabled': ParameterValue(restart, value_type=bool),
                'rtk_restart_after_sec': ParameterValue(after, value_type=float),
            }],
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value='/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00',
            description='Stable USB serial path for the SMC-2000 u-blox receiver'),
        DeclareLaunchArgument(
            'rtk_restart_enabled', default_value='false',
            description='RTK 가 끊기면 GPS 노드를 죽였다 바로 다시 띄운다'),
        DeclareLaunchArgument(
            'rtk_restart_after_sec', default_value='5.0',
            description='이 시간 동안 RTK 가 없으면 재시작'),
        gps_node(True),
        gps_node(False),
    ])
