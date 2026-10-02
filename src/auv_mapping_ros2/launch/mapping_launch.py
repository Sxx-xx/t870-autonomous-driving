import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # 1. 패키지 경로 설정
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_auv_mapping_ros2 = get_package_share_directory('auv_mapping_ros2')

    # 2. 가제보 리소스 경로 설정 (내 패키지의 모델들을 인식시키기 위함)
    # 기존 /opt/ros 경로 뒤에 성재 님의 패키지 상위 경로를 추가합니다.
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(pkg_auv_mapping_ros2, '..'), ':/opt/ros/jazzy/share/gz-sim-8']
    )

    # 3. 가제보 실행 (World 파일 로드)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        # -r: 자동 시작, -v 4: 상세 로그 출력
        launch_arguments={
            'gz_args': f'-r -v 4 {os.path.join(pkg_auv_mapping_ros2, "worlds", "underwater.world")}'
        }.items()
    )

    # 4. URDF/Xacro 파일 처리
    urdf_file = os.path.join(pkg_auv_mapping_ros2, 'urdf', 'auv.xacro')
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 5. Robot State Publisher (TF 및 모델 정보 발행)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 6. 가제보에 로봇 스폰(Spawn) 시키기
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'auv',
            '-z', '-2.0', # 수심 2m 지점에서 시작
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

    # 7. ROS-GZ Bridge (Jazzy/Harmonic 규격에 맞춤)
    # [ 는 GZ -> ROS (센서 전용), @ 는 양방향 의미
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/sonar_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/auv/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' # 오도메트리 추가
        ],
        remappings=[
            ('/sonar_scan', '/sonar_scan_ros'),
            ('/model/auv/odometry', '/odom')
        ],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        bridge
    ])