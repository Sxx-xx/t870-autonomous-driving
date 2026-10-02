"""T870 competition mission stack. All waypoint zones are disabled by default."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    nodes=[]
    ranges={name:LaunchConfiguration(name+'_ranges') for name in
            ('avoid','estop','parallel','tpark','traffic_light','lane_signal')}
    common=[DeclareLaunchArgument(name+'_ranges',default_value='') for name in ranges]
    common += [DeclareLaunchArgument('parking_armed',default_value='false'),
               DeclareLaunchArgument('enable_mission_webcam',default_value='false'),
               DeclareLaunchArgument('lane_signal_path_file',default_value='')]
    nodes.append(Node(package='t870_control',executable='mission_zone_manager_node',
        parameters=[{name+'_ranges':value for name,value in ranges.items()}],output='screen'))
    for executable in ('mission_mux_node','obstacle_avoidance_node','emergency_stop_node','traffic_light_node'):
        nodes.append(Node(package='t870_control',executable=executable,output='screen'))
    nodes += [Node(package='t870_control',executable='parallel_park_node',output='screen',
                   parameters=[{'armed':LaunchConfiguration('parking_armed')}]),
              Node(package='t870_control',executable='t_park_node',output='screen',
                   parameters=[{'armed':LaunchConfiguration('parking_armed')}]),
              Node(package='t870_control',executable='lane_signal_node',output='screen',
                   parameters=[{'lane_path_file':LaunchConfiguration('lane_signal_path_file')}]),
              Node(package='t870_control',executable='webcam_pub_node',output='screen',
                   condition=IfCondition(LaunchConfiguration('enable_mission_webcam')))]
    return LaunchDescription(common+nodes)
