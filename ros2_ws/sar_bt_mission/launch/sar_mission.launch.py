import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_sar   = get_package_share_directory('sar_bt_mission')
    pkg_storagy = get_package_share_directory('storagy')

    map_yaml = LaunchConfiguration(
        'map',
        default=os.path.join(pkg_storagy, 'map', 'map.yaml'))

    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_sar, 'param', 'nav2_params.yaml'))

    nav_to_pose_bt = os.path.join(
        pkg_sar, 'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery.xml')

    nav_through_poses_bt = os.path.join(
        pkg_sar, 'behavior_trees',
        'navigate_through_poses_w_replanning_and_recovery.xml')

    storagy_nav2_launch = os.path.join(
        pkg_storagy, 'launch', 'navigation2', 'bringup_launch.py')

    with open(os.path.join(pkg_storagy, 'urdf', 'storagy.urdf'), 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
            description='Use simulation clock if true'),
        DeclareLaunchArgument('map', default_value=map_yaml,
            description='Full path to map.yaml'),
        DeclareLaunchArgument('params_file', default_value=params_file,
            description='Full path to nav2 params yaml'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_z', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),

        # 1. robot_state_publisher (TF 브로드캐스팅)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': use_sim_time,
            }]),

        # 2. Nav2 (AMCL + Planner + Controller + BT Navigator)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(storagy_nav2_launch),
            launch_arguments={
                'map': map_yaml,
                'use_sim_time': use_sim_time,
                'use_respawn': 'false',
                'params_file': params_file,
                'default_nav_to_pose_bt_xml': nav_to_pose_bt,
                'default_nav_through_poses_bt_xml': nav_through_poses_bt,
                'initial_pose_x': LaunchConfiguration('initial_pose_x'),
                'initial_pose_y': LaunchConfiguration('initial_pose_y'),
                'initial_pose_z': LaunchConfiguration('initial_pose_z'),
                'initial_pose_yaw': LaunchConfiguration('initial_pose_yaw'),
            }.items()),

        # 3. 빨간색 검출 노드 (카메라 HSV → /red_detected, /red_target_point)
        Node(
            package='sar_bt_mission',
            executable='red_detector_node.py',
            name='red_detector',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),

        # 4. SAR 미션 BT 노드
        Node(
            package='sar_bt_mission',
            executable='sar_bt_node',
            name='sar_bt_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),

        # 5. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_storagy, 'rviz', 'storagy.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
