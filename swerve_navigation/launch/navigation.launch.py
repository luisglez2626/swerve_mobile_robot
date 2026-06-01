# File: ~/ros2_ws/src/swerve_robot/swerve_navigation/launch/navigation.launch.py

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    nav2_bringup_pkg = FindPackageShare('nav2_bringup')
    swerve_nav_pkg = FindPackageShare('swerve_navigation')

    map_file = PathJoinSubstitution([swerve_nav_pkg, 'maps', 'maze_map.yaml'])
    params_file = PathJoinSubstitution([swerve_nav_pkg, 'config', 'nav2_params.yaml'])
    ekf_config = PathJoinSubstitution([swerve_nav_pkg, 'config', 'ekf.yaml'])

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_pkg, 'launch', 'bringup_launch.py'])
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': 'true'
        }.items()
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([nav2_launch, ekf_node])