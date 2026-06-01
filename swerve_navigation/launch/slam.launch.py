# File: ~/ros2_ws/src/swerve_robot/swerve_navigation/launch/slam.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_nav = FindPackageShare('swerve_navigation')
    
    slam_config = PathJoinSubstitution([pkg_nav, 'config', 'slam_toolbox.yaml'])
    ekf_config = PathJoinSubstitution([pkg_nav, 'config', 'ekf.yaml'])

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': True}
        ]
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

    return LaunchDescription([
        slam_node,
        ekf_node
    ])