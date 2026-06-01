# File: ~/ros2_ws/src/swerve_robot/swerve_bringup/launch/sim_bringup.launch.py

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_bringup = FindPackageShare('swerve_bringup')
    pkg_description = FindPackageShare('swerve_description')
    pkg_gazebo = FindPackageShare('swerve_gazebo')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    # Launch Configurations
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Paths
    world_file = PathJoinSubstitution([pkg_gazebo, 'worlds', 'maze.sdf'])
    urdf_file = PathJoinSubstitution([pkg_description, 'urdf', 'swerve_robot.urdf.xacro'])
    bridge_config = PathJoinSubstitution([pkg_bringup, 'config', 'bridge.yaml'])
    rviz_config_file = PathJoinSubstitution([pkg_bringup, 'config', 'view_robot.rviz'])

    # Declare Launch Arguments
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RViz'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', urdf_file]), value_type=str),
            'use_sim_time': True
        }]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': ['-r ', world_file]}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'swerve_robot', '-x', '0.0', '-y', '0.0', '-z', '0.3'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': True
        }],
        output='screen'
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': True}]
    )
    
    steering_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['steering_controller'],
        parameters=[{'use_sim_time': True}]
    )
    
    velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['velocity_controller'],
        parameters=[{'use_sim_time': True}]
    )

    kinematics = Node(
        package='swerve_control',
        executable='swerve_kinematics',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        declare_use_rviz_cmd,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        joint_state_broadcaster,
        steering_controller,
        velocity_controller,
        kinematics,
        rviz_node
    ])