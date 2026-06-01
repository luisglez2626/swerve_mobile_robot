Swerve Mobile Robot

A complete ROS 2 Humble software stack for a 4-wheel swerve drive mobile robot. This project features custom kinematics, autonomous frontier exploration, and a PyQt6 operator dashboard. The simulation runs in Gazebo Fortress.

Core Features

Swerve Kinematics and Control
Custom C++ kinematics node solving forward and inverse kinematics for 4 independently steered and driven wheels. Features include shortest path angle optimization and cosine scaled velocity limits to prevent arcing during rapid directional changes.

Autonomous RRT Exploration
A standalone C++ node that completely automates the mapping process. It reads the live SLAM occupancy grid, computes Rapidly exploring Random Trees to identify frontiers, and commands the Nav2 Action Server. It features strict obstacle inflation radius checks and an asynchronous watchdog to blacklist unreachable local minima.

Tight Space Navigation
Highly tuned Navigation 2 stack specifically configured to allow holonomic movement with exactly 20cm clearance margins. Uses the DWB Local Planner with scaled acceleration limits to eliminate dynamic odometry drift.

Live Operator Dashboard
A multithreaded PyQt6 graphical interface integrating ROS 2 Action Clients and TF2 listeners. Allows for manual holonomic teleoperation via a virtual joystick, predefined waypoint deployments, and live diagnostics of global coordinates and individual wheel speeds.

Dynamic SDF World Generation
Python tooling to convert JSON architectural layouts directly into scaled XML Gazebo SDF environments.

System Requirements

Ubuntu 22.04 LTS

ROS 2 Humble

Gazebo Fortress

Python Dependencies: PyQt6

Usage Modes

The system operates in three distinct modes depending on the mission requirement. Ensure hardware rendering is enabled in your primary terminal before launching the simulation.

Export NVIDIA variables
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

Mode 1: Autonomous Exploration

ros2 launch swerve_bringup sim_bringup.launch.py

ros2 launch swerve_navigation slam.launch.py

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:=install/swerve_navigation/share/swerve_navigation/config/nav2_params.yaml

ros2 run swerve_gui gui

ros2 run swerve_exploration rrt_node

Mode 2: Manual Mapping

ros2 launch swerve_bringup sim_bringup.launch.py

ros2 launch swerve_navigation slam.launch.py

ros2 run swerve_gui gui

Mode 3: Pure Navigation

ros2 launch swerve_bringup sim_bringup.launch.py

ros2 launch swerve_navigation navigation.launch.py

ros2 run swerve_gui gui