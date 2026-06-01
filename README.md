# 🤖 Swerve Agrobot: Autonomous Navigation & Exploration Stack

A comprehensive ROS 2 Humble stack for an 8-motor independent swerve drive robot. Engineered for high-precision holonomic navigation in constrained environments, featuring C++ kinematics, RRT-based exploration, and a PyQt6 operator dashboard.

## 📦 Package Ecosystem (7 Packages)

*   **`swerve_bringup`**: Orchestrates system-wide launches, hardware bridges, and unified parameter loading.
*   **`swerve_control`**: C++ Kinematics core. Implements IK/FK with shortest-path optimization and cosine velocity scaling.
*   **`swerve_description`**: URDF/Xacro definitions. Features a 0.42m cylindrical chassis to maximize Lidar visibility.
*   **`swerve_exploration`**: Frontier detection via a custom C++ RRT implementation for fully autonomous mapping.
*   **`swerve_gazebo`**: Manages the Ignition Fortress environment and dynamic world generation tools.
*   **`swerve_gui`**: PyQt6 Operator Dashboard featuring a **Virtual Joystick** for real-time holonomic control.
*   **`swerve_navigation`**: Optimized Nav2 stack, SLAM Toolbox, and EKF-based odometry fusion.

## 🚀 Technical Highlights

*   **Holonomic Precision**: Independent 4-module control for crab, snake, and zero-turn maneuvers.
*   **20cm Wall Constraint**: Specifically tuned Nav2 costmaps (0.62m inflation) for close-proximity interactions.
*   **Autonomous Frontier Mapping**: RRT exploration with a watchdog system to prevent local minima traps.
*   **Precision EKF Fusion**: Eliminates kinematic drift by fusing wheel odometry with high-frequency IMU data.
*   **Operator UI**: Multi-threaded dashboard with live diagnostics and manual override capability.

## 🏗️ World Generation from JSON

Located in `swerve_gazebo/worlds/`, the `json_to_sdf.py` tool converts architectural JSON coordinates into native, high-performance SDF worlds for Gazebo Fortress.

**Supported Parameters:**
*   `--input`: Path to the input JSON file.
*   `--output`: Name of the generated `.sdf` file.
*   `--hall_width`: Distance between wall centers (m).
*   `--wall_height`: Physical height of wall assets (m).
*   `--wall_thickness`: Collision and visual thickness (m).

```bash
python3 json_to_sdf.py --input maze.json --output maze.sdf --hall_width 3.0 --wall_height 1.5
```

## 🛠️ System Requirements

*   **OS**: Ubuntu 22.04 LTS
*   **ROS Version**: ROS 2 Humble
*   **Simulator**: Gazebo Fortress (v6.16+)
*   **GPU**: NVIDIA Dedicated GPU (Highly recommended for RViz/Gazebo stability)

## 🎮 Usage Instructions

### 0. Graphics Initialization (CRITICAL)
To prevent UI artifacts and Gazebo crashes on hybrid-graphics laptops, always export these variables:
```bash
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
```

### Mode 1: Autonomous Exploration (Mapping)
Automatically map an unknown environment using RRT frontiers.
1. **Simulation**: `ros2 launch swerve_bringup sim_bringup.launch.py`
2. **SLAM**: `ros2 launch swerve_navigation slam.launch.py`
3. **NavStack**: `ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:=install/swerve_navigation/share/swerve_navigation/config/nav2_params.yaml`
4. **Exploration**: `ros2 run swerve_exploration rrt_node`
5. **Dashboard**: `ros2 run swerve_gui gui`

**Saving the map:** Wait for the RRT node to print `Exploration physically complete`. If it leaves a tight corner unmapped, use the GUI Joystick to drive the robot into the corner. Once satisfied, open a new terminal and save the map to a custom directory to avoid overwriting your default map:
```bash
mkdir -p ~/ros2_ws/src/swerve_robot/swerve_navigation/map_RRT_generated
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/swerve_robot/swerve_navigation/map_RRT_generated/maze_map

```


### Mode 2: Manual Teleoperation (Mapping)
Map a new environment entirely by hand without autonomous intervention.
1. **Simulation**: `ros2 launch swerve_bringup sim_bringup.launch.py`
2. **SLAM**: `ros2 launch swerve_navigation slam.launch.py`
3. **Dashboard**: `ros2 run swerve_gui gui`

**PROCEDURE:**
Use the Virtual Joystick in the Operator Dashboard to drive the robot through the maze until the map is fully generated in RViz. Click the Save SLAM Map button in the GUI to automatically save the map to
`swerve_navigation/maps/maze_map`.

### Mode 3: Pure Navigation (Pre-built Map)
Run navigation missions on a map you have already saved. SLAM is disabled.
1. **Simulation**: `ros2 launch swerve_bringup sim_bringup.launch.py`
2. **Navigation**: `ros2 launch swerve_navigation navigation.launch.py`
3. **Dashboard**: `ros2 run swerve_gui gui`

**PROCEDURE:**
First, use the 2D Pose Estimate tool in RViz to tell AMCL where the robot is starting. Once localized, use the GUI's Deploy to Waypoint dropdown, click SEND HOME, or use the 2D Goal Pose tool in RViz to command the robot.



## 📈 Implementation Details

### Swerve Optimization
The `swerve_control` node implements:
*   **Shortest Path Logic**: Prevents modules from rotating more than 90° by inverting wheel velocity when necessary.
*   **Anti-Arcing**: Uses cosine scaling of velocity commands during steering transitions to ensure the robot maintains its trajectory heading.
*   **Hysteresis**: Prevents "jitter" when the robot is nearly stationary by maintaining the last known steering angle for commands under 0.01 m/s.

### Navigation Tuning
The `nav2_params.yaml` is tuned for holonomic swerve dynamics:
*   **Planner**: `NavfnPlanner` with A* disabled for faster global path recalculation.
*   **Controller**: `DWBLocalPlanner` with specialized critics (`PathAlign`, `GoalAlign`, `BaseObstacle`) to handle the circular chassis profile.