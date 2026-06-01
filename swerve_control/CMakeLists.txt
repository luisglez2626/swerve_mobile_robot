# File: ~/ros2_ws/src/swerve_robot/swerve_control/CMakeLists.txt

cmake_minimum_required(VERSION 3.8)
project(swerve_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(swerve_kinematics src/swerve_kinematics.cpp)
ament_target_dependencies(swerve_kinematics 
  rclcpp 
  geometry_msgs 
  std_msgs 
  sensor_msgs 
  nav_msgs 
  tf2 
  tf2_geometry_msgs
)

install(TARGETS swerve_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()