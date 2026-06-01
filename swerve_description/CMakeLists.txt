# File: ~/ros2_ws/src/swerve_robot/swerve_description/CMakeLists.txt

cmake_minimum_required(VERSION 3.8)
project(swerve_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()