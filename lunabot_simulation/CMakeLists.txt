cmake_minimum_required(VERSION 3.5)
project(lunabot_simulation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Compiler options for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Define executables
add_executable(blade_joint_controller src/blade_joint_controller.cpp)
add_executable(topic_remapper src/topic_remapper.cpp)

# Set target dependencies
ament_target_dependencies(blade_joint_controller rclcpp sensor_msgs std_msgs)
ament_target_dependencies(topic_remapper rclcpp geometry_msgs nav_msgs)

# Install targets
install(TARGETS
  blade_joint_controller
  topic_remapper
  DESTINATION lib/${PROJECT_NAME}
)

# Install URDF, meshes, worlds, and models
install(
  DIRECTORY urdf meshes worlds models
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(PROGRAMS
  src/teleop/keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()