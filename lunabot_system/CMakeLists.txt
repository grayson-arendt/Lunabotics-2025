cmake_minimum_required(VERSION 3.5)
project(lunabot_system)

# Compiler settings
add_definitions(-w)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Compile options for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sparkcan REQUIRED)

# Define executables
add_executable(navigation_client src/control/navigation_client.cpp)
add_executable(robot_controller src/control/robot_controller.cpp)
add_executable(hardware_monitor src/utils/hardware_monitor.cpp)
add_executable(imu_rotator src/utils/imu_rotator.cpp)
add_executable(odom_tf_publisher src/utils/odom_tf_publisher.cpp)

# Set target dependencies
ament_target_dependencies(navigation_client rclcpp rclcpp_action nav2_msgs geometry_msgs ${sparkcan_LIBRARIES})
ament_target_dependencies(robot_controller rclcpp sensor_msgs geometry_msgs ${sparkcan_LIBRARIES})
ament_target_dependencies(hardware_monitor rclcpp sensor_msgs)
ament_target_dependencies(imu_rotator rclcpp sensor_msgs)
ament_target_dependencies(odom_tf_publisher rclcpp nav_msgs tf2_ros geometry_msgs)

# Install targets
install(TARGETS
  navigation_client
  robot_controller
  hardware_monitor
  imu_rotator
  odom_tf_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
