cmake_minimum_required(VERSION 3.5)
project(lunabot_autonomous)

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

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake_python REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Control.msg"
)

# Control
add_executable(robot_controller src/physical_robot/control/robot_controller.cpp)

# Scripts
add_executable(digging_script src/physical_robot/behaviors/digging_script.cpp)

# Simulation
add_executable(blade_joint_controller src/simulated_robot/blade_joint_controller.cpp)
add_executable(topic_remap src/simulated_robot/topic_remap.cpp)

# System
add_executable(hardware_monitor src/physical_robot/system/hardware_monitor.cpp)
add_executable(imu_rotator src/physical_robot/system/imu_rotator.cpp)
add_executable(navigator_client src/physical_robot/system/navigator_client.cpp)

ament_target_dependencies(robot_controller rclcpp sensor_msgs geometry_msgs)

ament_target_dependencies(blade_joint_controller rclcpp sensor_msgs std_msgs)
ament_target_dependencies(topic_remap rclcpp geometry_msgs nav_msgs)

ament_target_dependencies(hardware_monitor rclcpp sensor_msgs)
ament_target_dependencies(imu_rotator rclcpp sensor_msgs)
ament_target_dependencies(navigator_client rclcpp rclcpp_action nav2_msgs geometry_msgs)

target_include_directories(robot_controller PRIVATE include)
target_include_directories(digging_script PRIVATE include)

rosidl_target_interfaces(robot_controller ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(navigator_client ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  robot_controller
  digging_script
  blade_joint_controller
  topic_remap
  hardware_monitor
  imu_rotator
  navigator_client
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
   src/simulated_robot/teleop/keyboard_teleop.py
   DESTINATION lib/${PROJECT_NAME}
)

set(PHOENIX_LIBS_DIR /usr/lib/phoenix)

target_link_libraries(robot_controller ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)
target_link_libraries(robot_controller ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so)
target_link_libraries(robot_controller ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixTools.so)

target_link_libraries(digging_script ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)
target_link_libraries(digging_script ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so)
target_link_libraries(digging_script ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixTools.so)

ament_package()
