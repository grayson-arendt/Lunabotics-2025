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
find_package(OpenCV 4.5 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sparkcan REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Localization.action"
)

# Define executables
add_executable(localization_server src/control/localization_server.cpp)
add_executable(navigation_client src/control/navigation_client.cpp)
add_executable(robot_controller src/control/robot_controller.cpp)
add_executable(hardware_monitor src/utils/hardware_monitor.cpp)
add_executable(imu_rotator src/utils/imu_rotator.cpp)

# Set target dependencies
ament_target_dependencies(localization_server rclcpp rclcpp_action nav2_msgs tf2_ros geometry_msgs sensor_msgs cv_bridge)
ament_target_dependencies(navigation_client rclcpp rclcpp_action nav2_msgs tf2_ros geometry_msgs)
ament_target_dependencies(robot_controller rclcpp sensor_msgs geometry_msgs sparkcan)
ament_target_dependencies(hardware_monitor rclcpp sensor_msgs)
ament_target_dependencies(imu_rotator rclcpp sensor_msgs tf2_ros)

# Link OpenCV libraries to localization_server
target_link_libraries(localization_server ${OpenCV_LIBS})

# Add dependencies to generated actions 
rosidl_target_interfaces(navigation_client ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(localization_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_include_directories(robot_controller PRIVATE include)

set(PHOENIX_LIBS_DIR /usr/lib/phoenix)

target_link_libraries(robot_controller ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)
target_link_libraries(robot_controller ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so)
target_link_libraries(robot_controller ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixTools.so)


# Install targets
install(TARGETS
  localization_server
  navigation_client
  robot_controller
  hardware_monitor
  imu_rotator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
