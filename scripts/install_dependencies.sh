#!/bin/bash

# Install ROS 2 packages
install_ros_packages() {
    local ros_packages=(
        "ros-humble-rtabmap"
        "ros-humble-rtabmap-ros"
        "ros-humble-rplidar-ros"
        "ros-humble-apriltag-ros"
        "ros-humble-laser-filters"
        "ros-humble-robot-localization"
        "ros-humble-imu-complementary-filter"
        "ros-humble-gazebo-ros2-control"
        "ros-humble-ros2-control"
        "ros-humble-ros2-controllers"
    )
    apt install -y "${ros_packages[@]}"
}

# Install Phoenix 5 
install_phoenix() {
    cd "$(dirname "$0")"
    mv phoenix /usr/lib/
    echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/phoenix/' >> ~/.bashrc
    source ~/.bashrc
}

# Main script
main() {
    install_ros_packages
    install_phoenix
}

main
