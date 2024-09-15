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
    )
    apt install -y "${ros_packages[@]}"
}

# Remove conflicting ROS packages
remove_conflicting_ros_packages() {
    local remove_packages=(
        "ros-humble-librealsense2"
        "ros-humble-realsense2-camera"
    )
    dpkg -r --force-depends "${remove_packages[@]}"
}

# Install Phoenix 5 and Phoenix 6
install_phoenix() {
    curl -s --compressed -o /usr/share/keyrings/ctr-pubkey.gpg "https://deb.ctr-electronics.com/ctr-pubkey.gpg"
    curl -s --compressed -o /etc/apt/sources.list.d/ctr2024.list "https://deb.ctr-electronics.com/ctr2024.list"
    apt update
    apt install phoenix6

    cd "$(dirname "$0")"
    mv phoenix /usr/lib/
}

# Main script
main() {
    install_ros_packages
    install_phoenix
    remove_conflicting_ros_packages
}

main
