#!/bin/bash

install_dependencies() {
    cd ../../..

    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init
    fi

    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
}

install_sparkcan() {
    sudo add-apt-repository ppa:graysonarendt/sparkcan
    sudo apt update
    sudo apt install sparkcan -y
}

install_sensors() {
    sudo apt install ros-humble-realsense2-*
    sudo apt install ros-humble-rplidar-ros
}

main() {
    install_dependencies
    install_sparkcan
    install_sensors
}

main