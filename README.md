## Overview

This repository contains code made by the College of DuPage team for the 2024-2025 NASA Lunabotics competition. It is made for ROS 2 Humble on Ubuntu 22.04.

## Hardware

- `Intel NUC 13 Pro`
- `RPLidar S2L`
- `RPLidar A3`
- `Intel RealSense D455 Depth Camera`
- `Intel RealSense T265 Tracking Camera`
- `CTRE Falcon 500 (x4)`
- `CTRE Talon SRX (x4)`
- `Turnigy 14.8V 8000mAh LiPo Battery`
- `Turnigy 14.8V 12000mAh LiPo Battery`
- `AndyMark Power Distribution Panel`
- `MKS CANable Pro`

## Dependencies

- `rtabmap`
- `rtabmap_ros`
- `rplidar_ros`
- `apriltag_ros`
- `navigation2`
- `robot_localization`
- `laser_filters`
- `imu_complementary_filter`

## Installation

#### 1. Make a workspace and clone the repository.

```bash
mkdir -p lunabot_ws/src
cd lunabot_ws/src
git clone https://github.com/grayson-arendt/Lunabotics-2025.git
```

#### 2. Run the install_dependencies script to install the required dependencies and build.

```bash
cd lunabot_ws/src/Lunabotics-2025/
sudo ./scripts/install_dependencies.sh
sudo ./scripts/setup_udev_rules.sh 
cd ../..
colcon build
```
#### 3. Repeat the last two steps with the [external-dev](https://github.com/grayson-arendt/Lunabotics-2025/tree/external-dev?tab=readme-ov-file) branch on the host computer (not robot computer). This branch is for visualizing the robot in RViz2.

## Setup Permissions

The rplidar_ros package needs to access /dev/ttyUSB0 and /dev/ttyUSB1 (using both lidars). While you can run `sudo chmod +x 777 /dev/ttyUSB0` for example, it would need to be ran each time on startup. To fix this, run the command below and restart the computer.

```bash
sudo usermod -a -G dialout $USER
```

If the lidars are not under /dev/ttyUSB0 and /dev/ttyUSB1 (which may happen when disconnected and reconnected), use this command with one lidar connected at a time to check the number at the end. Adjust the parameter in hardware_launch.py based off the number. 

```bash
ls /dev/ttyUSB*
```

## Running Real Robot

Each launch file should be ran in a new terminal window. Every step except for step 3 must be ran on the robot computer through an SSH connection, while step 3 will be ran locally on the host laptop. 

#### 1. Initialize SocketCAN communication 
```bash
cd ~/lunabot_ws/src/Lunabotics-2025/scripts/
chmod +x canable_start.sh
./canable_start.sh
```

#### 2. Navigate to ROS 2 workspace and install:
```bash
cd ~/lunabot_ws
source install/setup.bash
```

#### 3. Visualize with RViz2 (host computer):
```bash
ros2 launch lunabot_bringup external_launch.py
```

#### 4. Startup hardware:

```bash
ros2 launch lunabot_bringup hardware_launch.py 
```

#### 5. Startup RTAB-Map:

```bash
ros2 launch lunabot_bringup mapping_launch.py
```

#### 6. Startup Navigation2:

```bash
ros2 launch lunabot_bringup navigation_launch.py
```

In RViz2 on the host computer, you will now be able to select a "Nav2 Goal" in the GUI and have the robot navigate to that location. 

#### (Optional) 7. Startup action client:

```bash
ros2 run lunabot_autonomous navigator_client
```

The action client will send two goals, one for excavation zone and another for construction zone. After the goal has been reached, it will publish to /control topic and enable the specific
motors for the mechanisms for the zone.

<p align="center">
  <img src="sample.png">
</p>


# Running Simulated Robot

The simulation in Gazebo may be slow and laggy depending on the specifications of your computer. You can control the robot with an XBox One controller using the control_mode:=xbox argument. The left joystick will move the drivetrain, while the right joystick y-axis will rotate the bull dozer blade.

#### 1. Navigate to ROS 2 workspace and install:
```bash
cd ~/lunabot_ws
source install/setup.bash
```

#### 2. Start Gazebo and RViz2:
```bash
ros2 launch lunabot_bringup simulation_launch.py control_mode:=keyboard #control_mode:=xbox
```

## Structure

**lunabot_autonomous**
  - **include**
    - **ctre** (CTRE Phoenix C++ API for using Falcon 500 motors)
  - **src**
    - **physical_robot**
      - **behaviors**
        - digging_script.cpp (Autonomous script to handle digging behaviors)
      - **control**
        - motor_test.cpp (Simple node for testing Phoenix 5 motors on the physical robot)
        - robot_controller.cpp (Generates percent output/velocity commands for the robot's physical motors)
      - **system**
        - hardware_monitor.cpp (Monitors liveliness of hardware topics like sensors and motors)
        - imu_rotator.cpp (Rotates IMU values into the ENU coordinate frame)
        - navigator_client.cpp (Action client that sends goals and motor control commands)
    - **simulated_robot**
      - **teleop**
        - keyboard_teleop.py (Allows control of the robot using the keyboard in the simulation)
      - blade_joint_controller.cpp (Controls the blade joint movement in the Gazebo simulation)
      - topic_remap.cpp (Remaps ros2 control controller topics)

**lunabot_bringup**
  - **behavior_trees**
    - navigate_to_pose_w_replanning_goal_patience_and_recovery.xml (Behavior tree for Navigation2)
  - **config**
    - default_view.rviz (RViz2 configuration file)
  - **launch**
    - external_launch.py (Launches RViz2 and robot state/joint publisher nodes)
    - hardware_launch.py (Launches physical robot hardware like lidars, cameras, and controllers)
    - mapping_launch.py (Launches RTAB-Map for SLAM mapping)
    - navigation_launch.py (Launches Navigation2 for autonomous navigation)
  - **params**
    - a3_lidar_params.yaml (Parameters for laser_filters for A3 lidar)
    - ekf_params.yaml (Parameters for the robot_localization ekf_node)
    - nav2_params.yaml (Parameters for Navigation2)
    - s2l_lidar_params.yaml (Parameters for laser_filters for S2L lidar)
    - sim_params.yaml (Parameters for Gazebo controllers for simulation)
    - tag_params.yaml (Parameters for apriltag_ros)

**lunabot_description**
  - **meshes** (Meshes for robot model in RViz2)
    - base_link.stl
    - camera_link.stl
    - ebox_link.stl
    - lidar1_link.stl
    - lidar2_link.stl
    - nuc_link.stl
    - wheel_link.stl
  - **urdf**
    - common_properties.xacro (Defines material colors and physical properties)
    - sim_bot.xacro (URDF for the simulation bot used in Gazebo)
    - test_bot.xacro (URDF for the physical test version of the robot used)
  - **worlds** (Worlds for Gazebo simulation)
    - moon.world (Gazebo world representing the moon environment for simulation)

**lunabot_external** (Packages from external sources)
  - rf2o_laser_odometry
  
**scripts**
  - canable_start.sh (Script for setting up CAN interface)
  - install_dependencies.sh (Script for installing dependencies)
  - setup_udev_rules.sh (Script for setting up D456 camera udev rules)

  