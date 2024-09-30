# Project Overview

This repository contains the software developed by the College of DuPage team for the 2024-2025 NASA Lunabotics competition. It is built for ROS 2 Humble on Ubuntu 22.04.

## System Components

- ASRock 4X4 BOX-8840U
- RPLidar S2L
- RPLidar A3
- Intel RealSense D456 Depth Camera
- REV Robotics Neo Vortex (x4)
- REV Robotics Spark Flex (x4)
- REV Robotics Spark Max (x4)
- Turnigy 14.8V 8000mAh LiPo Battery
- Turnigy 14.8V 12000mAh LiPo Battery
- AndyMark Power Distribution Panel
- MKS CANable Pro

## Software Requirements

- rtabmap
- rtabmap_ros
- rplidar_ros
- apriltag_ros
- navigation2
- robot_localization
- laser_filters
- imu_complementary_filter
- ros2_control
- ros2_controllers
- gazebo_ros2_control
- gazebo_ros_pkgs
- rviz2
- xacro

## Installation

Note: You will need to have already installed ROS 2 Humble before continuing with installation. The guide can be found [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). Install both `ros-humble-desktop` and `ros-dev-tools`.

#### (Optional) 1. Append lines to .bashrc

.bashrc is a script that runs everytime a new terminal window is opened and has various configurations, environment variables, and commands for setup. There is a bug in the VSCode terminal that will cause a symbol lookup error, so you have to unset the path variable using `unset GTK_path`. If you haven't already added `source /opt/ros/humble/setup.bash` to your .bashrc file, it simply runs the setup script for ROS 2 Humble.

```bash
echo 'unset GTK_PATH' >> ~/.bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```

This will permanently append these two lines to your .bashrc file, so there is no need to run it again. If you want to edit the file manually, use `nano ~/.bashrc` or `gedit ~/.bashrc` if you prefer a text editor GUI instead.

#### 2. Setup workspace and clone repository

```bash
mkdir -p ~/lunabot_ws/src
cd ~/lunabot_ws/src
git clone https://github.com/grayson-arendt/Lunabotics-2025.git
```

#### 3. Install dependencies

Run the installation script to install the required dependencies. `chmod +x` gives permission for the script to be executable. In any step that `chmod +x` is used, it will only need to be ran once.

```bash
cd ~/lunabot_ws/src/Lunabotics-2025/scripts
chmod +x install_dependencies.sh
sudo ./install_dependencies.sh
```

#### 4. Build the workspace

```bash
cd ~/lunabot_ws
colcon build
```

## Simulating the Robot

Gazebo simulation can be slow depending on your computer's specifications. You can control the robot with an Xbox One controller by specifying the control mode with `control_mode:=xbox`. In this mode, the left joystick controls the drivetrain, while the right joystick’s y-axis moves the dozer blade.

#### 1. Navigate to workspace and source setup

```bash
cd ~/lunabot_ws
source install/setup.bash
```

#### 2. Launch Gazebo and RViz2

```bash
ros2 launch lunabot_bringup simulation_launch.py control_mode:=keyboard #control_mode:=xbox
```

<p align="center">
  <img src="sample2.png">
</p>

## Running the Physical Robot

### Configure Device Permissions

The rplidar_ros package needs to access /dev/ttyUSB0 and /dev/ttyUSB1 (using both lidars). While you can run `sudo chmod +x /dev/ttyUSB0` for example, it would need to be ran each time on startup.

#### 1. Add user to dialout group then restart the computer

```bash
sudo usermod -a -G dialout $USER
```

Use `ls /dev/ttyUSB*` to identify device numbers if the lidars are disconnected and reconnected, then adjust the lidar parameters in `hardware_launch.py` accordingly.

#### 2. Setup camera udev rules

```bash
cd ~/lunabot_ws/src/Lunabotics-2025/scripts
chmod +x setup_udev_rules.sh
sudo ./setup_udev_rules.sh
```

Make sure all cameras are unplugged while setting up the udev rules.

### Running Launch Files

#### 1. Initialize SocketCAN

```bash
cd ~/lunabot_ws/src/Lunabotics-2025/scripts/
chmod +x canable_start.sh
./canable_start.sh
```

#### 2. Source workspace setup

```bash
cd ~/lunabot_ws
source install/setup.bash
```

#### Open separate terminal windows and source the workspace setup for each next step:

#### 3. Visualize with RViz2 (on host computer)

```bash
ros2 launch lunabot_bringup external_launch.py
```

#### 4. Launch hardware

```bash
ros2 launch lunabot_bringup hardware_launch.py
```

#### 5. Start RTAB-Map for mapping

```bash
ros2 launch lunabot_bringup mapping_launch.py
```

#### 6. Start Navigation2 for navigation

```bash
ros2 launch lunabot_bringup navigation_launch.py
```

In RViz2 on the host computer, you will now be able to select a "Nav2 Goal" in the GUI and have the robot navigate to that location.

#### (Optional) 7. Start navigation client

```bash
ros2 run lunabot_system navigation_client
```

The action client will send two goals, one for excavation zone and another for construction zone.

<p align="center">
  <img src="sample.png">
</p>

## Project Structure

**lunabot_bringup**: This package contains the launch files to bring up various robot components.
- **launch**
  - **external_launch.py**: Launches the necessary nodes for RViz2 and robot state/joint publishers. This is used on an external computer/laptop to visualize the robot and its state in real time.
  - **hardware_launch.py**: Launches the nodes required to bring up the physical robot's hardware, including lidar sensors, depth cameras, and robot controller.
  - **mapping_launch.py**: Launches RTAB-Map, a real-time appearance-based mapping node used for simultaneous localization and mapping (SLAM). This allows the robot to create a map of its environment while also localizing itself within that map.
  - **navigation_launch.py**: Launches Navigation2, which provides autonomous navigation capabilities. It uses sensor data, the map, and the robot’s position to plan and execute paths to goals set by the user or an action client.
  - **simulation_launch.py**: Launches the necessary nodes for simulating the robot in Gazebo and RViz2.

**lunabot_config**: This package contains configuration files such as behavior trees, RViz settings, and parameters for the robot and sensors.
- **behavior_trees**
  - **navigate_to_pose_w_replanning_goal_patience_and_recovery.xml**: A behavior tree used with the Navigation2 stack to enable goal replanning, patience, and recovery behaviors.
- **config**
  - **robot_view.rviz**: Configuration file for RViz2, defining how the robot and its environment are displayed.
- **params**
  - **a3_lidar_params.yaml**: Parameter file containing settings for processing data from the RPLidar A3.
  - **ekf_params.yaml**: Parameters for the Extended Kalman Filter (EKF) used by the robot localization node to fuse sensor data like IMU, lidar odometry, and wheel encoders.
  - **nav2_params.yaml**: Configuration parameters for the Navigation2 stack.
  - **s2_lidar_params.yaml**: Similar to `a3_lidar_params.yaml`, but for the RPLidar S2L.
  - **sim_params.yaml**: Parameters for the ros2_control controllers that simulate the robot’s movement in Gazebo.
  - **tag_params.yaml**: Parameters for the apriltag_ros package, which detects AprilTag fiducial markers for robot localization.

**lunabot_external**
- **rf2o_laser_odometry**: A package used to compute laser odometry, estimating the robot’s position over time based on lidar data.

**lunabot_simulation**: This package contains assets and code for simulating the robot in Gazebo.
- **meshes**: Contains the 3D models used to visualize the robot in Gazebo and RViz2.
  - **base_link.stl**: Mesh for the robot's base frame.
  - **blade_link.stl**: Mesh for the robot's bulldozer blade.
  - **camera_link.stl**: Mesh for the camera.
  - **ebox_link.stl**: Mesh for the electronics box containing the onboard computer and other electronics.
  - **lidar1_link.stl**: Mesh for the RPLidar A3 mount.
  - **lidar2_link.stl**: Mesh for the RPLidar S2 mount.
  - **nuc_link.stl**: Mesh for the Intel NUC.
  - **wheel_link.stl**: Mesh for the robot’s wheels.
- **models**: Contains environmental models for the Gazebo simulation.
  - **column.stl**: Model of the Artemis Arena central support column.
  - **lunar_surface.stl**: A model representing the lunar surface, simulating rough terrain for the robot.
  - **rock_rough.stl**: A rough rock model used for simulating obstacles.
  - **rock_round.stl**: A round rock model used for simulating obstacles.
- **urdf**
  - **common_properties.xacro**: Defines common properties like material colors for various parts of the robot.
  - **sim_bot.xacro**: URDF file for the simulated robot in Gazebo.
  - **test_bot.xacro**: URDF file for a test version of the physical robot.
- **worlds**
  - **artemis_arena.world**: A Gazebo world simulating the Artemis Arena, which includes terrain and obstacles similar to the Lunabotics competition.

**lunabot_system**: This package contains motor controller and utilities to operate the physical robot.
- **src**
  - **control**
    - **robot_controller.cpp**: This node controls the entire robot, processing both autonomous `cmd_vel` commands and manual inputs from a game controller.
    - **navigation_client.cpp**: This action client sends goals to the navigation action server and activates robot mechanisms when each goal is reached.
  - **utils**
    - **hardware_monitor.cpp**: Monitors various hardware components (e.g., sensors) and flags errors if they fail to send data.
    - **imu_rotator.cpp**: Processes IMU data, rotating it into the East-North-Up (ENU) frame for use in localization.

**scripts**
- **canable_start.sh**: A script for setting up the CAN interface, enabling communication between motor controllers and the onboard computer.
- **install_dependencies.sh**: A script to install the necessary dependencies for the robot's software stack.
- **setup_udev_rules.sh**: A script for configuring udev rules for the Intel RealSense D456 camera.