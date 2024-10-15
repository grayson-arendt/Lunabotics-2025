# Project Overview

This repository contains the software developed by the College of DuPage team for the 2024-2025 NASA Lunabotics competition. It is built for ROS 2 Humble on Ubuntu 22.04.

## System Components

- ASRock 4X4 BOX-8840U
- RPLidar S2L
- RPLidar S3
- Intel RealSense D455 Depth Camera
- Intel RealSense D456 Depth Camera
- REV Robotics NEO V1.1 (x2)
- REV Robotics Spark Max (x4)
- REV Robotics Power Distribution Hub
- Turnigy 14.8V 8000mAh LiPo Battery
- Turnigy 14.8V 12000mAh LiPo Battery
- ODrive USB-CAN Adapter

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
./install_dependencies.sh
```

#### 4. Build the workspace

Building may take some time due to the rtabmap and rtabmap_ros packages in lunabot_external. Various flags such as `DRTABMAP_SYNC_MULTI_RGBD=ON` need to be set to enable extra features for RTAB-Map.

To avoid building the entire workspace all over again after the initial build if you make changes, use `colcon build --packages-select name_of_package` and choose the package that you made changes to for rebuilding. You can list multiple packages after the `--packages-select` flag.

```bash
cd ~/lunabot_ws
colcon build --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DWITH_OPENCV=ON -DWITH_APRILTAG=ON -DWITH_OPENGV=OFF --parallel-workers 4 # Modify number as needed, this is how many packages are built concurrently
```

#### 5. (Optional) Set MAKEFLAG and Rebuild
If your computer keeps crashing while trying to build, `colcon build` may be trying to do too many things at once. Setting this flag to `-j4` limits each package's internal make jobs to 4 threads. You can either increase or reduce both this and `--parallel-workers`, increasing will make it build faster while decreasing will put less load on your computer but will build slower.

```bash
export MAKEFLAGS="-j4" # Modify number as needed
```

Next, rebuild using the same commands in step **4. Build the workspace**.

## Simulating the Robot
Note: The side of the robot without the bulldozer blade is considered the front of the robot, as the camera is able to see obstacles better on that side. Therefore, controls may seem reversed but it is intentional.

There are two modes for simulating the robot: **manual** and **autonomous**. 

##### Manual Mode
This launches a teleop node for controlling the Gazebo robot with either a keyboard or Xbox controller along with Navigation2 and RTABMap, but does not launch the `localization_server` or `navigation_client`. In this mode, you can drive the robot around, map the arena, and play around with setting Nav2 goals in RViz2.

##### Autonomous Mode
This launches `localization_server` and `navigation_client` and will not allow the user to teleop the robot. Instead, it will follow the commands in the server and client and perform a one-cycle autonomy sequence. 

#### 1. Navigate to workspace and source setup

```bash
cd ~/lunabot_ws
source install/setup.bash
```
#### Open separate terminal windows and source the workspace setup for each next step:

#### 2. Visualize with RViz2

```bash
ros2 launch lunabot_bringup visualization_launch.py
```

#### 3. Launch simulation

```bash
ros2 launch lunabot_bringup simulation_launch.py # teleop_mode:=xbox (for Xbox controller) robot_mode:=autonomous (to run in autonomous mode)
```

<p align="center">
  <img src="demo.png">
</p>

## Running the Physical Robot


### SSH Into Robot Computer

SSH (Secure Shell) allows you access the a computer over the network and run commands. The host computer is the computer that you are personally interfacing with, such as laptop. For any future steps that require being ran on the robot computer, you will need to be connected via SSH.

#### 1. Create SSH-key (host computer)

```bash
ssh-keygen
```

#### 2. Get username and IP address (robot computer)

```bash
whoami
```
This will return the username of the robot computer, although you can also see the username just by looking at the terminal. It is the first name before the @, for example, the username would be `grayson` for `grayson@NUC`.

Next, get the IP address:
```bash
hostname -I
```

#### 3. Establish SSH connection (host computer)

Using the username and IP address from the previous step, now you can connect to the robot computer. It may look something like this for example:

```bash
ssh asrock@192.168.10.1 # (General format: username@ip_address)
```
 It will ask you if you are sure you want to connect, type `yes`. Then, confirm by typing in the robot computer's password. 

### Configure Device Permissions 

#### 1. Add user to dialout group then restart (robot computer)

```bash
sudo usermod -a -G dialout $USER
```

Use `ls /dev/ttyUSB*` to identify device numbers if the lidars are disconnected and reconnected, then adjust the lidar `"serial_port"` parameters in `real_launch.py` accordingly.

#### 2. Setup camera udev rules (robot computer)

```bash
cd ~/lunabot_ws/src/Lunabotics-2025/scripts
chmod +x setup_udev_rules.sh
sudo ./setup_udev_rules.sh
```

Make sure all cameras are unplugged while setting up the udev rules.

### Running Launch Files

#### 1. Initialize SocketCAN (robot computer)

```bash
cd ~/lunabot_ws/src/Lunabotics-2025/scripts/
chmod +x canable_start.sh
./canable_start.sh
```

#### 2. Source workspace setup (both host and robot computer)

```bash
cd ~/lunabot_ws
source install/setup.bash
```

#### Open separate terminal windows and source the workspace setup for each next step:

#### 3. Visualize with RViz2 (host computer)

```bash
ros2 launch lunabot_bringup visualization_launch.py visualization_mode:=real
```

#### 4. Launch the real robot (robot computer)

```bash
ros2 launch lunabot_bringup real_launch.py # robot_mode:=autonomous (to run in autonomous mode)
```

## Project Structure

**lunabot_bringup**: This package contains the launch files to bring up various robot components.
- **launch**
  - **real_launch.py**: Launches the nodes required for bringing up the physical robot hardware and sensors in a real-world environment.
  - **simulation_launch.py**: Launches the necessary nodes for simulating the robot in Gazebo.
  - **visualization_launch.py**: Launches RViz2 to visualize the robot and its sensor data.

**lunabot_config**: This package contains configuration files for behavior trees, RViz2 settings, and sensor parameters.
- **behavior_trees**
  - **navigate_through_poses_w_replanning_and_recovery.xml**: A behavior tree used with Navigation2 to implement behaviors like goal replanning and recovery for NavigateThroughPoses action.
  - **navigate_to_pose_w_replanning_goal_patience_and_recovery.xml**: A behavior tree used with Navigation2 to implement behaviors like goal replanning, patience, and recovery NavigateToPose action.
- **params**
  - **gazebo_params.yaml**: Configuration parameters for controllers in the Gazebo simulation.
  - **nav2_real_params.yaml**: Parameters for configuring Navigation2 when running on the physical robot.
  - **nav2_sim_params.yaml**: Parameters for configuring Navigation2 in simulation.
  - **rtabmap_params.yaml**: Parameters for configuring RTAB-Map.
  - **s2l_filter_params.yaml**: Parameters for filtering lidar data from the RPLidar S2L sensor.
- **rviz**
  - **robot_view.rviz**: Configuration file for RViz2 that defines how the robot and its environment are visualized.

**lunabot_external** This folder contains external packages that need to be built from source.

**lunabot_simulation**: This package contains assets and code for simulating the robot in Gazebo.
- **meshes**: Contains the 3D models used to visualize the robot in Gazebo and RViz2.
- **models**: Contains environmental models for the Gazebo simulation.
- **src**
  - **blade_joint_controller.cpp**: Source code for controlling the bulldozer blade's joint.
  - **teleop**: Contains teleop scripts.
    - **keyboard_teleop.py**: Script for teleoping the robot using keyboard inputs.
- **urdf**
  - **real_bot.xacro**: URDF description of the real robot.
  - **simulation_bot.xacro**: URDF description of the robot used in Gazebo for simulation.
- **worlds**
  - **artemis_arena.world**: Gazebo world file simulating the Artemis Arena for testing the robot.

**lunabot_system**: This package contains various autonomy/manual controllers and utilities.
- **action**
  - **Localization.action**: Action definition for localization.
- **src**
  - **control**
    - **localization_server.cpp**: Server responsible for handling localization with an AprilTag.
    - **navigation_client.cpp**: Receives localization response and sends navigation goals and triggers robot behaviors when goals are reached.
    - **robot_controller.cpp**: Converts `/cmd_vel` commands and `/joy` inputs into physical motor speed outputs.
  - **utils**
    - **hardware_monitor.cpp**: Monitors hardware topics and outputs error messages if sensor data is not received.
    - **imu_rotator.cpp**: Processes and rotates IMU data into the East-North-Up (ENU) frame.

**scripts**: Various setup and utility scripts.
- **canable_start.sh**: Sets up the CAN bus interface for motor controller communication.
- **config**
  - **99-realsense-d4xx-mipi-dfu.rules**: Udev rules for RealSense D456 cameras.
  - **99-realsense-libusb.rules**: Udev rules for RealSense cameras using the USB interface.
- **install_dependencies.sh**: Script to install required dependencies for the robot software.
- **setup_udev_rules.sh**: Script to set up udev rules for the Intel RealSense camera.

