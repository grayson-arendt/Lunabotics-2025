# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# DESCRIPTION #
# ----------- #
# Use this launch file to launch a t265 device.
# command line example:
# ros2 launch realsense2_camera rs_t265_launch.py

# The Parameters available for definition in the command line are described in rs_launch.configurable_parameters
# For example: to disable fisheye sensors:
# ros2 launch realsense2_camera rs_d455_launch.py

"""Launch realsense2_camera node."""
import copy
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
import rs_launch

local_parameters = [{'name': 'device_type', 'default': 'd455', 'description': 'choose device by type'},
                    {'name': 'camera_name', 'default': 'd455', 'description': 'camera namespace'},
                    {'name': 'camera_namespace', 'default': 'd455', 'description': 'namespace'},
                    {'name': 'enable_gyro', 'default': 'true', 'description': 'enable gyro'},
                    {'name': 'enable_accel','default': 'true', 'description': 'enable accel'},
                    {'name': 'unite_imu_method','default': '2', 'description': 'unite imu method'},
                    {'name': 'depth_module.profile','default': '848x480x60', 'description': 'depth profile'},
                    {'name': 'rgb_module.profile','default': '848x480x60', 'description': 'rgb profile'},
                   ]

def generate_launch_description():
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) + 
        [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rs_launch.py']),
            launch_arguments=rs_launch.set_configurable_parameters(local_parameters).items(),
        ),
    ])