import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import (
    get_package_share_path,
    get_package_share_directory,
)


def generate_launch_description():
    realsense_launch_path = os.path.join(
        get_package_share_path("realsense2_camera"),
        "launch",
        "rs_launch.py",
    )

    # RPLidar A3
    lidar1 = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        remappings=[("/scan", "/scan")],
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": "/dev/ttyUSB0",
                "serial_baudrate": 256000,
                "scan_frequency": 25.0,
                "frame_id": "lidar1_link",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "Sensitivity",
            }
        ],
        output="screen",
    )

    # RPLidar S2L
    lidar2 = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        remappings=[("/scan", "/scan2")],
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": "/dev/ttyUSB1",
                "serial_baudrate": 1000000,
                "frame_id": "lidar2_link",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "DenseBoost",
            }
        ],
        output="screen",
    )

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={
            "camera_name": "d455",
            "camera_namespace": "d455",
            "device_type": "d455",
            "enable_gyro": "true",
            "enable_accel": "true",
            "unite_imu_method": "2",
            "depth_module.profile": "640x360x90",
            "rgb_camera.profile": "640x360x90",
        }.items(),
    )

    imu_rotator = Node(package="lunabot_autonomous", executable="imu_rotator")

    d455_imu_filter = Node(
        package="imu_complementary_filter",
        executable="complementary_filter_node",
        name="complementary_filter_gain_node",
        output="screen",
        parameters=[
            {"publish_tf": False},
            {"fixed_frame": "odom"},
            {"do_bias_estimation": True},
            {"do_adaptive_gain": True},
            {"use_mag": False},
            {"gain_acc": 0.01},
            {"gain_mag": 0.01},
        ],
    )

    lidar1_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        remappings=[("/scan", "/scan_raw")],
        parameters=[
            PathJoinSubstitution(
                [
                    get_package_share_directory("lunabot_bringup"),
                    "params",
                    "a3_lidar_params.yaml",
                ]
            )
        ],
    )

    lidar2_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        remappings=[("/scan", "/scan2_raw"), ("/scan_filtered", "/scan2_filtered")],
        parameters=[
            PathJoinSubstitution(
                [
                    get_package_share_directory("lunabot_bringup"),
                    "params",
                    "s2_lidar_params.yaml",
                ]
            )
        ],
    )

    lidar1_odom = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        parameters=[
            {
                "laser_scan_topic": "/scan",
                "odom_topic": "/odom_lidar",
                "publish_tf": False,
                "base_frame_id": "base_link",
                "odom_frame_id": "odom",
                "init_pose_from_topic": "",
                "freq": 30.0,
            }
        ],
        arguments=["--ros-args", "--disable-stdout-logs", "--disable-rosout-logs"],
    )

    robot_controller = Node(
        package="lunabot_autonomous",
        executable="robot_controller",
        parameters=[
            {
                "xbox_mode": True,
                "outdoor_mode": False,
            }
        ],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    hardware_monitor = Node(package="lunabot_autonomous", executable="hardware_monitor")

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("lunabot_bringup"),
                "params",
                "ekf_params.yaml",
            )
        ],
    )

    apriltag = Node(
        name="apriltag",
        package="apriltag_ros",
        executable="apriltag_node",
        parameters=[
            os.path.join(
                get_package_share_directory("lunabot_bringup"),
                "params",
                "tag_params.yaml",
            )
        ],
        remappings=[
            ("/image_rect", "/d455/color/image_raw"),
            ("/camera_info", "/d455/color/camera_info"),
            ("/detections", "/tag_detections"),
        ],
    )

    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
        name="static_transform_publisher",
    )

    base_link_to_d455_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.4305",
            "0",
            "0.6096",
            "0",
            "-0.5236",
            "0",
            "base_link",
            "d455_link",
        ],
        output="screen",
        name="static_transform_publisher",
    )

    return LaunchDescription(
        [
            lidar1,
            lidar1_odom,
            lidar2,
            apriltag,
            realsense,
            imu_rotator,
            d455_imu_filter,
            ekf,
            robot_controller,
            hardware_monitor,
            base_link_to_d455_tf,
            map_to_odom_tf,
        ]
    )
