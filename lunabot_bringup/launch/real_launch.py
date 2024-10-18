import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_dir = get_package_share_directory("lunabot_config")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    realsense_dir = get_package_share_directory("realsense2_camera")

    nav2_params_file = os.path.join(config_dir, "params", "nav2_real_params.yaml")
    rtabmap_params_file = os.path.join(config_dir, "params", "rtabmap_params.yaml")

    declare_robot_mode = DeclareLaunchArgument(
        "robot_mode", default_value="manual", choices=["manual", "autonomous"]
    )

    rgbd_sync1_node = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="rgbd_sync1",
        output="screen",
        parameters=[{"use_sim_time": False, "approx_sync": True, "sync_queue_size": 1000}],
        remappings=[
            ("rgb/image", "/d456/color/image_raw"),
            ("depth/image", "/d456/depth/image_rect_raw"),
            ("rgb/camera_info", "/d456/color/camera_info"),
            ("rgbd_image", "/d456/rgbd_image"),
        ],
        namespace="d456",
        arguments=["--ros-args", "--log-level", "error"],
    )

    rgbd_sync2_node = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="rgbd_sync2",
        output="screen",
        parameters=[{"use_sim_time": False, "approx_sync": True, "sync_queue_size": 1000}],
        remappings=[
            ("rgb/image", "/d455/color/image_raw"),
            ("depth/image", "/d455/depth/image_rect_raw"),
            ("rgb/camera_info", "/d455/color/camera_info"),
            ("rgbd_image", "/d455/rgbd_image"),
        ],
        namespace="d455",
        arguments=["--ros-args", "--log-level", "error"],
    )

    slam_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "rgbd_cameras": 2,
                "subscribe_depth": False,
                "subscribe_rgbd": True,
                "subscribe_rgb": False,
                "subscribe_odom_info": False,
                "frame_id": "base_link",
                "map_frame_id": "map",
                "odom_frame_id": "odom",
                "publish_tf": True,
                "publish_tf_odom": True,
                "database_path": "",
                "approx_sync": True,
                "sync_queue_size": 1000,
                "subscribe_scan_cloud": False,
                "subscribe_scan": True,
                "wait_imu_to_init": True,
                "imu_topic": "/d456/imu/data",
            },
            rtabmap_params_file,
        ],
        remappings=[
            ("rgbd_image0", "/d456/rgbd_image"),
            ("rgbd_image1", "/d455/rgbd_image"),
            ("scan", "/scan"),
        ],
        arguments=["--ros-args", "--log-level", "error"],
    )

    icp_odometry_node = Node(
        package="rtabmap_odom",
        executable="icp_odometry",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "frame_id": "base_link",
                "odom_frame_id": "odom",
                "publish_tf": True,
                "approx_sync": True,
                "Reg/Strategy": "1",
                "Odom/Strategy": "0",
                "Odom/FilteringStrategy": "1",
                "Odom/KalmanProcessNoise": "0.001",
                "Odom/KalmanMeasurementNoise": "0.01",
                "Odom/GuessMotion": "true",
                "Odom/GuessSmoothingDelay": "0.1",
                "Icp/VoxelSize": "0.02",
                "Icp/PointToPlane": "true",
                "Icp/PointToPlaneRadius": "0.0",
                "Icp/PointToPlaneK": "20",
                "Icp/CorrespondenceRatio": "0.3",
                "Icp/PMOutlierRatio": "0.65",
                "Icp/Epsilon": "0.0013",
                "Icp/MaxCorrespondenceDistance": "0.05",
            }
        ],
        remappings=[
            ("scan", "/scan"),
            ("odom", "/odom"),
        ],
        arguments=["--ros-args", "--log-level", "error"],
    )

    localization_server_node = Node(
        package="lunabot_system",
        executable="localization_server",
        name="localization_server",
        output="screen",
    )

    navigation_client_node = Node(
        package="lunabot_system",
        executable="navigation_client",
        name="navigation_client",
        output="screen",
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": nav2_params_file,
        }.items(),
    )

    s3_lidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": "/dev/ttyUSB0",
                "serial_baudrate": 1000000,
                "frame_id": "lidar1_link",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "DenseBoost",
            }
        ],
        output="screen",
    )

    s2l_lidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
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
        remappings=[("/scan", "/scan2")],
    )

    d455_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_dir, "launch", "rs_launch.py")
        ),
        launch_arguments={
            "camera_name": "d455",
            "camera_namespace": "",
            "device_type": "d455",
            "publish_tf": "true",
            "enable_gyro": "true",
            "enable_accel": "true",
            "unite_imu_method": "2",
            "depth_module.profile": "640x360x30",
            "rgb_camera.profile": "640x360x30",
        }.items(),
    )

    d456_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_dir, "launch", "rs_launch.py")
        ),
        launch_arguments={
            "camera_name": "d456",
            "camera_namespace": "",
            "device_type": "d456",
            "publish_tf": "true",
            "enable_gyro": "true",
            "enable_accel": "true",
            "unite_imu_method2": "2",
            "depth_module.profile": "640x360x30",
            "rgb_camera.profile": "640x360x30",
        }.items(),
    )

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
        remappings=[
            ("imu/data_raw", "/d455/imu/data_raw"),
            ("imu/data", "/d455/imu/data"),
        ],
    )

    d456_imu_filter = Node(
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
        remappings=[
            ("imu/data_raw", "/d456/imu/data_raw"),
            ("imu/data", "/d456/imu/data"),
        ],
    )

    imu_rotator_node = Node(package="lunabot_system", executable="imu_rotator")

    robot_controller_node = Node(
        package="lunabot_system",
        executable="robot_controller",
        parameters=[
            {
                "xbox_mode": True,
                "outdoor_mode": False,
            }
        ],
    )

    ld = LaunchDescription()

    ld.add_action(declare_robot_mode)
    ld.add_action(rgbd_sync1_node)
    ld.add_action(rgbd_sync2_node)
    ld.add_action(s3_lidar_node)
    ld.add_action(s2l_lidar_node)
    ld.add_action(d455_launch)
    ld.add_action(d456_launch)
    ld.add_action(imu_rotator_node)
    ld.add_action(d455_imu_filter)
    ld.add_action(d456_imu_filter)
    ld.add_action(robot_controller_node)

    ld.add_action(
        GroupAction(
            actions=[
                TimerAction(
                    period=2.0,
                    actions=[
                        icp_odometry_node,
                    ],
                ),
                TimerAction(
                    period=8.0,
                    actions=[
                        slam_node,
                    ],
                ),
                TimerAction(
                    period=15.0,
                    actions=[
                        nav2_launch,
                    ],
                ),
            ],
            condition=LaunchConfigurationEquals("robot_mode", "manual"),
        )
    )

    ld.add_action(
        GroupAction(
            actions=[
                TimerAction(
                    period=5.0,
                    actions=[
                        localization_server_node,
                        navigation_client_node,
                    ],
                ),
                TimerAction(
                    period=50.0,
                    actions=[
                        icp_odometry_node,
                        slam_node,
                    ],
                ),
                TimerAction(
                    period=60.0,
                    actions=[
                        nav2_launch,
                    ],
                ),
            ],
            condition=LaunchConfigurationEquals("robot_mode", "autonomous"),
        )
    )

    return ld
