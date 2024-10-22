import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    GroupAction,
    ExecuteProcess,
)
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_dir = get_package_share_directory("lunabot_config")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    nav2_params_file = os.path.join(config_dir, "params", "nav2_sim_params.yaml")
    rtabmap_params_file = os.path.join(config_dir, "params", "rtabmap_params.yaml")
    ekf_params_file = os.path.join(config_dir, "params", "ekf_params.yaml")

    declare_robot_mode = DeclareLaunchArgument(
        "robot_mode", default_value="manual", choices=["manual", "autonomous"]
    )

    declare_teleop_mode = DeclareLaunchArgument(
        "teleop_mode", default_value="keyboard", choices=["keyboard", "xbox"]
    )

    topic_remapper_node = Node(
        package="lunabot_simulation", executable="topic_remapper"
    )

    rgbd_sync1_node = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="rgbd_sync1",
        output="screen",
        parameters=[
            {"use_sim_time": True, "approx_sync": True, "sync_queue_size": 1000}
        ],
        remappings=[
            ("rgb/image", "/d456/color/image_raw"),
            ("depth/image", "/d456/depth/image_rect_raw"),
            ("rgb/camera_info", "/d456/color/camera_info"),
            ("rgbd_image", "rgbd_image"),
        ],
        namespace="d456",
        arguments=["--ros-args", "--log-level", "error"],
    )

    rgbd_sync2_node = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="rgbd_sync2",
        output="screen",
        parameters=[
            {"use_sim_time": True, "approx_sync": True, "sync_queue_size": 1000}
        ],
        remappings=[
            ("rgb/image", "/d455/color/image_raw"),
            ("depth/image", "/d455/depth/image_rect_raw"),
            ("rgb/camera_info", "/d455/color/camera_info"),
            ("rgbd_image", "rgbd_image"),
        ],
        namespace="d455",
        arguments=["--ros-args", "--log-level", "error"],
    )

    slam_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="log",
        parameters=[
            {
                "use_sim_time": True,
                "rgbd_cameras": 2,
                "subscribe_depth": False,
                "subscribe_rgbd": True,
                "subscribe_rgb": False,
                "subscribe_odom_info": False,
                "odom_sensor_sync": True,
                "frame_id": "base_link",
                "map_frame_id": "map",
                "odom_frame_id": "odom",
                "publish_tf": True,
                "publish_tf_odom": False,
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
                "frame_id": "base_link",
                "odom_frame_id": "odom",
                "publish_tf": False,
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
            ("odom", "/icp_odom"),
        ],
        arguments=["--ros-args", "--log-level", "error"],
    )

    rgbd_odometry_node = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        output="screen",
        parameters=[
            {
                "frame_id": "base_link",
                "odom_frame_id": "odom",
                "publish_tf": False,
                "approx_sync": True,
                "subscribe_rgbd": True,
            }
        ],
        remappings=[("rgbd_image", "/d455/rgbd_image"), ("odom", "/rgbd_odom")],
        arguments=["--ros-args", "--log-level", "error"],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
            },
            ekf_params_file,
        ],
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
            "use_sim_time": "true",
            "params_file": nav2_params_file,
        }.items(),
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        condition=LaunchConfigurationEquals("teleop_mode", "xbox"),
    )

    teleop_twist_joy_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[
            {
                "require_enable_button": False,
                "axis_linear.x": 1,
                "axis_angular.yaw": 0,
                "enable_turbo_button": 5,
                "scale_linear_turbo.x": 1.5,
                "scale_angular_turbo.yaw": 1.5,
            }
        ],
        condition=LaunchConfigurationEquals("teleop_mode", "xbox"),
    )

    keyboard_teleop_node = ExecuteProcess(
        cmd=[
            "gnome-terminal",
            "--",
            "bash",
            "-c",
            "ros2 run lunabot_simulation keyboard_teleop.py; exit",
        ],
        output="screen",
        condition=LaunchConfigurationEquals("teleop_mode", "keyboard"),
    )

    ld = LaunchDescription()

    ld.add_action(declare_robot_mode)
    ld.add_action(declare_teleop_mode)
    ld.add_action(topic_remapper_node)
    ld.add_action(rgbd_sync1_node)
    ld.add_action(rgbd_sync2_node)

    ld.add_action(
        GroupAction(
            actions=[
                TimerAction(
                    period=2.0,
                    actions=[
                        rgbd_odometry_node,
                        icp_odometry_node,
                        ekf_node,
                    ],
                ),
                TimerAction(
                    period=5.0,
                    actions=[
                        slam_node,
                    ],
                ),
                TimerAction(
                    period=20.0,
                    actions=[
                        nav2_launch,
                    ],
                ),
                joy_node,
                teleop_twist_joy_node,
                keyboard_teleop_node,
            ],
            condition=LaunchConfigurationEquals("robot_mode", "manual"),
        )
    )

    ld.add_action(
        GroupAction(
            actions=[
                TimerAction(
                    period=3.0,
                    actions=[
                        localization_server_node,
                        navigation_client_node,
                    ],
                ),
                TimerAction(
                    period=50.0,
                    actions=[
                        rgbd_odometry_node,
                        icp_odometry_node,
                        ekf_node,
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
