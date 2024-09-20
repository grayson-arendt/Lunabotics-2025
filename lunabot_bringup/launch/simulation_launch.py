import os
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_path
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)


def generate_launch_description():
    declare_control = DeclareLaunchArgument(
        "control_method",
        default_value="keyboard",
        description="Select control method: 'keyboard' or other options",
    )

    rviz_config_path = os.path.join(
        get_package_share_path("lunabot_config"),
        "rviz",
        "robot_view.rviz",
    )

    urdf_path = os.path.join(
        get_package_share_path("lunabot_simulation"), "urdf", "sim_bot.xacro"
    )

    world_path = os.path.join(
        get_package_share_path("lunabot_simulation"), "worlds", "artemis_arena.world"
    )

    robot_description = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_path("gazebo_ros"), "launch", "gazebo.launch.py"
            )
        ),
        launch_arguments={"world": world_path}.items(),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "sim_bot",
            "-x",
            "2.6",
            "-y",
            "1.5",
            "-Y",
            "-2.5",
            "-z",
            "0.4",
        ],
        output="screen",
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        condition=LaunchConfigurationEquals("control_method", "xbox"),
    )

    joy_teleop_node = Node(
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
        remappings=[("/cmd_vel", "/diff_drive_controller/cmd_vel_unstamped")],
        condition=LaunchConfigurationEquals("control_method", "xbox"),
    )

    keyboard_teleop_node = ExecuteProcess(
        cmd=[
            "gnome-terminal",
            "--",
            "bash",
            "-c",
            "ros2 run lunabot_simulation keyboard_teleop.py;  exec bash",
        ],
        condition=LaunchConfigurationEquals("control_method", "keyboard"),
        output="screen",
    )

    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
        name="static_transform_publisher",
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_path("lunabot_config"),
                "params",
                "ekf_params.yaml",
            )
        ],
    )

    imu_filter_node = Node(
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

    lidar_odom = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        parameters=[
            {
                "laser_scan_topic": "/scan",
                "odom_topic": "/lidar_odom",
                "publish_tf": False,
                "base_frame_id": "base_link",
                "odom_frame_id": "odom",
                "init_pose_from_topic": "",
                "freq": 30.0,
            }
        ],
        arguments=["--ros-args", "--disable-stdout-logs", "--disable-rosout-logs"],
    )

    blade_joint_controller_node = Node(
        package="lunabot_simulation", executable="blade_joint_controller"
    )

    topic_remapper_node = Node(
        package="lunabot_simulation", executable="topic_remapper"
    )

    odom_tf_publisher = Node(
        package="lunabot_system", executable="odom_tf_publisher"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    return LaunchDescription(
        [
            declare_control,
            gazebo_launch,
            rviz2_node,
            joy_node,
            joy_teleop_node,
            keyboard_teleop_node,
            ekf_node,
            imu_filter_node,
            spawn_entity_node,
            joint_state_broadcaster_spawner,
            diff_drive_controller_spawner,
            position_controller_spawner,
            blade_joint_controller_node,
            robot_state_publisher_node,
            topic_remapper_node,
            lidar_odom,
            map_to_odom_tf,
        ]
    )
