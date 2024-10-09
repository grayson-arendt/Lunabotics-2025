import os
import random
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    simulation_dir = get_package_share_directory("lunabot_simulation")
    config_dir = get_package_share_directory("lunabot_config")

    rviz_config_file = os.path.join(config_dir, "rviz", "robot_view.rviz")
    urdf_sim_file = os.path.join(simulation_dir, "urdf", "simulation_bot.xacro")
    urdf_real_file = os.path.join(simulation_dir, "urdf", "real_bot.xacro")
    world_file = os.path.join(simulation_dir, "worlds", "artemis_arena.world")
    rviz_config_file = os.path.join(config_dir, "rviz", "robot_view.rviz")

    robot_sim_description = Command(["xacro ", urdf_sim_file])
    robot_real_description = Command(["xacro ", urdf_real_file])

    orientations = {"north": 0.0, "east": 1.5708, "south": 3.1416, "west": -1.5708}
    robot_orientation = random.choice(list(orientations.values()))

    declare_visualization_mode = DeclareLaunchArgument(
        "visualization_mode", default_value="simulation", choices=["simulation", "real"]
    )

    rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
            )
        ),
        launch_arguments={"world": world_file}.items(),
    )

    spawn_robot_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "simulation_bot",
            "-x",
            "2.5",
            "-y",
            "1.6",
            "-Y",
            str(robot_orientation),
            "-z",
            "0.35",
        ],
        output="screen",
    )

    robot_sim_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_sim_description, "use_sim_time": True}],
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

    blade_joint_controller_node = Node(
        package="lunabot_simulation", executable="blade_joint_controller"
    )

    robot_real_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_real_description, "use_sim_time": False}],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": False}],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
    )


    ld = LaunchDescription()

    ld.add_action(declare_visualization_mode )
    ld.add_action(rviz_launch)

    ld.add_action(
        GroupAction(
            actions=[
                gazebo_launch,
                spawn_robot_node,
                robot_sim_state_publisher,
                joint_state_broadcaster_spawner,
                diff_drive_controller_spawner,
                position_controller_spawner,
                blade_joint_controller_node,
            ],
            condition=LaunchConfigurationEquals("visualization_mode", "simulation"),
        )
    )

    ld.add_action(
        GroupAction(
            actions=[
                joy_node,
                robot_real_state_publisher_node,
                joint_state_publisher_node
            ],
            condition=LaunchConfigurationEquals("visualization_mode", "real"),
        )
    )

    return ld
