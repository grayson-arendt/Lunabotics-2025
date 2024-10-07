import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    simulation_dir = get_package_share_directory("lunabot_simulation")
    config_dir = get_package_share_directory("lunabot_config")
    rviz_config_file = os.path.join(config_dir, "rviz", "robot_view.rviz")
    urdf_file = os.path.join(simulation_dir, "urdf", "bulldozer_bot.xacro")

    rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
    )

    robot_description = Command(["xacro ", urdf_file])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": False}],
    )

    ld = LaunchDescription()

    ld.add_action(rviz_launch)
    ld.add_action(joy_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)

    return ld
