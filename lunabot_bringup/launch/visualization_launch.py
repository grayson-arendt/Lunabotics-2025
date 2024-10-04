import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_dir = get_package_share_directory("lunabot_config")
    rviz_config_file = os.path.join(config_dir, "rviz", "robot_view.rviz")

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

    ld = LaunchDescription()

    ld.add_action(rviz_launch)
    ld.add_action(joy_node)

    return ld
