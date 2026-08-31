#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EqualsSubstitution


def generate_launch_description():
    declare_use_sim = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Whether we are in simulation or not",
    )

    # In simulation, use sim-compatible servers (control bucket via position controller)
    sim_servers = GroupAction(
        actions=[
            Node(
                package="lunabot_nav",
                executable="excavation_server_sim",
                name="excavation_server",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="lunabot_nav",
                executable="depositing_server_sim",
                name="depositing_server",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
        ],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("use_sim"), "true")),
    )

    # For real hardware, use hardware servers (control bucket via CAN motors)
    real_servers = GroupAction(
        actions=[
            Node(
                package="lunabot_nav",
                executable="assisted_excavation_server",
                name="assisted_excavation_server",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
            Node(
                package="lunabot_nav",
                executable="auto_excavation_server",
                name="auto_excavation_server",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
            Node(
                package="lunabot_nav",
                executable="assisted_depositing_server",
                name="assisted_depositing_server",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
            Node(
                package="lunabot_nav",
                executable="auto_depositing_server",
                name="auto_depositing_server",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
        ],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("use_sim"), "false")),
    )

    return LaunchDescription([
        declare_use_sim,
        sim_servers,
        real_servers,
    ])
