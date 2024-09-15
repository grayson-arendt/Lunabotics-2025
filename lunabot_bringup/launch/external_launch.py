import os
import xacro
from ament_index_python.packages import (
    get_package_share_path,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():

    bringup_dir = get_package_share_directory("lunabot_bringup")

    urdf_path = os.path.join(
        get_package_share_path("lunabot_description"), "urdf", "test_bot.xacro"
    )

    description = xacro.process_file(urdf_path).toxml()

    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    rviz_config_file = LaunchConfiguration("rviz_config")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="navigation",
        description=(
            "Top-level namespace. The value will be used to replace the "
            "<robot_namespace> keyword on the rviz config file."
        ),
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        description="Whether to apply a namespace to the navigation stack",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(bringup_dir, "params", "default_view.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    start_rviz_cmd = Node(
        condition=UnlessCondition(use_namespace),
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    namespaced_rviz_config_file = ReplaceString(
        source_file=rviz_config_file,
        replacements={"<robot_namespace>": ("/", namespace)},
    )

    start_namespaced_rviz_cmd = Node(
        condition=IfCondition(use_namespace),
        package="rviz2",
        executable="rviz2",
        namespace=namespace,
        arguments=["-d", namespaced_rviz_config_file],
        output="screen",
        remappings=[
            ("/map", "map"),
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("/goal_pose", "goal_pose"),
            ("/clicked_point", "clicked_point"),
            ("/initialpose", "initialpose"),
        ],
    )

    exit_event_handler = RegisterEventHandler(
        condition=UnlessCondition(use_namespace),
        event_handler=OnProcessExit(
            target_action=start_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason="rviz exited")),
        ),
    )

    exit_event_handler_namespaced = RegisterEventHandler(
        condition=IfCondition(use_namespace),
        event_handler=OnProcessExit(
            target_action=start_namespaced_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason="rviz exited")),
        ),
    )

    start_robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": description}, {"use_sim_time": False}],
    )

    start_joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": False}],
    )

    start_joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    ld.add_action(start_rviz_cmd)
    ld.add_action(start_namespaced_rviz_cmd)
    ld.add_action(start_joint_state_publisher_node)
    ld.add_action(start_robot_state_publisher_node)
    ld.add_action(start_joy_node)

    ld.add_action(exit_event_handler)
    ld.add_action(exit_event_handler_namespaced)

    return ld
