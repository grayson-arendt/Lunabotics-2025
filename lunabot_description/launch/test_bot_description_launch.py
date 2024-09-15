from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    rviz_config_path = os.path.join(
        get_package_share_path("lunabot_description"),
        "rviz_config",
        "test_bot_config.rviz",
    )

    urdf_path = os.path.join(
        get_package_share_path("lunabot_description"), "urdf", "sim_bot.xacro"
    )

    #world_path = os.path.join(
    #    get_package_share_path("lunabot_description"), "worlds", "battlebots.world"
    #)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_path("gazebo_ros"), "launch", "gazebo.launch.py"
            )
        ),
    )
    #launch_arguments={"world": world_path}.items(),

    robot_description = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "sim_bot"],
        output="screen",
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": True}],
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
    )

    teleop_node = Node(
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
    )

    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
        name="static_transform_publisher",
    )


    return LaunchDescription(
        [
            gazebo,
            robot_state_publisher_node,
            joint_state_publisher_node,
            spawn_entity,
            rviz2_node,
            joy_node,
            teleop_node,
            map_to_odom_tf
        ]
    )
