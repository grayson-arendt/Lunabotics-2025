import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    config_dir = get_package_share_directory("lunabot_config")
    use_sim = LaunchConfiguration("use_sim").perform(context)
    
    if use_sim.lower() == "true":
        fast_lio_config = os.path.join(
            config_dir, "params", "fast_lio", "mid360_sim.yaml"
        )
    else:
        fast_lio_config = os.path.join(
            config_dir, "params", "fast_lio", "mid360_real.yaml"
        )

    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        output="screen",
        parameters=[
            fast_lio_config,
            {
                "use_sim_time": LaunchConfiguration("use_sim"),
            },
        ],
        remappings=[
            ("/Odometry", "/lio_odom"),
        ],
    )

    return [
        fast_lio_node,
    ]


def generate_launch_description():
    declare_use_sim = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Whether we are in simulation or not",
    )

    return LaunchDescription([
        declare_use_sim,
        OpaqueFunction(function=launch_setup),
    ])
