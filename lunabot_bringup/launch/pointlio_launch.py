import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    config_dir = get_package_share_directory("lunabot_config")
    use_sim = LaunchConfiguration("use_sim").perform(context)
    
    if use_sim.lower() == "true":
        point_lio_config = os.path.join(
            config_dir, "params", "point_lio", "mid360_sim.yaml"
        )
    else:
        point_lio_config = os.path.join(
            config_dir, "params", "point_lio", "mid360_real.yaml"
        )

    ukf_params_file = os.path.join(
        config_dir, "params", "robot_localization", "ukf_params.yaml"
    )

    point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="laserMapping",
        output="screen",
        parameters=[
            point_lio_config,
            {
                "use_sim_time": LaunchConfiguration("use_sim"),
            },
        ],
        remappings=[
            ("/aft_mapped_to_init", "/lio_odom"),
        ],
    )

    ukf_node = Node(
        package="robot_localization",
        executable="ukf_node",
        name="ukf_filter_node",
        output="screen",
        parameters=[
            ukf_params_file,
            {"use_sim_time": LaunchConfiguration("use_sim")},
        ],
    )

    return [
        point_lio_node,
        #ukf_node,
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
