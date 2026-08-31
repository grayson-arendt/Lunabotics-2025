import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, EqualsSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
    OpaqueFunction,
    GroupAction,
)


def set_orientation(context, *args, **kwargs):
    orientations = {"north": -1.5708, "east": 3.1416, "south": 1.5708, "west": 0.0}
    chosen_orientation = context.launch_configurations.get("robot_heading")
    return [SetLaunchConfiguration("robot_heading", str(orientations.get(chosen_orientation, 0.0)))]


def set_spawn_coordinates(context, *args, **kwargs):
    world_coords = {
        "ucf": {"x": "1.0", "y": "-3.0", "z": "0.35"},
        "artemis": {"x": "2.5", "y": "1.6", "z": "0.35"},
    }
    world_type = context.launch_configurations.get("arena_type")
    coords = world_coords.get(world_type, world_coords["artemis"])
    return [
        SetLaunchConfiguration("spawn_x", coords["x"]),
        SetLaunchConfiguration("spawn_y", coords["y"]),
        SetLaunchConfiguration("spawn_z", coords["z"]),
    ]


def set_world_file(context, *args, **kwargs):
    sim_dir = get_package_share_directory("lunabot_sim")
    world_type = context.launch_configurations.get("arena_type")
    world_files = {
        "ucf": os.path.join(sim_dir, "worlds", "high_resolution", "ucf", "ucf_arena.world"),
        "artemis": os.path.join(sim_dir, "worlds", "high_resolution", "artemis", "artemis_arena.world"),
    }
    return [SetLaunchConfiguration("world_file", world_files.get(world_type, world_files["artemis"]))]


def set_robot_urdf(context, *args, **kwargs):
    description_dir = get_package_share_directory("lunabot_description")
    robot_type = context.launch_configurations.get("robot_type")
    urdf_file = os.path.join(description_dir, "urdf", f"{robot_type}.urdf.xacro")
    return [SetLaunchConfiguration("urdf_file", urdf_file)]


def set_robot_entity_name(context, *args, **kwargs):
    robot_type = context.launch_configurations.get("robot_type")
    return [SetLaunchConfiguration("robot_entity", robot_type)]


def create_sim_group(context, *args, **kwargs):
    if context.launch_configurations.get("use_sim") != "true":
        return []

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
            )
        ),
        launch_arguments={
            "gui": LaunchConfiguration("sim_gui"),
            "world": LaunchConfiguration("world_file"),
        }.items(),
    )

    spawn_robot_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", LaunchConfiguration("robot_entity"),
            "-x", LaunchConfiguration("spawn_x"),
            "-y", LaunchConfiguration("spawn_y"),
            "-Y", LaunchConfiguration("robot_heading"),
            "-z", LaunchConfiguration("spawn_z"),
        ],
        output="screen",
    )

    topic_remapper_node = Node(
        package="lunabot_util",
        executable="topic_remapper",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    camera_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["camera_controller", "--controller-manager", "/controller_manager"],
    )

    actions_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lunabot_bringup"), "launch", "actions_launch.py"
            )
        ),
        launch_arguments={"use_sim": "true"}.items(),
    )

    actions = [
        sim_launch,
        spawn_robot_node,
        topic_remapper_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        camera_controller_spawner,
        actions_launch,
    ]

    if context.launch_configurations.get("robot_type") == "v2_bot":
        actions.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=["position_controller", "--controller-manager", "/controller_manager"],
        ))

    return actions


def generate_launch_description():
    config_dir = get_package_share_directory("lunabot_config")
    description_dir = get_package_share_directory("lunabot_description")

    rviz_config_file = os.path.join(config_dir, "rviz", "robot_view.rviz")
    gui_params_file = os.path.join(config_dir, "params", "gui", "gui_params.yaml")

    declare_robot_type = DeclareLaunchArgument(
        "robot_type",
        default_value="v2_bot",
        choices=["v1_bot", "v2_bot"],
        description="Which robot model to use (v1_bot or v2_bot)",
    )

    declare_use_sim = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Specifies whether the robot is in simulation mode 'true' or real-world mode 'false'.",
    )

    declare_robot_heading = DeclareLaunchArgument(
        "robot_heading",
        default_value="north",
        choices=["north", "east", "south", "west"],
        description="Sets the starting orientation of the robot. Choose a cardinal direction.",
    )

    declare_sim_gui = DeclareLaunchArgument(
        "sim_gui",
        default_value="true",
        choices=["true", "false"],
        description="Sets whether to open Gazebo with its GUI.",
    )

    declare_arena_type = DeclareLaunchArgument(
        "arena_type",
        default_value="artemis",
        choices=["ucf", "artemis"],
        description="Choose the arena: 'ucf' for UCF arena or 'artemis' for Artemis arena.",
    )

    declare_viz_mode = DeclareLaunchArgument(
        "viz_mode",
        default_value="gui",
        choices=["rviz", "gui"],
        description="Choose visualization mode: 'rviz' for RViz2 or 'gui' for custom PyQt GUI.",
    )

    rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    custom_gui_node = Node(
        package="lunabot_gui",
        executable="lunabot_gui",
        name="lunabot_gui",
        parameters=[
            gui_params_file,
            {'mode': LaunchConfiguration('use_sim', default='true')},
        ],
        output="screen",
    )

    bandwidth_monitor_node = Node(
        package="lunabot_util",
        executable="bandwidth_monitor.py",
        name="bandwidth_monitor",
        output="screen",
        parameters=[
            gui_params_file,
            {'interface': 'wlo1'},
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(
                    ["xacro ", LaunchConfiguration("urdf_file"), " use_sim:=", LaunchConfiguration("use_sim")]
                ),
                "use_sim_time": LaunchConfiguration("use_sim"),
            }
        ],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": False}],
    )

    image_compressor_node = Node(
        package="lunabot_util",
        executable="image_compressor.py",
        name="image_compressor",
        output="screen",
        parameters=[
            {"jpeg_quality": 25},
            {"scale": 1.0},
        ],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

    joint_state_publisher_real = GroupAction(
        actions=[joint_state_publisher_node],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("use_sim"), "false")),
    )

    joy_group = GroupAction(
        actions=[joy_node],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("use_sim"), "false")),
    )

    image_compressor_sim_node = Node(
        package="lunabot_util",
        executable="image_compressor.py",
        name="image_compressor",
        output="screen",
        parameters=[
            {"jpeg_quality": 45},
            {"scale": 1.0},
        ],
    )

    image_compressor_sim = GroupAction(
        actions=[image_compressor_sim_node],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("use_sim"), "true")),
    )

    rviz_group = GroupAction(
        actions=[rviz_launch],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("viz_mode"), "rviz")),
    )

    custom_gui_group = GroupAction(
        actions=[custom_gui_node],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("viz_mode"), "gui")),
    )

    ld = LaunchDescription()

    ld.add_action(declare_robot_type)
    ld.add_action(declare_use_sim)
    ld.add_action(declare_robot_heading)
    ld.add_action(declare_sim_gui)
    ld.add_action(declare_arena_type)
    ld.add_action(declare_viz_mode)

    ld.add_action(OpaqueFunction(function=set_robot_urdf))
    ld.add_action(OpaqueFunction(function=set_robot_entity_name))
    ld.add_action(OpaqueFunction(function=set_orientation))
    ld.add_action(OpaqueFunction(function=set_world_file))
    ld.add_action(OpaqueFunction(function=set_spawn_coordinates))

    ld.add_action(rviz_group)
    ld.add_action(custom_gui_group)
    ld.add_action(robot_state_publisher)
    ld.add_action(bandwidth_monitor_node)
    ld.add_action(joint_state_publisher_real)
    ld.add_action(joy_group)
    ld.add_action(image_compressor_sim)

    ld.add_action(OpaqueFunction(function=create_sim_group))

    return ld
