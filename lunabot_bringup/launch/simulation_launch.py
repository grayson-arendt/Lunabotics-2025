import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    rviz_config_path = os.path.join(
        get_package_share_path("lunabot_bringup"),
        "config",
        "robot_view.rviz",
    )

    urdf_path = os.path.join(
        get_package_share_path("lunabot_description"), "urdf", "sim_bot.xacro"
    )

    world_path = os.path.join(
        get_package_share_path("lunabot_description"), "worlds", "moon.world"
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
        condition=LaunchConfigurationEquals("control_method", "xbox")
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
        remappings=[
            ("/cmd_vel", "/diff_drive_controller/cmd_vel_unstamped")
        ],
        condition=LaunchConfigurationEquals("control_method", "xbox")
    )

    keyboard_teleop_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'bash', '-c', 'ros2 run lunabot_autonomous keyboard_teleop.py;  exec bash'],
        condition=LaunchConfigurationEquals("control_method", "keyboard"),
        output='screen'
    )

    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
        name="static_transform_publisher",
    )

    blade_joint_controller_node = Node(
        package="lunabot_autonomous", executable="blade_joint_controller"
    )

    topic_remap_node = Node(
        package="lunabot_autonomous", executable="topic_remap"
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
        
    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            diff_drive_controller_spawner,
            position_controller_spawner,
            gazebo_launch,
            rviz2_node,
            blade_joint_controller_node,
            robot_state_publisher_node,
            spawn_entity_node,
            joy_node,
            joy_teleop_node,
            keyboard_teleop_node,
            topic_remap_node,
            map_to_odom_tf,
        ]
    )
