import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_dir = get_package_share_directory("carter_multi_nav")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static"), ("odom", "chassis/odom")]
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={
                "use_sim_time": use_sim_time,
                "autostart": autostart,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(package_dir, "config", "nav2_carter_params.yaml"),
            ),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_respawn", default_value="false"),
            DeclareLaunchArgument("log_level", default_value="info"),
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            GroupAction(
                actions=[
                    SetParameter("use_sim_time", use_sim_time),
                    Node(
                        package="nav2_controller",
                        executable="controller_server",
                        output="screen",
                        respawn=use_respawn,
                        respawn_delay=2.0,
                        namespace=namespace,
                        parameters=[configured_params],
                        arguments=["--ros-args", "--log-level", log_level],
                        remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
                    ),
                    Node(
                        package="nav2_smoother",
                        executable="smoother_server",
                        name="smoother_server",
                        namespace=namespace,
                        output="screen",
                        respawn=use_respawn,
                        respawn_delay=2.0,
                        parameters=[configured_params],
                        arguments=["--ros-args", "--log-level", log_level],
                        remappings=remappings,
                    ),
                    Node(
                        package="nav2_planner",
                        executable="planner_server",
                        name="planner_server",
                        namespace=namespace,
                        output="screen",
                        respawn=use_respawn,
                        respawn_delay=2.0,
                        parameters=[configured_params],
                        arguments=["--ros-args", "--log-level", log_level],
                        remappings=remappings,
                    ),
                    Node(
                        package="nav2_behaviors",
                        executable="behavior_server",
                        name="behavior_server",
                        namespace=namespace,
                        output="screen",
                        respawn=use_respawn,
                        respawn_delay=2.0,
                        parameters=[configured_params],
                        arguments=["--ros-args", "--log-level", log_level],
                        remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
                    ),
                    Node(
                        package="nav2_bt_navigator",
                        executable="bt_navigator",
                        name="bt_navigator",
                        namespace=namespace,
                        output="screen",
                        respawn=use_respawn,
                        respawn_delay=2.0,
                        parameters=[configured_params],
                        arguments=["--ros-args", "--log-level", log_level],
                        remappings=remappings,
                    ),
                    Node(
                        package="nav2_waypoint_follower",
                        executable="waypoint_follower",
                        name="waypoint_follower",
                        namespace=namespace,
                        output="screen",
                        respawn=use_respawn,
                        respawn_delay=2.0,
                        parameters=[configured_params],
                        arguments=["--ros-args", "--log-level", log_level],
                        remappings=remappings,
                    ),
                    Node(
                        package="nav2_velocity_smoother",
                        executable="velocity_smoother",
                        name="velocity_smoother",
                        namespace=namespace,
                        output="screen",
                        respawn=use_respawn,
                        respawn_delay=2.0,
                        parameters=[configured_params],
                        arguments=["--ros-args", "--log-level", log_level],
                        remappings=remappings
                        + [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
                    ),
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_navigation",
                        namespace=namespace,
                        output="screen",
                        arguments=["--ros-args", "--log-level", log_level],
                        parameters=[
                            {"use_sim_time": use_sim_time},
                            {"autostart": autostart},
                            {"node_names": lifecycle_nodes},
                        ],
                    ),
                ],
            ),
        ]
    )
