import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def _is_true(raw_value: str) -> bool:
    return str(raw_value or "").strip().lower() in {"1", "true", "yes", "on"}


def _launch_setup(context, *_args, **_kwargs):
    namespace = LaunchConfiguration("namespace").perform(context).strip()
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_time_bool = _is_true(use_sim_time)
    params_file = LaunchConfiguration("params_file").perform(context)
    autostart = LaunchConfiguration("autostart").perform(context)
    use_respawn = _is_true(LaunchConfiguration("use_respawn").perform(context))
    log_level = LaunchConfiguration("log_level").perform(context)
    lifecycle_ready_timeout = LaunchConfiguration("lifecycle_ready_timeout").perform(
        context
    )
    lifecycle_ready_timeout_value = float(lifecycle_ready_timeout)
    nav_target_linear_speed = float(
        LaunchConfiguration("nav_target_linear_speed").perform(context)
    )
    lookahead_dist = max(1.0, nav_target_linear_speed * 2.5)
    min_lookahead_dist = max(0.5, nav_target_linear_speed * 1.25)
    max_lookahead_dist = max(2.0, nav_target_linear_speed * 5.0)

    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
        ("odom", "chassis/odom"),
        # Force costmap sensor subscriptions onto the robot-level topics rather
        # than nested local_costmap/global_costmap relative names.
        ("scan_filtered", ["/", namespace, "/scan_filtered"]),
        ("map", ["/", namespace, "/planning_map"]),
        ("planning_map", ["/", namespace, "/planning_map"]),
    ]
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "desired_linear_vel": f"{nav_target_linear_speed:.3f}",
                "lookahead_dist": f"{lookahead_dist:.3f}",
                "min_lookahead_dist": f"{min_lookahead_dist:.3f}",
                "max_lookahead_dist": f"{max_lookahead_dist:.3f}",
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

    lifecycle_bringup_gate = Node(
        package="carter_multi_nav",
        executable="lifecycle_bringup_gate",
        name="nav2_bringup_gate",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time_bool,
                "managed_nodes": lifecycle_nodes,
                "timeout": lifecycle_ready_timeout_value,
            }
        ],
    )

    return [
        GroupAction(
            actions=[
                SetParameter("use_sim_time", use_sim_time_bool),
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
                lifecycle_bringup_gate,
            ],
        )
    ]


def generate_launch_description():
    package_dir = get_package_share_directory("carter_multi_nav")
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
            DeclareLaunchArgument("lifecycle_ready_timeout", default_value="45.0"),
            DeclareLaunchArgument("nav_target_linear_speed", default_value="0.80"),
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
