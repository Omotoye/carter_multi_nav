import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from carter_multi_nav.common import DEFAULT_ROOT_POSES, DEFAULT_ROBOTS, parse_pose_csv


def _pose_default(robot_name: str) -> str:
    x, y, z, yaw = DEFAULT_ROOT_POSES[robot_name]
    return f"{x},{y},{z},{yaw}"


def _parse_robot_names(raw_value: str):
    robots = [part.strip() for part in str(raw_value or "").split(",") if part.strip()]
    return robots or list(DEFAULT_ROBOTS)


def _is_true(raw_value: str) -> bool:
    return str(raw_value or "").strip().lower() in {"1", "true", "yes", "on"}


def _serialize_root_poses(robot_root_poses):
    return ";".join(
        f"{robot_name}={x},{y},{z},{yaw}"
        for robot_name, (x, y, z, yaw) in robot_root_poses.items()
    )


def _launch_setup(context, *_args, **_kwargs):
    package_dir = get_package_share_directory("carter_multi_nav")
    robot_stack_launch = os.path.join(package_dir, "launch", "robot_stack.launch.py")
    rviz_config = os.path.join(package_dir, "rviz", "shared_view.rviz")

    robot_names = _parse_robot_names(LaunchConfiguration("robots").perform(context))
    unsupported = [robot for robot in robot_names if robot not in DEFAULT_ROOT_POSES]
    if unsupported:
        raise RuntimeError(
            "Unsupported robot names: %s. This workspace is configured for %s."
            % (", ".join(unsupported), ", ".join(DEFAULT_ROBOTS))
        )
    robot_root_poses = {
        robot_name: parse_pose_csv(
            LaunchConfiguration(f"{robot_name}_pose").perform(context),
            robot_name,
        )
        for robot_name in robot_names
    }
    serialized_root_poses = _serialize_root_poses(robot_root_poses)

    actions = []

    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    autostart = LaunchConfiguration("autostart").perform(context)
    slam_params_file = LaunchConfiguration("slam_params_file").perform(context)
    nav2_params_file = LaunchConfiguration("nav2_params_file").perform(context)
    laser_filter_file = LaunchConfiguration("laser_filter_file").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)
    wait_for_nav_ready = LaunchConfiguration("wait_for_nav_ready").perform(context)
    nav_ready_timeout = LaunchConfiguration("nav_ready_timeout").perform(context)
    scan_gate_max_rotation_per_scan_deg = LaunchConfiguration(
        "scan_gate_max_rotation_per_scan_deg"
    ).perform(context)
    scan_gate_max_angular_velocity = LaunchConfiguration(
        "scan_gate_max_angular_velocity"
    ).perform(context)
    scan_gate_holdoff_after_rotation = LaunchConfiguration(
        "scan_gate_holdoff_after_rotation"
    ).perform(context)
    slam_peer_exclusion_enabled = LaunchConfiguration(
        "slam_peer_exclusion_enabled"
    ).perform(context)
    slam_share_localized_scans = LaunchConfiguration(
        "slam_share_localized_scans"
    ).perform(context)
    shared_map_source = LaunchConfiguration("shared_map_source").perform(context).strip()
    if shared_map_source not in robot_names:
        shared_map_source = robot_names[0]
    peer_exclusion_margin = LaunchConfiguration("peer_exclusion_margin").perform(context)
    nav_target_linear_speed = LaunchConfiguration("nav_target_linear_speed").perform(
        context
    )
    map_tf_smoothing_enabled = LaunchConfiguration(
        "map_tf_smoothing_enabled"
    ).perform(context)
    map_tf_smoothing_alpha = LaunchConfiguration("map_tf_smoothing_alpha").perform(
        context
    )
    map_tf_max_translation_jump = LaunchConfiguration(
        "map_tf_max_translation_jump"
    ).perform(context)
    map_tf_max_rotation_jump = LaunchConfiguration(
        "map_tf_max_rotation_jump"
    ).perform(context)

    for robot_name in robot_names:
        root_x, root_y, root_z, root_yaw = robot_root_poses[robot_name]
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_stack_launch),
                launch_arguments={
                    "robot_name": robot_name,
                    "use_sim_time": use_sim_time,
                    "root_x": str(root_x),
                    "root_y": str(root_y),
                    "root_z": str(root_z),
                    "root_yaw": str(root_yaw),
                    "slam_params_file": slam_params_file,
                    "nav2_params_file": nav2_params_file,
                    "laser_filter_file": laser_filter_file,
                    "autostart": autostart,
                    "log_level": log_level,
                    "wait_for_nav_ready": wait_for_nav_ready,
                    "nav_ready_timeout": nav_ready_timeout,
                    "all_robot_names": ",".join(robot_names),
                    "all_robot_root_poses": serialized_root_poses,
                    "scan_gate_max_rotation_per_scan_deg": scan_gate_max_rotation_per_scan_deg,
                    "scan_gate_max_angular_velocity": scan_gate_max_angular_velocity,
                    "scan_gate_holdoff_after_rotation": scan_gate_holdoff_after_rotation,
                    "slam_peer_exclusion_enabled": slam_peer_exclusion_enabled,
                    "slam_share_localized_scans": slam_share_localized_scans,
                    "peer_exclusion_margin": peer_exclusion_margin,
                    "nav_target_linear_speed": nav_target_linear_speed,
                }.items(),
            )
        )

    actions.extend(
        [
            Node(
                package="carter_multi_nav",
                executable="tf_aggregate_relay",
                name="tf_aggregate_relay",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": _is_true(use_sim_time),
                        "robot_names": robot_names,
                        "shared_map_source": shared_map_source,
                        "map_tf_smoothing_enabled": _is_true(map_tf_smoothing_enabled),
                        "map_tf_smoothing_alpha": float(map_tf_smoothing_alpha),
                        "map_tf_max_translation_jump": float(
                            map_tf_max_translation_jump
                        ),
                        "map_tf_max_rotation_jump": float(map_tf_max_rotation_jump),
                    }
                ],
            ),
            Node(
                package="carter_multi_nav",
                executable="map_selector",
                name="map_selector",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": _is_true(use_sim_time),
                        "robot_names": robot_names,
                        "preferred_source": shared_map_source,
                        "merge_publish_frequency": 1.0,
                    }
                ],
            ),
            Node(
                package="carter_multi_nav",
                executable="scan_relay",
                name="scan_relay",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": _is_true(use_sim_time),
                        "robot_names": robot_names,
                    }
                ],
            ),
            Node(
                package="carter_multi_nav",
                executable="multi_robot_viz",
                name="multi_robot_viz",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": _is_true(use_sim_time),
                        "robot_names": robot_names,
                        "enable_shared_map_merge": False,
                    }
                ],
            ),
        ]
    )

    if _is_true(LaunchConfiguration("rviz_goal_router").perform(context)):
        actions.append(
            Node(
                package="carter_multi_nav",
                executable="rviz_goal_router",
                name="rviz_goal_router",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": _is_true(use_sim_time),
                        "robot_names": robot_names,
                        "input_topic": "/goal_pose",
                        "action_name": "navigate_to_pose",
                    }
                ],
            )
        )

    if _is_true(LaunchConfiguration("rviz").perform(context)):
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": _is_true(use_sim_time)}],
            )
        )

    return actions


def generate_launch_description():
    package_dir = get_package_share_directory("carter_multi_nav")
    default_cyclonedds_uri = os.environ.get(
        "CYCLONEDDS_URI",
        f"file://{os.path.join(package_dir, 'config', 'cyclonedds_multi_robot.xml')}",
    )
    default_rmw_implementation = os.environ.get(
        "RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("robots", default_value="carter1,carter2,carter3"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rviz_goal_router", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("wait_for_nav_ready", default_value="true"),
            DeclareLaunchArgument("nav_ready_timeout", default_value="30.0"),
            DeclareLaunchArgument(
                "scan_gate_max_rotation_per_scan_deg", default_value="1.25"
            ),
            DeclareLaunchArgument("scan_gate_max_angular_velocity", default_value="0.25"),
            DeclareLaunchArgument("scan_gate_holdoff_after_rotation", default_value="0.40"),
            DeclareLaunchArgument("slam_peer_exclusion_enabled", default_value="false"),
            DeclareLaunchArgument("slam_share_localized_scans", default_value="true"),
            DeclareLaunchArgument("peer_exclusion_margin", default_value="0.10"),
            DeclareLaunchArgument("nav_target_linear_speed", default_value="0.80"),
            DeclareLaunchArgument("map_tf_smoothing_enabled", default_value="true"),
            DeclareLaunchArgument("map_tf_smoothing_alpha", default_value="0.40"),
            DeclareLaunchArgument("map_tf_max_translation_jump", default_value="0.05"),
            DeclareLaunchArgument("map_tf_max_rotation_jump", default_value="0.02"),
            DeclareLaunchArgument("shared_map_source", default_value="carter1"),
            DeclareLaunchArgument("carter1_pose", default_value=_pose_default("carter1")),
            DeclareLaunchArgument("carter2_pose", default_value=_pose_default("carter2")),
            DeclareLaunchArgument("carter3_pose", default_value=_pose_default("carter3")),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=os.path.join(package_dir, "config", "slam_toolbox_multi.yaml"),
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=os.path.join(package_dir, "config", "nav2_carter_params.yaml"),
            ),
            DeclareLaunchArgument(
                "laser_filter_file",
                default_value=os.path.join(package_dir, "config", "laser_box_filter.yaml"),
            ),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument(
                "cyclonedds_uri",
                default_value=default_cyclonedds_uri,
            ),
            DeclareLaunchArgument(
                "rmw_implementation",
                default_value=default_rmw_implementation,
            ),
            SetEnvironmentVariable("CYCLONEDDS_URI", LaunchConfiguration("cyclonedds_uri")),
            SetEnvironmentVariable(
                "RMW_IMPLEMENTATION", LaunchConfiguration("rmw_implementation")
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
