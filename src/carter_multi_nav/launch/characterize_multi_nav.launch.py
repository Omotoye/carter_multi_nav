import os
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from carter_multi_nav.common import DEFAULT_ROOT_POSES, DEFAULT_ROBOTS


def _parse_robot_names(raw_value: str):
    robots = [part.strip() for part in str(raw_value or "").split(",") if part.strip()]
    return robots or list(DEFAULT_ROBOTS)


def _root_pose_entries(robot_names):
    entries = []
    for robot_name in robot_names:
        x, y, z, yaw = DEFAULT_ROOT_POSES[robot_name]
        entries.append(f"{robot_name}={x},{y},{z},{yaw}")
    return entries


def _launch_setup(context, *_args, **_kwargs):
    package_dir = get_package_share_directory("carter_multi_nav")
    workspace_root = str(Path(package_dir).resolve().parents[3])

    robot_names = _parse_robot_names(LaunchConfiguration("robots").perform(context))
    run_id = LaunchConfiguration("run_id").perform(context).strip()
    if not run_id:
        run_id = datetime.now().strftime("%Y%m%d_%H%M%S")

    output_root = LaunchConfiguration("output_root").perform(context).strip()
    if not output_root:
        output_root = os.path.join(workspace_root, "log", "characterization")
    output_root = os.path.abspath(os.path.expanduser(output_root))
    output_dir = os.path.join(output_root, run_id)
    os.makedirs(output_dir, exist_ok=True)

    actions = [
        LogInfo(msg=f"Characterization output directory: {output_dir}"),
        Node(
            package="carter_multi_nav",
            executable="multi_robot_map_metrics",
            name="multi_robot_map_metrics",
            output="screen",
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time").perform(context).strip().lower()
                    in {"1", "true", "yes", "on"},
                    "robot_names": robot_names,
                    "robot_root_poses": _root_pose_entries(robot_names),
                    "report_interval": float(
                        LaunchConfiguration("report_interval").perform(context)
                    ),
                    "output_dir": output_dir,
                }
            ],
        ),
    ]

    for robot_name in robot_names:
        actions.append(
            Node(
                package="carter_multi_nav",
                executable="scan_motion_diagnostics",
                name=f"{robot_name}_scan_motion_diagnostics",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time").perform(context).strip().lower()
                        in {"1", "true", "yes", "on"},
                        "robot_name": robot_name,
                        "report_interval": float(
                            LaunchConfiguration("report_interval").perform(context)
                        ),
                        "output_dir": output_dir,
                    }
                ],
            )
        )

    benchmark_goal_file = LaunchConfiguration("benchmark_goal_file").perform(context).strip()
    run_benchmark = (
        LaunchConfiguration("run_benchmark").perform(context).strip().lower()
        in {"1", "true", "yes", "on"}
    )
    if run_benchmark and benchmark_goal_file:
        actions.append(
            Node(
                package="carter_multi_nav",
                executable="nav_benchmark_runner",
                name="nav_benchmark_runner",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time").perform(context).strip().lower()
                        in {"1", "true", "yes", "on"},
                        "goal_file": benchmark_goal_file,
                        "scenario_names_csv": LaunchConfiguration("scenario_names_csv").perform(context),
                        "output_dir": output_dir,
                        "robot_names": robot_names,
                    }
                ],
            )
        )

    record_bag = (
        LaunchConfiguration("record_bag").perform(context).strip().lower()
        in {"1", "true", "yes", "on"}
    )
    if record_bag:
        topics = ["/tf", "/tf_static", "/localized_scan", "/clock"]
        for robot_name in robot_names:
            topics.extend(
                [
                    f"/{robot_name}/front_2d_lidar/scan",
                    f"/{robot_name}/scan_filtered",
                    f"/{robot_name}/scan_peer_filtered",
                    f"/{robot_name}/scan_motion_safe",
                    f"/{robot_name}/chassis/odom",
                    f"/{robot_name}/map",
                    f"/{robot_name}/planning_map",
                    f"/{robot_name}/cmd_vel",
                    f"/{robot_name}/plan",
                ]
            )
        actions.append(
            ExecuteProcess(
                cmd=["ros2", "bag", "record", "-o", os.path.join(output_dir, "bag")] + topics,
                output="screen",
            )
        )

    return actions


def generate_launch_description():
    package_dir = get_package_share_directory("carter_multi_nav")
    return LaunchDescription(
        [
            DeclareLaunchArgument("robots", default_value="carter1,carter2,carter3"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("run_id", default_value=""),
            DeclareLaunchArgument("output_root", default_value=""),
            DeclareLaunchArgument("report_interval", default_value="5.0"),
            DeclareLaunchArgument("record_bag", default_value="false"),
            DeclareLaunchArgument("run_benchmark", default_value="false"),
            DeclareLaunchArgument(
                "benchmark_goal_file",
                default_value=os.path.join(package_dir, "config", "nav_benchmark_goals.yaml"),
            ),
            DeclareLaunchArgument("scenario_names_csv", default_value=""),
            OpaqueFunction(function=_launch_setup),
        ]
    )
