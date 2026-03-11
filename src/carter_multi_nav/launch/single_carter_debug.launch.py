import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from carter_multi_nav.common import DEFAULT_ROOT_POSES, parse_pose_csv


def _is_true(raw_value: str) -> bool:
    return str(raw_value or "").strip().lower() in {"1", "true", "yes", "on"}


def _launch_setup(context, *_args, **_kwargs):
    package_dir = get_package_share_directory("carter_multi_nav")
    robot_stack_launch = os.path.join(package_dir, "launch", "robot_stack.launch.py")
    rviz_config = os.path.join(package_dir, "rviz", "shared_view.rviz")

    robot_name = LaunchConfiguration("robot_name").perform(context).strip()
    if robot_name not in DEFAULT_ROOT_POSES:
        raise RuntimeError(
            "Unsupported robot '%s'. Expected one of: %s"
            % (robot_name, ", ".join(DEFAULT_ROOT_POSES.keys()))
        )

    root_x, root_y, root_z, root_yaw = parse_pose_csv(
        LaunchConfiguration("robot_pose").perform(context),
        robot_name,
    )
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    autostart = LaunchConfiguration("autostart").perform(context)
    slam_params_file = LaunchConfiguration("slam_params_file").perform(context)
    nav2_params_file = LaunchConfiguration("nav2_params_file").perform(context)
    laser_filter_file = LaunchConfiguration("laser_filter_file").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)
    wait_for_nav_ready = LaunchConfiguration("wait_for_nav_ready").perform(context)
    nav_ready_timeout = LaunchConfiguration("nav_ready_timeout").perform(context)

    actions = [
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
            }.items(),
        ),
        Node(
            package="carter_multi_nav",
            executable="tf_aggregate_relay",
            name="tf_aggregate_relay",
            output="screen",
            parameters=[
                {
                    "use_sim_time": _is_true(use_sim_time),
                    "robot_names": [robot_name],
                    "shared_map_source": robot_name,
                    "shared_map_source_topic": "/shared_map_source",
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
                    "robot_names": [robot_name],
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
                    "robot_names": [robot_name],
                    "preferred_source": robot_name,
                    "output_topic": "/shared_map",
                    "source_name_topic": "/shared_map_source",
                }
            ],
        ),
    ]

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
            DeclareLaunchArgument("robot_name", default_value="carter1"),
            DeclareLaunchArgument("robot_pose", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("wait_for_nav_ready", default_value="true"),
            DeclareLaunchArgument("nav_ready_timeout", default_value="30.0"),
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
