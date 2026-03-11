import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _static_transform_node(
    *,
    name,
    namespace,
    use_sim_time,
    parent_frame,
    child_frame,
    x,
    y,
    z,
    roll,
    pitch,
    yaw,
):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        namespace=namespace,
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "--x",
            x,
            "--y",
            y,
            "--z",
            z,
            "--roll",
            roll,
            "--pitch",
            pitch,
            "--yaw",
            yaw,
            "--frame-id",
            parent_frame,
            "--child-frame-id",
            child_frame,
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )


def _is_true(raw_value: str) -> bool:
    return str(raw_value or "").strip().lower() in {"1", "true", "yes", "on"}


def _launch_setup(context, *_args, **_kwargs):
    package_dir = get_package_share_directory("carter_multi_nav")
    slam_share_dir = get_package_share_directory("slam_toolbox")
    slam_launch_path = os.path.join(
        slam_share_dir, "launch", "online_async_decentralized_multirobot_launch.py"
    )
    if not os.path.exists(slam_launch_path):
        slam_launch_path = os.path.expanduser(
            "~/carter_nav_ws/src/slam_toolbox/launch/online_async_decentralized_multirobot_launch.py"
        )

    robot_name = LaunchConfiguration("robot_name").perform(context).strip()
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    root_x = LaunchConfiguration("root_x").perform(context)
    root_y = LaunchConfiguration("root_y").perform(context)
    root_z = LaunchConfiguration("root_z").perform(context)
    root_yaw = LaunchConfiguration("root_yaw").perform(context)
    slam_params_file = LaunchConfiguration("slam_params_file").perform(context)
    nav2_params_file = LaunchConfiguration("nav2_params_file").perform(context)
    laser_filter_file = LaunchConfiguration("laser_filter_file").perform(context)
    autostart = LaunchConfiguration("autostart").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)
    wait_for_nav_ready = LaunchConfiguration("wait_for_nav_ready").perform(context)
    nav_ready_timeout = LaunchConfiguration("nav_ready_timeout").perform(context)
    use_sim_time_bool = _is_true(use_sim_time)

    nav2_launch_arguments = {
        "namespace": robot_name,
        "use_sim_time": use_sim_time,
        "autostart": autostart,
        "params_file": nav2_params_file,
        "log_level": log_level,
    }

    def _launch_nav2_after_gate(event, _context):
        if getattr(event, "returncode", 1) == 0:
            return [
                LogInfo(msg=f"Nav readiness gate passed for {robot_name}; starting Nav2."),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(package_dir, "launch", "nav2_stack.launch.py")
                    ),
                    launch_arguments=dict(nav2_launch_arguments).items(),
                ),
            ]

        return [
            LogInfo(
                msg=(
                    f"Nav readiness gate failed for {robot_name}; "
                    "Nav2 will not be started for this robot."
                )
            )
        ]

    actions = [
        _static_transform_node(
            name="root_pose_tf",
            namespace=robot_name,
            use_sim_time=use_sim_time_bool,
            parent_frame="global_odom",
            child_frame="odom",
            x=root_x,
            y=root_y,
            z=root_z,
            roll="0.0",
            pitch="0.0",
            yaw=root_yaw,
        ),
        _static_transform_node(
            name="base_footprint_tf",
            namespace=robot_name,
            use_sim_time=use_sim_time_bool,
            parent_frame="base_link",
            child_frame="base_footprint",
            x="0.0",
            y="0.0",
            z="0.0",
            roll="0.0",
            pitch="0.0",
            yaw="0.0",
        ),
        _static_transform_node(
            name="front_lidar_tf",
            namespace=robot_name,
            use_sim_time=use_sim_time_bool,
            parent_frame="base_link",
            child_frame="front_2d_lidar",
            x="0.026",
            y="0.0",
            z="0.418",
            roll="0.0",
            pitch="0.0",
            yaw="3.141592653589793",
        ),
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            name="scan_to_scan_filter_chain",
            namespace=robot_name,
            output="screen",
            parameters=[laser_filter_file, {"use_sim_time": use_sim_time_bool}],
            remappings=[
                ("scan", "front_2d_lidar/scan"),
                ("scan_filtered", "scan_filtered"),
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
            ],
        ),
        Node(
            package="carter_multi_nav",
            executable="scan_motion_gate",
            name="scan_motion_gate",
            namespace=robot_name,
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time_bool,
                    "scan_in": "scan_filtered",
                    "scan_out": "scan_motion_safe",
                    "odom_topic": "chassis/odom",
                    "max_rotation_per_scan_deg": 1.25,
                    "max_angular_velocity": 0.25,
                    "holdoff_after_rotation": 0.40,
                }
            ],
        ),
        Node(
            package="carter_multi_nav",
            executable="planning_map_clearer",
            name="planning_map_clearer",
            namespace=robot_name,
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time_bool,
                    "input_topic": "map",
                    "output_topic": "planning_map",
                    "map_frame": "map",
                    "base_frame": "base_footprint",
                    "clear_radius": 0.60,
                }
            ],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                "namespace": robot_name,
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "slam_params_file": slam_params_file,
            }.items(),
        ),
    ]

    if _is_true(wait_for_nav_ready):
        nav_ready_gate = Node(
            package="carter_multi_nav",
            executable="nav_ready_gate",
            name="nav_ready_gate",
            namespace=robot_name,
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time_bool,
                    "map_topic": "planning_map",
                    "scan_topic": "scan_filtered",
                    "odom_topic": "chassis/odom",
                    "map_frame": "map",
                    "base_frame": "base_footprint",
                    "timeout": float(nav_ready_timeout),
                }
            ],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        )
        actions.extend(
            [
                nav_ready_gate,
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=nav_ready_gate,
                        on_exit=_launch_nav2_after_gate,
                    ),
                ),
            ]
        )
    else:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(package_dir, "launch", "nav2_stack.launch.py")
                ),
                launch_arguments=nav2_launch_arguments.items(),
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("root_x", default_value="0.0"),
            DeclareLaunchArgument("root_y", default_value="0.0"),
            DeclareLaunchArgument("root_z", default_value="0.0"),
            DeclareLaunchArgument("root_yaw", default_value="0.0"),
            DeclareLaunchArgument("slam_params_file"),
            DeclareLaunchArgument("nav2_params_file"),
            DeclareLaunchArgument("laser_filter_file"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("wait_for_nav_ready", default_value="true"),
            DeclareLaunchArgument("nav_ready_timeout", default_value="30.0"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
