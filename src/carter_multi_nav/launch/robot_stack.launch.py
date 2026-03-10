import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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


def generate_launch_description():
    package_dir = get_package_share_directory("carter_multi_nav")
    slam_share_dir = get_package_share_directory("slam_toolbox")
    slam_launch_path = os.path.join(
        slam_share_dir, "launch", "online_async_decentralized_multirobot_launch.py"
    )
    if not os.path.exists(slam_launch_path):
        slam_launch_path = os.path.expanduser(
            "~/carter_nav_ws/src/slam_toolbox/launch/online_async_decentralized_multirobot_launch.py"
        )

    robot_name = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    root_x = LaunchConfiguration("root_x")
    root_y = LaunchConfiguration("root_y")
    root_z = LaunchConfiguration("root_z")
    root_yaw = LaunchConfiguration("root_yaw")
    slam_params_file = LaunchConfiguration("slam_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    laser_filter_file = LaunchConfiguration("laser_filter_file")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("root_x", default_value="0.0"),
            DeclareLaunchArgument("root_y", default_value="0.0"),
            DeclareLaunchArgument("root_z", default_value="0.0"),
            DeclareLaunchArgument("root_yaw", default_value="0.0"),
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
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            _static_transform_node(
                name="root_pose_tf",
                namespace=robot_name,
                use_sim_time=use_sim_time,
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
                use_sim_time=use_sim_time,
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
                use_sim_time=use_sim_time,
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
                parameters=[laser_filter_file, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("scan", "front_2d_lidar/scan"),
                    ("scan_filtered", "scan_filtered"),
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                ],
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(package_dir, "launch", "nav2_stack.launch.py")
                ),
                launch_arguments={
                    "namespace": robot_name,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": nav2_params_file,
                    "log_level": log_level,
                }.items(),
            ),
        ]
    )
