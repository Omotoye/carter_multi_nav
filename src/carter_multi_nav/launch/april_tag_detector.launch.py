from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_csv(raw_value: str):
    return [part.strip() for part in str(raw_value or "").split(",") if part.strip()]


def _is_true(raw_value: str) -> bool:
    return str(raw_value or "").strip().lower() in {"1", "true", "yes", "on"}


def _launch_setup(context, *_args, **_kwargs):
    robot_names = _parse_csv(LaunchConfiguration("robots").perform(context))
    image_topics = _parse_csv(LaunchConfiguration("image_topics").perform(context))
    parameters = {
        "use_sim_time": _is_true(LaunchConfiguration("use_sim_time").perform(context)),
        "robot_names": robot_names,
        "image_topic_suffix": LaunchConfiguration("image_topic_suffix").perform(context),
        "tag_family": LaunchConfiguration("tag_family").perform(context),
        "min_tag_id": int(LaunchConfiguration("min_tag_id").perform(context)),
        "max_tag_id": int(LaunchConfiguration("max_tag_id").perform(context)),
        "publish_topic": LaunchConfiguration("publish_topic").perform(context),
        "publish_by_robot_topic": LaunchConfiguration("publish_by_robot_topic").perform(
            context
        ),
        "publish_rate_hz": float(LaunchConfiguration("publish_rate_hz").perform(context)),
        "detection_hold_time": float(
            LaunchConfiguration("detection_hold_time").perform(context)
        ),
        "max_processing_fps": float(
            LaunchConfiguration("max_processing_fps").perform(context)
        ),
        "stale_image_timeout": float(
            LaunchConfiguration("stale_image_timeout").perform(context)
        ),
        "log_detection_changes": _is_true(
            LaunchConfiguration("log_detection_changes").perform(context)
        ),
    }
    if image_topics:
        parameters["image_topics"] = image_topics

    return [
        Node(
            package="carter_multi_nav",
            executable="april_tag_detector",
            name="april_tag_detector",
            output="screen",
            parameters=[parameters],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("robots", default_value="carter1,carter2,carter3"),
            DeclareLaunchArgument("image_topics", default_value=""),
            DeclareLaunchArgument(
                "image_topic_suffix",
                default_value="front_stereo_camera/left/image_raw",
            ),
            DeclareLaunchArgument("tag_family", default_value="36h11"),
            DeclareLaunchArgument("min_tag_id", default_value="1"),
            DeclareLaunchArgument("max_tag_id", default_value="10"),
            DeclareLaunchArgument(
                "publish_topic", default_value="/april_tag/detected_ids"
            ),
            DeclareLaunchArgument(
                "publish_by_robot_topic",
                default_value="/april_tag/detections_by_robot",
            ),
            DeclareLaunchArgument("publish_rate_hz", default_value="5.0"),
            DeclareLaunchArgument("detection_hold_time", default_value="1.0"),
            DeclareLaunchArgument("max_processing_fps", default_value="6.0"),
            DeclareLaunchArgument("stale_image_timeout", default_value="1.0"),
            DeclareLaunchArgument("log_detection_changes", default_value="true"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
