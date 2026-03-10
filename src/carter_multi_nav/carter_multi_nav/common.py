import math


DEFAULT_ROBOTS = ("carter1", "carter2", "carter3")
DEFAULT_ROOT_POSES = {
    "carter1": (0.0, 0.0, 0.0, math.pi),
    "carter2": (4.0, -1.0, 0.0, math.pi),
    "carter3": (7.0, 3.0, 0.0, math.pi),
}

ROOT_FRAME = "global_odom"
ODOM_FRAME = "odom"
MAP_FRAME = "map"
BASE_FRAME = "base_link"
BASE_FOOTPRINT_FRAME = "base_footprint"
LIDAR_FRAME = "front_2d_lidar"

LIDAR_TRANSLATION = (0.026, 0.0, 0.418)
LIDAR_RPY = (0.0, 0.0, math.pi)

FOOTPRINT_POLYGON = [
    [0.43, 0.30],
    [0.43, -0.30],
    [-0.43, -0.30],
    [-0.43, 0.30],
]


def parse_pose_csv(raw_value: str, robot_name: str):
    text = str(raw_value or "").strip()
    if not text:
        return DEFAULT_ROOT_POSES[robot_name]

    parts = [part.strip() for part in text.split(",") if part.strip()]
    if len(parts) != 4:
        raise ValueError(
            f"Pose for {robot_name} must be 'x,y,z,yaw', got '{raw_value}'"
        )
    return tuple(float(part) for part in parts)


def prefixed_frame(robot_name: str, frame_id: str) -> str:
    frame = str(frame_id or "").lstrip("/")
    if not frame:
        return frame
    if frame == ROOT_FRAME:
        return frame
    if frame.startswith(f"{robot_name}/"):
        return frame
    return f"{robot_name}/{frame}"
