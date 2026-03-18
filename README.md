# carter_multi_nav

Standalone ROS 2 Jazzy workspace for three Carter robots with:

- one namespaced Nav2 stack per robot
- one namespaced decentralized `slam_toolbox` instance per robot
- lidar self-filtering before SLAM and costmaps
- a shared RViz operator view for TF, scans, and a canonical shared map
- a vendored `slam_toolbox` source overlay with the local Jazzy compatibility fixes needed by this workspace

This repository is intended to be cloned directly as the workspace root at `~/carter_nav_ws`.

## Workspace Layout

```text
carter_nav_ws/
├── src/
│   ├── carter_multi_nav/
│   └── slam_toolbox/
├── build/         # ignored
├── install/       # ignored
└── log/           # ignored
```

## What This Workspace Launches

For each robot (`carter1`, `carter2`, `carter3`) the workspace launches:

- static TF for `global_odom -> odom`
- static TF for `base_link -> base_footprint`
- static TF for `base_link -> front_2d_lidar`
- `laser_filters` on `/<robot>/front_2d_lidar/scan -> /<robot>/scan_filtered`
- `scan_peer_exclusion` on `/<robot>/scan_filtered -> /<robot>/scan_peer_filtered`
- `planning_map_clearer` on `/shared_map -> /<robot>/planning_map`
- decentralized `slam_toolbox`
- a full Nav2 stack

Shared visualization utilities also launch:

- `tf_aggregate_relay` to publish a collision-free shared `/tf` and `/tf_static`
- `scan_relay` to republish RViz-friendly scan topics with prefixed frames
- `map_selector` to republish a canonical map source to `/shared_map`
- RViz with a preconfigured multi-robot view

## Supported Environment

- Ubuntu 24.04
- ROS 2 Jazzy
- live Carter robots already publishing:
  - `/<robot>/front_2d_lidar/scan`
  - `/<robot>/chassis/odom`
  - `/<robot>/tf`
  - `/clock`

## System Dependencies

Install the ROS packages used by this workspace:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-laser-filters \
  ros-jazzy-rviz2 \
  ros-jazzy-xacro
```

If `rosdep` has not been initialized on the machine yet:

```bash
sudo rosdep init
rosdep update
```

## Clone and Build

Clone the repository directly into `~/carter_nav_ws`:

```bash
git clone git@github.com:Omotoye/carter_multi_nav.git ~/carter_nav_ws
cd ~/carter_nav_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --allow-overriding slam_toolbox --packages-up-to carter_multi_nav
source install/setup.bash
```

After sourcing the workspace, the package hook exports the CycloneDDS settings used by
the launch files. Use `carter_ros2 ...` for CLI inspection and goal sending so the ROS 2
daemon is reset before graph queries and namespaced actions/topics show up consistently.

Why `--allow-overriding slam_toolbox` is required:

- this repo vendors `src/slam_toolbox`
- many Jazzy systems also have the binary `slam_toolbox` package installed
- the workspace must prefer the vendored source overlay

## Launch

Launch the full three-robot stack:

```bash
cd ~/carter_nav_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch carter_multi_nav multi_carter_mapping_nav.launch.py
```

Launch a single robot for debugging:

```bash
ros2 launch carter_multi_nav single_carter_debug.launch.py robot_name:=carter1
```

Useful launch arguments:

- `robots:=carter1,carter2,carter3`
- `shared_map_source:=carter1`
- `rviz:=true`
- `use_sim_time:=true`
- `scan_gate_max_rotation_per_scan_deg:=1.5`
- `scan_gate_max_angular_velocity:=0.30`
- `scan_gate_holdoff_after_rotation:=0.10`
- `carter1_pose:=0.0,0.0,0.0,3.141592653589793`
- `carter2_pose:=4.0,-1.0,0.0,3.141592653589793`
- `carter3_pose:=7.0,3.0,0.0,3.141592653589793`

## Default Robot Start Poses

These are the default `global_odom -> odom` start poses used by the launch files:

- `carter1`: `x=0.0, y=0.0, yaw=pi`
- `carter2`: `x=4.0, y=-1.0, yaw=pi`
- `carter3`: `x=7.0, y=3.0, yaw=pi`

These values matter when you choose initial navigation goals.

## How to Test Navigation

The current RViz setup is visualization-first. It does not yet provide a per-robot goal tool routing layer, so the cleanest way to test navigation is from the CLI using the namespaced Nav2 actions.

First, confirm the actions are available:

```bash
carter_ros2 action list | grep navigate_to_pose
```

Expected:

```text
/carter1/navigate_to_pose
/carter2/navigate_to_pose
/carter3/navigate_to_pose
```

Then send a goal to a specific robot.

Example for `carter1`:

```bash
carter_ros2 action send_goal /carter1/navigate_to_pose nav2_msgs/action/NavigateToPose \
'{
  pose: {
    header: {frame_id: map},
    pose: {
      position: {x: 1.0, y: 0.5, z: 0.0},
      orientation: {z: 0.0, w: 1.0}
    }
  }
}' --feedback
```

Example for `carter2`:

```bash
carter_ros2 action send_goal /carter2/navigate_to_pose nav2_msgs/action/NavigateToPose \
'{
  pose: {
    header: {frame_id: map},
    pose: {
      position: {x: 4.8, y: -0.4, z: 0.0},
      orientation: {z: 0.0, w: 1.0}
    }
  }
}' --feedback
```

Guidance:

- use `frame_id: map`
- start with short goals close to the robot's current pose
- choose free space visible on the map, not unknown space
- use `--feedback` so you can see whether planning and control are active

Useful checks while testing:

```bash
carter_ros2 action info /carter1/navigate_to_pose
carter_ros2 topic echo /carter1/cmd_vel --once
carter_ros2 topic echo /carter1/plan --once
```

## Scan Diagnostics

If mapping falls apart while the robot is moving or rotating, measure the scan stream before changing more SLAM parameters:

```bash
ros2 run carter_multi_nav scan_motion_diagnostics --ros-args \
  -p robot_name:=carter1 \
  -p use_sim_time:=true
```

This diagnostic reports:

- observed scan rate
- scan timestamp lag
- median and peak angular velocity from odometry
- median and peak degrees of robot rotation during a single scan
- whether the scan metadata is internally inconsistent

The most important number is `rotation per scan`. If that grows too large during turns, the lidar is being used faster than the platform can rotate cleanly, and Nav2 rotational limits need to be reduced further or the upstream scan stream needs motion compensation.

This workspace now also includes a scan gate for SLAM:

- `/<robot>/scan_filtered` remains the lidar self-filtered topic
- `/<robot>/scan_motion_safe` is the SLAM input topic after motion gating

The gate drops scans when angular motion is high enough to exceed the configured per-scan rotation threshold. This is a stabilization measure for mapping, not a substitute for proper lidar deskewing.

## Shared Map Behavior

`/shared_map` is a canonical shared map selected from one robot's namespaced map topic.

By default:

- `/shared_map` republishes `/carter1/map`

This is not a separate centralized map-merge node.

Each robot plans against its own `/<robot>/planning_map`, which is derived from `/shared_map`.
`planning_map_clearer` removes all robot footprints from that planning map using the shared
aggregated TF tree so peer robots do not remain baked in as occupied cells.

The scan paths are split on purpose:

- `/<robot>/scan_filtered` remains the local obstacle source
- `/<robot>/scan_peer_filtered` removes peer robot returns for shared-map global planning

## Key Topics

- `/<robot>/scan_filtered`
- `/<robot>/scan_peer_filtered`
- `/<robot>/planning_map`
- `/<robot>/map`
- `/<robot>/cmd_vel`
- `/<robot>/navigate_to_pose`
- `/shared_map`
- `/rviz/<robot>/front_2d_lidar/scan`
- `/rviz/<robot>/scan_filtered`

## Troubleshooting

### `colcon build` fails in `slam_toolbox`

Make sure you build with:

```bash
colcon build --symlink-install --allow-overriding slam_toolbox --packages-up-to carter_multi_nav
```

This workspace uses the vendored `src/slam_toolbox`, not the binary package version.

### RViz opens but you do not see TF or scans

Check that these helper nodes are running:

```bash
ros2 node list | grep -E 'tf_aggregate_relay|scan_relay|map_selector'
```

Then verify shared visualization topics:

```bash
ros2 topic echo --once /tf
ros2 topic echo --once /shared_map
ros2 topic echo --once /rviz/carter1/scan_filtered
```

### `/shared_map` is empty

Wait until the selected source robot has published map data, then check:

```bash
ros2 topic echo --once /carter1/map
ros2 topic echo --once /shared_map
```

If needed, choose a different canonical map source:

```bash
ros2 launch carter_multi_nav multi_carter_mapping_nav.launch.py shared_map_source:=carter2
```

### Nav2 is up but a robot will not move to a goal

Check:

```bash
ros2 action list | grep navigate_to_pose
ros2 topic echo /carter1/cmd_vel --once
ros2 topic echo /carter1/scan_filtered --once
ros2 topic echo /carter1/scan_peer_filtered --once
ros2 topic echo /carter1/planning_map --once
```

Typical causes:

- goal is in unknown space
- goal is outside the robot's current map
- lidar data is missing
- peer robot occupancy is still present because shared TF is missing or stale
- that robot's SLAM node is not publishing a valid `map -> odom` transform yet

### You see DDS participant exhaustion or discovery issues

This repo includes `src/carter_multi_nav/config/cyclonedds_multi_robot.xml` and the launch files set `CYCLONEDDS_URI` automatically.

If you still see DDS-related failures, check whether your shell already exported a conflicting value:

```bash
echo $CYCLONEDDS_URI
unset CYCLONEDDS_URI
```

Then source the workspace again and relaunch.

### One robot crashes while the others stay up

Start by isolating that robot:

```bash
ros2 launch carter_multi_nav single_carter_debug.launch.py robot_name:=carter3
```

That separates robot-local TF, lidar, SLAM, and Nav2 issues from the full three-robot bringup.

## License

Apache-2.0. See [LICENSE](LICENSE).
