import argparse
import glob
import json
import math
import os
from statistics import median

import yaml

from carter_multi_nav.report_utils import ensure_output_dir, write_json


def _safe_median(values):
    values = [value for value in values if value is not None]
    return median(values) if values else None


def _round_value(value, digits=3):
    if value is None:
        return None
    return round(float(value), digits)


def _load_scan_summaries(input_dir: str):
    summaries = []
    for path in sorted(glob.glob(os.path.join(input_dir, "scan_motion_*.json"))):
        with open(path, "r", encoding="utf-8") as handle:
            summaries.append(json.load(handle))
    if not summaries:
        raise RuntimeError("No scan_motion_*.json summaries found in %s" % input_dir)
    return summaries


def _build_recommendations(scan_summaries, nav_target_linear_speed: float):
    effective_durations = []
    median_periods = []
    stamp_lags = []
    for summary in scan_summaries:
        effective_duration = summary.get("median_effective_scan_duration_sec")
        if effective_duration is None:
            effective_duration = (
                summary.get("median_reported_scan_time_sec")
                or summary.get("median_stamp_period_sec")
                or summary.get("median_receipt_period_sec")
            )
        if effective_duration is not None:
            effective_durations.append(float(effective_duration))

        period = summary.get("median_stamp_period_sec") or summary.get(
            "median_receipt_period_sec"
        )
        if period is not None:
            median_periods.append(float(period))

        lag = summary.get("median_stamp_lag_sec")
        if lag is not None:
            stamp_lags.append(float(lag))

    if not effective_durations:
        raise RuntimeError("No effective scan duration values were found")

    safe_omegas = [
        math.radians(1.0) / duration for duration in effective_durations if duration > 0.0
    ]
    max_safe_omega = min(safe_omegas)
    limited_omega = 0.9 * max_safe_omega
    minimum_time_interval = _safe_median(median_periods)
    if minimum_time_interval is None:
        minimum_time_interval = _safe_median(effective_durations)
    minimum_time_interval = round(float(minimum_time_interval), 2)

    throttle_scans = 1
    median_stamp_lag = _safe_median(stamp_lags)
    if median_stamp_lag is not None and median_stamp_lag > max(0.10, 0.5 * minimum_time_interval):
        throttle_scans = 2

    minimum_travel_distance = max(
        0.08,
        nav_target_linear_speed * minimum_time_interval,
    )
    lookahead_dist = max(1.0, nav_target_linear_speed * 2.5)
    min_lookahead_dist = max(0.5, nav_target_linear_speed * 1.25)
    max_lookahead_dist = max(2.0, nav_target_linear_speed * 5.0)
    linear_cap = max(nav_target_linear_speed * 1.25, nav_target_linear_speed)
    linear_accel = max(nav_target_linear_speed * 2.5, 0.5)

    nav_yaml = {
        "controller_server": {
            "ros__parameters": {
                "FollowPath": {
                    "desired_linear_vel": _round_value(nav_target_linear_speed),
                    "lookahead_dist": _round_value(lookahead_dist),
                    "min_lookahead_dist": _round_value(min_lookahead_dist),
                    "max_lookahead_dist": _round_value(max_lookahead_dist),
                    "rotate_to_heading_angular_vel": _round_value(limited_omega),
                }
            }
        },
        "velocity_smoother": {
            "ros__parameters": {
                "max_velocity": [
                    _round_value(linear_cap),
                    0.0,
                    _round_value(limited_omega),
                ],
                "max_accel": [_round_value(linear_accel), 0.0, 3.0],
                "max_decel": [-_round_value(linear_accel), 0.0, -3.0],
            }
        },
    }
    slam_yaml = {
        "scan_motion_gate": {
            "ros__parameters": {
                "max_angular_velocity": _round_value(max_safe_omega),
                "max_rotation_per_scan_deg": 1.0,
            }
        },
        "$(var namespace)/slam_toolbox": {
            "ros__parameters": {
                "throttle_scans": throttle_scans,
                "minimum_time_interval": _round_value(minimum_time_interval, 2),
                "minimum_travel_distance": _round_value(minimum_travel_distance, 3),
            }
        },
    }
    metadata = {
        "effective_durations_sec": [_round_value(value, 4) for value in effective_durations],
        "median_periods_sec": [_round_value(value, 4) for value in median_periods],
        "median_stamp_lag_sec": _round_value(median_stamp_lag, 4),
        "max_safe_omega_rad_s": _round_value(max_safe_omega, 4),
        "limited_omega_rad_s": _round_value(limited_omega, 4),
        "minimum_time_interval_sec": _round_value(minimum_time_interval, 2),
        "minimum_travel_distance_m": _round_value(minimum_travel_distance, 3),
        "throttle_scans": throttle_scans,
        "nav_target_linear_speed_m_s": _round_value(nav_target_linear_speed, 3),
    }
    return nav_yaml, slam_yaml, metadata


def main():
    parser = argparse.ArgumentParser(
        description="Generate recommended Nav2 and SLAM parameter overlays from characterization output."
    )
    parser.add_argument("--input-dir", required=True, help="Characterization output directory")
    parser.add_argument(
        "--output-dir",
        default="",
        help="Directory to write recommended_nav2.yaml and recommended_slam.yaml",
    )
    parser.add_argument(
        "--nav-target-linear-speed",
        type=float,
        default=0.40,
        help="Target linear speed in m/s for the generated Nav2 overlay",
    )
    args = parser.parse_args()

    input_dir = os.path.abspath(os.path.expanduser(args.input_dir))
    output_dir = ensure_output_dir(args.output_dir or input_dir)
    scan_summaries = _load_scan_summaries(input_dir)
    nav_yaml, slam_yaml, metadata = _build_recommendations(
        scan_summaries, float(args.nav_target_linear_speed)
    )

    nav_path = os.path.join(output_dir, "recommended_nav2.yaml")
    slam_path = os.path.join(output_dir, "recommended_slam.yaml")
    metadata_path = os.path.join(output_dir, "recommended_params_metadata.json")

    with open(nav_path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(nav_yaml, handle, sort_keys=False)
    with open(slam_path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(slam_yaml, handle, sort_keys=False)
    write_json(metadata_path, metadata)

    print(f"Wrote {nav_path}")
    print(f"Wrote {slam_path}")
    print(f"Wrote {metadata_path}")


if __name__ == "__main__":
    main()
