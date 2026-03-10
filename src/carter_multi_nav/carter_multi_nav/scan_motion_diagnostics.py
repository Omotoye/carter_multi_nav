from collections import deque
import math
from statistics import median

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan


def _safe_median(values):
    return median(values) if values else None


def _format_seconds(value):
    if value is None:
        return "n/a"
    return f"{value:.3f}s"


def _format_hz(period_seconds):
    if period_seconds is None or period_seconds <= 0.0:
        return "n/a"
    return f"{1.0 / period_seconds:.2f} Hz"


def _format_deg(value):
    if value is None:
        return "n/a"
    return f"{value:.2f} deg"


def _format_rad(value):
    if value is None:
        return "n/a"
    return f"{value:.3f} rad/s"


class ScanMotionDiagnostics(Node):
    def __init__(self):
        super().__init__("scan_motion_diagnostics")

        self.declare_parameter("robot_name", "carter1")
        self.declare_parameter("scan_topic", "")
        self.declare_parameter("odom_topic", "")
        self.declare_parameter("report_interval", 5.0)
        self.declare_parameter("window_size", 200)
        self.declare_parameter("rotation_target_deg", 2.0)
        self.declare_parameter("stale_odom_timeout", 0.5)

        robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        report_interval = (
            self.get_parameter("report_interval").get_parameter_value().double_value
        )
        window_size = self.get_parameter("window_size").get_parameter_value().integer_value
        self._rotation_target_deg = (
            self.get_parameter("rotation_target_deg").get_parameter_value().double_value
        )
        self._stale_odom_timeout = (
            self.get_parameter("stale_odom_timeout").get_parameter_value().double_value
        )

        if not scan_topic:
            scan_topic = f"/{robot_name}/front_2d_lidar/scan"
        if not odom_topic:
            odom_topic = f"/{robot_name}/chassis/odom"

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._robot_name = robot_name
        self._scan_topic = scan_topic
        self._odom_topic = odom_topic

        self._receipt_periods = deque(maxlen=window_size)
        self._stamp_periods = deque(maxlen=window_size)
        self._scan_times = deque(maxlen=window_size)
        self._time_increment_durations = deque(maxlen=window_size)
        self._stamp_lags = deque(maxlen=window_size)
        self._rotation_per_scan_deg = deque(maxlen=window_size)
        self._odom_angular_velocity = deque(maxlen=window_size)

        self._scan_count = 0
        self._angle_increment_sign_flips = 0
        self._metadata_mismatch_count = 0
        self._metadata_sample_count = 0

        self._last_receipt_time = None
        self._last_scan_stamp = None
        self._last_angle_increment_sign = None

        self._latest_omega_z = None
        self._latest_odom_time = None

        self.create_subscription(LaserScan, scan_topic, self._handle_scan, sensor_qos)
        self.create_subscription(Odometry, odom_topic, self._handle_odom, sensor_qos)
        self.create_timer(report_interval, self._report)

        self.get_logger().info(
            "Monitoring scan distortion for '%s' on scan='%s' odom='%s'"
            % (robot_name, scan_topic, odom_topic)
        )

    def _handle_odom(self, msg: Odometry):
        self._latest_omega_z = float(msg.twist.twist.angular.z)
        self._latest_odom_time = Time.from_msg(msg.header.stamp)
        self._odom_angular_velocity.append(abs(self._latest_omega_z))

    def _handle_scan(self, msg: LaserScan):
        self._scan_count += 1

        now = self.get_clock().now()
        stamp = Time.from_msg(msg.header.stamp)

        if self._last_receipt_time is not None:
            self._receipt_periods.append(
                (now - self._last_receipt_time).nanoseconds / 1e9
            )
        self._last_receipt_time = now

        if self._last_scan_stamp is not None:
            stamp_delta = (stamp - self._last_scan_stamp).nanoseconds / 1e9
            if stamp_delta > 0.0:
                self._stamp_periods.append(stamp_delta)
        self._last_scan_stamp = stamp

        lag = (now - stamp).nanoseconds / 1e9
        if lag >= 0.0:
            self._stamp_lags.append(lag)

        scan_time = float(msg.scan_time)
        if scan_time > 0.0:
            self._scan_times.append(scan_time)

        beam_count = len(msg.ranges)
        time_increment = float(msg.time_increment)
        if time_increment > 0.0 and beam_count > 1:
            sweep_duration = time_increment * (beam_count - 1)
            self._time_increment_durations.append(sweep_duration)
            self._metadata_sample_count += 1
            if scan_time > 0.0 and abs(sweep_duration - scan_time) > max(0.02, 0.25 * scan_time):
                self._metadata_mismatch_count += 1

        sign = 0
        if msg.angle_increment > 0.0:
            sign = 1
        elif msg.angle_increment < 0.0:
            sign = -1
        if (
            self._last_angle_increment_sign is not None
            and sign != 0
            and sign != self._last_angle_increment_sign
        ):
            self._angle_increment_sign_flips += 1
        if sign != 0:
            self._last_angle_increment_sign = sign

        effective_duration = self._effective_scan_duration(scan_time)
        if effective_duration is None:
            return

        if self._latest_omega_z is None or self._latest_odom_time is None:
            return

        odom_age = abs((stamp - self._latest_odom_time).nanoseconds) / 1e9
        if odom_age > self._stale_odom_timeout:
            return

        rotation_deg = math.degrees(abs(self._latest_omega_z) * effective_duration)
        self._rotation_per_scan_deg.append(rotation_deg)

    def _effective_scan_duration(self, scan_time):
        if scan_time and scan_time > 0.0:
            return scan_time

        duration = _safe_median(self._time_increment_durations)
        if duration and duration > 0.0:
            return duration

        duration = _safe_median(self._stamp_periods)
        if duration and duration > 0.0:
            return duration

        duration = _safe_median(self._receipt_periods)
        if duration and duration > 0.0:
            return duration

        return None

    def _report(self):
        if self._scan_count == 0:
            self.get_logger().warn("No scan messages received yet.")
            return

        receipt_period = _safe_median(self._receipt_periods)
        stamp_period = _safe_median(self._stamp_periods)
        scan_time = _safe_median(self._scan_times)
        lag = _safe_median(self._stamp_lags)
        rotation_deg = _safe_median(self._rotation_per_scan_deg)
        peak_rotation_deg = (
            max(self._rotation_per_scan_deg) if self._rotation_per_scan_deg else None
        )
        omega = _safe_median(self._odom_angular_velocity)
        peak_omega = max(self._odom_angular_velocity) if self._odom_angular_velocity else None

        lines = [
            "Scan diagnostics for %s" % self._robot_name,
            "  scans observed: %d" % self._scan_count,
            "  receipt period: %s (%s)"
            % (_format_seconds(receipt_period), _format_hz(receipt_period)),
            "  stamp period: %s (%s)"
            % (_format_seconds(stamp_period), _format_hz(stamp_period)),
            "  reported scan_time: %s" % _format_seconds(scan_time),
            "  stamp lag: %s" % _format_seconds(lag),
            "  median angular velocity: %s" % _format_rad(omega),
            "  peak angular velocity: %s" % _format_rad(peak_omega),
            "  median rotation per scan: %s" % _format_deg(rotation_deg),
            "  peak rotation per scan: %s" % _format_deg(peak_rotation_deg),
            "  angle_increment sign flips: %d" % self._angle_increment_sign_flips,
        ]

        if self._metadata_sample_count > 0:
            mismatch_ratio = self._metadata_mismatch_count / float(self._metadata_sample_count)
            lines.append(
                "  scan metadata mismatch ratio: %.1f%%" % (100.0 * mismatch_ratio)
            )

        recommendations = self._build_recommendations(
            receipt_period=receipt_period,
            stamp_period=stamp_period,
            scan_time=scan_time,
            lag=lag,
            rotation_deg=rotation_deg,
            peak_rotation_deg=peak_rotation_deg,
        )
        if recommendations:
            lines.append("  recommendations:")
            for recommendation in recommendations:
                lines.append("    - %s" % recommendation)

        self.get_logger().info("\n".join(lines))

    def _build_recommendations(
        self,
        *,
        receipt_period,
        stamp_period,
        scan_time,
        lag,
        rotation_deg,
        peak_rotation_deg,
    ):
        recommendations = []
        effective_duration = self._effective_scan_duration(scan_time)

        if lag is not None and lag > 0.10:
            recommendations.append(
                "scan timestamps lag the current clock by about %.0f ms; keep slam_toolbox transform_timeout at 0.5 s or higher and verify the lidar driver stamps from /clock"
                % (lag * 1000.0)
            )

        if (
            self._metadata_sample_count > 0
            and self._metadata_mismatch_count / float(self._metadata_sample_count) > 0.20
        ):
            recommendations.append(
                "scan_time and time_increment disagree often; the sweep metadata itself may be unreliable, which limits how much SLAM tuning can fix"
            )

        if self._angle_increment_sign_flips > 0:
            recommendations.append(
                "angle_increment changed sign during the run; that indicates the scan stream is not directionally stable and should be investigated upstream of SLAM"
            )

        if effective_duration is not None:
            target_rad = math.radians(self._rotation_target_deg)
            recommended_max_omega = target_rad / effective_duration if effective_duration > 0.0 else None
            if recommended_max_omega is not None:
                if peak_rotation_deg is not None and peak_rotation_deg > self._rotation_target_deg * 2.0:
                    recommendations.append(
                        "cap rotation near %.3f rad/s (%.1f deg/s) to stay around %.1f deg of motion per scan; the current platform motion is too fast for this scan stream"
                        % (
                            recommended_max_omega,
                            math.degrees(recommended_max_omega),
                            self._rotation_target_deg,
                        )
                    )
                elif rotation_deg is not None and rotation_deg > self._rotation_target_deg:
                    recommendations.append(
                        "median rotation per scan is above target; keep rotate_to_heading_angular_vel and velocity_smoother theta max_velocity at or below %.3f rad/s"
                        % recommended_max_omega
                    )

        period = stamp_period if stamp_period is not None else receipt_period
        if period is not None and period > 0.18:
            recommendations.append(
                "scan rate is fairly low; prefer conservative motion and keep slam_toolbox throttle_scans >= 2 if mapping still tears during turns"
            )

        if not recommendations:
            recommendations.append(
                "current scan timing looks usable; if the map still tears under motion, the remaining issue is likely upstream distortion in the lidar or odom stream rather than mapper update rate alone"
            )

        return recommendations


def main(args=None):
    rclpy.init(args=args)
    node = ScanMotionDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
