from typing import Dict, List, Set

from carter_multi_nav_msgs.msg import RobotTagDetections, RobotTagDetectionsArray
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
try:
    from pupil_apriltags import Detector as PupilAprilTagDetector
except ImportError:
    PupilAprilTagDetector = None
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray

from carter_multi_nav.common import DEFAULT_ROBOTS


_APRILTAG_DICTIONARIES = {
    "16h5": getattr(cv2.aruco, "DICT_APRILTAG_16h5", cv2.aruco.DICT_APRILTAG_16H5),
    "25h9": getattr(cv2.aruco, "DICT_APRILTAG_25h9", cv2.aruco.DICT_APRILTAG_25H9),
    "36h10": getattr(
        cv2.aruco, "DICT_APRILTAG_36h10", cv2.aruco.DICT_APRILTAG_36H10
    ),
    "36h11": getattr(
        cv2.aruco, "DICT_APRILTAG_36h11", cv2.aruco.DICT_APRILTAG_36H11
    ),
}

_PUPIL_APRILTAG_FAMILIES = {
    "16h5": "tag16h5",
    "25h9": "tag25h9",
    "36h10": "tag36h10",
    "36h11": "tag36h11",
}


class AprilTagDetector(Node):
    def __init__(self):
        super().__init__("april_tag_detector")

        self.declare_parameter("robot_names", list(DEFAULT_ROBOTS))
        self.declare_parameter("image_topics", [])
        self.declare_parameter("image_topic_suffix", "front_stereo_camera/left/image_raw")
        self.declare_parameter("tag_family", "36h11")
        self.declare_parameter("min_tag_id", 1)
        self.declare_parameter("max_tag_id", 10)
        self.declare_parameter("publish_topic", "/april_tag/detected_ids")
        self.declare_parameter(
            "publish_by_robot_topic", "/april_tag/detections_by_robot"
        )
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("detection_hold_time", 1.0)
        self.declare_parameter("stale_image_timeout", 1.0)
        self.declare_parameter("max_processing_fps", 6.0)
        self.declare_parameter("log_detection_changes", True)

        robot_names = list(
            self.get_parameter("robot_names").get_parameter_value().string_array_value
        )
        image_topics = list(
            self.get_parameter("image_topics").get_parameter_value().string_array_value
        )
        image_topic_suffix = (
            self.get_parameter("image_topic_suffix").get_parameter_value().string_value
        )
        tag_family = self.get_parameter("tag_family").get_parameter_value().string_value
        self._min_tag_id = self.get_parameter("min_tag_id").get_parameter_value().integer_value
        self._max_tag_id = self.get_parameter("max_tag_id").get_parameter_value().integer_value
        publish_topic = self.get_parameter("publish_topic").get_parameter_value().string_value
        publish_by_robot_topic = (
            self.get_parameter("publish_by_robot_topic").get_parameter_value().string_value
        )
        publish_rate_hz = (
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )
        self._detection_hold_time = (
            self.get_parameter("detection_hold_time").get_parameter_value().double_value
        )
        self._stale_image_timeout = (
            self.get_parameter("stale_image_timeout").get_parameter_value().double_value
        )
        self._max_processing_fps = (
            self.get_parameter("max_processing_fps").get_parameter_value().double_value
        )
        self._log_detection_changes = (
            self.get_parameter("log_detection_changes").get_parameter_value().bool_value
        )

        if not image_topics:
            suffix = str(image_topic_suffix or "").strip().lstrip("/")
            image_topics = [f"/{robot_name}/{suffix}" for robot_name in robot_names]
        if not image_topics:
            raise RuntimeError("No image topics configured for AprilTag detection.")
        if self._min_tag_id > self._max_tag_id:
            raise RuntimeError(
                "Invalid tag range: min_tag_id=%d max_tag_id=%d"
                % (self._min_tag_id, self._max_tag_id)
            )
        if publish_rate_hz <= 0.0:
            raise RuntimeError("publish_rate_hz must be > 0.0")
        if self._max_processing_fps <= 0.0:
            raise RuntimeError("max_processing_fps must be > 0.0")

        self._bridge = CvBridge()
        self._dictionary = self._create_dictionary(tag_family)
        self._detector_parameters = self._create_opencv_detector_parameters()
        self._pupil_detector = self._create_pupil_detector(tag_family)
        self._scales_to_try = (1.0, 0.75, 0.5, 1.5)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._camera_topics = image_topics
        self._robot_names = robot_names
        self._camera_robot_names = {
            topic: self._resolve_robot_name(topic, robot_names) for topic in image_topics
        }
        self._camera_detections: Dict[str, Set[int]] = {topic: set() for topic in image_topics}
        self._camera_last_image_time = {topic: None for topic in image_topics}
        self._camera_last_processed_time = {topic: None for topic in image_topics}
        self._camera_last_seen_tag_time = {topic: {} for topic in image_topics}
        self._last_logged_camera_detections: Dict[str, List[int]] = {
            topic: [] for topic in image_topics
        }
        self._last_published_ids: List[int] = []

        for image_topic in image_topics:
            self.create_subscription(
                Image,
                image_topic,
                lambda msg, topic=image_topic: self._handle_image(topic, msg),
                sensor_qos,
            )

        self._publisher = self.create_publisher(Int32MultiArray, publish_topic, 10)
        self._by_robot_publisher = self.create_publisher(
            RobotTagDetectionsArray, publish_by_robot_topic, 10
        )
        self.create_timer(1.0 / publish_rate_hz, self._publish_detected_ids)

        self.get_logger().info(
            "Watching %d image topics for AprilTags %d-%d with family %s and %.2fs hold: %s"
            % (
                len(self._camera_topics),
                self._min_tag_id,
                self._max_tag_id,
                tag_family,
                self._detection_hold_time,
                ", ".join(self._camera_topics),
            )
        )
        self.get_logger().info("Publishing aggregated detections to '%s'" % publish_topic)
        self.get_logger().info(
            "Publishing per-robot detections to '%s'" % publish_by_robot_topic
        )
        backend_names = ["opencv"]
        if self._pupil_detector is not None:
            backend_names.insert(0, "pupil_apriltags")
        self.get_logger().info(
            "Detection backends: %s, max_processing_fps=%.1f"
            % (", ".join(backend_names), self._max_processing_fps)
        )

    def _create_dictionary(self, tag_family: str):
        normalized = str(tag_family or "").strip().lower()
        dictionary_id = _APRILTAG_DICTIONARIES.get(normalized)
        if dictionary_id is None:
            supported = ", ".join(sorted(_APRILTAG_DICTIONARIES.keys()))
            raise RuntimeError(
                "Unsupported tag_family '%s'. Supported AprilTag families: %s"
                % (tag_family, supported)
            )
        return cv2.aruco.getPredefinedDictionary(dictionary_id)

    def _create_pupil_detector(self, tag_family: str):
        if PupilAprilTagDetector is None:
            return None

        family = _PUPIL_APRILTAG_FAMILIES.get(str(tag_family or "").strip().lower())
        if family is None:
            return None

        return PupilAprilTagDetector(
            families=family,
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.5,
            debug=0,
        )

    def _create_opencv_detector_parameters(self):
        parameters = cv2.aruco.DetectorParameters_create()
        parameters.adaptiveThreshWinSizeMin = 3
        parameters.adaptiveThreshWinSizeMax = 53
        parameters.adaptiveThreshWinSizeStep = 5
        parameters.minMarkerPerimeterRate = 0.01
        parameters.maxMarkerPerimeterRate = 8.0
        parameters.minCornerDistanceRate = 0.01
        if hasattr(cv2.aruco, "CORNER_REFINE_APRILTAG"):
            parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        return parameters

    def _resolve_robot_name(self, image_topic: str, robot_names: List[str]) -> str:
        topic_parts = [part for part in str(image_topic or "").split("/") if part]
        if topic_parts and topic_parts[0] in robot_names:
            return topic_parts[0]
        return topic_parts[0] if topic_parts else str(image_topic or "")

    def _handle_image(self, image_topic: str, msg: Image):
        receipt_time = self.get_clock().now()
        self._camera_last_image_time[image_topic] = receipt_time
        last_processed_time = self._camera_last_processed_time[image_topic]
        if last_processed_time is not None:
            seconds_since_processed = (receipt_time - last_processed_time).nanoseconds / 1e9
            if seconds_since_processed < (1.0 / self._max_processing_fps):
                return
        self._camera_last_processed_time[image_topic] = receipt_time

        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except CvBridgeError:
            try:
                image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            except CvBridgeError as exc:
                self.get_logger().warn(
                    "Failed to decode image from '%s': %s" % (image_topic, exc)
                )
                return

        try:
            gray = self._to_grayscale(image)
            detected_ids = self._detect_ids(gray)
        except Exception as exc:
            self.get_logger().warn(
                "Failed AprilTag detection on '%s': %s" % (image_topic, exc)
            )
            return

        self._camera_detections[image_topic] = set(detected_ids)
        for tag_id in detected_ids:
            self._camera_last_seen_tag_time[image_topic][tag_id] = receipt_time

        if self._log_detection_changes and detected_ids != self._last_logged_camera_detections[
            image_topic
        ]:
            self._last_logged_camera_detections[image_topic] = detected_ids
            if detected_ids:
                self.get_logger().info(
                    "Detected AprilTags on '%s': %s"
                    % (image_topic, ", ".join(str(tag_id) for tag_id in detected_ids))
                )

    def _to_grayscale(self, image):
        if image.ndim == 2:
            return image

        if image.ndim != 3:
            raise ValueError("Unsupported image shape %s" % (image.shape,))

        channels = image.shape[2]
        if channels == 1:
            return image[:, :, 0]
        if channels == 3:
            return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if channels == 4:
            return cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)

        raise ValueError("Unsupported channel count %d" % channels)

    def _detect_ids(self, gray_image) -> List[int]:
        prepared = self._prepare_detection_image(gray_image)
        detected_ids = self._run_detection_passes(prepared)
        if detected_ids:
            return detected_ids

        enhanced = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(prepared)
        return self._run_detection_passes(enhanced)

    def _prepare_detection_image(self, gray_image):
        if gray_image.dtype != np.uint8:
            gray_image = cv2.normalize(gray_image, None, 0, 255, cv2.NORM_MINMAX)
            gray_image = gray_image.astype(np.uint8)
        return gray_image

    def _run_detection_passes(self, gray_image) -> List[int]:
        detected_ids = set()

        for scaled_image in self._scaled_images(gray_image):
            detected_ids.update(self._detect_with_pupil(scaled_image))
            detected_ids.update(self._detect_with_opencv(scaled_image))

        return sorted(
            tag_id
            for tag_id in detected_ids
            if self._min_tag_id <= tag_id <= self._max_tag_id
        )

    def _scaled_images(self, gray_image):
        scaled_images = []
        height, width = gray_image.shape[:2]

        for scale in self._scales_to_try:
            if scale == 1.0:
                scaled_images.append(gray_image)
                continue

            scaled_width = int(round(width * scale))
            scaled_height = int(round(height * scale))
            if scaled_width < 80 or scaled_height < 80:
                continue
            if scaled_width > 2200 or scaled_height > 2200:
                continue

            interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_CUBIC
            scaled_images.append(
                cv2.resize(
                    gray_image,
                    (scaled_width, scaled_height),
                    interpolation=interpolation,
                )
            )

        return scaled_images

    def _detect_with_pupil(self, gray_image):
        if self._pupil_detector is None:
            return []

        detections = self._pupil_detector.detect(gray_image, estimate_tag_pose=False)
        return [int(detection.tag_id) for detection in detections]

    def _detect_with_opencv(self, gray_image):
        _corners, ids, _rejected = cv2.aruco.detectMarkers(
            gray_image,
            self._dictionary,
            parameters=self._detector_parameters,
        )
        if ids is None:
            return []
        return [int(raw_id) for raw_id in ids.flatten().tolist()]

    def _publish_detected_ids(self):
        now = self.get_clock().now()
        aggregated_ids = set()
        detections_by_robot = {}

        for image_topic in self._camera_topics:
            last_image_time = self._camera_last_image_time[image_topic]
            if last_image_time is None:
                current_tag_ids = []
            else:
                age_seconds = (now - last_image_time).nanoseconds / 1e9
                if age_seconds > self._stale_image_timeout:
                    current_tag_ids = []
                else:
                    current_tag_ids = []
                    for (
                        tag_id,
                        last_seen_time,
                    ) in self._camera_last_seen_tag_time[image_topic].items():
                        detection_age_seconds = (now - last_seen_time).nanoseconds / 1e9
                        if detection_age_seconds <= self._detection_hold_time:
                            current_tag_ids.append(tag_id)

            current_tag_ids = sorted(set(current_tag_ids))
            aggregated_ids.update(current_tag_ids)
            robot_name = self._camera_robot_names[image_topic]
            detections_by_robot[robot_name] = RobotTagDetections(
                robot_name=robot_name,
                image_topic=image_topic,
                tag_ids=current_tag_ids,
            )

        published_ids = sorted(aggregated_ids)
        self._publisher.publish(Int32MultiArray(data=published_ids))
        self._by_robot_publisher.publish(
            RobotTagDetectionsArray(
                stamp=now.to_msg(),
                detections=[
                    detections_by_robot[robot_name]
                    for robot_name in sorted(detections_by_robot.keys())
                ],
            )
        )

        if self._log_detection_changes and published_ids != self._last_published_ids:
            self._last_published_ids = published_ids
            if published_ids:
                self.get_logger().info(
                    "Aggregated AprilTags: %s"
                    % ", ".join(str(tag_id) for tag_id in published_ids)
                )
            else:
                self.get_logger().info("Aggregated AprilTags: none")


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        try:
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass
