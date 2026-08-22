#!/usr/bin/env python3
"""ROS 2 node for synchronized RGB/depth ArUco detection, 3D association, robust
6-DoF pose estimation, depth+PnP fusion, and time-aware temporal tracking.

Pipeline per synchronized RGB/depth pair:

    RGB, depth, CameraInfo
        -> ArUco detection + 2D geometric validation      (Stage 1)
        -> 2D image-quality gating (blur/contrast/geometry) (Improvement #7)
        -> RGB-D depth association                         (Stage 2)
        -> robust multi-candidate solvePnP                 (Stage 3, Improvement #1)
        -> time-aware tracking + depth/PnP fusion filter    (Stage 4, Improvements #2, #5, #9, #10)
        -> quality flags + split confidences + covariance   (Improvements #3, #4, #22)
        -> periodic diagnostics                             (Improvement #14/#15)

Competition mode (Improvement #20) disables debug-image publishing and
verbose logging without touching the estimation/tracking pipeline itself.
"""
from __future__ import annotations

from collections import deque
import math
import time
from dataclasses import dataclass
from typing import Any
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from marker_detection.msg import (Marker3DDetection, Marker3DDetectionArray,
                                  MarkerDetection, MarkerDetectionArray, MarkerPose, MarkerPoseArray,
                                  MarkerTrackedPose, MarkerTrackedPoseArray)
from . import diagnostics as diag
from . import image_quality
from .aruco_detector import ArucoDetector
from .camera_model import RgbCameraModel
from .detection_validator import DetectionValidator, ValidatedMarker
from .depth_processor import DepthEstimate, DepthProcessor
from .image_processor import bgr_to_gray
from .pose_estimator import PoseEstimate, PoseEstimator
from .quality_flags import QualityFlag
from .sync_buffer import DepthSyncBuffer
from .tracker_manager import RawPose, TrackerManager, TrackedPose, TrackState

_DIAG_LEVEL = {diag.HealthLevel.OK: DiagnosticStatus.OK, diag.HealthLevel.WARN: DiagnosticStatus.WARN,
              diag.HealthLevel.ERROR: DiagnosticStatus.ERROR}


@dataclass(frozen=True)
class AssociatedMarker:
    marker: ValidatedMarker
    depth: DepthEstimate
    position: tuple[float, float, float] | None
    detection_confidence: float = 0.0
    quality_flags: int = 0


@dataclass(frozen=True)
class AssociatedPose:
    association: AssociatedMarker
    pose: PoseEstimate
    position_difference_m: float
    valid: bool
    quality_flags: int = 0


class MarkerDetectionNode(Node):
    """Publishes validated marker IDs and unmodified OpenCV-order image corners."""

    def __init__(self) -> None:
        super().__init__("marker_detection")
        self._declare_parameters()
        self._validate_parameters()
        self.bridge = CvBridge()
        self.camera_info: CameraInfo | None = None
        self._camera_info_warned = False
        self._camera_info_received_monotonic: float | None = None
        self._processing_times: deque[float] = deque(maxlen=300)
        self._last_stats_log = time.monotonic()
        # Part 2: RGB detection must not depend on depth. `_last_rgb_frame` tracks
        # overall pipeline liveness (any RGB frame); `_last_synchronized_frame`
        # specifically tracks the last frame where a matching depth frame was
        # also found, i.e. the last time 3D/depth-fused output was possible.
        self._last_rgb_frame = time.monotonic()
        self._last_synchronized_frame = time.monotonic()
        self._diag_window_start = time.monotonic()
        self._sync_deltas: deque[float] = deque(maxlen=300)
        self._counts = dict(detections=0, rejected_quality=0, rejected_pose=0,
                            confirmed=0, degraded=0, lost=0,
                            frames_with_depth=0, frames_without_depth=0)
        self._competition_mode = bool(self.get_parameter("competition_mode").value)

        detector_values = {name: self.get_parameter(name).value for name in (
            "adaptiveThreshWinSizeMin", "adaptiveThreshWinSizeMax", "adaptiveThreshWinSizeStep",
            "adaptiveThreshConstant", "minMarkerPerimeterRate", "maxMarkerPerimeterRate",
            "polygonalApproxAccuracyRate", "minCornerDistanceRate", "minDistanceToBorder",
            "perspectiveRemovePixelPerCell", "perspectiveRemoveIgnoredMarginPerCell",
            "maxErroneousBitsInBorderRate", "errorCorrectionRate")}
        self.detector = ArucoDetector(self.get_parameter("aruco_dictionary").value, detector_values)
        self.validator = DetectionValidator(
            list(self.get_parameter("allowed_marker_ids").value),
            float(self.get_parameter("min_marker_size_px").value),
            float(self.get_parameter("min_marker_area_px").value),
            int(self.get_parameter("min_distance_to_border").value))
        self.depth_processor = DepthProcessor(
            float(self.get_parameter("depth_scale").value),
            float(self.get_parameter("min_depth_m").value),
            float(self.get_parameter("max_depth_m").value),
            float(self.get_parameter("depth_roi_shrink_ratio").value),
            int(self.get_parameter("min_valid_depth_samples").value),
            self.get_parameter("depth_outlier_method").value,
            float(self.get_parameter("depth_outlier_threshold").value))
        self._pose_estimator_kwargs = self._build_pose_estimator_kwargs()
        self._marker_sizes = self._build_marker_size_overrides()
        self._default_marker_size_m = float(self.get_parameter("marker_size_m").value)
        self._pose_estimators: dict[float, PoseEstimator] = {}
        self.pose_estimator = self._pose_estimator_for_size(self._default_marker_size_m)  # Warm the default.
        self.detection_pub = self.create_publisher(
            MarkerDetectionArray, self.get_parameter("detection_topic").value, 10)
        self.detection_3d_pub = self.create_publisher(
            Marker3DDetectionArray, self.get_parameter("marker_3d_detection_topic").value, 10)
        self.raw_pose_pub = self.create_publisher(MarkerPoseArray, self.get_parameter("raw_pose_topic").value, 10)
        self.tracked_pose_pub = self.create_publisher(
            MarkerTrackedPoseArray, self.get_parameter("marker_pose_topic").value, 10)
        self.debug_pub = self.create_publisher(Image, self.get_parameter("debug_image_topic").value, 10)
        self.debug_3d_pub = self.create_publisher(
            Image, self.get_parameter("debug_3d_image_topic").value, 10)
        self.pose_debug_pub = self.create_publisher(Image, self.get_parameter("pose_debug_image_topic").value, 10)
        self.tracking_debug_pub = self.create_publisher(
            Image, self.get_parameter("tracking_debug_image_topic").value, 10)
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, self.get_parameter("diagnostics_topic").value, 10)
        self.tracker_manager = TrackerManager(
            min_confirmations=int(self.get_parameter("min_confirmations").value),
            confirmation_window_seconds=float(self.get_parameter("confirmation_window_seconds").value),
            degraded_after_seconds=float(self.get_parameter("degraded_after_seconds").value),
            lost_timeout_seconds=float(self.get_parameter("lost_timeout_seconds").value),
            track_timeout_seconds=float(self.get_parameter("track_timeout_seconds").value),
            process_noise=float(self.get_parameter("position_process_noise").value),
            measurement_noise=float(self.get_parameter("position_measurement_noise").value),
            initial_covariance=float(self.get_parameter("position_initial_covariance").value),
            orientation_alpha=float(self.get_parameter("orientation_alpha").value),
            innovation_gate_threshold=float(self.get_parameter("innovation_gate_threshold").value),
            max_orientation_innovation_deg=float(self.get_parameter("max_orientation_innovation_deg").value),
            max_reprojection_error_px=float(self.get_parameter("tracking_max_reprojection_error_px").value),
            position_jump_tolerance_m=float(self.get_parameter("position_jump_tolerance_m").value),
            max_linear_velocity_mps=float(self.get_parameter("max_linear_velocity_mps").value))
        # Part 2 (critical fix): RGB and depth are now subscribed to independently,
        # not fused with an ApproximateTimeSynchronizer before reaching the
        # detection callback. A depth stall used to silently stop the
        # synchronizer from firing at all, which stopped even 2D detection.
        # Now every RGB frame is processed as it arrives (_rgb_callback); depth
        # frames are buffered (_depth_callback) and the best-effort closest match
        # within sync_slop is used for 3D association/PnP fusion when one
        # exists, falling back to 2D-only output (no fabricated 3D) when it
        # doesn't. sync_queue_size/sync_slop keep their previous meaning.
        self._depth_buffer: DepthSyncBuffer[Image] = DepthSyncBuffer(
            int(self.get_parameter("sync_queue_size").value), float(self.get_parameter("sync_slop").value))
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.get_parameter("camera_info_topic").value, self._camera_info_callback,
            qos_profile_sensor_data)
        self.rgb_sub = self.create_subscription(
            Image, self.get_parameter("rgb_topic").value, self._rgb_callback, qos_profile_sensor_data)
        self.depth_sub = self.create_subscription(
            Image, self.get_parameter("depth_topic").value, self._depth_callback, qos_profile_sensor_data)
        self.create_timer(5.0, self._report_sync_health)
        if self.get_parameter("publish_diagnostics").value:
            period = 1.0 / max(float(self.get_parameter("diagnostic_rate_hz").value), 0.01)
            self.create_timer(period, self._publish_diagnostics)
        mode = "COMPETITION" if self._competition_mode else "DEVELOPMENT"
        self.get_logger().info(
            f"ArUco detector initialized using {self.detector.api_name}. Mode: {mode}.")

    def _declare_parameters(self) -> None:
        defaults: dict[str, Any] = {
            "rgb_topic": "/camera/color/image_raw", "depth_topic": "/camera/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/camera/color/camera_info", "debug_image_topic": "/marker_detection/debug_image",
            "detection_topic": "/marker_detections", "aruco_dictionary": "DICT_6X6_250",
            "use_clahe": False, "clahe_clip_limit": 2.0, "clahe_tile_grid_size": 8,
            "sync_queue_size": 10, "sync_slop": 0.05, "min_marker_size_px": 10.0,
            "min_marker_area_px": 50.0, "min_distance_to_border": 5,
            "publish_debug_image": True, "show_debug_info": True, "allowed_marker_ids": [],
            "marker_3d_detection_topic": "/marker_3d_detections",
            "debug_3d_image_topic": "/marker_detection/3d_debug_image",
            # For 16UC1 only: must match the RealSense driver's configured depth units.
            "depth_scale": 0.001, "min_depth_m": 0.15, "max_depth_m": 10.0,
            "depth_roi_shrink_ratio": 0.20, "min_valid_depth_samples": 20,
            "depth_outlier_method": "mad", "depth_outlier_threshold": 3.0,
            "min_depth_variance_m2": 1e-6,
            "publish_3d_debug_image": True, "max_depth_processing_time_ms": 20.0,
            "raw_pose_topic": "/marker_poses_raw", "marker_pose_topic": "/marker_poses", "marker_size_m": 0.10,
            "solvepnp_method": "IPPE_SQUARE", "max_reprojection_error_px": 3.0,
            "min_marker_depth_m": 0.15, "max_marker_depth_m": 10.0,
            "validate_against_depth": True, "max_depth_position_difference_m": 0.20,
            "publish_pose_debug_image": True, "pose_debug_image_topic": "/marker_detection/pose_debug_image",
            "publish_euler_angles": True, "draw_pose_axes": True,
            # Improvement #21: per-marker physical size overrides (parallel arrays; global marker_size_m is the fallback).
            "per_marker_size_ids": [0], "per_marker_size_values": [0.10],
            "use_per_marker_sizes": False,
            # Improvement #1: robust multi-candidate pose selection.
            "max_viewing_angle_deg": 75.0, "high_viewing_angle_warn_deg": 60.0,
            "high_reprojection_warn_px": -1.0, "depth_mismatch_quality_threshold": 0.5,
            "ambiguity_score_margin": 0.05,
            "candidate_weight_reprojection": 0.35, "candidate_weight_geometry": 0.15,
            "candidate_weight_depth": 0.15, "candidate_weight_temporal_position": 0.20,
            "candidate_weight_temporal_orientation": 0.10, "candidate_weight_viewing_angle": 0.05,
            # Improvement #3: pose covariance derivation.
            "pnp_lateral_uncertainty_px": 1.0, "pnp_z_uncertainty_ratio": 0.03, "min_position_variance_m2": 1e-6,
            # Improvement #10: dt-aware outlier rejection (shared meaning with the pose estimator's temporal terms).
            "position_jump_tolerance_m": 0.05, "max_linear_velocity_mps": 2.0,
            "orientation_jump_tolerance_deg": 10.0, "max_angular_velocity_dps": 180.0,
            # Improvement #7: 2D image-quality gating (independent of pose/depth).
            "min_blur_score": 15.0, "min_blur_score_hard": 3.0,
            "min_contrast_score": 8.0, "min_contrast_score_hard": 2.0,
            "max_side_length_cv": 0.35, "image_quality_shrink_ratio": 0.15,
            "min_pose_quality_marker_size_px": 20.0,
            # Improvement #5: time-aware tracking (replaces frame-counted confirmation/loss).
            "min_confirmations": 3, "confirmation_window_seconds": 1.0,
            "degraded_after_seconds": 0.5, "lost_timeout_seconds": 1.5, "track_timeout_seconds": 5.0,
            "position_process_noise": 0.5, "position_measurement_noise": 0.01,
            "position_initial_covariance": 0.1, "orientation_alpha": 0.25,
            "innovation_gate_threshold": 11.34, "max_orientation_innovation_deg": 45.0,
            "tracking_max_reprojection_error_px": 3.0,
            "tracking_debug_image_topic": "/marker_tracking/debug_image", "publish_tracking_debug_image": True,
            # Improvement #20: competition vs development mode.
            "competition_mode": False,
            # Improvement #14/#15: diagnostics and performance reporting.
            "publish_diagnostics": True, "diagnostics_topic": "/marker_detection/diagnostics",
            "diagnostic_rate_hz": 1.0, "latency_warn_p95_ms": 40.0, "latency_error_p95_ms": 100.0,
            "sync_stale_warn_s": 1.0,
            "adaptiveThreshWinSizeMin": 3, "adaptiveThreshWinSizeMax": 23,
            "adaptiveThreshWinSizeStep": 10, "adaptiveThreshConstant": 7.0,
            "minMarkerPerimeterRate": 0.03, "maxMarkerPerimeterRate": 4.0,
            "polygonalApproxAccuracyRate": 0.03, "minCornerDistanceRate": 0.05,
            "minDistanceToBorder": 3, "perspectiveRemovePixelPerCell": 4,
            "perspectiveRemoveIgnoredMarginPerCell": 0.13,
            "maxErroneousBitsInBorderRate": 0.35, "errorCorrectionRate": 0.6,
            # Part 10: optional single fallback detection pass (CLAHE-enhanced), only
            # run when the primary pass finds zero markers. Disabled by default;
            # preserves current detector behavior unchanged unless explicitly enabled.
            "use_adaptive_multiscale_detection": False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _validate_parameters(self) -> None:
        p = self.get_parameter
        if p("sync_queue_size").value < 1 or p("sync_slop").value <= 0:
            raise ValueError("sync_queue_size must be >= 1 and sync_slop must be > 0.")
        if p("clahe_clip_limit").value <= 0 or p("clahe_tile_grid_size").value < 1:
            raise ValueError("CLAHE parameters must be positive.")
        if p("adaptiveThreshWinSizeMin").value < 3 or p("adaptiveThreshWinSizeMax").value < p("adaptiveThreshWinSizeMin").value:
            raise ValueError("Invalid adaptive threshold window size range.")
        if p("depth_scale").value <= 0 or p("min_depth_m").value <= 0 or p("max_depth_m").value <= p("min_depth_m").value:
            raise ValueError("Invalid depth scale or valid depth range.")
        if p("marker_size_m").value <= 0 or p("max_reprojection_error_px").value <= 0:
            raise ValueError("marker_size_m and max_reprojection_error_px must be positive.")
        if p("min_confirmations").value < 1 or p("confirmation_window_seconds").value <= 0:
            raise ValueError("Invalid tracking confirmation-window parameters.")
        if not (0.0 < p("degraded_after_seconds").value <= p("lost_timeout_seconds").value
                <= p("track_timeout_seconds").value):
            raise ValueError(
                "Require 0 < degraded_after_seconds <= lost_timeout_seconds <= track_timeout_seconds.")
        if not 0.0 < p("orientation_alpha").value <= 1.0:
            raise ValueError("orientation_alpha must be in (0, 1].")
        if len(p("per_marker_size_ids").value) != len(p("per_marker_size_values").value):
            raise ValueError("per_marker_size_ids and per_marker_size_values must be the same length.")
        if p("use_per_marker_sizes").value and any(v <= 0 for v in p("per_marker_size_values").value):
            raise ValueError("All per_marker_size_values must be positive.")

    def _build_pose_estimator_kwargs(self) -> dict[str, Any]:
        p = self.get_parameter
        warn_px = float(p("high_reprojection_warn_px").value)
        return dict(
            max_viewing_angle_deg=float(p("max_viewing_angle_deg").value),
            high_viewing_angle_warn_deg=float(p("high_viewing_angle_warn_deg").value),
            high_reprojection_warn_px=None if warn_px < 0 else warn_px,
            depth_mismatch_quality_threshold=float(p("depth_mismatch_quality_threshold").value),
            max_depth_position_difference_m=float(p("max_depth_position_difference_m").value),
            position_jump_tolerance_m=float(p("position_jump_tolerance_m").value),
            max_linear_velocity_mps=float(p("max_linear_velocity_mps").value),
            orientation_jump_tolerance_deg=float(p("orientation_jump_tolerance_deg").value),
            max_angular_velocity_dps=float(p("max_angular_velocity_dps").value),
            pnp_lateral_uncertainty_px=float(p("pnp_lateral_uncertainty_px").value),
            pnp_z_uncertainty_ratio=float(p("pnp_z_uncertainty_ratio").value),
            min_position_variance_m2=float(p("min_position_variance_m2").value),
            ambiguity_score_margin=float(p("ambiguity_score_margin").value),
            candidate_weights=dict(
                reprojection=float(p("candidate_weight_reprojection").value),
                geometry=float(p("candidate_weight_geometry").value),
                depth=float(p("candidate_weight_depth").value),
                temporal_position=float(p("candidate_weight_temporal_position").value),
                temporal_orientation=float(p("candidate_weight_temporal_orientation").value),
                viewing_angle=float(p("candidate_weight_viewing_angle").value)))

    def _build_marker_size_overrides(self) -> dict[int, float]:
        if not self.get_parameter("use_per_marker_sizes").value:
            return {}
        ids = self.get_parameter("per_marker_size_ids").value
        values = self.get_parameter("per_marker_size_values").value
        return {int(marker_id): float(size) for marker_id, size in zip(ids, values)}

    def _pose_estimator_for_size(self, size_m: float) -> PoseEstimator:
        estimator = self._pose_estimators.get(size_m)
        if estimator is None:
            estimator = PoseEstimator(size_m, self.get_parameter("solvepnp_method").value,
                                      float(self.get_parameter("max_reprojection_error_px").value),
                                      float(self.get_parameter("min_marker_depth_m").value),
                                      float(self.get_parameter("max_marker_depth_m").value),
                                      **self._pose_estimator_kwargs)
            self._pose_estimators[size_m] = estimator
        return estimator

    def _pose_estimator_for_marker(self, marker_id: int) -> PoseEstimator:
        return self._pose_estimator_for_size(self._marker_sizes.get(marker_id, self._default_marker_size_m))

    def _camera_info_callback(self, message: CameraInfo) -> None:
        if message.width == 0 or message.height == 0 or len(message.k) != 9 or message.k[0] <= 0 or message.k[4] <= 0:
            self.get_logger().warning("Received invalid CameraInfo; retaining no calibration.")
            return
        self.camera_info = message
        self._camera_info_received_monotonic = time.monotonic()

    def _to_bgr(self, message: Image) -> np.ndarray:
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
        if image is None or image.size == 0:
            raise ValueError("Received empty RGB image.")
        encoding = message.encoding.lower()
        if encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if encoding == "bgr8":
            return image
        if encoding in ("mono8", "8uc1"):
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        raise ValueError(f"Unsupported RGB encoding '{message.encoding}'; expected rgb8, bgr8, or mono8.")

    def _depth_callback(self, depth_message: Image) -> None:
        """Part 2: depth arrival is fully decoupled from RGB detection. Depth
        frames are only buffered here; _rgb_callback pulls the closest match
        (if any) independently, so a depth stall never blocks this callback
        from feeding the buffer, and never blocks 2D detection either."""
        if depth_message.width == 0 or depth_message.height == 0 or not depth_message.data:
            self.get_logger().warning("Ignoring empty aligned depth frame.")
            return
        stamp = depth_message.header.stamp
        timestamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        self._depth_buffer.push(timestamp, depth_message)

    def _detect_markers(self, bgr: np.ndarray
                       ) -> tuple[list[ValidatedMarker], dict[int, tuple[float, int]], int]:
        """Stage 1 detection, with an optional Part-10 fallback pass.

        Pass 1 is the existing, unmodified detection path. If it finds zero
        markers and use_adaptive_multiscale_detection is enabled, a single
        secondary pass with CLAHE-enhanced preprocessing is tried (reusing
        bgr_to_gray's existing CLAHE support, not a new algorithm); if that
        pass finds markers, it is used, otherwise the (still empty) first
        pass's result stands. Disabled by default: zero behavior change
        unless explicitly enabled, and never more than one extra pass.

        Returns (markers, quality, rejected_by_quality_count)."""
        gray = bgr_to_gray(bgr, self.get_parameter("use_clahe").value,
                           float(self.get_parameter("clahe_clip_limit").value),
                           int(self.get_parameter("clahe_tile_grid_size").value))
        result = self._validate_and_gate(gray, bgr)
        if result[0] or not self.get_parameter("use_adaptive_multiscale_detection").value:
            return result
        fallback_gray = bgr_to_gray(bgr, True, float(self.get_parameter("clahe_clip_limit").value),
                                    int(self.get_parameter("clahe_tile_grid_size").value))
        fallback = self._validate_and_gate(fallback_gray, bgr)
        return fallback if fallback[0] else result

    def _validate_and_gate(self, gray: np.ndarray, bgr: np.ndarray
                           ) -> tuple[list[ValidatedMarker], dict[int, tuple[float, int]], int]:
        detected = [valid for raw in self.detector.detect(gray)
                   if (valid := self.validator.validate(raw, bgr.shape[1], bgr.shape[0])) is not None]
        markers, quality = self._apply_image_quality_gate(gray, detected)
        return markers, quality, len(detected) - len(markers)

    def _rgb_callback(self, rgb_message: Image) -> None:
        """Part 2 (critical fix): 2D ArUco detection runs on every RGB frame,
        unconditionally. A matching depth frame (within sync_slop) is used for
        3D association/PnP fusion when available; when it is not, detection,
        quality gating, and monocular PnP still proceed and publish normally
        -- only the depth-derived 3D position is marked unavailable (never
        fabricated). Depth returning simply means the next frame finds a
        match again; nothing needs to be explicitly "resumed"."""
        started = time.perf_counter()
        self._last_rgb_frame = time.monotonic()
        try:
            bgr = self._to_bgr(rgb_message)
            markers, quality, rejected_by_quality = self._detect_markers(bgr)
            depth_match = self._depth_buffer.match(
                float(rgb_message.header.stamp.sec) + float(rgb_message.header.stamp.nanosec) * 1e-9)
            depth_started = time.perf_counter()
            if depth_match is None:
                associated = self._invalid_associations(markers, quality, bgr.shape[:2])
                self._counts["frames_without_depth"] += 1
            else:
                depth_message = depth_match.payload
                depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough")
                if depth is None or depth.size == 0:
                    raise ValueError("Matched aligned depth frame is empty.")
                if depth.shape[:2] != bgr.shape[:2]:
                    self.get_logger().error(
                        f"Aligned depth dimensions {depth.shape[:2]} do not match RGB dimensions {bgr.shape[:2]}; "
                        "skipping 3D association for this frame.")
                    associated = self._invalid_associations(markers, quality, bgr.shape[:2])
                    self._counts["frames_without_depth"] += 1
                else:
                    depth_m = self.depth_processor.to_meters(depth, depth_message.encoding)
                    associated = self._associate_3d(markers, depth_m, quality)
                    self._last_synchronized_frame = time.monotonic()
                    self._sync_deltas.append(depth_match.delta_s)
                    self._counts["frames_with_depth"] += 1
            depth_elapsed_ms = (time.perf_counter() - depth_started) * 1000.0
            if depth_elapsed_ms > self.get_parameter("max_depth_processing_time_ms").value:
                self.get_logger().warning(f"Depth association took {depth_elapsed_ms:.2f} ms.")
        except (CvBridgeError, ValueError, cv2.error) as error:
            self.get_logger().error(f"Skipping invalid RGB frame: {error}")
            return
        if self.camera_info is None and not self._camera_info_warned:
            self.get_logger().warning("No valid CameraInfo yet; 2D detections continue, but future pose estimation is unavailable.")
            self._camera_info_warned = True
        stamp = rgb_message.header.stamp
        timestamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        self._counts["detections"] += len(markers)
        self._counts["rejected_quality"] += rejected_by_quality
        self._publish_detections(rgb_message, markers, quality)
        self._publish_3d_detections(rgb_message, associated)
        poses = self._estimate_poses(associated, timestamp)
        self._counts["rejected_pose"] += sum(1 for item in poses if not item.valid)
        self._publish_poses(rgb_message, poses)
        tracked = self._track_poses(poses, rgb_message)
        for track in tracked:
            if track.state == TrackState.CONFIRMED:
                self._counts["confirmed"] += 1
            elif track.state == TrackState.DEGRADED:
                self._counts["degraded"] += 1
            elif track.state == TrackState.LOST:
                self._counts["lost"] += 1
        self._publish_tracked_poses(rgb_message, tracked)
        elapsed = time.perf_counter() - started
        self._processing_times.append(elapsed)
        if not self._competition_mode:
            if self.get_parameter("publish_debug_image").value:
                self._publish_debug(rgb_message, bgr, markers, elapsed)
            if self.get_parameter("publish_3d_debug_image").value:
                self._publish_3d_debug(rgb_message, bgr, associated)
            if self.get_parameter("publish_pose_debug_image").value:
                self._publish_pose_debug(rgb_message, bgr, poses)
            if self.get_parameter("publish_tracking_debug_image").value:
                self._publish_tracking_debug(rgb_message, bgr, tracked)
        self._log_stats_periodically(len(markers))

    def _apply_image_quality_gate(self, gray: np.ndarray, markers: list[ValidatedMarker]
                                  ) -> tuple[list[ValidatedMarker], dict[int, tuple[float, int]]]:
        """Improvement #7: blur/contrast/geometry checks, independent of depth or pose.

        Returns the surviving markers and a detection_confidence/quality_flags lookup
        keyed by object identity (robust even if two detections somehow share an ID).
        """
        laplacian = image_quality.full_frame_laplacian(gray)
        shrink = float(self.get_parameter("image_quality_shrink_ratio").value)
        min_blur, min_blur_hard = self.get_parameter("min_blur_score").value, self.get_parameter("min_blur_score_hard").value
        min_contrast, min_contrast_hard = self.get_parameter("min_contrast_score").value, self.get_parameter("min_contrast_score_hard").value
        max_cv = float(self.get_parameter("max_side_length_cv").value)
        min_pose_size = float(self.get_parameter("min_pose_quality_marker_size_px").value)
        kept: list[ValidatedMarker] = []
        quality: dict[int, tuple[float, int]] = {}
        for marker in markers:
            metrics = image_quality.assess(gray, marker.corners, laplacian, shrink)
            if not metrics.convex or metrics.side_length_cv > max_cv:
                continue  # Structurally invalid quadrilateral: hard reject, not just a flag.
            if metrics.blur_score < min_blur_hard or metrics.contrast_score < min_contrast_hard:
                continue  # Far too blurred/flat to trust for any downstream stage.
            flags = QualityFlag.VALID
            if metrics.blur_score < min_blur or metrics.contrast_score < min_contrast:
                flags |= QualityFlag.LOW_IMAGE_QUALITY
            bounds = marker.corners.max(axis=0) - marker.corners.min(axis=0)
            if float(min(bounds)) < min_pose_size:
                flags |= QualityFlag.LOW_MARKER_RESOLUTION
            blur_quality = min(1.0, metrics.blur_score / max(min_blur, 1e-6))
            contrast_quality = min(1.0, metrics.contrast_score / max(min_contrast, 1e-6))
            geometry_quality = max(0.0, 1.0 - min(metrics.side_length_cv, 1.0))
            confidence = max(0.0, min(1.0, (blur_quality + contrast_quality + geometry_quality) / 3.0))
            quality[id(marker)] = (confidence, int(flags))
            kept.append(marker)
        return kept, quality

    def _associate_3d(self, markers: list[ValidatedMarker], depth_m: np.ndarray,
                      quality: dict[int, tuple[float, int]]) -> list[AssociatedMarker]:
        model = RgbCameraModel.from_camera_info(self.camera_info) if self.camera_info else None
        associated: list[AssociatedMarker] = []
        for marker in markers:
            estimate = self.depth_processor.estimate(depth_m, marker.corners)
            position = model.deproject(*marker.center, estimate.depth_m) if estimate.valid and model else None
            confidence, flags = quality.get(id(marker), (0.0, int(QualityFlag.VALID)))
            if not estimate.valid:
                flags |= int(QualityFlag.LOW_DEPTH_QUALITY)
            associated.append(AssociatedMarker(marker, estimate, position, confidence, flags))
        return associated

    def _invalid_associations(self, markers: list[ValidatedMarker], quality: dict[int, tuple[float, int]],
                              shape: tuple[int, int]) -> list[AssociatedMarker]:
        empty = np.zeros(shape, dtype=np.uint8)
        result = []
        for marker in markers:
            confidence, flags = quality.get(id(marker), (0.0, int(QualityFlag.VALID)))
            result.append(AssociatedMarker(
                marker, DepthEstimate(False, float("nan"), 0, 0.0, float("nan"), empty), None,
                confidence, flags | int(QualityFlag.LOW_DEPTH_QUALITY)))
        return result

    def _depth_z_variance(self, estimate: DepthEstimate) -> float | None:
        """Variance of the robust median depth estimate (Improvement #3), from sample
        dispersion (MAD) and count -- not a fixed constant. See depth_processor.DepthEstimate."""
        if not estimate.valid or estimate.valid_samples < 1:
            return None
        std = 1.4826 * estimate.mad_m  # MAD -> Gaussian-equivalent std.
        # Approximate variance of a sample median under near-Gaussian noise: (pi/2) * var / n.
        variance = (std ** 2) * (math.pi / 2.0) / estimate.valid_samples
        return max(variance, float(self.get_parameter("min_depth_variance_m2").value))

    def _previous_pose(self, marker_id: int, timestamp: float
                       ) -> tuple[np.ndarray | None, np.ndarray | None, float | None]:
        """Improvement #1: look up the existing Step-4 track (if any) to disambiguate
        this frame's IPPE candidates. Read-only; does not mutate tracker state."""
        track = self.tracker_manager.tracks.get(marker_id)
        if track is None:
            return None, None, None
        return track.filter.state[:3].copy(), track.orientation.copy(), max(0.0, timestamp - track.last_timestamp)

    def _estimate_poses(self, associated: list[AssociatedMarker], timestamp: float) -> list[AssociatedPose]:
        output: list[AssociatedPose] = []
        for item in associated:
            estimator = self._pose_estimator_for_marker(item.marker.marker_id)
            depth_position = np.asarray(item.position) if item.position is not None else None
            depth_z_variance = self._depth_z_variance(item.depth) if item.position is not None else None
            previous_position, previous_orientation, dt = self._previous_pose(item.marker.marker_id, timestamp)
            pose = estimator.estimate(item.marker.corners, self.camera_info, depth_position=depth_position,
                                      depth_z_variance=depth_z_variance, previous_position=previous_position,
                                      previous_orientation=previous_orientation, dt_since_previous_s=dt)
            difference = (float(np.linalg.norm(np.asarray(item.position) - pose.tvec.reshape(3)))
                          if item.position is not None and pose.tvec is not None else float("nan"))
            depth_consistent = (not self.get_parameter("validate_against_depth").value or np.isnan(difference) or
                                difference <= self.get_parameter("max_depth_position_difference_m").value)
            flags = int(QualityFlag(item.quality_flags) | QualityFlag(pose.quality_flags))
            output.append(AssociatedPose(item, pose, difference, pose.valid and depth_consistent, flags))
        return output

    def _report_sync_health(self) -> None:
        """Report absent RGB frames and absent depth-matched pairs at a bounded
        rate, without console flooding. These are reported separately (Part 2):
        depth loss alone no longer implies RGB/detection loss."""
        now = time.monotonic()
        if now - self._last_rgb_frame >= 5.0:
            self.get_logger().warning("No RGB frame received in the last 5 seconds; check rgb_topic.")
        elif now - self._last_synchronized_frame >= 5.0:
            self.get_logger().warning(
                "No RGB/aligned-depth match in the last 5 seconds (RGB detection continues 2D-only); "
                "check depth_topic and sync_slop.")

    def _publish_detections(self, image: Image, markers: list[ValidatedMarker],
                            quality: dict[int, tuple[float, int]]) -> None:
        output = MarkerDetectionArray()
        output.header = image.header
        for marker in markers:
            message = MarkerDetection()
            message.id, message.center_x, message.center_y, message.area_px = (
                marker.marker_id, marker.center[0], marker.center[1], marker.area_px)
            flattened = marker.corners.flatten().tolist()
            (message.corner_1_x, message.corner_1_y, message.corner_2_x, message.corner_2_y,
             message.corner_3_x, message.corner_3_y, message.corner_4_x, message.corner_4_y) = flattened
            confidence, flags = quality.get(id(marker), (0.0, 0))
            message.detection_confidence, message.quality_flags = confidence, flags
            output.markers.append(message)
        self.detection_pub.publish(output)

    def _publish_3d_detections(self, image: Image, associated: list[AssociatedMarker]) -> None:
        output = Marker3DDetectionArray()
        output.header = image.header
        for item in associated:
            marker, estimate = item.marker, item.depth
            message = Marker3DDetection()
            message.header = image.header
            message.id, message.center_x, message.center_y, message.area_px = (
                marker.marker_id, marker.center[0], marker.center[1], marker.area_px)
            (message.corner_1_x, message.corner_1_y, message.corner_2_x, message.corner_2_y,
             message.corner_3_x, message.corner_3_y, message.corner_4_x, message.corner_4_y) = marker.corners.flatten().tolist()
            message.valid_depth = estimate.valid
            message.valid_depth_samples = estimate.valid_samples
            message.depth_quality = estimate.quality
            message.depth_mad = estimate.mad_m
            message.depth_m = estimate.depth_m
            variance = self._depth_z_variance(estimate)
            message.depth_z_variance = variance if variance is not None else float("nan")
            if item.position is not None:
                message.depth_m = estimate.depth_m
                message.position_x, message.position_y, message.position_z = item.position
                message.valid_3d = True
            else:
                message.valid_3d = False
            output.markers.append(message)
        self.detection_3d_pub.publish(output)

    def _publish_poses(self, image: Image, poses: list[AssociatedPose]) -> None:
        output = MarkerPoseArray()
        output.header = image.header
        for item in poses:
            marker, pose = item.association.marker, item.pose
            message = MarkerPose()
            message.header = image.header
            message.id = marker.marker_id
            message.reprojection_error_px = pose.mean_reprojection_error_px
            message.max_reprojection_error_px = pose.max_reprojection_error_px
            message.position_difference_m = item.position_difference_m
            message.viewing_angle_deg = pose.viewing_angle_deg
            message.detection_confidence = item.association.detection_confidence
            message.pose_quality = pose.pose_quality
            message.quality_flags = item.quality_flags
            if pose.position_noise is not None:
                covariance = np.zeros((3, 3))
                covariance[:, :] = pose.position_noise
                message.position_covariance = covariance.reshape(9).tolist()
            if item.association.position is not None:
                message.depth_position_x, message.depth_position_y, message.depth_position_z = item.association.position
            if pose.tvec is not None:
                message.position.x, message.position.y, message.position.z = pose.tvec.reshape(3).tolist()
            if pose.quaternion is not None:
                message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w = pose.quaternion
            if pose.euler is not None and self.get_parameter("publish_euler_angles").value:
                message.roll, message.pitch, message.yaw = pose.euler
            message.pose_valid = item.valid
            output.markers.append(message)
        self.raw_pose_pub.publish(output)

    def _track_poses(self, poses: list[AssociatedPose], image: Image) -> list[TrackedPose]:
        raw = []
        for item in poses:
            pose, association = item.pose, item.association
            depth_z = association.position[2] if association.position is not None else None
            depth_variance = self._depth_z_variance(association.depth) if association.position is not None else None
            if pose.tvec is None or pose.quaternion is None:
                raw.append(RawPose(association.marker.marker_id, np.full(3, np.nan), np.full(4, np.nan), False,
                                   pose.mean_reprojection_error_px, quality_flags=item.quality_flags))
            else:
                raw.append(RawPose(association.marker.marker_id, pose.tvec.reshape(3), np.asarray(pose.quaternion),
                                   item.valid, pose.mean_reprojection_error_px, position_noise=pose.position_noise,
                                   depth_z_m=depth_z, depth_z_variance=depth_variance,
                                   detection_confidence=association.detection_confidence, pose_quality=pose.pose_quality,
                                   viewing_angle_deg=pose.viewing_angle_deg, quality_flags=item.quality_flags))
        stamp = image.header.stamp
        return self.tracker_manager.process(raw, float(stamp.sec) + float(stamp.nanosec) * 1e-9)

    def _publish_tracked_poses(self, image: Image, tracks: list[TrackedPose]) -> None:
        output = MarkerTrackedPoseArray()
        output.header = image.header
        for track in tracks:
            message = MarkerTrackedPose()
            message.header = image.header
            message.id = track.marker_id
            message.position.x, message.position.y, message.position.z = track.position.tolist()
            message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w = track.orientation.tolist()
            message.detection_valid, message.tracking_valid, message.track_state = track.detection_valid, track.tracking_valid, track.state.value
            message.confidence, message.reprojection_error_px = track.confidence, track.reprojection_error_px
            message.position_innovation_m, message.orientation_innovation_deg = track.position_innovation_m, track.orientation_innovation_deg
            message.detection_count, message.missed_frames = track.detection_count, track.missed_frames
            message.detection_confidence, message.pose_quality = track.detection_confidence, track.pose_quality
            message.tracking_confidence, message.viewing_angle_deg = track.tracking_confidence, track.viewing_angle_deg
            message.distance_m, message.quality_flags, message.depth_fused = track.distance_m, track.quality_flags, track.depth_fused
            message.age_seconds = track.age_seconds
            if track.position_covariance is not None:
                covariance = np.zeros((3, 3))
                covariance[:, :] = track.position_covariance
                message.position_covariance = covariance.reshape(9).tolist()
            output.markers.append(message)
        self.tracked_pose_pub.publish(output)

    def _publish_debug(self, image: Image, bgr: np.ndarray, markers: list[ValidatedMarker], elapsed: float) -> None:
        debug = bgr.copy()
        for marker in markers:
            points = marker.corners.astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(debug, [points], True, (0, 255, 0), 2)
            center = tuple(round(value) for value in marker.center)
            cv2.circle(debug, center, 4, (0, 0, 255), -1)
            cv2.putText(debug, f"ID: {marker.marker_id}", (center[0] + 6, center[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        if self.get_parameter("show_debug_info").value:
            cv2.putText(debug, f"markers: {len(markers)}  {elapsed * 1000:.1f} ms", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        debug_message = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        debug_message.header = image.header
        self.debug_pub.publish(debug_message)

    def _publish_3d_debug(self, image: Image, bgr: np.ndarray, associated: list[AssociatedMarker]) -> None:
        debug = bgr.copy()
        for item in associated:
            marker, estimate = item.marker, item.depth
            colour = (0, 255, 0) if item.position is not None else (0, 0, 255)
            cv2.polylines(debug, [marker.corners.astype(np.int32).reshape((-1, 1, 2))], True, colour, 2)
            debug[estimate.roi_mask > 0] = (0.4 * debug[estimate.roi_mask > 0] + 0.6 * np.array(colour)).astype(np.uint8)
            center = tuple(round(value) for value in marker.center)
            label = (f"ID {marker.marker_id}: X={item.position[0]:.2f} Y={item.position[1]:.2f} Z={item.position[2]:.2f}m n={estimate.valid_samples}"
                     if item.position is not None else f"ID {marker.marker_id}: NO VALID DEPTH (n={estimate.valid_samples})")
            cv2.putText(debug, label, (center[0] + 6, center[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 2)
        output = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        output.header = image.header
        self.debug_3d_pub.publish(output)

    def _publish_pose_debug(self, image: Image, bgr: np.ndarray, poses: list[AssociatedPose]) -> None:
        debug = bgr.copy()
        camera = PoseEstimator.camera_parameters(self.camera_info) if self.camera_info else None
        for item in poses:
            marker, pose = item.association.marker, item.pose
            colour = (0, 255, 0) if item.valid else (0, 0, 255)
            cv2.polylines(debug, [marker.corners.astype(np.int32).reshape((-1, 1, 2))], True, colour, 2)
            center = tuple(round(value) for value in marker.center)
            if pose.valid and pose.tvec is not None and pose.rvec is not None:
                if camera and self.get_parameter("draw_pose_axes").value:
                    size = self._marker_sizes.get(marker.marker_id, self._default_marker_size_m)
                    cv2.drawFrameAxes(debug, camera[0], camera[1], pose.rvec, pose.tvec, size * 0.5)
                x, y, z = pose.tvec.reshape(3)
                label = (f"ID {marker.marker_id}: X={x:.2f} Y={y:.2f} Z={z:.2f}m reproj={pose.mean_reprojection_error_px:.2f}px "
                        f"ang={pose.viewing_angle_deg:.0f}deg q={pose.pose_quality:.2f}")
            else:
                label = f"ID {marker.marker_id}: INVALID POSE"
            cv2.putText(debug, label, (center[0] + 6, center[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.42, colour, 2)
        output = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        output.header = image.header
        self.pose_debug_pub.publish(output)

    def _publish_tracking_debug(self, image: Image, bgr: np.ndarray, tracks: list[TrackedPose]) -> None:
        """Tracking diagnostics; predicted DEGRADED/LOST poses are explicitly labelled as unobserved."""
        debug = bgr.copy()
        y = 24
        colours = {TrackState.CONFIRMED: (0, 255, 0), TrackState.NEW: (255, 255, 0),
                  TrackState.DEGRADED: (0, 165, 255), TrackState.LOST: (0, 0, 255)}
        for track in tracks:
            colour = colours.get(track.state, (255, 255, 255))
            text = (f"ID {track.marker_id} {track.state.value} conf={track.confidence:.2f} "
                    f"(det={track.detection_confidence:.2f} pose={track.pose_quality:.2f} trk={track.tracking_confidence:.2f}) "
                    f"XYZ=({track.position[0]:.2f},{track.position[1]:.2f},{track.position[2]:.2f}) "
                    f"reproj={track.reprojection_error_px:.2f}px innov={track.position_innovation_m:.2f}m/"
                    f"{track.orientation_innovation_deg:.1f}deg miss={track.missed_frames} depth_fused={track.depth_fused}")
            cv2.putText(debug, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.40, colour, 1)
            y += 18
        output = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        output.header = image.header
        self.tracking_debug_pub.publish(output)

    def _log_stats_periodically(self, count: int) -> None:
        now = time.monotonic()
        if now - self._last_stats_log < 5.0 or not self._processing_times:
            return
        values = list(self._processing_times)
        level = self.get_logger().debug if self._competition_mode else self.get_logger().info
        level("markers=%d, processing avg=%.2f ms max=%.2f ms, effective=%.1f Hz" % (
            count, 1000 * sum(values) / len(values), 1000 * max(values), len(values) / 5.0))
        self._last_stats_log = now

    def _publish_diagnostics(self) -> None:
        """Improvements #14 (diagnostics) and #15 (performance): a single throttled
        DiagnosticArray summarizing sync health, latency percentiles, and pipeline counts."""
        now = time.monotonic()
        window = max(now - self._diag_window_start, 1e-6)
        stats = diag.latency_stats(list(self._processing_times), window)
        # Part 8: RGB-depth sync-delta distribution over matched pairs, and
        # RGB/sync liveness reported as two independent gaps (Part 2: a depth
        # stall alone must not read as "the pipeline is unhealthy" the same
        # way an RGB stall does).
        sync_stats = diag.sync_delta_stats(list(self._sync_deltas))
        rgb_gap = now - self._last_rgb_frame
        sync_gap = now - self._last_synchronized_frame
        rgb_level = diag.synchronization_level(rgb_gap, float(self.get_parameter("sync_stale_warn_s").value))
        latency_level = diag.latency_level(stats, float(self.get_parameter("latency_warn_p95_ms").value),
                                           float(self.get_parameter("latency_error_p95_ms").value))
        # Overall health follows RGB/latency; a depth-matched-pair gap alone is
        # reported (sync_level) but does not by itself make the node ERROR,
        # since 2D detection is expected to keep working through it.
        level = diag.overall_level(rgb_level, latency_level)
        status = DiagnosticStatus()
        status.name, status.hardware_id = "marker_detection: pipeline", "realsense_d435"
        status.level = _DIAG_LEVEL[level]
        status.message = {diag.HealthLevel.OK: "Nominal", diag.HealthLevel.WARN: "Degraded",
                          diag.HealthLevel.ERROR: "Unhealthy"}[level]
        camera_info_age_s = (now - self._camera_info_received_monotonic
                            if self.camera_info is not None and self._camera_info_received_monotonic is not None
                            else float("nan"))
        status.values = [
            KeyValue(key="mode", value="competition" if self._competition_mode else "development"),
            KeyValue(key="processing_fps", value=f"{stats.fps:.2f}"),
            KeyValue(key="latency_avg_ms", value=f"{stats.avg_ms:.2f}"),
            KeyValue(key="latency_p50_ms", value=f"{stats.p50_ms:.2f}"),
            KeyValue(key="latency_p95_ms", value=f"{stats.p95_ms:.2f}"),
            KeyValue(key="latency_p99_ms", value=f"{stats.p99_ms:.2f}"),
            KeyValue(key="latency_max_ms", value=f"{stats.max_ms:.2f}"),
            KeyValue(key="seconds_since_last_rgb_frame", value=f"{rgb_gap:.2f}"),
            KeyValue(key="seconds_since_last_sync_pair", value=f"{sync_gap:.2f}"),
            KeyValue(key="sync_delta_p50_ms", value=f"{sync_stats.p50_ms:.2f}"),
            KeyValue(key="sync_delta_p95_ms", value=f"{sync_stats.p95_ms:.2f}"),
            KeyValue(key="sync_delta_max_ms", value=f"{sync_stats.max_ms:.2f}"),
            KeyValue(key="frames_with_depth", value=str(self._counts["frames_with_depth"])),
            KeyValue(key="frames_without_depth", value=str(self._counts["frames_without_depth"])),
            KeyValue(key="camera_info_age_s",
                    value=f"{camera_info_age_s:.2f}" if np.isfinite(camera_info_age_s) else "nan"),
            KeyValue(key="camera_info_received", value=str(self.camera_info is not None)),
            KeyValue(key="detections", value=str(self._counts["detections"])),
            KeyValue(key="rejected_by_image_quality", value=str(self._counts["rejected_quality"])),
            KeyValue(key="rejected_pose", value=str(self._counts["rejected_pose"])),
            KeyValue(key="tracks_confirmed", value=str(self._counts["confirmed"])),
            KeyValue(key="tracks_degraded", value=str(self._counts["degraded"])),
            KeyValue(key="tracks_lost", value=str(self._counts["lost"])),
            KeyValue(key="active_tracks", value=str(len(self.tracker_manager.tracks))),
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diagnostics_pub.publish(message)
        self._diag_window_start = now
        self._sync_deltas.clear()
        self._counts = {key: 0 for key in self._counts}


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MarkerDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
