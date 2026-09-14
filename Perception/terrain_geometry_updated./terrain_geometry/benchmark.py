#!/usr/bin/env python3
"""
benchmark.py

Standalone offline benchmark for the terrain_geometry perception pipeline.

Deliberately NOT part of the production node/callback path (terrain_node.py
never imports this module) -- per the task's requirement that benchmarking
support must not become a mandatory part of the real-time production node.
Run it manually, from the command line, against either synthetic point
clouds or a recorded ros2 bag.

Measures, per run:
    - average / minimum / maximum / standard deviation of total
      per-frame processing time
    - p95 and maximum latency (more informative than average FPS alone
      for a real-time perception pipeline, since it surfaces occasional
      slow frames that an average would hide)
    - estimated average FPS
    - point count at every pipeline stage (averaged across frames)

USAGE:
    # Synthetic point clouds -- no ROS bag required, useful for quick
    # local benchmarking or CI:
    python3 benchmark.py --synthetic --frames 100

    # Replay a recorded PointCloud2 topic from a ros2 bag:
    python3 benchmark.py --bag /path/to/bag --topic /camera/depth/color/points

Only depends on rosbag2_py / rclpy.serialization for the --bag path; the
--synthetic path has no ROS dependency at all and will run in a plain
Python + NumPy/SciPy/scikit-learn environment.
"""

from __future__ import annotations

import argparse
import statistics
import time
from dataclasses import dataclass, field
from typing import Iterator, List, Optional

import numpy as np

from terrain_geometry.roi_filter import ROIFilter
from terrain_geometry.ground_removal import GroundRemoval, GroundNotFoundError
from terrain_geometry.voxel_filter import VoxelFilter, VoxelFilterError
from terrain_geometry.outlier_filter import RadiusOutlierFilter
from terrain_geometry.clustering import DBSCANClusterer
from terrain_geometry.obstacle_features import (
    ObstacleFeatureExtractor,
    group_points_by_label,
)
from terrain_geometry.obstacle_tracking import ObstacleTracker


# --------------------------------------------------------------------------- #
# Per-frame result bookkeeping
# --------------------------------------------------------------------------- #


@dataclass
class FrameResult:
    """Timing and point-count outcome for a single benchmarked frame."""

    total_ms: float
    input_count: int
    roi_count: int
    non_ground_count: int
    voxel_count: int
    filtered_count: int
    cluster_count: int


@dataclass
class BenchmarkSummary:
    """Aggregate statistics across every benchmarked frame."""

    num_frames: int
    avg_ms: float
    min_ms: float
    max_ms: float
    stddev_ms: float
    p95_ms: float
    avg_fps: float
    avg_input_count: float
    avg_roi_count: float
    avg_non_ground_count: float
    avg_voxel_count: float
    avg_filtered_count: float
    avg_cluster_count: float

    def format_report(self) -> str:
        return (
            "TerrainGeometry benchmark summary "
            f"({self.num_frames} frames)\n"
            f"  latency:  avg={self.avg_ms:.2f} ms  min={self.min_ms:.2f} ms  "
            f"max={self.max_ms:.2f} ms  stddev={self.stddev_ms:.2f} ms  "
            f"p95={self.p95_ms:.2f} ms\n"
            f"  throughput: avg_fps={self.avg_fps:.1f}\n"
            f"  point counts (avg/frame): input={self.avg_input_count:.0f} "
            f"roi={self.avg_roi_count:.0f} non_ground={self.avg_non_ground_count:.0f} "
            f"voxel={self.avg_voxel_count:.0f} filtered={self.avg_filtered_count:.0f} "
            f"clusters={self.avg_cluster_count:.1f}"
        )


def summarize(results: List[FrameResult]) -> BenchmarkSummary:
    """Reduces a list of per-frame FrameResult into aggregate BenchmarkSummary."""
    if not results:
        raise ValueError("Cannot summarize zero frames.")

    totals = [r.total_ms for r in results]
    totals_sorted = sorted(totals)
    p95_index = min(int(round(0.95 * (len(totals_sorted) - 1))), len(totals_sorted) - 1)

    avg_ms = statistics.mean(totals)
    return BenchmarkSummary(
        num_frames=len(results),
        avg_ms=avg_ms,
        min_ms=min(totals),
        max_ms=max(totals),
        stddev_ms=statistics.stdev(totals) if len(totals) > 1 else 0.0,
        p95_ms=totals_sorted[p95_index],
        avg_fps=(1000.0 / avg_ms) if avg_ms > 0.0 else 0.0,
        avg_input_count=statistics.mean(r.input_count for r in results),
        avg_roi_count=statistics.mean(r.roi_count for r in results),
        avg_non_ground_count=statistics.mean(r.non_ground_count for r in results),
        avg_voxel_count=statistics.mean(r.voxel_count for r in results),
        avg_filtered_count=statistics.mean(r.filtered_count for r in results),
        avg_cluster_count=statistics.mean(r.cluster_count for r in results),
    )


# --------------------------------------------------------------------------- #
# The pipeline under benchmark (mirrors terrain_node.py's _cloud_callback,
# minus all ROS I/O -- pure computation only)
# --------------------------------------------------------------------------- #


class BenchmarkPipeline:
    """Wires up the same stage objects terrain_node.py uses, without ROS."""

    def __init__(
        self,
        enable_tracking: bool = False,
        roi_bounds: Optional[tuple] = None,
    ) -> None:
        min_x, max_x, min_y, max_y, min_z, max_z = roi_bounds or (
            0.2, 8.0, -4.0, 4.0, -0.5, 1.5,
        )
        self.roi = ROIFilter(min_x, max_x, min_y, max_y, min_z, max_z)
        self.ground_removal = GroundRemoval(backend="auto")
        self.voxel = VoxelFilter(0.05, 0.05, 0.05)
        self.outlier = RadiusOutlierFilter(search_radius=0.2, min_neighbors=5)
        self.clusterer = DBSCANClusterer(eps=0.3, min_points=5)
        self.extractor = ObstacleFeatureExtractor()
        self.tracker = ObstacleTracker(0.5, 3, 0.5) if enable_tracking else None

    def run_frame(self, xyz: np.ndarray) -> FrameResult:
        t_start = time.perf_counter()
        input_count = xyz.shape[0]

        roi_xyz = self.roi.filter(xyz)
        if roi_xyz.shape[0] == 0:
            return self._empty_result(t_start, input_count, 0)

        try:
            _ground_xyz, obstacle_xyz = self.ground_removal.segment(roi_xyz)
        except GroundNotFoundError:
            return self._empty_result(t_start, input_count, roi_xyz.shape[0])

        if obstacle_xyz.shape[0] == 0:
            return self._empty_result(
                t_start, input_count, roi_xyz.shape[0], non_ground_count=0
            )

        try:
            voxel_xyz = self.voxel.filter(obstacle_xyz)
        except VoxelFilterError:
            return self._empty_result(
                t_start, input_count, roi_xyz.shape[0], obstacle_xyz.shape[0]
            )

        inlier_xyz, _outliers = self.outlier.filter(voxel_xyz)
        if inlier_xyz.shape[0] == 0:
            return self._empty_result(
                t_start,
                input_count,
                roi_xyz.shape[0],
                obstacle_xyz.shape[0],
                voxel_xyz.shape[0],
            )

        labels, _colors, _summaries = self.clusterer.cluster(inlier_xyz)
        clusters = group_points_by_label(inlier_xyz, labels)
        features = self.extractor.extract_all(clusters) if clusters else []

        if self.tracker is not None:
            features = self.tracker.update(features)

        total_ms = (time.perf_counter() - t_start) * 1000.0
        return FrameResult(
            total_ms=total_ms,
            input_count=input_count,
            roi_count=roi_xyz.shape[0],
            non_ground_count=obstacle_xyz.shape[0],
            voxel_count=voxel_xyz.shape[0],
            filtered_count=inlier_xyz.shape[0],
            cluster_count=len(features),
        )

    @staticmethod
    def _empty_result(
        t_start: float,
        input_count: int,
        roi_count: int,
        non_ground_count: int = 0,
        voxel_count: int = 0,
    ) -> FrameResult:
        total_ms = (time.perf_counter() - t_start) * 1000.0
        return FrameResult(
            total_ms=total_ms,
            input_count=input_count,
            roi_count=roi_count,
            non_ground_count=non_ground_count,
            voxel_count=voxel_count,
            filtered_count=0,
            cluster_count=0,
        )


# --------------------------------------------------------------------------- #
# Synthetic frame generation
# --------------------------------------------------------------------------- #


def synthetic_frames(num_frames: int, seed: int = 42) -> Iterator[np.ndarray]:
    """Yields synthetic point clouds: a ground plane plus 1-3 moving boxes.

    Not a substitute for real sensor data -- useful for a smoke-test /
    relative-performance benchmark without needing a recorded bag.
    """
    rng = np.random.default_rng(seed)
    for i in range(num_frames):
        ground = np.column_stack(
            [
                rng.uniform(-2, 10, 6000),
                rng.uniform(-6, 6, 6000),
                rng.uniform(-0.05, 0.05, 6000),
            ]
        ).astype(np.float32)

        boxes = []
        num_boxes = 1 + (i % 3)
        for b in range(num_boxes):
            cx = 2.0 + 0.05 * i + b * 1.5
            cy = -1.0 + b * 1.0
            box = rng.uniform(
                low=[cx - 0.2, cy - 0.2, 0.0],
                high=[cx + 0.2, cy + 0.2, 0.5],
                size=(200, 3),
            ).astype(np.float32)
            boxes.append(box)

        yield np.vstack([ground] + boxes)


def bag_frames(bag_path: str, topic: str) -> Iterator[np.ndarray]:
    """Yields XYZ arrays decoded from PointCloud2 messages in a ros2 bag.

    Requires rosbag2_py, rclpy, and sensor_msgs -- only imported here (not
    at module load time) so `--synthetic` mode never needs a ROS install.
    """
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import PointCloud2
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(
            "Reading a ros2 bag requires a full ROS 2 install "
            "(rosbag2_py, rclpy, sensor_msgs, rosidl_runtime_py). "
            f"Import failed: {exc}"
        ) from exc

    from terrain_geometry.tf_transform import pointcloud2_to_xyz_array

    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    if topic not in type_map:
        raise ValueError(
            f"Topic '{topic}' not found in bag. Available topics: "
            f"{sorted(type_map.keys())}"
        )
    msg_type = get_message(type_map[topic])

    while reader.has_next():
        read_topic, data, _t = reader.read_next()
        if read_topic != topic:
            continue
        msg: PointCloud2 = deserialize_message(data, msg_type)
        yield pointcloud2_to_xyz_array(msg)


# --------------------------------------------------------------------------- #
# CLI entry point
# --------------------------------------------------------------------------- #


def main() -> None:
    parser = argparse.ArgumentParser(description="Benchmark the terrain_geometry pipeline.")
    source_group = parser.add_mutually_exclusive_group(required=True)
    source_group.add_argument(
        "--synthetic", action="store_true", help="Benchmark against synthetic point clouds."
    )
    source_group.add_argument(
        "--bag", type=str, default=None, help="Path to a ros2 bag directory to replay."
    )
    parser.add_argument(
        "--topic",
        type=str,
        default="/camera/depth/color/points",
        help="PointCloud2 topic to read from the bag (only used with --bag).",
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=100,
        help="Number of synthetic frames to generate (only used with --synthetic).",
    )
    parser.add_argument(
        "--enable-tracking",
        action="store_true",
        help="Run the temporal obstacle tracker as part of each frame.",
    )
    args = parser.parse_args()

    if args.synthetic:
        frame_source: Iterator[np.ndarray] = synthetic_frames(args.frames)
    else:
        frame_source = bag_frames(args.bag, args.topic)

    pipeline = BenchmarkPipeline(enable_tracking=args.enable_tracking)

    results: List[FrameResult] = []
    for xyz in frame_source:
        results.append(pipeline.run_frame(xyz))

    if not results:
        print("No frames were processed -- nothing to report.")
        return

    summary = summarize(results)
    print(summary.format_report())


if __name__ == "__main__":
    main()
