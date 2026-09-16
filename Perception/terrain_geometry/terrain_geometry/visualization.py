#!/usr/bin/env python3
"""
visualization.py

RViz2 MarkerArray construction for the terrain_geometry package's
Obstacle Feature Extraction stage.

This module has a single responsibility: turn a list of
`ObstacleFeature` objects into a `visualization_msgs/MarkerArray`
containing, for every obstacle:
    - a cube Marker sized/positioned from its bounding box, and
    - a text Marker displaying its obstacle ID.

If an obstacle's `ObstacleFeature` carries populated OBB fields
(`obb_center`, `obb_extents`, `obb_quaternion` -- only present when the
extractor's optional PCA-based OBB computation is enabled), the cube
marker is drawn as that tighter, rotation-aware box instead of the
axis-aligned one; otherwise it falls back to the AABB, exactly as
before. This keeps the default visual output unchanged unless OBB
computation is explicitly turned on upstream.

No feature-extraction math and no ROS subscription/publisher wiring
lives here -- see `obstacle_features.py` and `terrain_node.py`.
"""

from __future__ import annotations

from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from visualization_msgs.msg import Marker, MarkerArray

from terrain_geometry.obstacle_features import ObstacleFeature


class ObstacleMarkerBuilder:
    """Builds a MarkerArray of cube + text markers for detected obstacles.

    Each obstacle contributes exactly two markers, sharing the same
    numeric `id` base so both remain uniquely identifiable:
        - namespace "obstacle_cubes", id = obstacle.id
        - namespace "obstacle_labels", id = obstacle.id
    Using distinct namespaces (rather than distinct numeric ID ranges)
    is the standard RViz2-safe way to keep two logically different
    marker types from colliding, since Marker uniqueness is scoped to
    (namespace, id) pairs, not id alone.
    """

    CUBE_NAMESPACE = "obstacle_cubes"
    LABEL_NAMESPACE = "obstacle_labels"

    # RGBA color for the cube markers (semi-transparent cyan).
    CUBE_COLOR = (0.1, 0.7, 1.0, 0.5)
    # RGBA color for the text labels (solid white).
    LABEL_COLOR = (1.0, 1.0, 1.0, 1.0)

    LABEL_TEXT_HEIGHT = 0.15  # meters
    # How far above the box's top face to float the text label so it
    # doesn't visually overlap the cube itself.
    LABEL_Z_OFFSET = 0.10  # meters

    # A small epsilon floor avoids a degenerate zero-thickness cube for
    # perfectly planar clusters (e.g. a flat wall segment).
    MIN_CUBE_DIMENSION = 0.01  # meters

    def build_marker_array(
        self, obstacles: list[ObstacleFeature], header: Header
    ) -> MarkerArray:
        """Build the full MarkerArray for the given obstacles.

        Args:
            obstacles: List of extracted obstacle features for this
                frame (may be empty).
            header: ROS header (frame_id + stamp) to reuse for every
                marker, so all markers align with the source cloud.

        Returns:
            A `visualization_msgs/MarkerArray`. When `obstacles` is
            empty, this still contains a single DELETEALL marker so
            stale markers from a previous frame are cleared in RViz2
            rather than left behind.
        """
        marker_array = MarkerArray()

        # Clear all previously published markers first. Without this,
        # if the obstacle count drops between frames (e.g. an obstacle
        # leaves the field of view), its old marker would remain
        # displayed forever, since Marker IDs are not automatically
        # garbage-collected by RViz2.
        marker_array.markers.append(self._build_delete_all_marker(header))

        for obstacle in obstacles:
            marker_array.markers.append(self._build_cube_marker(obstacle, header))
            marker_array.markers.append(self._build_text_marker(obstacle, header))

        return marker_array

    def _build_delete_all_marker(self, header: Header) -> Marker:
        """Build a marker that deletes every previously published marker."""
        marker = Marker()
        marker.header = header
        marker.ns = "cleanup"
        marker.id = 999999
        marker.action = Marker.DELETEALL
        return marker

    def _build_cube_marker(self, obstacle: ObstacleFeature, header: Header) -> Marker:
        """Build a cube Marker representing one obstacle's bounding box.

        Uses the PCA-based OBB (tighter, rotation-aware) when present
        on `obstacle`; otherwise falls back to the axis-aligned box
        (identity orientation), exactly matching the previous behavior.
        """
        marker = Marker()
        marker.header = header
        marker.ns = self.CUBE_NAMESPACE
        marker.id = obstacle.id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        has_obb = (
            obstacle.obb_center is not None
            and obstacle.obb_extents is not None
            and obstacle.obb_quaternion is not None
        )

        pose = Pose()
        if has_obb:
            pose.position = Point(
                x=float(obstacle.obb_center[0]),
                y=float(obstacle.obb_center[1]),
                z=float(obstacle.obb_center[2]),
            )
            qx, qy, qz, qw = obstacle.obb_quaternion
            pose.orientation = Quaternion(
                x=float(qx), y=float(qy), z=float(qz), w=float(qw)
            )
            marker.scale = Vector3(
                x=max(float(obstacle.obb_extents[0]), self.MIN_CUBE_DIMENSION),
                y=max(float(obstacle.obb_extents[1]), self.MIN_CUBE_DIMENSION),
                z=max(float(obstacle.obb_extents[2]), self.MIN_CUBE_DIMENSION),
            )
        else:
            pose.position = Point(
                x=float(obstacle.centroid[0]),
                y=float(obstacle.centroid[1]),
                z=float(obstacle.centroid[2]),
            )
            pose.orientation.w = 1.0  # Axis-aligned: identity rotation.
            # AABB extents map directly to cube scale (depth=X, width=Y,
            # height=Z), matching the ObstacleFeature dimension
            # convention.
            marker.scale = Vector3(
                x=max(obstacle.depth, self.MIN_CUBE_DIMENSION),
                y=max(obstacle.width, self.MIN_CUBE_DIMENSION),
                z=max(obstacle.height, self.MIN_CUBE_DIMENSION),
            )
        marker.pose = pose

        r, g, b, a = self.CUBE_COLOR
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

        # Persist until explicitly replaced/deleted next frame rather
        # than auto-expiring, since publish rate can vary with sensor
        # frame rate.
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        marker.frame_locked = False

        return marker

    def _build_text_marker(self, obstacle: ObstacleFeature, header: Header) -> Marker:
        """Build a text Marker displaying the obstacle's ID above its box."""
        marker = Marker()
        marker.header = header
        marker.ns = self.LABEL_NAMESPACE
        marker.id = obstacle.id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Float the label above whichever box is being rendered: the
        # OBB's own top (center.z + half its z-extent) when present,
        # otherwise the AABB's top face -- so the label never buries
        # itself inside a rotated box.
        if obstacle.obb_center is not None and obstacle.obb_extents is not None:
            top_z = float(obstacle.obb_center[2]) + float(obstacle.obb_extents[2]) / 2.0
            label_x = float(obstacle.obb_center[0])
            label_y = float(obstacle.obb_center[1])
        else:
            top_z = float(obstacle.max_point[2])
            label_x = float(obstacle.centroid[0])
            label_y = float(obstacle.centroid[1])

        pose = Pose()
        pose.position = Point(
            x=label_x,
            y=label_y,
            z=top_z + self.LABEL_Z_OFFSET,
        )
        pose.orientation.w = 1.0
        marker.pose = pose

        marker.scale.z = self.LABEL_TEXT_HEIGHT  # Only scale.z is used for text height.

        r, g, b, a = self.LABEL_COLOR
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

        marker.text = f"ID {obstacle.id}"

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        marker.frame_locked = False

        return marker
