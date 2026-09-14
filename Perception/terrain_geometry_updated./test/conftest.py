"""
conftest.py

Installs minimal stand-in modules for the handful of ROS 2 message
packages this package's PURE-COMPUTATION modules (occupancy_grid.py,
costmap_inflation.py) touch just to construct/read plain message
objects (Header, Point, Pose, Quaternion, OccupancyGrid, MapMetaData).

WHY THIS EXISTS
    This test environment has no ROS 2 distribution installed (no
    rclpy, no message packages) -- only pip-installable numpy/scipy/
    scikit-learn. The modules under test never call into rclpy or any
    real ROS communication; they only read/write plain attributes on
    message objects. A tiny attribute-bag stub that mimics that
    surface is enough to exercise 100% of the real logic (grid math,
    coordinate conversion, inflation) without needing colcon/ROS.

    If a real ROS 2 Python environment IS present (e.g. running these
    tests via `colcon test` / `ament_python`), the real packages are
    used instead -- this conftest only installs a stub for a dotted
    module path that isn't already importable, so it never shadows a
    genuine ROS installation.

    `rclpy`/`sensor_msgs`/`sensor_msgs_py`/`visualization_msgs`/
    `terrain_geometry_msgs`/`tf2_ros`/`rcl_interfaces` are NOT stubbed
    here -- they're needed by terrain_node.py, tf_transform.py,
    clustering.py, visualization.py, which therefore are NOT covered
    by this pure-Python test run. See the project report for what a
    real `colcon build && colcon test` on ROS 2 Jazzy would additionally
    exercise.
"""

import sys
import types


def _install_stub(module_path: str, **classes) -> None:
    try:
        __import__(module_path)
        return  # Real package already available -- don't shadow it.
    except ImportError:
        pass

    module = types.ModuleType(module_path)
    for name, cls in classes.items():
        setattr(module, name, cls)
    sys.modules[module_path] = module

    # Also register the parent package (e.g. "std_msgs") so
    # `from std_msgs.msg import Header` resolves correctly even though
    # only "std_msgs.msg" was inserted into sys.modules.
    parent_name = module_path.rsplit(".", 1)[0]
    if parent_name not in sys.modules:
        parent = types.ModuleType(parent_name)
        sys.modules[parent_name] = parent


class _AttrBag:
    """A plain object that accepts arbitrary keyword args at
    construction and free attribute assignment afterward -- enough to
    stand in for an auto-generated ROS 2 message class for the
    attribute-only usage this package's pure-computation modules make
    of it (never real serialization/publishing)."""

    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)


class _Header(_AttrBag):
    def __init__(self, **kwargs):
        self.stamp = kwargs.pop("stamp", _AttrBag(sec=0, nanosec=0))
        self.frame_id = kwargs.pop("frame_id", "")
        super().__init__(**kwargs)


class _Point(_AttrBag):
    def __init__(self, x=0.0, y=0.0, z=0.0, **kwargs):
        super().__init__(x=x, y=y, z=z, **kwargs)


class _Quaternion(_AttrBag):
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0, **kwargs):
        super().__init__(x=x, y=y, z=z, w=w, **kwargs)


class _Vector3(_AttrBag):
    def __init__(self, x=0.0, y=0.0, z=0.0, **kwargs):
        super().__init__(x=x, y=y, z=z, **kwargs)


class _Pose(_AttrBag):
    def __init__(self, **kwargs):
        self.position = kwargs.pop("position", _Point())
        self.orientation = kwargs.pop("orientation", _Quaternion())
        super().__init__(**kwargs)


class _MapMetaData(_AttrBag):
    pass


class _OccupancyGrid(_AttrBag):
    pass


_install_stub("std_msgs.msg", Header=_Header, String=_AttrBag)
_install_stub("geometry_msgs.msg", Point=_Point, Pose=_Pose, Quaternion=_Quaternion, Vector3=_Vector3)
_install_stub("nav_msgs.msg", OccupancyGrid=_OccupancyGrid, MapMetaData=_MapMetaData)
