import os
from glob import glob
from setuptools import setup, find_packages

package_name = "terrain_geometry"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=[
        "setuptools",
        "numpy",
        # scikit-learn provides the KD-tree-accelerated DBSCAN used by
        # clustering.py (Step 5).
        "scikit-learn>=1.2.0",
        # scipy backs the cKDTree radius outlier removal (Step 4) and
        # the distance_transform_edt costmap inflation (Step 8).
        "scipy>=1.9.0",
        # Optional, native Concentric Zone Model ground segmentation
        # (Step 2). If unavailable, ground_removal.py automatically
        # falls back to a pure-NumPy vectorized concentric slope/height
        # threshold, so this is a soft dependency.
        # "pypatchworkpp",
    ],
    zip_safe=True,
    maintainer="maintainer",
    maintainer_email="maintainer@example.com",
    description=(
        "Unified single-node terrain/obstacle perception pipeline for "
        "the Intel RealSense D435: TF transform, ground removal, voxel "
        "downsampling, radius outlier removal, DBSCAN clustering, "
        "feature extraction, occupancy grid rasterization, and "
        "exponential-decay costmap inflation."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "terrain_node = terrain_geometry.terrain_node:main",
        ],
    },
)
