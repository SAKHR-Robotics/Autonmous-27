# 📋 Detailed GitHub Projects Task Breakdown (`PerTesterDoc`)

Below is the complete, developer-friendly task decomposition ready for GitHub Projects / Issues.

> For full architectural blueprints, mathematical scoring formulas, system requirements, and scenario tables, refer to [README.md](file:///e:/meseket/Autonmous-27/testing/Perception/README.md).

---

### 📌 Task 1: Package Scaffolding, Process Lifecycle & Configuration
* **Files to Create**:
  * `testing/Perception/package.xml`
  * `testing/Perception/setup.py`
  * `testing/Perception/setup.cfg`
  * `testing/Perception/perception_benchmarking/__init__.py`
  * `testing/Perception/config/benchmark_config.yaml`
  * `testing/Perception/config/scenarios.yaml`

* **Detailed Implementation Recipe**:
  1. **Dependencies**:
     Declare dependencies in `package.xml`: `rclpy`, `sensor_msgs`, `sensor_msgs_py`, `geometry_msgs`, `nav_msgs`, `vision_msgs`, `visualization_msgs`, `tf2_ros`, `tf2_geometry_msgs`, `cv_bridge`, `terrain_geometry_msgs`, `marker_detection_msgs`.
  2. **Process Cleanup Utility (`clean_old_processes`)**:
     Implement a robust cleanup routine in `perception_benchmarking/utils.py` matching PathPlanner's `clean_old_processes()`:
     ```python
     def clean_old_processes():
         print("[INFO] Cleaning up stale ROS 2 perception processes...")
         targets = ["terrain_geometry_node", "marker_detection_node", "mock_sensor_node", "benchmarking_node"]
         for target in targets:
             try:
                 subprocess.run(["pkill", "-f", "-9", target], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
             except Exception:
                 pass
         time.sleep(1.0)
     ```
  3. **Configuration YAMLs**:
     * `config/benchmark_config.yaml`: Contains tolerances, weights, and directory paths.
     * `config/scenarios.yaml`: Fully defines all 9 scenarios with obstacle lists, centroids, dimensions, ground types, and ArUco poses.

* **Acceptance Criteria**:
  * `colcon build --packages-select perception_benchmarking` builds without errors.
  * `python3 -c "from perception_benchmarking.utils import clean_old_processes; clean_old_processes()"` runs cleanly.

---

### 📌 Task 2: Autonomous 3D Terrain & PointCloud Generator
* **File to Create**: `testing/Perception/perception_benchmarking/synthetic_generator.py`

* **Detailed Implementation Recipe**:
  1. **Ground Synthesis**:
     * Flat: $z = \mathcal{N}(0, 0.01)\,\text{m}$.
     * Slopes: $z(x, y) = x \tan(\theta_{\text{pitch}}) + y \tan(\theta_{\text{roll}})$.
     * Mars Undulation: Superposition of low-frequency sine/cosine waves.
  2. **Obstacle Synthesis**:
     * Inject triaxial ellipsoids at $(x_0, y_0, z_0)$ with semi-axes $(a, b, c)$:
       $$\text{Points: } x = x_0 + a r \sin\theta \cos\phi, \quad y = y_0 + b r \sin\theta \sin\phi, \quad z = z_0 + c r \cos\theta \quad (\text{for } z \ge z_0)$$
     * Inject craters using 2D Gaussian height depression.
  3. **Sensor Noise & PointCloud2 Conversion**:
     * Add quadratic depth noise: $\sigma(z) = 0.005 + 0.0012 \cdot z^2$.
     * Serialize NumPy array `[x, y, z, intensity, ground_flag]` to `sensor_msgs/msg/PointCloud2` using `sensor_msgs_py.point_cloud2.create_cloud()`.
     * Automatically cache generated files to `synthetic_data/pointclouds/<scenario_id>.npy` to ensure instant load times on subsequent runs.

* **Acceptance Criteria**:
  * If `synthetic_data/` is empty, generator creates point cloud files in $<50\,\text{ms}$.
  * Generated PointCloud2 contains valid optical frame headers and zero NaN/Inf entries.

---

### 📌 Task 3: Autonomous ArUco RGB-D Scene Generator
* **File to Create**: `testing/Perception/perception_benchmarking/synthetic_generator.py` (Vision Module)

* **Detailed Implementation Recipe**:
  1. **Marker Rendering & Geometric Projection**:
     * Generate canonical ArUco image (`cv2.aruco.generateImageMarker(dictionary, marker_id, side_px)`).
     * Define 3D corner coordinates in marker local frame ($z=0$).
     * Given target pose $\mathbf{T}_{\text{cam}}^{\text{marker}} = [\mathbf{R} \mid \mathbf{t}]$, project 3D corners to 2D image coordinates:
       $$\mathbf{u} = K \cdot (\mathbf{R} \cdot \mathbf{X} + \mathbf{t})$$
     * Use `cv2.getPerspectiveTransform` and `cv2.warpPerspective` to render the marker onto the 1280x720 RGB canvas.
  2. **Aligned Depth Image Rendering**:
     * Render a 16-bit single-channel depth image (`np.uint16` in millimeters) corresponding to the planar distance $Z$ of the marker surface.
  3. **Visual Perturbations**:
     * Support configurable shadow/occlusion masks ($10\%\text{--}40\%$ polygon overlays).
     * Add lighting attenuation and Gaussian blur for stress testing.

* **Acceptance Criteria**:
  * Projected ArUco corners match theoretical pinhole camera coordinates within $<0.1\,\text{px}$.
  * OpenCV `cv2.aruco.detectMarkers` detects synthetic markers at distances up to $8.0\,\text{m}$.

---

### 📌 Task 4: Mock Sensor & Transform Broadcaster Node
* **File to Create**: `testing/Perception/perception_benchmarking/mock_sensor_node.py`

* **Detailed Implementation Recipe**:
  1. **ROS 2 Node Implementation (`MockSensorNode`)**:
     * Subscribes to `/benchmark/load_scenario` (`std_msgs/msg/String`).
     * Publishes:
       * `/camera/depth/color/points` (`sensor_msgs/msg/PointCloud2` at 10 Hz)
       * `/camera/color/image_raw` (`sensor_msgs/msg/Image` at 10 Hz via `cv_bridge`)
       * `/camera/camera_info` (`sensor_msgs/msg/CameraInfo` at 10 Hz)
       * `/benchmark/ground_truth_markers` (`visualization_msgs/msg/MarkerArray`)
       * `/benchmark/ground_truth_meta` (`std_msgs/msg/String` - JSON encoded GT objects)
  2. **TF Tree Broadcaster**:
     * Broadcasts `base_link -> camera_link -> camera_depth_optical_frame` and `camera_color_optical_frame` via `tf2_ros.StaticTransformBroadcaster`.
  3. **Synchronized Playback**:
     * Stamp all published sensor messages with `node.get_clock().now().to_msg()` to enable exact downstream latency profiling.

* **Acceptance Criteria**:
  * `ros2 topic hz /camera/depth/color/points` outputs steady $10.0 \pm 0.5\,\text{Hz}$.
  * TF tree lookup from `base_link` to `camera_depth_optical_frame` succeeds without delay.

---

### 📌 Task 5: Mathematical Metrics Evaluator Engine
* **File to Create**: `testing/Perception/perception_benchmarking/metrics_evaluator.py`

* **Detailed Implementation Recipe**:
  1. **Hungarian Bipartite Matching**:
     * Compute pairwise Euclidean distance matrix $C_{ij} = \|\mathbf{c}_{i,\text{det}} - \mathbf{c}_{j,\text{gt}}\|$.
     * Apply `scipy.optimize.linear_sum_assignment(C)`.
     * Pair matches if distance $< 0.5\,\text{m}$; classify unmatched as FP/FN.
  2. **3D Bounding Box IoU**:
     * Calculate 3D intersection volume and union volume for axis-aligned/oriented boxes.
  3. **ArUco 6-DOF Errors**:
     * Translation: $e_{\text{trans}} = \|\mathbf{t}_{\text{est}} - \mathbf{t}_{\text{gt}}\|_2 \times 1000\,\text{mm}$.
     * Rotation: $e_{\text{rot}} = 2 \arccos(\min(1.0, |\mathbf{q}_{\text{est}} \cdot \mathbf{q}_{\text{gt}}|)) \times \frac{180}{\pi}$.
  4. **Ground Segmentation & Costmap Scores**:
     * Point-level Ground Precision, Recall, $F_1$.
     * 2D Costmap grid correlation.
  5. **Composite Score Calculation**:
     * Evaluates $S_{\text{overall}}$ based on the formulas in README Section 7.

* **Acceptance Criteria**:
  * Unit tests in `test/test_metrics_evaluator.py` pass with $100\%$ code coverage on mathematical routines.

---

### 📌 Task 6: Perception Benchmarking Coordinator Node
* **File to Create**: `testing/Perception/perception_benchmarking/benchmarking_node.py`

* **Detailed Implementation Recipe**:
  1. **Scenario Orchestration State Machine**:
     ```
     [STARTUP] --> [CLEAN_PROCESSES] --> [LOAD_SCENARIO_i] --> [FEED_MOCK_DATA]
                                                                     |
     [GENERATE_REPORTS] <-- [EVALUATE_SCENARIO_i] <-- [CAPTURE_OUTPUTS_50_FRAMES]
     ```
  2. **Subscribed Evaluation Topics**:
     * `/terrain/costmap` (`nav_msgs/msg/OccupancyGrid`)
     * `/terrain/obstacle_features` (`terrain_geometry_msgs/msg/ObstacleFeatureArray`)
     * `/perception/obstacles_only` (`vision_msgs/msg/Detection3DArray`)
     * `/perception/aruco_pose` (`geometry_msgs/msg/PoseStamped`)
     * `/terrain/debug/ground_cloud` (`sensor_msgs/msg/PointCloud2`)
  3. **Latency Measurement**:
     * Calculate $\Delta T = t_{\text{received}} - t_{\text{sensor\_header\_stamp}}$ in milliseconds.
  4. **Data Logging**:
     * Save raw frame data and scenario scorecards to `reports/benchmark_raw.json`.

* **Acceptance Criteria**:
  * Node executes all 9 scenarios in batch mode without manual intervention and generates aggregated summary logs.

---

### 📌 Task 7: Live RViz2 Telemetry & Visual Overlay
* **Files to Create**:
  * `testing/Perception/rviz/perception_benchmark.rviz`
  * `testing/Perception/launch/live_test.launch.py`

* **Detailed Implementation Recipe**:
  1. **RViz Marker Conventions**:
     * **Ground Truth Obstacles**: Semi-transparent **Green Wireframe Cubes** (`RGBA = 0.0, 1.0, 0.0, 0.4`).
     * **Perception Detections**: Solid **Yellow/Red Cubes** (`RGBA = 1.0, 0.8, 0.0, 0.8`).
     * **Error Offset Vectors**: Cyan/Magenta `Marker.LINE_LIST` connecting true centroids to detected centroids.
     * **True vs Estimated ArUco Axes**: RGB arrows showing 6-DOF orientation comparisons.
  2. **Pre-Configured Displays in RViz**:
     * Fixed Frame: `base_link`.
     * PointCloud2 Displays: `/camera/depth/color/points` (Z-color), `/terrain/debug/ground_cloud` (Green), `/terrain/debug/clustered_cloud` (Cluster Colors).
     * Map Display: `/terrain/costmap`.
     * MarkerArray Displays: `/benchmark/ground_truth_markers`, `/terrain/obstacle_markers`.
  3. **Live Test Launcher (`live_test.launch.py`)**:
     * Launches `mock_sensor_node`, SUT perception nodes, `benchmarking_node`, and `rviz2`.

* **Acceptance Criteria**:
  * `ros2 launch perception_benchmarking live_test.launch.py scenario:=scattered_boulders` opens RViz2 with all visual layers active and visible.

---

### 📌 Task 8: Automated HTML, PDF & CSV Report Generator
* **Files to Create**:
  * `testing/Perception/perception_benchmarking/report_generator.py`
  * `testing/Perception/launch/benchmark.launch.py`

* **Detailed Implementation Recipe**:
  1. **Matplotlib Visual Figures**:
     * `obstacle_error_scatter.png`: Centroid position error vs distance from camera.
     * `aruco_range_accuracy.png`: Translation & rotation error degradation curves across $1.0\text{--}8.0\,\text{m}$.
     * `latency_boxplot.png`: End-to-end latency boxplots across all 9 scenarios.
     * `bev_detection_overlay.png`: 2D Bird's Eye View comparing true obstacle circles against detected bounding box footprints.
  2. **HTML Dashboard & PDF Export**:
     * Interactive HTML template with dark-mode Glassmorphism styling, pass/fail score badges, and embedded Base64 plots.
     * Export to printable PDF via `matplotlib.backends.backend_pdf` or `weasyprint`.
     * Export raw metrics table to `reports/benchmark_summary.csv`.
  3. **Automated Batch Launcher (`benchmark.launch.py`)**:
     * Supports `gui:=true` (Live RViz sequential batch) and `gui:=false` (Headless CI/CD run).

* **Acceptance Criteria**:
  * `ros2 launch perception_benchmarking benchmark.launch.py` executes all 9 scenarios, saves complete HTML/PDF/CSV reports in `reports/`, and outputs a terminal summary table.

---

## 🚀 Quick Execution Guide

```bash
# 1. Build the benchmarking package
colcon build --packages-select perception_benchmarking
source install/setup.bash

# 2. Run Mode 1: Live Interactive Test on a specific scenario with RViz
ros2 launch perception_benchmarking live_test.launch.py scenario:=scattered_boulders

# 3. Run Mode 2: Live Sequential Batch Benchmark with RViz
ros2 launch perception_benchmarking benchmark.launch.py gui:=true

# 4. Run Mode 3: Headless Automated CI/CD Benchmark & PDF Report Generation
ros2 launch perception_benchmarking benchmark.launch.py gui:=false
```
