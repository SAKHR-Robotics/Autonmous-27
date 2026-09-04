# 👁️ Perception Testing & Benchmarking Suite (`perception_benchmarking`)

An automated, self-contained testing, validation, and benchmarking framework for the **Perception Subsystem** (`terrain_geometry` and `marker_detection`).

This testing suite operates analogously to the PathPlanner benchmark suite (`testing/PathPlanner`), serving as both an **autonomous data feeder (mock sensor pipeline)** and an **automated evaluator** that assesses perception accuracy numerically and visually in RViz and standalone reports.

---

## 🎯 1. The End Goal: What Does This Tester Achieve?

The ultimate objective of the Perception Tester is to provide a **100% self-contained, reproducible, and automated verification harness** for the entire perception stack before running code on physical rover hardware or heavy Gazebo simulations:

> **"Can our perception pipeline accurately filter ground planes, segment Martian boulders, generate collision-free 2D costmaps, and resolve ArUco 6-DOF poses across extreme distances and noise—without needing physical cameras, Gazebo, or manual bag file playback?"**

### 🌟 Key Deliverables & End-State Value:
1. **Zero-Dependency Testing (Self-Contained Ingestion)**:
   * Test the perception nodes instantly on any machine without needing Gazebo worlds, real RealSense/LiDAR sensors, or recorded ROS bags.
   * If input data is missing, the suite **autonomously generates** mathematically precise 3D Point Clouds (flat terrain, slopes, craters, rock fields) and photorealistic ArUco RGB-D images on the fly.
2. **Automated "Ground Truth vs Estimation" Scoring**:
   * Bridges the gap between raw algorithm output and objective truth.
   * Quantifies exactly how well `terrain_geometry` extracts ground planes, clusters obstacles, and draws bounding boxes (3D IoU, Centroid RMSE in cm, False Positives/Negatives).
   * Quantifies the accuracy of `marker_detection` 6-DOF SolvePnP & Kalman filter tracking (Translation error in mm, Angular error in degrees) across varying distances ($0.5\,\text{m}$ to $10\,\text{m}$) and viewing angles.
3. **Real-Time Costmap & Nav2 Verification**:
   * Evaluates the integrity of the 2D obstacle costmap (`/terrain/costmap`) to guarantee that obstacles are inflated correctly without phantom obstacles or blind spots before feeding into the Path Planner.
4. **Dual-Mode Visual Inspection (RViz & Reporting)**:
   * **Live RViz2 Overlay**: Renders **Green Bounding Boxes** for Ground Truth and **Red/Yellow Bounding Boxes** for algorithm detections, linked by dynamic offset error vector lines.
   * **Offline Executive Reports**: Generates publication-grade **HTML, PDF, and CSV reports** with Matplotlib error curves, latency histograms, and pass/fail scorecards.
5. **Regression & Safety Gatekeeper (CI/CD)**:
   * Serves as an automated quality gate in the development pipeline. Whenever filter parameters, clustering thresholds, or camera calibrations are tuned, this suite runs all scenarios to ensure detection rates did not degrade and processing latency stays within the $<100\,\text{ms}$ real-time budget.

---

## 🏗️ 2. Tester Operational Architecture & Working Mechanism

```
+---------------------------------------------------------------------------------------------------+
|                                     TESTER ORCHESTRATION LAYER                                    |
|                               (benchmarking_node / live_test.py)                                  |
|                                                                                                   |
|  • Cleans stale ROS processes (pkill -f -9 terrain_geometry / marker_detection / mock_sensor)     |
|  • Auto-generates missing synthetic PointClouds & RGB-D scenes into synthetic_data/               |
|  • Iterates through 9 Scenarios defined in config/scenarios.yaml                                  |
|  • Injects Sensor Feeds via MockSensorNode:                                                       |
|      -> /camera/depth/color/points (sensor_msgs/PointCloud2)                                      |
|      -> /camera/color/image_raw    (sensor_msgs/Image)                                            |
|      -> /camera/camera_info        (sensor_msgs/CameraInfo)                                       |
|      -> /tf, /tf_static            (tf2_msgs/TFMessage: base_link -> camera_link -> optical)      |
|  • Broadcasts Ground Truth Visuals:                                                               |
|      -> /benchmark/ground_truth_markers (visualization_msgs/MarkerArray: Green Cubes/Axes)        |
+-----------------------------------+---------------------------------------------------------------+
                                    |
                                    v
+---------------------------------------------------------------------------------------------------+
|                                 PERCEPTION SYSTEM UNDER TEST (SUT)                                |
|                                (terrain_geometry + marker_detection)                              |
|                                                                                                   |
|  1. Ground Segmentation (Patchwork++ / RANSAC):                                                   |
|     --> Extracts ground plane: /terrain/debug/ground_cloud (sensor_msgs/PointCloud2)              |
|                                                                                                   |
|  2. Downsampling & Clustering (Voxel Grid + DBSCAN):                                              |
|     --> Clusters non-ground points into 3D objects: /terrain/debug/clustered_cloud                |
|     --> Computes 3D BBoxes & Features: /terrain/obstacle_features & /perception/obstacles_only    |
|                                                                                                   |
|  3. 2D Costmap Generation (Obstacle Layer Projection + Inflation):                                |
|     --> Emits 2D Grid: /terrain/costmap (nav_msgs/OccupancyGrid, 5cm res)                         |
|                                                                                                   |
|  4. ArUco Marker Detection & Tracking (OpenCV SolvePnP + Kalman Filter):                         |
|     --> Computes 6-DOF Pose: /perception/aruco_pose (geometry_msgs/PoseStamped)                   |
+-----------------------------------+---------------------------------------------------------------+
                                    |
                                    v
+---------------------------------------------------------------------------------------------------+
|                                  EVALUATION & METRICS ENGINE                                      |
|                                     (metrics_evaluator.py)                                        |
|                                                                                                   |
|  • Spatial & Geometric Accuracy:                                                                  |
|    - 3D Bounding Box IoU (Intersection over Union)                                                |
|    - Centroid Position RMSE (Delta x, Delta y, Delta z in meters) via Hungarian Bipartite Match   |
|    - ArUco Translation Error (mm) & Quaternion Geodesic Angular Error (degrees)                   |
|    - Ground Classification Precision, Recall, and F1-score (%)                                    |
|    - 2D Costmap Occupancy Footprint Correlation (%)                                               |
|  • Temporal Performance:                                                                          |
|    - End-to-End Latency per frame (ms) & Pipeline Frame Rate (FPS)                                |
|  • Composite Grading:                                                                             |
|    - Overall Perception Score = weighted sum of IoU, RMSE, Recall, Pose, F1, Latency              |
+-----------------------------------+---------------------------------------------------------------+
                                    |
                                    v
+---------------------------------------------------------------------------------------------------+
|                                  VISUALIZATION & REPORTING LAYER                                  |
|                                      (report_generator.py)                                        |
|                                                                                                   |
|  1. Live RViz2 Visual Telemetry:                                                                  |
|     - Green Ground Truth boxes + Red/Yellow Estimated boxes + Dynamic Offset Error Vectors.       |
|  2. Automated Analytical Reports:                                                                 |
|     - Generates CSV summaries, HTML glassmorphism dashboards, and printable PDF reports.          |
+---------------------------------------------------------------------------------------------------+
```

---

## 🎮 3. The Three Operational Modes

### Mode 1: Live Interactive Single-Scenario Test (`live_test.launch.py`)
* Cleans up old processes and synthesizes missing scene data on demand.
* Launches `mock_sensor_node`, `terrain_geometry_node`, `marker_detection_node`, and `rviz2`.
* Continuously publishes at 10 Hz, letting the developer inspect 3D bounding boxes, cluster separation, costmap inflation, and TF frames interactively.

### Mode 2: Live Sequential Batch RViz Benchmark (`benchmark.launch.py gui:=true`)
* Cycles through all **9 Mars Yard scenarios** sequentially inside RViz2.
* Automatically transitions scenes every $N$ seconds: clears previous markers, loads new terrain and ArUco feeds, draws dynamic error lines, and updates live scorecards.

### Mode 3: Headless Automated CI/CD Benchmark (`benchmark.launch.py gui:=false`)
* Runs headless without GUI overhead for automated CI/CD pipelines.
* Evaluates 50 frames per scenario, scores all 10 mathematical metrics against strict pass/fail tolerance gates, and generates HTML, PDF, and CSV reports.

---

## 📡 4. System Requirements & Topic Interfaces

### 4.1 Input Topics Required by Perception Nodes
| Topic Name | Message Type | Target Node | Purpose |
| :--- | :--- | :--- | :--- |
| `/camera/depth/color/points` | `sensor_msgs/msg/PointCloud2` | `terrain_geometry_node` | 3D Point cloud of terrain & obstacles in `camera_depth_optical_frame` |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | `marker_detection_node` | 8-bit RGB camera feed (e.g. 1280x720 or 640x480) |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | `marker_detection_node` | Camera intrinsic matrix ($K$) and distortion coefficients ($D$) |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Both nodes | Dynamic & static transforms linking sensor frames to `base_link` |

### 4.2 Output Topics to be Monitored & Evaluated
| Topic Name | Message Type | Source Node | Metrics to Evaluate |
| :--- | :--- | :--- | :--- |
| `/terrain/costmap` | `nav_msgs/msg/OccupancyGrid` | `terrain_geometry_node` | 2D obstacle footprint IoU, inflation boundary accuracy |
| `/terrain/obstacle_features` | `terrain_geometry_msgs/msg/ObstacleFeatureArray` | `terrain_geometry_node` | 3D obstacle centroid RMSE, dimension errors ($\Delta w, \Delta l, \Delta h$) |
| `/perception/obstacles_only` | `vision_msgs/msg/Detection3DArray` | `terrain_geometry_node` | 3D Bounding Box IoU, Precision, Recall, False Positives |
| `/perception/aruco_pose` | `geometry_msgs/msg/PoseStamped` | `marker_detection_node` | 6-DOF Translation Error (mm) & Rotation Angle Error ($^\circ$) |
| `/terrain/debug/ground_cloud` | `sensor_msgs/msg/PointCloud2` | `terrain_geometry_node` | Ground segmentation Precision / Recall against true ground labels |

---

## 🤖 5. Autonomous Mock & Synthetic Data Generation

If external datasets or bag files are missing, the benchmarking suite autonomously synthesizes complete 3D and 2D sensor feeds with mathematically exact Ground Truth.

### 5.1 Synthetic 3D Point Cloud Generation (`synthetic_generator.py`)
1. **Ground Surface Generation**:
   * *Flat Plane*: $z = 0 + \mathcal{N}(0, \sigma_{\text{ground}})$.
   * *Inclined Slope*: $z(x, y) = x \tan(\theta_{\text{pitch}}) + y \tan(\theta_{\text{roll}})$.
   * *Rough Mars Terrain (Perlin/Simplex Noise)*: $z(x, y) = \sum_{k=1}^N A_k \sin(f_k x + \phi_k) \cos(f_k y + \psi_k)$.
2. **Positive Obstacles (Boulders & Rocks)**:
   * Upper hemisphere / triaxial ellipsoid:
     $$\frac{(x - x_0)^2}{a^2} + \frac{(y - y_0)^2}{b^2} + \frac{(z - z_0)^2}{c^2} \le 1, \quad z \ge z_0$$
3. **Negative Obstacles (Craters / Trenches)**:
   * Parabolic / Gaussian depression:
     $$z(x, y) = z_{\text{ground}} - d_{\text{crater}} \exp\left(-\frac{(x - x_c)^2 + (y - y_c)^2}{2 r_c^2}\right)$$
4. **Sensor Realism & Noise Model**:
   * Radial depth jitter: $\sigma_d(r) = \sigma_0 + k \cdot r^2$ (simulating RealSense depth degradation over range).
   * Dropout rate: random sparsity of $1\% - 5\%$ simulating infrared surface absorption or clipping.

### 5.2 Synthetic ArUco RGB-D Scene Generation
1. **Marker Rendering & Perspective Warping**:
   * Generate canonical ArUco binary image (e.g. `DICT_4X4_50`, ID: 0 to 49) using OpenCV ArUco.
   * Given ground truth pose $\mathbf{T}_{\text{cam}}^{\text{marker}} = [\mathbf{R} \mid \mathbf{t}]$ and camera matrix $K$:
     $$\mathbf{p}_{\text{img}} \sim K \cdot (\mathbf{R} \cdot \mathbf{p}_{\text{marker\_3D}} + \mathbf{t})$$
   * Warp marker quad into the background RGB canvas with perspective transform (`cv2.warpPerspective`).
2. **Depth Map Alignment**:
   * Render corresponding 16-bit depth image (`16UC1` in mm) matching the marker plane depth $z$.
3. **Simulated Disturbances**:
   * Brightness scaling, Gaussian blur, Poisson noise, partial occlusions (e.g. $10\%$ to $40\%$ corner masking).

---

## 🗺️ 6. The 9 Mars Yard Benchmark Scenarios

| # | Scenario ID | Description | Difficulty | Key Perception Stress Test |
|---|:---|:---|:---:|:---|
| 1 | `flat_open_terrain` | Smooth Mars sand with zero obstacles | Baseline | False positive rate & ground plane stability |
| 2 | `scattered_boulders` | 4 rocks of radii $0.15\text{--}0.45\,\text{m}$ at $2.0\text{--}6.0\,\text{m}$ | Medium | DBSCAN clustering separation & centroid RMSE |
| 3 | `canyon_gate_walls` | Two large rock walls forming a narrow $1.2\,\text{m}$ passage | Hard | Bounding box sizing & costmap passage opening |
| 4 | `rough_terrain_slopes` | $15^\circ$ inclined surface with embedded boulders | Medium | Patchwork++ normal estimation on tilted ground |
| 5 | `crater_and_depression` | $0.4\,\text{m}$ deep, $1.2\,\text{m}$ radius negative crater | Hard | Negative obstacle / ground dropoff detection |
| 6 | `dense_rock_labyrinth` | 8 tightly packed boulders with mutual occlusions | Extreme | Cluster under-segmentation & over-segmentation |
| 7 | `aruco_distance_sweep` | ArUco Marker ID 0 tested at $1.0\,\text{m}, 3.0\,\text{m}, 6.0\,\text{m}, 8.0\,\text{m}$ | Hard | SolvePnP degradation & Kalman filter convergence |
| 8 | `aruco_steep_yaw` | ArUco Marker at $3.0\,\text{m}$ with $\pm 45^\circ$ and $\pm 60^\circ$ yaw | Hard | Perspective foreshortening & corner jitter |
| 9 | `dynamic_shadow_occlusion` | ArUco Marker with $30\%$ corner masked by rover shadow | Extreme | Detection recall under partial marker occlusion |

---

## 📊 7. Mathematical Evaluation Metrics & Scoring Engine

### 7.1 3D Bounding Box Intersection-over-Union (3D IoU)
Given Ground Truth box $B_{gt}$ and Detected box $B_{det}$:
$$\text{IoU}_{3D} = \frac{\text{Vol}(B_{gt} \cap B_{det})}{\text{Vol}(B_{gt} \cup B_{det})} = \frac{\text{Vol}(B_{gt} \cap B_{det})}{\text{Vol}(B_{gt}) + \text{Vol}(B_{det}) - \text{Vol}(B_{gt} \cap B_{det})}$$
* Bipartite matching via Hungarian Algorithm (`scipy.optimize.linear_sum_assignment`) pairs detected boxes with ground truth instances based on centroid distance.

### 7.2 Centroid Position RMSE
For $M$ matched obstacles:
$$\text{RMSE}_{\text{pos}} = \sqrt{\frac{1}{M} \sum_{i=1}^M \|\mathbf{c}_{i, det} - \mathbf{c}_{i, gt}\|^2}$$
$$\Delta x = |x_{det} - x_{gt}|, \quad \Delta y = |y_{det} - y_{gt}|, \quad \Delta z = |z_{det} - z_{gt}|$$

### 7.3 ArUco 6-DOF Pose Estimation Error
* **Translation Error**:
  $$e_{\text{trans}} = \|\mathbf{t}_{\text{est}} - \mathbf{t}_{\text{gt}}\|_2 = \sqrt{(x_e - x_g)^2 + (y_e - y_g)^2 + (z_e - z_g)^2}$$
* **Rotation Geodesic Error**:
  $$e_{\text{rot}} = 2 \arccos(\min(1.0, |\mathbf{q}_{\text{est}} \cdot \mathbf{q}_{\text{gt}}|)) \times \frac{180}{\pi}$$

### 7.4 Ground Segmentation Accuracy
Using point-level ground truth binary labels ($y_i \in \{0: \text{obstacle}, 1: \text{ground}\}$):
$$\text{Precision} = \frac{TP}{TP + FP}, \quad \text{Recall} = \frac{TP}{TP + FN}, \quad F_1 = \frac{2 \cdot \text{Precision} \cdot \text{Recall}}{\text{Precision} + \text{Recall}}$$

### 7.5 Pipeline Latency & Throughput
* **Frame Latency**: $\Delta T = t_{\text{published\_output}} - t_{\text{input\_sensor\_stamp}}$ (measured in ms).
* **Processing Rate**: $\text{FPS} = \frac{N_{\text{frames}}}{\sum \Delta T}$.

### 7.6 Composite Perception Benchmark Score ($S_{\text{overall}} \in [0, 100]$)

$$S_{\text{overall}} = 0.25 S_{\text{IoU}} + 0.20 S_{\text{RMSE}} + 0.20 S_{\text{Recall}} + 0.15 S_{\text{ArUco}} + 0.10 S_{\text{Ground}} + 0.10 S_{\text{Latency}}$$

---

## 📁 8. Directory Structure

```
testing/Perception/
├── README.md                           # Architecture, Math, Metrics & End Goal documentation
├── PerTesterDoc.md                     # GitHub Project Tasks & Implementation Roadmap
├── package.xml                         # ROS 2 package manifest
├── setup.py                            # Python package setup
├── setup.cfg                           # Setup configuration
├── config/
│   ├── benchmark_config.yaml           # Global pass/fail thresholds and test parameters
│   └── scenarios.yaml                  # Predefined testing scenes and ground truth metadata
├── launch/
│   ├── benchmark.launch.py             # Batch automated benchmarking launcher
│   └── live_test.launch.py             # Interactive visual test launcher with RViz
├── perception_benchmarking/
│   ├── __init__.py
│   ├── utils.py                        # Process cleanup & shared math utilities
│   ├── synthetic_generator.py          # Math-based point cloud and ArUco scene generator
│   ├── mock_sensor_node.py             # ROS 2 publisher for point clouds, images, TF, and camera info
│   ├── benchmarking_node.py            # Central orchestrator and topic synchronizer
│   ├── metrics_evaluator.py            # Quantitative mathematical evaluation engine
│   └── report_generator.py             # Matplotlib, HTML, and PDF report builder
├── rviz/
│   └── perception_benchmark.rviz       # RViz configuration with ground-truth vs output displays
├── synthetic_data/                     # Storage for generated/cached test scenes
│   ├── pointclouds/
│   └── images/
└── reports/                            # Generated benchmark outputs (CSV, HTML, PDF)
```

---

## 🚀 9. Quickstart & Usage

### 1. Build the Benchmarking Package
```bash
colcon build --packages-select perception_benchmarking
source install/setup.bash
```

### 2. Mode 1: Run Interactive Live RViz Test (Single Scenario)
```bash
ros2 launch perception_benchmarking live_test.launch.py scenario:=scattered_boulders
```

### 3. Mode 2: Run Sequential Batch RViz Benchmark (Live GUI)
```bash
ros2 launch perception_benchmarking benchmark.launch.py gui:=true
```

### 4. Mode 3: Run Automated Headless CI/CD Benchmark & Generate Reports
```bash
ros2 launch perception_benchmarking benchmark.launch.py gui:=false
```

### 5. Manually Re-generate Summary Reports
```bash
ros2 run perception_benchmarking report_generator --input-csv reports/benchmark_summary.csv
```
