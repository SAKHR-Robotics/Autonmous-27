import os
import sys
import time
from playwright.sync_api import sync_playwright

HTML_TEMPLATE = """<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <script src="https://cdn.jsdelivr.net/npm/mermaid@10/dist/mermaid.min.js"></script>
    <style>
        body {{
            margin: 0;
            padding: 32px;
            background-color: #0b0f19;
            color: #f3f4f6;
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
            display: inline-block;
            min-width: 950px;
        }}
        .header {{
            margin-bottom: 24px;
            border-bottom: 1px solid #1e293b;
            padding-bottom: 16px;
        }}
        .title {{
            font-size: 24px;
            font-weight: 700;
            color: #f8fafc;
            display: flex;
            align-items: center;
            gap: 12px;
        }}
        .badge {{
            font-size: 11px;
            font-weight: 700;
            text-transform: uppercase;
            letter-spacing: 0.05em;
            padding: 4px 12px;
            border-radius: 9999px;
            background-color: #1e1b4b;
            color: #a5b4fc;
            border: 1px solid #6366f1;
        }}
        .subtitle {{
            font-size: 13px;
            color: #94a3b8;
            margin-top: 6px;
        }}
        .mermaid {{
            background-color: #0d1322;
            border: 1px solid #1e293b;
            border-radius: 14px;
            padding: 28px;
            box-shadow: 0 12px 30px -5px rgba(0, 0, 0, 0.6);
        }}
        .edgeLabel {{
            background-color: #0b0f19 !important;
            color: #38bdf8 !important;
            padding: 3px 6px !important;
            border-radius: 4px !important;
            border: 1px solid #1e293b !important;
            font-size: 11px !important;
        }}
    </style>
</head>
<body>
    <div class="header">
        <div class="title">
            <span>{title}</span>
            <span class="badge">{badge}</span>
        </div>
        <div class="subtitle">{subtitle}</div>
    </div>
    <div class="mermaid">
{mermaid_code}
    </div>
    <script>
        mermaid.initialize({{
            startOnLoad: true,
            theme: 'dark',
            themeVariables: {{
                darkMode: true,
                background: '#0d1322',
                primaryColor: '#1e293b',
                primaryTextColor: '#f8fafc',
                primaryBorderColor: '#475569',
                lineColor: '#6366f1',
                secondaryColor: '#1e1b4b',
                tertiaryColor: '#0b0f19',
                edgeLabelBackground: '#0b0f19',
                fontFamily: '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif',
                fontSize: '12px'
            }},
            flowchart: {{
                curve: 'basis',
                htmlLabels: true
            }}
        }});
    </script>
</body>
</html>
"""

ALL_DIAGRAMS = [
    # =========================================================================
    # 1. PERCEPTION SUBSYSTEM (General_Docs/1-Perception/)
    # =========================================================================
    {
        "folder": "1-Perception",
        "filename": "00_perception_master_pipeline.png",
        "title": "Perception Subsystem Master Pipeline",
        "badge": "Perception Master",
        "subtitle": "Dual-stream architecture: 3D terrain geometry point cloud processing and ArUco 6-DoF visual landmark estimation",
        "mermaid": """flowchart TD
    classDef sensor fill:#0f172a,stroke:#64748b,stroke-width:2px,color:#f8fafc;
    classDef pcl fill:#064e3b,stroke:#059669,stroke-width:2px,color:#f8fafc;
    classDef aruco fill:#1e1b4b,stroke:#6366f1,stroke-width:2px,color:#f8fafc;
    classDef out fill:#065f46,stroke:#34d399,stroke-width:2px,color:#f8fafc;

    subgraph SENSORS ["📡 SENSOR INPUTS (RealSense D435i)"]
        S_CLOUD["<b>/camera/depth/color/points</b><br/><i>(sensor_msgs/PointCloud2)</i>"]:::sensor
        S_RGB["<b>/camera/image_raw</b><br/><i>(sensor_msgs/Image)</i>"]:::sensor
        S_INFO["<b>/camera/camera_info</b><br/><i>(sensor_msgs/CameraInfo)</i>"]:::sensor
        S_TF["<b>TF System</b><br/><i>(camera_link ➔ base_link)</i>"]:::sensor
    end

    subgraph TERRAIN ["🪨 3D TERRAIN GEOMETRY PIPELINE (terrain_geometry)"]
        T_ROI["<b>1. Forward ROI Spatial Crop</b><br/>Filters points outside rover envelope"]:::pcl
        T_GND["<b>2. Patchwork++ Ground Removal</b><br/>Separates terrain ground from boulders"]:::pcl
        T_VOX["<b>3. Voxel Grid Downsampling</b><br/>5cm leaf size balances fidelity and speed"]:::pcl
        T_ROR["<b>4. Radius Outlier Removal</b><br/>Eliminates floating dust and sunlight noise"]:::pcl
        T_CLUST["<b>5. DBSCAN Euclidean Clustering</b><br/>Extracts discrete rock centroids & 3D BBoxes"]:::pcl
        T_MEM["<b>6. Persistent Memory & EMA Tracker</b><br/>Tracks obstacles & retains blind spots"]:::pcl
    end

    subgraph ARUCO ["🎯 ARUCO LANDMARK POSE ESTIMATION (marker_detection)"]
        A_DETECT["<b>1. Marker Detector & Corner Refinement</b><br/>Sub-pixel corner detection from dictionary"]:::aruco
        A_PNP["<b>2. OpenCV solvePnP 6-DoF Solver</b><br/>Projects 2D corners into 3D camera frame"]:::aruco
        A_KALMAN["<b>3. Kalman Filter & Covariance Gate</b><br/>Smooths jitter and rejects false landmark detections"]:::aruco
    end

    subgraph OUTPUTS ["📤 PERCEPTION SYSTEM OUTPUTS"]
        OUT_OBSTACLES["<b>To Nav2 Costmap:</b><br/>• /perception/obstacles_only (3D Bounding Boxes)<br/>• /terrain/costmap (2D Occupancy Grid)"]:::out
        OUT_ARUCO["<b>To RTAB-Map SLAM:</b><br/>• /perception/aruco_pose (6-DoF Constraint)"]:::out
        OUT_RVIZ["<b>To RViz Dashboard:</b><br/>• /terrain/obstacle_markers (3D Markers & Telemetry)"]:::out
    end

    S_CLOUD --> T_ROI
    S_TF --> T_ROI
    T_ROI --> T_GND --> T_VOX --> T_ROR --> T_CLUST --> T_MEM

    S_RGB --> A_DETECT
    S_INFO --> A_PNP
    A_DETECT --> A_PNP --> A_KALMAN

    T_MEM -->|"<b>/perception/obstacles_only</b>"| OUT_OBSTACLES
    T_MEM -->|"<b>/terrain/obstacle_markers</b>"| OUT_RVIZ
    A_KALMAN -->|"<b>/perception/aruco_pose</b>"| OUT_ARUCO
"""
    },
    {
        "folder": "1-Perception",
        "filename": "01_terrain_geometry_6stage_pipeline.png",
        "title": "3D Point Cloud Terrain Geometry 6-Stage Pipeline",
        "badge": "Perception Block 1",
        "subtitle": "Complete pipeline: raw point cloud filtering, ground segmentation, downsampling, clustering, and memory",
        "mermaid": """flowchart TD
    classDef stage fill:#064e3b,stroke:#059669,stroke-width:2px,color:#f8fafc;
    classDef io fill:#0f172a,stroke:#38bdf8,stroke-width:2px,color:#f8fafc;

    IN_RAW["<b>Raw Depth Point Cloud</b><br/>/camera/depth/color/points (~300,000 points @ 30Hz)"]:::io

    STAGE1["<b>Stage 1: ROI Spatial Crop</b><br/>X [-2.5m, +2.5m], Y [-2.5m, +2.5m], Z [-1.0m, +1.0m]"]:::stage
    STAGE2["<b>Stage 2: Patchwork++ Ground Removal</b><br/>Concentric zone model fits ground surface and separates boulders"]:::stage
    STAGE3["<b>Stage 3: Voxel Grid Downsampling</b><br/>5cm voxel leaf size generates uniform density cloud (~8,000 points)"]:::stage
    STAGE4["<b>Stage 4: Radius Outlier Removal (ROR)</b><br/>Min neighbors = 4 within 0.15m radius to strip dust/sunlight noise"]:::stage
    STAGE5["<b>Stage 5: DBSCAN Euclidean Clustering</b><br/>Min points = 15, Epsilon = 0.12m &rarr; Clusters segmented into discrete obstacles"]:::stage
    STAGE6["<b>Stage 6: Persistent Memory & EMA Tracking</b><br/>Tracks obstacle IDs across frames and retains blind-spot memory for Nav2"]:::stage

    OUT_OBST["<b>Published Obstacles & 3D BBoxes</b><br/>/perception/obstacles_only (vision_msgs/Detection3DArray)"]:::io

    IN_RAW --> STAGE1 --> STAGE2 --> STAGE3 --> STAGE4 --> STAGE5 --> STAGE6 --> OUT_OBST
"""
    },
    {
        "folder": "1-Perception",
        "filename": "02_roi_and_patchwork_ground_removal.png",
        "title": "Deep Dive: ROI Spatial Crop & Patchwork++ Ground Separation",
        "badge": "Perception Block 2",
        "subtitle": "Camera-to-body coordinate transform crop and concentric zone ground plane fitting",
        "mermaid": """flowchart LR
    classDef input fill:#0f172a,stroke:#64748b,stroke-width:2px,color:#f8fafc;
    classDef proc fill:#064e3b,stroke:#059669,stroke-width:2px,color:#f8fafc;
    classDef out fill:#065f46,stroke:#34d399,stroke-width:2px,color:#f8fafc;

    subgraph Inputs ["Sensor Inputs"]
        CLOUD["<b>/camera/depth/color/points</b><br/>sensor_msgs/PointCloud2"]:::input
        TF["<b>TF: base_link ➔ camera_link</b><br/>Physical extrinsic orientation"]:::input
    end

    subgraph ROI ["Stage 1: Spatial ROI Box"]
        CROP["<b>Spatial PassThrough Filter</b><br/>• Drops points behind rover<br/>• Limits range to 4.5m forward<br/>• Filters sky and chassis self-hits"]:::proc
    end

    subgraph Patchwork ["Stage 2: Patchwork++ Algorithm"]
        CZM["<b>Concentric Zone Model (CZM)</b><br/>Divides ground into 4 radial rings & sectors"]:::proc
        RANSAC["<b>Sector Ground Plane Fit</b><br/>Estimates local surface normal vector per bin"]:::proc
        SEPARATE{"Distance to ground &lt; threshold?"}:::proc
    end

    subgraph Outputs ["Separated Clouds"]
        NON_GROUND["<b>Obstacle Candidate Cloud</b><br/>(Sent to Voxel Downsampling)"]:::out
        GROUND_DEBUG["<b>/terrain/debug/ground_cloud</b><br/>(Visual ground verification)"]:::out
    end

    CLOUD --> CROP
    TF --> CROP
    CROP --> CZM --> RANSAC --> SEPARATE
    SEPARATE -->|"NO (Protruding Rock)"| NON_GROUND
    SEPARATE -->|"YES (Ground Plane)"| GROUND_DEBUG
"""
    },
    {
        "folder": "1-Perception",
        "filename": "03_voxel_downsampling_and_outlier_removal.png",
        "title": "Deep Dive: Voxel Grid Downsampling & Outlier Filtering",
        "badge": "Perception Block 3",
        "subtitle": "Compute reduction via uniform 5cm voxelization and airborne sensor noise suppression",
        "mermaid": """flowchart LR
    classDef inout fill:#0f172a,stroke:#38bdf8,stroke-width:2px,color:#f8fafc;
    classDef proc fill:#064e3b,stroke:#059669,stroke-width:2px,color:#f8fafc;

    IN_CLOUD["<b>Non-Ground Candidate Points</b><br/>High density (~80k points)"]:::inout

    subgraph VoxelStage ["Stage 3: Voxel Grid Filter"]
        LEAF["<b>3D Voxel Leaf = 0.05m</b><br/>Groups points into 5cm cubic centroids"]:::proc
        NORM["<b>Centroid Normalization</b><br/>Downsamples cloud to ~8k uniform points"]:::proc
    end

    subgraph RORStage ["Stage 4: Radius Outlier Removal"]
        SEARCH["<b>kd-Tree Spatial Neighbor Search</b><br/>Search radius = 0.15m"]:::proc
        GATE{"Neighbor count &gt;= 4?"}:::proc
        DROP["<b>Discard Noise Point</b><br/>Dust, Sunlight IR artifacts"]:::proc
        KEEP["<b>Retain Valid Structure Point</b><br/>Solid rock surface point"]:::proc
    end

    OUT_CLOUD["<b>Cleaned Obstacle Point Cloud</b><br/>Ready for DBSCAN Clustering"]:::inout

    IN_CLOUD --> LEAF --> NORM --> SEARCH --> GATE
    GATE -->|"NO"| DROP
    GATE -->|"YES"| KEEP
    KEEP --> OUT_CLOUD
"""
    },
    {
        "folder": "1-Perception",
        "filename": "04_dbscan_clustering_and_bounding_boxes.png",
        "title": "Deep Dive: DBSCAN Euclidean Clustering & 3D Bounding Boxes",
        "badge": "Perception Block 4",
        "subtitle": "Unsupervised density-based obstacle clustering, centroid math, and oriented bounding boxes",
        "mermaid": """flowchart TD
    classDef proc fill:#064e3b,stroke:#059669,stroke-width:2px,color:#f8fafc;
    classDef decision fill:#854d0e,stroke:#facc15,stroke-width:2px,color:#f8fafc;
    classDef out fill:#065f46,stroke:#34d399,stroke-width:2px,color:#f8fafc;

    IN_FILTERED["<b>Cleaned Point Cloud</b> (Uniform density)"]

    subgraph Clustering ["DBSCAN Density Clustering"]
        DBSCAN_ALG["<b>kd-Tree Euclidean Cluster Extraction</b><br/>• Epsilon (&epsilon;) = 0.12m<br/>• Minimum Points (MinPts) = 15"]:::proc
        CLUSTER_EVAL{"Cluster size within [15, 3000] pts?"}:::decision
        DISCARD["Reject cluster as stray noise"]:::decision
    end

    subgraph Extraction ["3D Geometry Extraction per Cluster"]
        CENTROID["<b>Centroid Calculation:</b><br/>C_xyz = &Sigma;(P_i) / N"]:::proc
        BBOX["<b>3D Bounding Box Dimensions:</b><br/>Width (dx), Length (dy), Height (dz)"]:::proc
        MSG["<b>Build Detection3D Message</b><br/>geometry_msgs/Pose + Vector3 bbox size"]:::proc
    end

    OUT_BBOX["<b>/perception/local_bboxes</b><br/>vision_msgs/Detection3DArray"]:::out

    IN_FILTERED --> DBSCAN_ALG --> CLUSTER_EVAL
    CLUSTER_EVAL -->|"NO"| DISCARD
    CLUSTER_EVAL -->|"YES"| CENTROID --> BBOX --> MSG --> OUT_BBOX
"""
    },
    {
        "folder": "1-Perception",
        "filename": "05_persistent_memory_and_ema_tracker.png",
        "title": "Deep Dive: Persistent Memory Node & EMA Rock Tracking",
        "badge": "Perception Block 5",
        "subtitle": "Inter-frame obstacle tracking, IoU matching, Exponential Moving Average smoothing, and blind-spot retention",
        "mermaid": """flowchart LR
    classDef proc fill:#064e3b,stroke:#059669,stroke-width:2px,color:#f8fafc;
    classDef state fill:#1e1b4b,stroke:#818cf8,stroke-width:2px,color:#f8fafc;
    classDef out fill:#065f46,stroke:#34d399,stroke-width:2px,color:#f8fafc;

    NEW_BBOXES["<b>New Detections:</b><br/>/perception/local_bboxes"]
    MEMORY_DB[("<b>Persistent Memory DB:</b><br/>Active Tracked Obstacles")]:::state

    subgraph Matcher ["Data Association"]
        DIST["<b>Euclidean Distance / IoU Gate</b><br/>Matches new bbox with existing tracks"]:::proc
        BRANCH{"Track Match Found?"}:::proc
    end

    subgraph Update ["State Filtering"]
        EMA["<b>EMA Position Update (&alpha;=0.7):</b><br/>P_new = &alpha; &middot; P_meas + (1 - &alpha;) &middot; P_prev"]:::proc
        NEW_ID["<b>Assign New Obstacle ID</b><br/>Initialize track lifetime = 50 frames"]:::proc
        DECAY["<b>Blind-Spot Retention Decay</b><br/>Obstacles outside FOV decay gracefully"]:::proc
    end

    OUT_OBST["<b>/perception/obstacles_only</b><br/>Smoothed persistent obstacles for Nav2"]:::out

    NEW_BBOXES --> DIST
    MEMORY_DB --> DIST
    DIST --> BRANCH
    BRANCH -->|"YES"| EMA --> MEMORY_DB
    BRANCH -->|"NO"| NEW_ID --> MEMORY_DB
    MEMORY_DB --> DECAY --> OUT_OBST
"""
    },
    {
        "folder": "1-Perception",
        "filename": "06_aruco_detection_and_pnp_block.png",
        "title": "Deep Dive: ArUco Vision & solvePnP 6-DoF Landmark Tracking",
        "badge": "Perception Block 6",
        "subtitle": "Sub-pixel corner detection, perspective-n-point 3D pose extraction, and covariance filtering for SLAM drift reset",
        "mermaid": """flowchart LR
    classDef input fill:#0f172a,stroke:#64748b,stroke-width:2px,color:#f8fafc;
    classDef proc fill:#1e1b4b,stroke:#818cf8,stroke-width:2px,color:#f8fafc;
    classDef out fill:#064e3b,stroke:#34d399,stroke-width:2px,color:#f8fafc;

    subgraph Inputs ["Camera Feed"]
        RGB["<b>RGB Image Frame</b><br/>/camera/image_raw (1280x720)"]:::input
        INFO["<b>Camera Intrinsics Matrix</b><br/>fx, fy, cx, cy & distortion coeffs"]:::input
    end

    subgraph Detection ["ArUco Detection (marker_detection)"]
        DICT["<b>1. Dictionary Lookup</b><br/>DICT_4X4_50 / DICT_5X5_100"]:::proc
        SUBPIX["<b>2. Corner Refinement</b><br/>cv2.cornerSubPix interpolation"]:::proc
    end

    subgraph Solver ["Pose Estimation"]
        PNP["<b>3. OpenCV solvePnP</b><br/>Calculates rotation (rvec) & translation (tvec)"]:::proc
        COV["<b>4. Reprojection Error & Covariance Gate</b><br/>Rejects error &gt; 2.0px & assigns pose covariance"]:::proc
    end

    subgraph Output ["SLAM Anchor"]
        ARUCO_OUT["<b>/perception/aruco_pose</b><br/>geometry_msgs/PoseStamped<br/>Global SLAM Loop Closure"]:::out
    end

    RGB --> DICT --> SUBPIX --> PNP
    INFO --> PNP
    PNP --> COV --> ARUCO_OUT
"""
    },
    {
        "folder": "1-Perception",
        "filename": "07_terrain_costmap_generation.png",
        "title": "Deep Dive: Direct 2D Terrain Costmap Generation",
        "badge": "Perception Block 7",
        "subtitle": "Projecting 3D persistent obstacle bounding boxes into a 5cm 2D occupancy grid with safety inflation",
        "mermaid": """flowchart LR
    classDef inout fill:#0f172a,stroke:#38bdf8,stroke-width:2px,color:#f8fafc;
    classDef proc fill:#064e3b,stroke:#059669,stroke-width:2px,color:#f8fafc;

    IN_BBOX["<b>/perception/obstacles_only</b><br/>3D Bounding Boxes"]:::inout

    subgraph GridProject ["Grid Projection & Inflation"]
        RASTER["<b>2D Grid Rasterization</b><br/>Resolution = 0.05m (5cm per cell)<br/>Origin aligned to base_link"]:::proc
        FOOTPRINT["<b>Rover Footprint Envelope</b><br/>Marks cells within rock perimeter as LETHAL (100)"]:::proc
        INFLATE["<b>Exponential Safety Inflation</b><br/>Cost = 100 &middot; exp(-decay &middot; dist)"]:::proc
    end

    OUT_GRID["<b>/terrain/costmap</b><br/>nav_msgs/OccupancyGrid @ 10Hz"]:::inout

    IN_BBOX --> RASTER --> FOOTPRINT --> INFLATE --> OUT_GRID
"""
    },

    # =========================================================================
    # 2. SLAM SUBSYSTEM (General_Docs/2-SLAM/)
    # =========================================================================
    {
        "folder": "2-SLAM",
        "filename": "00_slam_master_pipeline.png",
        "title": "SLAM & State Estimation Master Architecture Pipeline",
        "badge": "SLAM Master",
        "subtitle": "Complete 6-block architecture: wheel kinematics, sand slip rejection, 100Hz EKF fusion, and RTAB-Map visual graph SLAM",
        "mermaid": """flowchart TD
    classDef hw fill:#0f172a,stroke:#64748b,stroke-width:2px,color:#f8fafc;
    classDef block fill:#1e1b4b,stroke:#818cf8,stroke-width:2px,color:#f8fafc;
    classDef ekf fill:#581c87,stroke:#c084fc,stroke-width:2px,color:#f8fafc;
    classDef nav fill:#7f1d1d,stroke:#f87171,stroke-width:2px,color:#f8fafc;
    classDef out fill:#064e3b,stroke:#34d399,stroke-width:2px,color:#f8fafc;

    subgraph Inputs ["📥 SENSOR & HARDWARE INPUTS"]
        IN_TICKS["Raw Wheel Encoders<br/><b>/wheel/ticks</b><br/><i>(std_msgs/Int64MultiArray)</i>"]:::hw
        IN_IMU["BNO055 9-DoF IMU<br/><b>/imu/data</b><br/><i>(sensor_msgs/Imu)</i>"]:::hw
        IN_RGB["RealSense RGB Feed<br/><b>/camera/image_raw</b><br/><i>(sensor_msgs/Image)</i>"]:::hw
        IN_DEPTH["RealSense Depth Stream<br/><b>/camera/depth/image_raw</b><br/><i>(sensor_msgs/Image)</i>"]:::hw
        IN_ARUCO["ArUco Landmark Poses<br/><b>/perception/aruco_pose</b><br/><i>(geometry_msgs/PoseStamped)</i>"]:::hw
        IN_ROCKS["Terrain Rock PointCloud<br/><b>/bridge/pointcloud</b><br/><i>(sensor_msgs/PointCloud2)</i>"]:::hw
    end

    subgraph Processing ["⚙️ SLAM PROCESSING MODULES"]
        NODE_ODOM["<b>Block 1: encoder_ticks_to_odom</b><br/>• 4-wheel differential kinematics<br/>• Single-wheel slip isolation<br/>• Outputs raw linear & angular twist"]:::block
        NODE_SLIP["<b>Block 2: heuristic_slip_checker</b><br/>• Compares wheel yaw vs IMU gyro<br/>• Clamps linear velocity on slip<br/>• Dynamically inflates covariance (10^3)"]:::block
        NODE_VISION["<b>Block 3: vision_helper</b><br/>• Decimation & spatial noise reduction<br/>• Temporal filter & hole filling"]:::block
        NODE_EKF["<b>Block 4: robot_localization (EKF)</b><br/>• Fuses wheel twist + IMU @ 100Hz<br/>• Smooth zero-latency local odometry<br/>• Broadcasts odom ➔ base_link TF"]:::ekf
        NODE_RTAB["<b>Block 5: RTAB-Map SLAM</b><br/>• FAST/GFTT visual feature memory<br/>• Loop closure & ArUco constraint graph<br/>• Calculates map ➔ odom drift offset"]:::ekf
        NODE_COSTMAP["<b>Block 6: nav2_costmap_2d</b><br/>• Static global map + dynamic rock layer<br/>• Exponential inflation safety corridor"]:::nav
    end

    subgraph Outputs ["📤 SLAM SYSTEM OUTPUTS"]
        OUT_LOCAL["<b>To MPPI Local Controller:</b><br/>• /odometry/filtered (100Hz)<br/>• TF: odom ➔ base_link"]:::out
        OUT_GLOBAL["<b>To Smac Global Planner:</b><br/>• /map (OccupancyGrid)<br/>• TF: map ➔ odom (Drift correction)"]:::out
        OUT_COSTMAP["<b>To Nav2 Obstacle Avoidance:</b><br/>• /global_costmap/costmap<br/>• /local_costmap/costmap"]:::out
        OUT_SLIP["<b>To Rover Safety Layer:</b><br/>• /wheel/slip_detected"]:::out
    end

    IN_TICKS --> NODE_ODOM
    NODE_ODOM -->|"<b>/wheel/odom_raw</b>"| NODE_SLIP
    NODE_ODOM -->|"<b>/wheel/single_wheel_slip</b>"| NODE_SLIP

    IN_IMU --> NODE_SLIP
    NODE_SLIP -->|"<b>/wheel/odom_filtered</b> (cov inflated)"| NODE_EKF
    NODE_SLIP -->|"<b>/wheel/slip_detected</b>"| OUT_SLIP

    IN_IMU --> NODE_EKF
    IN_DEPTH --> NODE_VISION
    NODE_VISION -->|"<b>/camera/depth/filtered</b>"| NODE_RTAB
    IN_RGB --> NODE_RTAB
    IN_ARUCO --> NODE_RTAB

    NODE_EKF -->|"<b>/odometry/filtered</b> (100Hz)"| NODE_RTAB
    NODE_EKF -->|"TF: odom ➔ base_link"| OUT_LOCAL
    NODE_RTAB -->|"<b>/map</b>"| NODE_COSTMAP
    NODE_RTAB -->|"TF: map ➔ odom"| OUT_GLOBAL
    IN_ROCKS --> NODE_COSTMAP
    NODE_COSTMAP --> OUT_COSTMAP
"""
    },
    {
        "folder": "2-SLAM",
        "filename": "01_encoder_ticks_to_odom_kinematics.png",
        "title": "Deep Dive: Wheel Odometry Kinematics (Block 1)",
        "badge": "SLAM Block 1",
        "subtitle": "encoder_ticks_to_odom.py: differential skid-steer kinematics and per-wheel slip isolation",
        "mermaid": """flowchart LR
    classDef input fill:#0f172a,stroke:#64748b,stroke-width:2px,color:#f8fafc;
    classDef proc fill:#1e1b4b,stroke:#818cf8,stroke-width:2px,color:#f8fafc;
    classDef out fill:#064e3b,stroke:#34d399,stroke-width:2px,color:#f8fafc;

    TICKS["<b>/wheel/ticks</b><br/>[std_msgs/Int64MultiArray]<br/>4x wheel encoder counts"]:::input

    subgraph Math ["Kinematics Math"]
        DELTA["<b>Tick Delta Calculation:</b><br/>&Delta;ticks = ticks_curr - ticks_prev<br/>&Delta;s = (&Delta;ticks / CPR) &middot; 2&pi;R"]:::proc
        DIFF["<b>Skid-Steer Kinematics:</b><br/>v_x = (v_right + v_left) / 2<br/>&omega;_z = (v_right - v_left) / wheelbase"]:::proc
        ISOLATE["<b>Single-Wheel Slip Isolation:</b><br/>Compares Front vs Rear speed per side.<br/>Flags individual spinning wheel."]:::proc
    end

    subgraph Outputs ["Published Topics"]
        RAW_ODOM["<b>/wheel/odom_raw</b><br/>nav_msgs/Odometry (twist only)"]:::out
        SLIP_FLAG["<b>/wheel/single_wheel_slip</b><br/>std_msgs/Bool"]:::out
        SPEEDS["<b>/wheel/per_wheel_speeds</b><br/>std_msgs/Float64MultiArray"]:::out
    end

    TICKS --> DELTA --> DIFF --> RAW_ODOM
    DELTA --> ISOLATE --> SLIP_FLAG
    DELTA --> SPEEDS
"""
    },
    {
        "folder": "2-SLAM",
        "filename": "02_heuristic_slip_checker_and_covariance.png",
        "title": "Deep Dive: Heuristic Slip Checker & Covariance Inflation (Block 2)",
        "badge": "SLAM Block 2",
        "subtitle": "heuristic_slip_checker.py: comparing wheel yaw vs IMU gyro, clamping velocity, and dynamically scaling EKF covariance",
        "mermaid": """flowchart TD
    classDef input fill:#0f172a,stroke:#64748b,stroke-width:2px,color:#f8fafc;
    classDef check fill:#854d0e,stroke:#facc15,stroke-width:2px,color:#f8fafc;
    classDef proc fill:#1e1b4b,stroke:#818cf8,stroke-width:2px,color:#f8fafc;
    classDef out fill:#064e3b,stroke:#34d399,stroke-width:2px,color:#f8fafc;

    IN_ODOM["<b>/wheel/odom_raw</b> (v_wheel, &omega;_wheel)"]:::input
    IN_IMU["<b>/imu/data</b> (&omega;_imu_gyro)"]:::input

    subgraph Checks ["Slip Detection Heuristics"]
        YAW_DIFF["<b>Yaw Rate Divergence:</b><br/>|&omega;_wheel - &omega;_imu| &gt; 0.20 rad/s"]:::check
        STALL_CHECK["<b>Wheel Spin with Zero Motion:</b><br/>v_wheel &gt; 0.15 m/s while IMU linear accel &approx; 0"]:::check
        DECISION{"Is Wheel Slipping in Sand?"}:::check
    end

    subgraph Actions ["Adaptive Filtering Action"]
        NORMAL["<b>Normal Traction Mode:</b><br/>• Pass raw v_x and &omega;_z unaltered<br/>• Normal covariance: cov(v) = 0.05"]:::proc
        SLIP_ACTIVE["<b>Slip Rejection Mode:</b><br/>• Clamp linear velocity v_x = 0.0 m/s<br/>• Overwrite &omega;_z with IMU gyro reading<br/>• Inflate velocity covariance: cov(v) = 1,000.0"]:::proc
    end

    subgraph Outputs ["Safe Filtered Output"]
        OUT_ODOM["<b>/wheel/odom_filtered</b> &rarr; Sent to EKF Node"]:::out
        OUT_SAFETY["<b>/wheel/slip_detected</b> &rarr; Safety Alert"]:::out
    end

    IN_ODOM --> YAW_DIFF
    IN_IMU --> YAW_DIFF
    IN_ODOM --> STALL_CHECK
    IN_IMU --> STALL_CHECK
    YAW_DIFF --> DECISION
    STALL_CHECK --> DECISION
    DECISION -->|"NO"| NORMAL --> OUT_ODOM
    DECISION -->|"YES"| SLIP_ACTIVE --> OUT_ODOM
    SLIP_ACTIVE --> OUT_SAFETY
"""
    },
    {
        "folder": "2-SLAM",
        "filename": "03_realsense_depth_postprocessing_filters.png",
        "title": "Deep Dive: RealSense D435 Post-Processing Filters (Block 3)",
        "badge": "SLAM Block 3",
        "subtitle": "vision_helper: decimation, spatial smoothing, temporal persistence, and sunlight IR noise suppression",
        "mermaid": """flowchart LR
    classDef inout fill:#0f172a,stroke:#38bdf8,stroke-width:2px,color:#f8fafc;
    classDef filt fill:#1e1b4b,stroke:#818cf8,stroke-width:2px,color:#f8fafc;

    RAW_DEPTH["<b>Raw Depth Stream:</b><br/>/camera/depth/image_raw (848x480 @ 30Hz)"]:::inout

    subgraph Filters ["D435 Filter Chain"]
        F1["<b>1. Decimation Filter:</b><br/>Downsamples 2x &rarr; reduces compute load"]:::filt
        F2["<b>2. Spatial Filter:</b><br/>Edge-preserving 1D/2D smoothing"]:::filt
        F3["<b>3. Temporal Filter:</b><br/>Averages pixel depth over 3 frames to stop flicker"]:::filt
        F4["<b>4. Hole-Filling Filter:</b><br/>Interpolates missing specular reflection pixels"]:::filt
        F5["<b>5. Max Range Threshold:</b><br/>Clamps depth values &gt; 4.0m to Infinity"]:::filt
    end

    CLEAN_DEPTH["<b>/camera/depth/filtered</b><br/>Input to RTAB-Map SLAM Node"]:::inout

    RAW_DEPTH --> F1 --> F2 --> F3 --> F4 --> F5 --> CLEAN_DEPTH
"""
    },
    {
        "folder": "2-SLAM",
        "filename": "04_robot_localization_ekf_100hz_fusion.png",
        "title": "Deep Dive: robot_localization EKF 100Hz Local Fusion (Block 4)",
        "badge": "SLAM Block 4",
        "subtitle": "Continuous high-rate sensor fusion of wheel twist and IMU attitude, guaranteeing zero-jump local odometry",
        "mermaid": """flowchart TD
    classDef input fill:#0f172a,stroke:#64748b,stroke-width:2px,color:#f8fafc;
    classDef ekf fill:#581c87,stroke:#c084fc,stroke-width:2px,color:#f8fafc;
    classDef out fill:#064e3b,stroke:#34d399,stroke-width:2px,color:#f8fafc;

    subgraph Inputs ["Sensor Inputs @ 50-100Hz"]
        WHEEL["<b>/wheel/odom_filtered:</b><br/>Linear velocity v_x, angular &omega;_z<br/>Dynamic covariance matrix"]:::input
        IMU["<b>/imu/data:</b><br/>Orientation (Roll, Pitch, Yaw)<br/>Angular velocity (&omega;_x, &omega;_y, &omega;_z)"]:::input
    end

    subgraph EKF ["robot_localization EKF Node (ekf_node)"]
        FUSION["<b>15-State Extended Kalman Filter:</b><br/>• Position (X, Y, Z)<br/>• Orientation (Roll, Pitch, Yaw)<br/>• Velocities (X_dot, Y_dot, Z_dot)<br/>• Angular Velocities (&omega;_x, &omega;_y, &omega;_z)<br/>• Linear Accelerations"]:::ekf
        REJECT["<b>Automatic Slip Rejection:</b><br/>When wheel cov = 1000, Kalman gain K &rarr; 0<br/>EKF relies exclusively on IMU gyro & attitude"]:::ekf
    end

    subgraph Outputs ["Smooth Local Trajectory @ 100Hz"]
        ODOM_FILT["<b>/odometry/filtered</b> (nav_msgs/Odometry)<br/>Sent to MPPI Controller & RTAB-Map"]:::out
        TF_ODOM["<b>TF Broadcast: odom ➔ base_link</b><br/>Continuous, zero coordinate jump"]:::out
    end

    WHEEL --> FUSION
    IMU --> FUSION
    FUSION --> REJECT
    REJECT --> ODOM_FILT
    REJECT --> TF_ODOM
"""
    },
    {
        "folder": "2-SLAM",
        "filename": "05_rtabmap_visual_slam_and_loop_closure.png",
        "title": "Deep Dive: RTAB-Map Visual SLAM & Loop Closures (Block 5)",
        "badge": "SLAM Block 5",
        "subtitle": "Visual FAST/GFTT feature tracking, memory management graph, ArUco landmark constraints, and map->odom drift offset",
        "mermaid": """flowchart TD
    classDef input fill:#0f172a,stroke:#64748b,stroke-width:2px,color:#f8fafc;
    classDef slam fill:#581c87,stroke:#c084fc,stroke-width:2px,color:#f8fafc;
    classDef out fill:#064e3b,stroke:#34d399,stroke-width:2px,color:#f8fafc;

    subgraph Streams ["RGB-D & Odometry Streams"]
        RGB["<b>/camera/image_raw</b>"]:::input
        DEPTH["<b>/camera/depth/filtered</b>"]:::input
        ODOM["<b>/odometry/filtered</b> (from EKF)"]:::input
        ARUCO["<b>/perception/aruco_pose</b> (Landmark)"]:::input
    end

    subgraph RTAB ["RTAB-Map SLAM Core"]
        FEATURES["<b>Visual Feature Extractor:</b><br/>FAST / GFTT corners + BRIEF descriptors"]:::slam
        MEMORY["<b>Memory Management:</b><br/>Working Memory (WM) + Long-Term Memory (LTM)"]:::slam
        LOOP["<b>Bayesian Loop Closure Detection:</b><br/>Identifies previously visited locations"]:::slam
        GRAPH["<b>Pose Graph Optimization:</b><br/>g2o / GTSAM optimizes camera trajectory<br/>Incorporates 6-DoF ArUco landmark constraints"]:::slam
    end

    subgraph Outputs ["Global World SLAM Outputs"]
        MAP_GRID["<b>/map (OccupancyGrid)</b><br/>Static 2D obstacle & free-space grid"]:::out
        TF_MAP["<b>TF Broadcast: map ➔ odom (1-5 Hz)</b><br/>Absorbs global drift without jerking local controller"]:::out
    end

    RGB --> FEATURES
    DEPTH --> FEATURES
    ODOM --> MEMORY
    FEATURES --> MEMORY --> LOOP --> GRAPH
    ARUCO --> GRAPH
    GRAPH --> MAP_GRID
    GRAPH --> TF_MAP
"""
    },
    {
        "folder": "2-SLAM",
        "filename": "06_nav2_costmap_2d_layering.png",
        "title": "Deep Dive: Nav2 Costmap 2D Layering & Inflation (Block 6)",
        "badge": "SLAM Block 6",
        "subtitle": "nav2_costmap_2d: fusing static SLAM map, dynamic perception rocks, and safety inflation envelopes",
        "mermaid": """flowchart LR
    classDef inout fill:#0f172a,stroke:#38bdf8,stroke-width:2px,color:#f8fafc;
    classDef layer fill:#7f1d1d,stroke:#ef4444,stroke-width:2px,color:#f8fafc;

    subgraph Ingestion ["Map & Point Cloud Inputs"]
        MAP_IN["<b>/map:</b> Static OccupancyGrid (RTAB-Map)"]:::inout
        ROCKS_IN["<b>/bridge/pointcloud:</b> Dynamic rocks (Perception)"]:::inout
    end

    subgraph Layers ["Costmap Layer Plugin Stack"]
        L1["<b>1. Static Layer:</b><br/>Projects global SLAM walls/borders"]:::layer
        L2["<b>2. Obstacle Layer:</b><br/>Ray-traces real-time 3D rock points"]:::layer
        L3["<b>3. Inflation Layer:</b><br/>Exponential decay around obstacles<br/>Inscribed radius = 0.50m (rover body)"]:::layer
    end

    subgraph Costmaps ["Planning Grids"]
        GLOBAL_CM["<b>/global_costmap/costmap:</b><br/>Used by Smac Global Planner"]:::inout
        LOCAL_CM["<b>/local_costmap/costmap:</b><br/>Used by MPPI Local Controller"]:::inout
    end

    MAP_IN --> L1
    ROCKS_IN --> L2
    L1 --> L3
    L2 --> L3
    L3 --> GLOBAL_CM
    L3 --> LOCAL_CM
"""
    },

    # =========================================================================
    # 3. NAV2 & PATH PLANNING (General_Docs/3-Nav2/)
    # =========================================================================
    {
        "folder": "3-Nav2",
        "filename": "00_nav2_master_pipeline.png",
        "title": "Nav2 Navigation & Path Planning Master Architecture",
        "badge": "Nav2 Master",
        "subtitle": "Smac Hybrid A* Reeds-Shepp global planner, MPPI 2,000 rollout controller, and costmap obstacle bridge",
        "mermaid": """flowchart TD
    classDef ext fill:#0f172a,stroke:#64748b,stroke-width:2px,color:#f8fafc;
    classDef bridge fill:#1e1b4b,stroke:#818cf8,stroke-width:2px,color:#f8fafc;
    classDef navCore fill:#7f1d1d,stroke:#ef4444,stroke-width:2px,color:#f8fafc;
    classDef ctrl fill:#7c2d12,stroke:#ea580c,stroke-width:2px,color:#f8fafc;

    subgraph Inputs ["🌐 EXTERNAL SUBSYSTEM INPUTS"]
        IN_MAP["<b>RTAB-Map SLAM:</b><br/>/map (OccupancyGrid)"]:::ext
        IN_ODOM["<b>robot_localization EKF:</b><br/>/odometry/filtered & TFs"]:::ext
        IN_PERC["<b>Perception Subsystem:</b><br/>/perception/obstacles_only"]:::ext
        IN_GOAL["<b>Mission Dispatcher / UI:</b><br/>/goal_pose (Target Waypoint)"]:::ext
    end

    subgraph Ingestion ["🌉 INGESTION & BRIDGE LAYER"]
        NODE_BRIDGE["<b>costmap_bridge_node</b><br/>Samples 3D bounding box perimeters<br/>into dense synthetic PointCloud2"]:::bridge
    end

    subgraph NavCore ["🧭 NAV2 NAVIGATION CORE"]
        GC["<b>Global Costmap Server</b><br/>Static /map + Obstacle Layer + Inflation"]:::navCore
        LC["<b>Local Costmap Server (10x10m)</b><br/>Rolling window in odom frame"]:::navCore
        BT["<b>Behavior Tree Navigator</b><br/>Central orchestrator for planning & recovery"]:::navCore
        SMAC["<b>Smac Hybrid A* Global Planner</b><br/>Kinematically feasible Reeds-Shepp routes"]:::navCore
        MPPI["<b>MPPI Local Controller</b><br/>2,000 parallel rollouts @ 20Hz"]:::navCore
        SAFETY["<b>Collision Monitor & Smoother</b><br/>Velocity profile ramping & safety limits"]:::navCore
    end

    subgraph Outputs ["⚙️ ACTUATION & MOTOR CONTROL"]
        MOTOR_DRV["<b>motor_driver Node</b><br/>Differential / Skid-Steer Kinematics"]:::ctrl
        MOTORS["<b>Hardware Actuators</b><br/>4x Wheel ESCs & High-Torque DC Motors"]:::ctrl
    end

    IN_PERC --> NODE_BRIDGE
    NODE_BRIDGE -->|"<b>/bridge/pointcloud</b>"| GC
    NODE_BRIDGE -->|"<b>/bridge/pointcloud</b>"| LC
    IN_MAP --> GC
    IN_ODOM --> LC
    IN_ODOM --> MPPI

    IN_GOAL --> BT
    BT -->|"ComputePathToPose"| SMAC
    GC --> SMAC
    SMAC -->|"<b>/plan</b> (Global Waypoints)"| BT
    BT -->|"FollowPath (/plan)"| MPPI
    LC --> MPPI

    MPPI -->|"<b>/cmd_vel_nav</b>"| SAFETY
    SAFETY -->|"<b>/cmd_vel</b> (Twist)"| MOTOR_DRV
    MOTOR_DRV -->|"Wheel PWM / Speeds"| MOTORS

    MPPI -.->|"NO_VALID_TRAJECTORY (Blocked)"| BT
    BT -.->|"Trigger Recovery / Replan"| SMAC
"""
    },
    {
        "folder": "3-Nav2",
        "filename": "01_costmap_bridge_ingestion_block.png",
        "title": "Deep Dive: Costmap Bridge Ingestion Node (Block 1)",
        "badge": "Nav2 Block 1",
        "subtitle": "costmap_bridge_node: converting 3D bounding boxes into dense boundary point clouds for 2D costmaps",
        "mermaid": """flowchart LR
    classDef inout fill:#0f172a,stroke:#38bdf8,stroke-width:2px,color:#f8fafc;
    classDef proc fill:#1e1b4b,stroke:#818cf8,stroke-width:2px,color:#f8fafc;

    IN_BBOX["<b>/perception/obstacles_only</b><br/>vision_msgs/Detection3DArray<br/>3D Bounding Boxes & Centroids"]:::inout

    subgraph Bridge ["costmap_bridge_node (C++)"]
        PERIM["<b>1. Perimeter Extraction:</b><br/>Calculates 4 ground edges per box"]:::proc
        SAMPLE["<b>2. Linear Interpolation:</b><br/>Samples synthetic points every 0.04m along edges"]:::proc
        PC_BUILD["<b>3. PointCloud2 Generation:</b><br/>Packages points with timestamp & frame_id"]:::proc
    end

    OUT_CLOUD["<b>/bridge/pointcloud</b><br/>sensor_msgs/PointCloud2<br/>Ingested by Global & Local Costmaps"]:::inout

    IN_BBOX --> PERIM --> SAMPLE --> PC_BUILD --> OUT_CLOUD
"""
    },
    {
        "folder": "3-Nav2",
        "filename": "02_global_and_local_costmap_servers.png",
        "title": "Deep Dive: Global vs Local Costmap Servers (Block 2)",
        "badge": "Nav2 Block 2",
        "subtitle": "nav2_costmap_2d: comparing global macro cost grid vs 10x10m rolling window local costmap",
        "mermaid": """flowchart TD
    classDef map fill:#7f1d1d,stroke:#ef4444,stroke-width:2px,color:#f8fafc;
    classDef inout fill:#0f172a,stroke:#38bdf8,stroke-width:2px,color:#f8fafc;

    subgraph Global ["Global Costmap Server (nav2_costmap_2d)"]
        GC_SPEC["• Frame: map<br/>• Size: Entire Mars Yard (50x50m)<br/>• Resolution: 0.05m<br/>• Tracks full static world + explored boulders"]:::map
        GC_LAYERS["Plugins: StaticLayer + ObstacleLayer + InflationLayer"]:::map
        GC_OUT["<b>/global_costmap/costmap</b> &rarr; Smac Planner"]:::inout
    end

    subgraph Local ["Local Costmap Server (nav2_costmap_2d)"]
        LC_SPEC["• Frame: odom (Rolling Window)<br/>• Size: 10x10 meters centered on rover<br/>• Resolution: 0.04m<br/>• Real-time immediate hazard avoidance"]:::map
        LC_LAYERS["Plugins: ObstacleLayer + InflationLayer (High Decay)"]:::map
        LC_OUT["<b>/local_costmap/costmap</b> &rarr; MPPI Controller"]:::inout
    end

    GC_SPEC --> GC_LAYERS --> GC_OUT
    LC_SPEC --> LC_LAYERS --> LC_OUT
"""
    },
    {
        "folder": "3-Nav2",
        "filename": "03_smac_hybrid_a_star_planner_block.png",
        "title": "Deep Dive: Smac Hybrid A* Global Planner (Block 3)",
        "badge": "Nav2 Block 3",
        "subtitle": "nav2_smac_planner: 3D search space (X, Y, &theta;) generating kinematically feasible Reeds-Shepp curves",
        "mermaid": """flowchart LR
    classDef inout fill:#0f172a,stroke:#38bdf8,stroke-width:2px,color:#f8fafc;
    classDef smac fill:#7f1d1d,stroke:#ef4444,stroke-width:2px,color:#f8fafc;

    subgraph Inputs ["Inputs"]
        START["Rover Start Pose (X, Y, &theta;)"]:::inout
        GOAL["Goal Pose (X, Y, &theta;)"]:::inout
        GRID["Global Costmap Grid"]:::inout
    end

    subgraph SmacSearch ["Smac Hybrid A* Core"]
        HEURISTIC["<b>Dual Heuristics:</b><br/>• Non-Holonomic Distance (Dubins/Reeds-Shepp)<br/>• 2D Obstacle Dijkstra BFS"]:::smac
        EXPAND["<b>Node Expansion:</b><br/>Forward & Reverse primitives<br/>Min Turning Radius = 0.8m"]:::smac
        ANALYTIC["<b>Analytic Expansion:</b><br/>Direct Reeds-Shepp curve to goal when collision-free"]:::smac
        SMOOTH["<b>Path Smoother:</b><br/>Conjugate gradient curvature minimization"]:::smac
    end

    OUT_PATH["<b>/plan (nav_msgs/Path)</b><br/>Continuous smooth waypoints to goal"]:::inout

    START --> HEURISTIC
    GOAL --> HEURISTIC
    GRID --> HEURISTIC
    HEURISTIC --> EXPAND --> ANALYTIC --> SMOOTH --> OUT_PATH
"""
    },
    {
        "folder": "3-Nav2",
        "filename": "04_mppi_controller_rollouts_block.png",
        "title": "Deep Dive: MPPI Controller 2,000 Rollouts @ 20Hz (Block 4)",
        "badge": "Nav2 Block 4",
        "subtitle": "nav2_mppi_controller: Model Predictive Path Integral controller optimizing trajectories with multi-criteria critics",
        "mermaid": """flowchart TD
    classDef inout fill:#0f172a,stroke:#38bdf8,stroke-width:2px,color:#f8fafc;
    classDef mppi fill:#7f1d1d,stroke:#ef4444,stroke-width:2px,color:#f8fafc;
    classDef critic fill:#1e1b4b,stroke:#818cf8,stroke-width:2px,color:#f8fafc;

    INPUTS["<b>Inputs:</b> /plan, /local_costmap/costmap, /odometry/filtered @ 20Hz"]:::inout

    subgraph Sampling ["Trajectory Generation"]
        ROLLOUTS["<b>Generate 2,000 Stochastic Rollouts:</b><br/>Applies Gaussian noise to control sequences over 2.5s horizon"]:::mppi
    end

    subgraph Critics ["Cost Function Evaluation"]
        C1["<b>GoalCritic:</b> Rewards closing distance to goal"]:::critic
        C2["<b>PathAlignCritic:</b> Penalizes deviation from global route"]:::critic
        C3["<b>ObstacleCritic:</b> Severe penalty for high-costmap cells"]:::critic
        C4["<b>ConstraintCritic:</b> Enforces rover max speed & skid limits"]:::critic
    end

    subgraph Optimization ["Optimal Control Synthesis"]
        WEIGHT["<b>Softmax Weighting:</b><br/>w_k = exp(-1/&lambda; &middot; Cost_k) / &Sigma;"]:::mppi
        CMD["<b>Optimal Control Command:</b><br/>u* = &Sigma;(w_k &middot; u_k)"]:::mppi
    end

    OUT_CMD["<b>/cmd_vel_nav:</b> Optimal linear (v) & angular (&omega;) velocity"]:::inout

    INPUTS --> ROLLOUTS --> C1 & C2 & C3 & C4 --> WEIGHT --> CMD --> OUT_CMD
"""
    },
    {
        "folder": "3-Nav2",
        "filename": "05_behavior_tree_replanning_block.png",
        "title": "Deep Dive: Behavior Tree Replanning & Recovery Escalation (Block 5)",
        "badge": "Nav2 Block 5",
        "subtitle": "Central navigation state machine: dynamic obstacle detection, detour replanning, and recovery behaviors",
        "mermaid": """flowchart TD
    classDef bt fill:#1e1b4b,stroke:#818cf8,stroke-width:2px,color:#f8fafc;
    classDef mppi fill:#7f1d1d,stroke:#ef4444,stroke-width:2px,color:#f8fafc;
    classDef decision fill:#854d0e,stroke:#facc15,stroke-width:2px,color:#f8fafc;
    classDef act fill:#064e3b,stroke:#34d399,stroke-width:2px,color:#f8fafc;

    BT["<b>Behavior Tree Navigator</b><br/>Orchestrates global route replanning & recovery sequences"]:::bt
    SMAC["<b>Smac Hybrid A* Global Planner</b><br/>Computes macro route /plan on updated Global Costmap"]:::bt
    MPPI["<b>MPPI Local Controller (20Hz)</b><br/>Simulates 2,000 trajectory rollouts in parallel"]:::mppi

    BT -->|"ComputePathToPose"| SMAC
    SMAC -->|"/plan"| BT
    BT -->|"FollowPath (/plan)"| MPPI

    MPPI -->|"Evaluation Cycle"| EVAL{"Can MPPI swerve safely?"}:::decision
    EVAL -->|"YES (Path Open)"| DRIVE["<b>Output /cmd_vel</b><br/>Smoothly bypasses obstacle & rejoins global plan"]:::act
    EVAL -->|"NO (Trapped / Blocked)"| FAIL["<b>Return FAILURE</b><br/>NO_VALID_TRAJECTORY"]:::decision

    FAIL -->|"Trigger Immediate Replan"| BT
    BT -->|"Compute New Detour"| SMAC
    SMAC -->|"If Path Blocked Everywhere"| REC["<b>Escalated Recovery Behaviors:</b><br/>1. Clear Costmap (Remove transient sensor noise)<br/>2. BackUp 0.8m safely<br/>3. Spin 360° to scan terrain<br/>4. Re-attempt global planning"]:::mppi
"""
    },
    {
        "folder": "3-Nav2",
        "filename": "06_velocity_smoother_and_skid_steer_bridge.png",
        "title": "Deep Dive: Velocity Smoother & Motor Driver Bridge (Block 6)",
        "badge": "Nav2 Block 6",
        "subtitle": "Velocity ramping, acceleration jerk limiting, and differential/skid-steer motor command translation",
        "mermaid": """flowchart LR
    classDef inout fill:#0f172a,stroke:#38bdf8,stroke-width:2px,color:#f8fafc;
    classDef proc fill:#7c2d12,stroke:#ea580c,stroke-width:2px,color:#f8fafc;

    IN_RAW_CMD["<b>/cmd_vel_nav</b><br/>Raw MPPI velocity command"]:::inout

    subgraph Smoother ["Velocity Smoother & Safety Monitor"]
        RAMP["<b>Acceleration Limiter:</b><br/>Clamps linear & angular acceleration slopes"]:::proc
        ENVELOPE["<b>Safety Enclosure:</b><br/>Slows rover near obstacles or on steep pitch angles"]:::proc
    end

    subgraph MotorBridge ["motor_driver Node"]
        SKID["<b>Skid-Steer Kinematics:</b><br/>&omega;_left = (v - &omega; &middot; W/2) / R<br/>&omega;_right = (v + &omega; &middot; W/2) / R"]:::proc
        DEADBAND["<b>Deadband & Slew Rate Compensation</b>"]:::proc
    end

    OUT_MOTORS["<b>Hardware Motor Commands</b><br/>Left/Right PWM & Direction to ESCs"]:::inout

    IN_RAW_CMD --> RAMP --> ENVELOPE --> SKID --> DEADBAND --> OUT_MOTORS
"""
    },

    # =========================================================================
    # 4. HARDWARE & ELECTRICAL ARCHITECTURE (General_Docs/4-Hardware/)
    # =========================================================================
    {
        "folder": "4-Hardware",
        "filename": "00_hardware_master_architecture.png",
        "title": "Hardware Electrical & Computing Master Architecture",
        "badge": "Hardware Master",
        "subtitle": "Master power distribution, NVIDIA Jetson Orin Nano compute, STM32 Blackpill ECU, sensors, and 4WD skid-steer actuation",
        "mermaid": """flowchart TD
    classDef compute fill:#1e1b4b,stroke:#6366f1,stroke-width:2px,color:#f8fafc;
    classDef power fill:#854d0e,stroke:#facc15,stroke-width:2px,color:#f8fafc;
    classDef mcu fill:#064e3b,stroke:#10b981,stroke-width:2px,color:#f8fafc;
    classDef act fill:#7c2d12,stroke:#ea580c,stroke-width:2px,color:#f8fafc;
    classDef sens fill:#0f172a,stroke:#38bdf8,stroke-width:2px,color:#f8fafc;

    subgraph PowerSystem ["⚡ POWER DISTRIBUTION LAYER"]
        BAT["<b>20V LiPo / Li-Ion Battery Pack</b><br/>High-discharge main supply"]:::power
        PDB["<b>Custom Power Distribution Board (PDB)</b><br/>Fuses, current sensors & voltage regulators"]:::power
        REG_JETSON["19V / 12V Buck-Boost (Compute)"]:::power
        REG_LOGIC["5V / 3.3V Regulators (Logic & Micro)"]:::power
        RAW_BUS["20V High-Current Raw Bus (Motors)"]:::power
    end

    subgraph ComputeLayer ["🧠 MASTER COMPUTING (Jetson Orin Nano 8GB)"]
        JETSON["<b>NVIDIA Jetson Orin Nano</b><br/>Ubuntu 24.04 + ROS 2 Jazzy<br/>• SLAM & RTAB-Map<br/>• 3D Terrain Perception<br/>• Nav2 MPPI & Smac Planner"]:::compute
    end

    subgraph LowLevelECU ["⚡ LOW-LEVEL ECU (STM32 Blackpill)"]
        STM32["<b>STM32F411 Blackpill Microcontroller</b><br/>Bare-metal / FreeRTOS motor control<br/>• 4x Quadrature Encoder Interrupts<br/>• Hardware PWM Generation<br/>• Safety E-Stop Interlock"]:::mcu
    end

    subgraph Sensors ["📡 SENSOR ARRAY"]
        REALSENSE["<b>Intel RealSense D435i</b><br/>RGB-D Stereo Camera"]:::sens
        IMU["<b>BNO055 / MPU-6050 IMU</b><br/>9-DoF Orientation & Gyro"]:::sens
        ENCODERS["<b>4x Wheel Encoders</b><br/>High-resolution optical ticks"]:::sens
    end

    subgraph Drivetrain ["⚙️ DRIVETRAIN & ACTUATORS"]
        DRIVERS["<b>4x High-Power DC Motor Drivers</b><br/>H-Bridge PWM / Direction"]:::act
        MOTORS["<b>4x Planetary Gearbox DC Motors</b><br/>High-torque 4WD Skid-Steer"]:::act
    end

    BAT --> PDB
    PDB --> RAW_BUS --> DRIVERS
    PDB --> REG_JETSON --> JETSON
    PDB --> REG_LOGIC --> STM32

    JETSON <-->|"High-Speed USB Serial (115200 / 921600 baud)"| STM32
    REALSENSE -->|"USB 3.0 (RGB-D Stream)"| JETSON
    IMU -->|"I2C / UART (/imu/data)"| JETSON
    ENCODERS -->|"Hardware Ticks / Timer Interrupts"| STM32
    STM32 -->|"PWM & Direction Control"| DRIVERS
    DRIVERS --> MOTORS
"""
    },
    {
        "folder": "4-Hardware",
        "filename": "01_power_distribution_block.png",
        "title": "Deep Dive: Power Distribution & Voltage Regulation (Block 1)",
        "badge": "Hardware Block 1",
        "subtitle": "Main battery supply, Power Distribution Board (PDB), buck-boost regulation, and isolated ground domains",
        "mermaid": """flowchart LR
    classDef power fill:#854d0e,stroke:#facc15,stroke-width:2px,color:#f8fafc;
    classDef load fill:#1e1b4b,stroke:#818cf8,stroke-width:2px,color:#f8fafc;

    BAT["<b>20V Main Battery Pack</b><br/>LiPo / Li-Ion High Discharge"]:::power

    subgraph PDB ["Custom PDB & Safety Protection"]
        ESTOP["<b>Emergency Stop (E-Stop)</b><br/>Physical & Wireless Relay Killswitch"]:::power
        FUSE["<b>Inline Fuse & Reverse Polarity</b><br/>Overcurrent & surge protection"]:::power
    end

    subgraph Rails ["Regulated Voltage Rails"]
        R1["<b>19V / 12V Buck-Boost:</b><br/>Supplies Jetson Orin Nano"]:::power
        R2["<b>5V / 3A BEC:</b><br/>Supplies STM32 & USB Peripherals"]:::power
        R3["<b>3.3V LDO:</b><br/>Supplies IMU & Logic Sensors"]:::power
        R4["<b>20V High-Current Raw Rail:</b><br/>Direct supply to 4x Motor Drivers"]:::power
    end

    subgraph Loads ["Subsystem Loads"]
        L_JETSON["NVIDIA Jetson Orin Nano"]:::load
        L_STM32["STM32 Blackpill ECU"]:::load
        L_SENSORS["BNO055 IMU & RealSense"]:::load
        L_MOTORS["4x DC Drive Motors"]:::load
    end

    BAT --> ESTOP --> FUSE
    FUSE --> R1 --> L_JETSON
    FUSE --> R2 --> L_STM32
    FUSE --> R3 --> L_SENSORS
    FUSE --> R4 --> L_MOTORS
"""
    },
    {
        "folder": "4-Hardware",
        "filename": "02_master_compute_and_microcontroller_bridge.png",
        "title": "Deep Dive: Master Compute & Microcontroller Bridge (Block 2)",
        "badge": "Hardware Block 2",
        "subtitle": "Jetson Orin Nano high-level autonomous intelligence communicating with STM32 Blackpill ECU via USB Serial",
        "mermaid": """flowchart LR
    classDef compute fill:#1e1b4b,stroke:#6366f1,stroke-width:2px,color:#f8fafc;
    classDef mcu fill:#064e3b,stroke:#10b981,stroke-width:2px,color:#f8fafc;
    classDef comm fill:#0f172a,stroke:#38bdf8,stroke-width:2px,color:#f8fafc;

    subgraph Master ["NVIDIA Jetson Orin Nano (ROS 2 Jazzy)"]
        ROS_NAV["<b>Nav2 MPPI Controller:</b><br/>Outputs desired /cmd_vel (v, &omega;)"]:::compute
        SERIAL_NODE["<b>serial_motor_bridge Node:</b><br/>Encodes packet: [START, v_L, v_R, CRC, END]"]:::compute
    end

    subgraph SerialLink ["High-Speed USB Serial Link"]
        USB["<b>USB CDC / UART @ 921,600 baud</b><br/>• Downlink: Motor Speed Setpoints (50 Hz)<br/>• Uplink: Raw Encoder Ticks & Faults (50 Hz)"]:::comm
    end

    subgraph Slave ["STM32F411 Blackpill ECU (Bare-Metal / FreeRTOS)"]
        PARSER["<b>Packet Parser & Checksum:</b><br/>Validates CRC & applies PID setpoint"]:::mcu
        PID["<b>Hardware Timer PWM & PID Loop:</b><br/>Controls H-Bridge drivers @ 20 kHz"]:::mcu
        ENCODER_TIMER["<b>Hardware Encoder Timers:</b><br/>TIM2, TIM3, TIM4, TIM5 capture ticks"]:::mcu
    end

    ROS_NAV --> SERIAL_NODE --> USB --> PARSER --> PID
    ENCODER_TIMER --> USB
"""
    },
    {
        "folder": "4-Hardware",
        "filename": "03_drivetrain_and_actuators_block.png",
        "title": "Deep Dive: 4WD Skid-Steer Drivetrain & Encoders (Block 3)",
        "badge": "Hardware Block 3",
        "subtitle": "4x Planetary DC Motors, H-Bridge Drivers, and optical quadrature encoder feedback",
        "mermaid": """flowchart LR
    classDef ctrl fill:#064e3b,stroke:#10b981,stroke-width:2px,color:#f8fafc;
    classDef driver fill:#7c2d12,stroke:#ea580c,stroke-width:2px,color:#f8fafc;
    classDef motor fill:#1e1b4b,stroke:#6366f1,stroke-width:2px,color:#f8fafc;
    classDef sens fill:#0f172a,stroke:#38bdf8,stroke-width:2px,color:#f8fafc;

    MCU["<b>STM32 Blackpill ECU</b>"]:::ctrl

    subgraph Drivers ["Motor Drivers"]
        DRV1["Left-Front Driver (H-Bridge)"]:::driver
        DRV2["Left-Rear Driver (H-Bridge)"]:::driver
        DRV3["Right-Front Driver (H-Bridge)"]:::driver
        DRV4["Right-Rear Driver (H-Bridge)"]:::driver
    end

    subgraph Actuators ["High-Torque DC Motors"]
        M1["Left-Front DC Motor"]:::motor
        M2["Left-Rear DC Motor"]:::motor
        M3["Right-Front DC Motor"]:::motor
        M4["Right-Rear DC Motor"]:::motor
    end

    subgraph Feedback ["Quadrature Optical Encoders"]
        E1["Encoder LF (Channel A/B)"]:::sens
        E2["Encoder LR (Channel A/B)"]:::sens
        E3["Encoder RF (Channel A/B)"]:::sens
        E4["Encoder RR (Channel A/B)"]:::sens
    end

    MCU -->|"PWM & DIR"| DRV1 --> M1
    MCU -->|"PWM & DIR"| DRV2 --> M2
    MCU -->|"PWM & DIR"| DRV3 --> M3
    MCU -->|"PWM & DIR"| DRV4 --> M4

    M1 -.-> E1 -->|"Ticks"| MCU
    M2 -.-> E2 -->|"Ticks"| MCU
    M3 -.-> E3 -->|"Ticks"| MCU
    M4 -.-> E4 -->|"Ticks"| MCU
"""
    },
    {
        "folder": "4-Hardware",
        "filename": "04_sensor_suite_and_buses_block.png",
        "title": "Deep Dive: Sensor Suite & Hardware Buses (Block 4)",
        "badge": "Hardware Block 4",
        "subtitle": "Complete pinout & communication bus mapping for RealSense D435i, BNO055 IMU, and Wheel Encoders",
        "mermaid": """flowchart TD
    classDef sens fill:#0f172a,stroke:#38bdf8,stroke-width:2px,color:#f8fafc;
    classDef bus fill:#854d0e,stroke:#facc15,stroke-width:2px,color:#f8fafc;
    classDef host fill:#1e1b4b,stroke:#6366f1,stroke-width:2px,color:#f8fafc;

    subgraph Sensors ["Sensors"]
        CAM["<b>Intel RealSense D435i</b><br/>• RGB Camera (1920x1080)<br/>• Infrared Stereo Depth (848x480)<br/>• Internal 6-DoF IMU"]:::sens
        IMU["<b>BNO055 / MPU-6050 IMU</b><br/>• 3-Axis Gyroscope (&omega;)<br/>• 3-Axis Accelerometer (a)<br/>• Onboard DMP Fusion"]:::sens
        ENCS["<b>4x Wheel Encoders</b><br/>• Dual-channel Quadrature (A/B)<br/>• ~1,000 CPR Resolution"]:::sens
    end

    subgraph Buses ["Communication Protocols"]
        USB3["<b>USB 3.0 Gen 1 (5 Gbps)</b><br/>Bulk high-throughput image streams"]:::bus
        I2C["<b>I2C / UART Bus (400 kHz / 115.2 kbps)</b><br/>Orientation, gravity & raw angular rates"]:::bus
        GPIO["<b>Hardware Timer GPIO Interrupts</b><br/>Direct hardware pulse counting"]:::bus
    end

    subgraph Hosts ["Processing Hosts"]
        JETSON["<b>NVIDIA Jetson Orin Nano</b>"]:::host
        STM32["<b>STM32 Blackpill ECU</b>"]:::host
    end

    CAM --> USB3 --> JETSON
    IMU --> I2C --> JETSON
    ENCS --> GPIO --> STM32
"""
    },
    {
        "folder": "4-Hardware",
        "filename": "05_rep105_coordinate_tf_tree.png",
        "title": "Rover Coordinate Transform Tree (REP-105)",
        "badge": "System TF Tree",
        "subtitle": "Strict separation between continuous 100Hz local odometry (odom) and discrete global SLAM drift offsets (map)",
        "mermaid": """flowchart TD
    classDef globalFrame fill:#581c87,stroke:#c084fc,stroke-width:2px,color:#f8fafc;
    classDef localFrame fill:#1e1b4b,stroke:#818cf8,stroke-width:2px,color:#f8fafc;
    classDef bodyFrame fill:#064e3b,stroke:#34d399,stroke-width:2px,color:#f8fafc;
    classDef sensorFrame fill:#0f172a,stroke:#38bdf8,stroke-width:2px,color:#f8fafc;

    MAP["<b>map (Global Frame)</b><br/>Origin of the world / Mars Yard.<br/>Published by RTAB-Map SLAM (1-5 Hz).<br/>Absorbs global drift & loop closures."]:::globalFrame

    ODOM["<b>odom (Local Odometry Frame)</b><br/>Continuous, smooth, zero-jump coordinate frame.<br/>Published by robot_localization EKF (100 Hz).<br/>Used by MPPI Local Controller."]:::localFrame

    BASE["<b>base_link (Rover Kinematic Center)</b><br/>Physical center of rover on the ground plane.<br/>Tracks vehicle heading and velocity."]:::bodyFrame

    CHASSIS["chassis<br/>Main structural frame"]:::bodyFrame
    IMU["imu_link<br/>BNO055 IMU mount"]:::sensorFrame
    CAM["camera_link<br/>RealSense D435 mounting frame"]:::sensorFrame
    OPTICAL["camera_depth_optical_frame<br/>ROS optical coordinates (Z forward, X right, Y down)"]:::sensorFrame

    MAP -->|"<b>RTAB-Map SLAM (1-5 Hz)</b><br/>Global Drift Offset"| ODOM
    ODOM -->|"<b>robot_localization EKF (100 Hz)</b><br/>Continuous High-Rate Pose"| BASE
    BASE -->|"Static TF"| CHASSIS
    BASE -->|"Static TF"| IMU
    BASE -->|"Static TF"| CAM
    CAM -->|"Static Optical Rotation"| OPTICAL
"""
    }
]

def main():
    base_docs = os.path.abspath("e:/SHAKR/Autonmous-27/General_Docs")
    
    print(f"Generating {len(ALL_DIAGRAMS)} architecture diagrams across General_Docs folders...")
    start_time = time.time()
    
    with sync_playwright() as p:
        browser = p.chromium.launch()
        page = browser.new_page(device_scale_factor=2)
        
        for idx, diag in enumerate(ALL_DIAGRAMS, 1):
            folder_path = os.path.join(base_docs, diag["folder"])
            os.makedirs(folder_path, exist_ok=True)
            
            out_path = os.path.join(folder_path, diag["filename"]).replace(os.sep, "/")
            temp_html = out_path.replace(".png", "_temp.html")
            
            html = HTML_TEMPLATE.format(
                title=diag["title"],
                badge=diag["badge"],
                subtitle=diag["subtitle"],
                mermaid_code=diag["mermaid"]
            )
            
            with open(temp_html, "w", encoding="utf-8") as f:
                f.write(html)
            
            try:
                page.goto(f"file:///{temp_html}")
                page.wait_for_selector(".mermaid svg", timeout=12000)
                body = page.locator("body")
                body.screenshot(path=out_path)
                print(f"[{idx:02d}/{len(ALL_DIAGRAMS):02d}] Generated: {diag['folder']}/{diag['filename']}")
            except Exception as e:
                print(f"[{idx:02d}/{len(ALL_DIAGRAMS):02d}] ERROR on {diag['filename']}: {e}")
            finally:
                if os.path.exists(temp_html):
                    os.remove(temp_html)
                    
        browser.close()
        
    elapsed = time.time() - start_time
    print(f"\nAll {len(ALL_DIAGRAMS)} diagrams generated successfully in {elapsed:.1f}s.")

if __name__ == "__main__":
    main()
