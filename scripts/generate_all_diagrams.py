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
            min-width: 900px;
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
            padding: 4px 10px;
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
        /* Tooltip and label styling overrides */
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

DIAGRAMS = [
    # -------------------------------------------------------------
    # 1. SLAM Full Pipeline
    # -------------------------------------------------------------
    {
        "filename": "slam_pipeline.png",
        "title": "SLAM & State Estimation Architecture Pipeline",
        "badge": "SLAM Subsystem",
        "subtitle": "Multi-tier state estimation: wheel tick differential kinematics, slip rejection, 100Hz EKF fusion, and RTAB-Map visual loop closure",
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
    NODE_ODOM -->|"<b>/wheel/odom_raw</b><br/>(nav_msgs/Odometry)"| NODE_SLIP
    NODE_ODOM -->|"<b>/wheel/single_wheel_slip</b><br/>(std_msgs/Bool)"| NODE_SLIP

    IN_IMU --> NODE_SLIP
    NODE_SLIP -->|"<b>/wheel/odom_filtered</b><br/>(cov inflated)"| NODE_EKF
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

    # -------------------------------------------------------------
    # 2. Perception Full Pipeline
    # -------------------------------------------------------------
    {
        "filename": "perception_pipeline.png",
        "title": "Perception Subsystem Architecture Pipeline",
        "badge": "Perception Subsystem",
        "subtitle": "Dual-stream perception: 3D point cloud terrain geometry segmentation and high-precision ArUco solvePnP landmark tracking",
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
        T_ROI["<b>1. Forward ROI Spatial Crop</b><br/>Filters points outside rover traverse envelope"]:::pcl
        T_GND["<b>2. Patchwork++ Ground Separation</b><br/>Concentric zone elevation fitting removes surface ground"]:::pcl
        T_VOX["<b>3. Voxel Grid Downsampling</b><br/>5cm leaf size balances fidelity and real-time compute"]:::pcl
        T_ROR["<b>4. Radius Outlier Removal</b><br/>Eliminates floating dust and sunlight noise points"]:::pcl
        T_CLUST["<b>5. DBSCAN Euclidean Clustering</b><br/>Extracts discrete rock centroids & 3D bounding boxes"]:::pcl
        T_MEM["<b>6. Persistent Memory & EMA Tracker</b><br/>Tracks obstacles across frames & retains blind spots"]:::pcl
    end

    subgraph ARUCO ["🎯 ARUCO LANDMARK POSE ESTIMATION (marker_detection)"]
        A_DETECT["<b>1. Marker Detector & Corner Refinement</b><br/>Sub-pixel corner detection from 1000h dictionary"]:::aruco
        A_PNP["<b>2. OpenCV solvePnP 6-DoF Solver</b><br/>Projects 2D corners into 3D camera coordinates"]:::aruco
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

    # -------------------------------------------------------------
    # 3. Path Planning Full Pipeline
    # -------------------------------------------------------------
    {
        "filename": "path_planning_pipeline.png",
        "title": "Path Planning & Nav2 Navigation Architecture Pipeline",
        "badge": "Navigation Subsystem",
        "subtitle": "Nav2 integration: Smac Hybrid A* Reeds-Shepp global planner, MPPI 2,000 rollout controller, and costmap obstacle bridge",
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

    # -------------------------------------------------------------
    # 4. Hardware & Control Architecture
    # -------------------------------------------------------------
    {
        "filename": "hardware_control_pipeline.png",
        "title": "Hardware Electrical & Computing Architecture",
        "badge": "Hardware & Electronics",
        "subtitle": "High-current power distribution, Jetson Orin Nano master compute, STM32 Blackpill ECU, sensors, and 4WD skid-steer actuation",
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

    # -------------------------------------------------------------
    # 5. Block Deep Dive: SLAM Slip & EKF
    # -------------------------------------------------------------
    {
        "filename": "slam_slip_and_ekf_block.png",
        "title": "Deep Dive: Wheel Odometry Kinematics & 100Hz EKF Fusion",
        "badge": "SLAM Block Deep-Dive",
        "subtitle": "Isolation of single-wheel sand slip, dynamic covariance scaling (10^3), and high-rate local pose estimation",
        "mermaid": """flowchart LR
    classDef hw fill:#0f172a,stroke:#64748b,stroke-width:2px,color:#f8fafc;
    classDef node fill:#1e1b4b,stroke:#818cf8,stroke-width:2px,color:#f8fafc;
    classDef ekf fill:#581c87,stroke:#c084fc,stroke-width:2px,color:#f8fafc;
    classDef out fill:#064e3b,stroke:#34d399,stroke-width:2px,color:#f8fafc;

    subgraph Inputs ["Hardware Inputs"]
        TICKS["<b>/wheel/ticks</b><br/>4-wheel encoder counts"]:::hw
        IMU["<b>/imu/data</b><br/>BNO055 Angular Velocity & Yaw"]:::hw
    end

    subgraph Block1 ["Block 1: Kinematics"]
        ENC_NODE["<b>encoder_ticks_to_odom.py</b><br/>• Converts tick deltas to twist (vx, wz)<br/>• Computes per-wheel speeds<br/>• Flags single-wheel slip per side"]:::node
    end

    subgraph Block2 ["Block 2: Slip Checker"]
        SLIP_NODE["<b>heuristic_slip_checker.py</b><br/>• Compares wheel yaw vs IMU gyro<br/>• Detects sand spin & rover stall<br/>• If slipping: clamps vx = 0.0 & scales cov to 1000"]:::node
    end

    subgraph Block4 ["Block 4: EKF Local Estimator"]
        EKF_NODE["<b>robot_localization ekf_node</b><br/>• Fuses wheel twist + IMU roll/pitch/yaw<br/>• Continuous 100Hz smooth odometry<br/>• Ignores wheel velocity when cov inflated"]:::ekf
    end

    subgraph Outputs ["Local Outputs"]
        ODOM_FILT["<b>/odometry/filtered</b><br/>(100Hz zero-jump pose)"]:::out
        TF_ODOM["<b>TF: odom ➔ base_link</b><br/>(Local tracking frame)"]:::out
    end

    TICKS --> ENC_NODE
    ENC_NODE -->|"<b>/wheel/odom_raw</b>"| SLIP_NODE
    ENC_NODE -->|"<b>/wheel/single_wheel_slip</b>"| SLIP_NODE
    IMU --> SLIP_NODE
    SLIP_NODE -->|"<b>/wheel/odom_filtered</b><br/>(Dynamic covariance)"| EKF_NODE
    IMU --> EKF_NODE
    EKF_NODE --> ODOM_FILT
    EKF_NODE --> TF_ODOM
"""
    },

    # -------------------------------------------------------------
    # 6. Block Deep Dive: SLAM RTAB-Map & Costmap
    # -------------------------------------------------------------
    {
        "filename": "slam_rtabmap_costmap_block.png",
        "title": "Deep Dive: RTAB-Map Visual SLAM & Nav2 Costmap Ingestion",
        "badge": "SLAM Block Deep-Dive",
        "subtitle": "Visual FAST/GFTT feature tracking, ArUco landmark 6-DoF constraint fusion, and layered occupancy grid generation",
        "mermaid": """flowchart LR
    classDef sens fill:#0f172a,stroke:#64748b,stroke-width:2px,color:#f8fafc;
    classDef filter fill:#1e1b4b,stroke:#818cf8,stroke-width:2px,color:#f8fafc;
    classDef slam fill:#581c87,stroke:#c084fc,stroke-width:2px,color:#f8fafc;
    classDef costmap fill:#7f1d1d,stroke:#ef4444,stroke-width:2px,color:#f8fafc;
    classDef out fill:#064e3b,stroke:#34d399,stroke-width:2px,color:#f8fafc;

    subgraph Inputs ["Sensor & Landmark Streams"]
        DEPTH_RAW["Raw Depth Stream<br/><b>/camera/depth/image_raw</b>"]:::sens
        RGB_RAW["Raw Color Stream<br/><b>/camera/image_raw</b>"]:::sens
        ARUCO_POSE["ArUco 6-DoF Pose<br/><b>/perception/aruco_pose</b>"]:::sens
        ROCKS["Terrain Obstacles<br/><b>/bridge/pointcloud</b>"]:::sens
    end

    subgraph VisionFilter ["Block 3: RealSense Filter"]
        V_FILTER["<b>vision_helper</b><br/>• Decimation (2x)<br/>• Spatial filter<br/>• Temporal persistence<br/>• Hole filling"]:::filter
    end

    subgraph GlobalSLAM ["Block 5: RTAB-Map SLAM"]
        RTAB["<b>rtabmap_slam Node</b><br/>• FAST/GFTT feature tracking<br/>• Torus graph optimization<br/>• ArUco landmark fusion<br/>• Computes map ➔ odom drift"]:::slam
    end

    subgraph NavCostmap ["Block 6: Nav2 Costmap 2D"]
        CM["<b>nav2_costmap_2d Server</b><br/>• Static Layer: /map<br/>• Obstacle Layer: PointCloud<br/>• Inflation Layer: Radius"]:::costmap
    end

    subgraph Outputs ["Planning Targets"]
        TF_MAP["<b>TF: map ➔ odom</b><br/>(1-5 Hz drift correction)"]:::out
        OUT_COSTMAP["<b>/global_costmap/costmap</b><br/>(Nav2 Planning Grid)"]:::out
    end

    DEPTH_RAW --> V_FILTER
    V_FILTER -->|"Filtered Depth"| RTAB
    RGB_RAW --> RTAB
    ARUCO_POSE -->|"6-DoF Landmark"| RTAB
    RTAB --> TF_MAP
    RTAB -->|"<b>/map</b> (OccupancyGrid)"| CM
    ROCKS --> CM
    CM --> OUT_COSTMAP
"""
    },

    # -------------------------------------------------------------
    # 7. Block Deep Dive: Perception Terrain Geometry
    # -------------------------------------------------------------
    {
        "filename": "perception_terrain_geometry_block.png",
        "title": "Deep Dive: 3D Point Cloud Terrain Geometry Pipeline",
        "badge": "Perception Block Deep-Dive",
        "subtitle": "8-stage real-time spatial filtering, Patchwork++ ground removal, DBSCAN clustering, and EMA obstacle tracking",
        "mermaid": """flowchart TD
    classDef stage fill:#064e3b,stroke:#059669,stroke-width:2px,color:#f8fafc;
    classDef inout fill:#0f172a,stroke:#38bdf8,stroke-width:2px,color:#f8fafc;

    IN_RAW["<b>Raw Depth Point Cloud</b><br/>/camera/depth/color/points (~300,000 points @ 30Hz)"]:::inout

    STAGE1["<b>Stage 1: ROI Spatial Crop</b><br/>Crops bounds: X [-2.5m, +2.5m], Y [-2.5m, +2.5m], Z [-1.0m, +1.0m]"]:::stage
    STAGE2["<b>Stage 2: Patchwork++ Ground Removal</b><br/>Concentric zone model separates ground terrain from protruding boulders"]:::stage
    STAGE3["<b>Stage 3: Voxel Grid Downsampling</b><br/>5cm voxel leaf size yields uniform density cloud (~8,000 points)"]:::stage
    STAGE4["<b>Stage 4: Radius Outlier Removal (ROR)</b><br/>Eliminates floating airborne dust and sunlight IR reflections"]:::stage
    STAGE5["<b>Stage 5: DBSCAN Euclidean Clustering</b><br/>Min points = 15, Epsilon = 0.12m &rarr; Extracts isolated rock clusters"]:::stage
    STAGE6["<b>Stage 6: Persistent Memory & EMA Tracking</b><br/>Matches cluster IDs across frames & maintains blind-spot retention"]:::stage

    OUT_OBST["<b>Smoothed Obstacles & Bounding Boxes</b><br/>/perception/obstacles_only &rarr; Sent to Nav2 Costmap Server"]:::inout

    IN_RAW --> STAGE1 --> STAGE2 --> STAGE3 --> STAGE4 --> STAGE5 --> STAGE6 --> OUT_OBST
"""
    },

    # -------------------------------------------------------------
    # 8. Block Deep Dive: Perception ArUco Detection
    # -------------------------------------------------------------
    {
        "filename": "perception_aruco_vision_block.png",
        "title": "Deep Dive: ArUco Landmark 6-DoF solvePnP Pipeline",
        "badge": "Perception Block Deep-Dive",
        "subtitle": "Sub-pixel corner detection, perspective-n-point 3D pose extraction, and covariance filtering for SLAM drift reset",
        "mermaid": """flowchart LR
    classDef input fill:#0f172a,stroke:#64748b,stroke-width:2px,color:#f8fafc;
    classDef proc fill:#1e1b4b,stroke:#818cf8,stroke-width:2px,color:#f8fafc;
    classDef out fill:#064e3b,stroke:#34d399,stroke-width:2px,color:#f8fafc;

    subgraph Inputs ["Camera Streams"]
        RGB["<b>RGB Image Frame</b><br/>/camera/image_raw (1280x720)"]:::input
        INFO["<b>Camera Intrinsics</b><br/>fx, fy, cx, cy & distortion"]:::input
    end

    subgraph Stages ["Processing Stages (marker_detection)"]
        S1["<b>1. Dictionary Match</b><br/>Adaptive threshold & border check<br/>(DICT_4X4_50 / DICT_5X5_100)"]:::proc
        S2["<b>2. Corner Refinement</b><br/>cv2.cornerSubPix interpolation<br/>Sub-pixel pixel precision"]:::proc
        S3["<b>3. OpenCV solvePnP</b><br/>Calculates 3D rotation (rvec)<br/>and translation (tvec) in camera frame"]:::proc
        S4["<b>4. Kalman Filter Gate</b><br/>Rejects reprojection error > 2px<br/>Smooths 6-DoF pose coordinates"]:::proc
    end

    subgraph Output ["SLAM Landmark"]
        ARUCO_OUT["<b>/perception/aruco_pose</b><br/>(geometry_msgs/PoseStamped)<br/>Global loop closure anchor"]:::out
    end

    RGB --> S1 --> S2 --> S3
    INFO --> S3
    S3 --> S4 --> ARUCO_OUT
"""
    },

    # -------------------------------------------------------------
    # 9. Block Deep Dive: Nav2 MPPI Dynamic Avoidance
    # -------------------------------------------------------------
    {
        "filename": "nav2_mppi_dynamic_avoidance_block.png",
        "title": "Deep Dive: Nav2 MPPI Controller & Behavior Tree Replanning",
        "badge": "Nav2 Block Deep-Dive",
        "subtitle": "2,000 parallel trajectory rollouts @ 20Hz, dynamic obstacle swerving, and autonomous recovery behavior escalation",
        "mermaid": """flowchart TD
    classDef bt fill:#1e1b4b,stroke:#818cf8,stroke-width:2px,color:#f8fafc;
    classDef mppi fill:#7f1d1d,stroke:#ef4444,stroke-width:2px,color:#f8fafc;
    classDef decision fill:#854d0e,stroke:#facc15,stroke-width:2px,color:#f8fafc;
    classDef act fill:#064e3b,stroke:#34d399,stroke-width:2px,color:#f8fafc;

    BT["<b>Behavior Tree Navigator</b><br/>Orchestrates global route replanning & recovery sequences"]:::bt
    SMAC["<b>Smac Hybrid A* Global Planner</b><br/>Computes macro route /plan on updated Global Costmap"]:::bt
    MPPI["<b>MPPI Local Controller (20Hz)</b><br/>Simulates 2,000 trajectory rollouts in parallel<br/>Critics: GoalCritic, PathAlignCritic, ObstacleCritic"]:::mppi

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

    # -------------------------------------------------------------
    # 10. Coordinate Systems & TF Tree (REP-105)
    # -------------------------------------------------------------
    {
        "filename": "tf_tree_architecture.png",
        "title": "Rover Coordinate Transform Tree (REP-105)",
        "badge": "System Coordinate Frames",
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
    docs_dir = os.path.abspath("e:/SHAKR/Autonmous-27/General_Docs")
    os.makedirs(docs_dir, exist_ok=True)
    
    print(f"Generating {len(DIAGRAMS)} architecture diagrams into: {docs_dir}")
    start_time = time.time()
    
    with sync_playwright() as p:
        browser = p.chromium.launch()
        page = browser.new_page(device_scale_factor=2)
        
        for idx, diag in enumerate(DIAGRAMS, 1):
            out_path = os.path.join(docs_dir, diag["filename"]).replace(os.sep, "/")
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
                # Take screenshot of the body
                body = page.locator("body")
                body.screenshot(path=out_path)
                print(f"[{idx}/{len(DIAGRAMS)}] Generated: {diag['filename']}")
            except Exception as e:
                print(f"[{idx}/{len(DIAGRAMS)}] ERROR on {diag['filename']}: {e}")
            finally:
                if os.path.exists(temp_html):
                    os.remove(temp_html)
                    
        browser.close()
        
    elapsed = time.time() - start_time
    print(f"All diagrams generated successfully in {elapsed:.1f}s.")

if __name__ == "__main__":
    main()
