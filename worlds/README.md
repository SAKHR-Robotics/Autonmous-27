# Worlds Package (`worlds`)

A standalone ROS 2 Humble package that organizes, stores, and simulates Gazebo worlds for the Autonomous Rover environment (Mars Yard). It includes 3D terrain/rock/ArUco models, pre-computed metric elevation & cost map datasets, and offline toolchains for generating and visualizing terrain maps.

---

## 1. Directory Structure

```text
Autonmous_Ws/worlds/
├── CMakeLists.txt              # Colcon build configuration (installs worlds, launch, data, models, maps_tools)
├── package.xml                 # ROS 2 package manifest
├── README.md                   # Complete user guide and documentation
│
├── worlds/                     # Gazebo simulation world files (.world / SDF 1.7)
│   ├── final_world_R&A.world   # Primary world with Mars Yard terrain, 9 rock obstacles, and 15 ArUco markers
│   └── marsyard.world          # Base Mars Yard world
│
├── models/                     # 3D assets and model definitions
│   ├── mars_yard/              # Mars Yard terrain visual and collision meshes (.obj)
│   ├── rocks/                  # Individual rock obstacle models (rock_1 to rock_9)
│   └── aruco/                  # ArUco marker board models (aruco_1 to aruco_15)
│
├── launch/                     # ROS 2 launch scripts
│   ├── launch_map.launch.py    # Master parameterized launcher for any world with Gazebo bridge
│   ├── final_world_R_A.launch.py # Shortcut launch script for final_world_R&A.world
│   ├── marsyard.launch.py      # Shortcut launch script for base marsyard.world
│   ├── world_Rotated.launch.py # Shortcut for rotated world
│   └── world_Rotated_Aruco.launch.py
│
├── data/                       # Precomputed map datasets & visualizations
│   ├── heightmap.npz           # Metric elevation grid (Z-heights, coordinates, resolution)
│   ├── heightmap.png           # Grayscale heightmap preview
│   ├── heightmap_hillshade.png # Shaded relief elevation map
│   ├── heightmap_contour.png   # Topographic contour map
│   ├── heightmap_3d.png        # 3D perspective terrain mesh plot
│   ├── costmap.npz             # Directional and total terrain traversal costs
│   ├── costmap.png             # 3-panel cost preview (Total, X-slope, Y-slope)
│   ├── costmap_detailed.png    # Comprehensive multi-panel cost, slope, and roughness analysis
│   ├── csv/                    # CSV exports of cost matrices
│   │   ├── total_cost.csv      # Overall terrain cost matrix (0-100)
│   │   ├── cost_x.csv          # X-directional gradient cost
│   │   └── cost_y.csv          # Y-directional gradient cost
│   ├── obstacle_data.npy       # Binary registry of obstacle coordinates and yaw
│   ├── obstacle_data_info.txt  # Human-readable table of rocks and ArUco markers
│   └── metadata.txt            # Dataset metadata and bounds
│
└── maps_tools/                 # Map generation and visualization toolchain
    ├── heightmap/
    │   ├── heightmap_generator.py # Converts .world meshes into a 2D metric elevation grid (.npz/.png)
    │   └── visualize_heightmap.py # 2D/3D visualization & plotting tool using matplotlib
    └── costmap/
        └── costmap_generator.py   # Computes terrain gradient, Laplacian roughness, and cost grids (.npz/.csv)
```

---

## 2. How to Build the Package

Before running the launch files, compile the workspace with `colcon`:

```bash
# Navigate to the workspace root
cd ~/Desktop/MESEKET/Autonmous-27

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Build the worlds package
colcon build --packages-select worlds

# Source the workspace overlay
source install/setup.bash
```

---

## 3. How to Launch Simulation Worlds

### Option A: Launch `final_world_R&A.world` (Recommended)
Launch the primary simulation world with Mars Yard terrain, rock obstacles, and ArUco markers:
```bash
ros2 launch worlds final_world_R_A.launch.py
```

### Option B: Parameterized Launch (Any World)
Launch any custom or generated `.world` file in the `worlds/` directory:
```bash
ros2 launch worlds launch_map.launch.py world:=final_world_R&A.world
```
Or for base Mars Yard:
```bash
ros2 launch worlds launch_map.launch.py world:=marsyard.world
```

---

## 4. How to Use Map Tools (`maps_tools`)

The `maps_tools` directory contains standalone Python scripts for generating elevation grids and cost maps from Gazebo SDF worlds.

### Step 1: Generate Heightmap from `.world`
Run `heightmap_generator.py` on the target `.world` file:
```bash
python3 Autonmous_Ws/worlds/maps_tools/heightmap/heightmap_generator.py \
  Autonmous_Ws/worlds/worlds/final_world_R&A.world \
  -o Autonmous_Ws/worlds/data/heightmap.npz \
  --preview Autonmous_Ws/worlds/data/heightmap.png \
  --resolution 0.25
```
**Options:**
- `world`: Path to the `.world` file.
- `-o`, `--output`: Target `.npz` file path.
- `--preview`: Optional grayscale preview PNG path.
- `--resolution`: Grid cell size in metres (default: `0.25`).
- `--model-path`: Extra model search directories (auto-searches `models/`, `models/rocks/`, `models/aruco/`).

---

### Step 2: Generate Terrain Costmap from Heightmap
Run `costmap_generator.py` on the generated `heightmap.npz`:
```bash
python3 Autonmous_Ws/worlds/maps_tools/costmap/costmap_generator.py \
  Autonmous_Ws/worlds/data/heightmap.npz \
  -o Autonmous_Ws/worlds/data/costmap.npz \
  --preview Autonmous_Ws/worlds/data/costmap.png \
  --csv-dir Autonmous_Ws/worlds/data/csv
```
**Options:**
- `heightmap`: Input `heightmap.npz` path.
- `-o`, `--output`: Target `costmap.npz` path.
- `--preview`: 3-panel grayscale preview PNG (Total, X-slope, Y-slope).
- `--csv-dir`: Folder to output `cost_x.csv`, `cost_y.csv`, and `total_cost.csv`.
- `--gradient-scale`: Weight for slope steepness (default: `150.0`).
- `--stability-scale`: Weight for surface roughness/Laplacian (default: `90.0`).

---

## 5. How to Visualize Map Data

Use `visualize_heightmap.py` to inspect and plot the generated elevation data interactively or save figures.

### Interactive 3D Perspective View
Opens an interactive 3D rotatable mesh on your desktop:
```bash
python3 Autonmous_Ws/worlds/maps_tools/heightmap/visualize_heightmap.py \
  Autonmous_Ws/worlds/data/heightmap.npz \
  --type 3d \
  --cmap terrain
```

### Interactive 2D Shaded Relief Map (Hillshade)
Simulates a sun angle for clear visual depth of slopes and rock obstacles:
```bash
python3 Autonmous_Ws/worlds/maps_tools/heightmap/visualize_heightmap.py \
  Autonmous_Ws/worlds/data/heightmap.npz \
  --hillshade \
  --cmap terrain
```

### Interactive 2D Topographic Contours
Displays topographic elevation contour lines:
```bash
python3 Autonmous_Ws/worlds/maps_tools/heightmap/visualize_heightmap.py \
  Autonmous_Ws/worlds/data/heightmap.npz \
  --type contour \
  --levels 25 \
  --cmap terrain
```

### Saving Figures Directly to File
Add `-o <filename>.png` to any visualization command to export the figure directly:
```bash
# Save 3D surface plot
python3 Autonmous_Ws/worlds/maps_tools/heightmap/visualize_heightmap.py \
  Autonmous_Ws/worlds/data/heightmap.npz \
  --type 3d -o Autonmous_Ws/worlds/data/heightmap_3d.png

# Save 2D Hillshade plot
python3 Autonmous_Ws/worlds/maps_tools/heightmap/visualize_heightmap.py \
  Autonmous_Ws/worlds/data/heightmap.npz \
  --hillshade -o Autonmous_Ws/worlds/data/heightmap_hillshade.png

# Save Contour plot
python3 Autonmous_Ws/worlds/maps_tools/heightmap/visualize_heightmap.py \
  Autonmous_Ws/worlds/data/heightmap.npz \
  --type contour -o Autonmous_Ws/worlds/data/heightmap_contour.png
```
