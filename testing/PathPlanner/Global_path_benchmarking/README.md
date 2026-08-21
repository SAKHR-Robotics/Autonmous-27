# ROS 2 Path Planner & Controller Benchmarking Suite

This repository provides a black-box testing and benchmarking harness specifically designed for evaluating your ROS 2 path planning and control module (`erc_path_planner` with **Smac Hybrid A\*** and **MPPI Controller**). 

---

## 🌐 ROS 2 Version Compatibility (Jazzy & Humble)

This testing suite is **100% dual-compatible with both ROS 2 Jazzy and ROS 2 Humble**.
- The core Python testing architecture (`rclpy`) uses standard, version-agnostic interfaces for `nav_msgs`, `geometry_msgs`, and topic lifecycle management.
- It runs seamlessly on ROS 2 Jazzy and ROS 2 Humble without requiring any code edits or conversions.

---

## 🛠️ How It Works Under the Hood

The suite performs automated black-box evaluation of your path planner (`erc_path_planner`) across 9 pre-configured test scenarios:

1. **Scenario Orchestration (`testing_node`):**
   - Automatically launches your ROS 2 path planner bringup (`ros2 launch erc_path_planner path_planning.launch.py`).
   - Loads the scenario costmap PNG and converts it to a standard `nav_msgs/OccupancyGrid` on `/global_costmap/costmap` (and `/map`).
   - Publishes the target destination on `/goal_pose` (`geometry_msgs/PoseStamped`).
   - Dynamically injects mid-run dynamic obstacles to test real-time MPPI/Smac replanning.

2. **Metrics Evaluation (`benchmarking_node`):**
   - Subscribes to the generated global path (`/plan`) and motor velocity commands (`/cmd_vel`).
   - Calculates path length, planning latency, footprint collision count against obstacle cells, safety distance margins, turning angles, and curvature radii.
   - Computes a weighted **Path Planning Score** out of 100 points (Passing threshold: $\ge 85.0$).

3. **Report Generation (`report_generator.py`):**
   - Generates visual path plots (PNG), numerical summaries (JSON & CSV), an interactive HTML report (`reports/numerical_report.html`), and a printable PDF report (`reports/numerical_report.pdf`).

---

## 🏗️ Building the Tester

Per workspace rules, always run `colcon build` from the project root directory (`/home/saif/Desktop/MESEKET/Autonmous-27`):

```bash
# 1. Navigate to the project root directory
cd /home/saif/Desktop/MESEKET/Autonmous-27

# 2. Build the benchmark package along with your path planner
colcon build --packages-select global_path_benchmarking erc_path_planner terrain_geometry_msgs

# 3. Source the workspace
source install/setup.bash
```

---

## 🚀 How to Run the Benchmark

### 1. Pre-Run Scenario Verification (Visual Check)
Generate reference path PNGs to verify start/goal placements before executing tests:
```bash
ros2 launch global_path_benchmarking benchmark.launch.py verify:=true
```
*Outputs verification images to `reports/verify_scenario_<id>.png`.*

### 2. Run All 9 Benchmark Scenarios
To run the full suite against `erc_path_planner`:
```bash
ros2 launch global_path_benchmarking benchmark.launch.py clean:=true
```

### 3. Run a Specific Scenario
To test a single targeted scenario (e.g. `scattered_rocks_detour` or `marsyard_labyrinth`):
```bash
ros2 launch global_path_benchmarking benchmark.launch.py scenario_id:=scattered_rocks_detour clean:=true
```

---

## 🗺️ How to Add Custom Maps & Test Scenarios

You can easily add custom Marsyard maps or challenge arenas to the benchmark suite:

### Step 1: Create a Map PNG Image
Add a PNG image inside `maps/` (recommended size: `200x200` pixels):
- **White pixels (`255`):** Free navigable space.
- **Black pixels (`0`):** Hard lethal obstacles (walls, big rocks).
- **Gray pixels (`1`–`254`):** Rough terrain / slopes (higher cost).

### Step 2: Register Scenario in `config/scenarios.yaml`
Open `config/scenarios.yaml` and add a new scenario block:

```yaml
scenarios:
  - id: "custom_marsyard_arena"
    map_image: "maps/custom_marsyard_arena.png" # Path to your map image
    resolution: 0.05                             # 0.05 meters per pixel
    origin: [-5.0, -5.0]                         # Bottom-left map origin (X, Y)
    robot_radius: 0.35                           # Rover footprint radius in meters
    
    start: [-4.0, -4.0]                          # Start coordinates (X, Y in meters)
    goal: [4.0, 4.0]                             # Goal coordinates (X, Y in meters)
    
    # Ordered reference route for path length comparison
    reference_path:
      - [-4.0, -4.0]
      - [0.0, 0.0]
      - [4.0, 4.0]
      
    # (Optional) Dynamic obstacle injected mid-test
    dynamic_obstacles:
      - trigger_time: 0.5                        # Seconds after start
        x: 0.0
        y: 0.0
        radius: 0.4                              # Obstacle radius in meters
```

### Step 3: Verify and Benchmark
1. Run pre-run verification to check your reference path plot:
   ```bash
   ros2 launch global_path_benchmarking benchmark.launch.py verify:=true
   ```
2. Benchmark your path planner on the new scenario:
   ```bash
   ros2 launch global_path_benchmarking benchmark.launch.py scenario_id:=custom_marsyard_arena clean:=true
   ```

---

## 📊 Benchmark Evaluation Metrics

| Metric | Sub-score Weight | Target Condition |
| :--- | :--- | :--- |
| **Planning Success ($S_{success}$)** | 25% | Path successfully found to `/goal_pose` |
| **Planning Time ($S_{time}$)** | 15% | Latency $\le 2.0\text{s}$ |
| **Obstacle Avoidance ($S_{obstacle}$)** | 25% | Zero footprint collisions with lethal cells |
| **Path Cost ($S_{cost}$)** | 15% | Minimal traversal of high-slope/rough terrain |
| **Path Length Ratio ($S_{length}$)** | 10% | Length ratio $\le 1.35$ vs perfect reference |
| **Dynamic Replanning ($S_{replan}$)** | 10% | Successful detour around dynamic obstacles $\le 2.0\text{s}$ |
