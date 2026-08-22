# ROS 2 Global Path Planning Benchmarking Tool

This repository provides a standalone, black-box testing and benchmarking harness for ROS 2 global path planning modules, designed for the **ERC (European Rover Challenge) Autonomous Competition** and structured similarly to the **BARN Challenge**.

---

## Features

1. **Black-Box Testing:** Evaluates any path planner by interacting through standard ROS 2 interfaces (specifically the Nav2 `ComputePathToPose` action interface) without needing to modify the planner's internal source code.
2. **Automatic Mock Maps:** Generates synthetic costmaps (PNG) and elevation maps (NumPy `.npy`) automatically on first run to allow instant testing and verification.
3. **Dynamic Re-planning Evaluation:** Simulates costmap updates mid-planning by introducing dynamic obstacles and scoring the planner's detour efficiency and latency.
4. **Visual Comparison & Reports:**
   * **Pre-run Validation:** Generates PNGs showing only the map and reference path (with rover footprint safety boundaries) so you can visually check that your ground-truth is correct.
   * **Post-run Metrics:** Generates comparison PNGs showing the **Reference Path (Green)** and **Planned Path (Red)** with safety circles, as well as a JSON report containing all raw metrics.
5. **Stale Log & Report Purging:** Automatically deletes all stale report files (`results.json`, `numerical_report.md`, `numerical_report.pdf`, and `result_scenario_*.png` plots) from previous executions before a new run starts.
6. **Single-Scenario Filtering:** When running a single targeted scenario (using `scenario_id:=<id>`), the generated PDF and Markdown reports dynamically filter and adjust to display metrics for *only* that active test run.
7. **Passing Threshold Validation**: Evaluates the planner against a passing threshold of **$\ge$ 85.0 / 100** points.
8. **Lingering Process Cleanup:** Automatically terminates stale ROS 2, Nav2, or Gazebo processes from previous runs before starting a new test.
9. **Standard ROS 2 Launch Integration:** Fully configurable using a Python launch file supporting scenario selection and coordinate overrides.

---

## Package Directory Structure

```text
Global_path_benchmarking/
├── config/
│   └── scenarios.yaml          # Test scenarios definition (Start/Goal/Reference Paths)
├── global_path_benchmarking/
│   ├── __init__.py
│   ├── benchmarker.py          # Core benchmarking node, publishers, and report engine
│   └── mock_planner.py         # Mock planner server (Straight-Line & A*) for verification
├── launch/
│   └── benchmark.launch.py     # ROS 2 Launch file for scenario selection and overrides
├── maps/                       # PNG and .npy map storage (auto-generated if missing)
├── reports/                    # Holds output JSON data and comparison PNGs
├── package.xml                 # ROS 2 package configuration
├── setup.cfg
├── setup.py                    # ROS 2 build entrypoints
└── README.md                   # This instruction file
```

---

## Test Scenarios

The suite includes **9 pre-configured test scenarios** spanning simple baselines and complex navigation challenges:

1. **`empty_straight`**: Straight-line path in open field (tests basic connection).
2. **`scattered_rocks_detour`**: Multiple static rocks. Spawns a dynamic obstacle mid-run to test detour re-planning.
3. **`canyon_gate_passage`**: Navigates through a narrow gateway corridor.
4. **`marsyard_rough_slopes` [HARD]**: Injects high-cost gray zones representing slopes. Planners must detour around them to maintain low path cost.
5. **`marsyard_labyrinth` [HARD]**: A winding maze of walls. Tests footprint safety limits (planners without obstacle inflation will collide in corridors).
6. **`canyon_gate_blocked` [HARD]**: Spawns a large dynamic obstacle that completely closes off the passage, testing failure handling and abort latency.
7. **`marsyard_snake_passage` [HARD]**: A tight S-curve winding channel. Tests path smoothing and fine resolution collision checking.
8. **`dead_end_trap` [HARD]**: A U-shaped pocket blocking the goal. Tests heuristic trap escape (planners must route backward to find the exit).
9. **`crater_field` [HARD]**: Dense field of scattered variable-slope cost zones (gray values 140–180), evaluating cost-aware trajectory weaving.

---

## Getting Started

### 1. Build the Package
Ensure you have ROS 2 Humble sourced, then build the package inside your ROS 2 workspace:

```bash
# Navigate to your workspace directory (e.g. /home/saif/Desktop/testing)
cd /home/saif/Desktop/testing

# Build the package
colcon build --packages-select global_path_benchmarking

# Source the workspace
source install/setup.bash
```

---

## How to Run

### Step 1: Pre-Run Reference Path Verification (Recommended)
Before running the planner, check if your reference paths are safe and correct. Run the benchmarker in verification mode:

```bash
ros2 launch global_path_benchmarking benchmark.launch.py verify:=true
```
* **Output:** This generates PNGs in the `reports/` directory named `verify_scenario_<scenario_id>.png`. Open them to visually inspect the map and the green reference line.

### Step 2: Run the Benchmark

#### Running with the Safe A* Mock Planner (Expect High Scores)
By default, the launch file starts our mock planner in **A* mode**, which safely detours around obstacles:

```bash
ros2 launch global_path_benchmarking benchmark.launch.py
```

#### Running with the Straight-Line Mock Planner (Expect Collisions / Failures)
To test how the benchmarker handles a failing planner (e.g. hitting walls or failing safety margins), toggle `use_astar:=false`:

```bash
ros2 launch global_path_benchmarking benchmark.launch.py use_astar:=false
```

#### Running a Specific Scenario
You can choose a single scenario to run (e.g. `crater_field`) and automatically purge stale files using the `scenario_id` argument:

```bash
ros2 launch global_path_benchmarking benchmark.launch.py scenario_id:=crater_field clean:=true
```

---

## Swapping in Your Own Global Planner

To test your team's actual global path planner:
1. Make sure your global planner is running and hosts the Nav2 `compute_path_to_pose` Action Server.
2. Run the benchmarker node on its own, pointing it to your config:
   ```bash
   ros2 run global_path_benchmarking benchmarker --config path/to/scenarios.yaml
   ```
3. The benchmarker will send planning requests to your planner, record the output paths, calculate performance metrics, and save them.

---

## How to Add or Edit Scenarios

You can edit `config/scenarios.yaml` to define your own Marsyard testing maps. 

```yaml
scenarios:
  - id: "marsyard_rocky_center"
    map_image: "maps/your_marsyard_map.png" # Path to your map PNG
    resolution: 0.05                       # Resolution in meters/pixel
    origin: [-10.0, -10.0]                 # Coordinates of bottom-left corner
    robot_radius: 0.35                     # Rover radius footprint in meters
    
    start: [-2.0, 3.0]                     # Start coordinates (meters)
    goal: [8.0, 12.0]                      # Goal coordinates (meters)
    
    # Ordered coordinates representing the perfect reference route
    reference_path:
      - [-2.0, 3.0]
      - [0.0, 5.0]
      - [3.0, 8.0]
      - [8.0, 12.0]
      
    # Optional dynamic obstacle to trigger replanning mid-test
    dynamic_obstacles:
      - trigger_time: 1.0                  # Trigger time in seconds
      - x: 3.0
        y: 8.0
        radius: 0.5                        # Size of obstacle in meters
```

---

## Benchmarking Metrics and Scoring

The Path Planning Score is calculated out of 100 based on the following weights:

$$\text{Planning Score} = 0.25 \times S_{success} + 0.15 \times S_{time} + 0.25 \times S_{obstacle} + 0.15 \times S_{cost} + 0.10 \times S_{length} + 0.10 \times S_{replan}$$

| Sub-score | Weight | Raw Target | Calculation Formula |
| :--- | :--- | :--- | :--- |
| **Planning Success ($S_{success}$)** | 25% | Path found | `100` if path returned, `0` if failed/timeout. |
| **Planning Time ($S_{time}$)** | 15% | $\le 2.0\text{ seconds}$ | `100` if $T \le 2\text{s}$, `0` if $T \ge 10\text{s}$, else linear interpolation: $100 \times (1 - \frac{T-2}{8})$. |
| **Obstacle Avoidance ($S_{obstacle}$)** | 25% | $0$ collisions | `100` if path footprint never touches an obstacle (cost $\ge 100$), else `0`. |
| **Path Cost ($S_{cost}$)** | 15% | As low as possible | $100 - C_{avg}$, where $C_{avg}$ is the mean cost of cells traversed. |
| **Path Length Ratio ($S_{length}$)**| 10% | $\le 1.35$ | `100` if ratio $R \le 1.0$, `0` if $R \ge 1.35$, else: $100 \times (1 - \frac{R-1.0}{0.35})$. |
| **Replanning ($S_{replan}$)** | 10% | $\le 2.0\text{ seconds}$ detour | `100` if planner successfully detours dynamic obstacles under 2 seconds, else `0`. |
