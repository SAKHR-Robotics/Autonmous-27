# 📈 ROS 2 Path Planning Benchmark Report
This report summarizes the performance evaluation of the global path planner. The target pass condition is a **Path Planning Score** >= threshold defined in config.

## 📊 Performance Summary Table
| Scenario ID | Outcome | Score | Success | Planning Time | Blocked Cells | Avg Cost | Replanning |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| `marsyard_labyrinth` | **❌ FAIL** | `65.0/100` | YES | `0.026 s` | `15` | `0.0` | `0.0` |
| `crater_field` | **✅ PASS** | `97.3/100` | YES | `2.074 s` | `0` | `17.0` | `100.0` |

---

## 🔍 Detailed Scenario Analyses & Plots

### 📍 Scenario: `marsyard_labyrinth` (❌ FAIL)
- **Final Score**: `65.00 / 100`
- **Planning Time**: `0.026 s`
- **Path Length / Ratio**: `5.84 m` (Ratio: `0.52`)
- **Footprint Collisions (Blocked Cells)**: `15` cells
- **Average Traversed Cost**: `0.0`
- **Safety Margin**: `0.02 m`
- **Replanning Status**: Score `0.0/100` in `0.000 s`

#### Path Visualizer:
![marsyard_labyrinth Plot](file:///home/khallaf/Autonmous-27/testing/PathPlanner/src/global_path_benchmarking/reports/result_scenario_marsyard_labyrinth.png)

> [!WARNING]
> **Collision Warning**: The path center line or its robot footprint (0.3m) intersected wall obstacles. Implement obstacle inflation to fix this.

---

### 📍 Scenario: `crater_field` (✅ PASS)
- **Final Score**: `97.30 / 100`
- **Planning Time**: `2.074 s`
- **Path Length / Ratio**: `5.84 m` (Ratio: `0.52`)
- **Footprint Collisions (Blocked Cells)**: `0` cells
- **Average Traversed Cost**: `17.0`
- **Safety Margin**: `0.75 m`
- **Replanning Status**: Score `100.0/100` in `0.000 s`

#### Path Visualizer:
![crater_field Plot](file:///home/khallaf/Autonmous-27/testing/PathPlanner/src/global_path_benchmarking/reports/result_scenario_crater_field.png)


---

