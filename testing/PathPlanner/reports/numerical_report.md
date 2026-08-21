# 📈 ROS 2 Path Planning Benchmark Report
This report summarizes the performance evaluation of the global path planner. The target pass condition is a **Path Planning Score** >= threshold defined in config.

## 📊 Performance Summary Table
| Scenario ID | Outcome | Score | Success | Planning Time | Blocked Cells | Avg Cost | Replanning |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| `empty_straight` | **✅ PASS** | `100.0/100` | YES | `0.014 s` | `0` | `0.0` | `100.0` |
| `scattered_rocks_detour` | **❌ FAIL** | `64.2/100` | YES | `0.171 s` | `23` | `0.0` | `0.0` |
| `canyon_gate_passage` | **✅ PASS** | `100.0/100` | YES | `0.017 s` | `0` | `0.0` | `100.0` |
| `marsyard_rough_slopes` | **✅ PASS** | `94.1/100` | YES | `0.516 s` | `0` | `0.0` | `100.0` |
| `marsyard_labyrinth` | **❌ FAIL** | `55.0/100` | YES | `0.476 s` | `241` | `0.0` | `0.0` |
| `canyon_gate_blocked` | **✅ PASS** | `90.0/100` | YES | `0.017 s` | `0` | `0.0` | `0.0` |
| `marsyard_snake_passage` | **❌ FAIL** | `63.4/100` | YES | `0.259 s` | `32` | `0.0` | `0.0` |
| `dead_end_trap` | **❌ FAIL** | `64.1/100` | YES | `0.006 s` | `6` | `5.9` | `0.0` |
| `crater_field` | **✅ PASS** | `97.8/100` | YES | `0.278 s` | `0` | `0.0` | `100.0` |

---

## 🔍 Detailed Scenario Analyses & Plots

### 📍 Scenario: `empty_straight` (✅ PASS)
- **Final Score**: `100.00 / 100`
- **Planning Time**: `0.014 s`
- **Path Length / Ratio**: `8.49 m` (Ratio: `1.00`)
- **Footprint Collisions (Blocked Cells)**: `0` cells
- **Average Traversed Cost**: `0.0`
- **Safety Margin**: `1.78 m`
- **Replanning Status**: Score `100.0/100` in `0.000 s`

#### Path Visualizer:
![empty_straight Plot](file:///home/saif/Desktop/ROAR/gpp benchmarking/reports/result_scenario_empty_straight.png)


---

### 📍 Scenario: `scattered_rocks_detour` (❌ FAIL)
- **Final Score**: `64.19 / 100`
- **Planning Time**: `0.171 s`
- **Path Length / Ratio**: `11.64 m` (Ratio: `1.03`)
- **Footprint Collisions (Blocked Cells)**: `23` cells
- **Average Traversed Cost**: `0.0`
- **Safety Margin**: `0.04 m`
- **Replanning Status**: Score `0.0/100` in `0.163 s`

#### Path Visualizer:
![scattered_rocks_detour Plot](file:///home/saif/Desktop/ROAR/gpp benchmarking/reports/result_scenario_scattered_rocks_detour.png)

> [!WARNING]
> **Collision Warning**: The path center line or its robot footprint (0.3m) intersected wall obstacles. Implement obstacle inflation to fix this.

---

### 📍 Scenario: `canyon_gate_passage` (✅ PASS)
- **Final Score**: `100.00 / 100`
- **Planning Time**: `0.017 s`
- **Path Length / Ratio**: `8.00 m` (Ratio: `1.00`)
- **Footprint Collisions (Blocked Cells)**: `0` cells
- **Average Traversed Cost**: `0.0`
- **Safety Margin**: `0.78 m`
- **Replanning Status**: Score `100.0/100` in `0.000 s`

#### Path Visualizer:
![canyon_gate_passage Plot](file:///home/saif/Desktop/ROAR/gpp benchmarking/reports/result_scenario_canyon_gate_passage.png)


---

### 📍 Scenario: `marsyard_rough_slopes` (✅ PASS)
- **Final Score**: `94.08 / 100`
- **Planning Time**: `0.516 s`
- **Path Length / Ratio**: `13.66 m` (Ratio: `1.21`)
- **Footprint Collisions (Blocked Cells)**: `0` cells
- **Average Traversed Cost**: `0.0`
- **Safety Margin**: `0.73 m`
- **Replanning Status**: Score `100.0/100` in `0.000 s`

#### Path Visualizer:
![marsyard_rough_slopes Plot](file:///home/saif/Desktop/ROAR/gpp benchmarking/reports/result_scenario_marsyard_rough_slopes.png)


---

### 📍 Scenario: `marsyard_labyrinth` (❌ FAIL)
- **Final Score**: `55.00 / 100`
- **Planning Time**: `0.476 s`
- **Path Length / Ratio**: `20.16 m` (Ratio: `1.78`)
- **Footprint Collisions (Blocked Cells)**: `241` cells
- **Average Traversed Cost**: `0.0`
- **Safety Margin**: `0.04 m`
- **Replanning Status**: Score `0.0/100` in `0.000 s`

#### Path Visualizer:
![marsyard_labyrinth Plot](file:///home/saif/Desktop/ROAR/gpp benchmarking/reports/result_scenario_marsyard_labyrinth.png)

> [!WARNING]
> **Collision Warning**: The path center line or its robot footprint (0.3m) intersected wall obstacles. Implement obstacle inflation to fix this.

---

### 📍 Scenario: `canyon_gate_blocked` (✅ PASS)
- **Final Score**: `90.00 / 100`
- **Planning Time**: `0.017 s`
- **Path Length / Ratio**: `8.00 m` (Ratio: `1.00`)
- **Footprint Collisions (Blocked Cells)**: `0` cells
- **Average Traversed Cost**: `0.0`
- **Safety Margin**: `0.78 m`
- **Replanning Status**: Score `0.0/100` in `0.008 s`

#### Path Visualizer:
![canyon_gate_blocked Plot](file:///home/saif/Desktop/ROAR/gpp benchmarking/reports/result_scenario_canyon_gate_blocked.png)


---

### 📍 Scenario: `marsyard_snake_passage` (❌ FAIL)
- **Final Score**: `63.37 / 100`
- **Planning Time**: `0.259 s`
- **Path Length / Ratio**: `11.96 m` (Ratio: `1.06`)
- **Footprint Collisions (Blocked Cells)**: `32` cells
- **Average Traversed Cost**: `0.0`
- **Safety Margin**: `0.04 m`
- **Replanning Status**: Score `0.0/100` in `0.000 s`

#### Path Visualizer:
![marsyard_snake_passage Plot](file:///home/saif/Desktop/ROAR/gpp benchmarking/reports/result_scenario_marsyard_snake_passage.png)

> [!WARNING]
> **Collision Warning**: The path center line or its robot footprint (0.3m) intersected wall obstacles. Implement obstacle inflation to fix this.

---

### 📍 Scenario: `dead_end_trap` (❌ FAIL)
- **Final Score**: `64.12 / 100`
- **Planning Time**: `0.006 s`
- **Path Length / Ratio**: `5.00 m` (Ratio: `1.00`)
- **Footprint Collisions (Blocked Cells)**: `6` cells
- **Average Traversed Cost**: `5.9`
- **Safety Margin**: `0.00 m`
- **Replanning Status**: Score `0.0/100` in `0.000 s`

#### Path Visualizer:
![dead_end_trap Plot](file:///home/saif/Desktop/ROAR/gpp benchmarking/reports/result_scenario_dead_end_trap.png)

> [!WARNING]
> **Collision Warning**: The path center line or its robot footprint (0.3m) intersected wall obstacles. Implement obstacle inflation to fix this.

---

### 📍 Scenario: `crater_field` (✅ PASS)
- **Final Score**: `97.78 / 100`
- **Planning Time**: `0.278 s`
- **Path Length / Ratio**: `12.19 m` (Ratio: `1.08`)
- **Footprint Collisions (Blocked Cells)**: `0` cells
- **Average Traversed Cost**: `0.0`
- **Safety Margin**: `0.78 m`
- **Replanning Status**: Score `100.0/100` in `0.000 s`

#### Path Visualizer:
![crater_field Plot](file:///home/saif/Desktop/ROAR/gpp benchmarking/reports/result_scenario_crater_field.png)


---

