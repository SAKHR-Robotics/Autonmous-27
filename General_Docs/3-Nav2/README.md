# 🗺️ Nav2 Navigation & Path Planning Diagrams (`General_Docs/3-Nav2/`)

This directory contains the visual pipeline graphs and granular block deep-dive diagrams for the **Nav2 Navigation & Path Planning Subsystem** (Smac Hybrid A* planner, MPPI controller, costmap bridges, and Behavior Trees).

---

## 📸 Diagram Gallery

### 1. Master Pipeline
![Nav2 Master Pipeline](00_nav2_master_pipeline.png)

---

### 2. Costmap Bridge Ingestion Node (3D BBox to PointCloud2)
![Costmap Bridge](01_costmap_bridge_ingestion_block.png)

---

### 3. Global vs Local Rolling Window Costmap Servers
![Costmap Servers](02_global_and_local_costmap_servers.png)

---

### 4. Smac Hybrid A* Global Planner (Reeds-Shepp Search)
![Smac Global Planner](03_smac_hybrid_a_star_planner_block.png)

---

### 5. MPPI Controller (2,000 Rollouts @ 20Hz & Critics)
![MPPI Controller](04_mppi_controller_rollouts_block.png)

---

### 6. Behavior Tree Replanning & Recovery Escalation
![Behavior Tree Navigator](05_behavior_tree_replanning_block.png)

---

### 7. Velocity Smoother & Skid-Steer Motor Driver Bridge
![Velocity Smoother](06_velocity_smoother_and_skid_steer_bridge.png)
