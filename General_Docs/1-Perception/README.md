# 👁️ Perception Subsystem Diagrams (`General_Docs/1-Perception/`)

This directory contains the visual pipeline graphs and granular block deep-dive diagrams for the **Perception Subsystem** (3D terrain point cloud geometry and ArUco marker vision).

---

## 📸 Diagram Gallery

### 1. Master Pipeline
![Perception Master Pipeline](00_perception_master_pipeline.png)

---

### 2. 3D Terrain Geometry 6-Stage Pipeline
![Terrain Geometry Pipeline](01_terrain_geometry_6stage_pipeline.png)

---

### 3. Spatial ROI Crop & Patchwork++ Ground Separation
![Patchwork Ground Removal](02_roi_and_patchwork_ground_removal.png)

---

### 4. Voxel Grid Downsampling & Radius Outlier Removal
![Voxel and Outlier Filter](03_voxel_downsampling_and_outlier_removal.png)

---

### 5. DBSCAN Euclidean Clustering & 3D Bounding Boxes
![DBSCAN Clustering](04_dbscan_clustering_and_bounding_boxes.png)

---

### 6. Persistent Memory Node & EMA Rock Tracking
![Persistent Memory EMA](05_persistent_memory_and_ema_tracker.png)

---

### 7. ArUco Corner Extraction & solvePnP 6-DoF Landmark Estimation
![ArUco Vision](06_aruco_detection_and_pnp_block.png)

---

### 8. Direct 2D Terrain Costmap Generation
![Terrain Costmap](07_terrain_costmap_generation.png)
