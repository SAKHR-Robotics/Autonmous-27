# General Architecture & Engineering Documentation (`General_Docs/`)

This directory houses system-level diagrams, hardware component specifications, pipeline graphs, and architectural guides for the Autonomous-27 rover.

---

## 📌 Document Catalog

| Document / Asset | Description | Format / Tool |
| :--- | :--- | :--- |
| [`rover_architecture.html`](rover_architecture.html) | Interactive 3D/Visual Rover System Blueprint and sub-system walkthrough | HTML5 / Modern CSS |
| [`rover_detailed_pipeline.png`](rover_detailed_pipeline.png) | End-to-end node architecture and topic data flow diagram | Graphviz PNG / DOT |
| [`rover_blackbox_pipeline.png`](rover_blackbox_pipeline.png) | High-level module boundary and external sensor/actuator contract diagram | Graphviz PNG / DOT |
| [`FlowWTopics.png`](FlowWTopics.png) | Granular inter-node message routing diagram | Graphviz PNG / DOT |
| [`hardware_specs/`](hardware_specs/) | Electrical schematics, motor specifications, battery power distribution, and mechanical constraints | Markdown / Specs |
| [`humble_to_jazzy.md`](humble_to_jazzy.md) | ROS 2 Jazzy (Ubuntu 24.04) migration changes and API compatibility notes | Markdown |
| [`Docker_Beginners_Guide.pdf`](Docker_Beginners_Guide.pdf) | Containerized development environment tutorial | PDF |

---

## 🌐 Coordinate Systems & TF Tree (REP-105)

```
map (Global World / RTAB-Map SLAM)
 └── odom (Continuous Smooth Local Odometry / robot_localization EKF)
      └── base_link (Rover Kinematic Center)
           ├── chassis
           ├── camera_link
           │    ├── camera_depth_frame -> camera_depth_optical_frame
           │    └── my_robot/camera_link/camera (Gazebo sensor bridge)
           └── imu_link
```
