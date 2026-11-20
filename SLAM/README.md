# SLAM — ROS 2 Jazzy / Ubuntu 24.04

Start with [the running guide](rover_slam/README.md). It covers standalone
Gazebo, hardware, each processing stage, testing helpers and whole-system use.
Implementation status and remaining checks: [SLAMImprovments.md](SLAMImprovments.md).
Current launch inventory, staged review and Docker evidence:
[VALIDATION.md](VALIDATION.md).

## What runs here

```text
one wheel source -> optional encoder converter -> slip covariance filter
                                                   |              IMU
                                                   v               |
                                              local EKF <----------+
                                                   |
                                  /odometry/filtered + odom->base_link
                                                   |
RGB + aligned depth + calibration ----------------> RTAB-Map
Perception native landmark observations ---------->    |
                                                  /map + map->odom
```

The EKF is local planar state estimation; RTAB-Map supplies global corrections.
It does not feed a second velocity observation back into this EKF. Learned
marker landmarks are not automatically surveyed global-position fixes.

## Production versus testing

| Location | Responsibility |
| --- | --- |
| `rover_slam/rover_slam/` | Encoder processing, pure wheel math, slip assessment |
| `rover_slam/launch/` | Independent stages, production master, optional sensor driver/TF/RViz |
| `rover_slam/config/` | Geometry profiles, EKF/RTAB-Map settings, optional costmap, RViz views |
| `../testing/mocks/slam/slam_mock_tools/` | Landmark and obstacle smoke fixtures |
| `../testing/benchmarking/slam/slam_test_tools/` | EKF monitor, wrappers, compatibility adapters and regressions |

Production starts no synthetic inputs or artificial map/odom localization.
The retained `ekf_validator.launch.py` is an explicit compatibility entry point
into the testing package; the master never starts it.

## Quick start with an already running Gazebo rover

The reviewed Rover description leaves `base_link` as the root for the EKF and
places `base_footprint` below it. Start the selected `final_world_R&A.world`
with all three optional Rover TF publishers disabled; see
[VALIDATION.md](VALIDATION.md) for evidence and remaining accuracy gates.

From the workspace root in a sourced terminal:

```bash
ros2 launch rover_slam slam_bringup.launch.py use_sim_time:=true \
  odom_topic:=/odom launch_encoder:=false launch_rviz:=true
```

This needs real simulated RGB-D, CameraInfo, IMU, `/odom`, `/clock` and sensor
TF. It does not start Gazebo or generate those inputs. The full guide below
explains the terminal sequence and expected outputs.

## Changes and limits

Wheel input is selected explicitly; rates use measurement intervals. Missing
input stops wheel output rather than becoming a newly stamped zero velocity.
Slip changes covariance while preserving measured twist. Mapping consumes the
EKF message and its covariance. Databases survive startup by default.

Physical geometry, calibration, terrain slip thresholds and supported camera
profiles require measurement. Planar TF does not establish physical roll/pitch;
keep Perception's gravity-dependent terrain risk disabled until that is solved.
Diagnostics are not a motor interlock. See [the staged review](VALIDATION.md)
for Docker/Gazebo evidence and remaining accuracy/hardware checks.
