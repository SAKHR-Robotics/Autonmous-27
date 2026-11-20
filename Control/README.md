# Rover control — GUI first, then hardware

Start with the existing Mars Rover GUI driving Gazebo and displaying measured feedback. Add real Jetson → CAN → STM32 control only after that simulation loop works.

The detailed architecture, interfaces, and implementation checklist are in [CONTROL_PLAN.md](CONTROL_PLAN.md). Four-wheel steering remains a later upgrade described in [steering addition.md](../../steering%20addition.md).

## Phases and current status

| Phase | Result | Status |
| --- | --- | --- |
| 1 — GUI + Gazebo teleop | Driving, camera feeds, wheel RPM, IMU, pose, and stop/timeout handling | Integrated; live Docker movement/feedback/cameras/timeout verified; operator checklist remains |
| 2 — Shared command ownership | Jetson selects manual or autonomous control; Nav2 action feedback/cancel in GUI | Planned |
| 3 — Jetson + STM32 motor loop | Wheel-speed commands and measured feedback through CAN/micro-ROS | Planned; board models and CAN variant required |
| 4 — Payload controls | Drill, load cells, auxiliary outputs, and later arm interface | Planned |
| 5 — Steering + integrated autonomy | Four steering motors, matching odometry/navigation, full rover operation | Deferred until the current skid-steer system is finished |

## Structure

```text
Control/
├── README.md
├── CONTROL_PLAN.md
├── launch/
│   └── gui_teleop.launch.py     # Gazebo + command guard + existing GUI
├── scripts/
│   ├── run_gui_teleop.sh        # Environment checks and launch entry point
│   ├── run_gui_docker.sh        # Recommended host entry point: Jazzy/Harmonic Docker
│   └── teleop_guard.py          # Manual-only /cmd_vel_manual → /cmd_vel watchdog
├── tests/
│   └── check_gui_teleop.py       # ROS + GUI component check; synthetic sensors
└── docs/
    └── control_guide.html       # Earlier skid-steer reference
```

The GUI stays in the sibling `MESEKET/GUI/` directory. It is not copied into this workspace. These Control scripts run from source; no new ROS package has been introduced yet. The motor-controller package and embedded firmware will be added in Phase 3.

## Phase 1: launch the existing GUI with Gazebo

### Recommended: run both applications in Docker

From the `Autonmous-27` project root, in your desktop terminal:

```bash
bash Autonmous_Ws/Control/scripts/run_gui_docker.sh
```

This uses the existing `autonomous27:jazzy` image and builds a separate `autonomous27:gui` image if needed. It opens Gazebo and the rover GUI in the same Jazzy/Harmonic environment. The first GUI image build downloads Qt dependencies; later starts reuse it. The compose configuration mounts both source trees read-only and builds only the robot-description/world packages inside the container, leaving host build outputs alone.

The session uses ROS domain `174` by default, isolated from the normal domain. Set `ROVER_ROS_DOMAIN_ID` explicitly if you need another domain. The launcher mounts your desktop Xauthority file rather than changing X-server access permissions. Close the GUI or press Ctrl+C in the launcher terminal to stop the session.

The reproducible container definitions are `Autonmous_Ws/docker/Dockerfile.gui` and `Autonmous_Ws/docker/compose.gui.yaml`. If the base image is missing, build it with the project's existing Dockerfile first.

### Alternative: run inside an already prepared environment

Use a shell in the project's **ROS 2 Jazzy / Gazebo Harmonic environment**, with the rover description already built. Source its workspace install. Do not start Nav2, another teleop, the old Tkinter teleop, or a second simulator in the same ROS domain for this standalone session.

From the `Autonmous-27` project root:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash Autonmous_Ws/Control/scripts/run_gui_teleop.sh
```

If the install is under `Autonmous_Ws/install/`, the script also recognizes that location. For another directory layout or GUI interpreter:

```bash
bash Autonmous_Ws/Control/scripts/run_gui_teleop.sh \
  gui_dir:=/absolute/path/to/GUI \
  gui_python:=/absolute/path/to/python \
  world:=world1.world
```

The default world is the local `empty_with_sensors.sdf` so initial checks do not depend on Mars Yard assets. The launch opens Gazebo and the actual existing GUI directly in a standalone rover profile. Closing the GUI shuts down this launch's processes.

The GUI Python must import `rclpy`, `PySide6`, and `numpy`, plus the application's existing dependencies. A host virtual environment is not portable into a container with a different Python version. If Gazebo runs inside Docker and the GUI stays on the laptop, use the split procedure below. The original `docker-compose.yml` mounts the autonomous workspace only; the new `compose.gui.yaml` also mounts the GUI and installs its dependencies.

### Operator steps

1. Press **CONNECT ROS 2** and confirm wheel RPM, IMU, and odometry begin arriving.
2. Select **ROS 2 Teleop (/cmd_vel_manual)** under OUTPUT, then press **ARM**.
3. Focus the drive window (outside editable settings), then hold `W/S` for forward/reverse, `Q/E` to turn, or `W` with `A/D` for curved travel. Observe the rover in Gazebo and measured wheel RPM in the GUI.
4. Under **Camera**, select **ROS 2 · RGB Camera** and Connect. Switch to **Depth Heatmap** to inspect depth. Camera streams remain opt-in.
5. Release controls and check stopping; press `Space`/`Esc` for the latched stop. Reset the latch and arm again before continuing.

The simulated wheels have fixed steering. This profile keeps their displayed steering angles at zero, disables steering-mode selection, and uses differential-drive body commands. Individual motor-test controls remain preview-only and do not independently actuate Gazebo joints.

### What is available now

| GUI data/control | Phase 1 topic | Meaning |
| --- | --- | --- |
| Manual command | `/cmd_vel_manual` → guard → `/cmd_vel` | Body forward speed (m/s) and yaw rate (rad/s) |
| Four drive RPM readings | `/joint_states_gz` | Measured simulation wheel angular velocity; GUI converts rad/s to RPM |
| Pose and raw odometry | `/odom` | Gazebo odometry; not an EKF estimate in this session |
| IMU | `/imu/data` | Simulated sensor output |
| RGB and depth | `/camera/image_raw`, `/camera/depth/image_raw` | Gazebo images bridged into ROS |

SLAM maps, Nav2 paths/goals, perception overlays, slip diagnostics, steering feedback, drill/load cells, and arm feedback are not started by Phase 1. Their GUI inputs are disabled/blank in this session rather than displaying fabricated values. Saved Simulation/Hardware topic profiles are not overwritten by the special session profile.

### Split operation: simulator environment and laptop GUI

Start the simulator in the supported environment, without the old teleop or Nav2:

```bash
ros2 launch my_robot_description gazebo.launch.py \
  world:=empty_with_sensors.sdf publish_map_tf:=true \
  publish_camera_tf:=true bridge_sim_tf:=true
```

In a second shell in that same ROS environment, run the guard from its mounted source path:

```bash
python3 /absolute/path/to/Control/scripts/teleop_guard.py
```

On the GUI machine, source its compatible ROS environment and run:

```bash
cd /absolute/path/to/GUI
.venv/bin/python control_station.py --gazebo-teleop
```

Match `ROS_DOMAIN_ID`, make DDS discovery/network access available, and confirm the chosen ROS/RMW versions actually interoperate before driving. Cross-distribution interoperability is not assumed. The combined launcher is preferred when both applications can use one supported environment.

### Command timeout and limitations

The GUI clears a stale UI setpoint after 0.3 seconds. The guard uses a wall-clock receipt timeout of 0.3 seconds, bounds speed to ±0.5 m/s and yaw rate to ±1.0 rad/s, and sends zero for stale, nonfinite, or unsupported motion inputs. Its publication interval is 0.05 seconds. This guard is exclusively for manual simulation and does not arbitrate autonomous commands.

Keep this launch isolated from hardware. It is not an STM32 watchdog or a physical emergency-stop circuit. Phase 2 adds authoritative command ownership; Phase 3 adds embedded timeout/fault handling.

## Phase 3 preview: commands to STM32s

The planned hardware flow is:

```text
GUI manual command ─┐
                    ├→ Jetson mode/safety selector → base controller
Nav2 velocity ──────┘       → wheel-speed targets → micro-ROS Agent/CAN → STM32 motor loops
STM32 encoders/speeds/faults → Jetson feedback adapter → odometry/EKF + GUI
```

The current skid-steer conversion is `v_left = v_x - yaw_rate * track_width / 2` and `v_right = v_x + yaw_rate * track_width / 2`. Convert each wheel's linear target to rad/s using its measured radius. STM32 firmware closes the motor-speed loop and publishes measurements; the global path stays on the Jetson.

Before implementing the transport, identify every STM32 model, Classical CAN versus CAN FD, transceivers, bitrate, board/node mapping, encoder resolution, and firmware/Agent versions. Define fixed command/feedback layouts, units, timestamps, expiry, status, and reconnect behavior. CAN transport for micro-ROS must be selected and validated for those actual components; it is not automatically enabled by naming a ROS topic.

Simulation currently uses wheel radius `0.06 m` and track width `0.49 m`. Earlier control documentation used `0.15 m` and `0.65 m`; those are unverified hardware values and must not silently replace the simulation dimensions.

## Validation status

The laptop host has Humble/Fortress; the selected Docker environment supplies Jazzy/Harmonic. Live Docker validation passed: GUI commands moved the rover about `0.506 m`, all four wheels reported about `31.83 RPM`, IMU and pose were received, RGB/depth arrived at `640×480`, and stale commands stopped the simulated body. A separate focused-window `W` key check moved it about `0.338 m`, showed about `19.89 RPM` on all wheels, and stopped on key release. Component/regression checks also passed. Physical gamepad, focus-loss, and remaining operator acceptance checks are listed in the plan; no STM32/CAN hardware was tested.

Component-check results and remaining acceptance checks are recorded in [CONTROL_PLAN.md](CONTROL_PLAN.md#phase-1-validation-record).
