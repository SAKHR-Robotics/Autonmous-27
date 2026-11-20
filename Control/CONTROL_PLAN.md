# Control implementation plan

Implement GUI simulation first, keep the current skid-steer navigation usable, and then add the hardware and payload interfaces in phases. This plan covers four drive motors now and four additional steering motors later.

## Architecture and ownership

```text
LAPTOP                                    JETSON
GUI navigation goals ──────────────────→ Nav2 planner + MPPI
GUI manual velocity ──────────────┐           │ autonomous velocity
                                 └────→ mode/safety selector
                                             ↓
                                       base controller
                                             ↓
GUI drill/arm requests ────────────→ subsystem controllers
                                             ↓
                                    micro-ROS Agent + CAN
                                             ↕
STM32s: local motor loops, encoder/load-cell acquisition, watchdogs, faults
                                             ↓
                                    Jetson feedback adapters
                                             ↓
                              wheel odometry → slip handling → EKF
                                             ↓
                                 navigation feedback + GUI telemetry
```

The GUI is an operator client, the Jetson owns navigation and command selection, and the STM32s own local actuator loops. Keep camera streams, point clouds, maps, and global paths on the compute/network side. Only the commands and sensor feedback needed by embedded boards should cross CAN.

## Phase 1 — use the existing GUI to drive and inspect Gazebo

### Scope

Use `MESEKET/GUI/control_station.py --gazebo-teleop` with the standalone Gazebo launch, without Nav2, SLAM, perception, STM32s, or physical motors. Show real simulation readings rather than relying on local animation as evidence of movement.

### Implementation

- [x] Add a Control entry point launching Gazebo, a manual timeout guard, and the existing GUI.
- [x] Add a session-only GUI profile for `/joint_states_gz`, `/imu/data`, `/odom`, RGB, and depth; use simulation time for the GUI ROS node.
- [x] Route manual commands through `/cmd_vel_manual` and the guard to `/cmd_vel`; retain explicit arm/disarm and latched stop controls.
- [x] Keep wheel angles fixed in the skid-steer visualization and disable unsupported navigation/payload inputs in this session.
- [ ] Complete the end-to-end operator acceptance checks below in a matching Gazebo environment.

The guard and scripts are source tools, not the production motor interface. The ROV/Arduino application remains separate.

### Acceptance checks

1. Launch from the documented command, connect the GUI, and confirm all four wheel RPM readings, IMU values, and changing odometry originate from Gazebo.
2. Drive forward/reverse and turn both ways; compare visible rover/wheel movement in Gazebo with GUI RPM and pose. Confirm wheel angles stay fixed.
3. Connect RGB and depth feeds and confirm new frames arrive while moving. Closing/detaching the camera must not change control ownership.
4. Check release-to-stop, disarm, latched stop/reset, window focus loss, and loss of GUI commands. A frozen UI must not leave an indefinitely refreshed nonzero command.
5. Close the GUI and confirm the combined launch shuts down its own simulator/guard processes. Record actual results without treating preview animation as measured feedback.

### Phase 1 validation record

The selected runtime is the project's Jazzy/Harmonic Docker image, extended with the GUI dependencies. The host's Humble/Fortress installation is not used to run this combined session. No simulator-version port is included.

| Check | Result |
| --- | --- |
| Python/shell syntax and launch argument loading | Passed |
| Existing GUI regression checks | Passed: kinematics, safety, parsers, offscreen Qt/map/camera/motor-station/CSV/lifecycle |
| Standalone GUI profile and ROS command/feedback checks | Passed with synthetic ROS messages: four RPM readings, IMU, pose, RGB/depth delivery, GUI-to-guard routing, clamp, stale-UI/input stopping, invalid/lateral input rejection, latched stop |
| Real Gazebo wheels, body motion, RGB/depth | Passed in Jazzy/Harmonic Docker: about 0.506 m body displacement, four wheel readings near 31.83 RPM, fresh IMU/pose, 640×480 RGB/depth, and stale-command stop |
| CAN, STM32s, physical motors | Outside Phase 1; not implemented or tested |

The integration check exposed and fixed an existing GUI failure: actual ROS IMU covariance arrays cannot be evaluated as booleans. The parser now checks their length explicitly, so IMU messages no longer terminate the telemetry worker for that reason.

Re-run the component integration from the project root in a sourced ROS shell, using the GUI's compatible Python. Use an unused ROS domain so test commands cannot reach another rover:

```bash
ROS_DOMAIN_ID=173 QT_QPA_PLATFORM=offscreen \
  ../GUI/.venv/bin/python Autonmous_Ws/Control/tests/check_gui_teleop.py
```

This exercises real ROS publishers/subscribers and GUI callbacks with synthetic sensor values. It does not establish Gazebo movement, rendered camera correctness, or hardware operation. The launcher preflight was also checked: this host's missing `gz sim` produces a clear error before starting the session.

The separate live Docker check used Gazebo's actual sensor topics and the GUI's command worker through the guard; it did not publish synthetic sensor readings. A further Qt keyboard check focused the drive window, held `W`, observed about `0.338 m` displacement and `19.89 RPM` on all four wheels, and verified stopping on key release. Initial keyboard injection without drive focus correctly produced no movement. Remaining operator checks include physical gamepad use, focus transitions, and the complete start/close workflow. Start the Docker session with `bash Autonmous_Ws/Control/scripts/run_gui_docker.sh` from the project root.

## Phase 2 — integrate manual and autonomous command ownership

The existing Nav2 flow is global planner → `/plan` → MPPI → `/cmd_vel_nav` → velocity smoother → `/cmd_vel`. The simulator consumes body velocity; a global path is not a motor command.

- [ ] Add a Jetson supervisor with explicit DISARMED, MANUAL, AUTONOMOUS, and FAULT states, source freshness checks, operator enable, and a published authoritative status.
- [ ] Give GUI and autonomous velocities separate inputs. Route both through one agreed selection/limiting chain, with exactly one final actuator-command publisher. Remap the current Nav2 final output when adding that selector.
- [ ] On manual takeover, cancel/suspend navigation and stop before handing over. Define whether a lost operator link cancels autonomy or permits an already approved mission to continue.
- [ ] Replace GUI `/goal_pose` publishing with a Nav2 action client supporting acceptance, progress, cancellation, and completion; base waypoint progression on action results.
- [ ] Add EKF/SLAM/perception progressively and show their actual data. Remove standalone static/Gazebo transforms when EKF and SLAM take ownership of `odom -> base_link` and `map -> odom`.

Completion: GUI manual and navigation operation both work, mode switches are visible and deterministic, and simultaneous publishers cannot fight over actuator output.

## Phase 3 — Jetson to STM32 drive control over CAN/micro-ROS

### Inputs needed

Confirm board models/count, motor-driver channels, encoder resolutions and gear ratios, measured wheel geometry, CAN variant/bitrate, transceivers/termination, and the supported micro-ROS firmware/Agent versions. Map each board and wheel by a stable identity.

### New software

- [ ] Add a ROS 2 interfaces package for compact, bounded command/feedback messages and shared units. Prefer fixed wheel channels over unbounded payloads for embedded use.
- [ ] Add a base-controller package in Control: selected body velocity → four wheel-speed targets using skid-steer kinematics initially.
- [ ] Add the Jetson micro-ROS Agent/transport launch and STM32 firmware integration. Validate the actual CAN client/Agent transport and multi-board routing before connecting all motors.
- [ ] Add feedback adapters publishing the project's selected encoder/joint-state input, board health, actual motor speeds, currents and faults; avoid duplicate odometry input sources.
- [ ] Add local firmware speed loops, command expiry, boot-disarmed state, disconnect/reconnect handling, and fault latching/recovery. Reconnection must not resume an old motion target.

### Proposed contracts — finalize before coding firmware

| Interface | Direction | Required content |
| --- | --- | --- |
| `/base/wheel_command` | Jetson → motor boards | Sequence, validity/expiry, enable state, named/fixed wheel targets in rad/s |
| `/base/wheel_feedback` | Motor boards → Jetson | Board/wheel identity, measurement timestamp, encoder counts, measured rad/s, current, faults |
| `/base/board_status` | Boards → Jetson | Heartbeat, calibration/ready state, watchdog and driver faults |
| `/control/state` | Jetson → GUI | Selected mode, enable state, active source, fault/stop reason |
| Existing odometry topics | Jetson → Nav2/GUI | `/wheel/odom_raw`, `/wheel/odom_filtered`, `/odometry/filtered` from actual measurements |

Topic names are proposals. Fix ordering, integer widths, timestamp semantics, transport addressing, QoS, rates, and timeout behavior in a protocol document. Do not send every ROS topic across CAN. Select rates from motor dynamics and a bus-load budget that includes serialization, fragmentation, acknowledgments and reconnect traffic. Fast motor loops stay local on STM32s.

If CAN FD is available, evaluate the supported micro-ROS CAN FD integration for the chosen boards. If Classical CAN is required, assess the exact transport and fragmentation/addressing work; do not assume CAN FD examples transfer unchanged. Transport validation is a prerequisite, not a claim of current support.

Completion: one wheel's command/feedback loop first, then all four; verify speed signs/scaling, timeouts, fault stops and measured odometry before enabling autonomous hardware movement.

## Phase 4 — drill, load cells, and later arm

- [ ] Give drill control its own enable/disable and bounded speed/direction interface, measured state/current/fault feedback, and clear conditions for driving while drilling.
- [ ] Publish calibrated, timestamped load-cell readings with explicit units; expose tare/calibration as acknowledged operations and show missing/stale data in the GUI.
- [ ] Add GUI panels backed by those interfaces. Display requested, accepted, and measured states separately; a button click does not prove an actuator moved.
- [ ] Add the arm later through a Jetson trajectory/action interface and joint feedback, with local STM32 position/velocity loops and joint limits.
- [ ] Define shared power, emergency-stop, tool/arm motion and drive interlocks. Record each command's safe timeout/disconnect behavior per subsystem.

Completion: each subsystem can be commanded and observed independently through the GUI, with acknowledged faults and no accidental coupling to wheel control.

## Phase 5 — four steering motors and full-system operation

Follow [steering addition.md](../../steering%20addition.md) after the current drivetrain is complete. Add four vertical steering joints, measured angle feedback, coordinated drive/steering control, steering-aware odometry and slip detection, and compatible navigation constraints.

Keep the GUI manual-command contract stable; extend the Jetson base controller and embedded command/feedback types with calibrated steering angles/rates. Reassess spin, reverse and crab capabilities from the real mechanics. Repeat simulation and hardware checks before enabling each additional mode.

## First task when resuming

Complete Phase 1 in the chosen simulator environment and record wheel movement, RPM/pose feedback, RGB/depth and stop behavior. Resolve the STM32 models and CAN variant before starting Phase 3. Do not implement steering or arm control as a prerequisite for current GUI teleop.
