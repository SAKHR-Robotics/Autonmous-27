# Jazzy perception: implementation and remaining limits

Base: `jazzy-port` at d05282b. Existing marker refinements, calibration-age
checks, duplicate-ID handling and Detection3DArray interfaces are preserved.
The older main-based patch was not cherry-picked.

## Corrections

| Area | Result |
| --- | --- |
| Failure semantics | Invalid/stale input, unavailable TF and segmentation failure report unavailable perception; optional costmap is unknown, not clear |
| Coverage | Default grid covers forward ROI; unknown-space mode frees only observed ground cells |
| TF | Timestamp lookup by default; rigid-mount caching is explicit; clock reversal clears temporal state |
| Marker map | Compose map<-camera with observations; no dependency on marker broadcaster arrival; reset deletes RViz markers |
| Memory | Optional bounded odom-frame association/expiry; exclusive obstacle publisher; optional measured-ground clearing |
| Surface risk | Optional observed-patch slope/residual roughness/step checks, requiring measured limits and gravity reference |
| Landmarks | Native RTAB-Map messages, age/confidence/quality gates, positive IDs and covariance handling |
| Consumers | Preserve box type; validate/cap bridge sampling, publish empty updates; SLAM consumes the bridge cloud; real depth is a separate clearing source |
| Test isolation | Runtime mocks and artificial localization launches are under testing/ |
| Linux startup | LF attributes prevent executable shebangs from becoming python3 plus carriage return |

## Limits requiring validation

1. **Diagnostics are not a motor interlock.** Navigation/control must enforce
   observation freshness and a safe stopping policy. An old apparently clear
   map is insufficient evidence to drive; this patch does not implement the
   final controller's interlock.
2. **Risk checks are geometric and conditional.** They require sufficient
   valid samples and a calibrated gravity-aligned frame. They are not a
   validated crater detector or semantic traversability classifier. No return
   means unknown, not a confirmed pit.
3. **Memory depends on localization.** Odometry error moves retained geometry.
   TTL removes unseen observations; marking-only costmaps may retain marks
   until real rays clear them. Validate association, TTL, ground clearing,
   camera origin and Nav2 clearing together before use.
4. **External TF can still be late or absent.** The marker-broadcaster race is
   removed, but unavailable localization observations are rejected rather
   than fabricated. Memory also skips observations without valid TF.
5. **Calibration and competition sign-off remain external.** Camera mount,
   intrinsics, marker sizes, rover clearance/tilt limits, outdoor conditions
   and target-computer timing need measurements. Confirm the final ERC 2027
   rulebook; the official competitor zone linked 2026 materials at review time.

Bridge sampling is bounded, initially 200,000 points. Invalid/oversized geometry
is rejected and capped geometry is not complete scene reconstruction. Never
raytrace fabricated bounding-box points as measured free space. Adapt the real
clearing-cloud topic to the active camera; this requires its correct TF origin.

## Verification evidence

An isolated Linux Jazzy Docker environment built perception, the C++ planner
bridge and testing packages with artifacts under /tmp and ROS_DOMAIN_ID=87.
The original 109 marker tests passed. Additional tests exercised failure/clear
semantics, observed ground, dynamic TF, risk geometry, memory, map composition,
landmarks, launch selection and configuration scoping.

A real headless test ran synthetic sensors through the production terrain and
marker nodes and observed rock boxes, occupied costmap cells, depth-fused
marker poses and healthy diagnostics. A live bridge regression checked empty,
invalid and bounded geometry. Exact final counts are recorded in the completion
report; these are not hardware or ERC readiness claims.

Patchwork++ is absent in that image; live tests used the logged NumPy fallback.
GUI rendering, outdoor/physical sensors, RTAB-Map optimization, Nav2 lifecycle
and clearing performance, and a complete rover autonomy run remain unverified.
Broader Antigravity validation is a separate requested handoff.
