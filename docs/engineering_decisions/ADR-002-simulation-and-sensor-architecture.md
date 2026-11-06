# ADR-002: Native ArduPilot SITL + C++ `AP_Proximity_SITL` (`PRX1_TYPE = 10`) vs. Gazebo Harmonic on Apple M3 (8 GB RAM)

## Status
Accepted (2026-11-05)

## Context
The primary development machine is an Apple M3 Mac running macOS 26.2 with 8 GB unified memory and ~25 GB free disk space. We evaluated two simulation architectures for demonstrating physics-based flight and sensor-driven obstacle avoidance:
1. Full 3D Gazebo Harmonic (`gz-sim8`) + `ardupilot_gazebo` plugin.
2. Native ArduCopter SITL (`arducopter` V4.7.0-dev) built-in 6-DoF physics + ArduPilot's native C++ proximity raycasting driver (`AP_Proximity_SITL`, `PRX1_TYPE = 10`).

## Investigation Findings
- Inspecting `/opt/homebrew/Library/Taps/osrf/homebrew-simulation/Formula/gz-harmonic.rb` revealed that `gz-harmonic` is marked `deprecate! because: "freeimage is disabled"` and pulls in 25+ heavy C++/GUI dependencies (`gz-sim8`, `gz-rendering8`, `gz-gui8`, `ogre2.3`, `qt@5`, `bullet`, `dartsim`, `opencv`, `ffmpeg`, `protobuf`, `python@3.15`), consuming several gigabytes of disk and `3.5–5.0 GB` of unified RAM during 3D rendering.
- Earlier iterations of the repository injected synthetic `OBSTACLE_DISTANCE` MAVLink packets from Python (`PRX1_TYPE = 2`), which meant Python was telling the autopilot where the obstacle was rather than the simulator's sensor discovering an obstacle in the simulated world.
- Inspecting ArduPilot's C++ source code (`libraries/AP_Proximity/AP_Proximity_SITL.cpp`) showed that `arducopter` V4.7.0-dev includes a native C++ 8-sector proximity raycaster (`PRX1_TYPE = 10`) that casts rays in C++ against world obstacles loaded into `AC_PolyFence_loader` (`MAV_MISSION_TYPE_FENCE = 1`) while `FENCE_ENABLE = 0` keeps geofence failsafe actions off.

## Decision
Use native ArduCopter SITL (`arducopter` V4.7.0-dev, `< 45 MB` RSS on Apple M3) configured with `PRX1_TYPE = 10` (`AP_Proximity_SITL`), `PRX1_MAX = 50.0 m`, `OA_TYPE = 1` (`AP_OABendyRuler`), and world obstacle cylinders uploaded via `MAV_MISSION_TYPE_FENCE = 1` (`MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION = 5004`).

## Consequences
- **Positive:** Obstacles exist in the simulator's C++ world (`AC_PolyFence_loader`); raycasting, `AP_Proximity_Boundary_3D` filtering, `AP_OADatabase` population, and `AP_OABendyRuler` steering all happen inside the C++ autopilot firmware, emitting genuine `DISTANCE_SENSOR` and `ADSB_VEHICLE` (`OA_DB`) telemetry to Python.
- **Technical Detail:** Because `AP_Proximity_SITL::get_distance_to_fence` uses binary search on `[0, 200 m]` (`first probe = 100 m`), world obstacle cylinders are configured with a chord `> 100 m` along the approach ray (`radius_m = 80 m`, laterally offset `60 m` so they protrude `20 m` into the flight path).

