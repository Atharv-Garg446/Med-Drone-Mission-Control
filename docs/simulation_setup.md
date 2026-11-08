# Simulation Setup, Hardware Feasibility & Sensor Architecture

## 1. Hardware & OS Feasibility Assessment (Apple M3 / 8 GB RAM / macOS 26.2)

### 1.1 Measured Development Machine Constraints
- **SoC / CPU:** Apple M3 (8 cores, `arm64`)
- **Unified Memory:** `8 GB` (`8,589,934,592` bytes)
- **Operating System:** macOS `26.2` (Darwin `25.2.0`)
- **Free Disk Space:** `~24–25 GB` available on `/System/Volumes/Data`

### 1.2 Gazebo Harmonic vs. Native ArduPilot SITL Assessment
We inspected the OSRF Homebrew tap (`/opt/homebrew/Library/Taps/osrf/homebrew-simulation/Formula/gz-harmonic.rb` and `gz-sim8.rb`) directly on this machine:
1. **Deprecation & Dependency Footprint:** `osrf/simulation/gz-harmonic` is marked `deprecate! because: "freeimage is disabled"` in the tap and pulls in more than 25 heavy C++/GUI dependencies (`gz-sim8`, `gz-rendering8`, `gz-gui8`, `ogre2.3`, `qt@5`, `bullet`, `dartsim`, `ffmpeg`, `opencv`, `protobuf`, `python@3.15`), requiring multiple gigabytes of disk space and `3.5–5.0 GB` of unified memory when running Ogre2 Metal/OpenGL rendering alongside macOS WindowServer and the IDE.
2. **Native ArduPilot SITL C++ Physics & Raycasting (`arducopter` V4.7.0-dev):**
   - Built natively as a Mach-O 64-bit `arm64` executable at `/Users/mahigarg/Documents/Drone Shit/ardupilot/build/sitl/bin/arducopter`.
   - Uses `< 45 MB` RSS memory and `< 15%` of a single M3 performance core at `10x` real-time speedup (`--speedup 10`), leaving `> 3 GB` of free unified memory headroom.
   - Crucially, ArduPilot's SITL binary already compiles in **two native C++ simulated proximity sensors**:
     - `AP_Proximity_SITL` (`PRX1_TYPE = 10` in `libraries/AP_Proximity/AP_Proximity_SITL.cpp`)
     - `SIM::measure_distance_at_angle_bf` (`libraries/SITL/SITL.cpp` + `SIM_PS_LightWare_SF45B.cpp` / `SIM_PS_LD06.cpp`)

**Engineering Decision:** Use native ArduCopter SITL (`arducopter` V4.7.0-dev) with its built-in C++ 6-DoF flight dynamics, coupled 6-motor hexacopter airframe & 6S LiPo battery model (`--model hexa:med_drone_hexa_16.json` + `FRAME_CLASS = 2`), EKF3 state estimator, `SIM_Gripper_Servo` actuator, and native C++ `AP_Proximity_SITL` (`PRX1_TYPE = 10`) raycasting sensor as the primary simulation environment on the 8 GB Apple M3.

---

## 1.3 Coupled 6-Motor Hexacopter Airframe & 6S LiPo Physics (`med_drone_hexa_16.json`)

To eliminate any mismatch between Python's `MedDrone-Hexa-16` (`flightstack/vehicle_model.py`), ArduCopter's motor mixer (`AP_MotorsMatrix`), and ArduPilot's C++ physics engine (`SIM_Frame.cpp` + `SIM_Battery.cpp`), all three layers are configured as a unified 6-motor hexacopter on a 6S 16 Ah LiPo pack:

1. **Physics Airframe (`flightstack/params/med_drone_hexa_16.json` via `SitlProcess` `--model hexa:med_drone_hexa_16.json`):**
   - `num_motors = 6`, `disc_area = 1.4255` m² (`6` × `18-inch` rotors), `mass = 11.2` kg (`5.5` kg empty + `3.2` kg battery + `2.5` kg cargo allowance; plus `1.0` kg `SIM_Gripper_Servo` external load = `12.2` kg takeoff gross mass, dropping `1.0` kg upon release).
   - 6S 16 Ah LiPo parameters: `maxVoltage = 25.2` V, `refVoltage = 22.2` V, `battCapacityAh = 16.0` Ah, `refBatRes = 0.012` Ω, `refCurrent = 31.0` A, `hoverThrOut = 0.42`, `propExpo = 0.65`.
2. **Autopilot Parameters (`flightstack/params/sitl_delivery.parm`):**
   - `FRAME_CLASS = 2` (`MAV_TYPE_HEXAROTOR = 13` verified in live `HEARTBEAT` telemetry as `mav_type = "HEXAROTOR"`), `FRAME_TYPE = 0`.
   - `SIM_BATT_VOLTAGE = 25.04` (`6 × 4.173 V` = `100%` SoC on ArduPilot's `soc_table`), `SIM_BATT_CAP_AH = 16`, `BATT_CAPACITY = 16000`, `MOT_BAT_VOLT_MAX = 25.2`, `MOT_BAT_VOLT_MIN = 19.8`.
   - `EK3_DRAG_BCOEF_X/Y = 55.6`, `EK3_DRAG_MCOEF = 0.177`.
3. **Critical Engineering Lesson (`sitl_run_20261107_213336.jsonl`):**
   - Setting `FRAME_CLASS = 2` (6 motors in `AP_MotorsMatrix`) while leaving `SitlProcess` on `--model +` (4 motors in `SIM_Frame.cpp`) caused motors 5 and 6 to be ignored by the physics simulator and prevented takeoff. Pairing `--model hexa:med_drone_hexa_16.json` with `FRAME_CLASS = 2` restores full 6-motor thrust symmetry and realistic 6S voltage sag (`25.039 V -> 21.925 V` over the 2-stop mission).

---

## 2. How Sensor-Driven Obstacle Avoidance Works in C++ (`PRX1_TYPE = 10`)

Instead of spoofing proximity packets from Python (`PRX1_TYPE = 2` `OBSTACLE_DISTANCE`), the live SITL configuration (`flightstack/params/sitl_delivery.parm`) runs ArduPilot's internal C++ proximity driver (`PRX1_TYPE = 10`):

1. **World Geometry Storage (`AC_PolyFence_loader`):**
   - `AutopilotLink.upload_fence_obstacles()` uploads world geometry to the autopilot via the MAVLink Fence Protocol (`mission_type = 1`, `MAV_MISSION_TYPE_FENCE`):
     - Four `MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION` (`5001`) vertices forming a `~22 km × 22 km` bounding box around home so `AP::fence()->polyfence().inclusion_boundary_available()` returns `true` and `PRX1` sensor status transitions to `Status::Good`.
     - Cylindrical world obstacles (`MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION = 5004`) placed at target coordinates `(lat, lon, radius_m)`.
   - `FENCE_ENABLE` is set to `0` so ArduCopter does not trigger a geofence RTL failsafe, while `AP_Proximity_SITL` continues to raycast against `AP::fence()->polyfence().breached(loc)`.

2. **C++ 8-Sector Bisection Raycaster (`AP_Proximity_SITL::get_distance_to_fence`):**
   - On every proximity update loop in C++, `AP_Proximity_SITL` casts 8 horizontal rays (`0°, 45°, 90°, 135°, 180°, 225°, 270°, 315°` in body frame, converted to Earth frame via `sitl->state.yawDeg`).
   - Along each ray, it performs a bisection search on `[0, 200 m]` (`0.1 m` accuracy) testing `AP::fence()->polyfence().breached(loc)`.
   - Note on C++ Bisection Geometry: Because `AP_Proximity_SITL::get_distance_to_fence` starts its binary search at `test_dist = (0 + 200) * 0.5 = 100.0 m`, an obstacle chord along the ray must span at least `[d, 100 m]` for the first midpoint probe to return `breached == true`. Our world obstacle (`OBSTACLE_TOWER_01`, radius `80 m`, offset `60 m` laterally from the flight line so it protrudes `20 m` across the nominal leg with a `105.8 m` chord) satisfies this exact C++ bisection condition.
   - Any ray distance `> PRX1_MAX` (`50.0 m`) is filtered out by `AP_Proximity_Backend::ignore_reading()`, so the distant inclusion box is ignored and only obstacles within `50.0 m` are reported.

3. **3D Boundary, `AP_OADatabase`, and `AP_OABendyRuler` (`OA_TYPE = 1`):**
   - Valid ray hits (`<= 50.0 m`) update `frontend.boundary` (which streams `DISTANCE_SENSOR` messages for orientations `0..7` to Python) and call `database_push(yaw_angle_deg, fence_distance)` to insert 3D obstacle points into `AP_OADatabase` (which streams `ADSB_VEHICLE` messages with callsign `"OA_DB"` to Python).
   - In `AUTO` mode, `AP_OABendyRuler` (`OA_TYPE = 1`, `OA_MARGIN_MAX = 10.0 m`, `OA_BR_LOOKAHEAD = 35.0 m`) probes candidate horizontal bearings against `AP_OADatabase` and steers the UAV smoothly around the obstacle.

---

## 3. Reproducible Commands

```bash
# 1. Run fast unit, contract & vehicle/perception tests (no SITL required)
PYTHONPATH=drone_medical_router "/Users/mahigarg/Documents/Drone Shit/.venv-sitl/bin/python" -m unittest \
  drone_medical_router/tests/test_flightstack.py \
  drone_medical_router/tests/test_full_autonomy_acceptance.py \
  drone_medical_router/tests/test_gdacs_and_provenance.py \
  drone_medical_router/tests/test_vehicle_and_perception.py -v

# 2. Run the full single-UAV hexacopter SITL mission (GDACS + 4D NFZ replan + C++ OA + auto hover/land delivery)
PYTHONPATH=drone_medical_router "/Users/mahigarg/Documents/Drone Shit/.venv-sitl/bin/python" \
  drone_medical_router/sitl_mission_demo.py \
  --scenario jaipur_single_uav --mode auto --airspace-change --inject-obstacle --speedup 10

# 3. Run the 8-case controlled experimental validation suite
PYTHONPATH=drone_medical_router "/Users/mahigarg/Documents/Drone Shit/.venv-sitl/bin/python" \
  drone_medical_router/experiments_validation.py
```

