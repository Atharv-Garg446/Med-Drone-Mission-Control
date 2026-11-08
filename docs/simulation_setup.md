# Simulation Environment, Hardware Constraints, and Sensor Architecture

## 1. ArduPilot SITL Compilation and Environment Setup

The flight-stack demonstrations execute against native ArduCopter Software-In-The-Loop (SITL) firmware communicating with the Python mission runner over MAVLink v2 TCP (`127.0.0.1:5760`). Flight control, EKF3 state estimation, motor mixing, battery voltage sag, and proximity raycasting run inside the C++ ArduPilot process.

### 1.1 Building Native `arducopter` SITL
```bash
# Clone ArduPilot alongside this repository (flightstack/sitl.py searches ../ardupilot by default)
git clone --depth 1 --recurse-submodules --shallow-submodules \
    https://github.com/ArduPilot/ardupilot.git ../ardupilot

# Create virtual environment and install SITL dependencies
python3 -m venv ../.venv-sitl
../.venv-sitl/bin/pip install -r requirements.txt -r requirements-sitl.txt

# Configure and compile native arm64/x86_64 SITL binary
(cd ../ardupilot && ./waf configure --board sitl && ./waf copter)
```
The resulting binary is located at `../ardupilot/build/sitl/bin/arducopter`. Custom paths can be supplied via the `ARDUPILOT_HOME` or `ARDUCOPTER_SITL_BIN` environment variables.

---

## 2. Host Hardware Constraints (Apple M3, 8 GB Unified Memory)

- **Development Environment**: Apple M3 (8 cores, `arm64`), `8 GB` unified memory, macOS `26.2`.
- **Gazebo Harmonic vs. Native ArduPilot SITL**:
  - Running Gazebo Harmonic (`gz-sim8` + Ogre2 rendering + ROS 2 bridge) requires `3.5–5.0 GB` of unified memory during 3D scene rendering, which exceeds available memory headroom on an 8 GB host when run alongside ArduPilot SITL.
  - Native `arducopter` (`ArduCopter V4.7.0-dev`) uses `< 45 MB` RSS memory and `< 15%` of a single M3 core at `10x` real-time speedup (`--speedup 10`), and includes ArduPilot's built-in C++ proximity sensor (`AP_Proximity_SITL`, `PRX1_TYPE = 10`).

---

## 3. Coupled 6-Motor Hexacopter Airframe and 6S LiPo Model (`med_drone_hexa_16.json`)

To maintain consistency across Python's `MedDrone-Hexa-16` model (`flightstack/vehicle_model.py`), ArduCopter's motor mixer (`AP_MotorsMatrix`), and ArduPilot's C++ physics simulator (`SIM_Frame.cpp` + `SIM_Battery.cpp`), all three layers are configured as a 6-motor hexacopter on a 6S 16 Ah LiPo battery:

1. **Physics Airframe (`flightstack/params/med_drone_hexa_16.json` via `SitlProcess` `--model hexa:med_drone_hexa_16.json`)**:
   - `num_motors = 6`, `disc_area = 1.4255 m²` (`6` × `18-inch` rotors), `mass = 11.2 kg` (`5.5 kg` empty + `3.2 kg` battery + `2.5 kg` cargo allowance; plus `1.0 kg` `SIM_Gripper_Servo` external load = `12.2 kg` takeoff gross mass, dropping `1.0 kg` upon release).
   - 6S 16 Ah LiPo parameters: `maxVoltage = 25.2 V`, `refVoltage = 22.2 V`, `battCapacityAh = 16.0 Ah`, `refBatRes = 0.012 Ω`, `refCurrent = 31.0 A`, `hoverThrOut = 0.42`, `propExpo = 0.65`.
2. **Autopilot Parameters (`flightstack/params/sitl_delivery.parm`)**:
   - `FRAME_CLASS = 2` (`MAV_TYPE_HEXAROTOR = 13` reported in `HEARTBEAT` telemetry as `mav_type = "HEXAROTOR"`), `FRAME_TYPE = 0`.
   - `SIM_BATT_VOLTAGE = 25.04` (`6 × 4.173 V` = `100%` SoC on ArduPilot's `soc_table`), `SIM_BATT_CAP_AH = 16`, `BATT_CAPACITY = 16000`, `MOT_BAT_VOLT_MAX = 25.2`, `MOT_BAT_VOLT_MIN = 19.8`.
   - `EK3_DRAG_BCOEF_X/Y = 55.6`, `EK3_DRAG_MCOEF = 0.177`.
3. **Motor Mixer and Physics Frame Coupling**:
   - In `libraries/SITL/SIM_Frame.cpp`, launching SITL with `--model +` instantiates only 4 simulated motors (`quad_plus_motors`). If `FRAME_CLASS = 2` (6 motors in `AP_MotorsMatrix`) is used with `--model +`, motors 5 and 6 generate zero simulated thrust and the aircraft cannot take off. Pairing `--model hexa:med_drone_hexa_16.json` with `FRAME_CLASS = 2` aligns 6-motor thrust generation and 6S battery discharge (`25.039 V -> 21.925 V` over the 2-stop reference mission).

---

## 4. C++ Proximity Raycasting and Obstacle Avoidance (`PRX1_TYPE = 10`)

Instead of injecting synthetic `OBSTACLE_DISTANCE` packets from Python (`PRX1_TYPE = 2`), the SITL configuration (`flightstack/params/sitl_delivery.parm`) enables ArduPilot's internal C++ proximity driver (`PRX1_TYPE = 10`):

1. **World Geometry Upload (`AC_PolyFence_loader`)**:
   - `AutopilotLink.upload_fence_obstacles()` uploads world geometry to the autopilot via the MAVLink Fence Protocol (`mission_type = 1`, `MAV_MISSION_TYPE_FENCE`):
     - Four `MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION` (`5001`) vertices forming a `~22 km × 22 km` bounding box around home so `AP::fence()->polyfence().inclusion_boundary_available()` returns `true` and `PRX1` sensor status transitions to `Status::Good`.
     - Cylindrical world obstacles (`MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION = 5004`) placed at target coordinates `(lat, lon, radius_m)`.
   - `FENCE_ENABLE` is set to `0` so ArduCopter does not trigger a geofence RTL failsafe, while `AP_Proximity_SITL` raycasts against `AP::fence()->polyfence().breached(loc)`.
2. **C++ 8-Sector Bisection Raycaster (`AP_Proximity_SITL::get_distance_to_fence`)**:
   - On each proximity update cycle in C++, `AP_Proximity_SITL` casts 8 horizontal rays (`0°, 45°, 90°, 135°, 180°, 225°, 270°, 315°` in body frame, rotated by `sitl->state.yawDeg`).
   - Along each ray, it performs a binary search on `[0, 200 m]` (`0.1 m` resolution) testing `AP::fence()->polyfence().breached(loc)`.
   - Because `AP_Proximity_SITL::get_distance_to_fence` starts its binary search at `test_dist = 100.0 m`, an obstacle chord along the ray must span the `100.0 m` midpoint probe for `breached(100 m)` to evaluate `true`. The demo obstacle (`OBSTACLE_TOWER_01`, radius `80 m`, laterally offset `60 m` from the flight line so it protrudes `20 m` across the nominal leg with a `105.8 m` chord) satisfies this condition.
   - Any ray distance `> PRX1_MAX` (`50.0 m`) is filtered out by `AP_Proximity_Backend::ignore_reading()`.
3. **3D Boundary, `AP_OADatabase`, and `AP_OABendyRuler` (`OA_TYPE = 1`)**:
   - Valid ray hits (`<= 50.0 m`) update `frontend.boundary` (streaming `DISTANCE_SENSOR` messages for orientations `0..7` to Python) and call `database_push(yaw_angle_deg, fence_distance)` to insert 3D obstacle points into `AP_OADatabase` (streaming `ADSB_VEHICLE` messages with callsign `"OA_DB"` to Python).
   - In `AUTO` mode, `AP_OABendyRuler` (`OA_TYPE = 1`, `OA_MARGIN_MAX = 10.0 m`, `OA_BR_LOOKAHEAD = 35.0 m`) probes candidate bearings against `AP_OADatabase` and steers around the obstacle.

---

## 5. Verification Commands

```bash
# Run unit, contract, acceptance, and vehicle/perception tests (no SITL required)
python3 -m unittest \
  tests/test_flightstack.py \
  tests/test_full_autonomy_acceptance.py \
  tests/test_gdacs_and_provenance.py \
  tests/test_vehicle_and_perception.py -v

# Run the 2-stop single-UAV hexacopter SITL reference mission
python3 sitl_mission_demo.py \
  --scenario jaipur_single_uav --mode auto --airspace-change --inject-obstacle --speedup 10

# Run the 8-case controlled validation suite
python3 experiments_validation.py
```
