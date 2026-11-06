# University Application & Technical Interview Defence Notes

These notes explain the core engineering decisions, algorithms, C++ autopilot internals, and mathematical formulations in Med-Drone Mission Control so you can explain and defend every part of the system in technical interviews.

---

## 1. End-to-End System Narrative (60-Second Pitch)

> *"Med-Drone Mission Control is a single-UAV autonomous medical-delivery and disaster-response flight stack integrated with ArduCopter V4.7.0-dev SITL over MAVLink v2. Unlike a static coordinate router, it reasons about changing external conditions at three levels: first, it ingests live macro-level disaster alerts from the UN/EC GDACS REST API with disk caching and strict provenance tags, while implementing the official Indian DGCA DigitalSky OpenAPI 3.0 flight-plan validation contract; second, it monitors 4D spatio-temporal airspace restrictions in flight, commanding a tactical `LOITER` hold and re-solving an A* detour path that must pass hard MTOW, fresh-battery Wh reserve, cold-chain, and time-window feasibility gates before item-by-item verified MAVLink upload; third, it uses ArduPilot's native C++ proximity raycaster (`AP_Proximity_SITL`, `PRX1_TYPE = 10`) and `AP_OABendyRuler` to detect and steer around physical world obstacles in `AUTO` mode, before executing site-suitability-aware hover or ground-landing deliveries."*

---

## 2. Deep-Dive Technical Questions & Answers

### Q1: Why did you choose native ArduPilot SITL with `AP_Proximity_SITL` (`PRX1_TYPE = 10`) instead of Gazebo Harmonic on your Apple M3 Mac?
- **Hardware Reality:** My development machine is an Apple M3 with `8 GB` unified memory and `~25 GB` free disk space running macOS 26.2. Inspecting the OSRF Homebrew tap (`osrf/simulation/gz-harmonic.rb`) showed `gz-harmonic` is deprecated due to disabled `freeimage` and pulls in 25+ heavy C++/Ogre2/Qt5 dependencies requiring `3.5–5.0 GB` of unified RAM during 3D rendering.
- **Autopilot Fidelity:** Native `arducopter` V4.7.0-dev runs the exact C++ flight control loops, EKF3 state estimator, `AP_Mission` state machine, `AP_OADatabase`, and `AP_OABendyRuler` in `< 45 MB` RAM at `10x` real-time speedup.
- **Eliminating Spoofed Sensor Packets:** Earlier code sent synthetic `OBSTACLE_DISTANCE` MAVLink messages from Python (`PRX1_TYPE = 2`), which meant Python was telling the autopilot where the obstacle was. By switching to `PRX1_TYPE = 10` (`AP_Proximity_SITL` in `libraries/AP_Proximity/AP_Proximity_SITL.cpp`), world obstacles are uploaded into `AC_PolyFence_loader` (`MAV_MISSION_TYPE_FENCE = 1`, `FENCE_ENABLE = 0`), and the C++ autopilot firmware itself casts 8 horizontal rays, detects the obstacle boundary, populates `AP_OADatabase`, emits `DISTANCE_SENSOR` and `ADSB_VEHICLE` (`OA_DB`) telemetry, and steers around the obstacle via `AP_OABendyRuler`.

### Q2: What subtle C++ detail did you uncover in `AP_Proximity_SITL::get_distance_to_fence()`?
- In `libraries/AP_Proximity/AP_Proximity_SITL.cpp` (lines 97–108), ArduPilot computes the distance to the fence in each of the 8 sectors (`0°, 45°, ..., 315°`) using a **binary bisection search** on `[0, 200 m]` (`min_dist = 0`, `max_dist = PROXIMITY_MAX_RANGE = 200.0f`):
  ```cpp
  while (max_dist - min_dist > PROXIMITY_ACCURACY) {
      float test_dist = (max_dist + min_dist) * 0.5f;
      Location loc = current_loc;
      loc.offset_bearing(angle_deg, test_dist);
      if (AP::fence()->polyfence().breached(loc)) {
          max_dist = test_dist;
      } else {
          min_dist = test_dist;
      }
  }
  ```
- Because the first probe is at `test_dist = 100.0 m`, a small `10 m` radius exclusion circle located `30 m` ahead has `breached(100 m) == false`, causing the binary search to set `min_dist = 100.0 m` and miss the obstacle completely!
- By offsetting an `80 m` radius exclusion circle `60 m` laterally from the flight line, the obstacle protrudes `20 m` across the nominal path while presenting a chord of $2\sqrt{80^2 - 60^2} = 105.8\text{ m} > 100.0\text{ m}$ along the approach direction. That guarantees `breached(100 m) == true` and `breached(50 m) == true` when the UAV is within `50 m` (`PRX1_MAX`) of the obstacle's leading edge, allowing the C++ bisection loop to converge to the exact obstacle surface at `0.1 m` resolution.

### Q3: Why did the drone previously disarm and get stuck in an `"Arm: AUTO mode not armable"` loop at intermediate landing stops, and how did you fix it?
- **Root Cause in ArduPilot C++ (`AP_Mission.cpp`):** When ArduCopter executes `MAV_CMD_NAV_LAND` (`21`) in `AUTO` mode, `ModeAuto::verify_land()` checks `mission.continue_after_land_check_for_takeoff()`. With `MIS_OPTIONS = 4` (`ContinueAfterLand`), `AP_Mission::is_takeoff_next()` scans upcoming commands after `NAV_LAND` and explicitly skips `DO` commands (`MAV_CMD_DO_GRIPPER = 211`) and `MAV_CMD_NAV_DELAY` (`93`) while looking for `MAV_CMD_NAV_TAKEOFF` (`22`).
- **The Bug:** `flightstack/mission.py` previously defined `MAV_CMD_NAV_DELAY = 94` (which is actually `MAV_CMD_NAV_PAYLOAD_PLACE`). When `is_takeoff_next()` encountered command `94`, it hit `default: return false;` and immediately disarmed the motors on touchdown! Then Python's `MissionRunner.step()` saw `not tel.armed` and called `self.link.arm()` while the flight mode was still `AUTO`, which ArduCopter rejects with `"Arm: AUTO mode not armable"`.
- **The Fix:**
  1. Corrected `MAV_CMD_NAV_DELAY = 93` in `flightstack/mission.py`. Now `is_takeoff_next()` sees `NAV_LAND (21) -> DO_GRIPPER (211) -> NAV_DELAY (93) -> NAV_TAKEOFF (22)`, keeps the motors in `GROUND_IDLE`, releases the package on the ground, waits 5 seconds (`Delaying 5 sec`), and automatically takes off (`Mission: 7 Takeoff`) and flies `RTL`.
  2. Hardened `MissionRunner.step()` so that if the vehicle ever disarms at an intermediate stop, it switches `set_mode("GUIDED") -> arm() -> set_current_item(retakeoff_seq) -> set_mode("AUTO") -> start_mission()`, never calling `arm()` in `AUTO` mode.

### Q4: How do you prevent infeasible replans from being uploaded mid-flight?
- In `MissionRunner.apply_airspace_update()`, after computing a candidate A* detour plan (`build_delivery_mission`), the runner enforces four hard gates before uploading:
  1. **MTOW Gate:** `vm.total_mass_kg(rem_cargo) <= vm.mtow_kg`.
  2. **Fresh Battery Telemetry Gate:** Rejects the replan (`operator_required`) if `telemetry.battery.value` is `None`, `remaining_pct` is `None`, or `battery.age() > 10.0 s` (eliminating the previous bug where missing battery telemetry defaulted to `100.0%` SoC), and verifies `energy_eval.total_energy_wh <= usable_wh_remaining` above the `20%` reserve floor.
  3. **Cold-Chain Gate (`enforce_cold_chain = True`):** Computes cumulative ETA along the **detoured A* polylines** (`_polyline_km(new_plan.leg_paths[idx])`) plus per-stop dwell times, rejecting the replan if `eta_min > cold_chain_limit_minutes`.
  4. **Hard Time-Window Gate (`enforce_hard_deadlines = True`):** Rejects the replan if `eta_min > window_minutes`.

### Q5: How do you handle external airspace and disaster data honestly?
- We separate all inputs using `SourceClassification`:
  - **GDACS (`LIVE_EXTERNAL` / `CACHED`):** Live GeoJSON fetch from `https://www.gdacs.org/gdacsapi/api/Events/geteventlist/SEARCH` and `/api/polygons/getgeometry` with mandatory attribution (`"Global Disaster Awareness and Coordination System, GDACS."`). Used for macro-level hazard awareness, never confused with micro-level landing pad clearance or legal aviation NFZs.
  - **DigitalSky (`OFFICIAL_API_VALIDATION`):** Implements the genuine OpenAPI 3.0.0 schema (`digitalsky_interactive_maps_api_v1.0.yml`) for `POST /flightplan/validate/airspace` (`X-API-VERSION: 1`, base64 GeoJSON `geom`, `maxAltitude <= 400 ft`), and honestly reports `UNAUTHORISED_NO_PILOT_TOKEN` (`official_clearance = False`) when pilot JWT credentials are not present.
  - **Mappls (`MAP_VISUALIZATION_ONLY`):** Documented as a browser vector-tile overlay (`mappls.airspaceLayer`) with no polygon export API.

