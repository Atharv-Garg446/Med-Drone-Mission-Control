# Phase 0: System Audit & Implementation Roadmap

**Repository:** `Atharv-Garg446/Med-Drone-Mission-Control`  
**Audit Date:** October 2026  
**Host Environment:** macOS 26.2 (Darwin arm64), Apple Silicon, 8 GB RAM, Apple Clang 21, Python 3.10.13  
**Autopilot Integration:** ArduPilot SITL (`Copter-4.5` branch, native arm64 binary), pymavlink 2.4.50  

---

## 1. Executive Summary & Verification Context

This audit evaluates the codebase against the objective of transforming `Med-Drone-Mission-Control` from an algorithmic simulator into a verified, source-grounded prototype combining mission planning, official airspace constraints, dynamic obstacle handling, and an established autonomous flight stack (ArduPilot).

### Verified Test Baseline
- **Core algorithmic test suite:** 94 unit/regression tests (`tests/test_core_logic.py`) covering geometry, A* routing, CVRP heuristics, and constraints. All passing.
- **Flight-stack & airspace test suite:** 31 tests (`tests/test_flightstack.py`) covering telemetry parsing, staleness rejection, mission conversion, MAVLink upload/download protocols, and airspace GeoJSON/cache providers. All passing.
- **SITL integration test:** 1 hardware-in-the-loop test (`tests/test_sitl_integration.py`) executing against real ArduCopter SITL firmware. Passing in ~80 s.
- **Total active tests:** 126 automated tests passing clean.

---

## 2. Ground-Truth Architectural Audit

| Subsystem | What is Implemented & Tested | What Exists Only as an Abstraction | Custom Simulator Only | Actual End-to-End Integration | Bugs / Unsupported Assumptions Discovered |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **VRP & Heuristics** (`vrp_scratch.py`) | Nearest Neighbor, Urgency-Weighted NN, Clarke-Wright Savings, 2-opt intra-route, Or-opt inter-route. | OR-Tools baseline comparison (`vrp_ortools.py`) does not model cold-chain or urgency discounts. | N/A | Fully integrated into pre-flight mission planner and in-flight replanner. | None. Fast, deterministic, zero external dependencies. |
| **Airspace Pathfinding** (`nofly_astar.py`, `geo_utils.py`) | 8-connected grid A*, AABB pre-filtering, 50 m safety clearance, diagonal corner-cutting block, string-pulling smoothing. | Assumes 2D polygon extrusion; no 3D vertical corridor stepping. | Replays in Leaflet viewer. | Detour waypoints exported to QGC WPL 110 and uploaded directly to ArduPilot SITL AUTO missions. | Early versions evaluated full grid polygons on every call (fixed via bounding boxes & grid memoization). |
| **Autopilot Flight Stack** (`flightstack/`, `sitl_mission_demo.py`) | MAVLink telemetry parser with staleness detection (`telemetry.py`), mission generator (`mission.py`), MAVLink client (`mavlink_link.py`), native SITL manager (`sitl.py`), mission runner (`mission_runner.py`). | Gazebo camera/depth sensor simulation not native on macOS arm64 without virtualization. | Custom discrete-event simulator (`simulate_fleet.py`) handles multi-drone fleet dispatch. | **Verified in ArduCopter SITL:** Heartbeat, arming, AUTO takeoff, waypoint navigation, delivery (hover/land), RTL, and mid-flight airspace replanning. | **Critical discovery:** In ArduCopter, `NAV_LAND` followed by `NAV_TAKEOFF` disarms the motors and halts the mission unless `MIS_OPTIONS = 4` (`ContinueAfterLand`) is set. Fixed in `sitl_delivery.parm`. |
| **Airspace Provider** (`airspace_provider.py`) | GeoJSON ingestion, temporal validity filtering, category policy (red/yellow/green/TFR), cached fallback with `stale=True` flags. | Digital Sky REST interface schema is not public without active pilot authentication; no open geometry download endpoint exists. | Synthetic scenario polygons (`config.py`). | GeoJSON files feed directly into A* flight-matrix construction and pre-upload mission validation. | Assumption that Digital Sky publishes downloadable polygon layers was invalidated by web bundle inspection; Digital Sky provides an intersection-check API. |
| **Computer Vision & Safety** (`yolo_safety.py`) | YOLOv8 bounding-box classification for landing pads (person, vehicle, dog). | 3D depth/distance estimation: single bounding-box heuristics cannot measure physical distance reliably. | Placeholder image checks before landing. | Not yet hooked into live video stream or MAVLink `LANDING_TARGET` precision landing. | YOLO bounding-box pixel area was previously conflated with landing safety clearance without depth context. |
| **Vehicle Constraints & Energy** (`config.py`) | Distance-and-reserve battery budget, payload capacity limits, turnaround and dwell times. | Mass-dependent energy consumption (MTOW, airframe mass, payload mass scaling motor current). | Fixed battery percentage burn per km. | ArduPilot SITL simulated battery monitor (`SIM_BATT_CAP_AH`) reports voltage, current, and consumed mAh over MAVLink. | Simulator assumed constant power draw regardless of payload mass. |

---

## 3. High-Value Technical Additions

1. **Mass-Dependent Energy & Payload Model**:
   - Explicit vehicle mass breakdown: Frame Mass ($m_{\text{dry}}$), Battery Mass ($m_{\text{batt}}$), Maximum Takeoff Weight ($\text{MTOW}$), and Payload Mass ($m_{\text{cargo}}$).
   - Dynamic power estimation where hover and cruise power scale with $(m_{\text{total}})^{1.5}$ based on momentum theory, validating battery reserve under real payload drop-offs.
2. **ArUco Visual Precision Landing Integration**:
   - Detection of ArUco fiducial markers on landing zones using OpenCV.
   - Estimation of 3D camera-frame relative pose $(x, y, z)$ and yaw.
   - Generation of MAVLink `LANDING_TARGET` messages for ArduPilot precision loiter/land.
3. **Structured Dynamic Airspace Ingestion**:
   - Direct integration of verified airspace GeoJSON fixtures into the SITL mission runner.
   - Validation that dynamically introduced red zones instantly trigger in-flight LOITER and real-time replanning.

---

## 4. Phased Implementation Roadmap

### Milestone 1 (M1): Flight-Stack Foundation & Telemetry (Completed)
- **Status:** **VERIFIED & OPERATIONAL**
- **Artifacts:** `flightstack/telemetry.py`, `flightstack/mission.py`, `flightstack/mavlink_link.py`, `flightstack/sitl.py`, `flightstack/mission_runner.py`, `flightstack/params/sitl_delivery.parm`.
- **Acceptance Test:** `tests/test_sitl_integration.py` runs full SITL arming, takeoff, and stale-telemetry abort. 31 new unit tests pass in `tests/test_flightstack.py`.

### Milestone 2 (M2): Source-Grounded Airspace Adapter (Completed)
- **Status:** **VERIFIED & OPERATIONAL**
- **Artifacts:** `airspace_provider.py`, `docs/AIRSPACE_DATA.md`, `tests/fixtures/airspace_sample.geojson`.
- **Acceptance Test:** Unit tests verify category extraction, time-window validity filtering, cache expiration on outage, and credential gating for Digital Sky intersection checks.

### Milestone 3 (M3): Vehicle Physics, Mass & Energy Modeling
- **Deliverables:**
  - Create `flightstack/vehicle_model.py` defining airframe empty weight, battery specs, MTOW, and aerodynamic drag.
  - Compute payload-dependent power draw: $P_{\text{cruise}} = P_0 \left(\frac{m_{\text{total}}}{m_{\text{ref}}}\right)^{1.5} + \frac{1}{2}\rho v^3 C_D A$.
  - Update route feasibility checking to reject routes where payload + distance exceed energy reserves.
- **Acceptance Criteria:**
  - Test verifying a route feasible at 0 kg payload is diagnosed infeasible when loaded to full 8 kg payload due to increased discharge rate.
  - Energy prediction matched against ArduPilot SITL battery monitor logs within calibrated tolerance.
- **Estimated Effort:** 1 day.

### Milestone 4 (M4): Precision Landing & Visual Target Tracking
- **Deliverables:**
  - Implement `flightstack/precision_landing.py` with OpenCV ArUco detector (Dictionary `DICT_4X4_50`).
  - Compute 3D translation vector $(t_x, t_y, t_z)$ using camera intrinsic matrix.
  - Provide MAVLink `LANDING_TARGET` message builder (MAVLink message #149) with angle offsets or position vectors.
- **Acceptance Criteria:**
  - Unit tests verifying ArUco marker pose estimation with known ground-truth transforms.
  - Fallback mechanism ensuring mission aborts to safe hover-and-release if marker is obscured or landing site is obstructed.
- **Estimated Effort:** 1-2 days.

### Milestone 5 (M5): Dynamic Obstacle Perception & Reactive Avoidance
- **Deliverables:**
  - Implement `flightstack/obstacle_tracker.py` modeling moving obstacles with 3D position and velocity vectors.
  - Coordinate system transformation from sensor observation frame to local NEU/WGS-84 coordinates.
  - Conflict detection: predict time-to-closest-point-of-approach ($T_{\text{CPA}}$) and distance at closest approach ($D_{\text{CPA}}$).
  - Autopilot reactive response: trigger MAVLink `MAV_CMD_DO_REPOSITION` / `GUIDED` velocity nudge or invoke dynamic replan if obstacle corridor is blocked.
- **Acceptance Criteria:**
  - Deterministic simulation run where an obstacle crosses the flight corridor, triggers conflict detection, commands autopilot deviation, and logs full event sequence.
- **Estimated Effort:** 2 days.

### Milestone 6 (M6): System Validation & Final Documentation
- **Deliverables:**
  - End-to-end mission demonstration script running complete scenario with all components: mass-energy validation, airspace constraints, SITL flight, hover/land delivery, and mid-flight re-routing.
  - Update `README.md` and `DEVELOPMENT.md` reflecting verified, source-grounded performance and capabilities.
- **Acceptance Criteria:**
  - Full test suite passes (all 126+ unit and integration tests).
  - Reproducible single-command demo script logs verified mission completion.
- **Estimated Effort:** 1 day.
