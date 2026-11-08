# Med-Drone Mission Control — Authoritative Project Master Context

**Document Status**: Authoritative Persistent Source of Truth  
**Repository Path**: `/Users/mahigarg/Documents/Drone Shit/drone_medical_router`  
**Host Environment**: Apple M3 (8 GB Unified RAM, macOS Darwin 26.2.0 `arm64`)  
**Python Runtime**: Python 3.10.13 (`/Users/mahigarg/Documents/Drone Shit/.venv-sitl/bin/python`)  
**Autopilot Firmware**: ArduCopter V4.7.0-dev (`/Users/mahigarg/Documents/Drone Shit/ardupilot/build/sitl/bin/arducopter`)

---

## 1. Original Project Objectives

Med-Drone Mission Control is an educational engineering and simulation prototype for a **single-UAV autonomous medical-delivery and disaster-response system** centered on the Jaipur reference scenario (`jaipur_single_uav`), while preserving the multi-UAV fleet routing research (`simulate_fleet.py`, `vrp_scratch.py`, `replan.py`) as an optional subsystem.

Its eleven non-negotiable engineering objectives are:
1. **Real External Disaster Information**: Live ingestion, deduplication, expiry filtering, polygon footprint fetching, caching, and operational mission-impact assessment from the official **GDACS API** (`https://www.gdacs.org/gdacsapi`) with mandatory attribution (`"Global Disaster Awareness and Coordination System, GDACS."`).
2. **Official Indian DigitalSky Airspace Validation**: Exact OpenAPI 3.0.0 schema compliance (`digitalsky_interactive_maps_api_v1.0.yml`) for `POST /flightplan/validate/airspace`, including `Error` and HAL `ValidationError` (`_embeded.errors`) parsing, `X-API-VERSION: 1`, `maxAltitude <= 400 ft`, and honest reporting when `DIGITALSKY_AUTH_TOKEN` is absent (`UNAUTHORISED_NO_PILOT_TOKEN`). Mappls Web SDK layers (`mappls.addAirspaceLayers()`) are explicitly bounded as `MAP_VISUALIZATION_ONLY`.
3. **Changing Airspace Restrictions & In-Flight Replanning**: 4D airspace awareness (`[lat, lon, min_alt_agl_m, max_alt_agl_m, valid_from, valid_until]`), runtime monitoring via `DynamicAirspaceMonitor` (detecting newly active zones, expired restrictions, and API outages where restrictions must never be cleared), and mid-flight MAVLink mission upload (`include_takeoff=False`).
4. **Geospatial Route Planning & Obstacle Avoidance**: A* shortest-path route planning around buffered polygon no-fly zones (`nofly_astar.py`) paired with tactical sensor-driven obstacle avoidance via ArduPilot's native C++ `AP_Proximity_SITL` (`PRX1_TYPE = 10`) and `AP_OABendyRuler` (`OA_TYPE = 1`).
5. **Vehicle, Payload, Battery, Energy & Delivery Constraints**: First-principles aerodynamic power model (`vehicle_model.py`), MTOW validation (`17.5 kg`), fresh autopilot battery telemetry gating (never defaulting missing/stale battery to `100%` SoC), time-based cold-chain limits (`cold_chain_limit_minutes`), and hard delivery deadline windows (`window_minutes`) enforced on both initial plans and every candidate in-flight replan.
6. **ArduPilot Autopilot Integration**: Closed-loop MAVLink integration (`flightstack/mavlink_link.py`) with item-by-item readback verification (`MISSION_REQUEST_INT` / `MISSION_ITEM_INT`), `GUIDED` arming, `AUTO` mission execution, EKF/GNSS health gating, and `RTL` recovery.
7. **Realistic Physics-Based Simulation & Sensor Pipeline**: Native ArduCopter V4.7.0-dev 6-motor hexacopter SITL physics (`--model hexa:med_drone_hexa_16.json`, `FRAME_CLASS = 2`, 6S 16 Ah LiPo discharge curve) with C++ horizontal raycasting (`AP_Proximity_SITL` casting 8 horizontal rays against cylindrical world obstacles loaded into `AC_PolyFence_loader` with `FENCE_ENABLE = 0`), populating `AP_OADatabase` (`OA_DB`) and streaming `DISTANCE_SENSOR` + `ADSB_VEHICLE` telemetry.
8. **Computer-Vision / Perception Integration**: YOLOv8n streaming detector (`flightstack/perception_pipeline.py`) with Apple Silicon MPS GPU acceleration, strict separation of 2D monocular bearing rays from metric 3D coordinates (rejecting depthless observations in `ObstacleTracker`), and OpenCV ArUco precision landing (`flightstack/precision_landing.py`) emitting `LANDING_TARGET` (#149) in `MAV_FRAME_BODY_NED`.
9. **Complete Hover & Ground Delivery Workflows**: Deterministic per-stop delivery-method selection (`resolve_stop_delivery_mode` supporting `--mode auto`, `hover`, and `land`), separating arrival, stabilization, simulated release (`MAV_CMD_DO_GRIPPER` confirmed via `SIM_Gripper_Servo` `STATUSTEXT`), ground dwell (`MAV_CMD_NAV_DELAY = 93` under `MIS_OPTIONS = 4` `ContinueAfterLand`), retakeoff (`MAV_CMD_NAV_TAKEOFF = 22`), and mid-flight site-suitability revalidation (`apply_site_suitability_update`).
10. **Mission Recovery, Final Landing & Disarming**: Safe return-to-launch (`MAV_CMD_NAV_RETURN_TO_LAUNCH`), touchdown below `0.5 m` relative altitude, motor disarm confirmation, and reserve-battery emergency `RTL` failsafe.
11. **Reproducible Testing & Credible Evidence**: Automated unit, contract, acceptance, and live SITL tests (`tests/`), structured JSONL mission logs (`output/sitl_run_*.jsonl`), and synchronized Leaflet HTML replay (`visualize_sitl.py`).

---

## 2. Agreed Architecture and Scope

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                     External Intelligence & Airspace Layer                   │
│  • GDACSProvider / CachedGDACSProvider / GDACSMissionImpactAssessor          │
│    (Live GeoJSON flood/earthquake/cyclone alerts + polygon footprints)       │
│  • DigitalSkyIntersectionChecker (OpenAPI 3.0 POST /flightplan/validate)     │
│  • DynamicAirspaceMonitor + AirspaceZone (4D altitude & time-window checks)  │
└──────────────────────────────────────┬───────────────────────────────────────┘
                                       │ Provenance-tagged zones & site updates
                                       ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│                   Strategic & Tactical Mission Planner                       │
│  • resolve_stop_delivery_mode() -> deterministic per-stop hover / land / abort│
│  • build_delivery_mission() -> A* leg routing around 4D buffered NFZ polygons│
│  • Shared Feasibility Validator (_validate_plan_feasibility):                │
│    1. Gross mass <= MTOW (17.5 kg) & cargo <= max_payload_kg (8.0 kg)        │
│    2. Fresh battery SoC + leg-stepdown energy_wh <= usable_wh_remaining      │
│    3. Elapsed ETA <= time-based cold_chain_limit_minutes                     │
│    4. Arrival ETA <= window_minutes + full-route DigitalSky check            │
└──────────────────────────────────────┬───────────────────────────────────────┘
                                       │ Verified MAVLink MissionItems + Fence
                                       ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│                Closed-Loop Flight Stack & ArduCopter SITL                    │
│  • MissionRunner (10 Hz closed-loop state machine + 1 Hz telemetry logger)   │
│  • PayloadManager (LOADED -> RELEASE_COMMANDED -> DELIVERED / FAILED)        │
│  • PerceptionPipeline & PrecisionLandingController (component-tested)        │
│  • ArduCopter V4.7.0-dev Hexa SITL (hexa:med_drone_hexa_16.json + EKF3 + OA) │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Important Engineering Decisions and Why They Were Made

1. **Single Authoritative Per-Stop Delivery Mode Resolver (`resolve_stop_delivery_mode`)**:
   - *Decision*: Introduced `resolve_stop_delivery_mode(stop, requested_mode="auto", stop_modes=None)` in [`flightstack/payload.py`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/flightstack/payload.py) and set `--mode auto` as the default in [`sitl_mission_demo.py`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/sitl_mission_demo.py).
   - *Why*: Previously, `sitl_mission_demo.py` defaulted to `--mode hover` while passing `stop_modes={2: "land"}`, and `PayloadManager.from_stops` assigned uniform `mode` across all packages before `assess_site` ran at arrival. Now, `build_delivery_mission`, `PayloadManager.from_stops`, `assess_site`, and `MissionRunner` all use the exact same deterministic decision rule:
     - If `obstacle_clearance_m < 10.0` or `recommended_mode == "abort"`, the site is rejected (`ok=False, selected_mode="abort"`) and placed in `plan.unreachable`. `"abort"` is never treated as a concrete delivery method.
     - If `not is_safe_for_landing` or `surface_condition in ("flooded", "debris", "mud", "submerged", "collapsed")` or `slope_deg > 7.0` or `uncertainty != "known"`, `"land"` is forbidden and downgraded to `"hover"`.
     - In `"auto"` mode, safe landing sites with `recommended_mode == "land"` select `"land"` (`Jhalana Elevated Medical Post`), while flooded/unsafe sites select `"hover"` (`Gopalpura Flooded Relief Camp`).
2. **ArduCopter Intermediate Landing Continuation (`MAV_CMD_NAV_DELAY = 93` + `MIS_OPTIONS = 4`)**:
   - *Decision*: Used `MAV_CMD_NAV_DELAY` (command ID `93`) instead of `MAV_CMD_CONDITION_DELAY` (`112`) after `MAV_CMD_NAV_LAND` (`21`), combined with `MIS_OPTIONS = 4` (`ContinueAfterLand`).
   - *Why*: In ArduCopter's `ModeAuto::do_Wait_Delay` vs. `do_Nav_Delay`, `MAV_CMD_NAV_DELAY` is a navigation command slot that keeps the mission state machine advancing into `MAV_CMD_NAV_TAKEOFF` (`22`) after ground touch-down and disarm-prevention during ground dwell.
3. **Native C++ Proximity Raycaster (`AP_Proximity_SITL` `PRX1_TYPE = 10`) over Gazebo**:
   - *Decision*: Used ArduPilot's built-in `AP_Proximity_SITL` C++ raycaster against `AC_PolyFence_loader` cylindrical obstacles (`FENCE_ENABLE = 0`, `OA_TYPE = 1` BendyRuler) instead of Gazebo Harmonic + ROS 2.
   - *Why*: On an 8 GB RAM Apple M3 MacBook, Gazebo Harmonic + ROS 2 + ArduPilot SITL exceeds physical RAM (~1.3 GB free), causing swap thrashing and OOM. `AP_Proximity_SITL` runs inside the native `arm64` `arducopter` process with < 50 MB RAM while exercising the real C++ `AP_Proximity_SITL -> AP_OADatabase -> AP_OABendyRuler` sensor-to-steering pipeline.
4. **Operational GDACS Pipeline with Strict Semantic Boundaries (`GDACSMissionImpactAssessor`)**:
   - *Decision*: Built `GDACSMissionImpactAssessor` in [`gdacs_provider.py`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/gdacs_provider.py) and wired it into `MissionRunner` (`poll_external_updates`) and `sitl_mission_demo.py`.
   - *Why*: Coarse point-centroid disaster alerts without polygon footprints must remain `ADVISORY_REGIONAL_AWARENESS` (never fabricated into circular no-fly zones), whereas validated GDACS flood/cyclone polygon footprints intersecting a planned `"land"` stop deterministically downgrade that stop's `SiteSuitability` to `"hover"` and trigger an in-flight mission update.

---

## 4. Current Working Implementation

- **Single-UAV Reference Mission (`jaipur_single_uav`)**: 1 UAV (`UAV-1`), SMS Hospital Depot (`id=0`) -> Stop 1 (`Gopalpura Flooded Relief Camp`, `id=1`, flooded -> `hover`) -> Stop 2 (`Jhalana Elevated Medical Post`, `id=2`, clear -> `land`) -> RTL to SMS Hospital Depot.
- **Per-Stop Delivery Selection & Mid-Flight Site Revalidation**: `resolve_stop_delivery_mode` in `flightstack/payload.py`, `build_delivery_mission` in `flightstack/mission.py`, and `MissionRunner.apply_site_suitability_update` in `flightstack/mission_runner.py`.
- **Live GDACS Ingestion & Mission Impact Assessment**: `GDACSProvider`, `CachedGDACSProvider`, and `GDACSMissionImpactAssessor` in `gdacs_provider.py`.
- **DigitalSky OpenAPI 3.0 Client & Dynamic Airspace Monitor**: `DigitalSkyIntersectionChecker`, `parse_digitalsky_error_payload` (`Error` and HAL `ValidationError`), `MapplsAirspaceLayerSpec`, and `DynamicAirspaceMonitor` in `airspace_provider.py`.
- **4-Gate In-Flight Replan Validation**: `_validate_plan_feasibility` in `flightstack/mission_runner.py` checking MTOW, fresh battery Wh, time-based cold-chain limits (`cold_chain_limit_minutes`), hard delivery deadlines, and DigitalSky validation. Shared between initial validation (`validate_initial_mission`) and in-flight replanning (`_validate_candidate_replan`).
- **Initial Mission Feasibility Gate**: `validate_initial_mission` in `flightstack/mission_runner.py` — blocks `launch()` when energy, MTOW, cold-chain, or deadline constraints are not met. Uses the same `_validate_plan_feasibility` logic as in-flight replanning.
- **Pre-Mission GDACS Application**: `apply_initial_gdacs_impact` in `flightstack/mission_runner.py` — evaluates and idempotently applies GDACS impact to stops and zones before mission upload.
- **Native C++ Obstacle Avoidance**: `upload_fence_obstacles` in `flightstack/mavlink_link.py` + `PRX1_TYPE=10`, `OA_TYPE=1` in `flightstack/params/sitl_delivery.parm`.
- **Perception & Precision Landing**: `flightstack/perception_pipeline.py` (YOLOv8n + metric depth gating) and `flightstack/precision_landing.py` (ArUco -> `LANDING_TARGET` #149).

---

## 5. Partially Implemented Features

1. **Live Authenticated DigitalSky Clearance**: The client (`DigitalSkyIntersectionChecker`) implements the full OpenAPI 3.0 request/response/error contract (`POST /flightplan/validate/airspace`), tested against mock HTTP transports (`TestDigitalSkyOpenAPISchemaAndMappls`, `TestDigitalSkyErrorSchemasAndDynamicMonitor`). Live execution against `https://digitalsky.aai.aero` requires an authenticated DGCA pilot JWT token (`DIGITALSKY_AUTH_TOKEN`).
2. **Closed-Loop 3D Camera-in-the-Loop World Rendering**: `PerceptionPipeline` runs real YOLOv8n inference on Apple Silicon MPS and has an integration hook in `MissionRunner.step()`, and `PrecisionLandingController` runs real OpenCV ArUco detection, but both are component/acceptance-tested and **not active** (`perception_pipeline=None`, `precision_landing_controller=None`) in the headless `sitl_mission_demo.py` run because SITL physics does not render a 3D visual world locally on an 8 GB M3 without Gazebo. Likewise, `DynamicAirspaceMonitor` is unit-tested and interface-ready (`check_for_updates`), while `sitl_mission_demo.py` injects its 4D TFR via a deterministic hook.

---

## 6. Missing Features and External Dependencies

1. **DGCA DigitalSky Pilot JWT (`DIGITALSKY_AUTH_TOKEN`)**: External government credential required for live `POST /flightplan/validate/airspace` calls.
2. **Physical Airframe & Recipient Handover Verification Sensor**: Simulated payload release is confirmed via ArduPilot `SIM_Gripper_Servo` (`STATUSTEXT: "Gripper Released"`), whereas physical recipient handover requires weight-on-hook/optical confirmation hardware (`recipient_handover_verified=False`).
3. **Ubuntu 22.04 + 16 GB+ RAM Workstation for Full Gazebo Harmonic 3D World**: Required only if full 3D visual rendering alongside SITL is desired beyond ArduPilot's native C++ `AP_Proximity_SITL` raycaster.

---

## 7. Hardware and Simulation Constraints

- **Primary Machine**: Apple M3, 8 GB unified memory, macOS 26.2 (`arm64`), ~25 GB free disk space.
- **Feasibility Verdict**:
  - Native `arm64` ArduCopter V4.7.0-dev SITL runs at `10x` speedup using `< 50 MB` RAM and `< 15%` of one M3 core.
  - Native YOLOv8n inference runs on Apple M3 MPS at `21–63 FPS`.
  - Gazebo Harmonic + ROS 2 Humble alongside SITL is infeasible on 8 GB unified memory (`~1.3 GB` free RAM under normal IDE load).

---

## 8. Important Repository Files and Their Responsibilities

| File Path | Responsibility |
| :--- | :--- |
| [`AGENTS.md`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/AGENTS.md) | Non-negotiable engineering rules, data provenance taxonomy, and architectural invariants. |
| [`config.py`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/config.py) | Scenarios (`jaipur_single_uav`, `jaipur_flood`, `jaipur_earthquake`), `Location`, `DroneSpecs`, and planning constants. |
| [`nofly_astar.py`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/nofly_astar.py) | 2D polygon geometry, safety margin buffering, and A* visibility/grid shortest-path router. |
| [`airspace_provider.py`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/airspace_provider.py) | `SourceClassification`, `AirspaceZone` (4D), `DigitalSkyIntersectionChecker`, `parse_digitalsky_error_payload`, `DynamicAirspaceMonitor`, `MapplsAirspaceLayerSpec`. |
| [`gdacs_provider.py`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/gdacs_provider.py) | Live GDACS API client (`GDACSProvider`), caching (`CachedGDACSProvider`), and `GDACSMissionImpactAssessor`. |
| [`flightstack/payload.py`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/flightstack/payload.py) | `SiteSuitability`, `resolve_stop_delivery_mode`, `MedicalPackage`, and `PayloadManager` cargo state machine. |
| [`flightstack/mission.py`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/flightstack/mission.py) | `MissionItem`, `MissionPlan`, `build_delivery_mission` (supporting `"auto"`, `"hover"`, `"land"`), `validate_polyline`. |
| [`flightstack/mavlink_link.py`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/flightstack/mavlink_link.py) | MAVLink TCP connection, mission upload & item-by-item readback verification, `upload_fence_obstacles`, command ACK handling. |
| [`flightstack/mission_runner.py`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/flightstack/mission_runner.py) | Closed-loop execution, `_validate_candidate_replan`, `apply_airspace_update`, `apply_site_suitability_update`, `poll_external_updates`, obstacle avoidance. |
| [`flightstack/vehicle_model.py`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/flightstack/vehicle_model.py) | First-principles rotorcraft aerodynamic power, battery Wh, MTOW, cold-chain, and deadline constraint checks. |
| [`flightstack/perception_pipeline.py`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/flightstack/perception_pipeline.py) | YOLOv8n detector, camera intrinsics, 2D bearing vs. 3D metric depth projection (`SpatialProjector`). |
| [`flightstack/obstacle_tracker.py`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/flightstack/obstacle_tracker.py) | 3D velocity estimation, CPA conflict prediction, and MAVLink `OBSTACLE_DISTANCE` 72-bin encoder. |
| [`flightstack/precision_landing.py`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/flightstack/precision_landing.py) | OpenCV ArUco marker detection, camera-to-body NED transform, and MAVLink `LANDING_TARGET` (#149) sender. |
| [`sitl_mission_demo.py`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/sitl_mission_demo.py) | End-to-end single-UAV SITL mission runner (`--mode auto`, `--airspace-change`, `--inject-obstacle`). |
| [`visualize_sitl.py`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/visualize_sitl.py) | Generates synchronized interactive HTML flight replay (`output/sitl_flight_replay.html`) from JSONL run logs. |

---

## 9. Reproducible Setup and Execution Commands

```bash
# 1. Verify host environment, SITL binary, and Python dependencies
PYTHONPATH=drone_medical_router "/Users/mahigarg/Documents/Drone Shit/.venv-sitl/bin/python" drone_medical_router/check_env.py

# 2. Run the full automated unit, contract, acceptance, and SITL integration test suite
PYTHONPATH=drone_medical_router "/Users/mahigarg/Documents/Drone Shit/.venv-sitl/bin/python" -m unittest discover -s drone_medical_router/tests -p "test_*.py"

# 3. Run the complete 2-stop Jaipur Single-UAV SITL reference mission (--mode auto, 4D TFR replan, C++ obstacle avoidance)
PYTHONPATH=drone_medical_router "/Users/mahigarg/Documents/Drone Shit/.venv-sitl/bin/python" drone_medical_router/sitl_mission_demo.py --scenario jaipur_single_uav --mode auto --airspace-change --inject-obstacle --speedup 10

# 4. Run the controlled 1-stop obstacle-avoidance comparison pair
# 4a. Baseline (without obstacle)
PYTHONPATH=drone_medical_router "/Users/mahigarg/Documents/Drone Shit/.venv-sitl/bin/python" drone_medical_router/sitl_mission_demo.py --scenario jaipur_single_uav --max-stops 1 --mode hover --speedup 10
# 4b. Obstacle-present (identical scenario, stop count, mode, and speedup; differs ONLY in --inject-obstacle)
PYTHONPATH=drone_medical_router "/Users/mahigarg/Documents/Drone Shit/.venv-sitl/bin/python" drone_medical_router/sitl_mission_demo.py --scenario jaipur_single_uav --max-stops 1 --mode hover --inject-obstacle --speedup 10

# 5. Generate interactive HTML flight replay from the latest SITL JSONL log
PYTHONPATH=drone_medical_router "/Users/mahigarg/Documents/Drone Shit/.venv-sitl/bin/python" drone_medical_router/visualize_sitl.py
```

---

## 10. Requirements Traceability Matrix & Verified Evidence

| Objective ID | Requirement | Implementation Location | Automated Tests | Execution Evidence | Status |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **REQ-01** | Live GDACS external disaster ingestion, caching, attribution & operational mission impact | `gdacs_provider.py` (`GDACSProvider`, `CachedGDACSProvider`, `GDACSMissionImpactAssessor`), `flightstack/mission_runner.py` (`poll_external_updates`) | `tests/test_gdacs_and_provenance.py` (`TestGDACSUnitAndFixtures`, `TestLiveGDACSExternalAPI`, `test_gdacs_mission_impact_assessor_advisory_vs_validated_polygon`) | Live GDACS fetch cached in `output/gdacs_cache_latest.json` & logged in `output/sitl_run_*.jsonl` (`gdacs_live_context`, `gdacs_mission_impact`) | **VERIFIED** |
| **REQ-02** | DigitalSky OpenAPI 3.0 flight-plan validation (`POST /flightplan/validate/airspace`), `Error` & HAL `ValidationError` parsing, Mappls boundary | `airspace_provider.py` (`DigitalSkyIntersectionChecker`, `parse_digitalsky_error_payload`, `MapplsAirspaceLayerSpec`) | `tests/test_gdacs_and_provenance.py` (`TestDigitalSkyOpenAPISchemaAndMappls`, `test_digitalsky_hal_validation_error_and_500_error_parsing`) | `digitalsky_validation` event in SITL JSONL logs; contract tests against official YAML spec | **VERIFIED** (Live call gated on `DIGITALSKY_AUTH_TOKEN`) |
| **REQ-03** | 4D dynamic airspace monitoring (altitude bands, time windows, expiry, outage safety) & in-flight replanning | `airspace_provider.py` (`AirspaceZone`, `DynamicAirspaceMonitor`), `flightstack/mission_runner.py` (`apply_airspace_update`) | `tests/test_flightstack.py` (`TestAirspace4DValidation`), `tests/test_gdacs_and_provenance.py` (`test_dynamic_airspace_monitor_tracks_additions_expiry_and_outage`) | `airspace_update_applied` (`version: 2`, `detour_waypoints: 1`) in `output/sitl_run_*.jsonl` | **VERIFIED** |
| **REQ-04** | Geospatial A* route planning around buffered polygons | `nofly_astar.py`, `flightstack/mission.py` (`build_delivery_mission`, `validate_polyline`) | `tests/test_core_logic.py`, `tests/test_flightstack.py` (`TestMissionBuilder`) | `EXP-01` (`8.828 km -> 9.002 km` detour, `min_measured_clearance_m = 65.97 m >= 50.0 m`) + zero NFZ buffer violations across all planned and replanned SITL missions | **VERIFIED** |
| **REQ-05** | Shared feasibility validation (MTOW `17.5 kg`, payload `8.0 kg`, fresh battery Wh without double-reserve penalty, leg-by-leg payload mass step-down, time-based cold-chain limits, deadlines) on initial plans & replanned routes | `flightstack/vehicle_model.py`, `flightstack/mission_runner.py` (`_validate_plan_feasibility`, `validate_initial_mission`, `_validate_candidate_replan`) | `tests/test_full_autonomy_acceptance.py` (`TestMissionVerificationAndReplanConstraints`, `TestInitialMissionFeasibilityAndExternalData`), `tests/test_vehicle_and_perception.py` | Replan validation checks logged in `airspace_update_applied`; initial validation gates `launch()`; `output/experimental_validation_report.json` (`EXP-02`: `136.91 Wh <= 284.16 Wh`, `21%` SoC = `3.6 Wh` rejected) | **VERIFIED** |
| **REQ-06** | Closed-loop ArduPilot Hexacopter SITL MAVLink execution (`hexa:med_drone_hexa_16.json`, `FRAME_CLASS=2`, `MAV_TYPE_HEXAROTOR=13`) & item-by-item readback verification | `flightstack/mavlink_link.py` (`upload_mission`, `verify_mission_readback`), `flightstack/sitl.py`, `flightstack/params/med_drone_hexa_16.json` | `tests/test_flightstack.py` (`TestMavlinkLink`), `tests/test_sitl_integration.py`, `tests/test_vehicle_and_perception.py` | `mission_uploaded` with readback verification on v1 and v2 missions + `mav_type="HEXAROTOR"` in `output/sitl_run_20261108_034446.jsonl` | **VERIFIED** |
| **REQ-07** | Native C++ sensor-driven obstacle avoidance (`AP_Proximity_SITL` + `AP_OABendyRuler`) with controlled baseline comparison | `flightstack/mavlink_link.py` (`upload_fence_obstacles`), `flightstack/params/sitl_delivery.parm` (`PRX1_TYPE=10`, `OA_TYPE=1`) | `tests/test_flightstack.py` (`TestNativeObstacleAvoidanceProximity`) | Controlled 1-stop hexacopter pair (`output/sitl_run_20261108_035408.jsonl` baseline `0.0 m` deviation vs. `output/sitl_run_20261108_035631.jsonl` obstacle-injected `38.42 m` deviation) + 2-stop hexacopter full run (`output/sitl_run_20261108_034446.jsonl` `34.08 m` deviation); `EXP-03` | **VERIFIED** |
| **REQ-08** | Perception pipeline (YOLOv8n + metric depth gating) & ArUco precision landing (`LANDING_TARGET` #149) | `flightstack/perception_pipeline.py`, `flightstack/obstacle_tracker.py`, `flightstack/precision_landing.py` | `tests/test_full_autonomy_acceptance.py` (`TestPerceptionPipeline`, `TestPrecisionLandingIntegration`), `tests/test_vehicle_and_perception.py` | `perception/evaluate_model.py` + unit/acceptance suite (component-tested; not active in headless SITL demo) | **VERIFIED (COMPONENT)** |
| **REQ-09** | Deterministic per-stop delivery selection (`auto`, `hover`, `land`), causal `SIM_Gripper_Servo` confirmation, ground dwell (`NAV_DELAY=93`), retakeoff, and mid-flight site revalidation | `flightstack/payload.py` (`resolve_stop_delivery_mode`, `PayloadManager`), `flightstack/mission.py`, `flightstack/mission_runner.py` (`apply_site_suitability_update`) | `tests/test_full_autonomy_acceptance.py` (`test_auto_mode_selects_hover_for_stop1_and_land_for_stop2_consistently`, `test_mid_flight_site_suitability_update_rebuilds_and_uploads_mission`, `TestPayloadRunnerIntegrationAndDwell`) | Full 2-stop hexacopter SITL run with `--mode auto` (`output/sitl_run_20261108_034446.jsonl`): Stop 1 `hover` (`simulated_hover_gripper_release`), Stop 2 `land` + ground dwell + retakeoff (`simulated_ground_gripper_release_after_dwell`) | **VERIFIED** |
| **REQ-10** | Mission recovery, RTL landing, and motor disarm | `flightstack/mission_runner.py` (`RunnerState.finished`, `battery_reserve_breached` -> `RTL`) | `tests/test_sitl_integration.py`, `experiments_validation.py` (`EXP-08`) | `final_landing_confirmed` (`alt_rel_m = 0.019 m`, `armed: false`, `success: true`, `21.925 V`, `39%` battery left) in `output/sitl_run_20261108_034446.jsonl` | **VERIFIED** |

---

## 11. Remaining Work in Priority Order

All core engineering priorities (Priorities 0 through 5) from the Master Continuation Brief have been implemented and verified. Future extensions outside the current hardware/credential envelope are:
1. Obtain a registered DGCA pilot/operator JWT (`DIGITALSKY_AUTH_TOKEN`) to execute live authenticated `POST /flightplan/validate/airspace` requests against `https://digitalsky.aai.aero`.
2. Deploy the workspace on an Ubuntu 22.04 LTS machine (16 GB+ RAM, discrete GPU) to run Gazebo Harmonic 3D camera rendering alongside ArduPilot SITL.
3. Train a domain-specific aerial hazard YOLO model on annotated low-altitude disaster imagery (`perception/train_yolo.py`).

---

## 12. Explicit Features Deferred or Rejected

1. **Weather API Integration**: Explicitly deferred per project brief (`PART 2`: *"Weather integration was explicitly deferred. Do not reintroduce it during this work"*).
2. **Gazebo Harmonic + ROS 2 Local Execution on 8 GB M3 Mac**: Rejected after hardware feasibility assessment showed `< 1.5 GB` available RAM, which guarantees swap thrashing and OOM when paired with SITL. Replaced with ArduPilot's native C++ `AP_Proximity_SITL` raycaster.
3. **Treating `"abort"` as a Delivery Mode**: Explicitly rejected and prohibited in code (`CONCRETE_DELIVERY_MODES = ("hover", "land")`). Unsuitable stops (`obstacle_clearance_m < 10.0` or `recommended_mode == "abort"`) are rejected (`ok=False`) and placed in `plan.unreachable`.
4. **Fabricating Circular No-Fly Zones from Point-Only GDACS Alerts**: Explicitly rejected. GDACS alerts without official polygon footprints are retained strictly as `ADVISORY_REGIONAL_AWARENESS`.
