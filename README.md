# Med-Drone Mission Control

Med-Drone Mission Control is a work-in-progress Python and MAVLink autonomy prototype for single-UAV medical supply delivery in disaster-affected areas. The current milestone demonstrates a closed-loop mission workflow against ArduPilot's ArduCopter Software-In-The-Loop (SITL) simulator (`ArduCopter V4.7.0-dev`), combining A* route planning around restricted airspace, pre-launch and in-flight feasibility checks, proximity-based obstacle avoidance, and per-stop hover or ground-landing delivery sequences.

The repository also retains an earlier multi-drone Capacitated Vehicle Routing Problem (CVRP) solver and discrete-event fleet simulator used for logistics research.

---

## 1. Problem and Motivation

Flooding and infrastructure damage can cut off road access to clinics and relief camps while creating changing hazards for low-altitude aircraft. A medical delivery UAV operating in this environment cannot rely on a static waypoint list alone:

- Airspace restrictions can change while the aircraft is already airborne.
- Ground conditions at a relief camp (such as standing water, debris, or steep terrain) may make landing unsafe and require a low-altitude hover drop instead, or force the stop to be skipped if lateral clearance is inadequate.
- Any mid-flight detour must still respect the aircraft's maximum takeoff weight (MTOW), remaining battery energy above a reserve floor, cargo cold-chain time limits, and delivery deadlines before a new mission is uploaded to the autopilot.

This project explores how to connect route planning, external hazard and airspace data contracts, feasibility gating, and autopilot execution into a single verifiable software loop.

---

## 2. What the Current Prototype Does

Capabilities in the repository fall into four distinct maturity levels:

### A. Demonstrated in the Integrated SITL Mission (`sitl_mission_demo.py`)
- **Single-UAV Two-Stop Mission (`JAIPUR_SINGLE_UAV`)**: Plans and flies a sortie from SMS Hospital Depot (`26.9127, 75.8060`) to Stop 1 (`Gopalpura Flooded Relief Camp`, `2.0 kg`) and Stop 2 (`Jhalana Elevated Medical Post`, `1.5 kg`), then returns to launch (`RTL`).
- **Unified Six-Motor Hexacopter Configuration**: Pairs Python's `MedDrone-Hexa-16` vehicle model (`flightstack/vehicle_model.py`) with a custom 6-motor ArduPilot SITL physics frame (`flightstack/params/med_drone_hexa_16.json`, `--model hexa:med_drone_hexa_16.json`) and ArduCopter parameters (`FRAME_CLASS=2`, `FRAME_TYPE=0`, `MAV_TYPE_HEXAROTOR=13` in `flightstack/params/sitl_delivery.parm`) on a 6S 16 Ah LiPo discharge curve (`25.04 V` full to `19.8 V` minimum).
- **A* Pathfinding Around Restricted Polygons**: Routes each leg around polygon no-fly zones with a 50 m horizontal safety buffer (`nofly_astar.py`, `flightstack/mission.py`).
- **Shared Feasibility Gating**: Uses a single validator (`MissionRunner._validate_plan_feasibility`) before launch and before any mid-flight replan upload to check payload capacity (`<= 8.0 kg`), MTOW (`<= 17.5 kg`), fresh battery telemetry and remaining usable Watt-hours above a 20% reserve floor (stepping carried cargo mass down after each stop: `3.5 kg -> 1.5 kg -> 0.0 kg`), time-based cold-chain limits (`cold_chain_limit_minutes`), and delivery time windows (`window_minutes`).
- **MAVLink Upload and Readback Verification**: Uploads missions over MAVLink TCP and reads back every item from the autopilot (`verify_mission_readback`) to confirm command IDs, coordinates, and altitudes before arming or resuming `AUTO`.
- **ArduPilot C++ Proximity Raycasting and BendyRuler Avoidance**: Uploads cylindrical world obstacles to ArduPilot's fence loader (`MAV_MISSION_TYPE_FENCE=1`, `FENCE_ENABLE=0`). ArduCopter's built-in C++ sensor (`AP_Proximity_SITL`, `PRX1_TYPE=10`) raycasts 8 horizontal sectors against the obstacle, populates `AP_OADatabase` (`ADSB_VEHICLE` `OA_DB` telemetry), and steers around the obstacle in `AUTO` mode using `AP_OABendyRuler` (`OA_TYPE=1`).
- **Mid-Flight 4D Airspace Conflict and Replanning**: Detects when a time- and altitude-bounded restriction (`AirspaceZone`) blocks the remaining path, switches the autopilot to `LOITER`, computes an A* detour from the live EKF position estimate (`include_takeoff=False`), re-validates feasibility, uploads and verifies the updated mission, and resumes `AUTO`.
- **Per-Stop Delivery Selection, Gripper Feedback, and Landing Continuation**: Resolves each stop deterministically (`resolve_stop_delivery_mode`): executes a `15 m` hover release at flooded Stop 1 and a full ground landing (`MAV_CMD_NAV_LAND`), gripper release (`MAV_CMD_DO_GRIPPER`), 5-second ground dwell (`MAV_CMD_NAV_DELAY=93` with `MIS_OPTIONS=4` `ContinueAfterLand`), and retakeoff (`MAV_CMD_NAV_TAKEOFF=22`) at clear Stop 2. Release acknowledgement waits causally for ArduPilot's `"Gripper Released"` `STATUSTEXT` message.
- **RTL, Touchdown, and Disarm**: Flies back to home, lands (`alt_rel_m < 0.5 m`), and confirms motor disarm.

### B. Live External Data and API Contract Validation
- **GDACS Disaster Feed (`gdacs_provider.py`)**: Fetches live flood (`FL`), earthquake (`EQ`), and tropical cyclone (`TC`) GeoJSON alerts and polygon footprints from `https://www.gdacs.org/gdacsapi`, deduplicates episodes, filters expired alerts, caches snapshots to `output/gdacs_cache_latest.json` (`stale=True` on outage), and includes required GDACS attribution. Point-only alerts are kept as regional awareness (`ADVISORY_REGIONAL_AWARENESS`) rather than converted into circular no-fly zones; validated flood/cyclone polygons intersecting a planned landing stop downgrade that stop from `land` to `hover`.
- **DigitalSky OpenAPI 3.0 Contract (`airspace_provider.py`)**: Builds and validates full-route corridor payloads against the official DigitalSky specification (`docs/external_specs/digitalsky_interactive_maps_api_v1.0.yml`, `POST /flightplan/validate/airspace`, `X-API-VERSION: 1`, `maxAltitude <= 400 ft`) and parses HTTP 400 HAL `ValidationError` and HTTP 500 `Error` schemas. Live HTTP execution requires a registered DGCA pilot/operator JWT (`DIGITALSKY_AUTH_TOKEN`); without it, the checker reports `checked=False, official_clearance=False, status="UNAUTHORISED_NO_PILOT_TOKEN"`.
- **Mappls Airspace Layer Metadata**: Documented as `MAP_VISUALIZATION_ONLY` because the Mappls Web SDK provides browser vector-tile overlays without a polygon-export REST API.

### C. Component-Tested Only (Not Active in the Headless SITL Demo)
- **YOLOv8n Perception and Metric-Depth Gating (`flightstack/perception_pipeline.py`, `yolo_safety.py`)**: Tested on recorded/synthetic frames and Apple Silicon MPS (`perception/evaluate_model.py`). Strictly separates 2D monocular bearing rays from 3D metric coordinates and rejects depthless detections in `ObstacleTracker`. Not wired into `sitl_mission_demo.py` because headless ArduPilot SITL does not render a 3D camera world.
- **3D CPA Obstacle Tracker (`flightstack/obstacle_tracker.py`)**: Closest-point-of-approach conflict detection and 72-bin `OBSTACLE_DISTANCE` encoding, verified in unit tests (`tests/test_vehicle_and_perception.py`, `tests/test_flightstack.py`).
- **ArUco Precision Landing (`flightstack/precision_landing.py`)**: OpenCV ArUco marker detection, camera-to-body NED coordinate transform, and MAVLink `LANDING_TARGET` (#149) message generation with low-altitude target-loss abort (`LOITER`), verified in unit and acceptance tests.
- **Dynamic Airspace Polling Monitor (`DynamicAirspaceMonitor` in `airspace_provider.py`)**: Tracks 4D zone additions, expirations, and fail-safe upstream outages (`safe_to_clear_restrictions=False`) in unit tests and `EXP-04`/`EXP-07`. In `sitl_mission_demo.py`, the mid-flight 4D restriction (`TFR_POPUP_4D_01`) is injected via a deterministic callback after Stop 1.
- **Multi-Drone CVRP Solver and Fleet Simulator (`vrp_scratch.py`, `vrp_ortools.py`, `simulate_fleet.py`, `replan.py`)**: Standalone operations-research modules for multi-depot/multi-drone routing comparisons (`main.py`, `run_simulation.py`).

---

## 3. Repository Structure

```text
drone_medical_router/
├── sitl_mission_demo.py          # Main single-UAV ArduCopter SITL demonstration entry point
├── experiments_validation.py     # 8-case controlled validation suite -> output/experimental_validation_report.json
├── config.py                     # Scenarios (JAIPUR_SINGLE_UAV, JAIPUR_DISASTER, CHENNAI_FLOOD), specs, locations
├── geo_utils.py                  # Haversine distance, point-in-polygon, segment-polygon clearance helpers
├── nofly_astar.py                # 8-connected grid A* pathfinder with 50 m polygon buffer and string-pulling
├── airspace_provider.py          # SourceClassification, 4D AirspaceZone, DigitalSky OpenAPI client, DynamicAirspaceMonitor
├── gdacs_provider.py             # Live/cached GDACS client and GDACSMissionImpactAssessor
├── flightstack/
│   ├── sitl.py                   # ArduCopter SITL process launcher (defaults to --model hexa:med_drone_hexa_16.json)
│   ├── mavlink_link.py           # MAVLink TCP connection, mission upload/readback, fence obstacle upload
│   ├── telemetry.py              # Timestamped MAVLink state (position, EKF, GNSS, 6S battery, OA_DB, DISTANCE_SENSOR)
│   ├── mission.py                # MissionPlan builder, 4D polyline validator, QGC WPL 110 exporter
│   ├── mission_runner.py         # Closed-loop execution, shared 4-gate feasibility validator, replanning, RTL failsafe
│   ├── payload.py                # SiteSuitability, resolve_stop_delivery_mode, causal PayloadManager state machine
│   ├── vehicle_model.py          # First-principles rotorcraft hover/cruise power, MTOW, and leg-by-leg energy model
│   ├── perception_pipeline.py    # Component-tested YOLOv8n detector and 2D/3D spatial projector
│   ├── obstacle_tracker.py       # Component-tested 3D CPA tracker and OBSTACLE_DISTANCE encoder
│   ├── precision_landing.py      # Component-tested ArUco detector and MAVLink LANDING_TARGET controller
│   └── params/
│       ├── med_drone_hexa_16.json # 6-motor hexacopter physics & 6S 16 Ah LiPo parameters for ArduPilot SIM_Frame
│       └── sitl_delivery.parm    # ArduCopter SITL parameters (FRAME_CLASS=2, PRX1_TYPE=10, OA_TYPE=1, MIS_OPTIONS=4)
├── vrp_scratch.py                # Multi-heuristic CVRP solver (Nearest Neighbor, Urgency-NN, Clarke-Wright + 2-opt/Or-opt)
├── simulate_fleet.py             # Discrete-event multi-drone fleet simulator (research module)
├── visualize_sitl.py             # Generates output/sitl_flight_replay.html from SITL JSONL logs
├── tests/                        # 209 automated unit, contract, acceptance, and live SITL tests
└── docs/                         # Architecture, simulation setup, data provenance, and limitation notes
```

---

## 4. Demonstrated Mission Workflow

When `sitl_mission_demo.py` runs with `--scenario jaipur_single_uav --mode auto --airspace-change --inject-obstacle --speedup 10`:

1. **External Data & Pre-Mission Assessment**: Queries the live GDACS API (falling back to `output/gdacs_cache_latest.json` if offline) and applies any intersecting flood/cyclone polygon updates to stop suitability before building the initial route. Validates the full route corridor against the DigitalSky OpenAPI 3.0 request schema.
2. **Initial Route & Feasibility Gate**: Builds the 2-stop route around scenario no-fly zones (`nofly_astar.py`) and resolves per-stop modes (`Stop 1 -> hover`, `Stop 2 -> land`). Verifies MTOW (`12.2 kg <= 17.5 kg`), payload (`3.5 kg <= 8.0 kg`), required energy (`136.91 Wh <= 284.16 Wh` usable), cold-chain limits, and delivery windows.
3. **SITL Launch & Mission Upload**: Starts `arducopter` with `--model hexa:med_drone_hexa_16.json` and `sitl_delivery.parm` (`FRAME_CLASS=2`, `HEXAROTOR`), waits for 3D GNSS fix and healthy EKF3, re-validates feasibility against live battery telemetry (`25.039 V`, `100%` SoC), uploads the 13-item mission, verifies item-by-item readback, and uploads `OBSTACLE_TOWER_01` to `AC_PolyFence_loader`.
4. **Leg 1 & C++ Obstacle Avoidance**: Arms in `GUIDED`, enters `AUTO`, climbs to `60 m AGL`, and flies toward Stop 1. `AP_Proximity_SITL` (`PRX1_TYPE=10`) detects the cylindrical obstacle along Leg 1, streams `DISTANCE_SENSOR` and `ADSB_VEHICLE` (`OA_DB`) telemetry, and `AP_OABendyRuler` bends the trajectory laterally around the obstacle (`34.08 m` max lateral deviation in the 2-stop run).
5. **Stop 1 Hover Delivery**: Arrives at `Gopalpura Flooded Relief Camp`, descends to `15 m AGL`, triggers `MAV_CMD_DO_GRIPPER`, waits for `"Gripper Released"` `STATUSTEXT` confirmation (`simulated_hover_gripper_release`, `recipient_handover_verified=False`), and climbs back to `60 m`.
6. **Mid-Flight 4D Restriction & Replanning**: After Stop 1 is served, a 4D restriction (`TFR_POPUP_4D_01`, `0–120 m AGL`) is injected across Leg 2. `MissionRunner` detects the conflict, commands `LOITER`, computes an A* detour from the live EKF position, checks all feasibility gates on the detoured route (`version=2`, `9` items), uploads and verifies readback, and resumes `AUTO`.
7. **Stop 2 Ground Landing, Dwell, and Retakeoff**: Arrives at `Jhalana Elevated Medical Post`, lands (`MAV_CMD_NAV_LAND`), releases the second package (`MAV_CMD_DO_GRIPPER`), completes the 5-second ground dwell (`MAV_CMD_NAV_DELAY=93` under `MIS_OPTIONS=4` `ContinueAfterLand`), records `simulated_ground_gripper_release_after_dwell`, and automatically takes off (`MAV_CMD_NAV_TAKEOFF=22`) to `60 m`.
8. **Return to Launch & Disarm**: Executes `MAV_CMD_NAV_RETURN_TO_LAUNCH` back to SMS Hospital Depot, touches down (`alt_rel_m = 0.019 m`), and disarms (`21.925 V`, `39%` remaining, `9,721.0 mAh` consumed).

---

## 5. Simulation and Validation Evidence

### 5.1 Automated Test Suite (`209` Tests)
Running `unittest discover -s drone_medical_router/tests -p "test_*.py"` in the project's `.venv-sitl` environment executes **209 tests**:
- **In the full local development environment** (with compiled `arducopter` SITL binary, `pymavlink`, `opencv-contrib-python`, and network access to GDACS): **`209` passed, `0` skipped, `0` failed, `0` errors** (`106.7 s` total runtime, including the live hexacopter SITL flight in `tests/test_sitl_integration.py`).
- **Conditional skip guards in minimal/offline environments**:
  - `tests/test_sitl_integration.py` (`1` test): skipped if `pymavlink` or the compiled `arducopter` SITL binary is absent.
  - `tests/test_gdacs_and_provenance.py::TestLiveGDACSExternalAPI.test_live_gdacs_api_fetch_and_cache` (`1` test): skipped if `https://www.gdacs.org/gdacsapi` is unreachable due to no internet/sandbox restrictions.
  - `tests/test_full_autonomy_acceptance.py::TestPrecisionLandingIntegration` (`2` tests): skipped if `cv2.aruco` (`opencv-contrib-python`) is not installed.

### 5.2 Live Hexacopter SITL Logs (`output/`)
| Run Description | Log File | Airframe / Battery | Key Measured Metrics |
| :--- | :--- | :--- | :--- |
| **2-Stop Reference Mission** (`--mode auto --airspace-change --inject-obstacle --speedup 10`) | [`output/sitl_run_20261108_034446.jsonl`](output/sitl_run_20261108_034446.jsonl) | `hexa:med_drone_hexa_16.json`, `HEXAROTOR` (`13`), `25.039 V -> 21.925 V` (`39%` left, `9,721 mAh`) | `delivered: [1, 2]`, `1` in-flight 4D replan (`v2`), `17` proximity frames, `50` `OA_DB`, `max_oa_deviation_m = 34.08 m`, `min_obstacle_surface_clearance_m = 8.98 m`, `success: true` |
| **1-Stop Controlled OA Baseline** (`--max-stops 1 --mode hover --speedup 10`, no obstacle) | [`output/sitl_run_20261108_035408.jsonl`](output/sitl_run_20261108_035408.jsonl) | `hexa:med_drone_hexa_16.json`, `HEXAROTOR` (`13`), `25.039 V -> 22.979 V` (`58%` left, `6,589 mAh`) | `delivered: [1]`, `0` proximity frames, `0` `OA_DB`, `max_oa_deviation_m = 0.0 m`, `planned_km = 6.361 km`, `success: true` |
| **1-Stop Controlled OA Obstacle-Present** (`--max-stops 1 --mode hover --inject-obstacle --speedup 10`) | [`output/sitl_run_20261108_035631.jsonl`](output/sitl_run_20261108_035631.jsonl) | `hexa:med_drone_hexa_16.json`, `HEXAROTOR` (`13`), `25.039 V -> 22.948 V` (`58%` left, `6,699 mAh`) | `delivered: [1]`, `63` proximity frames, `50` `OA_DB`, `max_oa_deviation_m = 38.42 m`, `min_obstacle_surface_clearance_m = 11.64 m`, `+110 mAh` avoidance cost, `success: true` |

### 5.3 Controlled 8-Experiment Validation Suite (`experiments_validation.py`)
Running `python3 experiments_validation.py` generates [`output/experimental_validation_report.json`](output/experimental_validation_report.json) (`8/8` passing). Seven of these (`EXP-01`, `EXP-02`, `EXP-04` through `EXP-08`) are deterministic in-process subsystem experiments using `MissionRunner`, `PayloadManager`, and `_StubLink`; `EXP-03` audits the recorded SITL JSONL logs on disk:
- **`EXP-01` (Route Planning With vs. Without Restricted Areas)**: Full 2-stop round-trip route increases from `8.828 km` (`13` items) unrestricted to `9.002 km` (`14` items, `+0.174 km`, `1` detour waypoint) when a blocking zone is placed across Leg 1 (`Leg 1`: `3.181 km -> 3.355 km`), maintaining `min_measured_clearance_m = 65.97 m >= 50.0 m` (`safety_margin_enforced_m`).
- **`EXP-02` (Mission Feasibility Gates)**: Nominal `3.5 kg` mission passes (`136.91 Wh <= 284.16 Wh`, leg payloads `[3.5, 1.5, 0.0] kg`, cruise powers `[696.3, 525.0, 416.6] W`); deterministically rejects payload overload (`9.0 kg > 8.0 kg`), MTOW overload (`15.70 kg > 15.00 kg`), low battery (`21%` SoC = `3.6 Wh` usable above 20% floor < `136.9 Wh`), missing battery telemetry (`remaining_pct=None`), cold-chain breach (`ETA 9.9 min > 2.0 min`), and deadline breach (`ETA 9.9 min > 2.0 min`).
- **`EXP-03` (SITL Obstacle Avoidance & Log Audit)**: Audits all recorded `output/sitl_run_*.jsonl` files, verifying the hexacopter 2-stop run and 1-stop controlled OA comparison pair.
- **`EXP-04` (4D Restriction & In-Flight Replanning)**: Verifies that a high-altitude zone (`150–300 m AGL`) above the `60 m` cruise altitude does not trigger a replan, while an active `0–120 m AGL` TFR across Leg 2 triggers `AUTO -> LOITER -> AUTO` (`version=2`, remaining distance `5.888 km`).
- **`EXP-05` (Hover vs. Landing vs. Abort Site Suitability)**: Verifies `flooded -> hover` (both `auto` and forced `land`), `clear -> land`, `slope 9.5° > 7.0° -> hover`, and `clearance 6.0 m < 10.0 m -> abort` (`unreachable_stop_ids = [4]`).
- **`EXP-06` (Failed and Missing Actuator Feedback)**: Verifies causal `"Gripper Released"` confirmation (`duplicate_feedback_count = 1`, `recipient_handover_verified = False`), blocks completion when `STATUSTEXT` is missing (`MISSING_ACTUATOR_FEEDBACK -> FAILED`), and transitions to `FAILED` on `"Gripper fault: servo jammed"`.
- **`EXP-07` (External Data Outage & GDACS Semantics)**: Verifies `DynamicAirspaceMonitor` retains active zones on upstream outage (`safe_to_clear_on_outage = False`) and `GDACSMissionImpactAssessor` keeps point alerts as `ADVISORY_REGIONAL_AWARENESS` while downgrading Stop 2 to `hover` under a flood polygon.
- **`EXP-08` (Mission Recovery & Battery Reserve Failsafe)**: Verifies that `18%` SoC (`< 20%` reserve floor) logs `battery_reserve_breached`, commands `RTL`, and marks `mission_finished_on_disarm = True` after touchdown at home.

---

## 6. Current Limitations

1. **No Live DigitalSky Authentication Token**: Without a registered DGCA pilot/operator JWT (`DIGITALSKY_AUTH_TOKEN`), `DigitalSkyIntersectionChecker` validates request payloads against the local OpenAPI 3.0 schema and reports `checked=False, official_clearance=False`.
2. **No Open Real-Time Indian Airspace/NOTAM Polygon Feed**: Official AAI AIM India eAIP data (`aim-india.aai.aero`) is published as HTML/PDF coordinate tables rather than an unauthenticated GeoJSON REST API, and Mappls airspace layers are browser vector tiles only. Consequently, scenario no-fly zones and the mid-flight 4D TFR carry `SYNTHETIC_FIXTURE` provenance.
3. **Headless Simulation Without a 3D Visual World**: Running Gazebo Harmonic (`gz-sim8` + Ogre2 + ROS 2) alongside ArduPilot SITL exceeds the available memory on the 8 GB Apple M3 development host. As a result, `PerceptionPipeline` (YOLOv8n) and `PrecisionLandingController` (ArUco) are component-tested on recorded/synthetic frames and are not active during `sitl_mission_demo.py`.
4. **2D Bisection Raycaster Geometry**: ArduPilot's `AP_Proximity_SITL` (`PRX1_TYPE=10`) casts 8 horizontal rays and performs a binary search on `[0, 200 m]` whose first probe is at `100 m`. Small obstacles whose chord along the ray does not span `100 m` can be missed by the C++ bisection loop unless sized appropriately (such as the `80 m` radius world obstacle used in the demo).
5. **Simulated Gripper Mass vs. Python Cargo Mass**: ArduPilot's C++ `SIM_Gripper_Servo` adds a fixed `1.0 kg` external load in SITL physics and drops it on release, whereas Python's `EnergyEstimator` and `PayloadManager` track the exact per-stop medical package masses (`2.0 kg` at Stop 1, `1.5 kg` at Stop 2).
6. **No Physical Recipient Handover Sensor**: Simulated actuator release confirms only that the autopilot executed the gripper command (`recipient_handover_verified=False`).
7. **Time-Based Cold-Chain Check**: Cold-chain enforcement compares elapsed mission ETA against `cold_chain_limit_minutes`; it does not model internal container thermodynamics or ambient temperature.

---

## 7. Future Work

- Run ArduPilot SITL with Gazebo Harmonic and `ros_gz_bridge` on a 16 GB+ Ubuntu workstation so `PerceptionPipeline` and `PrecisionLandingController` operate in closed loop against rendered 3D camera and depth streams.
- Test authenticated DigitalSky flight-plan validation using a registered pilot/operator token (`DIGITALSKY_AUTH_TOKEN`).
- Ingest digitized eAIP / NOTAM polygons through `GeoJSONFileProvider` and wire a live polling source into `DynamicAirspaceMonitor`.
- Add weather- and wind-field-aware groundspeed and energy adjustments to mission feasibility checks.
- Integrate hardware payload release sensing and flight-controller testing on a physical hexacopter platform.
- Extend the MAVLink flight stack from single-UAV execution to multi-UAV SITL deconfliction.

---

## 8. Setup and Reproduction Instructions

### 8.1 Environment Setup
```bash
# 1. Create or activate a Python 3.10+ virtual environment and install dependencies
pip install -r requirements.txt
pip install -r requirements-sitl.txt

# 2. Verify environment, Python packages, and ArduCopter SITL binary path
python3 check_env.py
```
To run live SITL flights, compile ArduCopter SITL (`./waf configure --board sitl && ./waf copter`) in a sibling `../ardupilot` checkout or set `ARDUPILOT_HOME` / `ARDUCOPTER_SITL_BIN` (see [`docs/simulation_setup.md`](docs/simulation_setup.md)).

Optional environment variable for live DigitalSky HTTP calls:
```bash
export DIGITALSKY_AUTH_TOKEN="<your-dgca-pilot-jwt>"
```

### 8.2 Run the Automated Test Suite and Controlled Experiments
```bash
# Run all 209 automated unit, contract, acceptance, and SITL integration tests
python3 -m unittest discover -s tests -p "test_*.py" -v

# Run the 8-case controlled validation suite (writes output/experimental_validation_report.json)
python3 experiments_validation.py
```

### 8.3 Run the Single-UAV Hexacopter SITL Demonstration
```bash
# Full 2-stop mission (--mode auto, mid-flight 4D TFR replan, C++ obstacle avoidance)
python3 sitl_mission_demo.py \
  --scenario jaipur_single_uav \
  --mode auto \
  --airspace-change \
  --inject-obstacle \
  --speedup 10

# Controlled 1-stop obstacle-avoidance comparison pair
python3 sitl_mission_demo.py --scenario jaipur_single_uav --max-stops 1 --mode hover --speedup 10
python3 sitl_mission_demo.py --scenario jaipur_single_uav --max-stops 1 --mode hover --inject-obstacle --speedup 10

# Regenerate interactive HTML flight replay (output/sitl_flight_replay.html)
python3 visualize_sitl.py
```

---

## 9. Technical Documentation

- [`AGENTS.md`](AGENTS.md) — Engineering rules, data provenance classifications, and delivery mode invariants.
- [`docs/PROJECT_MASTER_CONTEXT.md`](docs/PROJECT_MASTER_CONTEXT.md) — Project scope, system architecture, requirements traceability matrix, and verified evidence.
- [`docs/IMPLEMENTATION_STATUS.md`](docs/IMPLEMENTATION_STATUS.md) — Capability verification ledger and maturity classifications.
- [`docs/architecture.md`](docs/architecture.md) — Data/control boundaries between SITL physics, autopilot telemetry, and planner state.
- [`docs/simulation_setup.md`](docs/simulation_setup.md) — Apple M3 hardware constraints, 6-motor hexacopter SITL config, and `AP_Proximity_SITL` C++ raycasting details.
- [`docs/mission_execution.md`](docs/mission_execution.md) — Feasibility gates, delivery state machine, and `MAV_CMD_NAV_DELAY=93` landing continuation mechanics.
- [`docs/data_sources_and_provenance.md`](docs/data_sources_and_provenance.md) — GDACS, DigitalSky OpenAPI 3.0, Mappls, and Indian aeronautical data source findings.
- [`docs/known_limitations.md`](docs/known_limitations.md) — Detailed technical limitations and scope boundaries.
- [`docs/demo_guide.md`](docs/demo_guide.md) — Step-by-step commands and expected log outputs.
- [`docs/interview_technical_notes.md`](docs/interview_technical_notes.md) & [`docs/PROJECT_INTERVIEW_MASTER.md`](docs/PROJECT_INTERVIEW_MASTER.md) — Technical reference and study notes.

---

## License

MIT License. See [`LICENSE`](LICENSE) for details.
