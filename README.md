# Med-Drone Mission Control

Med-Drone Mission Control is a work-in-progress single-UAV medical-delivery autonomy prototype implemented in Python and integrated over MAVLink v2 with ArduPilot's ArduCopter Software-In-The-Loop (SITL) simulator (`ArduCopter V4.7.0-dev`). The system combines geospatial A* pathfinding around restricted airspace, pre-launch and in-flight feasibility gating, native C++ proximity-sensor obstacle avoidance, and site-dependent hover or ground-landing delivery state machines.

The repository also retains an earlier multi-UAV Capacitated Vehicle Routing Problem (CVRP) solver and discrete-event fleet simulator as supporting operations-research modules.

---

## Operational Problem and Constraints

Flooding and infrastructure damage frequently isolate peripheral clinics and relief camps while creating dynamic hazards for low-altitude aircraft. An autonomous medical-delivery UAV operating under these conditions must satisfy several coupled operational constraints:

- **Dynamic 4D Airspace Restrictions**: Temporary Flight Restrictions (TFRs) bounded in horizontal geometry, altitude (`floor_m_agl` to `ceiling_m_agl`), and time (`valid_from` to `valid_to`) can activate while the aircraft is already airborne.
- **Heterogeneous Delivery Site Conditions**: Ground conditions at a relief camp (standing water, debris, or steep terrain) may make landing unsafe and require a low-altitude hover release instead, or require the stop to be skipped if lateral obstacle clearance is inadequate (`< 10.0 m`).
- **Physical and Clinical Feasibility Gates**: Both the initial flight plan and any mid-flight detour must satisfy Maximum Takeoff Weight (`MTOW <= 17.5 kg`), payload bay capacity (`<= 8.0 kg`), usable battery energy above a `20%` state-of-charge (SoC) reserve floor, time-based cargo cold-chain limits (`cold_chain_limit_minutes`), and hard delivery deadlines (`window_minutes`) before mission upload.

---

## System Capabilities and Integration Maturity

Capabilities in this repository are organized into four explicit maturity tiers:

### 1. Integrated SITL Mission Capabilities (`sitl_mission_demo.py`)
- **Single-UAV Two-Stop Reference Sortie (`JAIPUR_SINGLE_UAV`)**: Plans and executes a closed-loop mission from SMS Hospital Depot (`26.9127, 75.8060`) to Stop 1 (`Gopalpura Flooded Relief Camp`, `2.0 kg`) and Stop 2 (`Jhalana Elevated Medical Post`, `1.5 kg`), followed by Return-to-Launch (`RTL`).
- **Geospatial A* Pathfinding**: Routes each mission leg around polygon no-fly zones with a `50.0 m` horizontal safety buffer, cell-diagonal discretization compensation (`MAX_CELLS = 120`), and bidirectional line-of-sight string-pulling (`nofly_astar.py`, `flightstack/mission.py`).
- **Shared Four-Gate Feasibility Validator**: Uses a unified validator (`MissionRunner._validate_plan_feasibility`) prior to launch and prior to every mid-flight replan upload to enforce MTOW and payload limits, fresh autopilot battery telemetry (`remaining_pct` never defaulted to `100%` when missing or stale), leg-by-leg aerodynamic energy consumption as cargo mass steps down after each delivery (`3.5 kg -> 1.5 kg -> 0.0 kg`), time-based cold-chain limits, and delivery time windows.
- **MAVLink Upload and Item-by-Item Readback Verification**: Uploads missions over MAVLink TCP and reads back every mission item from autopilot memory (`verify_mission_readback`) to verify command IDs, coordinates, and altitudes before arming or resuming `AUTO`.
- **ArduPilot C++ Proximity Raycasting and BendyRuler Avoidance**: Uploads cylindrical world obstacles via the MAVLink Fence Protocol (`MAV_MISSION_TYPE_FENCE = 1`, `FENCE_ENABLE = 0`). ArduCopter's built-in C++ sensor (`AP_Proximity_SITL`, `PRX1_TYPE = 10`) casts 8 horizontal rays against world obstacles, populates `AP_OADatabase` (`ADSB_VEHICLE` `OA_DB` telemetry), and steers around obstacles in `AUTO` mode via `AP_OABendyRuler` (`OA_TYPE = 1`).
- **Mid-Flight 4D Airspace Conflict Detection and Replanning**: Detects when a time- and altitude-bounded restriction (`AirspaceZone`) intersects the remaining path, commands a `LOITER` hold, solves an A* detour from the live EKF position estimate (`include_takeoff = False`), validates all feasibility gates on the candidate plan, uploads and verifies the replacement mission, and resumes `AUTO`.
- **Deterministic Per-Stop Delivery Selection and Landing Continuation**: Resolves each stop deterministically (`resolve_stop_delivery_mode`): executes a `15 m AGL` hover release at flooded Stop 1 (`simulated_hover_gripper_release`) and a full ground landing (`MAV_CMD_NAV_LAND`), servo release (`MAV_CMD_DO_GRIPPER`), 5-second ground dwell (`MAV_CMD_NAV_DELAY = 93` under `MIS_OPTIONS = 4` `ContinueAfterLand`), and retakeoff (`MAV_CMD_NAV_TAKEOFF = 22`) at clear Stop 2 (`simulated_ground_gripper_release_after_dwell`). Actuator acknowledgement waits causally for ArduPilot's `"Gripper Released"` `STATUSTEXT` message.
- **Recovery, Touchdown, and Disarm**: Returns to launch (`MAV_CMD_NAV_RETURN_TO_LAUNCH`), confirms touchdown (`alt_rel_m < 0.5 m`), and verifies motor disarm. Includes an active `20%` SoC reserve failsafe that triggers immediate `RTL`.

### 2. Live External Data and Regulatory API Contract Validation
- **GDACS Disaster Feed (`gdacs_provider.py`)**: Queries live flood (`FL`), earthquake (`EQ`), and tropical cyclone (`TC`) GeoJSON alerts and polygon footprints from `https://www.gdacs.org/gdacsapi`, deduplicates episodes, filters expired alerts, caches snapshots to `output/gdacs_cache_latest.json` (`stale = True` on network outage), and attaches mandatory GDACS attribution. Point-only alerts are classified as regional awareness (`ADVISORY_REGIONAL_AWARENESS`) rather than converted into synthetic circular no-fly zones; validated flood or cyclone polygons intersecting a planned landing site downgrade that stop from `land` to `hover`.
- **DigitalSky OpenAPI 3.0 Contract (`airspace_provider.py`)**: Constructs and validates full-route corridor payloads against the official DigitalSky specification (`docs/external_specs/digitalsky_interactive_maps_api_v1.0.yml`, `POST /flightplan/validate/airspace`, `X-API-VERSION: 1`, `maxAltitude <= 400 ft`) and parses HTTP 400 HAL `ValidationError` and HTTP 500 `Error` payloads. Live HTTP requests require a registered DGCA pilot/operator JWT (`DIGITALSKY_AUTH_TOKEN`); when unauthenticated, the validator reports `checked = False`, `official_clearance = False`, and `status = "UNAUTHORISED_NO_PILOT_TOKEN"`.
- **Mappls Airspace Layer Classification**: Classified as `MAP_VISUALIZATION_ONLY` because the Mappls Web SDK exposes browser vector-tile overlays without a polygon-export REST API.

### 3. Component-Tested Subsystems (Not Active in the Headless SITL Mission)
- **YOLOv8n Perception and Metric-Depth Gating (`flightstack/perception_pipeline.py`, `yolo_safety.py`)**: Benchmarked on recorded and synthetic frames using Apple Silicon MPS acceleration (`perception/evaluate_model.py`). Enforces strict separation between 2D monocular bearing rays and 3D metric coordinates, rejecting depthless observations in `ObstacleTracker`. Not active in `sitl_mission_demo.py` because headless ArduPilot SITL does not render a 3D visual world.
- **3D CPA Obstacle Tracker (`flightstack/obstacle_tracker.py`)**: Closest-Point-of-Approach (`t_cpa`, `d_cpa`) conflict prediction and 72-bin MAVLink `OBSTACLE_DISTANCE` encoding, verified in unit tests (`tests/test_vehicle_and_perception.py`, `tests/test_flightstack.py`).
- **ArUco Precision Landing Controller (`flightstack/precision_landing.py`)**: OpenCV ArUco marker pose estimation, optical-to-body NED coordinate transformation, and MAVLink `LANDING_TARGET` (#149) message generation with low-altitude target-loss abort (`LOITER`).
- **Dynamic Airspace Polling Monitor (`DynamicAirspaceMonitor` in `airspace_provider.py`)**: Tracks 4D zone additions, expirations, and fail-safe upstream outages (`safe_to_clear_restrictions = False`) in unit tests and `EXP-07`. In `sitl_mission_demo.py`, the mid-flight 4D restriction (`TFR_POPUP_4D_01`) is injected via a deterministic callback after Stop 1.
- **Multi-UAV CVRP Solver and Fleet Simulator (`vrp_scratch.py`, `vrp_ortools.py`, `simulate_fleet.py`, `replan.py`)**: Standalone operations-research modules for multi-depot and multi-UAV routing comparisons (`main.py`, `run_simulation.py`).

### 4. Deferred Scope
- **Meteorological and Wind-Field Integration**: Live weather ingestion and wind-vector drag compensation are deferred to future work.
- **Physical Hardware and Custody Verification**: No physical flights or hardware recipient handover sensors are claimed (`recipient_handover_verified = False`).

---

## System Architecture and Subsystems

```text
drone_medical_router/
├── sitl_mission_demo.py          # Single-UAV ArduCopter SITL mission entry point
├── experiments_validation.py     # 8-case validation suite -> output/experimental_validation_report.json
├── config.py                     # Scenarios (JAIPUR_SINGLE_UAV, JAIPUR_DISASTER, CHENNAI_FLOOD), specs, locations
├── geo_utils.py                  # Haversine distance, point-in-polygon, segment-polygon clearance geometry
├── nofly_astar.py                # 8-connected grid A* router with 50 m polygon buffer and string-pulling
├── airspace_provider.py          # SourceClassification, 4D AirspaceZone, DigitalSky OpenAPI client, DynamicAirspaceMonitor
├── gdacs_provider.py             # Live/cached GDACS client and GDACSMissionImpactAssessor
├── flightstack/
│   ├── sitl.py                   # ArduCopter SITL launcher (defaults to --model hexa:med_drone_hexa_16.json)
│   ├── mavlink_link.py           # MAVLink TCP client, mission upload/readback, fence obstacle upload
│   ├── telemetry.py              # Timestamped MAVLink state (position, EKF, GNSS, 6S battery, OA_DB, DISTANCE_SENSOR)
│   ├── mission.py                # MissionPlan builder, 4D polyline validator, QGC WPL 110 exporter
│   ├── mission_runner.py         # Closed-loop state machine, shared feasibility validator, replanning, RTL failsafe
│   ├── payload.py                # SiteSuitability, resolve_stop_delivery_mode, causal PayloadManager state machine
│   ├── vehicle_model.py          # Rotorcraft momentum-theory hover/cruise power, MTOW, and leg-by-leg energy model
│   ├── perception_pipeline.py    # Component-tested YOLOv8n detector and 2D/3D spatial projector
│   ├── obstacle_tracker.py       # Component-tested 3D CPA tracker and OBSTACLE_DISTANCE encoder
│   ├── precision_landing.py      # Component-tested ArUco detector and MAVLink LANDING_TARGET controller
│   └── params/
│       ├── med_drone_hexa_16.json # 6-motor hexacopter physics & 6S 16 Ah LiPo parameters for ArduPilot SIM_Frame
│       └── sitl_delivery.parm    # ArduCopter parameters (FRAME_CLASS=2, PRX1_TYPE=10, OA_TYPE=1, MIS_OPTIONS=4)
├── vrp_scratch.py                # Multi-heuristic CVRP solver (Nearest Neighbor, Urgency-NN, Clarke-Wright + 2-opt/Or-opt)
├── simulate_fleet.py             # Discrete-event multi-drone fleet simulator (supporting research module)
├── visualize_sitl.py             # Generates output/sitl_flight_replay.html from SITL JSONL logs
├── tests/                        # 209 automated unit, contract, acceptance, and live SITL integration tests
└── docs/                         # Technical documentation and Architecture Decision Records (ADRs)
```

---

## Mission Execution Workflow

Running `sitl_mission_demo.py` with `--scenario jaipur_single_uav --mode auto --airspace-change --inject-obstacle --speedup 10` executes the following closed-loop sequence:

1. **External Hazard Ingestion & Pre-Mission Assessment**: Queries the live GDACS API (falling back to `output/gdacs_cache_latest.json` if offline) and applies any intersecting flood or cyclone polygon updates to stop suitability before route generation. Validates the full route corridor against the DigitalSky OpenAPI 3.0 request schema.
2. **Initial Route Generation & Feasibility Gate**: Computes the 2-stop route around active no-fly zones (`nofly_astar.py`) and resolves per-stop delivery modes (`Stop 1 -> hover`, `Stop 2 -> land`). Verifies MTOW (`12.2 kg <= 17.5 kg`), payload (`3.5 kg <= 8.0 kg`), required energy (`136.91 Wh <= 284.16 Wh` usable), cold-chain limits, and delivery windows.
3. **SITL Launch & Readback-Verified Upload**: Starts `arducopter` with `--model hexa:med_drone_hexa_16.json` and `sitl_delivery.parm` (`FRAME_CLASS = 2`, `HEXAROTOR`), waits for a 3D GNSS fix and healthy EKF3, re-validates feasibility against live battery telemetry (`25.039 V`, `100%` SoC), uploads the 13-item mission, verifies item-by-item readback, and uploads `OBSTACLE_TOWER_01` to `AC_PolyFence_loader`.
4. **Leg 1 & C++ Obstacle Avoidance**: Arms in `GUIDED`, transitions to `AUTO`, climbs to `60 m AGL`, and flies toward Stop 1. `AP_Proximity_SITL` (`PRX1_TYPE = 10`) detects the cylindrical obstacle along Leg 1, streams `DISTANCE_SENSOR` and `ADSB_VEHICLE` (`OA_DB`) telemetry, and `AP_OABendyRuler` steers laterally around the obstacle (`34.08 m` peak lateral deviation in the 2-stop run).
5. **Stop 1 Hover Delivery**: Arrives at `Gopalpura Flooded Relief Camp`, descends to `15 m AGL`, triggers `MAV_CMD_DO_GRIPPER`, waits causally for `"Gripper Released"` `STATUSTEXT` confirmation (`simulated_hover_gripper_release`, `recipient_handover_verified = False`), and climbs back to `60 m AGL`.
6. **Mid-Flight 4D Restriction & Replanning**: After Stop 1 is served, a 4D restriction (`TFR_POPUP_4D_01`, `0–120 m AGL`) activates across Leg 2. `MissionRunner` detects the conflict, commands `LOITER`, computes an A* detour from the live EKF position, verifies all four feasibility gates on the candidate route (`version = 2`, `9` items), uploads and verifies readback, and resumes `AUTO`.
7. **Stop 2 Ground Landing, Dwell, and Retakeoff**: Arrives at `Jhalana Elevated Medical Post`, lands (`MAV_CMD_NAV_LAND`), releases the second package (`MAV_CMD_DO_GRIPPER`), completes the 5-second ground dwell (`MAV_CMD_NAV_DELAY = 93` under `MIS_OPTIONS = 4` `ContinueAfterLand`), records `simulated_ground_gripper_release_after_dwell`, and automatically takes off (`MAV_CMD_NAV_TAKEOFF = 22`) to `60 m AGL`.
8. **Return to Launch & Disarm**: Executes `MAV_CMD_NAV_RETURN_TO_LAUNCH` to SMS Hospital Depot, confirms touchdown (`alt_rel_m = 0.019 m`), and disarms (`21.925 V`, `39%` remaining, `9,721.0 mAh` consumed).

---

## Simulation Environment and Vehicle Configuration

To maintain consistency across the Python energy model (`flightstack/vehicle_model.py`), ArduCopter's control mixer (`AP_MotorsMatrix`), and ArduPilot's SITL physics engine (`SIM_Frame.cpp` and `SIM_Battery.cpp`), all three layers are configured as a 6-motor hexacopter (`MedDrone-Hexa-16`) powered by a 6S 16 Ah LiPo battery:

- **Physics Model (`flightstack/params/med_drone_hexa_16.json`)**: Loaded via `--model hexa:med_drone_hexa_16.json`, defining `num_motors = 6`, `disc_area = 1.4255 m²`, `mass = 11.2 kg` (`12.2 kg` takeoff gross mass including the `1.0 kg` `SIM_Gripper_Servo` external load), `maxVoltage = 25.2 V`, `refVoltage = 22.2 V`, `battCapacityAh = 16.0 Ah`, and `refBatRes = 0.012 Ω`.
- **Autopilot Parameters (`flightstack/params/sitl_delivery.parm`)**: Sets `FRAME_CLASS = 2` (`MAV_TYPE_HEXAROTOR = 13`), `FRAME_TYPE = 0`, `SIM_BATT_VOLTAGE = 25.04`, `SIM_BATT_CAP_AH = 16`, `PRX1_TYPE = 10` (`AP_Proximity_SITL`), `OA_TYPE = 1` (`AP_OABendyRuler`), and `MIS_OPTIONS = 4` (`ContinueAfterLand`).
- **Host Hardware Footprint**: Native `arm64` `arducopter` SITL executes on an 8 GB unified-memory Apple M3 host using `< 45 MB` RSS at `10x` real-time speedup, avoiding the multi-gigabyte memory overhead of running Gazebo Harmonic and ROS 2 concurrently.

---

## Validation Methodology and Experimental Results

### 1. Automated Test Suite (`209` Tests)
Running `unittest discover -s tests -p "test_*.py"` in the configured `.venv-sitl` environment executes **209 automated tests**:
- **Full local development environment** (with compiled `arducopter` SITL binary, `pymavlink`, `opencv-contrib-python`, and network access to GDACS): **`209` passed, `0` skipped, `0` failed, `0` errors** (`106.7 s` total runtime, including the live hexacopter SITL flight in `tests/test_sitl_integration.py`).
- **Test breakdown by module (`94 + 41 + 33 + 24 + 16 + 1 = 209`)**:
  - `tests/test_core_logic.py` (`94` tests): Spherical geometry, A* clearance, CVRP heuristics, 2-opt/Or-opt, cold-chain and deadline constraints, and fleet simulation.
  - `tests/test_flightstack.py` (`41` tests): Telemetry freshness, 4D airspace validation, MAVLink upload/readback, and native proximity sensor configuration.
  - `tests/test_full_autonomy_acceptance.py` (`33` tests): Pre-mission and in-flight feasibility gates, deterministic delivery mode selection, causal gripper feedback, ground dwell, perception depth gating, and ArUco precision landing.
  - `tests/test_vehicle_and_perception.py` (`24` tests): Momentum-theory hover/cruise power, leg-by-leg payload mass step-down, 6S battery parameters, and 3D CPA obstacle tracking.
  - `tests/test_gdacs_and_provenance.py` (`16` tests): Live and fixture GDACS ingestion, episode deduplication, DigitalSky OpenAPI 3.0 request/error schemas, and `DynamicAirspaceMonitor` outage handling.
  - `tests/test_sitl_integration.py` (`1` test): Live ArduCopter SITL takeoff, waypoint navigation, mission readback, and stale-telemetry detection (skipped in CI when the `arducopter` SITL binary is absent).

### 2. Recorded Hexacopter SITL Flight Logs (`output/`)
| Mission Configuration | Log Artifact | Airframe & Battery Telemetry | Verified Flight Metrics |
| :--- | :--- | :--- | :--- |
| **2-Stop Reference Mission** (`--mode auto --airspace-change --inject-obstacle --speedup 10`) | [`output/sitl_run_20261108_034446.jsonl`](output/sitl_run_20261108_034446.jsonl) | `hexa:med_drone_hexa_16.json`, `HEXAROTOR` (`13`), `25.039 V -> 21.925 V` (`39%` left, `9,721 mAh`) | `delivered: [1, 2]`, `1` in-flight 4D replan (`v2`), `17` proximity frames, `50` `OA_DB`, `max_oa_deviation_m = 34.08 m`, `min_obstacle_surface_clearance_m = 8.98 m`, `success: true` |
| **1-Stop Controlled OA Baseline** (`--max-stops 1 --mode hover --speedup 10`, no obstacle) | [`output/sitl_run_20261108_035408.jsonl`](output/sitl_run_20261108_035408.jsonl) | `hexa:med_drone_hexa_16.json`, `HEXAROTOR` (`13`), `25.039 V -> 22.979 V` (`58%` left, `6,589 mAh`) | `delivered: [1]`, `0` proximity frames, `0` `OA_DB`, `max_oa_deviation_m = 0.0 m`, `planned_km = 6.361 km`, `success: true` |
| **1-Stop Controlled OA Obstacle-Present** (`--max-stops 1 --mode hover --inject-obstacle --speedup 10`) | [`output/sitl_run_20261108_035631.jsonl`](output/sitl_run_20261108_035631.jsonl) | `hexa:med_drone_hexa_16.json`, `HEXAROTOR` (`13`), `25.039 V -> 22.948 V` (`58%` left, `6,699 mAh`) | `delivered: [1]`, `63` proximity frames, `50` `OA_DB`, `max_oa_deviation_m = 38.42 m`, `min_obstacle_surface_clearance_m = 11.64 m`, `+110 mAh` avoidance cost, `success: true` |

### 3. Controlled 8-Experiment Validation Suite (`experiments_validation.py`)
Running `python3 experiments_validation.py` generates [`output/experimental_validation_report.json`](output/experimental_validation_report.json) (`8/8` passing). Seven experiments (`EXP-01`, `EXP-02`, `EXP-04` through `EXP-08`) run deterministic in-process subsystem checks against `MissionRunner`, `PayloadManager`, and `_StubLink`; `EXP-03` audits the recorded SITL JSONL logs on disk:
- **`EXP-01` (Route Planning With vs. Without Restricted Areas)**: Full 2-stop round-trip route increases from `8.828 km` (`13` items) unrestricted to `9.002 km` (`14` items, `+0.174 km`, `1` detour waypoint) when a blocking zone is placed across Leg 1 (`Leg 1`: `3.181 km -> 3.355 km`), maintaining `min_measured_clearance_m = 65.97 m >= 50.0 m` (`safety_margin_enforced_m`).
- **`EXP-02` (Mission Feasibility Gates)**: Nominal `3.5 kg` mission passes on the base `17.5 kg` MTOW vehicle profile (`136.91 Wh <= 284.16 Wh`, leg payloads `[3.5, 1.5, 0.0] kg`, cruise powers `[696.3, 525.0, 416.6] W`); deterministically rejects payload overload (`9.0 kg > 8.0 kg`), a deliberately derated `15.00 kg` MTOW test profile (`15.70 kg > 15.00 kg`; base vehicle model MTOW remains `17.5 kg`), low battery (`21%` SoC = `3.6 Wh` usable above 20% floor < `136.9 Wh`), missing battery telemetry (`remaining_pct = None`), cold-chain breach (`ETA 9.9 min > 2.0 min`), and deadline breach (`ETA 9.9 min > 2.0 min`).
- **`EXP-03` (SITL Obstacle Avoidance & Log Audit)**: Audits the 3 recorded hexacopter `output/sitl_run_*.jsonl` files individually, requiring all three checks to pass: (1) the 2-stop reference mission (`sitl_run_20261108_034446.jsonl`: `delivered == [1, 2]`, `{"1": "hover", "2": "land"}`, `max_oa_deviation_m = 34.08 m`), (2) the 1-stop baseline (`sitl_run_20261108_035408.jsonl`: `delivered == [1]`, `max_oa_deviation_m = 0.0 m`), and (3) the 1-stop obstacle-present run (`sitl_run_20261108_035631.jsonl`: `delivered == [1]`, `max_oa_deviation_m = 38.42 m`).
- **`EXP-04` (4D Restriction & In-Flight Replanning)**: Verifies that a high-altitude zone (`150–300 m AGL`) above the `60 m` cruise altitude does not trigger a replan, whereas an active `0–120 m AGL` TFR across Leg 2 triggers `AUTO -> LOITER -> AUTO` (`version = 2`, remaining distance `5.888 km`).
- **`EXP-05` (Hover vs. Landing vs. Abort Site Suitability)**: Verifies `flooded -> hover` (both `auto` and explicit `land`), `clear -> land`, `slope 9.5° > 7.0° -> hover`, and `clearance 6.0 m < 10.0 m -> abort` (`unreachable_stop_ids = [4]`).
- **`EXP-06` (Failed and Missing Actuator Feedback)**: Verifies causal `"Gripper Released"` confirmation (`duplicate_feedback_count = 1`, `recipient_handover_verified = False`), blocks completion when `STATUSTEXT` is missing (`MISSING_ACTUATOR_FEEDBACK -> FAILED`), and transitions to `FAILED` on `"Gripper fault: servo jammed"`.
- **`EXP-07` (External Data Outage & GDACS Semantics)**: Verifies `DynamicAirspaceMonitor` retains active zones on upstream outage (`safe_to_clear_on_outage = False`) and `GDACSMissionImpactAssessor` keeps point alerts as `ADVISORY_REGIONAL_AWARENESS` while downgrading Stop 2 to `hover` under a flood polygon.
- **`EXP-08` (Mission Recovery & Battery Reserve Failsafe)**: Verifies that `18%` SoC (`< 20%` reserve floor) logs `battery_reserve_breached`, commands `RTL`, and marks `mission_finished_on_disarm = True` after touchdown at home.

---

## Technical Limitations and Scope Boundaries

1. **Unauthenticated DigitalSky Execution**: Without a registered DGCA pilot/operator JWT (`DIGITALSKY_AUTH_TOKEN`), `DigitalSkyIntersectionChecker` validates request payloads against the local OpenAPI 3.0 schema and reports `checked = False, official_clearance = False`.
2. **No Open Real-Time Indian Airspace/NOTAM Polygon Feed**: Official AAI AIM India eAIP data (`aim-india.aai.aero`) is published as HTML/PDF coordinate tables rather than an unauthenticated GeoJSON REST API, and Mappls airspace layers provide browser vector tiles only. Scenario no-fly zones and the mid-flight 4D TFR therefore carry `SYNTHETIC_FIXTURE` provenance.
3. **Headless Simulation Without a 3D Visual World**: Running Gazebo Harmonic (`gz-sim8` + Ogre2 + ROS 2) alongside ArduPilot SITL exceeds available memory on an 8 GB unified-memory Apple M3 host. Consequently, `PerceptionPipeline` (YOLOv8n) and `PrecisionLandingController` (ArUco) are component-tested on recorded and synthetic frames and are not active during `sitl_mission_demo.py`.
4. **2D Bisection Raycaster Geometry**: ArduPilot's `AP_Proximity_SITL` (`PRX1_TYPE = 10`) casts 8 horizontal rays and performs a binary search on `[0, 200 m]` whose first probe is at `100 m`. Obstacles whose chord along the ray does not span `100 m` can be missed by the C++ bisection loop unless sized appropriately (such as the `80 m` radius world obstacle used in the demo).
5. **Simulated Gripper Mass vs. Python Cargo Mass**: ArduPilot's C++ `SIM_Gripper_Servo` attaches a single `1.0 kg` external load in SITL physics and drops it on first release, whereas Python's `EnergyEstimator` and `PayloadManager` track the per-stop package masses (`2.0 kg` at Stop 1, `1.5 kg` at Stop 2).
6. **No Physical Recipient Handover Sensor**: Simulated actuator release confirms only that the autopilot executed the servo command (`recipient_handover_verified = False`).
7. **Time-Based Cold-Chain Check**: Cold-chain enforcement compares elapsed mission ETA against `cold_chain_limit_minutes`; it does not model internal container thermodynamics or ambient temperature.

---

## Installation and Reproducibility

### 1. Environment Setup
```bash
# Install test, core, and SITL Python dependencies in a Python 3.10+ virtual environment
pip install -r requirements-test.txt
pip install -r requirements.txt
pip install -r requirements-sitl.txt

# Verify host environment, Python packages, and ArduCopter SITL binary path
python3 check_env.py
```
To run live SITL flights, compile ArduCopter SITL (`./waf configure --board sitl && ./waf copter`) in a sibling `../ardupilot` checkout or set `ARDUPILOT_HOME` / `ARDUCOPTER_SITL_BIN` (see [`docs/simulation_setup.md`](docs/simulation_setup.md)).

### 2. Automated Test Suite and Validation Experiments
```bash
# Run all 209 automated unit, contract, acceptance, and SITL integration tests
python3 -m unittest discover -s tests -p "test_*.py" -v

# Run the 8-case controlled validation suite (writes output/experimental_validation_report.json)
python3 experiments_validation.py
```

### 3. Single-UAV Hexacopter SITL Demonstration
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

## Technical Documentation

- [`docs/architecture.md`](docs/architecture.md) — Subsystem architecture, data/control boundaries, and strategic vs. tactical avoidance layers.
- [`docs/mission_execution.md`](docs/mission_execution.md) — Mission lifecycle, four-gate feasibility validation, delivery state machine, and `MAV_CMD_NAV_DELAY = 93` landing continuation mechanics.
- [`docs/simulation_setup.md`](docs/simulation_setup.md) — ArduPilot SITL compilation, hardware constraints, coupled 6-motor hexacopter & 6S LiPo configuration, and `AP_Proximity_SITL` C++ raycasting details.
- [`docs/data_sources_and_provenance.md`](docs/data_sources_and_provenance.md) — `SourceClassification` taxonomy, GDACS API contract, DigitalSky OpenAPI 3.0 validation, Mappls visualization scope, and Indian aeronautical data findings.
- [`docs/IMPLEMENTATION_STATUS.md`](docs/IMPLEMENTATION_STATUS.md) — Capability verification matrix and subsystem maturity classifications.
- [`docs/known_limitations.md`](docs/known_limitations.md) — Detailed technical limitations and scope boundaries.
- [`docs/demo_guide.md`](docs/demo_guide.md) — Step-by-step reproduction commands and expected log outputs.
- [`docs/PERCEPTION_TRAINING.md`](docs/PERCEPTION_TRAINING.md) — Dataset schema and training specification for the component-tested YOLOv8 perception module.
- [`docs/engineering_decisions/`](docs/engineering_decisions/) — Architecture Decision Records (`ADR-001` through `ADR-004`).

---

## License

MIT License. See [`LICENSE`](LICENSE) for details.
