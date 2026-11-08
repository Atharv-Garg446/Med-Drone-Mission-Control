# Med-Drone Mission Control — Agent Handoff Document

**Last Updated**: 2026-11-08 (Increment 3 & Final Portfolio Release Complete)  
**Workspace**: `/Users/mahigarg/Documents/Drone Shit/drone_medical_router`  
**Python Environment**: `/Users/mahigarg/Documents/Drone Shit/.venv-sitl/bin/python` (Python 3.10.13)  
**ArduPilot SITL Binary**: `/Users/mahigarg/Documents/Drone Shit/ardupilot/build/sitl/bin/arducopter` (ArduCopter V4.7.0-dev `arm64`, launched with `--model hexa:med_drone_hexa_16.json`, `FRAME_CLASS=2`, `MAV_TYPE_HEXAROTOR=13`)
**Portfolio Release Archive**: `/Users/mahigarg/Documents/Drone Shit/drone_medical_router_portfolio_release_20261108.zip`

---

## 1. Current State Summary

### 1.1 Original Engineering Priorities (COMPLETE)

All engineering priorities (`Priority 0` through `Priority 5`) from the Master Continuation Brief are implemented and verified:

1. **Priority 0 — Deterministic Per-Stop Delivery Decision Architecture**: `resolve_stop_delivery_mode` in `flightstack/payload.py`, `--mode auto` default. Stop 1 → `hover`, Stop 2 → `land`, unsafe → `abort`.
2. **Priority 1 — DigitalSky OpenAPI 3.0 & 4D Dynamic Airspace Monitoring**: Full contract client, error parsing, `DynamicAirspaceMonitor` (interface-ready, unit-tested, NOT active in demo).
3. **Priority 2 — Operational GDACS Pipeline**: `GDACSMissionImpactAssessor`, `ADVISORY_REGIONAL_AWARENESS` vs `DOWNGRADE_LAND_TO_HOVER`.
4. **Priority 3, 4, 5 — Replan Validation, Perception Hook, SITL Comparisons**: All implemented and verified.

### 1.2 Stabilisation Work (COMPLETE)

The following corrections were applied in the stabilisation pass:

**A. Payload Lifecycle (`flightstack/payload.py`, `flightstack/mission_runner.py`)**
- Causal gripper feedback: `acknowledge_actuator()` waits for ArduPilot `STATUSTEXT: "Gripper Released"` — never auto-acknowledges.
- Missing/failed acknowledgement → `FAILED` (not silently completed).
- `duplicate_feedback_count` tracked per-package and per-manager.
- `release_provenance` set at acknowledgement time: `simulated_hover_gripper_release` or `simulated_ground_gripper_release_after_dwell`.
- Ground dwell must complete before stop is marked served.
- `recipient_handover_verified=False` always in simulation.
- `failed_deliveries` tracked in `MissionRunnerState`.
- `HOVER_ROPE_RELEASE` retained as legacy enum alias only; all new code uses gripper terminology.

**B. Initial Mission Feasibility Gate (`flightstack/mission_runner.py`)**
- New shared `_validate_plan_feasibility()` — used by both `validate_initial_mission()` and `_validate_candidate_replan()`.
- `validate_initial_mission()` blocks `launch()` on failure.
- Checks: MTOW, payload capacity, battery telemetry (presence/freshness/energy), time-based cold-chain limits (`cold_chain_limit_minutes`), delivery deadlines, full-route DigitalSky polyline.
- Cold-chain check labeled `time_based_cold_chain_limit_minutes` (not "thermal model").

**C. Pre-Mission GDACS Application (`flightstack/mission_runner.py`, `sitl_mission_demo.py`)**
- `apply_initial_gdacs_impact()` evaluates and idempotently applies GDACS impact before mission upload.
- GDACS evaluation happens BEFORE `build_delivery_mission()` in `sitl_mission_demo.py`.
- Repeated application is idempotent (deduplicates zone_ids, compares stop suitability).

**D. DigitalSky Full-Route Validation (`sitl_mission_demo.py`)**
- DigitalSky validation now covers `full_route_polyline(home)` not just `leg_paths[0]`.

**E. Documentation Corrections**
- `simulated_hover_winch_release` → `simulated_hover_gripper_release` throughout.
- `proven_obstacle_avoidance` → `measured_oa_lateral_deviation_observed` throughout.
- `HOVER_ROPE_RELEASE` / `rope/winch` → gripper terminology in docs.
- `cold-chain thermal limits` → `time-based cold-chain limits` throughout.
- `DynamicAirspaceMonitor` marked NOT active in demo (interface exists, no live provider wired).
- Perception pipeline and precision landing marked NOT active in SITL demo.
- DigitalSky live integration marked BLOCKED on DGCA pilot JWT.
- Test count updated to 216+ (was 208).

### 1.3 Vehicle, SITL Physics & Energy Model Integrity (Increment 1 — COMPLETE)
- **Root Cause of Prior Failed Run (`sitl_run_20261107_213336.jsonl`) Resolved**: Setting `FRAME_CLASS=2` (6 motors in `AP_MotorsMatrix`) while leaving `SitlProcess` on `--model +` (4 motors in `SIM_Frame.cpp`) caused motors 5 & 6 to be ignored by the physics simulator and prevented takeoff.
- **Unified 6-Motor Hexacopter Physics (`flightstack/params/med_drone_hexa_16.json`, `flightstack/sitl.py`, `flightstack/params/sitl_delivery.parm`)**:
  - `SitlProcess` now defaults to `model="hexa:med_drone_hexa_16.json"` and automatically stages `med_drone_hexa_16.json` into `self.workdir` for `AP::FS().stat()`.
  - `med_drone_hexa_16.json` configures 6 motors (`num_motors: 6`), `disc_area: 1.4255` m², `mass: 11.2` kg (`12.2 kg` takeoff gross mass with `SIM_GRPS_ENABLE=1` external load, dropping `1.0 kg` on release), and 6S 16 Ah LiPo battery parameters (`maxVoltage: 25.2`, `refVoltage: 22.2`, `battCapacityAh: 16.0`, `refBatRes: 0.012`).
  - `TelemetryState._on_HEARTBEAT` records `mav_type_id=13` (`"HEXAROTOR"`) from autopilot heartbeats while ignoring GCS (`type=6`).
- **Energy Feasibility Corrections (`flightstack/vehicle_model.py`, `flightstack/mission_runner.py`)**:
  - Fixed double-reserve battery bug in `_validate_plan_feasibility` using `usable_span_pct = max(1.0, 100.0 - self.battery_reserve_pct)`.
  - Updated `EnergyEstimator.evaluate_route` aggregate overload to accept `leg_km_list`, step down carried payload mass after each delivery stop (`3.5 kg -> 1.5 kg -> 0.0 kg` on RTL), and catch MTOW/payload violations cleanly (`mtow_ok=False`).

### 1.4 Test & Live Hexacopter SITL Evidence (Increment 2 — COMPLETE)

- **Full test suite**: 219 tests, 0 failures, 1 skipped (exit code 0, including live hexacopter `test_sitl_integration.py`).
- **Controlled Experimental Validation Suite (`experiments_validation.py`)**: 8/8 controlled experiments passed (`output/experimental_validation_report.json`).
- **Complete 2-Stop End-to-End Hexacopter SITL Reference Mission**:
  - Log: [`output/sitl_run_20261108_034446.jsonl`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/output/sitl_run_20261108_034446.jsonl)
  - Verified: `model="hexa:med_drone_hexa_16.json"`, `mav_type="HEXAROTOR"` (`13`), 6S battery discharge `25.039 V -> 21.925 V` (`39%` remaining, `9,721.0 mAh` consumed), live GDACS pre-mission assessment (`OK_LIVE`, `100` active events), DigitalSky full-route validation (`4` waypoints), C++ `AP_Proximity_SITL` + `AP_OABendyRuler` avoidance (`17` frames, `50` `OA_DB` items, `max_oa_deviation_m = 34.08 m`), Stop 1 `hover` (`simulated_hover_gripper_release`), mid-flight `TFR_POPUP_4D_01` replan, Stop 2 `land` + `MAV_CMD_NAV_DELAY` (`93`) 5 s ground dwell + retakeoff (`simulated_ground_gripper_release_after_dwell`, `recipient_handover_verified=False`), RTL touchdown (`alt_rel_m = 0.019 m`) & disarm (`success: true`, `delivered: [1, 2]`, `failed_deliveries: []`).
- **Controlled 1-Stop Hexacopter Obstacle-Avoidance Comparison Pair** (`--max-stops 1 --mode hover --speedup 10`, differing only in `--inject-obstacle`):
  - Baseline (`without --inject-obstacle`): [`output/sitl_run_20261108_035408.jsonl`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/output/sitl_run_20261108_035408.jsonl) — `mav_type="HEXAROTOR"`, `0` proximity frames, `max_oa_deviation_m = 0.0 m`, `6,589.0 mAh` consumed (`22.979 V`, `58%` left), `delivered: [1]`, `success: true`.
  - Obstacle-Present (`with --inject-obstacle`): [`output/sitl_run_20261108_035631.jsonl`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/output/sitl_run_20261108_035631.jsonl) — `mav_type="HEXAROTOR"`, `63` proximity frames, `50` `OA_DB` items, `max_oa_deviation_m = 38.42 m` (`min_obstacle_surface_clearance_m = 11.64 m`), `6,699.0 mAh` consumed (`+110.0 mAh` avoidance energy cost, `22.948 V`, `58%` left), `delivered: [1]`, `success: true`.
- **Previous SITL logs preserved** in `output/` (including historical quad runs and the diagnostic `sitl_run_20261107_213336.jsonl` motor-mismatch log).

---

## 2. Key Documents to Read First

1. [`AGENTS.md`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/AGENTS.md) — Non-negotiable engineering rules.
2. [`docs/PROJECT_MASTER_CONTEXT.md`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/docs/PROJECT_MASTER_CONTEXT.md) — Authoritative project state and Requirements Traceability Matrix.
3. [`docs/PROJECT_INTERVIEW_MASTER.md`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/docs/PROJECT_INTERVIEW_MASTER.md) — Complete technical & interview reference.
4. This document (`docs/AGENT_HANDOFF.md`).

---

## 3. How to Run Verification

```bash
# Full test suite (unit + contract + acceptance + SITL integration, ~100s)
cd "/Users/mahigarg/Documents/Drone Shit"
PYTHONPATH=drone_medical_router "/Users/mahigarg/Documents/Drone Shit/.venv-sitl/bin/python" \
  -m unittest discover -s drone_medical_router/tests -p "test_*.py"

# Full 2-stop SITL reference mission (--mode auto, 4D TFR replan, C++ OA)
PYTHONPATH=drone_medical_router "/Users/mahigarg/Documents/Drone Shit/.venv-sitl/bin/python" \
  drone_medical_router/sitl_mission_demo.py \
  --scenario jaipur_single_uav --mode auto --airspace-change --inject-obstacle --speedup 10
```

---

## 4. Terminology Rules (Stabilisation)

| Old Term | Corrected Term | Reason |
|---|---|---|
| `simulated_hover_winch_release` | `simulated_hover_gripper_release` | System uses `SIM_Gripper_Servo`, not a winch |
| `proven_obstacle_avoidance` | `measured_oa_lateral_deviation_observed` | Measurement, not proof |
| `cold-chain thermal limits` / `thermal model` | `time-based cold-chain limits` / `cold_chain_limit_minutes` | No actual thermal model exists |
| `HOVER_ROPE_RELEASE` (in docs) | Simulated gripper hover-release | Legacy enum alias retained in code |
| `DynamicAirspaceMonitor` "active" | Interface-ready, NOT active in demo | No live provider wired |
| Perception/PrecisionLanding "integrated" | Component-tested, NOT active in SITL demo | Not constructed in `sitl_mission_demo.py` |

---

## 5. Remaining Work & External Blockers

All local engineering increments (`Increment 1: Vehicle, SITL Physics & Energy Model Integrity`, `Increment 2: End-to-End Hexacopter SITL Verification & Controlled Experimental Suite`, and `Increment 3: Documentation Reconciliation & Portfolio Release Packaging`) are **COMPLETE** and verified.

### External Dependencies / Blockers (Cannot Be Resolved Locally Without Credentials or Hardware)
1. **Live Authenticated DigitalSky API Execution**: Blocked on a registered Indian DGCA pilot/operator JWT (`DIGITALSKY_AUTH_TOKEN`). Without this token, `DigitalSkyIntersectionChecker` validates the full-route polyline request against the OpenAPI 3.0 contract and honestly reports `checked=False, official_clearance=False, status="UNAUTHORISED_NO_PILOT_TOKEN"`.
2. **Closed-Loop 3D Visual Camera Simulation in SITL**: Blocked by the 8 GB unified memory limit on the Apple M3 host (running Gazebo Harmonic + Ogre2 + ROS 2 alongside SITL exceeds available RAM). `PerceptionPipeline` (YOLOv8n + metric depth gating) and `PrecisionLandingController` (ArUco -> `LANDING_TARGET` #149) are verified via component/acceptance tests and recorded/synthetic streams, and are explicitly marked **NOT active** in the headless `sitl_mission_demo.py` run.
3. **Physical Recipient Handover Verification**: Requires physical airframe hardware (load-cell/optical release confirmation or recipient token); simulation always sets `recipient_handover_verified=False`.
4. **Live Continuous Airspace Polling Provider**: `DynamicAirspaceMonitor` is implemented and unit-tested (`EXP-03`), while `sitl_mission_demo.py` triggers its 4D pop-up TFR (`TFR_POPUP_4D_01`) via a deterministic in-flight callback because no open unauthenticated real-time Indian NOTAM/TFR polygon stream exists.

---

## 6. Non-Negotiable Constraints

- Never claim simulated actuator release is physical handover.
- Never fabricate NFZ from GDACS point alerts.
- Never default missing battery to 100%.
- `recipient_handover_verified=False` always in simulation.
- Cold-chain is a time-based limit, not a thermal model.
- Keep docs synchronized with code.
- Preserve all existing SITL logs in `output/`.
