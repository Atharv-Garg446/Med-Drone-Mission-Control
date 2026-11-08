# Reproducible Demo & Verification Guide

All commands below are designed to be executed from the workspace root (`/Users/mahigarg/Documents/Drone Shit`) using the configured virtual environment (`/Users/mahigarg/Documents/Drone Shit/.venv-sitl/bin/python`) and compiled ArduCopter SITL binary (`/Users/mahigarg/Documents/Drone Shit/ardupilot/build/sitl/bin/arducopter`).

---

## 1. Fast Unit, Contract & Acceptance Test Suite (No SITL Required)

Runs 106 flightstack, vehicle/perception, acceptance, and live-GDACS contract tests in ~4 seconds (part of the full 209-test repository suite):

```bash
PYTHONPATH=drone_medical_router "/Users/mahigarg/Documents/Drone Shit/.venv-sitl/bin/python" -m unittest \
  drone_medical_router/tests/test_flightstack.py \
  drone_medical_router/tests/test_full_autonomy_acceptance.py \
  drone_medical_router/tests/test_gdacs_and_provenance.py \
  drone_medical_router/tests/test_vehicle_and_perception.py -v
```

To run the complete 209-test suite (including live hexacopter `test_sitl_integration.py`, ~107 seconds):

```bash
PYTHONPATH=drone_medical_router "/Users/mahigarg/Documents/Drone Shit/.venv-sitl/bin/python" \
  -m unittest discover -s drone_medical_router/tests -p "test_*.py" -v
```

To run the 8-case controlled experimental validation suite (`output/experimental_validation_report.json`):

```bash
PYTHONPATH=drone_medical_router "/Users/mahigarg/Documents/Drone Shit/.venv-sitl/bin/python" \
  drone_medical_router/experiments_validation.py
```

### What This Verifies:
- **Phase 1 (External Data & Provenance):** Live GDACS API fetch (`test_live_gdacs_api_fetch`), HTTP 200/204/malformed handling, episode deduplication, expiry filtering, disk cache fallback (`CachedGDACSProvider`), mandatory GDACS attribution, operational mission-impact assessment (`GDACSMissionImpactAssessor`), DigitalSky OpenAPI 3.0 payload/header/error construction (`FlightPlanAreaValidateRequest`, `Error`, HAL `ValidationError` `_embeded.errors`, `<= 400 ft` ceiling), `DynamicAirspaceMonitor`, and `MapplsAirspaceLayerSpec` (`MAP_VISUALIZATION_ONLY`).
- **Phase 2 (Sensor & Perception):** Monocular bearing-only vs. metric-depth 3D projection gating, precision landing ArUco detection & low-altitude abort (`LOITER`).
- **Phase 3 (Feasibility, Delivery & Landing):**
  - Deterministic per-stop delivery mode selection (`resolve_stop_delivery_mode` with `--mode auto`: Stop 1 `hover`, Stop 2 `land`, unsafe sites rejected as `abort` into `plan.unreachable`).
  - Mid-flight site suitability revalidation and mission replan (`apply_site_suitability_update`).
  - Missing or stale battery telemetry is rejected (`operator_required`), never defaulted to `100%` SoC.
  - Cold-chain and hard time-window breaches along detoured A* polylines are rejected (`operator_required`), never uploaded.
  - `MAV_CMD_NAV_DELAY == 93` for ArduCopter `ContinueAfterLand` (`MIS_OPTIONS = 4`).
  - Intermediate landing disarm recovery switches to `GUIDED` before calling `arm()`.

---

## 2. Full End-to-End Single-UAV Hexacopter SITL Demonstration (`--mode auto`)

Launches ArduCopter V4.7.0-dev SITL with the 6-motor hexacopter physics profile (`--model hexa:med_drone_hexa_16.json`, `FRAME_CLASS = 2`, `mav_type = "HEXAROTOR"`, `--speedup 10`) and executes the complete `JAIPUR_SINGLE_UAV` mission (`output/sitl_run_20261108_034446.jsonl`) with:
1. Live GDACS disaster feed query (`OK_LIVE`) + pre-mission operational impact assessment + honest full-route DigitalSky OpenAPI 3.0 check (`waypoints_checked: 4`, `official_clearance: false`).
2. Shared pre-launch feasibility validation (`validate_initial_mission()`: `136.91 Wh <= 284.16 Wh`, payload stepping down `3.5 -> 1.5 -> 0.0 kg`).
3. Native C++ `AP_Proximity_SITL` (`PRX1_TYPE = 10`) raycast detection & `AP_OABendyRuler` avoidance of `OBSTACLE_TOWER_01` on Leg 1 (`17` proximity frames, `50` `OA_DB` items, `max_oa_deviation_m = 34.08 m`, `min_obstacle_surface_clearance_m = 8.98 m`).
4. Automatic per-stop mode resolution (`--mode auto`): `hover` at Stop 1 (`Gopalpura Flooded Relief Camp`) with `SIM_Gripper_Servo` release (`simulated_hover_gripper_release`, `recipient_handover_verified = false`). Gripper acknowledgement is strictly causal (waits for ArduPilot `"Gripper Released"` `STATUSTEXT`).
5. Mid-flight 4D pop-up TFR (`TFR_POPUP_4D_01`) conflict detection, `LOITER` hold, A* detour replanning, 4-gate feasibility verification, mission upload + item-by-item readback verification (`19.2 ms`), and `AUTO` resumption.
6. Full ground landing (`NAV_LAND`), ground gripper release (`DO_GRIPPER`), 5-second ground dwell (`MAV_CMD_NAV_DELAY = 93`, `simulated_ground_gripper_release_after_dwell`, `recipient_handover_verified = false`), automatic retakeoff (`NAV_TAKEOFF = 22`), `RTL` to SMS Hospital depot (`alt_rel_m = 0.019 m`), and 6S LiPo discharge (`25.039 V -> 21.925 V`, `9,721.0 mAh` consumed, `39%` remaining).

```bash
PYTHONPATH=drone_medical_router "/Users/mahigarg/Documents/Drone Shit/.venv-sitl/bin/python" \
  drone_medical_router/sitl_mission_demo.py \
  --scenario jaipur_single_uav \
  --mode auto \
  --airspace-change \
  --inject-obstacle \
  --speedup 10
```

---

## 3. Controlled Hexacopter Obstacle-Avoidance Comparison (Baseline vs. Obstacle-Present)

To compare the 1-stop hexacopter trajectory **without** an obstacle against the identical 1-stop hexacopter trajectory **with** `OBSTACLE_TOWER_01` present in the simulator's C++ world (`AC_PolyFence_loader` + `AP_Proximity_SITL` `PRX1_TYPE = 10`), differing **only** in `--inject-obstacle`:

```bash
# Run A: Baseline (no obstacle)
PYTHONPATH=drone_medical_router "/Users/mahigarg/Documents/Drone Shit/.venv-sitl/bin/python" \
  drone_medical_router/sitl_mission_demo.py \
  --scenario jaipur_single_uav --mode hover --max-stops 1 --speedup 10

# Run B: Obstacle present in C++ world (identical scenario, airframe, stop count, mode, and speedup)
PYTHONPATH=drone_medical_router "/Users/mahigarg/Documents/Drone Shit/.venv-sitl/bin/python" \
  drone_medical_router/sitl_mission_demo.py \
  --scenario jaipur_single_uav --mode hover --max-stops 1 --inject-obstacle --speedup 10
```

### Verified Controlled 1-Stop Hexacopter Comparison Metrics:

| Metric | Run A: Baseline (`output/sitl_run_20261108_035408.jsonl`) | Run B: Obstacle-Present (`output/sitl_run_20261108_035631.jsonl`) |
|---|---|---|
| Scenario / Airframe / Stops / Mode / Speedup | `jaipur_single_uav`, `HEXAROTOR` (`13`), `1` stop (`[1]`), `hover`, `10x` (`6.361 km`) | `jaipur_single_uav`, `HEXAROTOR` (`13`), `1` stop (`[1]`), `hover`, `10x` (`6.361 km`) |
| `proximity_sensor_active` (`PRX1_TYPE = 10`) | `true` | `true` |
| `obstacle_entered_proximity_range` | `false` | `true` (`min_distance_m = 4.0 m..49.84 m`, `min_obstacle_surface_clearance_m = 11.64 m`) |
| `proximity_active_frames` (`DISTANCE_SENSOR` `< 50 m`) | `0` | `63` |
| `ardupilot_oa_db_confirmations` (`ADSB_VEHICLE` `OA_DB`) | `0` | `50` |
| `max_oa_deviation_m` (Lateral cross-track in `AUTO` mode) | **`0.0 m`** | **`38.42 m`** |
| `measured_oa_lateral_deviation_observed` | `false` | `true` |
| 6S Battery End State / Consumed | `22.979 V` / `6,589.0 mAh` (`58%` left) | `22.948 V` / `6,699.0 mAh` (`+110.0 mAh` avoidance cost, `58%` left) |

---

## 4. Inspecting the Generated Evidence Logs

Every SITL run writes two artifacts to `drone_medical_router/output/`:
- `output/sitl_run_<YYYYMMDD_HHMMSS>.jsonl`: Timestamped JSON-lines event and 1 Hz telemetry log.
- `output/sitl_mission_<YYYYMMDD_HHMMSS>.waypoints`: QGroundControl `QGC WPL 110` waypoint file.

To inspect the key autonomy events from the latest SITL run:

```bash
python3 -c '
import glob, json
latest = sorted(glob.glob("drone_medical_router/output/sitl_run_*.jsonl"))[-1]
print("Log file:", latest)
for line in open(latest):
    rec = json.loads(line)
    if rec["event"] not in ("telemetry", "autopilot_text"):
        print(f"[{rec[\"t_rel_s\"]:6.2f}s] {rec[\"event\"]:32s}", {k: v for k, v in rec.items() if k not in ("t_wall", "t_rel_s", "t", "event", "mission", "old_mission")})
'
```

