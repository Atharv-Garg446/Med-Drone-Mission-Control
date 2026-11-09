# Reproducible Demonstration and Verification Guide

All commands below assume the working directory is the repository root (`drone_medical_router/`), with the configured virtual environment (`../.venv-sitl/bin/python` or `python3`) and compiled ArduCopter SITL binary (`../ardupilot/build/sitl/bin/arducopter` or `ARDUPILOT_HOME` / `ARDUCOPTER_SITL_BIN`).

---

## 1. Automated Test Suite and Controlled Experiments

### 1.1 Fast Unit, Contract, and Acceptance Tests (No SITL Binary Required)
Runs 114 flight-stack (`41`), acceptance (`33`), GDACS/DigitalSky contract (`16`), and vehicle/perception (`24`) test methods in ~4 seconds:
```bash
python3 -m unittest \
  tests/test_flightstack.py \
  tests/test_full_autonomy_acceptance.py \
  tests/test_gdacs_and_provenance.py \
  tests/test_vehicle_and_perception.py -v
```

### 1.2 Full 209-Test Suite (Including Live Hexacopter SITL Integration)
Runs all 209 automated tests across `tests/test_*.py` (~107 seconds):
```bash
python3 -m unittest discover -s tests -p "test_*.py" -v
```

### 1.3 Controlled 8-Experiment Validation Suite
Runs the 8 validation experiments (`EXP-01` through `EXP-08`) and writes `output/experimental_validation_report.json`:
```bash
python3 experiments_validation.py
```

#### Verified Subsystem Behaviors:
- **External Data & Provenance (`EXP-07`, `test_gdacs_and_provenance.py`)**: Live GDACS API ingestion, HTTP 200/204/malformed handling, episode deduplication, expiry filtering, disk cache fallback (`CachedGDACSProvider`), mandatory GDACS attribution, operational mission-impact assessment (`GDACSMissionImpactAssessor`), DigitalSky OpenAPI 3.0 request/error validation (`FlightPlanAreaValidateRequest`, `Error`, HAL `ValidationError` `_embeded.errors`, `<= 400 ft` ceiling), `DynamicAirspaceMonitor` outage safety (`safe_to_clear_restrictions = False`), and `MapplsAirspaceLayerSpec` (`MAP_VISUALIZATION_ONLY`).
- **Sensor & Perception (`test_vehicle_and_perception.py`, `test_full_autonomy_acceptance.py`)**: Monocular bearing-only vs. metric-depth 3D projection gating (`ObstacleTracker` rejects depthless detections) and ArUco precision landing pose estimation with low-altitude target-loss abort (`LOITER`).
- **Feasibility, Delivery & Landing Continuation (`EXP-01`..`EXP-06`, `EXP-08`)**:
  - Deterministic per-stop delivery mode selection (`resolve_stop_delivery_mode` with `--mode auto`: Stop 1 `hover`, Stop 2 `land`, unsafe sites rejected as `abort` into `plan.unreachable`).
  - Mid-flight site suitability revalidation and mission replan (`apply_site_suitability_update`).
  - Missing or stale battery telemetry is rejected (`operator_required`), never defaulted to `100%` SoC.
  - Time-based cold-chain and hard time-window breaches along detoured A* polylines are rejected (`operator_required`) before upload.
  - `MAV_CMD_NAV_DELAY == 93` for ArduCopter `ContinueAfterLand` (`MIS_OPTIONS = 4`).
  - Intermediate landing disarm recovery switches to `GUIDED` before calling `arm()`.

---

## 2. Full Two-Stop Single-UAV Hexacopter SITL Demonstration (`--mode auto`)

Launches ArduCopter V4.7.0-dev SITL with the 6-motor hexacopter physics profile (`--model hexa:med_drone_hexa_16.json`, `FRAME_CLASS = 2`, `mav_type = "HEXAROTOR"`, `--speedup 10`) and executes the `JAIPUR_SINGLE_UAV` reference mission (`output/sitl_run_20261108_034446.jsonl`):
1. Live GDACS disaster feed query (`OK_LIVE`) + pre-mission operational impact assessment + full-route DigitalSky OpenAPI 3.0 validation (`waypoints_checked: 4`, `official_clearance: false`).
2. Pre-launch feasibility validation (`validate_initial_mission()`: `136.91 Wh <= 284.16 Wh`, payload stepping down `3.5 -> 1.5 -> 0.0 kg`).
3. Native C++ `AP_Proximity_SITL` (`PRX1_TYPE = 10`) raycast detection and `AP_OABendyRuler` avoidance of `OBSTACLE_TOWER_01` on Leg 1 (`17` proximity frames, `50` `OA_DB` items, `max_oa_deviation_m = 34.08 m`, `min_obstacle_surface_clearance_m = 8.98 m`).
4. Automatic per-stop mode resolution (`--mode auto`): `hover` at Stop 1 (`Gopalpura Flooded Relief Camp`) with causal `SIM_Gripper_Servo` release (`simulated_hover_gripper_release`, `recipient_handover_verified = false`).
5. Mid-flight 4D pop-up TFR (`TFR_POPUP_4D_01`) conflict detection, `LOITER` hold, A* detour replanning, 4-gate feasibility verification, mission upload + item-by-item readback verification (`19.2 ms`), and `AUTO` resumption.
6. Ground landing (`NAV_LAND`), ground gripper release (`DO_GRIPPER`), 5-second ground dwell (`MAV_CMD_NAV_DELAY = 93`, `simulated_ground_gripper_release_after_dwell`, `recipient_handover_verified = false`), automatic retakeoff (`NAV_TAKEOFF = 22`), `RTL` to SMS Hospital Depot (`alt_rel_m = 0.019 m`), and 6S LiPo discharge (`25.039 V -> 21.925 V`, `9,721.0 mAh` consumed, `39%` remaining).

```bash
python3 sitl_mission_demo.py \
  --scenario jaipur_single_uav \
  --mode auto \
  --airspace-change \
  --inject-obstacle \
  --speedup 10
```

---

## 3. Controlled Hexacopter Obstacle-Avoidance Comparison

Compares the 1-stop hexacopter trajectory without an obstacle against the identical 1-stop trajectory with `OBSTACLE_TOWER_01` loaded into `AC_PolyFence_loader` (`AP_Proximity_SITL` `PRX1_TYPE = 10`), differing only in `--inject-obstacle`:

```bash
# Run A: Baseline (no obstacle)
python3 sitl_mission_demo.py \
  --scenario jaipur_single_uav --mode hover --max-stops 1 --speedup 10

# Run B: Obstacle present in C++ world
python3 sitl_mission_demo.py \
  --scenario jaipur_single_uav --mode hover --max-stops 1 --inject-obstacle --speedup 10
```

### Verified Comparison Metrics:

| Metric | Run A: Baseline (`output/sitl_run_20261108_035408.jsonl`) | Run B: Obstacle-Present (`output/sitl_run_20261108_035631.jsonl`) |
| :--- | :--- | :--- |
| Scenario / Airframe / Stops / Mode / Speedup | `jaipur_single_uav`, `HEXAROTOR` (`13`), `1` stop (`[1]`), `hover`, `10x` (`6.361 km`) | `jaipur_single_uav`, `HEXAROTOR` (`13`), `1` stop (`[1]`), `hover`, `10x` (`6.361 km`) |
| `obstacle_entered_proximity_range` | `false` | `true` (`min_distance_m = 4.02 m`, `min_obstacle_surface_clearance_m = 11.64 m`) |
| `proximity_active_frames` (`DISTANCE_SENSOR` `< 50 m`) | `0` | `63` |
| `ardupilot_oa_db_confirmations` (`ADSB_VEHICLE` `OA_DB`) | `0` | `50` |
| `max_oa_deviation_m` (Lateral cross-track in `AUTO` mode) | **`0.0 m`** | **`38.42 m`** |
| `measured_oa_lateral_deviation_observed` | `false` | `true` |
| 6S Battery End State / Consumed | `22.979 V` / `6,589.0 mAh` (`58%` left) | `22.948 V` / `6,699.0 mAh` (`+110.0 mAh` avoidance cost, `58%` left) |

---

## 4. Inspecting Generated SITL Logs

Each SITL run writes two files to `output/`:
- `output/sitl_run_<YYYYMMDD_HHMMSS>.jsonl`: Timestamped JSON-lines event and 1 Hz telemetry log.
- `output/sitl_mission_<YYYYMMDD_HHMMSS>.waypoints`: QGroundControl `QGC WPL 110` waypoint file.

To inspect the key autonomy events from the latest SITL run:
```bash
python3 -c '
import glob, json
latest = sorted(glob.glob("output/sitl_run_*.jsonl"))[-1]
print("Log file:", latest)
for line in open(latest):
    rec = json.loads(line)
    if rec["event"] not in ("telemetry", "autopilot_text"):
        print(f"[{rec[\"t_rel_s\"]:6.2f}s] {rec[\"event\"]:32s}", {k: v for k, v in rec.items() if k not in ("t_wall", "t_rel_s", "t", "event", "mission", "old_mission")})
'
```
