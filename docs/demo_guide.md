# Reproducible Demo & Verification Guide

All commands below are designed to be executed from the workspace root (`/Users/mahigarg/Documents/Drone Shit`) using the configured virtual environment (`/Users/mahigarg/Documents/Drone Shit/.venv-sitl/bin/python`) and compiled ArduCopter SITL binary (`/Users/mahigarg/Documents/Drone Shit/ardupilot/build/sitl/bin/arducopter`).

---

## 1. Fast Unit, Contract & Acceptance Test Suite (No SITL Required)

Runs 81 unit, integration, and live-GDACS contract tests in ~3.5 seconds:

```bash
PYTHONPATH=drone_medical_router "/Users/mahigarg/Documents/Drone Shit/.venv-sitl/bin/python" -m unittest \
  drone_medical_router/tests/test_flightstack.py \
  drone_medical_router/tests/test_full_autonomy_acceptance.py \
  drone_medical_router/tests/test_gdacs_and_provenance.py -v
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

## 2. Full End-to-End Single-UAV SITL Demonstration (`--mode auto`)

Launches ArduCopter V4.7.0-dev SITL (`--speedup 10`) and executes the complete `JAIPUR_SINGLE_UAV` mission (`output/sitl_run_20261106_013542.jsonl`) with:
1. Live GDACS disaster feed query + operational mission-impact assessment + honest DigitalSky OpenAPI 3.0 check.
2. Native C++ `AP_Proximity_SITL` (`PRX1_TYPE = 10`) raycast detection & `AP_OABendyRuler` avoidance of `OBSTACLE_TOWER_01` on Leg 1 (`max_oa_deviation_m = 10.01 m`).
3. Automatic per-stop mode resolution (`--mode auto`): `hover` at Stop 1 (`Gopalpura Flooded Relief Camp`) with `SIM_Gripper_Servo` release (`simulated_hover_gripper_release`). Note: gripper acknowledgement is causal (waits for ArduPilot `STATUSTEXT`), not auto-acknowledged.
4. Mid-flight 4D pop-up TFR (`TFR_POPUP_4D_01`) conflict detection, `LOITER` hold, A* detour replanning, 4-gate feasibility verification, mission upload + item-by-item readback verification, and `AUTO` resumption.
5. Full ground landing (`NAV_LAND`), ground gripper release (`DO_GRIPPER`), 5-second ground dwell (`MAV_CMD_NAV_DELAY = 93`), automatic retakeoff (`NAV_TAKEOFF = 22`), and `RTL` to SMS Hospital depot at Stop 2 (`Jhalana Elevated Medical Post`).

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

## 3. Controlled Obstacle-Avoidance Comparison (Baseline vs. Obstacle-Present)

To compare the 1-stop trajectory **without** an obstacle against the identical 1-stop trajectory **with** `OBSTACLE_TOWER_01` present in the simulator's C++ world (`AC_PolyFence_loader` + `AP_Proximity_SITL` `PRX1_TYPE = 10`):

```bash
# Run A: Baseline (no obstacle)
PYTHONPATH=drone_medical_router "/Users/mahigarg/Documents/Drone Shit/.venv-sitl/bin/python" \
  drone_medical_router/sitl_mission_demo.py \
  --scenario jaipur_single_uav --mode hover --max-stops 1 --speedup 10

# Run B: Obstacle present in C++ world (identical scenario, stop count, mode, and speedup)
PYTHONPATH=drone_medical_router "/Users/mahigarg/Documents/Drone Shit/.venv-sitl/bin/python" \
  drone_medical_router/sitl_mission_demo.py \
  --scenario jaipur_single_uav --mode hover --max-stops 1 --inject-obstacle --speedup 10
```

### Verified Controlled 1-Stop Comparison Metrics:

| Metric | Run A: Baseline (`output/sitl_run_20261106_001944.jsonl`) | Run B: Obstacle-Present (`output/sitl_run_20261106_013239.jsonl`) |
|---|---|---|
| Scenario / Stops / Mode / Speedup | `jaipur_single_uav`, `1` stop (`[1]`), `hover`, `10x` (`6.361 km`) | `jaipur_single_uav`, `1` stop (`[1]`), `hover`, `10x` (`6.361 km`) |
| `proximity_sensor_active` (`PRX1_TYPE = 10`) | `true` | `true` |
| `obstacle_entered_proximity_range` | `false` | `true` (`min_distance_m = 0.29 m..49.12 m`, sectors `1, 2, 3`) |
| `proximity_active_frames` (`DISTANCE_SENSOR` `< 50 m`) | `0` | `73` |
| `ardupilot_oa_db_confirmations` (`ADSB_VEHICLE` `OA_DB`) | `0` | `50` |
| `max_oa_deviation_m` (Lateral cross-track in `AUTO` mode) | **`0.0 m`** | **`34.50 m`** |
| `measured_oa_lateral_deviation_observed` | `false` | `true` |
| Mission Duration / Battery Consumed | `87.715 s` / `6,646.0 mAh` (`58%` left) | `90.698 s` / `6,769.0 mAh` (`57%` left) |

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

