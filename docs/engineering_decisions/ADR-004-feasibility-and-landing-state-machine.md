# ADR-004: Hard Replan Feasibility Gating & Intermediate Landing Continuation (`MAV_CMD_NAV_DELAY = 93`)

## Status
Accepted (2026-11-05)

## Context
During Phase 0 auditing of the flight stack (`flightstack/mission.py` and `flightstack/mission_runner.py`), three critical operational defects were identified:
1. **Infeasible Replan Upload:** `apply_airspace_update()` logged warnings when a detoured route violated `cold_chain_limit_minutes` or `window_minutes` (and computed ETA using straight-line `haversine_km` instead of the detoured A* polyline), then uploaded the infeasible mission anyway.
2. **Optimistic Battery Fallback:** When `telemetry.battery.value` was `None`, `apply_airspace_update()` defaulted `current_pct` to `100.0%` SoC.
3. **Intermediate Landing Disarm Loop:** At landing delivery stops, ArduCopter disarmed immediately upon touchdown (`NAV_LAND`), and `MissionRunner.step()` repeatedly called `self.link.arm()` while in `AUTO` mode, producing an infinite `"Arm: AUTO mode not armable"` error loop until mission timeout.

## Root Cause Analysis & Decision
1. **Hard Replan Feasibility Gates (`MissionRunner.apply_airspace_update`):**
   - Reject any replan (`operator_required` + `LOITER`) if battery telemetry is missing (`None`), unknown (`remaining_pct is None`), or stale (`age > 10 s`). Never assume `100%` SoC.
   - Compute cumulative ETA along the actual detoured A* polylines (`_polyline_km(new_plan.leg_paths[idx])`) plus per-stop dwell times.
   - Reject any replan (`operator_required` + `LOITER`) when `eta_min > cold_chain_limit_minutes` (`enforce_cold_chain=True`) or `eta_min > window_minutes` (`enforce_hard_deadlines=True`).
2. **Fix `MAV_CMD_NAV_DELAY = 93` (was `94`) for `ContinueAfterLand` (`MIS_OPTIONS = 4`):**
   - Inspection of ArduPilot's `AP_Mission::is_takeoff_next()` (`libraries/AP_Mission/AP_Mission.cpp` lines 191–243) showed that when `MIS_OPTIONS` bit 2 (`ContinueAfterLand = 4`) is set, `ModeAuto::verify_land()` keeps the motors armed in `GROUND_IDLE` on touchdown **if and only if** `is_takeoff_next()` finds `MAV_CMD_NAV_TAKEOFF` (`22`) after skipping `DO` commands and `MAV_CMD_NAV_DELAY` (`93`).
   - Because `flightstack/mission.py` had a typo (`MAV_CMD_NAV_DELAY = 94`, which is `MAV_CMD_NAV_PAYLOAD_PLACE`), `is_takeoff_next()` returned `false` and disarmed on touchdown!
   - Fixing `MAV_CMD_NAV_DELAY = 93` allows ArduCopter to land, release the gripper (`211`), dwell 5 seconds on the ground (`93`), and automatically execute `MAV_CMD_NAV_TAKEOFF` (`22`) and `RTL` (`20`).
3. **Safe Disarm Recovery Fallback (`MissionRunner.step`):**
   - If the vehicle ever disarms at an intermediate stop, `MissionRunner` switches to `GUIDED` mode before calling `arm()`, sets the current item to `retakeoff_seq`, and switches back to `AUTO`. It never calls `arm()` in `AUTO` mode.

## Consequences
- Intermediate landing deliveries execute cleanly in ArduCopter SITL (`ground_landing_confirmed -> Gripper Released -> Delaying 5 sec -> Mission: 7 Takeoff -> RTL -> final_landing_confirmed -> mission_finished`), and infeasible replans are deterministically blocked before upload.

