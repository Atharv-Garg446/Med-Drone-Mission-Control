# Mission Execution, Feasibility Gating, Delivery & Landing State Machine

## 1. End-to-End Single-UAV Mission Lifecycle (`JAIPUR_SINGLE_UAV`)

The primary reference scenario (`JAIPUR_SINGLE_UAV` in `config.py`) operates a single hexacopter (`MedDrone-Hexa-16`, `5.5 kg` empty mass, `3.2 kg` 6S 16 Ah battery with `284.16 Wh` usable energy at `20%` reserve floor, `8.0 kg` max payload, `17.5 kg` MTOW, `10.0 m/s` cruise speed) from **SMS Hospital Depot** (`26.9127, 75.8060`) across two medical delivery stops:

1. **Stop 1 — Gopalpura Flooded Relief Camp (`26.8850, 75.7930`, `2.0 kg`, critical):**
   - `site_suitability`: `surface_condition = "flooded"`, `is_safe_for_landing = False`, `obstacle_clearance_m = 12.0 m`.
   - Even when the mission is launched with `--mode land`, `build_delivery_mission()` and `PayloadManager.assess_site()` automatically downgrade Stop 1 from `land` to `hover` (`hover_descend` to `15 m AGL` -> `MAV_CMD_DO_GRIPPER` release -> `climb` to `60 m AGL`).
2. **Stop 2 — Jhalana Elevated Medical Post (`26.8890, 75.8180`, `1.5 kg`, urgent):**
   - `site_suitability`: `surface_condition = "clear"`, `is_safe_for_landing = True`, `slope_deg = 2.0°`, `obstacle_clearance_m = 18.0 m`.
   - Cleared for full ground landing (`MAV_CMD_NAV_LAND` -> ground touchdown -> `MAV_CMD_DO_GRIPPER` release -> `MAV_CMD_NAV_DELAY` 5 s ground dwell -> `MAV_CMD_NAV_TAKEOFF` retakeoff to `60 m AGL`).
3. **Return to Launch (`MAV_CMD_NAV_RETURN_TO_LAUNCH`):**
   - Flies back to SMS Hospital Depot (`26.9127, 75.8060`), lands, disarms motors, and logs `final_landing_confirmed` and `mission_finished`.

---

## 2. Dynamic Replan Feasibility Gating (`MissionRunner.apply_airspace_update`)

When a dynamic 4D No-Fly Zone / Temporary Flight Restriction (`AirspaceZone`) is published mid-flight:

1. **4D Spatio-Temporal & Altitude Conflict Check (`validate_polyline`):**
   - Evaluates the remaining flight path from the autopilot's fresh EKF position (`fresh_position()`) against all active zones, checking horizontal polygon buffer (`safety_margin_m = 50.0 m`), vertical overlap (`floor_m_agl <= alt_m_agl <= ceiling_m_agl`), and time-window overlap (`valid_from <= segment_eta <= valid_to`).
2. **Tactical Hold (`LOITER`):**
   - Immediately switches ArduCopter from `AUTO` to `LOITER` while computing and validating a candidate replacement route via A* (`build_delivery_mission(..., include_takeoff=False)`).
3. **Four-Gate Hard Feasibility Check (Before Any Upload):**
   - **Gate 1 — Gross Mass / MTOW:** Verifies `vm.total_mass_kg(remaining_cargo_kg) <= vm.mtow_kg`.
   - **Gate 2 — Fresh Battery Telemetry & Usable Energy Budget:**
     - **Never defaults missing or stale battery telemetry to `100%` SoC.** If `telemetry.battery.value` is `None`, `remaining_pct` is `None`, or `battery.age() > 10.0 s`, the replan is immediately rejected (`operator_required`, `breach_type="battery_telemetry_missing"` or `"battery_telemetry_stale"`).
     - Verifies that `energy_eval.total_energy_wh <= usable_wh_remaining` above the `20%` reserve floor.
   - **Gate 3 — Cold-Chain Integrity (`enforce_cold_chain = True`):**
     - Computes cumulative arrival time (`eta_min`) along the **actual detoured A* leg polylines** (`_polyline_km(new_plan.leg_paths[idx])`) plus per-stop dwell times. If any stop's `eta_min > cold_chain_limit_minutes`, the replan is rejected (`operator_required`, `breach_type="cold_chain"`).
   - **Gate 4 — Hard Delivery Time Window (`enforce_hard_deadlines = True`):**
     - If any stop's `eta_min > window_minutes`, the replan is rejected (`operator_required`, `breach_type="time_window"`).
4. **Upload, Readback Verification & Resumption:**
   - Only if all four gates pass does `MissionRunner` upload the replacement mission (`upload(new_plan, verify=True)`), verify item-by-item readback from the autopilot (`verify_mission_readback`), set current mission item (`set_current_item(1)`), and switch back to `AUTO`.

---

## 3. Intermediate Landing, Ground Delivery & Resumption Mechanics

### 3.1 Root Cause & Fix of the Historical Intermediate Disarm Loop
- **Historical Defect:** In earlier runs, `flightstack/mission.py` defined `MAV_CMD_NAV_DELAY = 94` (which is actually `MAV_CMD_NAV_PAYLOAD_PLACE`, whereas `MAV_CMD_NAV_DELAY` is `93`). When ArduCopter touched down on `MAV_CMD_NAV_LAND` (`21`), `ModeAuto::verify_land()` called `AP_Mission::continue_after_land_check_for_takeoff()` -> `AP_Mission::is_takeoff_next()`. Because `is_takeoff_next()` in `AP_Mission.cpp` only skips `MAV_CMD_NAV_DELAY` (`93`) when scanning ahead for `MAV_CMD_NAV_TAKEOFF` (`22`), seeing command `94` caused ArduCopter to return `false` and disarm on touchdown. Then `MissionRunner.step()` called `self.link.arm()` while still in `AUTO` mode, triggering a repeated `"Arm: AUTO mode not armable"` error loop until timeout.
- **Two-Layer Engineering Fix:**
  1. **Correct MAVLink Command ID (`MAV_CMD_NAV_DELAY = 93` in `flightstack/mission.py`):** With `MIS_OPTIONS = 4` (`ContinueAfterLand`) and `MAV_CMD_NAV_DELAY = 93`, `AP_Mission::is_takeoff_next()` recognizes the upcoming sequence (`DO_GRIPPER (211) -> NAV_DELAY (93) -> NAV_TAKEOFF (22)`), keeps motors armed in `GROUND_IDLE` on touchdown, executes the gripper release and 5-second ground dwell, and automatically executes `NAV_TAKEOFF` (`22`) and `RTL` (`20`).
  2. **Safe Disarm Recovery Fallback (`flightstack/mission_runner.py`):** If the vehicle ever disarms on the ground at an intermediate stop (`not tel.armed` before all stops are delivered and returned home), `MissionRunner.step()` completes any pending ground delivery within `25 m` of the stop and executes the proper mode transition sequence: `set_mode("GUIDED") -> arm() -> set_current_item(retakeoff_seq) -> set_mode("AUTO") -> start_mission()`. It **never** calls `arm()` while in `AUTO` mode.

