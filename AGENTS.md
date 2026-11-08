# AGENTS.md — Non-Negotiable Engineering Rules & Project Continuity

This file defines the mandatory architectural, engineering, and scientific-honesty rules for all AI agents and engineers working on **Med-Drone Mission Control** (`drone_medical_router`).

---

## 1. Core System Identity & Scope

1. **Primary System (Single-UAV Autonomy):**
   - The primary operational system and end-to-end demonstration is a **single autonomous medical-delivery UAV** operating on the **Jaipur Single-UAV Medical Autonomy Reference** scenario (`JAIPUR_SINGLE_UAV` in [`config.py`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/config.py)).
   - It integrates: live external disaster intelligence (GDACS), official Indian DigitalSky OpenAPI 3.0 validation contracts, dynamic 4D airspace replanning, A* pathfinding, physical vehicle/battery/payload/cold-chain/deadline feasibility gates, native ArduCopter SITL (`ArduCopter V4.7.0-dev` configured as a unified 6-motor hexacopter via `--model hexa:med_drone_hexa_16.json` + `FRAME_CLASS = 2`), C++ proximity raycasting (`AP_Proximity_SITL` `PRX1_TYPE = 10`) with `AP_OABendyRuler` obstacle avoidance, computer-vision perception & fiducial landing modules, deterministic per-stop hover/land delivery state machines, intermediate landing continuation (`MIS_OPTIONS = 4`, `MAV_CMD_NAV_DELAY = 93`), and return-to-launch (`RTL`).
2. **Secondary Research Component (Multi-Drone Fleet Routing):**
   - Preserve [`simulate_fleet.py`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/simulate_fleet.py), [`vrp_scratch.py`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/vrp_scratch.py), [`vrp_ortools.py`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/vrp_ortools.py), and [`replan.py`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/replan.py) intact as an optional multi-UAV logistics research module.
   - Never globally force `fleet_size = 1` across multi-drone scenarios (`JAIPUR_DISASTER`, `JAIPUR_HOSPITAL`, `CHENNAI_FLOOD`), and never delete fleet routing tests.
3. **Deferred Scope (Do Not Reintroduce):**
   - Comprehensive weather integration and wind-field subsystems are explicitly deferred for this sprint. Do not spend engineering time adding weather APIs.

---

## 2. Strict Data Provenance & Scientific Honesty Rules

Every external, cached, simulated, or synthetic input must carry an explicit `SourceClassification` tag:
- `LIVE_EXTERNAL`: Retrieved live over the network during execution from an authoritative public API (e.g., `GDACSProvider`).
- `OFFICIAL_API_VALIDATION`: Contract-validated against an official regulatory OpenAPI schema (`DigitalSkyIntersectionChecker` against `docs/external_specs/digitalsky_interactive_maps_api_v1.0.yml`). Only sets `official_clearance = True` when an authenticated pilot JWT (`DIGITALSKY_AUTH_TOKEN`) is present and returns a valid non-intersecting response.
- `MAP_VISUALIZATION_ONLY`: Browser/UI tile overlays that do not expose polygon geometry downloads (`MapplsAirspaceLayerSpec`).
- `SIMULATED_SENSOR`: Internal simulator physics/raycasting or synthetic depth/camera sensors (`AP_Proximity_SITL` `PRX1_TYPE = 10`, `SyntheticDepthSource`).
- `CACHED`: Previously fetched live data served from disk/memory cache during an outage with `stale = True` (`CachedGDACSProvider`, `CachedAirspaceProvider`).
- `SYNTHETIC_FIXTURE`: Deterministic scenario polygons (`JAIPUR_AIRPORT_ZONE`, `CITY_PALACE_ZONE`, `TFR_POPUP_4D_01`) and static scenario site-suitability dictionaries.

### Mandatory Prohibitions:
- **Never** label a local scenario polygon or synthetic TFR as live DigitalSky or official DGCA/AAI data.
- **Never** bypass authentication, scrape protected endpoints, or fabricate pilot tokens.
- **Never** treat an unauthenticated DigitalSky check (`checked = False`) as flight permission, and never equate a green-zone intersection result with blanket operational approval.
- **Never** omit the mandatory GDACS attribution (`"Global Disaster Awareness and Coordination System, GDACS."`) when logging or displaying GDACS data.
- **Never** treat macro-level GDACS disaster centroids/polygons as legal aviation no-fly zones or as proof of micro-scale landing-pad clearance.
- **Never** fabricate 3D metric obstacle coordinates from a monocular 2D bounding box without a metric depth source.
- **Never** default missing, unknown (`None`), or stale battery telemetry to `100%` State of Charge (SoC).
- **Never** claim simulated actuator release (`SIM_Gripper_Servo`) is a verified physical gripper or real human recipient handover. Simulated release provenance is `simulated_hover_gripper_release` or `simulated_ground_gripper_release_after_dwell`, never described as "winch" in new code.

---

## 3. Delivery Decision & Mission Consistency Rules

1. **Single Deterministic Mode Resolution (`resolve_stop_delivery_mode`):**
   - Delivery mode selection (`"auto"`, `"hover"`, `"land"`) must be resolved deterministically once per stop and kept consistent across `MissionPlan`, uploaded MAVLink items, `PayloadManager`, runtime site assessment, and event logs.
   - In `"auto"` mode, each stop's `SiteSuitability.recommended_mode` is respected unless overridden by safety constraints or site uncertainty.
   - Explicit `"land"` requests must **never** override hard landing-safety constraints (`surface_condition` in `flooded`/`debris`/`mud`/`submerged`/`collapsed`, `slope_deg > 7.0`, `is_safe_for_landing == False`, or `uncertainty != "known"`).
   - If `obstacle_clearance_m < 10.0` or the site is marked `"abort"`, the site is unsuitable for **both** landing and hover release (`ok = False`, `selected_mode = "abort"`). `"abort"` must never be treated as a valid delivery method.
   - If site suitability changes in flight before delivery, `MissionRunner.apply_site_suitability_update()` must revalidate and replan/upload the updated MAVLink sequence before the stop is executed.

---

## 4. Continuity & Verification Rules

- Always read [`docs/PROJECT_MASTER_CONTEXT.md`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/docs/PROJECT_MASTER_CONTEXT.md) and [`docs/AGENT_HANDOFF.md`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/docs/AGENT_HANDOFF.md) before making architectural changes.
- Keep [`docs/PROJECT_MASTER_CONTEXT.md`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/docs/PROJECT_MASTER_CONTEXT.md), [`docs/PROJECT_INTERVIEW_MASTER.md`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/docs/PROJECT_INTERVIEW_MASTER.md), and [`docs/AGENT_HANDOFF.md`](file:///Users/mahigarg/Documents/Drone%20Shit/drone_medical_router/docs/AGENT_HANDOFF.md) synchronized with the actual codebase and verified test/SITL evidence.
- Controlled obstacle-avoidance comparisons must use **identical** scenario, stop count, delivery mode, speedup, and vehicle parameters, differing **only** in `--inject-obstacle`.
- Always pair `FRAME_CLASS = 2` (`sitl_delivery.parm`) with `--model hexa:med_drone_hexa_16.json` (`flightstack/sitl.py`). Never run `FRAME_CLASS = 2` against `--model +` (which instantiates only 4 motors in `SIM_Frame.cpp` and prevents takeoff).

