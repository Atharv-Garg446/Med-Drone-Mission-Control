#!/usr/bin/env python3
"""
experiments_validation.py
=========================
Controlled experimental validation suite for Med-Drone Mission Control.

Executes and records quantitative metrics for all 8 required engineering
experiments (Section 9 of the Final Engineering Review & Portfolio Release):
  1. Route planning with and without restricted areas.
  2. Mission feasibility across payload, MTOW, battery, cold-chain, and deadline cases.
  3. Obstacle-response baseline vs. obstacle-response enabled (from SITL JSONL logs).
  4. Changing 4D restriction and in-flight replanning.
  5. Hover vs. landing vs. abort site-suitability paths.
  6. Failed and missing actuator feedback.
  7. Stale or unavailable external hazard/airspace data.
  8. Mission recovery and return-to-launch behaviour.

Writes structured output to ``output/experimental_validation_report.json``.
"""

from __future__ import annotations

import glob
import json
import os
import sys
import time
from datetime import datetime, timedelta, timezone
from typing import Any, Dict, List

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from airspace_provider import (
    AirspaceUnavailable,
    AirspaceSnapshot,
    AirspaceZone,
    DynamicAirspaceMonitor,
    SourceClassification,
)
from config import JAIPUR_SINGLE_UAV, Location
from flightstack.mission import (
    MissionProfile,
    MissionValidationError,
    _polyline_km,
    build_delivery_mission,
    validate_polyline,
)
from flightstack.mission_runner import EventLog, MissionRunner
from flightstack.payload import (
    DeliveryState,
    PayloadManager,
    SiteSuitability,
    resolve_stop_delivery_mode,
)
from flightstack.telemetry import BatteryReading, PositionEstimate, TelemetryState
from flightstack.vehicle_model import (
    BatterySpec,
    EnergyEstimator,
    MulticopterModel,
    VehicleProfile,
)
from gdacs_provider import (
    CachedGDACSProvider,
    GDACSEvent,
    GDACSMissionImpactAssessor,
    GDACSSnapshot,
    GDACSUnavailable,
)
from geo_utils import point_to_segment_distance_km
from sitl_mission_demo import blocking_zone_across


class _StubLink:
    """Deterministic in-memory MAVLink stub for controlled non-SITL experiments."""

    def __init__(self, remaining_pct: int | None = 95, bat_age_s: float = 0.0):
        self.telemetry = TelemetryState()
        now = self.telemetry.clock()
        self.telemetry.position.value = PositionEstimate(
            lat=26.9127,
            lon=75.8010,
            alt_amsl_m=491.0,
            alt_rel_m=60.0,
            vn_ms=0.0,
            ve_ms=0.0,
            vd_ms=0.0,
            heading_deg=180.0,
            autopilot_time_ms=1000,
        )
        self.telemetry.position.rx_time = now
        if remaining_pct is not None:
            self.telemetry.battery.value = BatteryReading(
                voltage_v=24.2,
                current_a=28.5,
                remaining_pct=remaining_pct,
                consumed_mah=800.0,
            )
            self.telemetry.battery.rx_time = now - bat_age_s
        else:
            self.telemetry.battery.value = None
        self.telemetry.heartbeat.value = ("AUTO", True)
        self.telemetry.heartbeat.rx_time = now
        self.telemetry.mav_type_id = 13
        self.telemetry.mav_type = "HEXAROTOR"
        self._is_live_socket = True
        self.conn = None
        self.uploaded_missions: List[ Any ] = []
        self.mode_history: List[str] = ["AUTO"]

    def wait_ready(self, timeout: float = 45.0) -> None:
        return None

    def upload_mission(self, items, verify: bool = True) -> dict:
        self.uploaded_missions.append(list(items))
        return {"verified": bool(verify), "count": len(items)}

    def set_mode(self, mode: str) -> None:
        self.mode_history.append(mode)
        armed = self.telemetry.armed
        self.telemetry.heartbeat.value = (mode, armed)
        self.telemetry.heartbeat.rx_time = self.telemetry.clock()

    def arm(self, timeout: float = 15.0) -> None:
        mode = self.telemetry.mode or "GUIDED"
        self.telemetry.heartbeat.value = (mode, True)
        self.telemetry.heartbeat.rx_time = self.telemetry.clock()

    def start_mission(self) -> None:
        self.set_mode("AUTO")

    def set_current_item(self, seq: int) -> None:
        self.telemetry.mission_current.value = int(seq)
        self.telemetry.mission_current.rx_time = self.telemetry.clock()


def exp1_route_planning_with_and_without_restricted_areas() -> Dict[str, Any]:
    c = JAIPUR_SINGLE_UAV
    home = (c.depot.lat, c.depot.lon)
    profile = MissionProfile(delivery_mode="auto", cruise_speed_ms=10.0, safety_margin_m=50.0)

    plan_free = build_delivery_mission(home, c.deliveries, zones=[], profile=profile)
    block_poly = blocking_zone_across(home, (c.deliveries[0].lat, c.deliveries[0].lon), half_km=0.35)
    zones_with_block = [list(z) for z in c.no_fly_zones] + [block_poly]
    plan_restricted = build_delivery_mission(home, c.deliveries, zones=zones_with_block, profile=profile)

    validate_polyline(
        plan_restricted.full_route_polyline(home),
        zones_with_block,
        profile.safety_margin_m,
    )
    route_pts = plan_restricted.full_route_polyline(home)
    min_clearance_km = float("inf")
    for i in range(len(route_pts) - 1):
        a, b = route_pts[i], route_pts[i + 1]
        for poly in zones_with_block:
            n = len(poly)
            for j in range(n):
                v1, v2 = poly[j], poly[(j + 1) % n]
                min_clearance_km = min(
                    min_clearance_km,
                    point_to_segment_distance_km(v1, a, b),
                    point_to_segment_distance_km(a, v1, v2),
                    point_to_segment_distance_km(b, v1, v2),
                )
    min_clearance_m = round(min_clearance_km * 1000.0, 2)
    leg1_free_km = round(_polyline_km(plan_free.leg_paths[0]), 3)
    leg1_restricted_km = round(_polyline_km(plan_restricted.leg_paths[0]), 3)
    detour_items = [it for it in plan_restricted.items if it.role in ("detour", "detour_home")]
    return {
        "experiment": "EXP-01_Route_Planning_With_vs_Without_Restricted_Areas",
        "passed": bool(
            plan_restricted.planned_km > plan_free.planned_km
            and len(detour_items) > 0
            and len(plan_restricted.unreachable) == 0
            and min_clearance_m >= profile.safety_margin_m
        ),
        "unrestricted_planned_km": round(plan_free.planned_km, 3),
        "restricted_planned_km": round(plan_restricted.planned_km, 3),
        "detour_distance_delta_km": round(plan_restricted.planned_km - plan_free.planned_km, 3),
        "leg1_unrestricted_km": leg1_free_km,
        "leg1_restricted_km": leg1_restricted_km,
        "unrestricted_mission_items": len(plan_free.items),
        "restricted_mission_items": len(plan_restricted.items),
        "detour_waypoints_inserted": len(detour_items),
        "safety_margin_enforced_m": profile.safety_margin_m,
        "min_measured_clearance_m": min_clearance_m,
    }


def exp2_mission_feasibility_gates() -> Dict[str, Any]:
    c = JAIPUR_SINGLE_UAV
    home = (c.depot.lat, c.depot.lon)
    vp = VehicleProfile()
    bs = BatterySpec()
    ee = EnergyEstimator(vp, bs)

    base_profile = MissionProfile(delivery_mode="auto", cruise_speed_ms=10.0, vehicle_model=vp)
    plan_nom = build_delivery_mission(home, c.deliveries, [list(z) for z in c.no_fly_zones], base_profile)
    leg_kms = [_polyline_km(lp) for lp in plan_nom.leg_paths]
    nom_energy = ee.evaluate_route(
        plan_nom.planned_km,
        [(plan_nom.stops[0], 3.5), (plan_nom.stops[1], 1.5)],
        cruise_speed_ms=10.0,
        hover_time_s=base_profile.hover_hold_s,
        leg_km_list=leg_kms,
    )

    cases: Dict[str, Any] = {}
    # Case A: Nominal mission passes
    runner_nom = MissionRunner(_StubLink(remaining_pct=100), plan_nom, [], base_profile, home, log=EventLog(echo=False))
    res_nom = runner_nom.validate_initial_mission(require_battery_telemetry=True)
    cases["nominal_3_5kg_100pct_soc"] = {
        "feasible": res_nom["feasible"],
        "total_energy_wh": round(nom_energy.total_energy_wh, 2),
        "usable_energy_wh": round(nom_energy.usable_energy_wh, 2),
        "leg_payloads_kg": [lg.payload_carried_kg for lg in nom_energy.legs],
        "leg_cruise_powers_w": [round(lg.cruise_power_w, 1) for lg in nom_energy.legs],
    }

    # Case B: Payload capacity breach (9.0 kg > 8.0 kg max_payload_kg)
    stop_heavy = Location(1, "Overload Stop", 26.8850, 75.7930, demand_kg=9.0)
    plan_heavy = build_delivery_mission(home, [stop_heavy], [], base_profile)
    runner_heavy = MissionRunner(_StubLink(remaining_pct=100), plan_heavy, [], base_profile, home, log=EventLog(echo=False))
    try:
        runner_heavy.validate_initial_mission(require_battery_telemetry=True)
        cases["payload_capacity_breach_9kg"] = {"rejected": False}
    except MissionValidationError as e:
        cases["payload_capacity_breach_9kg"] = {"rejected": True, "error": str(e)}

    # Case C: MTOW breach (altitude/regulatory MTOW derated to 15.0 kg while cargo bay holds 7.0 kg <= 8.0 kg)
    vp_mtow = VehicleProfile(empty_mass_kg=5.5, battery_mass_kg=3.2, max_payload_kg=8.0, mtow_kg=17.5)
    vp_mtow.mtow_kg = 15.0  # 5.5 + 3.2 + 7.0 = 15.7 kg > 15.0 kg MTOW
    stop_mtow = Location(1, "MTOW Stop", 26.8850, 75.7930, demand_kg=7.0)
    prof_mtow = MissionProfile(delivery_mode="hover", cruise_speed_ms=10.0, vehicle_model=vp_mtow)
    plan_mtow = build_delivery_mission(home, [stop_mtow], [], prof_mtow)
    runner_mtow = MissionRunner(_StubLink(remaining_pct=100), plan_mtow, [], prof_mtow, home, log=EventLog(echo=False))
    try:
        runner_mtow.validate_initial_mission(require_battery_telemetry=True)
        cases["mtow_breach"] = {"rejected": False}
    except MissionValidationError as e:
        cases["mtow_breach"] = {"rejected": True, "error": str(e)}

    # Case D: Battery reserve breach (21% SoC -> only 3.55 Wh remaining above 20% floor < 39.2 Wh required)
    runner_low_bat = MissionRunner(_StubLink(remaining_pct=21), plan_nom, [], base_profile, home, log=EventLog(echo=False))
    try:
        runner_low_bat.validate_initial_mission(require_battery_telemetry=True)
        cases["low_battery_21pct_soc"] = {"rejected": False}
    except MissionValidationError as e:
        cases["low_battery_21pct_soc"] = {"rejected": True, "error": str(e)}

    # Case E: Missing battery telemetry (must never default to 100% SoC)
    runner_no_bat = MissionRunner(_StubLink(remaining_pct=None), plan_nom, [], base_profile, home, log=EventLog(echo=False))
    try:
        runner_no_bat.validate_initial_mission(require_battery_telemetry=True)
        cases["missing_battery_telemetry"] = {"rejected": False}
    except MissionValidationError as e:
        cases["missing_battery_telemetry"] = {"rejected": True, "error": str(e)}

    # Case F: Time-based cold-chain breach (cold_chain_limit_minutes=2.0 min < ~5 min transit)
    stop_cc = Location(1, "Cold Chain Stop", 26.8600, 75.7930, demand_kg=1.5, cold_chain_limit_minutes=2.0, window_minutes=60.0)
    plan_cc = build_delivery_mission(home, [stop_cc], [], base_profile)
    runner_cc = MissionRunner(_StubLink(remaining_pct=100), plan_cc, [], base_profile, home, log=EventLog(echo=False))
    try:
        runner_cc.validate_initial_mission(require_battery_telemetry=True)
        cases["cold_chain_limit_breach"] = {"rejected": False}
    except MissionValidationError as e:
        cases["cold_chain_limit_breach"] = {"rejected": True, "error": str(e)}

    # Case G: Delivery deadline window breach (window_minutes=2.0 min < ~5 min transit)
    stop_win = Location(1, "Deadline Stop", 26.8600, 75.7930, demand_kg=1.5, cold_chain_limit_minutes=60.0, window_minutes=2.0)
    plan_win = build_delivery_mission(home, [stop_win], [], base_profile)
    runner_win = MissionRunner(_StubLink(remaining_pct=100), plan_win, [], base_profile, home, log=EventLog(echo=False))
    try:
        runner_win.validate_initial_mission(require_battery_telemetry=True)
        cases["delivery_window_breach"] = {"rejected": False}
    except MissionValidationError as e:
        cases["delivery_window_breach"] = {"rejected": True, "error": str(e)}

    all_passed = (
        cases["nominal_3_5kg_100pct_soc"]["feasible"]
        and all(cases[k]["rejected"] for k in cases if k != "nominal_3_5kg_100pct_soc")
    )
    return {
        "experiment": "EXP-02_Mission_Feasibility_Gates_MTOW_Battery_ColdChain_Deadline",
        "passed": all_passed,
        "cases": cases,
    }


def exp3_sitl_obstacle_avoidance_and_reference_logs() -> Dict[str, Any]:
    out_dir = os.path.join(HERE, "output")
    required_files = (
        "sitl_run_20261108_034446.jsonl",  # Two-stop reference mission
        "sitl_run_20261108_035408.jsonl",  # One-stop controlled OA baseline
        "sitl_run_20261108_035631.jsonl",  # One-stop controlled OA obstacle-present
    )
    logs = sorted(glob.glob(os.path.join(out_dir, "sitl_run_*.jsonl")))
    summaries = []
    by_filename: Dict[str, Dict[str, Any]] = {}

    for path in logs:
        fname = os.path.basename(path)
        summary_evt = None
        autopilot_evt = None
        sitl_evt = None
        try:
            with open(path, "r") as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    obj = json.loads(line)
                    if obj.get("event") == "summary":
                        summary_evt = obj
                    elif obj.get("event") == "autopilot_ready":
                        autopilot_evt = obj
                    elif obj.get("event") == "sitl_started":
                        sitl_evt = obj
        except (OSError, json.JSONDecodeError):
            summary_evt = None

        if summary_evt is not None:
            tel = (autopilot_evt or {}).get("telemetry", {})
            bat_init = ((tel.get("battery") or {}).get("value") or {}).get("voltage_v")
            rec = {
                "log_file": fname,
                "sitl_model": (sitl_evt or {}).get("model", "+"),
                "mav_type": summary_evt.get("mav_type") or tel.get("mav_type"),
                "initial_voltage_v": bat_init,
                "success": summary_evt.get("success"),
                "delivered": summary_evt.get("delivered"),
                "failed_deliveries": summary_evt.get("failed_deliveries", []),
                "requested_delivery_mode": summary_evt.get("requested_delivery_mode"),
                "delivery_modes_executed": summary_evt.get("delivery_modes_executed"),
                "proximity_sensor_active": summary_evt.get("proximity_sensor_active"),
                "proximity_active_frames": summary_evt.get("proximity_active_frames", 0),
                "ardupilot_oa_db_confirmations": summary_evt.get("ardupilot_oa_db_confirmations", 0),
                "max_oa_deviation_m": summary_evt.get("max_oa_deviation_m", 0.0),
                "measured_oa_lateral_deviation_observed": summary_evt.get(
                    "measured_oa_lateral_deviation_observed",
                    summary_evt.get("proven_obstacle_avoidance"),
                ),
                "final_battery": summary_evt.get("final_battery"),
                "planned_km": summary_evt.get("planned_km"),
            }
            summaries.append(rec)
            by_filename[fname] = rec

    ref_2stop = by_filename.get("sitl_run_20261108_034446.jsonl")
    base_1stop = by_filename.get("sitl_run_20261108_035408.jsonl")
    obs_1stop = by_filename.get("sitl_run_20261108_035631.jsonl")

    ref_2stop_ok = (
        ref_2stop is not None
        and ref_2stop.get("sitl_model") == "hexa:med_drone_hexa_16.json"
        and ref_2stop.get("mav_type") == "HEXAROTOR"
        and ref_2stop.get("success") is True
        and ref_2stop.get("delivered") == [1, 2]
        and ref_2stop.get("failed_deliveries") == []
        and ref_2stop.get("requested_delivery_mode") == "auto"
        and ref_2stop.get("delivery_modes_executed") == {"1": "hover", "2": "land"}
        and ref_2stop.get("proximity_sensor_active") is True
        and ref_2stop.get("proximity_active_frames", 0) > 0
        and ref_2stop.get("ardupilot_oa_db_confirmations", 0) > 0
        and float(ref_2stop.get("max_oa_deviation_m", 0.0)) > 10.0
        and ref_2stop.get("measured_oa_lateral_deviation_observed") is True
    )

    base_1stop_ok = (
        base_1stop is not None
        and base_1stop.get("sitl_model") == "hexa:med_drone_hexa_16.json"
        and base_1stop.get("mav_type") == "HEXAROTOR"
        and base_1stop.get("success") is True
        and base_1stop.get("delivered") == [1]
        and base_1stop.get("failed_deliveries") == []
        and base_1stop.get("proximity_sensor_active") is False
        and base_1stop.get("proximity_active_frames", -1) == 0
        and base_1stop.get("ardupilot_oa_db_confirmations", -1) == 0
        and float(base_1stop.get("max_oa_deviation_m", -1.0)) == 0.0
        and base_1stop.get("measured_oa_lateral_deviation_observed") is False
    )

    obs_1stop_ok = (
        obs_1stop is not None
        and obs_1stop.get("sitl_model") == "hexa:med_drone_hexa_16.json"
        and obs_1stop.get("mav_type") == "HEXAROTOR"
        and obs_1stop.get("success") is True
        and obs_1stop.get("delivered") == [1]
        and obs_1stop.get("failed_deliveries") == []
        and obs_1stop.get("proximity_sensor_active") is True
        and obs_1stop.get("proximity_active_frames", 0) > 0
        and obs_1stop.get("ardupilot_oa_db_confirmations", 0) > 0
        and float(obs_1stop.get("max_oa_deviation_m", 0.0)) > 10.0
        and obs_1stop.get("measured_oa_lateral_deviation_observed") is True
    )

    all_required_present = all(fn in by_filename for fn in required_files)
    passed = all_required_present and ref_2stop_ok and base_1stop_ok and obs_1stop_ok

    return {
        "experiment": "EXP-03_SITL_Obstacle_Avoidance_and_Mission_Log_Evidence",
        "passed": passed,
        "required_logs_present": all_required_present,
        "individual_checks": {
            "two_stop_reference_mission_034446": ref_2stop_ok,
            "one_stop_baseline_035408": base_1stop_ok,
            "one_stop_obstacle_present_035631": obs_1stop_ok,
        },
        "sitl_runs_analyzed": len(summaries),
        "runs": summaries,
    }


def exp4_changing_4d_restriction_and_inflight_replanning() -> Dict[str, Any]:
    c = JAIPUR_SINGLE_UAV
    home = (c.depot.lat, c.depot.lon)
    profile = MissionProfile(delivery_mode="auto", cruise_speed_ms=10.0, cruise_alt_m=60.0, vehicle_model=VehicleProfile())
    zones = [list(z) for z in c.no_fly_zones]
    plan = build_delivery_mission(home, c.deliveries, zones, profile)

    link = _StubLink(remaining_pct=88)
    runner = MissionRunner(link, plan, zones, profile, home, log=EventLog(echo=False))
    runner.upload(plan)
    runner.state.took_off = True
    runner.state.delivered.append(1)
    stop1 = c.deliveries[0]
    stop2 = c.deliveries[1]
    link.telemetry.position.value = PositionEstimate(
        lat=stop1.lat, lon=stop1.lon, alt_amsl_m=491.0, alt_rel_m=60.0,
        vn_ms=0.0, ve_ms=0.0, vd_ms=0.0, heading_deg=120.0, autopilot_time_ms=50000,
    )
    link.telemetry.position.rx_time = link.telemetry.clock()

    now_utc = datetime.now(timezone.utc)
    poly = blocking_zone_across((stop1.lat, stop1.lon), (stop2.lat, stop2.lon))

    # 4a. High-altitude restriction [150m, 300m] above 60m cruise -> no conflict, no replan needed
    high_zone = AirspaceZone(
        zone_id="TFR_HIGH_ALT",
        category="temporary_restriction",
        polygon=poly,
        floor_m_agl=150.0,
        ceiling_m_agl=300.0,
        valid_from=now_utc - timedelta(minutes=10),
        valid_to=now_utc + timedelta(minutes=30),
    )
    runner.apply_airspace_update(
        zones + [high_zone],
        reason="high_alt_no_conflict",
        source={"provider": "scenario_fixture", "provenance_classification": "SYNTHETIC_FIXTURE"},
        now=now_utc,
    )
    replans_after_high = len(runner.state.replans)

    # 4b. Active 4D TFR [0m, 120m] across Leg 2 -> triggers LOITER + A* detour + MAVLink upload
    active_tfr = AirspaceZone(
        zone_id="TFR_ACTIVE_4D",
        category="temporary_restriction",
        polygon=poly,
        floor_m_agl=0.0,
        ceiling_m_agl=120.0,
        valid_from=now_utc - timedelta(minutes=5),
        valid_to=now_utc + timedelta(minutes=45),
    )
    runner.apply_airspace_update(
        zones + [active_tfr],
        reason="active_4d_tfr",
        source={"provider": "scenario_fixture", "provenance_classification": "SYNTHETIC_FIXTURE"},
        now=now_utc,
    )
    replans_after_active = len(runner.state.replans)

    return {
        "experiment": "EXP-04_Changing_4D_Restriction_and_InFlight_Replanning",
        "passed": (replans_after_high == 0 and replans_after_active == 1 and runner.state.mission_version == 2),
        "high_altitude_zone_triggered_replan": replans_after_high > 0,
        "active_4d_tfr_triggered_replan": replans_after_active == 1,
        "new_mission_version": runner.state.mission_version,
        "replan_record": runner.state.replans[0] if runner.state.replans else None,
        "mode_sequence": link.mode_history,
    }


def exp5_hover_vs_landing_vs_abort_site_suitability() -> Dict[str, Any]:
    c = JAIPUR_SINGLE_UAV
    stop1_flooded = c.deliveries[0]
    stop2_clear = c.deliveries[1]
    stop3_steep = Location(
        3, "Steep Ridge Camp", 26.8900, 75.8100, demand_kg=1.0,
        site_suitability={
            "is_safe_for_landing": True,
            "surface_condition": "clear",
            "slope_deg": 9.5,  # > 7.0 deg limit
            "obstacle_clearance_m": 20.0,
            "recommended_mode": "land",
            "uncertainty": "known",
        },
    )
    stop4_abort = Location(
        4, "Dense Canopy Post", 26.8950, 75.8150, demand_kg=1.0,
        site_suitability={
            "is_safe_for_landing": False,
            "surface_condition": "debris",
            "slope_deg": 4.0,
            "obstacle_clearance_m": 6.0,  # < 10.0 m minimum clearance
            "recommended_mode": "hover",
            "uncertainty": "known",
        },
    )

    r1 = resolve_stop_delivery_mode(stop1_flooded, requested_mode="auto")
    r1_forced_land = resolve_stop_delivery_mode(stop1_flooded, requested_mode="land")
    r2 = resolve_stop_delivery_mode(stop2_clear, requested_mode="auto")
    r3 = resolve_stop_delivery_mode(stop3_steep, requested_mode="land")
    r4 = resolve_stop_delivery_mode(stop4_abort, requested_mode="auto")

    home = (c.depot.lat, c.depot.lon)
    plan = build_delivery_mission(
        home,
        [stop1_flooded, stop2_clear, stop3_steep, stop4_abort],
        zones=[],
        profile=MissionProfile(delivery_mode="auto", cruise_speed_ms=10.0),
    )

    return {
        "experiment": "EXP-05_Hover_vs_Landing_vs_Abort_Site_Suitability",
        "passed": (
            r1[0] is True and r1[1] == "hover"
            and r1_forced_land[0] is True and r1_forced_land[1] == "hover"
            and r2[0] is True and r2[1] == "land"
            and r3[0] is True and r3[1] == "hover"
            and r4[0] is False and r4[1] == "abort"
            and [s.id for s in plan.stops] == [1, 2, 3]
            and [s.id for s in plan.unreachable] == [4]
        ),
        "stop1_flooded_auto": {"ok": r1[0], "mode": r1[1], "reason": r1[2]},
        "stop1_flooded_explicit_land_downgraded": {"ok": r1_forced_land[0], "mode": r1_forced_land[1], "reason": r1_forced_land[2]},
        "stop2_clear_auto": {"ok": r2[0], "mode": r2[1], "reason": r2[2]},
        "stop3_slope_9_5deg_downgraded": {"ok": r3[0], "mode": r3[1], "reason": r3[2]},
        "stop4_clearance_6m_rejected_abort": {"ok": r4[0], "mode": r4[1], "reason": r4[2]},
        "planned_stop_ids": [s.id for s in plan.stops],
        "unreachable_stop_ids": [s.id for s in plan.unreachable],
    }


def exp6_failed_and_missing_actuator_feedback() -> Dict[str, Any]:
    stops = JAIPUR_SINGLE_UAV.deliveries
    # 6a. Causal confirmation + duplicate feedback tracking
    pm_ok = PayloadManager.from_stops(stops, mode="auto")
    pm_ok.command_release(1, "hover", t_rel_s=10.0)
    before_ack = pm_ok.get_package(1).delivery_state.value
    pm_ok.record_actuator_text("Gripper Released", t_rel_s=10.5)
    pm_ok.record_actuator_text("Gripper Released", t_rel_s=10.6)  # duplicate
    ok_comp, _, _ = pm_ok.complete_delivery(1, t_rel_s=11.0)
    pkg1 = pm_ok.get_package(1)

    # 6b. Missing feedback must block complete_delivery and fail_release transitions to FAILED
    pm_missing = PayloadManager.from_stops(stops, mode="auto")
    pm_missing.command_release(1, "hover", t_rel_s=20.0)
    confirm_without_ack, _, _ = pm_missing.complete_delivery(1, t_rel_s=25.0)
    pm_missing.fail_release(1, t_rel_s=25.1, reason="MISSING_ACTUATOR_FEEDBACK")
    pkg_missing = pm_missing.get_package(1)

    # 6c. Explicit fault STATUSTEXT ("Gripper fault") transitions package to FAILED
    pm_fault = PayloadManager.from_stops(stops, mode="auto")
    pm_fault.command_release(1, "hover", t_rel_s=30.0)
    pm_fault.record_actuator_text("Gripper fault: servo jammed", t_rel_s=30.4)
    pkg_fault = pm_fault.get_package(1)

    return {
        "experiment": "EXP-06_Failed_and_Missing_Actuator_Feedback",
        "passed": (
            before_ack == DeliveryState.RELEASE_COMMANDED.value
            and ok_comp is True
            and pkg1.delivery_state == DeliveryState.DELIVERY_COMPLETE
            and pkg1.duplicate_feedback_count == 1
            and pkg1.recipient_handover_verified is False
            and confirm_without_ack is False
            and pkg_missing.delivery_state == DeliveryState.FAILED
            and pkg_fault.delivery_state == DeliveryState.FAILED
        ),
        "causal_ack_case": {
            "state_before_statustext": before_ack,
            "final_state": pkg1.delivery_state.value,
            "release_provenance": pkg1.release_provenance,
            "duplicate_feedback_count": pkg1.duplicate_feedback_count,
            "recipient_handover_verified": pkg1.recipient_handover_verified,
        },
        "missing_feedback_case": {
            "confirm_without_ack_allowed": confirm_without_ack,
            "final_state": pkg_missing.delivery_state.value,
            "failure_reason": pkg_missing.error_reason,
        },
        "fault_statustext_case": {
            "final_state": pkg_fault.delivery_state.value,
            "failure_reason": pkg_fault.error_reason,
        },
    }


def exp7_stale_or_unavailable_external_data() -> Dict[str, Any]:
    now_utc = datetime.now(timezone.utc)
    z1 = AirspaceZone(
        zone_id="ACTIVE_TFR_01",
        category="temporary_restriction",
        polygon=[(26.88, 75.78), (26.88, 75.80), (26.90, 75.80), (26.90, 75.78)],
        valid_from=now_utc - timedelta(minutes=10),
        valid_to=now_utc + timedelta(minutes=30),
    )

    state = {"fail": False}

    class _OutageAirspaceProvider:
        def fetch(self, bbox=None):
            if state["fail"]:
                raise AirspaceUnavailable("Simulated upstream airspace API outage")
            return AirspaceSnapshot(
                provider="test_provider",
                source="fixture://airspace",
                zones=[z1],
                retrieved_at=now_utc,
                official=False,
                stale=False,
                content_sha256="0" * 64,
            )

    mon = DynamicAirspaceMonitor(_OutageAirspaceProvider())
    poll_ok = mon.check_for_updates(current_zones=[], now=now_utc)
    state["fail"] = True
    poll_outage = mon.check_for_updates(current_zones=poll_ok["active_zones"], now=now_utc + timedelta(minutes=1))

    # GDACS point-only vs validated polygon impact
    ev_point = GDACSEvent(
        event_key="EQ_1001_1",
        event_type="EQ",
        event_id=1001,
        episode_id=1,
        name="Point-Only Regional Earthquake",
        description="Point centroid only",
        alert_level="Orange",
        alert_score=2.0,
        is_current=True,
        country="India",
        iso3="IND",
        centroid=(26.90, 75.80),
        polygons=[],
        from_date=now_utc - timedelta(hours=2),
        to_date=now_utc + timedelta(hours=24),
        provenance=SourceClassification.SYNTHETIC_FIXTURE,
    )
    stop2 = JAIPUR_SINGLE_UAV.deliveries[1]
    d = 0.005
    flood_poly = [[
        (stop2.lat - d, stop2.lon - d),
        (stop2.lat - d, stop2.lon + d),
        (stop2.lat + d, stop2.lon + d),
        (stop2.lat + d, stop2.lon - d),
        (stop2.lat - d, stop2.lon - d),
    ]]
    ev_poly = GDACSEvent(
        event_key="FL_1002_1",
        event_type="FL",
        event_id=1002,
        episode_id=1,
        name="Validated Flood Polygon Over Stop 2",
        description="Polygon footprint",
        alert_level="Red",
        alert_score=3.0,
        is_current=True,
        country="India",
        iso3="IND",
        centroid=(stop2.lat, stop2.lon),
        polygons=flood_poly,
        from_date=now_utc - timedelta(hours=2),
        to_date=now_utc + timedelta(hours=24),
        provenance=SourceClassification.SYNTHETIC_FIXTURE,
    )

    class _StubGDACS:
        def fetch(self):
            return GDACSSnapshot(
                provider="GDACS_FIXTURE",
                source_url="fixture://gdacs",
                events=[ev_point, ev_poly],
                retrieved_at=now_utc,
                content_sha256="0" * 64,
                provenance=SourceClassification.SYNTHETIC_FIXTURE,
            )

    assessor = GDACSMissionImpactAssessor(_StubGDACS(), fetch_missing_geometries=False)
    impact = assessor.evaluate_mission_impact(stops=JAIPUR_SINGLE_UAV.deliveries)

    stop2_updates = [u for u in impact["stop_suitability_updates"] if u["stop_id"] == 2]
    return {
        "experiment": "EXP-07_Stale_or_Unavailable_External_Data_and_GDACS_Semantics",
        "passed": (
            poll_ok["safe_to_clear_restrictions"] is True
            and poll_outage["status"] == "SOURCE_UNAVAILABLE"
            and poll_outage["safe_to_clear_restrictions"] is False
            and len(poll_outage["active_zones"]) == 1
            and len(impact["advisory_events"]) == 1
            and impact["advisory_events"][0]["classification"] == "ADVISORY_REGIONAL_AWARENESS"
            and len(stop2_updates) == 1
            and stop2_updates[0]["new_suitability"]["recommended_mode"] == "hover"
        ),
        "airspace_outage_retains_active_restrictions": len(poll_outage["active_zones"]) == 1,
        "safe_to_clear_on_outage": poll_outage["safe_to_clear_restrictions"],
        "gdacs_point_only_classification": impact["advisory_events"][0]["classification"],
        "gdacs_polygon_stop2_downgraded_to": stop2_updates[0]["new_suitability"]["recommended_mode"] if stop2_updates else None,
    }


def exp8_mission_recovery_and_rtl_behaviour() -> Dict[str, Any]:
    c = JAIPUR_SINGLE_UAV
    home = (c.depot.lat, c.depot.lon)
    profile = MissionProfile(delivery_mode="auto", cruise_speed_ms=10.0, vehicle_model=VehicleProfile())
    plan = build_delivery_mission(home, c.deliveries, [list(z) for z in c.no_fly_zones], profile)

    link = _StubLink(remaining_pct=18)  # Below 20% battery reserve floor!
    runner = MissionRunner(link, plan, [], profile, home, battery_reserve_pct=20, log=EventLog(echo=False))
    runner.state.took_off = True
    runner.step()

    reserve_breach_events = [e for e in runner.log.events if e["event"] == "battery_reserve_breached"]
    rtl_commanded = "RTL" in link.mode_history

    # Simulate touchdown & disarm at home after all stops resolved
    runner.state.delivered = [s.id for s in c.deliveries]
    link.telemetry.position.value = PositionEstimate(
        lat=home[0], lon=home[1], alt_amsl_m=431.1, alt_rel_m=0.1,
        vn_ms=0.0, ve_ms=0.0, vd_ms=0.0, heading_deg=0.0, autopilot_time_ms=90000,
    )
    link.telemetry.position.rx_time = link.telemetry.clock()
    link.telemetry.heartbeat.value = ("RTL", False)
    link.telemetry.heartbeat.rx_time = link.telemetry.clock()
    runner.step()

    return {
        "experiment": "EXP-08_Mission_Recovery_Battery_Reserve_Failsafe_and_Disarm",
        "passed": (
            len(reserve_breach_events) == 1
            and rtl_commanded
            and runner.state.finished is True
        ),
        "battery_reserve_breach_detected": len(reserve_breach_events) == 1,
        "emergency_rtl_commanded": rtl_commanded,
        "mission_finished_on_disarm": runner.state.finished,
    }


def run_all_experiments() -> Dict[str, Any]:
    results = [
        exp1_route_planning_with_and_without_restricted_areas(),
        exp2_mission_feasibility_gates(),
        exp3_sitl_obstacle_avoidance_and_reference_logs(),
        exp4_changing_4d_restriction_and_inflight_replanning(),
        exp5_hover_vs_landing_vs_abort_site_suitability(),
        exp6_failed_and_missing_actuator_feedback(),
        exp7_stale_or_unavailable_external_data(),
        exp8_mission_recovery_and_rtl_behaviour(),
    ]
    report = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "total_experiments": len(results),
        "passed_experiments": sum(1 for r in results if r["passed"]),
        "all_passed": all(r["passed"] for r in results),
        "experiments": results,
    }
    out_path = os.path.join(HERE, "output", "experimental_validation_report.json")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(report, f, indent=2)
    return report


if __name__ == "__main__":
    rep = run_all_experiments()
    print(f"Experimental Validation Suite: {rep['passed_experiments']}/{rep['total_experiments']} passed")
    for exp in rep["experiments"]:
        status = "PASS" if exp["passed"] else "FAIL"
        print(f"  [{status}] {exp['experiment']}")
    sys.exit(0 if rep["all_passed"] else 1)
