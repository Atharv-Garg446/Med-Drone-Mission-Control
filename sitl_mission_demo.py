#!/usr/bin/env python3
"""
sitl_mission_demo.py
====================
End-to-end demonstration: existing planner -> MAVLink -> ArduCopter SITL.

  1. Solve the Jaipur scenario with the existing from-scratch VRP solver
     (A*-costed flight matrix, payload/range/time-window constraints).
  2. Convert the first sortie into an ArduCopter AUTO mission (A* detour
     waypoints, hover-and-release or land delivery steps, RTL) and validate
     it against the no-fly zones + safety margin.
  3. Launch ArduCopter SITL (real autopilot firmware, built-in physics),
     upload, read back, arm, fly; monitor autopilot telemetry.
  4. Optional (--airspace-change): after the first delivery, publish a
     deterministic new restricted zone across the next leg. The runner
     detects the conflict, commands LOITER, re-plans from the autopilot's
     position estimate, uploads the new mission and resumes AUTO.

All events are logged to output/sitl_run_<timestamp>.jsonl.

Usage:
    python3 sitl_mission_demo.py                       # hover delivery, 1 sortie
    python3 sitl_mission_demo.py --airspace-change     # + mid-flight airspace update
    python3 sitl_mission_demo.py --mode land --speedup 5
Requires: pymavlink (requirements-sitl.txt) and a built ArduCopter SITL
binary (docs/SITL_SETUP.md).
"""

import argparse
from datetime import datetime, timedelta, timezone
import math
import os
import sys
import time

from airspace_provider import (
    AirspaceZone,
    DigitalSkyIntersectionChecker,
    MapplsAirspaceLayerSpec,
    ScenarioProvider,
)
from config import CITY_CONFIGS, JAIPUR_SINGLE_UAV
from distance_matrix import build_flight_distance_matrix
from gdacs_provider import (
    CachedGDACSProvider,
    GDACSMissionImpactAssessor,
    GDACSProvider,
    GDACSUnavailable,
)
from vrp_scratch import solve_vrp_from_scratch
from geo_utils import haversine_km
from flightstack.mission import (
    MissionProfile,
    MissionValidationError,
    build_delivery_mission,
    mission_to_qgc_wpl,
    _polyline_km,
)
from flightstack.mission_runner import EventLog, MissionRunner, plan_summary
from flightstack.mavlink_link import AutopilotLink
from flightstack.sitl import SitlProcess, SitlNotAvailable
from flightstack.vehicle_model import VehicleProfile, BatterySpec, EnergyEstimator

HERE = os.path.dirname(os.path.abspath(__file__))
HOME_ALT_AMSL = 431.0     # Jaipur ground elevation used for SITL home (approx.)


def blocking_zone_across(a, b, half_km=0.35):
    """Square zone centred on the midpoint of leg a->b (deterministic)."""
    mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
    dlat = half_km / 111.0
    dlon = half_km / (111.0 * math.cos(math.radians(mid[0])))
    return [(mid[0] - dlat, mid[1] - dlon), (mid[0] - dlat, mid[1] + dlon),
            (mid[0] + dlat, mid[1] + dlon), (mid[0] + dlat, mid[1] - dlon)]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", choices=sorted(CITY_CONFIGS.keys()), default="jaipur_single_uav",
                    help="scenario configuration (default: dedicated single-UAV reference)")
    ap.add_argument("--mode", choices=["auto", "hover", "land"], default="auto",
                    help="delivery mode selection policy: 'auto' uses per-stop site suitability recommended_mode")
    ap.add_argument("--speedup", type=float, default=5.0)
    ap.add_argument("--route", type=int, default=0, help="which solved sortie to fly (multi-drone scenarios)")
    ap.add_argument("--max-stops", type=int, default=3)
    ap.add_argument("--airspace-change", action="store_true")
    ap.add_argument("--inject-obstacle", action="store_true", help="simulate dynamic 3D aerial obstacle detection and avoidance")
    ap.add_argument("--enable-lidar", action="store_true", help="enable simulated LD06 360-degree serial lidar on SITL")
    ap.add_argument("--timeout", type=float, default=900.0, help="wall-clock seconds")
    ap.add_argument("--connect", default=None, help="use an already running autopilot URL")
    args = ap.parse_args(argv)

    city = CITY_CONFIGS.get(args.scenario, JAIPUR_SINGLE_UAV)
    snap = ScenarioProvider(city).fetch()
    zones = list(snap.zones)
    drone = city.drone
    home = (city.depot.lat, city.depot.lon)
    t0 = time.perf_counter()

    if drone.fleet_size == 1:
        # Dedicated single-UAV primary workflow: no multi-drone fleet solver coupling
        stops = list(city.deliveries)[: args.max_stops]
        routes = [[0] + [s.id for s in stops] + [0]]
    else:
        locs = city.all_locations
        matrix, _ = build_flight_distance_matrix(locs, [z.polygon for z in zones], drone_config=drone)
        routes = solve_vrp_from_scratch(locs, matrix, drone)
        route = routes[args.route]
        stops = [locs[i] for i in route if i != 0][: args.max_stops]

    solve_ms = (time.perf_counter() - t0) * 1000
    vp = VehicleProfile(max_payload_kg=drone.capacity_kg)
    bs = BatterySpec(reserve_fraction=drone.reserve_fraction)
    profile = MissionProfile(delivery_mode=args.mode, safety_margin_m=drone.safety_margin_m,
                             cruise_speed_ms=10.0, vehicle_model=vp)

    os.makedirs(os.path.join(HERE, "output"), exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    log = EventLog(os.path.join(HERE, "output", "sitl_run_%s.jsonl" % stamp))

    # 1. Query external GDACS disaster alert feed (with caching & operational impact assessment)
    #    BEFORE building and uploading the initial mission so relevant hazard polygons/site updates
    #    take effect on initial planning rather than merely being logged.
    gdacs_cache_path = os.path.join(HERE, "output", "gdacs_cache_latest.json")
    gdacs_cached_provider = CachedGDACSProvider(
        GDACSProvider(event_types=("FL", "EQ", "TC"), timeout_s=6.0),
        cache_path=gdacs_cache_path,
    )
    gdacs_assessor = GDACSMissionImpactAssessor(
        gdacs_cached_provider,
        relevance_radius_km=250.0,
        fetch_missing_geometries=True,
        avoid_red_hazard_polygons=True,
    )
    gdacs_summary = None
    try:
        gdacs_snap = gdacs_cached_provider.fetch()
        gdacs_summary = gdacs_snap.metadata()
        gdacs_summary["sample_events"] = [
            {
                "event_id": ev.event_id,
                "event_type": ev.event_type,
                "name": ev.name,
                "country": ev.country,
                "alert_level": ev.alert_level,
            }
            for ev in gdacs_snap.events[:3]
        ]
    except GDACSUnavailable as gdacs_err:
        gdacs_summary = {
            "provider": "GDACS",
            "provenance_classification": "LIVE_EXTERNAL",
            "available": False,
            "error": str(gdacs_err),
            "attribution": "Global Disaster Awareness and Coordination System, GDACS.",
        }

    # Pre-plan GDACS evaluation & idempotent application to stops and zones
    gdacs_pre_impact = gdacs_assessor.evaluate_mission_impact(
        stops=stops, leg_paths=None, home=home
    )
    gdacs_pre_applied = GDACSMissionImpactAssessor.apply_impact_to_stops_and_zones(
        gdacs_pre_impact, stops, zones
    )
    zones = gdacs_pre_applied["zones"]

    plan = build_delivery_mission(home, stops, zones, profile,
                                  start_time=datetime.now(timezone.utc))

    # Re-evaluate against the actual A*-detoured leg polylines and apply any newly intersected hazard polygons
    gdacs_impact = gdacs_assessor.evaluate_mission_impact(
        stops=plan.stops, leg_paths=plan.leg_paths, home=home
    )
    gdacs_post_applied = GDACSMissionImpactAssessor.apply_impact_to_stops_and_zones(
        gdacs_impact, stops, zones
    )
    zones = gdacs_post_applied["zones"]
    if gdacs_post_applied["changed"]:
        plan = build_delivery_mission(home, stops, zones, profile,
                                      start_time=datetime.now(timezone.utc))
    gdacs_impact["pre_mission_applied"] = {
        "changed": bool(gdacs_pre_applied["changed"] or gdacs_post_applied["changed"]),
        "updated_stop_ids": sorted(set(gdacs_pre_applied["updated_stop_ids"] + gdacs_post_applied["updated_stop_ids"])),
        "added_zone_ids": sorted(set(gdacs_pre_applied["added_zone_ids"] + gdacs_post_applied["added_zone_ids"])),
    }

    with open(os.path.join(HERE, "output", "sitl_mission_%s.waypoints" % stamp), "w") as f:
        f.write(mission_to_qgc_wpl(plan))

    # 2. Pre-flight physical energy, MTOW & cargo validation on the reference UAV
    ee = EnergyEstimator(vp, bs)
    rem_cargo = sum(s.demand_kg for s in plan.stops)
    running_cargo = rem_cargo
    stops_payload = []
    for s in plan.stops:
        stops_payload.append((s, running_cargo))
        running_cargo = max(0.0, running_cargo - float(getattr(s, "demand_kg", 0.0) or 0.0))
    leg_km_list = [_polyline_km(lp) for lp in plan.leg_paths] if plan.leg_paths else None
    energy_report = ee.evaluate_route(
        plan.planned_km,
        stops_payload,
        cruise_speed_ms=profile.cruise_speed_ms or 10.0,
        hover_time_s=profile.hover_hold_s,
        leg_km_list=leg_km_list,
    )

    # 3. Check DigitalSky OpenAPI 3.0 endpoint across the ENTIRE planned route (honestly gated unless pilot JWT is set)
    ds_checker = DigitalSkyIntersectionChecker()
    full_route_pts = plan.full_route_polyline(home)
    ds_result = ds_checker.validate_route_corridor(
        full_route_pts,
        alt_m_agl=profile.cruise_alt_m,
    )
    mappls_spec = MapplsAirspaceLayerSpec().to_dict()

    log.log("planned", scenario=city.name, single_uav=(drone.fleet_size == 1),
            solver_ms=round(solve_ms, 1),
            all_routes=routes, flown_stops=[s.id for s in stops],
            requested_delivery_mode=args.mode,
            selected_stop_modes=dict(plan.selected_modes),
            airspace_snapshot={
                "provider": snap.provider,
                "official": snap.official,
                "provenance_classification": snap.provenance_classification,
                "n_zones": len(zones),
            },
            digitalsky_validation={
                "checked": ds_result.checked,
                "intersects": ds_result.intersects,
                "status_code": (ds_result.raw or {}).get("status", "UNAUTHORISED_NO_PILOT_TOKEN"),
                "official_clearance": ds_result.official_clearance,
                "integration_status": "VERIFIED_LIVE" if ds_result.checked else "BLOCKED_MISSING_DGCA_PILOT_JWT",
                "waypoints_checked": len(full_route_pts),
                "dynamic_airspace_monitor_active": False,
                "provenance_classification": ds_result.provenance_classification,
                "detail": ds_result.detail,
            },
            mappls_layer_metadata=mappls_spec,
            gdacs_disaster_feed=gdacs_summary,
            gdacs_mission_impact=gdacs_impact,
            energy_validation={
                "vehicle": vp.name,
                "num_rotors": vp.num_rotors,
                "battery_cells": bs.cell_count,
                "nominal_voltage_v": bs.nominal_voltage_v,
                "capacity_mah": bs.capacity_mah,
                "empty_mass_kg": vp.empty_mass_kg,
                "battery_mass_kg": vp.battery_mass_kg,
                "initial_cargo_kg": round(rem_cargo, 2),
                "gross_takeoff_mass_kg": round(vp.empty_mass_kg + vp.battery_mass_kg + rem_cargo, 2),
                "max_payload_kg": vp.max_payload_kg,
                "mtow_kg": vp.mtow_kg,
                "total_energy_wh": round(energy_report.total_energy_wh, 2),
                "usable_energy_wh": round(energy_report.usable_energy_wh, 2),
                "reserve_margin_pct": round(energy_report.reserve_margin_pct, 1),
                "is_feasible": energy_report.is_feasible,
                "legs": [
                    {
                        "from": lg.from_index,
                        "to": lg.to_index,
                        "distance_km": round(lg.distance_km, 3),
                        "payload_carried_kg": round(lg.payload_carried_kg, 2),
                        "cruise_power_w": round(lg.cruise_power_w, 1),
                        "energy_wh": round(lg.energy_wh, 2),
                    }
                    for lg in energy_report.legs
                ],
                "cold_chain_check_type": "time_based_cold_chain_limit_minutes",
            },
            mission=plan_summary(plan))

    try:
        if not energy_report.is_feasible:
            log.log(
                "initial_mission_rejected",
                why=energy_report.failure_reason or "Initial route energy exceeds usable battery capacity",
                breach_type="energy_reserve",
            )
            log.close()
            return 4
        pre_validator = MissionRunner(
            None, plan, zones, profile, home, log=log, digitalsky_checker=ds_checker
        )
        pre_validator.validate_initial_mission(require_battery_telemetry=False)
    except MissionValidationError as e:
        log.log("initial_mission_failed", error=str(e))
        log.close()
        return 4

    sitl = None
    link = None
    try:
        if args.connect:
            url = args.connect
        else:
            sitl = SitlProcess(home=(home[0], home[1], HOME_ALT_AMSL, 0), speedup=args.speedup,
                               enable_lidar=args.enable_lidar,
                               extra_defaults=[os.path.join(HERE, "flightstack", "params",
                                                            "sitl_delivery.parm")]).start()
            url = sitl.url
            log.log("sitl_started", url=url, binary=sitl.binary, model=sitl.model,
                    speedup=args.speedup, workdir=sitl.workdir)
        link = AutopilotLink.connect(url)
        # Note: In headless SITL, obstacle sensing and avoidance are handled by ArduPilot's
        # native C++ AP_Proximity_SITL (PRX1_TYPE=10) + AP_OABendyRuler. The Python
        # ObstacleTracker / PerceptionPipeline / PrecisionLandingController are standalone
        # unit-tested modules and are not wired as active sensors in this headless demo.
        runner = MissionRunner(link, plan, zones, profile, home, log=log,
                               obstacle_tracker=None,
                               native_oa_enabled=bool(args.inject_obstacle),
                               gdacs_assessor=gdacs_assessor,
                               digitalsky_checker=ds_checker)
        runner.launch()

        if args.inject_obstacle and plan.leg_paths and len(plan.leg_paths[0]) >= 2:
            from flightstack.obstacle_tracker import geo_to_ned, ned_to_geo
            p0, p1 = plan.leg_paths[0][0], plan.leg_paths[0][-1]
            # Place obstacle at 35% of Leg 1, offset 60 m perpendicular to the leg with
            # radius 80 m (protruding 20 m across the flight path, chord 105.8 m > 100 m
            # so AP_Proximity_SITL's [0, 200m] binary search bisects the front boundary).
            mid_lat = p0[0] + 0.35 * (p1[0] - p0[0])
            mid_lon = p0[1] + 0.35 * (p1[1] - p0[1])
            a_ned = geo_to_ned(p0[0], p0[1], 0.0, home[0], home[1])
            b_ned = geo_to_ned(p1[0], p1[1], 0.0, home[0], home[1])
            m_ned = geo_to_ned(mid_lat, mid_lon, 0.0, home[0], home[1])
            ux, uy = b_ned[0] - a_ned[0], b_ned[1] - a_ned[1]
            ulen = math.hypot(ux, uy) or 1.0
            # Unit normal perpendicular to leg (-uy, ux)
            nx, ny = -uy / ulen, ux / ulen
            obs_n = m_ned[0] + nx * 60.0
            obs_e = m_ned[1] + ny * 60.0
            obs_lat, obs_lon, _ = ned_to_geo(obs_n, obs_e, 0.0, home[0], home[1])
            obs_radius_m = 80.0
            runner.register_world_obstacle(obs_lat, obs_lon, obs_radius_m, name="OBSTACLE_TOWER_01")
            n_obs = link.upload_fence_obstacles(home, [(obs_lat, obs_lon, obs_radius_m)])
            log.log(
                "world_obstacle_configured",
                obstacle_id="OBSTACLE_TOWER_01",
                lat=round(obs_lat, 7),
                lon=round(obs_lon, 7),
                radius_m=obs_radius_m,
                lateral_offset_m=60.0,
                path_protrusion_m=20.0,
                n_world_obstacles=n_obs,
                source_classification="SIMULATED_SENSOR",
                sensor_backend="AP_Proximity_SITL_PRX1_TYPE_10",
                note=(
                    "Cylindrical world obstacle uploaded to ArduPilot AC_PolyFence_loader; "
                    "C++ AP_Proximity_SITL (PRX1_TYPE=10) raycasts 8 sectors against it in real time"
                ),
            )

        injected = {"airspace_done": not args.airspace_change}

        def hook(r: MissionRunner):
            pending = r.pending_stops()
            if not pending:
                return

            last = next((s for s in r.plan.stops if s.id == r.state.delivered[-1]), None) if r.state.delivered else None
            nxt = pending[0]

            if not injected["airspace_done"] and r.state.delivered:
                poly = blocking_zone_across((last.lat, last.lon), (nxt.lat, nxt.lon))
                now_utc = datetime.now(timezone.utc)
                tfr_4d = AirspaceZone(
                    zone_id="TFR_POPUP_4D_01",
                    category="temporary_restriction",
                    polygon=poly,
                    name="Simulated Emergency 4D TFR",
                    floor_m_agl=0.0,
                    ceiling_m_agl=120.0,
                    valid_from=now_utc - timedelta(minutes=5),
                    valid_to=now_utc + timedelta(minutes=45),
                    properties={"provenance": "scenario_fixture", "official": False},
                )
                injected["airspace_done"] = True
                r.apply_airspace_update(r.zones + [tfr_4d], reason="scenario_4d_airspace_change",
                                        source={"provider": "scenario_fixture",
                                                "provenance_classification": "SYNTHETIC_FIXTURE",
                                                "kind": "simulated_temporary_restriction_4d",
                                                "zone_id": tfr_4d.zone_id,
                                                "floor_m_agl": tfr_4d.floor_m_agl,
                                                "ceiling_m_agl": tfr_4d.ceiling_m_agl,
                                                "valid_from": tfr_4d.valid_from.isoformat(),
                                                "valid_to": tfr_4d.valid_to.isoformat(),
                                                "official": False,
                                                "note": "deterministic 4D test zone, not official data",
                                                "polygon": poly},
                                        now=now_utc)


        ok = runner.run_until(lambda r: r.state.finished, args.timeout, hook)
        tel = link.telemetry
        executed_modes = {
            sid: runner.payload.get_package(sid).release_mode
            for sid in runner.state.delivered
            if runner.payload.get_package(sid) is not None
        }
        log.log("summary",
                success=ok and set(runner.state.delivered) >= {s.id for s in runner.plan.stops} and not runner.state.failed_deliveries,
                vehicle_name=vp.name,
                mav_type=tel.mav_type,
                mav_type_id=tel.mav_type_id,
                delivered=runner.state.delivered,
                failed_deliveries=runner.state.failed_deliveries,
                requested_delivery_mode=args.mode,
                delivery_modes_executed=executed_modes,
                replans=runner.state.replans,
                stale_events=runner.state.stale_events, rejected=tel.rejected,
                proximity_sensor_active=bool(args.inject_obstacle),
                obstacle_entered_proximity_range=(runner.state.proximity_active_frames > 0),
                proximity_active_frames=runner.state.proximity_active_frames,
                ardupilot_oa_db_confirmations=runner.state.oa_db_confirmations,
                max_oa_deviation_m=runner.state.max_oa_deviation_m,
                min_obstacle_surface_clearance_m=runner.state.min_obstacle_surface_clearance_m,
                min_proximity_ray_distance_m=runner.state.min_proximity_ray_distance_m,
                measured_oa_lateral_deviation_observed=bool(
                    runner.state.proximity_active_frames > 0 and runner.state.max_oa_deviation_m >= 2.0
                ),
                dynamic_airspace_monitor_active=False,
                perception_pipeline_active_in_demo=False,
                precision_landing_active_in_demo=False,
                final_battery=tel.battery.value.__dict__ if tel.battery.value else None,
                planned_km=round(runner.plan.planned_km, 3))
        link.close()
        return 0 if ok else 2
    except MissionValidationError as e:
        log.log("initial_mission_failed", error=str(e))
        return 4
    except SitlNotAvailable as e:
        log.log("sitl_unavailable", error=str(e))
        return 3
    finally:
        if link:
            try:
                link.close()
            except Exception:
                pass
        if sitl:
            sitl.stop()
        log.close()


if __name__ == "__main__":
    sys.exit(main())
