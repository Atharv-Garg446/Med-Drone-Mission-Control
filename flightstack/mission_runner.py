"""
flightstack/mission_runner.py
=============================
Executes a ``MissionPlan`` on an ArduPilot autopilot and keeps the planner
state (which stops are delivered, which mission is active) in sync with
autopilot telemetry.

Separation of concerns:
  * global planning   -- VRP + A* in the existing modules (via mission.py)
  * mission execution -- this module: upload, start, monitor, update
  * flight control    -- ArduCopter AUTO mode (position control, EKF)

Every decision is written to a JSON-lines event log with wall and monotonic
timestamps so a run can be audited afterwards.
"""

from __future__ import annotations

from datetime import datetime, timezone
import json
import math
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence

from config import Location
from flightstack.mission import (MissionPlan, MissionProfile, MissionValidationError,
                                 build_delivery_mission, validate_polyline,
                                 point_in_restricted)
from flightstack.payload import (PayloadManager, PackageStatus, ReleaseSource,
                                 DeliveryState, SiteSuitability)
from flightstack.telemetry import StaleTelemetryError


class EventLog:
    def __init__(self, path: Optional[str] = None, echo: bool = True):
        self.path = path
        self.echo = echo
        self.events: List[dict] = []
        self._fh = open(path, "w") if path else None
        self._t0 = time.monotonic()

    def elapsed(self) -> float:
        return round(time.monotonic() - self._t0, 3)

    def log(self, event: str, **data) -> dict:
        t_rel = round(time.monotonic() - self._t0, 3)
        rec = {"t_wall": time.time(), "t_rel_s": t_rel, "t": t_rel,
               "event": event, **data}
        self.events.append(rec)
        if self._fh:
            self._fh.write(json.dumps(rec, default=str) + "\n")
            self._fh.flush()
        if self.echo and event != "telemetry":
            short = {k: v for k, v in data.items() if k not in ("mission", "telemetry")}
            print("[%8.2fs] %-22s %s" % (rec["t_rel_s"], event, json.dumps(short, default=str)[:160]))
        return rec

    def close(self):
        if self._fh:
            self._fh.close()


def plan_summary(plan: MissionPlan) -> dict:
    return {
        "stops": [s.id for s in plan.stops],
        "unreachable": [s.id for s in plan.unreachable],
        "planned_km": round(plan.planned_km, 3),
        "selected_modes": dict(getattr(plan, "selected_modes", {})),
        "site_assessments": dict(getattr(plan, "site_assessments", {})),
        "items": [{"seq": i.seq, "cmd": i.command, "role": i.role, "stop": i.stop_id,
                   "lat": round(i.lat, 7), "lon": round(i.lon, 7), "alt": i.alt_m}
                  for i in plan.items],
    }


@dataclass
class RunnerState:
    delivered: List[int] = field(default_factory=list)
    failed_deliveries: List[int] = field(default_factory=list)
    mission_version: int = 0
    replans: List[dict] = field(default_factory=list)
    stale_events: int = 0
    took_off: bool = False
    finished: bool = False
    max_oa_deviation_m: float = 0.0
    proximity_active_frames: int = 0
    oa_db_confirmations: int = 0
    min_obstacle_surface_clearance_m: Optional[float] = None
    min_proximity_ray_distance_m: Optional[float] = None


def _normalize_zones(zones) -> list:
    """Preserve AirspaceZone 4D records intact while copying raw 2D polygon lists, deduplicating idempotently."""
    if hasattr(zones, "zones"):
        zones = zones.zones
    out = []
    seen_zone_ids = set()
    seen_polys = set()
    for z in zones:
        if hasattr(z, "polygon"):
            zid = getattr(z, "zone_id", None)
            if zid is not None:
                if zid in seen_zone_ids:
                    continue
                seen_zone_ids.add(zid)
            out.append(z)
        else:
            poly_list = [tuple(pt) for pt in z]
            poly_key = tuple(poly_list)
            if poly_key in seen_polys:
                continue
            seen_polys.add(poly_key)
            out.append(list(z))
    return out


class MissionRunner:
    def __init__(self, link, plan: MissionPlan, zones, profile: MissionProfile,
                 home: Sequence[float], log: Optional[EventLog] = None,
                 stale_limit_s: float = 2.0, battery_reserve_pct: int = 20,
                 obstacle_tracker: Optional[object] = None,
                 payload: Optional[PayloadManager] = None,
                 native_oa_enabled: bool = False,
                 airspace_monitor: Optional[object] = None,
                 gdacs_assessor: Optional[object] = None,
                 digitalsky_checker: Optional[object] = None,
                 perception_pipeline: Optional[object] = None,
                 precision_landing_controller: Optional[object] = None,
                 stop_modes: Optional[Dict[int, str]] = None,
                 actuator_ack_timeout_s: float = 15.0):
        self.link = link
        self.plan = plan
        self.zones = _normalize_zones(zones)
        self.profile = profile
        self.home = (home[0], home[1])
        self.log = log or EventLog()
        self.stale_limit_s = stale_limit_s
        self.state = RunnerState()
        self._last_tlog = 0.0
        self._stale = False
        self.battery_reserve_pct = battery_reserve_pct
        self._reserve_warned = False
        self._n_texts = 0
        self._n_oa_db = 0
        self.obstacle_tracker = obstacle_tracker
        self.native_oa_enabled = native_oa_enabled
        self.airspace_monitor = airspace_monitor
        self.gdacs_assessor = gdacs_assessor
        self.digitalsky_checker = digitalsky_checker
        self.perception_pipeline = perception_pipeline
        self.precision_landing_controller = precision_landing_controller
        self.stop_modes: Dict[int, str] = dict(stop_modes or {})
        self.actuator_ack_timeout_s = float(actuator_ack_timeout_s)
        self._last_obstacle_eval = 0.0
        self._last_proximity_stream = 0.0
        self._last_prox_log = 0.0
        self._last_dev_log = 0.0
        self._last_rearm_attempt = 0.0
        self._last_external_poll = 0.0
        self._last_perception_poll = 0.0
        self._landed_stops_logged = set()
        self._dwell_started_stops = set()
        self._dwell_completed_stops = set()
        self._avoiding_obstacles = set()
        self._world_obstacles: List[tuple] = []
        all_stops = list(plan.stops) + list(getattr(plan, "unreachable", ()))
        self.payload = payload or PayloadManager.from_stops(
            all_stops,
            mode=profile.delivery_mode,
            stop_modes=getattr(plan, "selected_modes", None) or self.stop_modes,
        )
        self._sync_payload_modes(plan)

    def register_world_obstacle(self, lat: float, lon: float, radius_m: float, name: str = "OBSTACLE") -> None:
        """Register a simulated world cylinder obstacle to measure actual surface clearance along flown telemetry."""
        self._world_obstacles.append((float(lat), float(lon), float(radius_m), str(name)))

    def _compute_cross_track_m(self, lat: float, lon: float) -> float:
        """Perpendicular lateral distance (m) from (lat, lon) to the current nominal leg."""
        pending = self.pending_stops()
        if not pending:
            p1 = self.home
            p2 = (self.plan.stops[-1].lat, self.plan.stops[-1].lon) if self.plan.stops else self.home
        else:
            nxt = pending[0]
            if self.state.delivered:
                last_id = self.state.delivered[-1]
                prev = next((s for s in self.plan.stops if s.id == last_id), None)
                p1 = (prev.lat, prev.lon) if prev else self.home
            else:
                p1 = self.home
            p2 = (nxt.lat, nxt.lon)
        from flightstack.obstacle_tracker import geo_to_ned
        a = geo_to_ned(p1[0], p1[1], 0.0, self.home[0], self.home[1])
        b = geo_to_ned(p2[0], p2[1], 0.0, self.home[0], self.home[1])
        p = geo_to_ned(lat, lon, 0.0, self.home[0], self.home[1])
        abx, aby = b[0] - a[0], b[1] - a[1]
        ab_len = math.hypot(abx, aby)
        if ab_len < 1.0:
            return 0.0
        # Cross product magnitude in 2D / segment length
        apx, apy = p[0] - a[0], p[1] - a[1]
        return abs(abx * apy - aby * apx) / ab_len

    def _stream_proximity_obstacle_distance(self, drone_pos, drone_heading_deg: float) -> None:
        """Stream 360-degree MAVLink OBSTACLE_DISTANCE message to ArduPilot AP_Proximity."""
        if not hasattr(self.link.conn, "mav"):
            return
        mav = getattr(self.link.conn, "mav", None)
        if not hasattr(mav, "obstacle_distance_send"):
            return

        now = time.monotonic()
        distances = [65535] * 72  # 72 bins, 5 degrees each, 65535 = no obstacle
        active_obs_id = None
        min_center_dist = None

        if self.obstacle_tracker and drone_pos is not None:
            from flightstack.obstacle_tracker import geo_to_ned
            d_ned = geo_to_ned(drone_pos.lat, drone_pos.lon, drone_pos.alt_rel_m, self.home[0], self.home[1])

            for tr in self.obstacle_tracker.tracks.values():
                if now - tr.last_update_s > self.obstacle_tracker.staleness_limit_s:
                    continue
                dx = tr.position_m.x - d_ned[0]
                dy = tr.position_m.y - d_ned[1]
                dz = tr.position_m.z - d_ned[2]
                horiz_dist = math.hypot(dx, dy)
                total_dist = math.sqrt(dx * dx + dy * dy + dz * dz)

                if 0.2 <= horiz_dist <= 35.0 and abs(dz) <= 25.0:
                    bearing_deg = (math.degrees(math.atan2(dy, dx)) + 360.0) % 360.0
                    rel_bearing_deg = (bearing_deg - (drone_heading_deg or 0.0) + 360.0) % 360.0
                    center_bin = int(round(rel_bearing_deg / 5.0)) % 72
                    center_cm = int(max(20, min(3490, total_dist * 100.0)))
                    distances[center_bin] = min(distances[center_bin], center_cm)

                    if tr.radius_m > 2.5 and horiz_dist > 0.5:
                        half_ang = math.degrees(math.asin(min(0.95, tr.radius_m / horiz_dist)))
                        n_bins = max(1, int(round(half_ang / 5.0)))
                        for offset in range(-n_bins, n_bins + 1):
                            b_idx = (center_bin + offset) % 72
                            distances[b_idx] = min(distances[b_idx], center_cm)

                    active_obs_id = tr.obstacle_id
                    if min_center_dist is None or horiz_dist < min_center_dist:
                        min_center_dist = horiz_dist

        active_bins = [i for i, d in enumerate(distances) if d < 65535]
        if active_bins:
            self.state.proximity_active_frames += 1
            if now - self._last_prox_log >= 1.0:
                self._last_prox_log = now
                self.log.log(
                    "proximity_obstacle_in_range",
                    obstacle_id=active_obs_id,
                    center_distance_m=round(min_center_dist or 0.0, 2),
                    min_bin_cm=min(distances[i] for i in active_bins),
                    active_bin_count=len(active_bins),
                    active_bins=active_bins[:12],
                )

        if hasattr(self.link, "send_obstacle_distance"):
            self.link.send_obstacle_distance(distances)
            return

        time_us = int(time.time() * 1e6)
        try:
            mav.obstacle_distance_send(
                time_us, 0, distances, 5, 20, 3500, 5.0, 0.0, 12
            )
        except Exception:
            pass

    # ------------------------------------------------------------------
    def upload(self, plan: MissionPlan, verify: bool = True) -> None:
        t0 = time.perf_counter()
        self.link.upload_mission(plan.items, verify=verify)
        n = len(plan.items)
        self.plan = plan
        self.state.mission_version += 1
        self.log.log("mission_uploaded", version=self.state.mission_version,
                     n_items=n, upload_ms=round((time.perf_counter() - t0) * 1000, 1),
                     mission=plan_summary(plan))

    def apply_initial_gdacs_impact(self, now: Optional[datetime] = None) -> Optional[dict]:
        """Evaluate and idempotently apply initial GDACS impact to stops and hazard zones before mission upload."""
        if self.gdacs_assessor is None:
            return None
        eval_time = now or datetime.now(timezone.utc)
        gdacs_impact = self.gdacs_assessor.evaluate_mission_impact(
            stops=self.plan.stops,
            leg_paths=self.plan.leg_paths,
            home=self.home,
            now=eval_time,
        )
        if not gdacs_impact.get("available", False):
            self.log.log(
                "gdacs_source_degraded",
                phase="pre_mission",
                status=gdacs_impact.get("status"),
                error=gdacs_impact.get("error"),
                attribution=gdacs_impact.get("attribution"),
            )
            return gdacs_impact

        self.log.log(
            "gdacs_mission_impact_assessed",
            phase="pre_mission",
            status=gdacs_impact.get("status"),
            provenance=gdacs_impact.get("provenance"),
            total_active_events=gdacs_impact.get("total_active_events"),
            advisory_count=len(gdacs_impact.get("advisory_events") or []),
            stop_updates_count=len(gdacs_impact.get("stop_suitability_updates") or []),
            hazard_zones_count=len(gdacs_impact.get("hazard_avoidance_zones") or []),
            attribution=gdacs_impact.get("attribution"),
        )
        from gdacs_provider import GDACSMissionImpactAssessor
        apply_res = GDACSMissionImpactAssessor.apply_impact_to_stops_and_zones(
            gdacs_impact, self.plan.stops, self.zones
        )
        self.zones = _normalize_zones(apply_res["zones"])
        if apply_res["changed"]:
            self.plan = build_delivery_mission(
                self.home,
                self.plan.stops,
                self.zones,
                self.profile,
                home=self.home,
                include_takeoff=True,
                stop_modes=self.stop_modes,
                start_time=eval_time,
            )
            self._sync_payload_modes(self.plan)
            self.log.log(
                "gdacs_pre_mission_applied",
                updated_stop_ids=apply_res["updated_stop_ids"],
                added_zone_ids=apply_res["added_zone_ids"],
                selected_modes=dict(self.plan.selected_modes),
                unreachable_stops=[s.id for s in self.plan.unreachable],
                planned_km=round(self.plan.planned_km, 3),
            )
        return gdacs_impact

    def validate_initial_mission(
        self,
        eval_time: Optional[datetime] = None,
        require_battery_telemetry: bool = True,
    ) -> dict:
        """Validate initial mission feasibility (MTOW, payload, battery energy/reserve, cold-chain time limit,
        delivery deadlines, route/return feasibility, and optional DigitalSky check).

        Raises ``MissionValidationError`` if any constraint is violated.
        """
        pos_obj = None
        if self.link is not None and hasattr(self.link, "telemetry"):
            try:
                pos_obj = self.link.telemetry.fresh_position(self.stale_limit_s)
            except Exception:
                pass
        start_pt = (pos_obj.lat, pos_obj.lon) if pos_obj is not None else self.home
        failure = self._validate_plan_feasibility(
            self.plan,
            start_pt=start_pt,
            eval_time=eval_time,
            is_replan=False,
            require_battery_telemetry=require_battery_telemetry,
        )
        if failure is not None:
            raise MissionValidationError(failure["why"])
        report = {
            "feasible": True,
            "stops": [s.id for s in self.plan.stops],
            "planned_km": round(self.plan.planned_km, 3),
            "selected_modes": dict(getattr(self.plan, "selected_modes", {})),
            "cold_chain_check_type": "time_based_cold_chain_limit_minutes",
        }
        self.log.log("initial_mission_validated", **report)
        return report

    def launch(self) -> None:
        self.apply_initial_gdacs_impact()
        self.link.wait_ready()
        self.log.log("autopilot_ready", telemetry=self.link.telemetry.snapshot())
        self.validate_initial_mission(require_battery_telemetry=True)
        self.upload(self.plan)
        pos = None
        try:
            pos = self.link.telemetry.fresh_position(self.stale_limit_s)
        except Exception:
            pass
        # Pulse proximity sensor before arming if using MAVLink PRX1_TYPE=2 mock
        if not getattr(self.link, "_is_live_socket", False):
            for _ in range(5):
                self._stream_proximity_obstacle_distance(pos, getattr(pos, "heading_deg", 0.0) if pos else 0.0)
                time.sleep(0.05)
        self.link.set_mode("GUIDED")
        self.link.arm()
        self.log.log("armed")
        self.link.set_mode("AUTO")
        self.link.start_mission()
        self.state.took_off = True
        self.log.log("mission_started", version=self.state.mission_version)

    # ------------------------------------------------------------------
    def pending_stops(self) -> List[Location]:
        return [
            s for s in self.plan.stops
            if s.id not in self.state.delivered and s.id not in self.state.failed_deliveries
        ]

    def step(self) -> None:
        """Poll telemetry once; update planner state; log at 1 Hz."""
        tel = self.link.telemetry
        now = time.monotonic()
        try:
            pos = tel.fresh_position(self.stale_limit_s)
            if self._stale:
                self._stale = False
                self.log.log("telemetry_recovered")
        except StaleTelemetryError as e:
            pos = None
            if not self._stale:
                self._stale = True
                self.state.stale_events += 1
                self.log.log("telemetry_stale", reason=str(e),
                             note="planner decisions suspended; autopilot failsafes remain active")

        cur = tel.mission_current.value

        # 0. Process autopilot STATUSTEXT messages first so simulated gripper feedback
        #    ("Mission: <seq> Gripper", "Gripper Released", "Gripper load released", or fault)
        #    is causally recorded before evaluating stop completion.
        texts = tel.status_texts
        new_texts = texts[self._n_texts:] if len(texts) >= self._n_texts else texts
        self._n_texts = len(texts)
        for _, sev, text in new_texts:
            self.log.log("autopilot_text", severity=sev, text=text)
            for stop_obj in self.plan.stops:
                sid = stop_obj.id
                if sid in self.state.delivered or sid in self.state.failed_deliveries:
                    continue
                pkg = self.payload.get_package(sid)
                if pkg is None:
                    continue
                rel_seq = getattr(self.plan, "release_seq", {}).get(sid, self.plan.delivery_seq.get(sid))
                if rel_seq is None:
                    continue
                is_mission_gripper_msg = f"Mission: {rel_seq} Gripper" in text
                is_gripper_feedback_msg = (
                    any(tok in text for tok in (
                        "Gripper Released", "Gripper load released", "Gripper load releasing",
                        "Gripper failed", "Gripper fault", "Gripper jam"
                    ))
                    and cur is not None and cur >= rel_seq - 1
                )
                if (is_mission_gripper_msg or is_gripper_feedback_msg) and pkg.delivery_state in (
                    DeliveryState.LOADED, DeliveryState.ARRIVED,
                    DeliveryState.SITE_ASSESSED, DeliveryState.METHOD_SELECTED,
                ):
                    if pkg.delivery_state == DeliveryState.LOADED:
                        self.payload.record_arrival(sid, self.log.elapsed())
                        site = SiteSuitability.from_any(getattr(stop_obj, "site_suitability", None))
                        eff_req = self.stop_modes.get(sid, self.profile.delivery_mode)
                        planned_mode = getattr(self.plan, "selected_modes", {}).get(sid, pkg.planned_mode)
                        ok, selected_mode, reason = self.payload.assess_site(
                            sid, site, eff_req, self.log.elapsed()
                        )
                        self.log.log(
                            "site_assessed",
                            stop_id=sid,
                            requested_mode=eff_req,
                            planned_mode=planned_mode,
                            selected_mode=selected_mode,
                            site_ok=ok,
                            reason=reason,
                            site_provenance=site.provenance,
                            site_uncertainty=site.uncertainty,
                        )
                    cmd_ok, _, cmd_err = self.payload.command_release(sid, pkg.release_mode, self.log.elapsed())
                    if cmd_ok:
                        self.log.log("release_commanded", stop_id=sid, mode=pkg.release_mode, release_seq=rel_seq)
                    else:
                        if sid not in self.state.failed_deliveries:
                            self.state.failed_deliveries.append(sid)
                        self.log.log("delivery_failed", stop_id=sid, reason=cmd_err or pkg.error_reason,
                                     payload=pkg.to_dict())
                    break

            act_res = self.payload.record_actuator_text(text, self.log.elapsed())
            if act_res is not None:
                cid, cpkg = act_res
                if cpkg.delivery_state == DeliveryState.RELEASE_ACKNOWLEDGED:
                    self.log.log(
                        "payload_actuator_confirmed",
                        stop_id=cid,
                        release_provenance=cpkg.release_provenance,
                        simulated_release_acknowledged=cpkg.simulated_release_acknowledged,
                        recipient_handover_verified=cpkg.recipient_handover_verified,
                        package=cpkg.to_dict(),
                    )
                elif cpkg.delivery_state == DeliveryState.FAILED:
                    if cid not in self.state.failed_deliveries:
                        self.state.failed_deliveries.append(cid)
                    self.log.log(
                        "payload_release_failed",
                        stop_id=cid,
                        reason=cpkg.error_reason,
                        package=cpkg.to_dict(),
                    )
                    self.log.log(
                        "delivery_failed",
                        stop_id=cid,
                        reason=cpkg.error_reason,
                        payload=cpkg.to_dict(),
                    )

        if cur is not None:
            # 1. State machine per stop: separate arrival, site assessment, simulated release command,
            #    causal simulated-gripper acknowledgement, ground dwell completion, and stop completion.
            for it in self.plan.items:
                if (
                    it.stop_id is not None
                    and it.stop_id not in self.state.delivered
                    and it.stop_id not in self.state.failed_deliveries
                ):
                    sid = it.stop_id
                    pkg = self.payload.get_package(sid)
                    if pkg is None:
                        continue

                    # Step A: Arrival overhead at stop (once MISSION_CURRENT advances past the 'arrive' waypoint)
                    if it.role == "arrive" and cur > it.seq and pkg.delivery_state == DeliveryState.LOADED:
                        self.payload.record_arrival(sid, self.log.elapsed())
                        stop_obj = next((s for s in self.plan.stops if s.id == sid), None)
                        site = SiteSuitability.from_any(getattr(stop_obj, "site_suitability", None))
                        eff_req = self.stop_modes.get(sid, self.profile.delivery_mode)
                        planned_mode = getattr(self.plan, "selected_modes", {}).get(sid, pkg.planned_mode)
                        ok, selected_mode, reason = self.payload.assess_site(
                            sid, site, eff_req, self.log.elapsed()
                        )
                        self.log.log(
                            "site_assessed",
                            stop_id=sid,
                            requested_mode=eff_req,
                            planned_mode=planned_mode,
                            selected_mode=selected_mode,
                            site_ok=ok,
                            reason=reason,
                            site_provenance=site.provenance,
                            site_uncertainty=site.uncertainty,
                        )
                        # If site suitability mutated before arrival so that the uploaded MAVLink
                        # sequence disagrees with selected_mode (or site is now abort), immediately
                        # hold and replan rather than blindly executing the outdated sequence.
                        if not ok or selected_mode == "abort" or (
                            planned_mode in ("hover", "land") and selected_mode != planned_mode
                        ):
                            self.apply_site_suitability_update(
                                sid,
                                site,
                                reason="runtime_arrival_revalidation",
                                source={"trigger": "arrival_overhead_mismatch"},
                                force_replan=True,
                            )
                            break

                    # Log ground touchdown when at a landing delivery stop
                    if (it.role == "land" and cur >= it.seq and sid not in self._landed_stops_logged
                            and pos is not None and pos.alt_rel_m <= 2.0):
                        self._landed_stops_logged.add(sid)
                        self.log.log("ground_landing_confirmed", stop_id=sid,
                                     alt_rel_m=round(pos.alt_rel_m, 2),
                                     lat=pos.lat, lon=pos.lon)

                    # Step B: Release commanded at release_seq (DO_GRIPPER).
                    # NEVER auto-acknowledge the gripper here; causal acknowledgement must come
                    # from simulated gripper feedback ("Gripper Released" / "Gripper load released").
                    rel_seq = getattr(self.plan, "release_seq", {}).get(sid, self.plan.delivery_seq.get(sid))
                    if rel_seq is not None and cur >= rel_seq:
                        if pkg.delivery_state in (DeliveryState.LOADED, DeliveryState.ARRIVED,
                                                  DeliveryState.SITE_ASSESSED, DeliveryState.METHOD_SELECTED):
                            cmd_ok, _, cmd_err = self.payload.command_release(
                                sid, pkg.release_mode, self.log.elapsed()
                            )
                            if cmd_ok:
                                self.log.log("release_commanded", stop_id=sid, mode=pkg.release_mode,
                                             release_seq=rel_seq)
                            else:
                                if sid not in self.state.failed_deliveries:
                                    self.state.failed_deliveries.append(sid)
                                self.log.log("delivery_failed", stop_id=sid,
                                             reason=cmd_err or pkg.error_reason,
                                             payload=pkg.to_dict())
                                continue

                    # Step B2: Ground dwell tracking for landed deliveries (MAV_CMD_NAV_DELAY = 93).
                    # Ground delivery must NOT be marked complete while cur == dwell_seq.
                    dwell_seq = getattr(self.plan, "dwell_seq", {}).get(sid)
                    comp_seq = self.plan.delivery_seq.get(sid)
                    if dwell_seq is not None and cur >= dwell_seq and sid not in self._dwell_started_stops:
                        self._dwell_started_stops.add(sid)
                        self.log.log("ground_dwell_started", stop_id=sid, dwell_seq=dwell_seq,
                                     hold_s=max(float(self.profile.hover_hold_s), 5.0))
                    if (
                        dwell_seq is not None
                        and comp_seq is not None
                        and cur >= comp_seq
                        and sid not in self._dwell_completed_stops
                    ):
                        self._dwell_completed_stops.add(sid)
                        self.payload.record_dwell_completed(sid, self.log.elapsed())
                        self.log.log("ground_dwell_completed", stop_id=sid,
                                     dwell_seq=dwell_seq, completion_seq=comp_seq)

                    # Step C: Stop completion once past completion sequence (post-release climb for hover;
                    # retakeoff AFTER ground_dwell for land). Never call confirm_custody on simulated release.
                    if comp_seq is not None and cur >= comp_seq:
                        if pkg.delivery_state == DeliveryState.RELEASE_ACKNOWLEDGED:
                            done_ok, _, _ = self.payload.complete_delivery(sid, self.log.elapsed())
                            if done_ok:
                                self.state.delivered.append(sid)
                                self.log.log(
                                    "delivery_complete",
                                    stop_id=sid,
                                    mission_seq=comp_seq,
                                    release_seq=rel_seq,
                                    release_provenance=pkg.release_provenance,
                                    simulated_release_acknowledged=pkg.simulated_release_acknowledged,
                                    stop_completion_recorded=pkg.stop_completion_recorded,
                                    recipient_handover_verified=pkg.recipient_handover_verified,
                                    payload=pkg.to_dict(),
                                    remaining_cargo_kg=self.payload.remaining_cargo_kg(),
                                    position=None if pos is None else (pos.lat, pos.lon, pos.alt_rel_m),
                                    battery=tel.battery.value.__dict__ if tel.battery.value else None,
                                )
                        elif pkg.delivery_state == DeliveryState.FAILED:
                            if sid not in self.state.failed_deliveries:
                                self.state.failed_deliveries.append(sid)
                            self.log.log("delivery_failed", stop_id=sid, mission_seq=comp_seq,
                                         reason=pkg.error_reason, payload=pkg.to_dict())
                        elif pkg.delivery_state == DeliveryState.RELEASE_COMMANDED:
                            elapsed_since_cmd = self.log.elapsed() - (pkg.commanded_time_s or 0.0)
                            if cur > comp_seq or elapsed_since_cmd >= self.actuator_ack_timeout_s:
                                self.payload.fail_release(
                                    sid, self.log.elapsed(), reason="MISSING_ACTUATOR_FEEDBACK"
                                )
                                if sid not in self.state.failed_deliveries:
                                    self.state.failed_deliveries.append(sid)
                                self.log.log(
                                    "delivery_failed",
                                    stop_id=sid,
                                    mission_seq=comp_seq,
                                    reason="MISSING_ACTUATOR_FEEDBACK",
                                    payload=pkg.to_dict(),
                                )

        # Observe native ArduPilot SITL C++ proximity raycaster (PRX1_TYPE=10) via DISTANCE_SENSOR
        prox_sectors = getattr(tel, "proximity_sectors_cm", {})
        prox_rx = getattr(tel, "proximity_sector_rx", {})
        for k in list(prox_sectors.keys()):
            if k in prox_rx and (now - prox_rx[k] > 1.5):
                del prox_sectors[k]
                prox_rx.pop(k, None)
        if prox_sectors:
            self.state.proximity_active_frames += 1
            min_cm = min(prox_sectors.values())
            min_ray_m = round(min_cm / 100.0, 2)
            if (
                self.state.min_proximity_ray_distance_m is None
                or min_ray_m < self.state.min_proximity_ray_distance_m
            ):
                self.state.min_proximity_ray_distance_m = min_ray_m
            if now - self._last_prox_log >= 1.0:
                self._last_prox_log = now
                self.log.log(
                    "sim_sensor_proximity_observation",
                    source_classification="SIMULATED_SENSOR",
                    sensor_backend="AP_Proximity_SITL_PRX1_TYPE_10",
                    sectors_cm=dict(prox_sectors),
                    min_distance_m=min_ray_m,
                )

        # Track actual horizontal surface clearance to any registered world obstacles
        if pos is not None and self.state.took_off and not self.state.finished and self._world_obstacles:
            from geo_utils import haversine_km
            for obs_lat, obs_lon, obs_r_m, _obs_name in self._world_obstacles:
                center_d_m = haversine_km((pos.lat, pos.lon), (obs_lat, obs_lon)) * 1000.0
                surf_clear_m = round(center_d_m - obs_r_m, 2)
                if (
                    self.state.min_obstacle_surface_clearance_m is None
                    or surf_clear_m < self.state.min_obstacle_surface_clearance_m
                ):
                    self.state.min_obstacle_surface_clearance_m = surf_clear_m

        # Proximity streaming for unit-test mocks (PRX1_TYPE=2); disabled on live SITL socket (PRX1_TYPE=10)
        if not getattr(self.link, "_is_live_socket", False):
            if now - self._last_proximity_stream >= 0.20 and pos is not None:
                self._last_proximity_stream = now
                self._stream_proximity_obstacle_distance(pos, getattr(pos, "heading_deg", 0.0))

        # Check ArduPilot AP_OADatabase telemetry feedback (ADSB_VEHICLE with callsign OA_DB)
        oa_items = getattr(tel, "oa_db_items", [])
        if len(oa_items) > self._n_oa_db:
            new_items = oa_items[self._n_oa_db:]
            self._n_oa_db = len(oa_items)
            self.state.oa_db_confirmations += len(new_items)
            latest = new_items[-1]
            self.log.log("ardupilot_oa_db_confirmed",
                         count=self.state.oa_db_confirmations,
                         latest_item=latest,
                         proximity_sectors_cm=dict(getattr(tel, "proximity_sectors_cm", {})))

        # Measure lateral trajectory deviation when native OA proximity sensing has seen an obstacle
        if (pos is not None and self.state.took_off and not self.state.finished
                and self.state.proximity_active_frames > 0 and len(self.state.replans) == 0):
            xtrack_m = self._compute_cross_track_m(pos.lat, pos.lon)
            if xtrack_m > self.state.max_oa_deviation_m:
                self.state.max_oa_deviation_m = round(xtrack_m, 2)
            if xtrack_m >= 1.5 and now - self._last_dev_log >= 1.0:
                self._last_dev_log = now
                self.log.log("oa_trajectory_deviation",
                             cross_track_m=round(xtrack_m, 2),
                             max_oa_deviation_m=self.state.max_oa_deviation_m,
                             min_obstacle_surface_clearance_m=self.state.min_obstacle_surface_clearance_m,
                             lat=pos.lat, lon=pos.lon, alt_rel_m=pos.alt_rel_m,
                             heading_deg=pos.heading_deg,
                             mode=tel.mode)

        # Optional computer-vision perception pipeline -> 3D spatial observation -> ObstacleTracker
        if (self.perception_pipeline is not None and pos is not None
                and self.state.took_off and not self.state.finished):
            if now - self._last_perception_poll >= 0.5:
                self._last_perception_poll = now
                from flightstack.obstacle_tracker import Vector3D, geo_to_ned
                d_ned = geo_to_ned(pos.lat, pos.lon, pos.alt_rel_m, self.home[0], self.home[1])
                drone_vec = Vector3D(d_ned[0], d_ned[1], d_ned[2])
                res_perc = self.perception_pipeline.process_next_frame(
                    drone_pos_ned=drone_vec,
                    drone_attitude_deg=(0.0, 0.0, pos.heading_deg or 0.0),
                )
                if res_perc.get("status") == "OK":
                    n_det = res_perc.get("detections_count", 0)
                    spatial_list = res_perc.get("spatial_observations") or []
                    accepted_3d = 0
                    if self.obstacle_tracker is not None:
                        for s_obs in spatial_list:
                            tr = self.obstacle_tracker.update_from_spatial_observation(s_obs)
                            if tr is not None:
                                accepted_3d += 1
                    if n_det > 0:
                        self.log.log(
                            "perception_observation",
                            provenance=res_perc.get("provenance"),
                            detections_2d=n_det,
                            accepted_3d_metric_tracks=accepted_3d,
                            inference_ms=res_perc.get("inference_ms"),
                        )

        # Periodic external data polling (DynamicAirspaceMonitor & GDACSMissionImpactAssessor)
        if self.state.took_off and not self.state.finished and (self.airspace_monitor or self.gdacs_assessor):
            poll_interval = getattr(self.airspace_monitor, "poll_interval_s", 15.0) if self.airspace_monitor else 15.0
            if now - self._last_external_poll >= poll_interval:
                self._last_external_poll = now
                self.poll_external_updates()

        # Dynamic Obstacle Perception & Conflict Response
        if (self.obstacle_tracker and pos is not None
                and self.state.took_off and not self.state.finished):
            if now - self._last_obstacle_eval >= 0.5:
                self._last_obstacle_eval = now
                from flightstack.obstacle_tracker import geo_to_ned, ned_to_geo, obstacle_to_exclusion_polygon
                self.obstacle_tracker.purge_stale_tracks(now)
                drone_ned = geo_to_ned(pos.lat, pos.lon, pos.alt_rel_m, self.home[0], self.home[1])
                drone_vel = (pos.vn_ms, pos.ve_ms, pos.vd_ms)
                alerts = self.obstacle_tracker.evaluate_conflict(drone_ned, drone_vel, now=now)
                conflicts = [a for a in alerts if a.has_conflict]
                if conflicts:
                    top = conflicts[0]
                    self.log.log("obstacle_conflict", obstacle_id=top.obstacle_id,
                                 action=top.action_required, d_cpa=top.d_cpa_m, t_cpa=top.t_cpa_s,
                                 current_d=top.current_distance_m,
                                 native_oa_enabled=self.native_oa_enabled)
                    if self.native_oa_enabled:
                        # Native ArduPilot BendyRuler (OA_TYPE=1, PRX1_TYPE=2) handles 3.0m..35.0m in AUTO mode;
                        # Python supervisor only intervenes with emergency LOITER if separation breaches 2.5m.
                        if top.current_distance_m < 2.5:
                            if tel.mode != "LOITER":
                                self.link.set_mode("LOITER")
                                self.log.log("tactical_hold_commanded", obstacle_id=top.obstacle_id,
                                             detail=f"Emergency stop bubble breached ({top.current_distance_m:.1f}m < 2.5m)")
                    elif top.action_required == "HOLD":
                        cur_mode = tel.mode
                        if cur_mode != "LOITER":
                            self.link.set_mode("LOITER")
                            self.log.log("tactical_hold_commanded", obstacle_id=top.obstacle_id,
                                         detail=top.detail)

                    elif top.action_required == "REROUTE" and top.obstacle_id not in self._avoiding_obstacles:
                        self._avoiding_obstacles.add(top.obstacle_id)
                        track = self.obstacle_tracker.tracks.get(top.obstacle_id)
                        if track:
                            obs_lat, obs_lon, _ = ned_to_geo(
                                track.position_m.x, track.position_m.y, track.position_m.z,
                                self.home[0], self.home[1]
                            )
                            exclusion_poly = obstacle_to_exclusion_polygon(
                                obs_lat, obs_lon, radius_m=max(track.radius_m + 35.0, 50.0)
                            )
                            self.apply_airspace_update(
                                self.zones + [exclusion_poly],
                                reason=f"obstacle_avoidance_{top.obstacle_id}",
                                source={
                                    "provider": "obstacle_tracker",
                                    "obstacle_id": top.obstacle_id,
                                    "d_cpa": top.d_cpa_m,
                                    "t_cpa": top.t_cpa_s,
                                    "polygon": exclusion_poly,
                                }
                            )

        b = tel.battery.value
        if (self.state.took_off and not self._reserve_warned and b is not None
                and b.remaining_pct is not None and b.remaining_pct < self.battery_reserve_pct):
            self._reserve_warned = True
            self.log.log("battery_reserve_breached", remaining_pct=b.remaining_pct,
                         reserve_pct=self.battery_reserve_pct, consumed_mah=b.consumed_mah,
                         pending_stops=[s.id for s in self.pending_stops()],
                         note="autopilot battery-monitor estimate (SITL model, uncalibrated)")
            if self.pending_stops():
                self.link.set_mode("RTL")
                self.log.log("battery_failsafe_triggered", action="RTL",
                             remaining_pct=b.remaining_pct,
                             reason="Battery level breached reserve floor with deliveries pending")

        if now - self._last_tlog >= 1.0:
            self._last_tlog = now
            self.log.log("telemetry", telemetry=tel.snapshot())

        if self.state.took_off and not tel.armed and tel.link_alive():
            from geo_utils import haversine_km
            cur_seq = tel.mission_current.value or 0
            at_home = (pos is not None and haversine_km(self.home, (pos.lat, pos.lon)) <= 0.03)
            # If the vehicle has returned home and disarmed while any stop is still in RELEASE_COMMANDED
            # without actuator feedback, fail that release explicitly rather than leaving it hanging.
            if cur_seq >= len(self.plan.items) - 1 or at_home:
                for stop_obj in list(self.pending_stops()):
                    pkg = self.payload.get_package(stop_obj.id)
                    if pkg is not None and pkg.delivery_state == DeliveryState.RELEASE_COMMANDED:
                        self.payload.fail_release(
                            stop_obj.id, self.log.elapsed(), reason="MISSING_ACTUATOR_FEEDBACK"
                        )
                        if stop_obj.id not in self.state.failed_deliveries:
                            self.state.failed_deliveries.append(stop_obj.id)
                        self.log.log(
                            "delivery_failed",
                            stop_id=stop_obj.id,
                            mission_seq=cur_seq,
                            reason="MISSING_ACTUATOR_FEEDBACK",
                            payload=pkg.to_dict(),
                        )
            is_final = (not self.pending_stops()) and (
                cur_seq >= len(self.plan.items) - 1 or at_home
            )
            if is_final:
                if not self.state.finished:
                    self.state.finished = True
                    self.log.log("final_landing_confirmed",
                                 position=None if pos is None else (pos.lat, pos.lon, pos.alt_rel_m))
                    self.log.log("mission_finished", delivered=self.state.delivered,
                                 failed_deliveries=self.state.failed_deliveries,
                                 battery=tel.battery.value.__dict__ if tel.battery.value else None)
            elif now - self._last_rearm_attempt >= 5.0:
                self._last_rearm_attempt = now
                # Vehicle disarmed on ground at an intermediate delivery stop:
                # Log ground touchdown if within 25 m, but NEVER fabricate actuator acknowledgement
                # or recipient custody on disarm.
                if pos is not None:
                    for stop_obj in self.pending_stops():
                        if haversine_km((pos.lat, pos.lon), (stop_obj.lat, stop_obj.lon)) <= 0.025:
                            sid = stop_obj.id
                            if sid not in self._landed_stops_logged:
                                self._landed_stops_logged.add(sid)
                                self.log.log("ground_landing_confirmed", stop_id=sid,
                                             alt_rel_m=round(pos.alt_rel_m, 2),
                                             lat=pos.lat, lon=pos.lon)
                # Never call arm() in AUTO mode ("Arm: AUTO mode not armable").
                # Switch to GUIDED -> arm -> set_current_item(retakeoff_seq) -> AUTO -> start_mission.
                retakeoff_seq = next(
                    (it.seq for it in self.plan.items if it.seq >= cur_seq and it.role == "retakeoff"),
                    cur_seq,
                )
                try:
                    self.link.set_mode("GUIDED")
                    self.link.arm(timeout=15.0)
                    self.link.set_current_item(retakeoff_seq)
                    self.link.set_mode("AUTO")
                    self.link.start_mission()
                    self.log.log("intermediate_retakeoff_resumed", retakeoff_seq=retakeoff_seq)
                except Exception as exc:
                    self.log.log("intermediate_rearm_retry", error=str(exc), retakeoff_seq=retakeoff_seq)

    def run_until(self, done: Callable[["MissionRunner"], bool], timeout_s: float,
                  hook: Optional[Callable[["MissionRunner"], None]] = None) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.step()
            if hook:
                hook(self)
            if done(self):
                return True
            time.sleep(0.1)
        self.log.log("timeout", after_s=timeout_s)
        return False

    # ------------------------------------------------------------------
    def remaining_polyline(self, pos) -> List[tuple]:
        """Current position followed by the nav points not yet reached."""
        cur = self.link.telemetry.mission_current.value or 0
        pts = [(pos.lat, pos.lon)]
        for it in self.plan.items:
            if it.seq >= cur and it.seq > 0 and it.is_nav and it.lat != 0.0 and it.role != "rtl":
                pts.append((it.lat, it.lon))
        pts.append(self.home)       # RTL leg
        return pts

    def _validate_plan_feasibility(
        self,
        plan: MissionPlan,
        start_pt: Sequence[float],
        eval_time: Optional[datetime] = None,
        is_replan: bool = True,
        require_battery_telemetry: bool = True,
    ) -> Optional[dict]:
        """Shared mission feasibility validator for both initial mission launch and in-flight replanning.

        Enforces:
          0. Route & return feasibility (reachable stops, full-route airspace clearance).
          1. Payload capacity (<= max_payload_kg) and vehicle MTOW (<= mtow_kg).
          2. Battery telemetry presence/freshness and usable battery energy + reserve margin.
          3. Time-based cold-chain limits (``cold_chain_limit_minutes``) and delivery deadlines (``window_minutes``).
          4. Full-route DigitalSky corridor validation when a checker is attached (never treating checked=False as clearance).
        """
        prefix = "REPLAN_INFEASIBLE" if is_replan else "INITIAL_MISSION_INFEASIBLE"

        def _fail(why_msg: str, breach_type: str, **extra) -> dict:
            if is_replan:
                self.link.set_mode("LOITER")
                return self.log.log("operator_required", why=why_msg, breach_type=breach_type, **extra)
            return self.log.log("initial_mission_rejected", why=why_msg, breach_type=breach_type, **extra)

        # 0. Route and return feasibility
        if not is_replan:
            if not plan.stops:
                return _fail(
                    f"{prefix}: No reachable or suitable delivery stops in initial mission",
                    "route_infeasible",
                )
            for u_stop in getattr(plan, "unreachable", ()):
                assess = getattr(plan, "site_assessments", {}).get(u_stop.id, {})
                if assess.get("ok", True) and assess.get("selected_mode") != "abort":
                    return _fail(
                        f"{prefix}: Stop {u_stop.id} ({u_stop.name}) has no legal route around restricted airspace",
                        "route_infeasible",
                        stop_id=u_stop.id,
                    )

        full_pts = plan.full_route_polyline(start_pt)
        if len(full_pts) < 2:
            return _fail(f"{prefix}: Route polyline has fewer than 2 points", "route_infeasible")
        try:
            validate_polyline(
                full_pts,
                self.zones,
                self.profile.safety_margin_m,
                alt_m_agl=self.profile.cruise_alt_m,
                start_time=eval_time,
                cruise_speed_ms=self.profile.cruise_speed_ms,
            )
        except MissionValidationError as e:
            return _fail(f"{prefix}: Route violates airspace restrictions ({e})", "route_infeasible")

        # 1. Payload capacity, Vehicle MTOW, and Battery Energy Reserve check
        plan_cargo_kg = sum(float(getattr(s, "demand_kg", 0.0) or 0.0) for s in plan.stops)
        rem_cargo = (
            self.payload.remaining_cargo_kg()
            if is_replan
            else max(self.payload.remaining_cargo_kg(), plan_cargo_kg)
        )
        vm = getattr(self.profile, "vehicle_model", None)
        if vm is not None:
            max_payload = getattr(vm, "max_payload_kg", None)
            if max_payload is not None and rem_cargo > max_payload + 1e-6:
                return _fail(
                    f"{prefix}: Payload mass {rem_cargo:.2f}kg exceeds max_payload_kg {max_payload:.2f}kg",
                    "payload_capacity",
                )
            try:
                tot_mass = vm.total_mass_kg(rem_cargo)
                if tot_mass > vm.mtow_kg + 1e-6:
                    return _fail(
                        f"{prefix}: Gross mass {tot_mass:.1f}kg exceeds MTOW {vm.mtow_kg:.1f}kg",
                        "mtow",
                    )
            except ValueError as e:
                b_type = (
                    "payload_capacity"
                    if ("capacity" in str(e).lower() or "payload" in str(e).lower())
                    else "mtow"
                )
                return _fail(f"{prefix}: {e}", b_type)

            bat_stamped = getattr(getattr(self.link, "telemetry", None), "battery", None)
            tel_bat = bat_stamped.value if bat_stamped is not None else None
            if require_battery_telemetry:
                if tel_bat is None or getattr(tel_bat, "remaining_pct", None) is None:
                    return _fail(
                        f"{prefix}: Battery telemetry missing or unknown (cannot assume 100% SoC)",
                        "battery_telemetry_missing",
                    )
                if bat_stamped is not None and bat_stamped.rx_time is not None:
                    bat_age = bat_stamped.age(self.link.telemetry.clock())
                    if bat_age > max(self.stale_limit_s * 5.0, 10.0):
                        return _fail(
                            f"{prefix}: Battery telemetry is stale (age {bat_age:.1f}s)",
                            "battery_telemetry_stale",
                        )
                current_pct = float(tel_bat.remaining_pct)
            else:
                current_pct = (
                    float(tel_bat.remaining_pct)
                    if (tel_bat is not None and getattr(tel_bat, "remaining_pct", None) is not None)
                    else 100.0
                )

            from flightstack.vehicle_model import EnergyEstimator
            ee = EnergyEstimator(vm)
            running_cargo = rem_cargo
            stops_payload = []
            for s in plan.stops:
                stops_payload.append((s, running_cargo))
                running_cargo = max(0.0, running_cargo - float(getattr(s, "demand_kg", 0.0) or 0.0))
            if not stops_payload and rem_cargo > 0.0:
                stops_payload = [(None, rem_cargo)]
            energy_eval = ee.evaluate_route(
                plan.planned_km,
                stops_payload,
                cruise_speed_ms=self.profile.cruise_speed_ms or 15.0,
                hover_time_s=self.profile.hover_hold_s if plan.stops else 0.0,
            )
            usable_wh_remaining = max(
                0.0, ((current_pct - self.battery_reserve_pct) / 100.0) * vm.usable_battery_wh
            )
            if energy_eval.total_energy_wh > usable_wh_remaining:
                return _fail(
                    (
                        f"{prefix}: Required energy {energy_eval.total_energy_wh:.1f}Wh "
                        f"exceeds usable remaining {usable_wh_remaining:.1f}Wh "
                        f"(current SoC: {current_pct}%, reserve floor: {self.battery_reserve_pct}%)"
                    ),
                    "energy_reserve",
                )

        # 2. Time-based cold-chain limit (cold_chain_limit_minutes) & delivery deadline (window_minutes)
        #    Note: This is an elapsed-time limit check against stop ETA, not a thermodynamic heat-transfer model.
        from flightstack.mission import _polyline_km
        from geo_utils import haversine_km

        mission_elapsed_min = (self.log.elapsed() / 60.0) if is_replan else 0.0
        cur_lat, cur_lon = float(start_pt[0]), float(start_pt[1])
        cum_flight_min = 0.0
        spd_ms = self.profile.cruise_speed_ms or 15.0
        for idx, s in enumerate(plan.stops):
            if idx < len(plan.leg_paths):
                leg_dist_km = _polyline_km(plan.leg_paths[idx])
            else:
                leg_dist_km = haversine_km((cur_lat, cur_lon), (s.lat, s.lon))
            cum_flight_min += (leg_dist_km * 1000.0 / spd_ms) / 60.0
            eta_min = mission_elapsed_min + cum_flight_min
            cold_limit = getattr(s, "cold_chain_limit_minutes", None)
            if cold_limit and eta_min > cold_limit:
                warn_evt = "replan_cold_chain_warning" if is_replan else "initial_cold_chain_warning"
                self.log.log(
                    warn_evt,
                    stop_id=s.id,
                    eta_min=round(eta_min, 1),
                    limit_min=cold_limit,
                    check_type="time_based_cold_chain_limit_minutes",
                    detail=f"Stop {s.id} ETA {eta_min:.1f}m breaches time-based cold-chain limit {cold_limit}m",
                )
                if getattr(self.profile, "enforce_cold_chain", True):
                    return _fail(
                        f"{prefix}: Stop {s.id} ETA {eta_min:.1f}m breaches time-based cold-chain limit {cold_limit}m",
                        "cold_chain",
                        stop_id=s.id,
                    )
            win_limit = getattr(s, "window_minutes", None)
            if win_limit and eta_min > win_limit:
                warn_evt = "replan_time_window_warning" if is_replan else "initial_time_window_warning"
                self.log.log(
                    warn_evt,
                    stop_id=s.id,
                    eta_min=round(eta_min, 1),
                    window_min=win_limit,
                    detail=f"Stop {s.id} ETA {eta_min:.1f}m breaches delivery window {win_limit}m",
                )
                if getattr(self.profile, "enforce_hard_deadlines", True):
                    return _fail(
                        f"{prefix}: Stop {s.id} ETA {eta_min:.1f}m breaches delivery window {win_limit}m",
                        "time_window",
                        stop_id=s.id,
                    )
            cum_flight_min += (self.profile.hover_hold_s or 0.0) / 60.0
            cur_lat, cur_lon = s.lat, s.lon

        # 3. Optional official DigitalSky OpenAPI 3.0 corridor check across full route polyline
        if self.digitalsky_checker is not None:
            ds_res = self.digitalsky_checker.validate_route_corridor(
                full_pts, alt_m_agl=self.profile.cruise_alt_m, start=eval_time
            )
            ds_evt = "replan_digitalsky_checked" if is_replan else "initial_digitalsky_checked"
            self.log.log(
                ds_evt,
                checked=ds_res.checked,
                intersects=ds_res.intersects,
                official_clearance=ds_res.official_clearance,
                waypoints_checked=len(full_pts),
                detail=ds_res.detail,
            )
            if ds_res.checked and ds_res.intersects is True:
                return _fail(
                    f"{prefix}: Official DigitalSky check reports restricted airspace intersection ({ds_res.detail})",
                    "digitalsky_airspace_intersection",
                )
            if getattr(self.profile, "require_official_digitalsky_clearance", False) and not ds_res.official_clearance:
                return _fail(
                    f"{prefix}: Official DigitalSky clearance required but unavailable (checked={ds_res.checked}, detail={ds_res.detail})",
                    "digitalsky_clearance_missing",
                )

        return None

    def _validate_candidate_replan(
        self,
        new_plan: MissionPlan,
        pos,
        eval_time: Optional[datetime] = None,
    ) -> Optional[dict]:
        """Run all physical, battery, cold-chain, deadline, and DigitalSky gates on a candidate replan.

        Returns None if feasible, or logs and returns an ``operator_required`` event dict
        (while holding in LOITER) if any constraint is breached.
        """
        return self._validate_plan_feasibility(
            new_plan,
            start_pt=(pos.lat, pos.lon),
            eval_time=eval_time,
            is_replan=True,
            require_battery_telemetry=True,
        )

    def _sync_payload_modes(self, plan: MissionPlan) -> None:
        for sid, sel_mode in getattr(plan, "selected_modes", {}).items():
            pkg = self.payload.get_package(sid)
            if pkg is not None and pkg.status != PackageStatus.DELIVERED:
                pkg.release_mode = sel_mode
                pkg.planned_mode = sel_mode
        for unreach_stop in getattr(plan, "unreachable", ()):
            pkg = self.payload.get_package(unreach_stop.id)
            if pkg is not None and pkg.status != PackageStatus.DELIVERED:
                assess = getattr(plan, "site_assessments", {}).get(unreach_stop.id, {})
                if not assess.get("ok", True) or assess.get("selected_mode") == "abort":
                    pkg.status = PackageStatus.FAILED
                    pkg.delivery_state = DeliveryState.FAILED
                    pkg.release_mode = "abort"
                    pkg.error_reason = assess.get("reason", "Site unsuitable for delivery")

    def apply_airspace_update(self, new_zones, reason: str, source: dict,
                              now: Optional[datetime] = None) -> dict:
        """Validate the active mission against updated airspace; if the
        remaining route is now illegal, hold, re-plan from the autopilot's
        current position estimate, upload and resume."""
        t0 = time.perf_counter()
        eval_time = now or datetime.now(timezone.utc)
        self.zones = _normalize_zones(new_zones)
        margin = self.profile.safety_margin_m
        try:
            pos = self.link.telemetry.fresh_position(self.stale_limit_s)
        except StaleTelemetryError as e:
            rec = self.log.log("airspace_update_deferred", reason=reason, why=str(e))
            return rec
        remaining = self.remaining_polyline(pos)
        try:
            validate_polyline(
                remaining, self.zones, margin,
                alt_m_agl=pos.alt_rel_m,
                start_time=eval_time,
                cruise_speed_ms=self.profile.cruise_speed_ms
            )
            return self.log.log("airspace_update_no_conflict", reason=reason, source=source,
                                check_ms=round((time.perf_counter() - t0) * 1000, 1))
        except MissionValidationError as e:
            conflict = str(e)
        self.log.log("airspace_conflict", reason=reason, source=source, detail=conflict,
                     position=(pos.lat, pos.lon, pos.alt_rel_m))

        if point_in_restricted((pos.lat, pos.lon), self.zones, 0.0,
                               alt_m_agl=pos.alt_rel_m, when=eval_time):
            self.link.set_mode("LOITER")
            return self.log.log("operator_required",
                                why="aircraft is inside the new restricted zone; holding")

        self.link.set_mode("LOITER")          # autopilot holds position while we plan
        t_hold = time.perf_counter()
        pos = self.link.telemetry.fresh_position(self.stale_limit_s)
        pending = self.pending_stops()
        try:
            new_plan = build_delivery_mission(
                (pos.lat, pos.lon),
                pending,
                self.zones,
                self.profile,
                home=self.home,
                include_takeoff=False,
                stop_modes=self.stop_modes,
                start_time=eval_time,
            )
        except MissionValidationError as e:
            self.link.set_mode("LOITER")
            return self.log.log("operator_required",
                                why=f"no safe replacement route: {e}")

        gate_failure = self._validate_candidate_replan(new_plan, pos, eval_time=eval_time)
        if gate_failure is not None:
            return gate_failure

        plan_ms = (time.perf_counter() - t_hold) * 1000
        old = plan_summary(self.plan)
        self._sync_payload_modes(new_plan)
        self.upload(new_plan)
        self.link.set_current_item(1)
        self.link.set_mode("AUTO")
        total_ms = (time.perf_counter() - t0) * 1000
        rec = {"reason": reason, "plan_ms": round(plan_ms, 1), "total_ms": round(total_ms, 1),
               "old_planned_km": old["planned_km"], "new_planned_km": round(new_plan.planned_km, 3),
               "dropped_unreachable": [s.id for s in new_plan.unreachable],
               "selected_modes": dict(new_plan.selected_modes),
               "version": self.state.mission_version}
        self.state.replans.append(rec)
        self.log.log("mission_replanned", old_mission=old, **rec)
        return rec

    def apply_site_suitability_update(
        self,
        stop_id: int,
        new_suitability,
        reason: str,
        source: Optional[dict] = None,
        now: Optional[datetime] = None,
        force_replan: bool = False,
    ) -> dict:
        """Update a pending stop's site suitability in flight and rebuild/upload the mission if its mode changes.

        Ensures the planner, uploaded MAVLink commands, and PayloadManager never disagree
        when site conditions degrade (e.g. from 'land' to 'hover' or 'abort').
        """
        from flightstack.payload import resolve_stop_delivery_mode

        t0 = time.perf_counter()
        eval_time = now or datetime.now(timezone.utc)
        stop_obj = next((s for s in self.plan.stops if s.id == stop_id), None)
        if stop_obj is None:
            return self.log.log(
                "site_suitability_update_ignored",
                stop_id=stop_id,
                reason=reason,
                why="stop not in active plan",
            )
        if stop_id in self.state.delivered:
            return self.log.log(
                "site_suitability_update_ignored",
                stop_id=stop_id,
                reason=reason,
                why="stop already delivered",
            )

        prev_mode = getattr(self.plan, "selected_modes", {}).get(stop_id)
        site_obj = SiteSuitability.from_any(new_suitability)
        stop_obj.site_suitability = site_obj
        eff_req = self.stop_modes.get(stop_id, self.profile.delivery_mode)
        ok, new_mode, assess_reason, _ = resolve_stop_delivery_mode(
            stop_obj, requested_mode=self.profile.delivery_mode, stop_modes=self.stop_modes
        )
        self.payload.assess_site(stop_id, site_obj, eff_req, self.log.elapsed())

        self.log.log(
            "site_suitability_updated",
            stop_id=stop_id,
            reason=reason,
            source=source or {},
            previous_mode=prev_mode,
            new_mode=new_mode,
            site_ok=ok,
            assess_reason=assess_reason,
            site_suitability=site_obj.to_dict(),
        )

        if not force_replan and ok and new_mode == prev_mode:
            return {
                "event": "site_suitability_no_mode_change",
                "stop_id": stop_id,
                "selected_mode": new_mode,
                "site_ok": ok,
            }

        try:
            pos = self.link.telemetry.fresh_position(self.stale_limit_s)
        except StaleTelemetryError as e:
            return self.log.log(
                "site_suitability_update_deferred",
                stop_id=stop_id,
                reason=reason,
                why=str(e),
            )

        self.link.set_mode("LOITER")
        t_hold = time.perf_counter()
        pending = self.pending_stops()
        try:
            new_plan = build_delivery_mission(
                (pos.lat, pos.lon),
                pending,
                self.zones,
                self.profile,
                home=self.home,
                include_takeoff=False,
                stop_modes=self.stop_modes,
                start_time=eval_time,
            )
        except MissionValidationError as e:
            self.link.set_mode("LOITER")
            return self.log.log(
                "operator_required",
                why=f"no safe replacement route after site update at stop {stop_id}: {e}",
                stop_id=stop_id,
            )

        if not new_plan.stops and new_plan.unreachable:
            # All remaining stops were rejected as 'abort' -> still validate and fly RTL home
            self.log.log(
                "site_abort_returning_home",
                dropped_stops=[s.id for s in new_plan.unreachable],
                reason=assess_reason,
            )

        gate_failure = self._validate_candidate_replan(new_plan, pos, eval_time=eval_time)
        if gate_failure is not None:
            return gate_failure

        plan_ms = (time.perf_counter() - t_hold) * 1000
        old = plan_summary(self.plan)
        self._sync_payload_modes(new_plan)
        self.upload(new_plan)
        self.link.set_current_item(1)
        self.link.set_mode("AUTO")
        total_ms = (time.perf_counter() - t0) * 1000
        rec = {
            "reason": reason,
            "stop_id": stop_id,
            "previous_mode": prev_mode,
            "new_mode": new_mode,
            "plan_ms": round(plan_ms, 1),
            "total_ms": round(total_ms, 1),
            "old_planned_km": old["planned_km"],
            "new_planned_km": round(new_plan.planned_km, 3),
            "dropped_unreachable": [s.id for s in new_plan.unreachable],
            "selected_modes": dict(new_plan.selected_modes),
            "version": self.state.mission_version,
        }
        self.state.replans.append(rec)
        self.log.log("mission_replanned", old_mission=old, **rec)
        return rec

    def poll_external_updates(self, now: Optional[datetime] = None) -> dict:
        """Poll attached DynamicAirspaceMonitor and GDACSMissionImpactAssessor during flight."""
        eval_time = now or datetime.now(timezone.utc)
        results: Dict[str, dict] = {}
        pos = None
        try:
            pos = self.link.telemetry.fresh_position(self.stale_limit_s)
        except Exception:
            pass

        # 1. Dynamic Airspace Monitor
        if self.airspace_monitor is not None:
            rem_pts = self.remaining_polyline(pos) if pos is not None else None
            alt_agl = pos.alt_rel_m if pos is not None else self.profile.cruise_alt_m
            as_upd = self.airspace_monitor.check_for_updates(
                self.zones, route_waypoints=rem_pts, alt_m_agl=alt_agl, now=eval_time
            )
            results["airspace_monitor"] = as_upd
            if not as_upd.get("available", False):
                self.log.log(
                    "airspace_source_degraded",
                    status=as_upd.get("status"),
                    error=as_upd.get("error"),
                    safe_to_clear_restrictions=as_upd.get("safe_to_clear_restrictions"),
                )
            else:
                if as_upd.get("expired_zone_ids"):
                    self.log.log(
                        "airspace_restriction_expired",
                        expired_zone_ids=as_upd["expired_zone_ids"],
                        provider=as_upd.get("provider"),
                    )
                    self.zones = _normalize_zones(as_upd["active_zones"])
                if as_upd.get("added_zone_ids"):
                    self.apply_airspace_update(
                        as_upd["active_zones"],
                        reason=f"dynamic_airspace_monitor_{as_upd.get('provider', 'external')}",
                        source={
                            "provider": as_upd.get("provider"),
                            "source": as_upd.get("source"),
                            "provenance_classification": as_upd.get("source_classification"),
                            "retrieved_at": as_upd.get("retrieved_at"),
                            "freshness_age_s": as_upd.get("freshness_age_s"),
                            "added_zone_ids": as_upd.get("added_zone_ids"),
                            "confidence": as_upd.get("confidence"),
                        },
                        now=eval_time,
                    )

        # 2. Operational GDACS Mission Impact Assessor
        if self.gdacs_assessor is not None:
            pending = self.pending_stops()
            gdacs_impact = self.gdacs_assessor.evaluate_mission_impact(
                stops=pending,
                leg_paths=self.plan.leg_paths,
                home=self.home,
                now=eval_time,
            )
            results["gdacs_impact"] = gdacs_impact
            if not gdacs_impact.get("available", False):
                self.log.log(
                    "gdacs_source_degraded",
                    status=gdacs_impact.get("status"),
                    error=gdacs_impact.get("error"),
                    attribution=gdacs_impact.get("attribution"),
                )
            else:
                existing_zone_ids = {getattr(z, "zone_id", None) for z in self.zones if hasattr(z, "zone_id")}
                new_hazard_zones = [
                    hz for hz in (gdacs_impact.get("hazard_avoidance_zones") or [])
                    if getattr(hz, "zone_id", None) not in existing_zone_ids
                ]
                pending_stop_updates = []
                for sup in gdacs_impact.get("stop_suitability_updates") or []:
                    s_obj = next((s for s in pending if s.id == sup["stop_id"]), None)
                    if s_obj is None:
                        continue
                    cur_site = SiteSuitability.from_any(getattr(s_obj, "site_suitability", None))
                    new_site = SiteSuitability.from_any(sup["new_suitability"])
                    if (
                        cur_site.provenance != new_site.provenance
                        or cur_site.surface_condition != new_site.surface_condition
                        or cur_site.is_safe_for_landing != new_site.is_safe_for_landing
                        or cur_site.recommended_mode != new_site.recommended_mode
                    ):
                        pending_stop_updates.append(sup)

                if gdacs_impact.get("changed", False) or pending_stop_updates or new_hazard_zones:
                    self.log.log(
                        "gdacs_mission_impact_assessed",
                        status=gdacs_impact.get("status"),
                        provenance=gdacs_impact.get("provenance"),
                        total_active_events=gdacs_impact.get("total_active_events"),
                        advisory_count=len(gdacs_impact.get("advisory_events") or []),
                        stop_updates_count=len(pending_stop_updates),
                        hazard_zones_count=len(new_hazard_zones),
                        attribution=gdacs_impact.get("attribution"),
                    )
                    for sup in pending_stop_updates:
                        self.apply_site_suitability_update(
                            sup["stop_id"],
                            sup["new_suitability"],
                            reason=f"gdacs_hazard_polygon_{sup['event_key']}",
                            source={
                                "provider": "GDACS",
                                "event_key": sup["event_key"],
                                "classification": sup["classification"],
                                "attribution": gdacs_impact.get("attribution"),
                            },
                            now=eval_time,
                        )
                    if new_hazard_zones:
                        self.apply_airspace_update(
                            self.zones + new_hazard_zones,
                            reason="gdacs_red_hazard_polygon_avoidance",
                            source={
                                "provider": "GDACS",
                                "classification": "VALIDATED_HAZARD_POLYGON",
                                "is_legal_aviation_nfz": False,
                                "attribution": gdacs_impact.get("attribution"),
                            },
                            now=eval_time,
                        )

        return results


