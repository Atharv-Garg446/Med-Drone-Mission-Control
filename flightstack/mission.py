"""
flightstack/mission.py
======================
Turns a planner route (ordered delivery stops) into an autopilot mission
(a list of MAVLink MISSION_ITEM_INT-equivalent records) for ArduCopter.

Global planning stays in the existing modules: every leg is routed with
``nofly_astar.route_avoiding_zones`` so the uploaded waypoints follow the same
buffered no-fly detours the VRP solver costed. Before a mission is returned it
is re-validated segment by segment against the zones + safety margin; a
mission that would cross restricted airspace is rejected, never uploaded.

Delivery modes
--------------
``hover``  Descend to a release altitude over the stop, hold, trigger the
           autopilot gripper (MAV_CMD_DO_GRIPPER release), climb back.
           Does not require a landing pad.
``land``   MAV_CMD_NAV_LAND at the stop, then MAV_CMD_NAV_TAKEOFF to resume.
           Only appropriate where a clear landing site is known.

This module is pure Python (no pymavlink) so it is covered by fast unit tests.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Sequence, Tuple

from config import Location
from geo_utils import haversine_km, segment_intersects_polygon, point_near_polygon
from nofly_astar import route_avoiding_zones

Point = Tuple[float, float]

# MAVLink command ids (common.xml)
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_NAV_DELAY = 93
MAV_CMD_DO_CHANGE_SPEED = 178
MAV_CMD_DO_GRIPPER = 211
GRIPPER_ACTION_RELEASE = 0

MAV_FRAME_GLOBAL = 0
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
MAV_FRAME_MISSION = 2

DELIVERY_MODES = ("auto", "hover", "land")
CONCRETE_DELIVERY_MODES = ("hover", "land")


class MissionValidationError(ValueError):
    """Raised when a generated mission would violate airspace constraints."""


@dataclass
class MissionItem:
    seq: int
    command: int
    frame: int = MAV_FRAME_GLOBAL_RELATIVE_ALT
    params: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    lat: float = 0.0
    lon: float = 0.0
    alt_m: float = 0.0
    autocontinue: int = 1
    # Planner annotation (not sent to the autopilot)
    role: str = ""
    stop_id: Optional[int] = None

    @property
    def is_nav(self) -> bool:
        return self.command in (MAV_CMD_NAV_WAYPOINT, MAV_CMD_NAV_LAND,
                                MAV_CMD_NAV_TAKEOFF, MAV_CMD_NAV_RETURN_TO_LAUNCH,
                                MAV_CMD_NAV_DELAY)


@dataclass
class MissionProfile:
    cruise_alt_m: float = 60.0          # AGL relative to home; <120 m (400 ft)
    release_alt_m: float = 15.0         # hover-release altitude
    hover_hold_s: float = 5.0
    cruise_speed_ms: Optional[float] = None
    delivery_mode: str = "hover"        # "auto" | "hover" | "land"
    gripper_instance: int = 1
    cell_size_km: float = 0.25
    safety_margin_m: float = 50.0
    vehicle_model: Optional[object] = None
    enforce_hard_deadlines: bool = True
    enforce_cold_chain: bool = True

    def __post_init__(self):
        if self.delivery_mode not in DELIVERY_MODES:
            raise ValueError("delivery_mode must be one of %s" % (DELIVERY_MODES,))
        if not (0 < self.release_alt_m <= self.cruise_alt_m):
            raise ValueError("release_alt_m must be in (0, cruise_alt_m]")
        if self.cruise_alt_m > 120.0:
            raise ValueError("cruise_alt_m above 120 m AGL is outside the green-zone ceiling")


@dataclass
class MissionPlan:
    items: List[MissionItem]
    stops: List[Location]
    leg_paths: List[List[Point]]                 # flown polyline per leg
    delivery_seq: Dict[int, int] = field(default_factory=dict)  # stop_id -> seq that completes stop delivery
    release_seq: Dict[int, int] = field(default_factory=dict)   # stop_id -> MAV_CMD_DO_GRIPPER seq
    dwell_seq: Dict[int, int] = field(default_factory=dict)     # stop_id -> MAV_CMD_NAV_DELAY seq (land mode)
    unreachable: List[Location] = field(default_factory=list)
    selected_modes: Dict[int, str] = field(default_factory=dict)  # stop_id -> 'hover' | 'land'
    site_assessments: Dict[int, dict] = field(default_factory=dict)  # stop_id -> assessment dict

    @property
    def planned_km(self) -> float:
        return sum(_polyline_km(p) for p in self.leg_paths)

    def nav_points(self) -> List[Point]:
        return [(it.lat, it.lon) for it in self.items
                if it.is_nav and it.command != MAV_CMD_NAV_RETURN_TO_LAUNCH and it.seq > 0
                and (it.lat != 0.0 or it.lon != 0.0)]

    def full_route_polyline(self, home: Optional[Point] = None) -> List[Point]:
        """Concatenate all leg_paths (outbound + inter-stop + return) into a single continuous polyline."""
        pts: List[Point] = []
        for leg in self.leg_paths:
            for pt in leg:
                if not pts or pts[-1] != pt:
                    pts.append(pt)
        if not pts:
            nav = self.nav_points()
            if nav:
                pts.extend(nav)
            if home and (not pts or pts[-1] != home):
                pts.append(home)
        return pts


def _polyline_km(pts: Sequence[Point]) -> float:
    return sum(haversine_km(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def _iter_zone_items(zones):
    """Accept a list of polygons/AirspaceZones or an AirspaceSnapshot."""
    if hasattr(zones, "zones"):
        return zones.zones
    return zones


def validate_polyline(
    points: Sequence[Point],
    zones,
    margin_m: float,
    endpoint_tolerance: bool = True,
    alt_m_agl: Optional[float] = None,
    start_time: Optional[datetime] = None,
    cruise_speed_ms: Optional[float] = None,
) -> None:
    """Raise MissionValidationError if any segment enters a zone+margin in space and time."""
    zone_list = _iter_zone_items(zones)
    current_time = start_time
    for i in range(len(points) - 1):
        p1, p2 = points[i], points[i + 1]
        dt_s = 0.0
        if cruise_speed_ms and cruise_speed_ms > 0:
            seg_dist_km = haversine_km(p1, p2)
            dt_s = (seg_dist_km * 1000.0) / cruise_speed_ms
        seg_start_t = current_time
        seg_end_t = (current_time + timedelta(seconds=dt_s)) if current_time else None

        for zone in zone_list:
            if hasattr(zone, "category") and zone.category == "green":
                continue
            if hasattr(zone, "altitude_conflicts") and not zone.altitude_conflicts(alt_m_agl):
                continue
            if hasattr(zone, "time_conflicts") and not zone.time_conflicts(seg_start_t, seg_end_t):
                continue

            poly = zone.polygon if hasattr(zone, "polygon") else zone
            if segment_intersects_polygon(p1, p2, poly, margin_m=margin_m):
                # The very first point may legitimately be the aircraft's
                # current position inside a newly-declared zone; that is
                # reported separately by the caller, not silently accepted.
                raise MissionValidationError(
                    "segment %d %s -> %s violates a restricted zone (+%.0f m)"
                    % (i, p1, p2, margin_m))

        if current_time and dt_s > 0:
            current_time = seg_end_t


def build_delivery_mission(
    start: Point,
    stops: Sequence[Location],
    zones,
    profile: MissionProfile,
    home: Optional[Point] = None,
    include_takeoff: bool = True,
    stop_modes: Optional[Dict[int, str]] = None,
    return_to: Optional[Point] = None,
    start_time: Optional[datetime] = None,
) -> MissionPlan:
    """Build an ArduCopter AUTO mission visiting ``stops`` in the given order.

    ``start`` is where the vehicle is (home for a fresh sortie, the current
    autopilot position estimate for an in-flight update). Stops that cannot be
    reached without entering restricted airspace are skipped and listed in
    ``MissionPlan.unreachable`` instead of producing an illegal straight line.
    The mission ends with RTL (``return_to`` is only used to validate and cost
    the final leg; the autopilot flies RTL to its own home).
    """
    stop_modes = stop_modes or {}
    home = home or start
    margin_km = profile.safety_margin_m / 1000.0
    items: List[MissionItem] = []

    def add(**kw) -> MissionItem:
        it = MissionItem(seq=len(items), **kw)
        items.append(it)
        return it

    # seq 0: home placeholder -- ArduPilot overwrites it with its own home.
    add(command=MAV_CMD_NAV_WAYPOINT, frame=MAV_FRAME_GLOBAL, lat=home[0], lon=home[1],
        alt_m=0.0, role="home")
    if include_takeoff:
        add(command=MAV_CMD_NAV_TAKEOFF, lat=0.0, lon=0.0, alt_m=profile.cruise_alt_m,
            role="takeoff")
    if profile.cruise_speed_ms:
        add(command=MAV_CMD_DO_CHANGE_SPEED, frame=MAV_FRAME_MISSION,
            params=(1.0, float(profile.cruise_speed_ms), -1.0, 0.0), role="speed")

    plan = MissionPlan(items=items, stops=[], leg_paths=[])
    cur = start
    zone_list = list(_iter_zone_items(zones))
    cur_time = start_time
    spd_ms = profile.cruise_speed_ms or 15.0

    def _active_polygons_for_leg(p_from: Point, p_to: Point, leg_start: Optional[datetime]):
        leg_end = None
        if leg_start is not None:
            # Conservative upper bound (3x straight-line distance) for detour search window
            est_s = max(60.0, (haversine_km(p_from, p_to) * 3000.0) / spd_ms)
            leg_end = leg_start + timedelta(seconds=est_s)
        polys = []
        for z in zone_list:
            if hasattr(z, "category") and z.category == "green":
                continue
            if hasattr(z, "altitude_conflicts") and not z.altitude_conflicts(profile.cruise_alt_m):
                continue
            if hasattr(z, "time_conflicts") and not z.time_conflicts(leg_start, leg_end):
                continue
            polys.append(z.polygon if hasattr(z, "polygon") else z)
        return polys

    from flightstack.payload import resolve_stop_delivery_mode

    for stop in stops:
        ok, mode, assess_reason, site = resolve_stop_delivery_mode(
            stop, requested_mode=profile.delivery_mode, stop_modes=stop_modes
        )
        eff_req = (stop_modes or {}).get(stop.id, profile.delivery_mode)
        plan.site_assessments[stop.id] = {
            "stop_id": stop.id,
            "ok": ok,
            "requested_mode": eff_req,
            "selected_mode": mode,
            "reason": assess_reason,
            "site_suitability": site.to_dict(),
        }
        if not ok or mode == "abort" or mode not in CONCRETE_DELIVERY_MODES:
            plan.unreachable.append(stop)
            continue

        target = (stop.lat, stop.lon)
        planning_polygons = _active_polygons_for_leg(cur, target, cur_time)
        path, dist_km, _ = route_avoiding_zones(cur, target, planning_polygons,
                                                profile.cell_size_km, margin_km)
        if path is None or dist_km == float("inf"):
            plan.unreachable.append(stop)
            continue
        validate_polyline(path, zone_list, profile.safety_margin_m,
                          alt_m_agl=profile.cruise_alt_m, start_time=cur_time,
                          cruise_speed_ms=profile.cruise_speed_ms)
        if cur_time is not None:
            leg_s = (dist_km * 1000.0) / spd_ms + float(profile.hover_hold_s)
            cur_time = cur_time + timedelta(seconds=leg_s)
        plan.leg_paths.append(list(path))
        plan.stops.append(stop)
        plan.selected_modes[stop.id] = mode
        for p in path[1:-1]:
            add(command=MAV_CMD_NAV_WAYPOINT, lat=p[0], lon=p[1],
                alt_m=profile.cruise_alt_m, role="detour")
        add(command=MAV_CMD_NAV_WAYPOINT, lat=target[0], lon=target[1],
            alt_m=profile.cruise_alt_m, role="arrive", stop_id=stop.id)
        if mode == "hover":
            add(command=MAV_CMD_NAV_WAYPOINT, params=(profile.hover_hold_s, 0, 0, 0),
                lat=target[0], lon=target[1], alt_m=profile.release_alt_m,
                role="hover_descend", stop_id=stop.id)
            rel = add(command=MAV_CMD_DO_GRIPPER, frame=MAV_FRAME_MISSION,
                      params=(float(profile.gripper_instance), GRIPPER_ACTION_RELEASE, 0, 0),
                      role="release", stop_id=stop.id)
            climb = add(command=MAV_CMD_NAV_WAYPOINT, lat=target[0], lon=target[1],
                        alt_m=profile.cruise_alt_m, role="climb", stop_id=stop.id)
            plan.release_seq[stop.id] = rel.seq
            # Delivery counts as done once the vehicle has released the package
            # and advanced to the post-release climb waypoint.
            plan.delivery_seq[stop.id] = climb.seq
        else:
            add(command=MAV_CMD_NAV_LAND, lat=target[0], lon=target[1], alt_m=0.0,
                role="land", stop_id=stop.id)
            # Explicit package release action while landed on the ground
            rel = add(command=MAV_CMD_DO_GRIPPER, frame=MAV_FRAME_MISSION,
                      params=(float(profile.gripper_instance), GRIPPER_ACTION_RELEASE, 0, 0),
                      role="land_release", stop_id=stop.id)
            # Ground dwell via NAV_DELAY (93) under AP_Mission ContinueAfterLand (MIS_OPTIONS=4)
            dwell_s = max(float(profile.hover_hold_s), 5.0)
            dwell = add(command=MAV_CMD_NAV_DELAY, params=(dwell_s, -1, -1, -1),
                        lat=0.0, lon=0.0, alt_m=0.0,
                        role="ground_dwell", stop_id=stop.id)
            to = add(command=MAV_CMD_NAV_TAKEOFF, alt_m=profile.cruise_alt_m,
                     role="retakeoff", stop_id=stop.id)
            plan.release_seq[stop.id] = rel.seq
            plan.dwell_seq[stop.id] = dwell.seq
            # Ground delivery is only complete AFTER the configured ground dwell has finished
            # (i.e. when the autopilot advances from ground_dwell to retakeoff).
            plan.delivery_seq[stop.id] = to.seq
        cur = target

    # Return leg: RTL flies a straight line home, so it must be validated too.
    ret = return_to or home
    planning_polygons = _active_polygons_for_leg(cur, ret, cur_time)
    path, dist_km, _ = route_avoiding_zones(cur, ret, planning_polygons, profile.cell_size_km, margin_km)
    if path is None or dist_km == float("inf"):
        raise MissionValidationError("no legal return path from %s to home" % (cur,))
    validate_polyline(path, zone_list, profile.safety_margin_m,
                      alt_m_agl=profile.cruise_alt_m, start_time=cur_time,
                      cruise_speed_ms=profile.cruise_speed_ms)
    plan.leg_paths.append(list(path))
    for p in path[1:-1]:
        add(command=MAV_CMD_NAV_WAYPOINT, lat=p[0], lon=p[1],
            alt_m=profile.cruise_alt_m, role="detour_home")
    add(command=MAV_CMD_NAV_RETURN_TO_LAUNCH, frame=MAV_FRAME_MISSION, role="rtl")
    return plan


def mission_to_qgc_wpl(plan: MissionPlan) -> str:
    """Serialise to QGC WPL 110 text (same format as waypoint_export.py)."""
    lines = ["QGC WPL 110"]
    for it in plan.items:
        p = it.params
        lines.append("\t".join(str(x) for x in (
            it.seq, 1 if it.seq == 0 else 0, it.frame, it.command,
            p[0], p[1], p[2], p[3], "%.7f" % it.lat, "%.7f" % it.lon,
            "%.2f" % it.alt_m, it.autocontinue)))
    return "\n".join(lines) + "\n"


def point_in_restricted(
    point: Point,
    zones,
    margin_m: float = 0.0,
    alt_m_agl: Optional[float] = None,
    when: Optional[datetime] = None,
) -> bool:
    for z in _iter_zone_items(zones):
        if hasattr(z, "category") and z.category == "green":
            continue
        if hasattr(z, "altitude_conflicts") and not z.altitude_conflicts(alt_m_agl):
            continue
        if hasattr(z, "time_conflicts") and not z.time_conflicts(when, when):
            continue
        poly = z.polygon if hasattr(z, "polygon") else z
        if point_near_polygon(point, poly, margin_m):
            return True
    return False
