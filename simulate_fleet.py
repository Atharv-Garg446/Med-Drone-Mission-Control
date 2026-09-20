"""
simulate_fleet.py
=================
Discrete-event fleet simulator for drone medical-supply delivery.

Runs a solved set of routes forward in simulated time, tracking every
drone's GPS position, battery, and cargo second by second. At random
(or user-specified) intervals, injects disruptive events:

  - Pop-up TFR (Temporary Flight Restriction): a new no-fly zone appears
    near a drone's future path, forcing rerouting via A*.
  - Emergency delivery: a new critical Location appears mid-mission,
    requiring instant replanning.
  - Drone failure: a drone "goes down," and its remaining undelivered
    stops must be redistributed to other active drones.

On every disruption, the simulator calls replan.py to re-solve the CVRP
for the remaining undelivered stops, measures the replan time, and
reassigns routes to active drones.

Usage:
    python3 simulate_fleet.py                               # default
    python3 simulate_fleet.py --city chennai_flood           # different city
    python3 simulate_fleet.py --events 5                     # more chaos
    python3 simulate_fleet.py --seed 42                      # reproducible
    python3 simulate_fleet.py --speed 10                     # 10x real-time
    python3 simulate_fleet.py --no-random --inject-tfr 120   # manual TFR at T+120s
"""

import argparse
import math
import random
import sys
import time
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple

from config import (
    CityConfig, DroneConfig, Location,
    CITY_CONFIGS, JAIPUR_DISASTER,
)
from distance_matrix import build_flight_distance_matrix
from vrp_scratch import solve_vrp_from_scratch, total_distance, is_route_feasible
from geo_utils import haversine_km
from replan import replan_from_remaining, try_insert_emergency_into_active_drones, ReplanResult
from nofly_astar import route_avoiding_zones
from cold_chain import check_cold_chain_all_routes
from time_windows import check_time_windows


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class DroneState:
    """Tracks one drone's real-time state during the simulation."""
    drone_id: int
    route: List[int]               # current route (indices into locations[])
    route_step: int = 0            # which leg we're on (route[step] -> route[step+1])
    leg_progress_km: float = 0.0   # km traveled on current leg
    total_distance_km: float = 0.0
    lat: float = 0.0
    lon: float = 0.0
    remaining_cargo_kg: float = 0.0
    status: str = "idle"         # idle, flying, delivering, returning, completed, failed
    deliveries_completed: List[int] = field(default_factory=list)
    launch_tick: int = 0
    available_tick: int = 0
    sortie_distance_km: float = 0.0
    sortie_launch_tick: int = 0
    active_path: List[Tuple[float, float]] = field(default_factory=list)

    @property
    def current_from(self) -> int:
        return self.route[self.route_step]

    @property
    def current_to(self) -> int:
        return self.route[self.route_step + 1]

    @property
    def remaining_stops(self) -> List[int]:
        """Location indices not yet delivered (excluding depot=0)."""
        future = self.route[self.route_step + 1:]
        return [n for n in future if n != 0]


@dataclass
class SimEvent:
    """A scheduled disruption event."""
    tick: int
    event_type: str    # "tfr", "emergency", "drone_failure"
    data: dict = field(default_factory=dict)
    handled: bool = False


@dataclass
class MissionReport:
    """Summary of the completed simulation."""
    total_ticks: int
    total_seconds: float
    deliveries_planned: int
    deliveries_completed: int
    events_injected: int
    events_handled: int
    replans_performed: int
    total_replan_time_ms: float
    avg_replan_time_ms: float
    cold_chain_violations: int
    time_window_misses: int
    total_flight_distance_km: float
    original_plan_distance_km: float
    distance_increase_pct: float
    drone_failures: int
    unserviceable_deliveries: int = 0


# ---------------------------------------------------------------------------
# Event generation
# ---------------------------------------------------------------------------

def generate_random_events(
    city: CityConfig,
    num_events: int,
    max_tick: int,
    rng: random.Random,
) -> List[SimEvent]:
    """Generate random disruption events spread across the simulation."""
    events = []
    event_types = ["tfr", "emergency", "drone_failure"]
    weights = [0.4, 0.4, 0.2]  # TFRs and emergencies more common

    for _ in range(num_events):
        tick = rng.randint(max_tick // 5, max_tick * 4 // 5)
        etype = rng.choices(event_types, weights=weights, k=1)[0]

        if etype == "tfr":
            # Random TFR near one of the delivery locations
            target = rng.choice(city.deliveries)
            offset_lat = rng.uniform(-0.015, 0.015)
            offset_lon = rng.uniform(-0.015, 0.015)
            size = rng.uniform(0.005, 0.015)
            center_lat = target.lat + offset_lat
            center_lon = target.lon + offset_lon
            zone = [
                (center_lat - size, center_lon - size),
                (center_lat - size, center_lon + size),
                (center_lat + size, center_lon + size),
                (center_lat + size, center_lon - size),
            ]
            events.append(SimEvent(
                tick=tick, event_type="tfr",
                data={"zone": zone, "reason": f"Pop-up TFR near {target.name.split(',')[0]}"},
            ))

        elif etype == "emergency":
            # Random emergency near the depot
            offset_lat = rng.uniform(-0.03, 0.03)
            offset_lon = rng.uniform(-0.03, 0.03)
            events.append(SimEvent(
                tick=tick, event_type="emergency",
                data={
                    "location": Location(
                        id=999,
                        name=f"EMERGENCY — Injected at T+{tick}s",
                        lat=city.depot.lat + offset_lat,
                        lon=city.depot.lon + offset_lon,
                        demand_kg=rng.uniform(1.5, 5.0),
                        urgency="critical",
                        window_minutes=rng.choice([10, 15, 20]),
                        cold_chain_limit_minutes=rng.choice([None, 30, 45]),
                    ),
                },
            ))

        elif etype == "drone_failure":
            events.append(SimEvent(
                tick=tick, event_type="drone_failure",
                data={"reason": "Simulated battery failure"},
            ))

    events.sort(key=lambda e: e.tick)
    return events


# ---------------------------------------------------------------------------
# Core simulator
# ---------------------------------------------------------------------------

class FleetSimulator:
    """Discrete-event fleet simulator with real-time replanning."""

    def __init__(
        self,
        city: CityConfig,
        events: List[SimEvent] = None,
        speed_multiplier: float = 1.0,
        live_print: bool = True,
        seed: Optional[int] = None,
    ):
        self.city = city
        self.locations = city.all_locations
        self.drone_config = city.drone
        self.no_fly_zones = list(city.no_fly_zones)
        self.events = sorted(events or [], key=lambda e: e.tick)
        self.speed_multiplier = speed_multiplier
        self.live_print = live_print
        self.seed = seed
        self.rng = random.Random(seed)

        # Build initial plan
        self.matrix, self.detours = build_flight_distance_matrix(
            self.locations, self.no_fly_zones, drone_config=self.drone_config
        )
        self.original_routes = solve_vrp_from_scratch(
            self.locations, self.matrix, self.drone_config
        )
        self.original_distance = total_distance(self.original_routes, self.matrix)

        # All delivery location IDs (excluding depot)
        self.all_delivery_ids = set(range(1, len(self.locations)))
        self.delivered_ids = set()
        self.unserviceable_ids = set()
        self.emergency_locations = []  # dynamically added locations

        # Stats
        self.tick = 0
        self.replans = 0
        self.total_replan_ms = 0.0
        self.cold_chain_violations = 0
        self.time_window_misses = 0
        self.drone_failures = 0
        self.events_handled = 0
        self.log: List[str] = []

        self.unassigned_routes = list(self.original_routes)
        self.drones: List[DroneState] = []
        self._init_drones()

        # --- Visual replay recording ---
        # Records every drone position every N ticks for the animated map
        self.record_interval = 5  # record every 5 seconds (ticks)
        self.position_history: List[dict] = []   # {tick, drone_id, lat, lon, status}
        self.event_history: List[dict] = []       # {tick, type, lat, lon, data}
        self.delivery_history: List[dict] = []    # {tick, drone_id, location_name, lat, lon}
        self.nfz_history: List[dict] = []         # {tick, zone_coords} — dynamic TFRs

    def _init_drones(self):
        """Create the physical drone fleet and start assigning routes."""
        depot = self.locations[0]
        num_drones = self.drone_config.fleet_size
        for i in range(num_drones):
            self.drones.append(DroneState(
                drone_id=i + 1,
                route=[],
                lat=depot.lat,
                lon=depot.lon,
                status="idle",
            ))

    def _init_leg_path(self, drone: DroneState) -> bool:
        """Initialize the active waypoint path for a drone from its CURRENT GPS position to its target stop.
        Returns True if a safe collision-free path was found, or False if unreachable (never uses straight-line fallback)."""
        if drone.route_step >= len(drone.route) - 1:
            return True
        p_start = (drone.lat, drone.lon)
        to_loc = self._get_location(drone.current_to)
        if not to_loc:
            return False
        p_end = (to_loc.lat, to_loc.lon)
        buffer_km = self.drone_config.safety_margin_m / 1000.0
        waypoints, dist_km, rerouted = route_avoiding_zones(p_start, p_end, self.no_fly_zones, buffer_km=buffer_km)
        if waypoints is None or math.isinf(dist_km):
            drone.active_path = []
            drone.active_path_length_km = float('inf')
            drone.leg_progress_km = 0.0
            return False
        drone.active_path = waypoints
        drone.active_path_length_km = dist_km
        drone.leg_progress_km = 0.0
        return True

    def _replan_active_drone(self, drone: DroneState) -> bool:
        """Re-evaluates an active drone's mission after an airspace change (TFR/NFZ) or route blockage.
        
        Decision Process:
        1. Attempt safe continuation to current destination.
        2. Evaluate remaining onboard deliveries against remaining battery range/reserve,
           delivery deadlines, cold-chain exposure, and dwell times.
        3. Reorder remaining stops if another permutation provides a feasible mission.
        4. If full remaining mission is infeasible, drop unreachable/violating stops to depot queue
           and deliver to feasible stops.
        5. If no remaining stops can be safely reached, execute Return to Launch (RTL) to depot.
        6. If even RTL is unreachable or out of battery range, transition to 'unserviceable'
           (holding / emergency landing) and return undelivered stops to queue.
        """
        if drone.status not in ("flying", "delivering"):
            return False
        if drone.route_step >= len(drone.route) - 1:
            return True

        p_curr = (drone.lat, drone.lon)
        buffer_km = self.drone_config.safety_margin_m / 1000.0
        depot = self.locations[0]
        max_allowed_dist = self.drone_config.max_range_km * (1.0 - self.drone_config.reserve_fraction)

        target_idx = drone.current_to
        future_stops = [idx for idx in drone.route[drone.route_step + 2 : -1]]
        rem_stops = ([target_idx] if target_idx != 0 else []) + future_stops

        # Case A: Drone is on return-to-depot leg
        if not rem_stops:
            pts, ret_dist, _ = route_avoiding_zones(p_curr, (depot.lat, depot.lon), self.no_fly_zones, buffer_km=buffer_km)
            if pts is None or math.isinf(ret_dist) or (drone.sortie_distance_km + ret_dist > max_allowed_dist + 1e-9):
                drone.status = "unserviceable"
                drone.active_path = []
                self._log(f"🚨 Drone {drone.drone_id} UNSERVICEABLE: return path to depot blocked or exceeds battery reserve — holding/emergency landing at ({drone.lat:.4f}, {drone.lon:.4f})")
                return False
            drone.active_path = pts
            drone.active_path_length_km = ret_dist
            drone.leg_progress_km = 0.0
            return True

        # Case B: Drone has remaining deliveries onboard
        def evaluate_sequence(seq, check_deadlines=True):
            curr = p_curr
            tot_dist = 0.0
            first_pts = None
            first_dist = 0.0
            elapsed_sortie_min = (self.tick - drone.sortie_launch_tick) / 60.0
            current_mission_min = self.tick / 60.0

            for i, stop_idx in enumerate(seq):
                loc = self._get_location(stop_idx)
                if not loc:
                    return False, None, 0.0
                pts, leg_km, _ = route_avoiding_zones(curr, (loc.lat, loc.lon), self.no_fly_zones, buffer_km=buffer_km)
                if pts is None or math.isinf(leg_km):
                    return False, None, 0.0
                if i == 0:
                    first_pts = pts
                    first_dist = leg_km
                tot_dist += leg_km
                leg_time = (leg_km / self.drone_config.cruise_speed_kmh) * 60.0
                elapsed_sortie_min += leg_time
                current_mission_min += leg_time

                if loc.cold_chain_limit_minutes is not None and elapsed_sortie_min > loc.cold_chain_limit_minutes + 1e-9:
                    return False, None, 0.0
                if check_deadlines and loc.window_minutes is not None:
                    deadline_min = (getattr(loc, 'request_time_min', 0.0) or 0.0) + loc.window_minutes
                    if current_mission_min > deadline_min + 1e-9:
                        return False, None, 0.0

                elapsed_sortie_min += self.drone_config.dwell_min
                current_mission_min += self.drone_config.dwell_min
                curr = (loc.lat, loc.lon)

            # Return leg from last stop to depot
            pts_ret, ret_km, _ = route_avoiding_zones(curr, (depot.lat, depot.lon), self.no_fly_zones, buffer_km=buffer_km)
            if pts_ret is None or math.isinf(ret_km):
                return False, None, 0.0
            tot_dist += ret_km

            if drone.sortie_distance_km + tot_dist > max_allowed_dist + 1e-9:
                return False, None, 0.0

            return True, first_pts, first_dist

        import itertools
        # 1. Try on-time completion (original sequence, then permutations)
        ok, first_pts, first_dist = evaluate_sequence(rem_stops, check_deadlines=True)
        if ok:
            drone.active_path = first_pts
            drone.active_path_length_km = first_dist
            drone.leg_progress_km = 0.0
            return True

        if len(rem_stops) <= 5:
            cand_perms = list(itertools.permutations(rem_stops))
        else:
            cand_perms = [list(reversed(rem_stops))]

        for perm in cand_perms:
            if list(perm) == rem_stops:
                continue
            ok, p_pts, p_dist = evaluate_sequence(list(perm), check_deadlines=True)
            if ok:
                new_seq = list(perm)
                drone.route = drone.route[:drone.route_step + 1] + new_seq + [0]
                drone.active_path = p_pts
                drone.active_path_length_km = p_dist
                drone.leg_progress_km = 0.0
                self._log(f"🔄 Drone {drone.drone_id} reordered remaining stops for time-window feasibility: {new_seq}")
                return True

        # 2. Reordering cannot satisfy all stops: try subsets with strict deadline checks
        for drop_idx in range(len(rem_stops)):
            subset = [s for j, s in enumerate(rem_stops) if j != drop_idx]
            if not subset:
                continue
            ok, s_pts, s_dist = evaluate_sequence(subset, check_deadlines=True)
            if ok:
                dropped_stop = rem_stops[drop_idx]
                drone.route = drone.route[:drone.route_step + 1] + subset + [0]
                drone.active_path = s_pts
                drone.active_path_length_km = s_dist
                drone.leg_progress_km = 0.0
                dropped_loc = self._get_location(dropped_stop)
                loc_name = dropped_loc.name.split(",")[0] if dropped_loc else f"#{dropped_stop}"
                deadline_min = ((getattr(dropped_loc, 'request_time_min', 0.0) or 0.0) + dropped_loc.window_minutes) if (dropped_loc and dropped_loc.window_minutes) else float('inf')
                if (self.tick / 60.0) >= deadline_min:
                    self.unserviceable_ids.add(dropped_stop)
                    self.time_window_misses += 1
                    self._log(f"⚠️ Drone {drone.drone_id} dropped stop {loc_name}: deadline expired (T+{deadline_min:.0f}m) — marked unserviceable/missed")
                else:
                    self.unassigned_routes.append([0, dropped_stop, 0])
                    self._log(f"⚠️ Drone {drone.drone_id} dropped stop {loc_name} (infeasible on current flight) — queued for depot redistribution")
                    self._trigger_replan("Redistribute dropped stop")
                return True

        # 3. No remaining deliveries can be served on time; attempt Return to Launch (RTL)
        pts_rtl, dist_rtl, _ = route_avoiding_zones(p_curr, (depot.lat, depot.lon), self.no_fly_zones, buffer_km=buffer_km)
        if pts_rtl is not None and not math.isinf(dist_rtl) and (drone.sortie_distance_km + dist_rtl <= max_allowed_dist + 1e-9):
            for st in rem_stops:
                st_loc = self._get_location(st)
                dl = ((getattr(st_loc, 'request_time_min', 0.0) or 0.0) + st_loc.window_minutes) if (st_loc and st_loc.window_minutes) else float('inf')
                if (self.tick / 60.0) >= dl:
                    self.unserviceable_ids.add(st)
                    self.time_window_misses += 1
                else:
                    self.unassigned_routes.append([0, st, 0])
            drone.route = drone.route[:drone.route_step + 1] + [0]
            drone.active_path = pts_rtl
            drone.active_path_length_km = dist_rtl
            drone.leg_progress_km = 0.0
            self._log(f"↩️ Drone {drone.drone_id} aborting mission — executing RTL to depot (all remaining deliveries infeasible)")
            return True

        # 4. Even RTL is impossible; transition to unserviceable (emergency landing / holding)
        for st in rem_stops:
            self.unserviceable_ids.add(st)
            self.time_window_misses += 1
        drone.status = "unserviceable"
        drone.active_path = []
        self._log(f"🚨 Drone {drone.drone_id} UNSERVICEABLE: no safe path or insufficient battery to return to depot — holding/emergency landing at ({drone.lat:.4f}, {drone.lon:.4f})")
        return False

    def _assign_routes(self):
        """Assign any queued routes to available idle drones, prioritizing urgency/deadlines
        and verifying feasibility at actual departure time."""
        if not self.unassigned_routes:
            return

        def route_urgency_deadline_key(r):
            rank = 2
            earliest_dl = float("inf")
            for n in r[1:-1]:
                loc = self._get_location(n)
                if loc:
                    u_rank = {"critical": 0, "urgent": 1, "routine": 2}.get(loc.urgency, 2)
                    if u_rank < rank:
                        rank = u_rank
                    if loc.window_minutes is not None:
                        dl = (getattr(loc, 'request_time_min', 0.0) or 0.0) + loc.window_minutes
                        if dl < earliest_dl:
                            earliest_dl = dl
            return (rank, earliest_dl)

        self.unassigned_routes.sort(key=route_urgency_deadline_key)

        depot = self.locations[0]
        for drone in self.drones:
            if drone.status == "idle" and self.tick >= drone.available_tick and self.unassigned_routes:
                route = self.unassigned_routes.pop(0)

                # Recheck route feasibility at departure time
                depart_min = self.tick / 60.0
                all_locs = list(self.locations) + list(self.emergency_locations)
                feasible = is_route_feasible(route, all_locs, self.matrix, self.drone_config, depart_minutes=depart_min)
                if not feasible:
                    # Attempt to salvage the route through permutation or dropping unserviceable stops
                    stops = route[1:-1]
                    salvaged = None
                    if len(stops) <= 5:
                        import itertools
                        for perm in itertools.permutations(stops):
                            cand = [0] + list(perm) + [0]
                            if is_route_feasible(cand, all_locs, self.matrix, self.drone_config, depart_minutes=depart_min):
                                salvaged = cand
                                self._log(f"🔄 Reordered queued route {route} -> {cand} to satisfy departure deadlines at T+{depart_min:.0f}m")
                                break
                    if salvaged:
                        route = salvaged
                    else:
                        # Cannot satisfy all stops together: drop stops whose deadline has passed or will pass
                        feasible_subset = []
                        for st in stops:
                            st_loc = self._get_location(st)
                            dl = ((getattr(st_loc, 'request_time_min', 0.0) or 0.0) + st_loc.window_minutes) if (st_loc and st_loc.window_minutes is not None) else float('inf')
                            if depart_min >= dl:
                                self.unserviceable_ids.add(st)
                                self.time_window_misses += 1
                                loc_name = st_loc.name.split(",")[0] if st_loc else f"#{st}"
                                self._log(f"❌ Stop {loc_name} unserviceable: deadline expired (T+{dl:.0f}m) before launch at T+{depart_min:.0f}m")
                            else:
                                feasible_subset.append(st)
                        
                        valid_cand = None
                        if feasible_subset:
                            if len(feasible_subset) <= 5:
                                import itertools
                                for perm in itertools.permutations(feasible_subset):
                                    cand = [0] + list(perm) + [0]
                                    if is_route_feasible(cand, all_locs, self.matrix, self.drone_config, depart_minutes=depart_min):
                                        valid_cand = cand
                                        break
                            else:
                                cand = [0] + feasible_subset + [0]
                                if is_route_feasible(cand, all_locs, self.matrix, self.drone_config, depart_minutes=depart_min):
                                    valid_cand = cand
                        
                        if valid_cand:
                            route = valid_cand
                            self._log(f"✂️ Dispatched feasible subset {route} after dropping unserviceable stops")
                        else:
                            for st in feasible_subset:
                                cand = [0, st, 0]
                                if not is_route_feasible(cand, all_locs, self.matrix, self.drone_config, depart_minutes=depart_min):
                                    self.unserviceable_ids.add(st)
                                    self.time_window_misses += 1
                                    st_loc = self._get_location(st)
                                    loc_name = st_loc.name.split(",")[0] if st_loc else f"#{st}"
                                    self._log(f"❌ Stop {loc_name} unserviceable: infeasible even as single-stop sortie at T+{depart_min:.0f}m")
                                else:
                                    self.unassigned_routes.append(cand)
                            # Do not launch an infeasible sortie
                            continue

                cargo = sum(
                    (self._get_location(n).demand_kg if self._get_location(n) else 0)
                    for n in route[1:-1]
                )
                drone.route = route
                drone.route_step = 0
                drone.remaining_cargo_kg = cargo
                drone.status = "flying"
                drone.launch_tick = self.tick
                drone.sortie_launch_tick = self.tick
                drone.sortie_distance_km = 0.0
                safe = self._init_leg_path(drone)
                if not safe:
                    self._replan_active_drone(drone)
                
                route_names = " → ".join(
                    (self._get_location(n).name.split(",")[0]
                     if self._get_location(n) else f"#{n}")
                    for n in drone.route
                )
                self._log(f"🚁 Drone {drone.drone_id} launched: {route_names}")

    def _log(self, msg: str):
        """Log a message with timestamp."""
        minutes = self.tick // 60
        seconds = self.tick % 60
        line = f"T+{minutes:02d}:{seconds:02d}  {msg}"
        self.log.append(line)
        if self.live_print:
            print(line)

    def _get_leg_distance(self, from_idx: int, to_idx: int) -> float:
        """Get distance between two location indices."""
        if from_idx < len(self.matrix) and to_idx < len(self.matrix):
            return self.matrix[from_idx][to_idx]
        loc_a = self._get_location(from_idx)
        loc_b = self._get_location(to_idx)
        if loc_a and loc_b:
            return haversine_km((loc_a.lat, loc_a.lon), (loc_b.lat, loc_b.lon))
        return 0.0

    def _get_location(self, idx: int) -> Optional[Location]:
        """Get a location by index, including emergency additions."""
        if idx < len(self.locations):
            return self.locations[idx]
        for loc in self.emergency_locations:
            if loc.id == idx:
                return loc
        return None

    def _move_drone(self, drone: DroneState, dt_seconds: float):
        """Advance a drone along its current leg or manage dwell time at a stop."""
        if drone.status == "delivering":
            drone.dwell_seconds_left -= dt_seconds
            if drone.dwell_seconds_left <= 0:
                was_intermediate_depot = (drone.current_to == 0 and (drone.route_step + 1) < len(drone.route) - 1)
                drone.route_step += 1
                if drone.route_step >= len(drone.route) - 1:
                    drone.status = "idle"
                    drone.available_tick = self.tick + int(self.drone_config.turnaround_min * 60)
                    drone.active_path = []
                else:
                    drone.status = "flying"
                    # If leaving depot from an intermediate stop, reset sortie distance/time and reload cargo
                    if was_intermediate_depot:
                        drone.sortie_distance_km = 0.0
                        drone.sortie_launch_tick = self.tick
                        # Reload cargo for upcoming stops until next depot return
                        next_stops = []
                        for s in drone.route[drone.route_step + 1:]:
                            if s == 0:
                                break
                            next_stops.append(s)
                        drone.remaining_cargo_kg = sum(
                            (self._get_location(n).demand_kg if self._get_location(n) else 0)
                            for n in next_stops
                        )
                        self._log(f"🚁 Drone {drone.drone_id} reloaded at depot ({drone.remaining_cargo_kg:.1f} kg) and launched for next sortie segment")
                    if not self._init_leg_path(drone):
                        self._replan_active_drone(drone)
            return

        if drone.status != "flying" or drone.route_step >= len(drone.route) - 1:
            return

        if not drone.active_path:
            if not self._init_leg_path(drone):
                self._replan_active_drone(drone)
                if not drone.active_path or drone.status != "flying":
                    return

        total_wp_dist = drone.active_path_length_km
        if total_wp_dist < 1e-9:
            self._arrive_at_stop(drone)
            return

        # Move forward
        speed_km_per_sec = self.drone_config.cruise_speed_kmh / 3600.0
        move_km = speed_km_per_sec * dt_seconds
        drone.leg_progress_km += move_km
        drone.total_distance_km += move_km
        drone.sortie_distance_km += move_km

        # Interpolate exact GPS coordinates along active_path
        frac = min(1.0, drone.leg_progress_km / total_wp_dist)
        waypoints = drone.active_path

        if len(waypoints) >= 2:
            seg_dists = [haversine_km(waypoints[i], waypoints[i + 1]) for i in range(len(waypoints) - 1)]
            target_dist = frac * total_wp_dist
            cumulative = 0.0
            for i, seg_d in enumerate(seg_dists):
                if cumulative + seg_d >= target_dist or i == len(seg_dists) - 1:
                    seg_frac = (target_dist - cumulative) / seg_d if seg_d > 1e-9 else 1.0
                    seg_frac = max(0.0, min(1.0, seg_frac))
                    p1 = waypoints[i]
                    p2 = waypoints[i + 1]
                    drone.lat = p1[0] + (p2[0] - p1[0]) * seg_frac
                    drone.lon = p1[1] + (p2[1] - p1[1]) * seg_frac
                    break
                cumulative += seg_d

        if drone.leg_progress_km >= total_wp_dist:
            self._arrive_at_stop(drone)

    def _arrive_at_stop(self, drone: DroneState):
        """Handle a drone arriving at a stop."""
        arrived_idx = drone.current_to
        arrived_loc = self._get_location(arrived_idx)
        name = arrived_loc.name.split(",")[0] if arrived_loc else f"Location {arrived_idx}"

        if arrived_idx == 0:
            if drone.route_step + 1 >= len(drone.route) - 1:
                # Returned to depot at mission end
                drone.status = "idle"
                drone.available_tick = self.tick + int(self.drone_config.turnaround_min * 60)
                drone.active_path = []
                self._log(f"🏠 Drone {drone.drone_id} returned to depot "
                          f"({drone.sortie_distance_km:.1f} km flown this sortie, "
                          f"{len(drone.deliveries_completed)} deliveries)")
            else:
                # Intermediate depot stop (turnaround + reload)
                drone.status = "delivering"
                drone.dwell_seconds_left = self.drone_config.turnaround_min * 60.0
                drone.active_path = []
                self._log(f"🏠 Drone {drone.drone_id} arrived at depot for turnaround & reload "
                          f"({self.drone_config.turnaround_min:.0f} min)")
        else:
            # Delivery
            if arrived_loc:
                drone.remaining_cargo_kg -= arrived_loc.demand_kg
            drone.deliveries_completed.append(arrived_idx)
            self.delivered_ids.add(arrived_idx)

            # Check cold-chain for this delivery (per-sortie elapsed flight time)
            elapsed_sortie_min = (self.tick - drone.sortie_launch_tick) / 60.0
            cc_status = ""
            if arrived_loc and arrived_loc.cold_chain_limit_minutes is not None:
                if elapsed_sortie_min > arrived_loc.cold_chain_limit_minutes:
                    cc_status = " ❄️ COLD-CHAIN VIOLATED"
                    self.cold_chain_violations += 1
                else:
                    pct = elapsed_sortie_min / arrived_loc.cold_chain_limit_minutes * 100
                    cc_status = f" ❄️ cold-chain {pct:.0f}%"

            # Check time window against request creation time (queue delay counts toward deadline)
            tw_status = ""
            if arrived_loc and arrived_loc.window_minutes is not None:
                req_time = getattr(arrived_loc, 'request_time_min', 0.0) or 0.0
                deadline_min = req_time + arrived_loc.window_minutes
                elapsed_mission_min = self.tick / 60.0
                if elapsed_mission_min > deadline_min + 1e-9:
                    tw_status = " ⏰ WINDOW MISSED"
                    self.time_window_misses += 1
                else:
                    tw_status = f" ⏱️ {elapsed_mission_min:.0f}/{deadline_min:.0f}min"

            self._log(f"📦 Drone {drone.drone_id} delivered to {name}"
                      f"{cc_status}{tw_status}")
            if arrived_loc:
                self.delivery_history.append({
                    "tick": self.tick, "drone_id": drone.drone_id,
                    "name": name, "lat": arrived_loc.lat, "lon": arrived_loc.lon,
                })

            # Enter delivering state to spend dwell_min at this stop
            drone.status = "delivering"
            drone.dwell_seconds_left = self.drone_config.dwell_min * 60.0
            drone.active_path = []

    def _handle_event(self, event: SimEvent):
        """Process a disruption event and trigger replanning."""
        event.handled = True
        self.events_handled += 1

        if event.event_type == "tfr":
            zone = event.data["reason"] if isinstance(event.data.get("reason"), list) else event.data["zone"]
            reason = event.data.get("reason", "Pop-up TFR injected")
            self._log(f"⚡ EVENT: {reason}")
            self.no_fly_zones.append(zone)
            self.nfz_history.append({"tick": self.tick, "zone": zone, "reason": reason})
            self.event_history.append({"tick": self.tick, "type": "tfr", "reason": reason})

            # Rebuild distance matrix for future legs
            all_locs = list(self.locations) + list(self.emergency_locations)
            self.matrix, self.detours = build_flight_distance_matrix(
                all_locs, self.no_fly_zones, drone_config=self.drone_config
            )

            # Re-evaluate all active flying/delivering drones from their EXACT current GPS positions forward
            for drone in self.drones:
                if drone.status in ("flying", "delivering"):
                    self._replan_active_drone(drone)

            self._trigger_replan("TFR injected")

        elif event.event_type == "emergency":
            loc = event.data["location"]
            req_time = getattr(loc, 'request_time_min', 0.0)
            if req_time <= 0:
                req_time = self.tick / 60.0
            # Assign a unique ID and store creation request time
            loc = Location(
                id=len(self.locations) + len(self.emergency_locations),
                name=loc.name, lat=loc.lat, lon=loc.lon,
                demand_kg=loc.demand_kg, urgency=loc.urgency,
                window_minutes=loc.window_minutes,
                cold_chain_limit_minutes=loc.cold_chain_limit_minutes,
                request_time_min=req_time,
            )
            self.emergency_locations.append(loc)
            self.all_delivery_ids.add(loc.id)
            deadline_str = f", deadline: T+{req_time + loc.window_minutes:.0f}m" if loc.window_minutes else ""
            self._log(f"🚨 EVENT: Emergency delivery at ({loc.lat:.4f}, {loc.lon:.4f}) — "
                      f"{loc.demand_kg:.1f}kg {loc.urgency}, {loc.window_minutes}min window{deadline_str}")
            self.event_history.append({"tick": self.tick, "type": "emergency", "lat": loc.lat, "lon": loc.lon, "name": loc.name})

            # Rebuild distance matrix immediately to include the emergency location
            all_locs = list(self.locations) + list(self.emergency_locations)
            self.matrix, self.detours = build_flight_distance_matrix(
                all_locs, self.no_fly_zones, drone_config=self.drone_config
            )

            # Check if an available idle drone at depot or a drone routing through depot can service it
            assignment = try_insert_emergency_into_active_drones(
                self.drones, loc, all_locs, self.no_fly_zones, self.drone_config, self.tick, cargo_onboard=False
            )
            if assignment:
                chosen_drone, new_route = assignment
                self.replans += 1
                if chosen_drone.status == "idle":
                    self.unassigned_routes.append(new_route)
                    self._assign_routes()
                else:
                    chosen_drone.route = new_route
                self._log(f"  ⚡ Assigned emergency directly to Drone {chosen_drone.drone_id}")
            else:
                # Emergency supplies originate physically at the depot; trigger depot-based replan
                self._trigger_replan("Emergency delivery added")

        elif event.event_type == "drone_failure":
            # Find an active drone to fail
            active = [d for d in self.drones if d.status in ("flying", "delivering")]
            if active:
                victim = self.rng.choice(active)
                victim.status = "failed"
                self.drone_failures += 1
                self._log(f"💥 EVENT: Drone {victim.drone_id} FAILED — "
                          f"{len(victim.remaining_stops)} undelivered stops to redistribute")
                self._trigger_replan(f"Drone {victim.drone_id} failure")
            else:
                self._log(f"💥 EVENT: Drone failure triggered but no active drones to fail")

    def _trigger_replan(self, reason: str):
        """Re-solve the CVRP for unassigned deliveries (emergencies or failed drones)."""
        # Find all assigned deliveries (on active drones)
        assigned = set()
        for d in self.drones:
            if d.status in ("flying", "queued", "delivering"):
                assigned.update(d.remaining_stops)
        
        # Collect all unassigned deliveries
        remaining = []
        for loc_id in self.all_delivery_ids:
            if loc_id not in self.delivered_ids and loc_id not in assigned:
                loc = self._get_location(loc_id)
                if loc:
                    remaining.append(loc)

        if not remaining:
            self._log(f"  🔄 Replan skipped — no unassigned deliveries")
            # We still need to update detours for active drones in case of TFR
            all_locs = list(self.locations) + list(self.emergency_locations)
            self.matrix, self.detours = build_flight_distance_matrix(
                all_locs, self.no_fly_zones, drone_config=self.drone_config
            )
            return

        self._log(f"  🔄 REPLANNING ({reason}): {len(remaining)} unassigned stops...")

        result = replan_from_remaining(
            depot=self.locations[0],
            remaining_deliveries=remaining,
            no_fly_zones=self.no_fly_zones,
            drone=self.drone_config,
            current_time_min=self.tick / 60.0,
        )

        self.replans += 1
        self.total_replan_ms += result.replan_time_ms

        self._log(f"  ✅ Replanned in {result.replan_time_ms:.0f}ms — "
                  f"{result.num_routes} new routes, {result.total_km:.1f} km")

        # Map the remapped routes back to real location IDs
        real_id_map = {0: 0}  # depot stays 0
        for i, loc in enumerate(remaining):
            real_id_map[i + 1] = loc.id

        # Replace the queue with newly planned routes for all remaining unassigned stops
        self.unassigned_routes = []
        for i, new_route in enumerate(result.new_routes):
            real_route = [real_id_map.get(idx, idx) for idx in new_route]
            self.unassigned_routes.append(real_route)
            
        self._assign_routes()

        # Rebuild the distance matrix for current no-fly zones
        # so future _get_leg_distance calls and detour waypoints are accurate.
        # Include emergency locations in the rebuild.
        all_locs = list(self.locations) + list(self.emergency_locations)
        self.matrix, self.detours = build_flight_distance_matrix(
            all_locs, self.no_fly_zones, drone_config=self.drone_config
        )

    def _all_done(self) -> bool:
        """Check if all deliveries are complete or marked unserviceable."""
        return (self.delivered_ids | self.unserviceable_ids) >= self.all_delivery_ids

    def run(self, max_ticks: int = 3600) -> MissionReport:
        """Run the simulation until all deliveries complete or max_ticks."""
        print(f"\n{'='*70}")
        print(f"  FLEET SIMULATION: {self.city.name}")
        print(f"{'='*70}")
        if self.city.scenario_description:
            print(f"\n  {self.city.scenario_description}\n")
        print(f"  {len(self.drones)} drones dispatched, "
              f"{len(self.all_delivery_ids)} delivery locations, "
              f"{len(self.events)} random events queued")
        print(f"  Original plan: {self.original_distance:.1f} km total")
        print(f"{'='*70}\n")

        # Log initial launches
        for drone in self.drones:
            if drone.status == "flying":
                route_names = " → ".join(
                    (self._get_location(n).name.split(",")[0]
                     if self._get_location(n) else f"#{n}")
                    for n in drone.route
                )
                self._log(f"🚁 Drone {drone.drone_id} launched: {route_names}")

        while self.tick < max_ticks:
            # Check for events at this tick
            for event in self.events:
                if not event.handled and event.tick <= self.tick:
                    self._handle_event(event)

            self._assign_routes()

            # Move all active drones
            for drone in self.drones:
                if drone.status in ("flying", "delivering"):
                    self._move_drone(drone, 1.0)

            # --- Record positions for visual replay ---
            if self.tick % self.record_interval == 0:
                for drone in self.drones:
                    if drone.status in ("flying", "delivering"):
                        self.position_history.append({
                            "tick": self.tick,
                            "drone_id": drone.drone_id,
                            "lat": drone.lat,
                            "lon": drone.lon,
                            "status": drone.status,
                        })

            # Check if done
            if self._all_done():
                # Wait for all routes to be flown and drones to return
                if not self.unassigned_routes:
                    all_returned = all(
                        d.status in ("idle", "failed")
                        for d in self.drones
                    )
                    if all_returned:
                        break

            self.tick += 1

            # Real-time pacing (if not running at max speed)
            if self.speed_multiplier < 100:
                time.sleep(1.0 / self.speed_multiplier)

        # Final report
        total_flight = sum(d.total_distance_km for d in self.drones)
        increase_pct = (
            100 * (total_flight - self.original_distance) / self.original_distance
            if self.original_distance > 0 else 0
        )

        report = MissionReport(
            total_ticks=self.tick,
            total_seconds=self.tick,
            deliveries_planned=len(self.all_delivery_ids),
            deliveries_completed=len(self.delivered_ids),
            events_injected=len(self.events),
            events_handled=self.events_handled,
            replans_performed=self.replans,
            total_replan_time_ms=self.total_replan_ms,
            avg_replan_time_ms=(self.total_replan_ms / self.replans
                                if self.replans > 0 else 0),
            cold_chain_violations=self.cold_chain_violations,
            time_window_misses=self.time_window_misses,
            total_flight_distance_km=total_flight,
            original_plan_distance_km=self.original_distance,
            distance_increase_pct=increase_pct,
            drone_failures=self.drone_failures,
            unserviceable_deliveries=len(self.unserviceable_ids),
        )

        self._print_report(report)
        return report

    def _print_report(self, report: MissionReport):
        """Print the final mission summary."""
        print(f"\n{'='*70}")
        print(f"  MISSION REPORT")
        print(f"{'='*70}")
        print(f"  Mission time:           {report.total_seconds // 60:.0f}m {report.total_seconds % 60:.0f}s")
        print(f"  Deliveries:             {report.deliveries_completed}/{report.deliveries_planned}"
              f" ({'+ emergencies' if self.emergency_locations else 'all planned'})")
        if report.unserviceable_deliveries > 0:
            print(f"  Unserviceable / missed: {report.unserviceable_deliveries}")
        print(f"  Events handled:         {report.events_handled} "
              f"({len([e for e in self.events if e.event_type == 'tfr'])} TFR, "
              f"{len([e for e in self.events if e.event_type == 'emergency'])} emergency, "
              f"{report.drone_failures} drone failure)")
        print(f"  Replans performed:      {report.replans_performed}")
        if report.replans_performed > 0:
            print(f"  Avg replan time:        {report.avg_replan_time_ms:.0f}ms")
            print(f"  Total replan time:      {report.total_replan_time_ms:.0f}ms")
        print(f"  Cold-chain violations:  {report.cold_chain_violations}")
        print(f"  Time-window misses:     {report.time_window_misses}")
        print(f"  Total flight distance:  {report.total_flight_distance_km:.1f} km")
        print(f"  Original plan distance: {report.original_plan_distance_km:.1f} km")
        print(f"  Distance increase:      {'+' if report.distance_increase_pct >= 0 else ''}"
              f"{report.distance_increase_pct:.1f}% (due to disruptions)")
        print(f"{'='*70}\n")


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Real-time fleet simulation with dynamic event injection"
    )
    parser.add_argument("--city", default="jaipur_disaster",
                        help=f"Scenario ({', '.join(CITY_CONFIGS.keys())})")
    parser.add_argument("--from-json", dest="json_file", default=None,
                        help="Load scenario from a JSON file (any location on Earth)")
    parser.add_argument("--random-at", nargs=2, type=float, default=None,
                        metavar=("LAT", "LON"),
                        help="Generate and simulate a random scenario at these GPS coords")
    parser.add_argument("--events", type=int, default=3,
                        help="Number of random disruption events (default: 3)")
    parser.add_argument("--seed", type=int, default=None,
                        help="Random seed for reproducibility")
    parser.add_argument("--speed", type=float, default=100.0,
                        help="Simulation speed multiplier (default: 100 = fast)")
    parser.add_argument("--max-ticks", type=int, default=3600,
                        help="Maximum simulation ticks/seconds (default: 3600)")
    parser.add_argument("--no-random", action="store_true",
                        help="Disable random events (use --inject-* flags instead)")
    parser.add_argument("--inject-tfr", type=int, nargs="*", default=[],
                        help="Inject a TFR at these tick numbers")
    parser.add_argument("--inject-emergency", type=int, nargs="*", default=[],
                        help="Inject an emergency at these tick numbers")
    parser.add_argument("--visualize", action="store_true",
                        help="Generate an animated HTML map replay after simulation")
    args = parser.parse_args()

    # Look up city
    if args.json_file:
        from load_scenario import load_scenario_from_json
        city = load_scenario_from_json(args.json_file)
    elif args.random_at:
        from load_scenario import generate_random_scenario
        city = generate_random_scenario(args.random_at[0], args.random_at[1],
                                         name=f"Random Scenario ({args.random_at[0]}, {args.random_at[1]})")
    else:
        city = CITY_CONFIGS.get(args.city)
    if city is None:
        print(f"Unknown city '{args.city}'. Available: {', '.join(CITY_CONFIGS.keys())}")
        sys.exit(1)

    rng = random.Random(args.seed)

    # Build events
    events = []
    if not args.no_random and args.events > 0:
        events = generate_random_events(city, args.events, args.max_ticks, rng)

    # Manual injections
    for tick in args.inject_tfr:
        target = rng.choice(city.deliveries)
        size = 0.01
        zone = [
            (target.lat - size, target.lon - size),
            (target.lat - size, target.lon + size),
            (target.lat + size, target.lon + size),
            (target.lat + size, target.lon - size),
        ]
        events.append(SimEvent(
            tick=tick, event_type="tfr",
            data={"zone": zone, "reason": f"Manual TFR near {target.name.split(',')[0]}"},
        ))

    for tick in args.inject_emergency:
        events.append(SimEvent(
            tick=tick, event_type="emergency",
            data={"location": Location(
                id=999, name=f"MANUAL EMERGENCY at T+{tick}s",
                lat=city.depot.lat + rng.uniform(-0.02, 0.02),
                lon=city.depot.lon + rng.uniform(-0.02, 0.02),
                demand_kg=3.0, urgency="critical", window_minutes=15,
            )},
        ))

    events.sort(key=lambda e: e.tick)

    # Run
    sim = FleetSimulator(city, events=events, speed_multiplier=args.speed, seed=args.seed)
    report = sim.run(max_ticks=args.max_ticks)

    # Generate visual replay map
    if args.visualize:
        # Generate the interactive simulation viewer (primary)
        from build_interactive_sim import build_interactive_html
        sim_path = build_interactive_html(sim, "output/interactive_sim.html")
        print(f"\n🗺️  Interactive simulation saved to: {sim_path}")
        print(f"    Open it in your browser to WATCH the drones fly!")

        # Also generate the folium replay (backup/simpler view)
        from visualize_simulation import render_simulation_map
        map_path = render_simulation_map(sim, "output/simulation_replay.html")
        if map_path:
            print(f"    Static replay also saved to: {map_path}")

        # Auto-open the interactive sim
        print()
        try:
            import webbrowser
            webbrowser.open(f"file://{os.path.abspath(sim_path)}")
        except Exception:
            pass

    # Exit code: 0 if all deliveries resolved (completed or unserviceable), 1 if not
    sys.exit(0 if (report.deliveries_completed + report.unserviceable_deliveries) >= report.deliveries_planned else 1)


if __name__ == "__main__":
    main()
