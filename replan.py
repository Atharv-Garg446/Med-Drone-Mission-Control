"""
replan.py
=========
Mid-flight replanning: given the current state of a fleet (some deliveries
already completed, some drones mid-air), rebuild the distance matrix for
the REMAINING undelivered stops and re-solve the CVRP from scratch.

This is the bridge between "static pre-flight planner" and "reactive
real-time system."  The key insight is that our from-scratch solver is
fast enough (<100ms for 10-stop problems) that we CAN re-solve the entire
remaining problem on every disruption, rather than needing incremental
patching.

What triggers a replan:
  - A pop-up TFR (Temporary Flight Restriction) appears mid-mission
  - An emergency delivery request arrives
  - A drone goes down and its remaining stops must be redistributed
  - A delivery is refused/cancelled at a stop

The replan does NOT move drones.  It produces new route assignments that
the simulator (simulate_fleet.py) then executes.
"""

import time
from typing import List, Optional, Tuple, Dict
from dataclasses import dataclass

from config import Location, DroneConfig, CityConfig
from distance_matrix import build_flight_distance_matrix
from vrp_scratch import solve_vrp_from_scratch, total_distance
from cold_chain import check_cold_chain_all_routes
from time_windows import check_time_windows


@dataclass
class ReplanResult:
    """Result of a mid-flight replanning operation."""
    new_routes: List[List[int]]
    total_km: float
    num_routes: int
    replan_time_ms: float
    remaining_locations: List[Location]
    remaining_matrix: List[List[float]]
    detours: Dict[Tuple[int, int], list]
    cold_chain_ok: bool
    windows_missed: int


def replan_from_remaining(
    depot: Location,
    remaining_deliveries: List[Location],
    no_fly_zones: list,
    drone: DroneConfig,
    current_time_min: float = 0.0,
) -> ReplanResult:
    """Re-solve the CVRP for just the remaining undelivered stops.

    Parameters
    ----------
    depot : Location
        The depot location (drones return here after each trip).
    remaining_deliveries : list[Location]
        Only the stops that have NOT yet been delivered to.  Their .id
        fields will be remapped to [1..N] internally so the solver sees
        a clean 0-indexed problem.
    no_fly_zones : list
        Current no-fly zones (including any newly-injected TFRs).
    drone : DroneConfig
        Physical drone limits.
    current_time_min : float
        Actual mission scheduling time for deadline and cold-chain evaluation.

    Returns
    -------
    ReplanResult with new routes (indices into the remapped location list,
    where 0 = depot), solve time, and constraint-check results.
    """
    # Remap IDs so the solver sees [0=depot, 1..N]
    remapped_depot = Location(
        id=0, name=depot.name, lat=depot.lat, lon=depot.lon,
        demand_kg=0.0,
    )
    remapped = [remapped_depot]
    for i, loc in enumerate(remaining_deliveries):
        remapped.append(Location(
            id=i + 1,
            name=loc.name,
            lat=loc.lat,
            lon=loc.lon,
            demand_kg=loc.demand_kg,
            urgency=loc.urgency,
            window_minutes=loc.window_minutes,
            cold_chain_limit_minutes=loc.cold_chain_limit_minutes,
            landing_zone_image=loc.landing_zone_image,
            request_time_min=loc.request_time_min or 0.0,
        ))

    t0 = time.perf_counter()
    matrix, detours = build_flight_distance_matrix(remapped, no_fly_zones, drone_config=drone)
    routes = solve_vrp_from_scratch(remapped, matrix, drone, depart_minutes=current_time_min)
    replan_ms = (time.perf_counter() - t0) * 1000

    total_km = total_distance(routes, matrix)

    # Check constraints on the new plan
    cc_report = check_cold_chain_all_routes(routes, remapped, matrix, drone)
    cc_ok = True
    if cc_report:
        for results in cc_report.values():
            for r in results:
                if r.violated:
                    cc_ok = False

    tw_report = check_time_windows(routes, remapped, matrix, drone, departure_times=[current_time_min] * len(routes))
    missed = 0
    if tw_report:
        for etas in tw_report.values():
            for eta in etas:
                if eta.missed_window:
                    missed += 1

    return ReplanResult(
        new_routes=routes,
        total_km=total_km,
        num_routes=len(routes),
        replan_time_ms=replan_ms,
        remaining_locations=remapped,
        remaining_matrix=matrix,
        detours=detours,
        cold_chain_ok=cc_ok,
        windows_missed=missed,
    )


def try_insert_emergency_into_active_drones(
    drones,
    emergency_loc: Location,
    all_locations: List[Location],
    no_fly_zones: list,
    drone_config: DroneConfig,
    current_tick: int,
    cargo_onboard: bool = False,
) -> Optional[Tuple[object, List[int]]]:
    """Test assigning an emergency delivery to an active or idle drone under physical cargo constraints.

    PHYSICAL CARGO MODEL:
    Newly requested supplies physically originate at the depot (index 0).
    - If cargo_onboard is True (e.g. interchangeable emergency kit pre-loaded):
      The airborne drone may deliver directly mid-sortie if within capacity/range/cold-chain.
    - If cargo_onboard is False (default):
      The drone MUST load the cargo at the depot first. For an airborne drone, the emergency
      is served as a subsequent sortie after returning to the depot (p_curr -> rem_stops -> depot -> emerg -> depot).
      For an idle drone at depot, it launches as [0, emerg.id, 0].
    
    Returns (selected_drone, new_full_route) or None if no feasible assignment.
    """
    import math
    from nofly_astar import route_avoiding_zones

    def get_loc(idx: int) -> Optional[Location]:
        for loc in all_locations:
            if loc.id == idx:
                return loc
        return None

    buffer_km = drone_config.safety_margin_m / 1000.0
    best_cost_increase = float("inf")
    best_candidate = None
    max_allowed_dist = drone_config.max_range_km * (1.0 - drone_config.reserve_fraction)
    depot_loc = all_locations[0]

    # 1. First consider idle drones already at the depot (fastest dispatch for newly requested depot cargo)
    for drone in drones:
        if drone.status == "idle" and current_tick >= drone.available_tick:
            pts, leg_km, _ = route_avoiding_zones(
                (depot_loc.lat, depot_loc.lon), (emergency_loc.lat, emergency_loc.lon), no_fly_zones, buffer_km=buffer_km
            )
            pts_ret, ret_km, _ = route_avoiding_zones(
                (emergency_loc.lat, emergency_loc.lon), (depot_loc.lat, depot_loc.lon), no_fly_zones, buffer_km=buffer_km
            )
            if pts and pts_ret and not math.isinf(leg_km) and not math.isinf(ret_km):
                tot_km = leg_km + ret_km
                if tot_km <= max_allowed_dist + 1e-9 and emergency_loc.demand_kg <= drone_config.capacity_kg + 1e-9:
                    leg_time_min = (leg_km / drone_config.cruise_speed_kmh) * 60.0
                    deadline_min = (emergency_loc.request_time_min or (current_tick / 60.0)) + (emergency_loc.window_minutes or float("inf"))
                    if (current_tick / 60.0) + leg_time_min <= deadline_min + 1e-9:
                        if tot_km < best_cost_increase:
                            best_cost_increase = tot_km
                            best_candidate = (drone, [0, emergency_loc.id, 0])

    if best_candidate:
        return best_candidate

    # 2. Consider active airborne drones
    for drone in drones:
        if drone.status != "flying" or drone.route_step >= len(drone.route) - 1:
            continue

        p_curr = (drone.lat, drone.lon)
        target_idx = drone.route[drone.route_step + 1]
        future_stops = [idx for idx in drone.route[drone.route_step + 2 : -1]]
        rem_stops = ([target_idx] if target_idx != 0 else []) + future_stops

        if cargo_onboard:
            # Emergency kit already onboard: test every insertion point in remaining stops of this sortie
            if drone.remaining_cargo_kg + emergency_loc.demand_kg > drone_config.capacity_kg + 1e-9:
                continue

            for k in range(len(rem_stops) + 1):
                cand_seq = rem_stops[:k] + [emergency_loc.id] + rem_stops[k:]
                curr_pos = p_curr
                cand_dist = 0.0
                elapsed_sortie_min = (current_tick - drone.sortie_launch_tick) / 60.0
                current_mission_min = current_tick / 60.0
                feasible = True

                for stop_id in cand_seq:
                    stop_loc = get_loc(stop_id)
                    if not stop_loc:
                        feasible = False
                        break
                    pts, leg_km, _ = route_avoiding_zones(curr_pos, (stop_loc.lat, stop_loc.lon), no_fly_zones, buffer_km=buffer_km)
                    if pts is None or math.isinf(leg_km):
                        feasible = False
                        break
                    cand_dist += leg_km
                    leg_time = (leg_km / drone_config.cruise_speed_kmh) * 60.0
                    elapsed_sortie_min += leg_time
                    current_mission_min += leg_time

                    if stop_loc.cold_chain_limit_minutes is not None and elapsed_sortie_min > stop_loc.cold_chain_limit_minutes + 1e-9:
                        feasible = False
                        break
                    dl = (stop_loc.request_time_min or 0.0) + (stop_loc.window_minutes or float("inf"))
                    if stop_loc.window_minutes is not None and current_mission_min > dl + 1e-9:
                        feasible = False
                        break

                    elapsed_sortie_min += drone_config.dwell_min
                    current_mission_min += drone_config.dwell_min
                    curr_pos = (stop_loc.lat, stop_loc.lon)

                if not feasible:
                    continue

                # Return leg to depot
                pts, ret_km, _ = route_avoiding_zones(curr_pos, (depot_loc.lat, depot_loc.lon), no_fly_zones, buffer_km=buffer_km)
                if pts is None or math.isinf(ret_km):
                    continue
                cand_dist += ret_km

                if drone.sortie_distance_km + cand_dist > max_allowed_dist + 1e-9:
                    continue

                if cand_dist < best_cost_increase:
                    best_cost_increase = cand_dist
                    full_new_route = drone.route[:drone.route_step + 1] + cand_seq + [0]
                    best_candidate = (drone, full_new_route)

        else:
            # Supplies are physically at depot:
            # Sortie 1: complete rem_stops and return to depot
            # Depot: turnaround dwell + reload emergency supplies
            # Sortie 2: depart depot -> emergency -> return to depot
            if emergency_loc.demand_kg > drone_config.capacity_kg + 1e-9:
                continue

            # Check Sortie 1 feasibility
            curr_pos = p_curr
            sortie1_dist = 0.0
            elapsed_sortie_min = (current_tick - drone.sortie_launch_tick) / 60.0
            current_mission_min = current_tick / 60.0
            sortie1_feasible = True

            for stop_id in rem_stops:
                stop_loc = get_loc(stop_id)
                if not stop_loc:
                    sortie1_feasible = False
                    break
                pts, leg_km, _ = route_avoiding_zones(curr_pos, (stop_loc.lat, stop_loc.lon), no_fly_zones, buffer_km=buffer_km)
                if pts is None or math.isinf(leg_km):
                    sortie1_feasible = False
                    break
                sortie1_dist += leg_km
                leg_time = (leg_km / drone_config.cruise_speed_kmh) * 60.0
                elapsed_sortie_min += leg_time
                current_mission_min += leg_time

                if stop_loc.cold_chain_limit_minutes is not None and elapsed_sortie_min > stop_loc.cold_chain_limit_minutes + 1e-9:
                    sortie1_feasible = False
                    break
                dl = (stop_loc.request_time_min or 0.0) + (stop_loc.window_minutes or float("inf"))
                if stop_loc.window_minutes is not None and current_mission_min > dl + 1e-9:
                    sortie1_feasible = False
                    break

                elapsed_sortie_min += drone_config.dwell_min
                current_mission_min += drone_config.dwell_min
                curr_pos = (stop_loc.lat, stop_loc.lon)

            if not sortie1_feasible:
                continue

            pts, ret1_km, _ = route_avoiding_zones(curr_pos, (depot_loc.lat, depot_loc.lon), no_fly_zones, buffer_km=buffer_km)
            if pts is None or math.isinf(ret1_km):
                continue
            sortie1_dist += ret1_km
            current_mission_min += (ret1_km / drone_config.cruise_speed_kmh) * 60.0

            if drone.sortie_distance_km + sortie1_dist > max_allowed_dist + 1e-9:
                continue

            # Depot turnaround: reloading cargo and resetting range budget
            t_sortie2_depart = current_mission_min + drone_config.turnaround_min

            # Check Sortie 2 feasibility (depot -> emergency -> depot)
            pts_e, leg_e_km, _ = route_avoiding_zones((depot_loc.lat, depot_loc.lon), (emergency_loc.lat, emergency_loc.lon), no_fly_zones, buffer_km=buffer_km)
            pts_ret2, ret2_km, _ = route_avoiding_zones((emergency_loc.lat, emergency_loc.lon), (depot_loc.lat, depot_loc.lon), no_fly_zones, buffer_km=buffer_km)
            if not pts_e or not pts_ret2 or math.isinf(leg_e_km) or math.isinf(ret2_km):
                continue

            sortie2_dist = leg_e_km + ret2_km
            if sortie2_dist > max_allowed_dist + 1e-9:
                continue

            leg_e_time = (leg_e_km / drone_config.cruise_speed_kmh) * 60.0
            t_emerg_arrival = t_sortie2_depart + leg_e_time

            # Cold chain from depot to emergency
            if emergency_loc.cold_chain_limit_minutes is not None and leg_e_time > emergency_loc.cold_chain_limit_minutes + 1e-9:
                continue

            # Deadline for emergency delivery
            emerg_deadline = (emergency_loc.request_time_min or (current_tick / 60.0)) + (emergency_loc.window_minutes or float("inf"))
            if emergency_loc.window_minutes is not None and t_emerg_arrival > emerg_deadline + 1e-9:
                continue

            # Both sorties feasible! Cost is distance of the emergency sortie
            if sortie2_dist < best_cost_increase:
                best_cost_increase = sortie2_dist
                full_new_route = drone.route[:drone.route_step + 1] + rem_stops + [0, emergency_loc.id, 0]
                best_candidate = (drone, full_new_route)

    return best_candidate
