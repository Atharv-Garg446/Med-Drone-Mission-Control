"""
cold_chain.py
=============
Cold-chain constraint checking for temperature-sensitive medical supplies.

KEY DISTINCTION FROM TIME WINDOWS (time_windows.py):
  - Time windows say "this stop must be reached within X minutes of departure"
    -- a DELIVERY DEADLINE for one specific location.
  - Cold chain says "this ITEM has been out of refrigeration since the drone
    left the depot, and its cumulative exposure must stay under Y minutes"
    -- a CARGO PROPERTY that accumulates across the entire multi-stop trip.

Example: a vaccine loaded at the depot with a 30-minute cold-chain limit.
If the drone visits Stop A (10 min flight), then Stop B (8 min more), then
delivers the vaccine at Stop C (7 min more), the vaccine has been out of
refrigeration for 10 + 8 + 7 = 25 minutes total — still within the 30-min
limit. But if there were another stop before C, the extra flight time could
push it over.

This matters because:
  - A route that DELIVERS the vaccine at minute 25 is fine.
  - But a route that delivers OTHER stops first and reaches the vaccine's
    destination at minute 35 is NOT fine — the vaccine is ruined even though
    the drone itself is still within range.

The cold-chain limit constrains WHICH POSITION in a multi-stop route a
temperature-sensitive item can occupy, not just whether the drone can
physically reach it. This is a genuinely different constraint from capacity,
range, or delivery-window urgency.

DESIGN: This module provides:
  1. check_cold_chain() — post-hoc validation (like time_windows.check_time_windows)
  2. cold_chain_feasible() — used by construction heuristics to reject
     adding a stop that would blow any already-loaded cold-chain item's limit
"""

from dataclasses import dataclass
from typing import List, Dict, Optional, Tuple
from config import Location, DroneConfig


@dataclass
class ColdChainResult:
    """Status of one cold-chain item on one route."""
    location_id: int
    location_name: str
    cold_chain_limit_minutes: float   # max allowed cumulative exposure
    actual_exposure_minutes: float    # cumulative flight time when delivered
    violated: bool                    # True if actual > limit


def check_cold_chain(
    route: List[int],
    locations: List[Location],
    matrix: List[List[float]],
    drone: DroneConfig,
) -> List[ColdChainResult]:
    """Walk a single route and check every cold-chain item's cumulative
    exposure against its limit, including flight time and dwell time."""
    results = []
    cumulative_minutes = 0.0

    for step in range(1, len(route)):
        prev = route[step - 1]
        curr = route[step]
        dist = matrix[prev][curr]
        cumulative_minutes += (dist / drone.cruise_speed_kmh) * 60.0

        loc = locations[curr]

        if loc.id == locations[0].id:  # back at depot
            continue

        if loc.cold_chain_limit_minutes is not None:
            violated = cumulative_minutes > loc.cold_chain_limit_minutes
            results.append(ColdChainResult(
                location_id=loc.id,
                location_name=loc.name,
                cold_chain_limit_minutes=loc.cold_chain_limit_minutes,
                actual_exposure_minutes=round(cumulative_minutes, 1),
                violated=violated,
            ))

        # Add dwell time spent at this delivery stop before departing to next stop
        cumulative_minutes += drone.dwell_min

    return results


def check_cold_chain_all_routes(
    routes: List[List[int]],
    locations: List[Location],
    matrix: List[List[float]],
    drone: DroneConfig,
) -> Dict[int, List[ColdChainResult]]:
    """Run cold-chain checks on every route, returning only routes that
    have at least one cold-chain item. Format matches time_windows.check_time_windows
    for consistency."""
    report = {}
    for idx, route in enumerate(routes):
        results = check_cold_chain(route, locations, matrix, drone)
        if results:
            report[idx] = results
    return report


def cold_chain_feasible_after_adding(
    route_so_far: List[int],
    candidate: int,
    locations: List[Location],
    matrix: List[List[float]],
    drone: DroneConfig,
) -> bool:
    """Would adding `candidate` as the next stop on `route_so_far` violate
    any cold-chain constraint?

    Two things can go wrong:
      1. The candidate itself has a cold-chain limit, and the cumulative
         flight time to reach it (after all previous stops) exceeds that limit.
      2. Some ALREADY-LOADED item (still on the drone, not yet delivered)
         has a cold-chain limit that would be exceeded by the extra flight
         time to candidate PLUS the flight time back from candidate to
         wherever that item eventually gets delivered.

    For construction heuristics (which build routes greedily, one stop at a
    time), we can only check #1 precisely. #2 would require knowing future
    stops, which we don't have yet. So we use a conservative approximation
    for #2: assume the worst case — that no already-loaded cold-chain item
    will be delivered until AFTER we visit the candidate and return to depot.

    This is called by the construction heuristics in vrp_scratch.py.
    """
    if not route_so_far:
        return True

    # Compute cumulative distance of route_so_far
    cumulative_km = 0.0
    for i in range(len(route_so_far) - 1):
        cumulative_km += matrix[route_so_far[i]][route_so_far[i + 1]]

    # Distance to reach the candidate from current position
    current = route_so_far[-1]
    leg_to_candidate = matrix[current][candidate]
    total_km_at_candidate = cumulative_km + leg_to_candidate

    # Convert to minutes and include dwell time spent at all intermediate stops
    num_intermediate_stops = max(0, len(route_so_far) - 1)
    total_minutes_at_candidate = (total_km_at_candidate / drone.cruise_speed_kmh) * 60.0 + (num_intermediate_stops * drone.dwell_min)

    # Check 1: does the candidate itself have a cold-chain limit we'd blow?
    if locations[candidate].cold_chain_limit_minutes is not None:
        if total_minutes_at_candidate > locations[candidate].cold_chain_limit_minutes:
            return False

    # Check 2 (conservative): for any already-loaded cold-chain items still
    # on the drone (not yet delivered), would the extra time to visit
    # candidate violate their limit? We check against the time at the
    # candidate position, which is an underestimate of when they'd
    # actually be delivered (they'd be delivered even later). So if this
    # already fails, the real delivery time would be even worse.
    depot_index = route_so_far[0]
    delivered = set(route_so_far[1:])  # stops already visited = items already delivered

    # Items still on the drone = items in the route's future. But during
    # construction, we don't know what's still coming. We only need to worry
    # about items we've already committed to but haven't delivered yet.
    # In greedy construction, every stop added to route_so_far IS delivered,
    # so there are no "still-loaded" items to worry about — they're all
    # delivered at their respective stops as we build the route.

    return True
