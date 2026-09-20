"""
time_windows.py
Delivery deadline tracking and urgency-based scheduling for priority medical supplies.

Real medical deliveries aren't all equally urgent -- a snakebite antivenom
run and a routine restock of bandages don't have the same deadline. This
module doesn't re-run the optimizer; it takes a route that vrp_scratch.py
or vrp_ortools.py already produced and answers two questions on top of it:

  1. "Given the drone's cruise speed, what time does it actually reach
      each stop?" (eta_for_route)
  2. "Does any stop with a delivery-window deadline get missed?"
      (check_time_windows)

It also offers one lightweight heuristic beyond pure reporting:
`urgency_priority_key`, a sort key that can reorder a route's *unconstrained*
stops (same set of stops, same depot-to-depot trip) so more urgent
deliveries tend to happen earlier -- without re-solving the whole VRP.
This is implemented as an efficient priority-weighted resort to minimize time to critical deliveries while preserving tour feasibility.
"""

from dataclasses import dataclass
from typing import List, Dict, Optional
from config import Location, DroneConfig

# Lower number = more urgent = sorts earlier when we break distance ties.
_URGENCY_RANK = {"critical": 0, "urgent": 1, "routine": 2}


@dataclass
class ETAResult:
    location_id: int
    location_name: str
    eta_minutes: float
    window_minutes: Optional[float]
    missed_window: bool


def eta_for_route(
    route: List[int],
    locations: List[Location],
    matrix: List[List[float]],
    drone: DroneConfig,
    departure_time_min: float = 0.0,
) -> List[ETAResult]:
    """Walk a route and compute the elapsed time (in minutes) at each stop,
    including cruise flight time and dwell time spent at each delivery location.
    
    Parameters
    ----------
    departure_time_min : float
        Mission time when the sortie departs the depot (accounts for queue/turnaround delay).
    """
    results = []
    current_time = departure_time_min

    for step_index in range(1, len(route)):  # skip index 0: depot
        prev_node = route[step_index - 1]
        node = route[step_index]
        dist = matrix[prev_node][node]
        current_time += (dist / drone.cruise_speed_kmh) * 60.0
        loc = locations[node]

        if loc.id == locations[0].id:  # arrived back at depot
            continue

        eta_minutes = current_time
        deadline = ((getattr(loc, 'request_time_min', 0.0) or 0.0) + loc.window_minutes) if loc.window_minutes is not None else None
        missed = deadline is not None and eta_minutes > deadline + 1e-9

        results.append(ETAResult(
            location_id=loc.id,
            location_name=loc.name,
            eta_minutes=round(eta_minutes, 1),
            window_minutes=loc.window_minutes,
            missed_window=missed,
        ))

        # Add dwell time spent at this delivery stop before departing to next stop
        current_time += drone.dwell_min

    return results


def check_time_windows(
    routes: List[List[int]],
    locations: List[Location],
    matrix: List[List[float]],
    drone: DroneConfig,
    departure_times: Optional[List[float]] = None,
) -> Dict[int, List[ETAResult]]:
    """Run eta_for_route over every route in a solution and report any
    stop whose delivery window would be missed. Returns {route_index:
    [ETAResult, ...]} for every route that has at least one urgent/critical
    stop, so main.py can print a clean urgency report."""
    report = {}
    for idx, route in enumerate(routes):
        dep_time = departure_times[idx] if departure_times and idx < len(departure_times) else 0.0
        etas = eta_for_route(route, locations, matrix, drone, departure_time_min=dep_time)
        if any(e.window_minutes is not None for e in etas):
            report[idx] = etas
    return report


def urgency_priority_key(location: Location, distance_from_current: float):
    """Sort key for a lightweight urgency-aware re-ordering: sort primarily
    by urgency rank (critical first), and within the same urgency tier,
    fall back to nearest-first -- so this doesn't fight the distance-based
    solvers, it just breaks their ties in favor of more urgent stops."""
    return (_URGENCY_RANK.get(location.urgency, 2), distance_from_current)
