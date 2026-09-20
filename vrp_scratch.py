"""
vrp_scratch.py
==============
Custom solver for the Capacitated Vehicle Routing Problem (CVRP) with range constraints.

Implements three construction heuristics and two local search improvement operators.

It solves the Capacitated VRP with a Range constraint using:

  CONSTRUCTION HEURISTICS:

    1. Nearest-Neighbor (nearest_neighbor_construction):
       Greedily add the closest feasible stop to the current trip. Fast,
       simple, usually 15-25% worse than optimal.

    2. Clarke-Wright Savings (clarke_wright_construction):
       Start with one trivial route per customer. Merge routes when the
       distance "saved" by visiting two customers consecutively exceeds
       the cost. Often outperforms nearest-neighbor because it considers
       global structure, not just the nearest point.

    3. Urgency-Weighted Nearest-Neighbor (urgency_nearest_neighbor_construction):
       Like nearest-neighbor, but gives time-critical deliveries a
       distance "discount" so they get served earlier. Produces solutions
       that respect medical urgency at a small distance cost.

  IMPROVEMENT OPERATORS:

    1. 2-opt (two_opt) -- INTRA-ROUTE:
       Reverse segments within a single route to eliminate edge crossings.
       Classic, simple, guaranteed not to make things worse.

    2. Or-opt relocate (or_opt_inter_route) -- INTER-ROUTE:
       Move individual stops from one route to a better position in
       another. Can reduce total distance by consolidating trips and
       escape local minima that intra-route 2-opt cannot.

Neither stage touches distance_matrix.py's zone-avoidance logic directly --
that's the point of building it as a *matrix* upstream. By the time this
file runs, matrix[i][j] already IS the shortest legal (zone-avoiding)
distance between i and j, so a solver that just minimizes total matrix
distance is automatically a solver that avoids no-fly zones. Separating
concerns this way is what keeps this file simple enough to hand-write.
"""

from typing import List, Tuple, Dict, Optional
from config import Location, DroneConfig
from cold_chain import cold_chain_feasible_after_adding


# ---------------------------------------------------------------------------
# Utility functions
# ---------------------------------------------------------------------------

def _route_distance(route: List[int], matrix: List[List[float]]) -> float:
    """Total distance of one route: depot -> stop -> stop -> ... -> depot.
    `route` is a list of location indices that starts and ends at the depot
    (index 0), e.g. [0, 3, 7, 0]."""
    total = 0.0
    for i in range(len(route) - 1):
        total += matrix[route[i]][route[i + 1]]
    return total


def _route_demand(route: List[int], locations: List[Location]) -> float:
    """Total payload demand on a route (excludes depot nodes)."""
    return sum(locations[n].demand_kg for n in route[1:-1])


import math

def is_route_feasible(route: List[int], locations: List[Location], matrix: List[List[float]], drone: DroneConfig, depart_minutes: float = 0.0) -> bool:
    """Check if a route satisfies ALL hard constraints:
    - No `inf` edges (unreachable)
    - Payload <= drone.capacity_kg
    - Distance <= drone.max_range_km * (1.0 - drone.reserve_fraction)
    - Time windows met for all stops
    - Cold chain limits met for all stops
    """
    total_demand = 0.0
    elapsed_km = 0.0
    current_time = depart_minutes
    
    for i in range(1, len(route)):
        prev = route[i-1]
        curr = route[i]
        
        dist = matrix[prev][curr]
        if math.isinf(dist):
            return False  # Unreachable edge
            
        elapsed_km += dist
        # Add flight time for this leg
        current_time += (dist / drone.cruise_speed_kmh) * 60.0
        
        if curr != route[-1]:  # Not the depot return
            loc = locations[curr]
            total_demand += loc.demand_kg
            if total_demand > drone.capacity_kg + 1e-9:
                return False
                
            if loc.window_minutes is not None:
                deadline = (getattr(loc, 'request_time_min', 0.0) or 0.0) + loc.window_minutes
                if current_time > deadline + 1e-9:
                    return False
                
            if loc.cold_chain_limit_minutes is not None and (current_time - depart_minutes) > loc.cold_chain_limit_minutes + 1e-9:
                return False
                
            # Add dwell time before departing for the next stop
            current_time += drone.dwell_min
            
    # Finally, check total flight range (with reserve margin)
    max_allowed_dist = drone.max_range_km * (1.0 - drone.reserve_fraction)
    if elapsed_km > max_allowed_dist + 1e-9:
        return False
        
    return True



# ---------------------------------------------------------------------------
# Stage 1: Construction Heuristics
# ---------------------------------------------------------------------------

def nearest_neighbor_construction(
    locations: List[Location],
    matrix: List[List[float]],
    drone: DroneConfig,
    depot_index: int = 0,
    depart_minutes: float = 0.0,
) -> List[List[int]]:
    """Build an initial set of routes with a greedy nearest-neighbor rule,
    respecting both the drone's payload CAPACITY and flight RANGE.

    The drone starts every route back at the depot with a full battery
    budget (max_range_km) and an empty cargo hold's worth of capacity
    (capacity_kg). We keep adding "go to the nearest customer we can still
    legally reach" until no more customers fit on this trip, then send the
    drone home and start a new trip. Repeat until every customer has been
    delivered to.

    Returns a list of routes; each route is a list of location indices
    starting and ending at `depot_index`, e.g. [[0,2,5,0], [0,1,4,0], ...].
    """
    n = len(locations)
    unvisited = set(range(n)) - {depot_index}
    routes: List[List[int]] = []

    while unvisited:
        # --- start a brand-new trip from the depot ---------------------
        current = depot_index
        route = [depot_index]
        remaining_capacity = drone.capacity_kg
        remaining_range = drone.max_range_km

        while True:
            # Find the nearest customer we can legally add to THIS trip.
            best_candidate = None
            best_distance = float("inf")

            for candidate in unvisited:
                leg_distance = matrix[current][candidate]
                test_route = route + [candidate, depot_index]
                if not is_route_feasible(test_route, locations, matrix, drone, depart_minutes=depart_minutes):
                    continue

                # Among everything still feasible, keep the closest one --
                # this greedy "always pick what's nearest right now" rule is
                # the "nearest neighbor" in this function's name.
                if leg_distance < best_distance:
                    best_distance = leg_distance
                    best_candidate = candidate

            if best_candidate is None:
                # Nothing left is feasible on this trip (out of capacity,
                # out of range, or out of customers) -- head home and, if
                # customers remain, a new trip will start for them above.
                break

            # Commit to visiting best_candidate next.
            route.append(best_candidate)
            remaining_capacity -= locations[best_candidate].demand_kg
            remaining_range -= best_distance
            current = best_candidate
            unvisited.remove(best_candidate)

        route.append(depot_index)  # fly home to close the loop
        routes.append(route)

        # Safety valve: if a single customer's demand exceeds the drone's
        # total capacity, or it's simply unreachable within max_range_km
        # even as the ONLY stop on a trip, no route will ever pick it up
        # and this would loop forever. Fail loudly instead of hanging.
        if unvisited and len(route) == 2:
            stranded = next(iter(unvisited))
            raise ValueError(
                f"Location '{locations[stranded].name}' can never be delivered to: "
                f"its demand ({locations[stranded].demand_kg}kg) exceeds drone "
                f"capacity, or a round trip to it exceeds max_range_km even alone."
            )

    return routes


def urgency_nearest_neighbor_construction(
    locations: List[Location],
    matrix: List[List[float]],
    drone: DroneConfig,
    depot_index: int = 0,
    depart_minutes: float = 0.0,
) -> List[List[int]]:
    """Nearest-neighbor construction biased toward urgent deliveries.

    Identical to nearest_neighbor_construction except for how we choose
    the "best" next stop: instead of pure distance, we use an urgency-
    weighted score:

        effective_distance = actual_distance * urgency_factor

    where urgency_factor is:
        critical = 0.70  (30% distance discount → prioritized)
        urgent   = 0.85  (15% discount)
        routine  = 1.00  (no discount)

    This means a critical stop 10km away competes equally with a routine
    stop 7km away, naturally prioritizing time-sensitive deliveries without
    completely ignoring distance efficiency. The result is a solution that
    usually costs a few percent more total distance but delivers critical
    supplies much sooner -- a real trade-off in medical logistics.
    """
    _URGENCY_DISCOUNT = {"critical": 0.70, "urgent": 0.85, "routine": 1.0}

    n = len(locations)
    unvisited = set(range(n)) - {depot_index}
    routes: List[List[int]] = []

    while unvisited:
        current = depot_index
        route = [depot_index]
        remaining_capacity = drone.capacity_kg
        remaining_range = drone.max_range_km

        while True:
            best_candidate = None
            best_score = float("inf")
            best_distance = float("inf")

            for candidate in unvisited:
                leg_distance = matrix[current][candidate]
                test_route = route + [candidate, depot_index]
                if not is_route_feasible(test_route, locations, matrix, drone, depart_minutes=depart_minutes):
                    continue

                # Score = distance * urgency factor. Lower is better.
                # Critical stops appear "closer" than they really are.
                discount = _URGENCY_DISCOUNT.get(locations[candidate].urgency, 1.0)
                score = leg_distance * discount

                if score < best_score:
                    best_score = score
                    best_distance = leg_distance  # track REAL distance for range accounting
                    best_candidate = candidate

            if best_candidate is None:
                break

            route.append(best_candidate)
            remaining_capacity -= locations[best_candidate].demand_kg
            # Use the REAL distance, not the discounted score, for range
            # accounting -- the drone doesn't actually fly less distance
            # just because the package is urgent.
            remaining_range -= best_distance
            current = best_candidate
            unvisited.remove(best_candidate)

        route.append(depot_index)
        routes.append(route)

        if unvisited and len(route) == 2:
            stranded = next(iter(unvisited))
            raise ValueError(
                f"Location '{locations[stranded].name}' can never be delivered to: "
                f"its demand ({locations[stranded].demand_kg}kg) exceeds drone "
                f"capacity, or a round trip to it exceeds max_range_km even alone."
            )

    return routes


def clarke_wright_construction(
    locations: List[Location],
    matrix: List[List[float]],
    drone: DroneConfig,
    depot_index: int = 0,
    depart_minutes: float = 0.0,
) -> List[List[int]]:
    """Clarke-Wright parallel savings algorithm.

    The idea, published by Clarke & Wright in 1964, is the opposite of
    nearest-neighbor's approach: instead of building one route at a time
    by adding stops greedily, we start with the WORST possible solution
    (every customer gets their own private trip) and ask "which pairs of
    trips would save the most distance if we merged them?"

    Concretely:
      1. Start: one route per customer: [depot, customer_i, depot].
      2. For every pair (i, j) of customers, compute:
             savings(i, j) = d(depot, i) + d(depot, j) - d(i, j)
         This is how much distance we'd save by visiting i and j on the
         same trip (consecutively) instead of in two separate depot runs.
      3. Sort all savings descending. Process them greedily: if i and j
         are on different routes AND merging those routes is feasible
         (capacity + range) AND i and j are both at the "edge" of their
         respective routes (adjacent to the depot, so they can be linked
         without reordering interior stops), merge the two routes.

    This often produces better solutions than nearest-neighbor because it
    has a more global view: it considers the structure of ALL remaining
    routes at each step, not just what's closest to where the drone
    currently is.
    """
    n = len(locations)
    customers = [i for i in range(n) if i != depot_index]

    if not customers:
        return []

    # Step 1: every customer gets their own trivial route.
    routes: List[List[int]] = []
    for c in customers:
        base_route = [depot_index, c, depot_index]
        if is_route_feasible(base_route, locations, matrix, drone, depart_minutes=depart_minutes):
            routes.append(base_route)
        else:
            raise ValueError(
                f"Location '{locations[c].name}' can never be delivered to: "
                f"fails capacity, range, time windows, or unreachable constraints."
            )

    # Step 2: compute savings for all customer pairs.
    # savings(i, j) = d(depot, i) + d(depot, j) - d(i, j)
    # Positive savings means merging is potentially worthwhile.
    savings: List[Tuple[float, int, int]] = []
    for ci, i in enumerate(customers):
        for j in customers[ci + 1:]:
            s = matrix[depot_index][i] + matrix[depot_index][j] - matrix[i][j]
            if s > 1e-9:
                savings.append((s, i, j))

    # Process merges in descending savings order (biggest wins first).
    savings.sort(key=lambda x: -x[0])

    def _find_route(customer: int) -> Optional[int]:
        """Return the index of the route containing this customer."""
        for idx, route in enumerate(routes):
            if customer in route[1:-1]:
                return idx
        return None

    for saving_val, i, j in savings:
        ri = _find_route(i)
        rj = _find_route(j)

        # Skip if already on the same route, or somehow not found.
        if ri is None or rj is None or ri == rj:
            continue

        route_a = routes[ri]
        route_b = routes[rj]

        # Check that i and j are at the EXTERIOR of their routes --
        # i.e., either the first or last customer (adjacent to the depot).
        # Interior stops can't be linked without breaking the route's
        # existing order, which could make the route worse.
        i_first = (route_a[1] == i)
        i_last = (route_a[-2] == i)
        j_first = (route_b[1] == j)
        j_last = (route_b[-2] == j)

        if not (i_first or i_last) or not (j_first or j_last):
            continue

        # Determine how to orient the two routes so i and j end up
        # adjacent in the merged result. There are four cases depending
        # on which end of each route the customer sits on:
        interior_a = route_a[1:-1]  # just the customers, no depots
        interior_b = route_b[1:-1]

        if i_last and j_first:
            # [..., i] + [j, ...] -> natural join
            merged_interior = interior_a + interior_b
        elif i_first and j_last:
            # [j, ...] joins [i, ...] -> swap order
            merged_interior = interior_b + interior_a
        elif i_last and j_last:
            # [..., i] + [..., j] -> reverse route_b so j comes first
            merged_interior = interior_a + interior_b[::-1]
        elif i_first and j_first:
            # [i, ...] + [j, ...] -> reverse route_a so i comes last
            merged_interior = interior_a[::-1] + interior_b
        else:
            continue  # shouldn't happen, but be safe

        merged = [depot_index] + merged_interior + [depot_index]

        if not is_route_feasible(merged, locations, matrix, drone, depart_minutes=depart_minutes):
            continue

        # Merge accepted! Replace both old routes with the merged one.
        # Remove in descending index order so the first pop doesn't
        # shift the second index.
        for idx in sorted([ri, rj], reverse=True):
            routes.pop(idx)
        routes.append(merged)

    return routes


# ---------------------------------------------------------------------------
# Stage 2: Improvement Operators
# ---------------------------------------------------------------------------

def two_opt(route: List[int], locations: List[Location], matrix: List[List[float]], drone: DroneConfig, max_iterations: int = 1000, depart_minutes: float = 0.0) -> List[int]:
    """Classic 2-opt local search, applied to a single route.

    A route like depot -> A -> B -> C -> D -> depot can sometimes be
    shortened by picking two edges and reversing the segment of the route
    between them -- e.g. if the path "crosses over itself" geometrically,
    uncrossing it (by reversing a middle chunk) is very often shorter.

    We try every possible pair of edges; if reversing the segment between
    them improves total distance, we keep the improvement and start over
    (a full pass with zero improvements found means we've reached a local
    optimum, and we stop).

    IMPORTANT INVARIANT: this only ever accepts a swap that makes the route
    STRICTLY SHORTER AND FEASIBLE. Reordering stops might make us miss a
    time window even if the total distance drops, so we must re-verify.
    """
    best_route = route[:]
    best_distance = _route_distance(best_route, matrix)
    improved = True
    iterations = 0

    # Indices 0 and len-1 are both the depot (route starts/ends there), so
    # we only ever reverse *interior* segments -- the depot itself never
    # moves within the route.
    while improved and iterations < max_iterations:
        improved = False
        iterations += 1

        for i in range(1, len(best_route) - 2):
            for j in range(i + 1, len(best_route) - 1):
                # Try reversing the segment between position i and j.
                # Before:  ... -> A -> [B .......... C] -> D -> ...
                # After:   ... -> A -> [C .......... B] -> D -> ...
                # Only the two "boundary" edges (A-B and C-D vs A-C and B-D)
                # actually change length -- everything inside the reversed
                # segment keeps the same neighbors, just in reverse order.
                new_route = best_route[:i] + best_route[i:j + 1][::-1] + best_route[j + 1:]
                new_distance = _route_distance(new_route, matrix)

                if new_distance < best_distance - 1e-9:  # tiny epsilon avoids float-noise loops
                    if is_route_feasible(new_route, locations, matrix, drone, depart_minutes=depart_minutes):
                        best_route = new_route
                        best_distance = new_distance
                        improved = True

    return best_route


def or_opt_inter_route(
    routes: List[List[int]],
    locations: List[Location],
    matrix: List[List[float]],
    drone: DroneConfig,
    depot_index: int = 0,
    max_iterations: int = 200,
    depart_minutes: float = 0.0,
) -> List[List[int]]:
    """Inter-route improvement: try relocating individual stops from one
    route to a better position in another route.

    This addresses the key limitation of intra-route 2-opt: 2-opt can
    reorder stops WITHIN a trip but can never move a stop FROM one trip
    TO another. If the construction heuristic made a poor assignment
    (put customer X on trip 1 when it would have been shorter on trip 2),
    2-opt alone can't fix that. Or-opt can.

    How it works:
      For every stop on every route, compute the cost of removing it
      (removal_saving = old edges in - new shortcut). Then for every
      other route, compute the cost of inserting it at every possible
      position (insertion_cost = new edges out - old edge). If the net
      change (removal_saving - insertion_cost) is positive AND the
      destination route stays within capacity and range, accept the move.

    We use steepest-descent: scan ALL possible moves, pick the single
    best one, apply it, and repeat until no improving move exists. This
    is slower than first-improvement but produces cleaner results and is
    easier to debug/explain.
    """
    routes = [r[:] for r in routes]  # deep copy so we don't mutate the caller's lists

    for iteration in range(max_iterations):
        best_move = None
        best_delta = 1e-9  # only accept moves that save at least this much

        for src_idx in range(len(routes)):
            src = routes[src_idx]

            for pos in range(1, len(src) - 1):
                node = src[pos]

                # Cost of removing `node` from its current route:
                # We replace edges (prev->node) + (node->next) with (prev->next).
                # Positive means the source route gets shorter (good).
                removal_saving = (
                    matrix[src[pos - 1]][node] + matrix[node][src[pos + 1]]
                    - matrix[src[pos - 1]][src[pos + 1]]
                )

                for dst_idx in range(len(routes)):
                    if dst_idx == src_idx:
                        continue
                    dst = routes[dst_idx]

                    # We will fully check feasibility later when we have the new route constructed

                    for ins_pos in range(1, len(dst)):
                        # Cost of inserting `node` at position `ins_pos`:
                        # We split edge (prev->next) into (prev->node) + (node->next).
                        # Positive means insertion makes dst route longer (bad).
                        insertion_cost = (
                            matrix[dst[ins_pos - 1]][node] + matrix[node][dst[ins_pos]]
                            - matrix[dst[ins_pos - 1]][dst[ins_pos]]
                        )

                        delta = removal_saving - insertion_cost
                        if delta <= best_delta:
                            continue

                        # Check full feasibility of destination and source after insertion.
                        new_dst = dst[:ins_pos] + [node] + dst[ins_pos:]
                        if not is_route_feasible(new_dst, locations, matrix, drone, depart_minutes=depart_minutes):
                            continue
                            
                        new_src = src[:pos] + src[pos + 1:]
                        if not is_route_feasible(new_src, locations, matrix, drone, depart_minutes=depart_minutes):
                            continue

                        best_delta = delta
                        best_move = (src_idx, pos, dst_idx, ins_pos, node)

        if best_move is None:
            break  # no improving move found -- we're at a local optimum

        src_idx, pos, dst_idx, ins_pos, node = best_move

        # Apply the move: remove from source, insert into destination.
        routes[src_idx] = routes[src_idx][:pos] + routes[src_idx][pos + 1:]
        routes[dst_idx] = routes[dst_idx][:ins_pos] + [node] + routes[dst_idx][ins_pos:]

        # If the source route is now empty (just depot->depot), drop it.
        routes = [r for r in routes if len(r) > 2]

    return routes


# ---------------------------------------------------------------------------
# Complete Solvers
# ---------------------------------------------------------------------------

def solve_vrp_from_scratch(
    locations: List[Location],
    matrix: List[List[float]],
    drone: DroneConfig,
    depot_index: int = 0,
    depart_minutes: float = 0.0,
) -> List[List[int]]:
    """Full from-scratch pipeline: try all three construction heuristics,
    apply 2-opt + or-opt improvement to each, and return the best solution
    found (lowest total distance).

    This multi-start approach is a simple but effective way to mitigate
    the sensitivity of greedy constructors to their starting choices --
    different heuristics make different mistakes, and the best result
    across all three is usually better than any single heuristic alone.
    """
    best_routes = None
    best_distance = float("inf")

    for name, constructor in _CONSTRUCTION_HEURISTICS.items():
        routes = constructor(locations, matrix, drone, depot_index, depart_minutes=depart_minutes)
        # Intra-route improvement
        routes = [two_opt(r, locations, matrix, drone, depart_minutes=depart_minutes) for r in routes]
        # Inter-route improvement
        routes = or_opt_inter_route(routes, locations, matrix, drone, depot_index, depart_minutes=depart_minutes)
        # Second pass of 2-opt after inter-route moves may have changed
        # route membership, creating new intra-route improvement opportunities.
        routes = [two_opt(r, locations, matrix, drone, depart_minutes=depart_minutes) for r in routes]

        dist = total_distance(routes, matrix)
        if dist < best_distance:
            best_distance = dist
            best_routes = routes

    return best_routes


def solve_vrp_with_heuristic(
    locations: List[Location],
    matrix: List[List[float]],
    drone: DroneConfig,
    construction: str = "nearest_neighbor",
    use_two_opt: bool = True,
    use_or_opt: bool = True,
    depot_index: int = 0,
    depart_minutes: float = 0.0,
) -> List[List[int]]:
    """Solve with a specific heuristic combination.

    This is the entry point used by experiments.py to systematically
    compare the effect of each construction heuristic and each improvement
    operator in isolation. The parameter names map directly to the table
    columns in the experiment output.

    Valid construction names:
        "nearest_neighbor", "urgency_nearest_neighbor", "clarke_wright"
    """
    constructor = _CONSTRUCTION_HEURISTICS.get(construction)
    if constructor is None:
        raise ValueError(
            f"Unknown construction heuristic: '{construction}'. "
            f"Valid options: {list(_CONSTRUCTION_HEURISTICS.keys())}"
        )

    routes = constructor(locations, matrix, drone, depot_index, depart_minutes=depart_minutes)

    if use_two_opt:
        routes = [two_opt(r, locations, matrix, drone, depart_minutes=depart_minutes) for r in routes]

    if use_or_opt:
        routes = or_opt_inter_route(routes, locations, matrix, drone, depot_index, depart_minutes=depart_minutes)
        # A second 2-opt pass after inter-route moves often finds a few
        # more intra-route improvements.
        if use_two_opt:
            routes = [two_opt(r, locations, matrix, drone, depart_minutes=depart_minutes) for r in routes]

    return routes


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------

def total_distance(routes: List[List[int]], matrix: List[List[float]]) -> float:
    """Sum of every route's distance -- the number we ultimately compare
    against the OR-Tools benchmark in main.py."""
    return sum(_route_distance(r, matrix) for r in routes)


# ---------------------------------------------------------------------------
# Registry of construction heuristics (used by solve_vrp_from_scratch and
# solve_vrp_with_heuristic). Defined at module level so experiments can
# iterate over it programmatically.
# ---------------------------------------------------------------------------

_CONSTRUCTION_HEURISTICS = {
    "nearest_neighbor": nearest_neighbor_construction,
    "urgency_nearest_neighbor": urgency_nearest_neighbor_construction,
    "clarke_wright": clarke_wright_construction,
}
